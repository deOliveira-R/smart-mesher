//! MOD-REFINE-U on dimension-2 meshes: quad 1 to 4 and tri 1 to 4 over a square, with analytic
//! areas (each child is a quarter of its parent) and the facet bookkeeping of a surface mesh.

use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{
    CellIx, Classification, FaceIx, FaceKind, LevelMap, MeshBuild, MeshDim, MeshEdit, MeshEntityRef, MeshRead, PointIx,
    SerialAlloc, VertexIx, validate,
};
use sm_modify::refine;

fn ent(kind: EntityKind, n: u64) -> Classification {
    Classification::on(EntityId::new(kind, n).expect("id"))
}

/// An `n x n` grid of unit squares as quads, or as `2 n^2` triangles with a diagonal each.
fn grid(n: usize, tri: bool) -> sm_mesh::Mesh {
    let mut b = MeshBuilder::surface();
    let m = n + 1;
    let ix = |i: usize, j: usize| VertexIx::new(j * m + i);
    for j in 0..m {
        for i in 0..m {
            let v = b.add_vertex(Point3::new(i as f64, j as f64, 0.0)).expect("vertex");
            let (bx, by) = (i == 0 || i == n, j == 0 || j == n);
            let c = match (bx, by) {
                (true, true) => ent(EntityKind::TopoVertex, 1 + (usize::from(i == n) + 2 * usize::from(j == n)) as u64),
                (true, false) => ent(EntityKind::TopoEdge, if i == 0 { 4 } else { 2 }),
                (false, true) => ent(EntityKind::TopoEdge, if j == 0 { 1 } else { 3 }),
                _ => ent(EntityKind::TopoFace, 1),
            };
            b.set_class(MeshEntityRef::Vertex(v), c).expect("class");
        }
    }
    let face = ent(EntityKind::TopoFace, 1);
    for j in 0..n {
        for i in 0..n {
            let quad = [ix(i, j), ix(i + 1, j), ix(i + 1, j + 1), ix(i, j + 1)];
            if tri {
                for t in [[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]] {
                    let a = b.add_polygon(&t).expect("tri");
                    b.set_class(MeshEntityRef::Cell(a.cell), face).expect("class");
                }
            } else {
                let a = b.add_polygon(&quad).expect("quad");
                b.set_class(MeshEntityRef::Cell(a.cell), face).expect("class");
            }
        }
    }
    for f in 0..b.face_count() {
        let fh = FaceIx::new(f);
        if b.face_degree(fh) == 1 {
            let l = b.face_loop(fh).to_vec();
            let p = b.vertex_point(l[0]);
            let q = b.vertex_point(l[1]);
            let side = if p.y == 0.0 && q.y == 0.0 {
                1
            } else if p.x == n as f64 && q.x == n as f64 {
                2
            } else if p.y == n as f64 && q.y == n as f64 {
                3
            } else {
                4
            };
            b.set_class(MeshEntityRef::Face(fh), ent(EntityKind::TopoEdge, side)).expect("facet class");
        }
    }
    b.finish(&mut SerialAlloc::new()).expect("finish").mesh
}

fn area(m: &dyn MeshRead, c: usize) -> f64 {
    let n = m.cell_normal(CellIx::new(c)).expect("2D normal");
    0.5 * (n.x * n.x + n.y * n.y + n.z * n.z).sqrt()
}

fn check(src: &sm_mesh::Mesh, per_parent: usize, side: usize) {
    assert_eq!(src.dim(), MeshDim::Two);
    assert!(validate(src).is_empty(), "{:?}", validate(src));
    let r = refine(src, None, true, MeshBuilder::surface(), &mut SerialAlloc::new()).expect("refine");
    let m = &r.finished.mesh;
    assert_eq!(m.dim(), MeshDim::Two);
    assert_eq!(m.cell_count(), per_parent * src.cell_count());
    assert_eq!(m.vertex_count(), (2 * side + 1) * (2 * side + 1));
    let v = validate(m);
    assert!(v.is_empty(), "validate: {v:?}");
    // Every child is a quarter of its parent, exactly; the total is the side squared.
    for (k, p) in r.coarse_of.iter().enumerate() {
        assert!((area(m, k) - area(src, p.get()) / 4.0).abs() <= 1e-15, "child {k}");
    }
    let total: f64 = (0..m.cell_count()).map(|c| area(m, c)).sum();
    assert!((total - (side * side) as f64).abs() < 1e-12);
    // Boundary facets double: 4 sides of `side` become `2 side` each.
    let bf = (0..m.face_count()).filter(|&f| m.face_kind(FaceIx::new(f)) == FaceKind::Boundary).count();
    assert_eq!(bf, 8 * side);
    // The facet map carries each boundary facet to its parent.
    LevelMap::new(m, src, r.coarse_of.clone(), Some(r.face_map.clone())).expect("level map with face map");
    assert_eq!(r.face_map.iter().filter(|&&x| x != sm_mesh_api::NO_CELL).count(), 2 * src.face_count());
    // Boundary nodes stay on their model edges.
    for v in 0..m.vertex_count() {
        let c = m.classification(sm_mesh_api::MeshEntityKind::Vertex, v as u32);
        let p = m.point(v);
        let on_side = p.x == 0.0 || p.y == 0.0 || p.x == side as f64 || p.y == side as f64;
        assert_eq!(on_side, c.entity.kind() != EntityKind::TopoFace, "node {v} at ({}, {}) is {:?}", p.x, p.y, c.entity);
    }
}

#[test]
fn quads_split_into_four_quarters() {
    check(&grid(3, false), 4, 3);
}

#[test]
fn triangles_split_into_four_quarters() {
    check(&grid(3, true), 4, 3);
}

#[test]
fn a_sheared_quad_mesh_keeps_the_quarters() {
    // A parallelogram of area 6: x -> x + 0.5 y.
    let mut m = grid(2, false);
    for i in 0..m.vertex_count() {
        let p = m.point(i);
        m.set_point(PointIx::new(i), Point3::new(p.x + 0.5 * p.y, 1.5 * p.y, 0.0)).expect("set");
    }
    let r = refine(&m, None, true, MeshBuilder::surface(), &mut SerialAlloc::new()).expect("refine");
    let total: f64 = (0..r.finished.mesh.cell_count()).map(|c| area(&r.finished.mesh, c)).sum();
    assert!((total - 4.0 * 1.5).abs() < 1e-12, "area {total}");
    for (k, p) in r.coarse_of.iter().enumerate() {
        assert!((area(&r.finished.mesh, k) - area(&m, p.get()) / 4.0).abs() <= 1e-15);
    }
}
