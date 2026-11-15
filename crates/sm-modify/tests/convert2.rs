//! MOD-CONV2 acceptance on the pure pass (`sm_modify::pass` with `Rule::TetToHex` / `Rule::TriToQuad`):
//! THex gives 4 hexes of V/4 per tet, TQuad 3 quads of A/3 per triangle, conformal on a 3x3x3 tet
//! block, positive scaled Jacobians for a Kuhn tet, carried sets and deck-pair faces split alike.

use std::collections::BTreeSet;

use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{
    CellIx, CellShape, Classification, FaceIx, FaceKind, LevelMap, MeshBuild, MeshEdit, MeshEntityRef, MeshRead, SerialAlloc, SetMembers,
    VertexIx, cell_volume, face_area_vector, validate,
};
use sm_modify::{RefineError, Refined, Rule, pass};
use sm_quality::{Metric, Options, compute};
use sm_testkit::meshes::{hex_block_2x2x2, single_tet, tet_block, tet_block_two_volumes};

fn thex(src: &dyn MeshRead) -> Refined<sm_mesh::Mesh> {
    pass(src, None, true, Rule::TetToHex, MeshBuilder::new(), &mut SerialAlloc::new()).expect("thex")
}

fn vol(m: &dyn MeshRead) -> f64 {
    (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum()
}

/// Acceptance 1: each tet gives 4 hexes of volume exactly V/4, for any tet (an affine image of the
/// regular case): a 2x2x2 block with its interior vertices displaced, so every tet has its own shape.
#[test]
fn c1_every_tet_gives_four_hexes_of_a_quarter_volume() {
    let mut fx = tet_block([3, 3, 3], [1.0; 3]).expect("block");
    for i in 0..fx.mesh.vertex_count() {
        let p = fx.mesh.point(i);
        if [p.x, p.y, p.z].iter().all(|c| *c > 0.0 && *c < 3.0) {
            let s = (i * 7) as f64;
            let q = Point3::new(p.x + 0.12 * s.sin(), p.y + 0.12 * (1.7 * s).cos(), p.z + 0.12 * (2.3 * s).sin());
            fx.mesh.set_point(sm_mesh_api::PointIx::new(i), q).expect("set");
        }
    }
    let src = &fx.mesh;
    let r = thex(src);
    let m = &r.finished.mesh;
    assert_eq!(m.cell_count(), 4 * src.cell_count());
    let mut shapes = BTreeSet::new();
    for p in 0..src.cell_count() {
        let v = cell_volume(src, CellIx::new(p));
        assert!(v > 0.0);
        shapes.insert((v * 1e9).round() as i64);
        let kids: Vec<usize> = (0..m.cell_count()).filter(|&k| r.coarse_of[k].get() == p).collect();
        assert_eq!(kids.len(), 4);
        for k in kids {
            assert_eq!(m.cell_shape(CellIx::new(k)), CellShape::Hex);
            assert!((cell_volume(m, CellIx::new(k)) - v / 4.0).abs() <= 1e-14 * v, "hex {k}: {} vs {}", cell_volume(m, CellIx::new(k)), v / 4.0);
        }
    }
    assert!(shapes.len() > 20, "the tets differ in size: {} distinct volumes", shapes.len());
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert!((vol(m) - vol(src)).abs() < 1e-12);
    LevelMap::new(m, src, r.coarse_of.clone(), Some(r.face_map.clone())).expect("map");
    let sj = compute(m, Metric::ScaledJacobian, &Options::default()).expect("sj");
    eprintln!("c1: warped tet block: {} tets -> {} hexes, scaled Jacobian min {:.4}", src.cell_count(), m.cell_count(), sj.iter().copied().fold(f64::INFINITY, f64::min));
}

fn tri_patch() -> sm_mesh::Mesh {
    // The unit square as two triangles, classified (cell on a face, edges on edges, vertices on vertices).
    let on = |k: EntityKind, n: u64| Classification::on(EntityId::new(k, n).expect("id"));
    let mut b = MeshBuilder::surface();
    let p = [[0., 0.], [1., 0.], [1., 1.], [0., 1.]];
    let vs: Vec<VertexIx> = p.iter().map(|q| b.add_vertex(Point3::new(q[0], q[1], 0.0)).expect("v")).collect();
    for (i, v) in vs.iter().enumerate() {
        b.set_class(MeshEntityRef::Vertex(*v), on(EntityKind::TopoVertex, i as u64 + 1)).expect("c");
    }
    let mut en = 0;
    for t in [[0usize, 1, 2], [0, 2, 3]] {
        let a = b.add_cell_from_vertices(CellShape::Tri, &t.map(|i| vs[i])).expect("tri");
        b.set_class(MeshEntityRef::Cell(a.cell), on(EntityKind::TopoFace, 1)).expect("c");
        for (f, r) in a.faces.iter().zip(&a.reused) {
            if !r {
                en += 1;
                b.set_class(MeshEntityRef::Face(f.face()), on(EntityKind::TopoEdge, en)).expect("c");
            }
        }
    }
    b.finish(&mut SerialAlloc::new()).expect("finish").mesh
}

/// Acceptance 2: each triangle gives 3 quads of area A/3.
#[test]
fn c2_every_triangle_gives_three_quads_of_a_third_of_the_area() {
    let src = tri_patch();
    let r = pass(&src, None, true, Rule::TriToQuad, MeshBuilder::surface(), &mut SerialAlloc::new()).expect("tquad");
    let m = &r.finished.mesh;
    assert_eq!(m.cell_count(), 6);
    for c in 0..6 {
        assert_eq!(m.cell_shape(CellIx::new(c)), CellShape::Quad);
        assert!((cell_volume(m, CellIx::new(c)) - 0.5 / 3.0).abs() < 1e-15, "quad {c}");
    }
    assert!((vol(m) - 1.0).abs() < 1e-15);
    assert_eq!(m.vertex_count(), 4 + 5 + 2, "corners, five edge midpoints (4 sides and the diagonal), two centroids");
    assert!(validate(m).is_empty());
    LevelMap::new(m, &src, r.coarse_of.clone(), Some(r.face_map.clone())).expect("map");
    // Quality: scaled Jacobian of the quads of a right triangle.
    let sj = compute(m, Metric::ScaledJacobian, &Options::default()).expect("sj");
    let min = sj.iter().copied().fold(f64::INFINITY, f64::min);
    eprintln!("c2: tquad of a right-triangle patch: 6 quads of area 1/6; min scaled Jacobian {min:.4}");
    assert!(min > 0.0);
}

/// Acceptance 3 and 4: conformal on a 3x3x3 Kuhn tet block (162 tets to 648 hexes, `validate` clean,
/// volume 27), positive scaled Jacobian for every hex; the measured minimum is printed.
#[test]
fn c3_c4_tet_block_3x3x3_is_conformal_and_hexes_are_positive() {
    let fx = tet_block([3, 3, 3], [1.0; 3]).expect("block");
    assert_eq!(fx.mesh.cell_count(), 162);
    let r = thex(&fx.mesh);
    let m = &r.finished.mesh;
    assert_eq!(m.cell_count(), 648);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert!((vol(m) - 27.0).abs() < 1e-12);
    let boundary_quads = (0..m.face_count()).filter(|&f| m.face_kind(FaceIx::new(f)) == FaceKind::Boundary).count();
    assert_eq!(boundary_quads, 3 * fx.boundary_faces, "each boundary triangle gives 3 quads");
    let area: f64 = (0..m.face_count())
        .map(FaceIx::new)
        .filter(|&f| m.face_kind(f) == FaceKind::Boundary)
        .map(|f| face_area_vector(m, f).norm())
        .sum();
    assert!((area - fx.area).abs() < 1e-12, "boundary area {area} vs {}", fx.area);
    let sj = compute(m, Metric::ScaledJacobian, &Options::default()).expect("sj");
    let min = sj.iter().copied().fold(f64::INFINITY, f64::min);
    let max = sj.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    eprintln!("c3: 3x3x3 Kuhn tet block: 162 tets -> {} hexes; scaled Jacobian min {min:.4}, max {max:.4}", m.cell_count());
    assert!(min > 0.0, "every hex has a positive scaled Jacobian");
}

/// Acceptance 5: blocks (cell sets) and side sets carry over; per-volume cell counts times 4, volumes
/// and the interface area between the two volumes are unchanged.
#[test]
fn c5_blocks_and_sidesets_carry_over() {
    let mut alloc = SerialAlloc::new();
    let mut fx = tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).expect("two volumes");
    let n0 = fx.mesh.cell_count();
    let left: Vec<FaceIx> = (0..fx.mesh.face_count())
        .map(FaceIx::new)
        .filter(|&f| fx.mesh.face_kind(f) == FaceKind::Boundary && fx.mesh.face_vertices(f).iter().all(|v| fx.mesh.point(v.get()).x == 0.0))
        .collect();
    assert_eq!(left.len(), 8);
    fx.mesh.add_set("xmin", SetMembers::Faces(left), &mut alloc).expect("set");
    let r = thex(&fx.mesh);
    let m = &r.finished.mesh;
    assert!(validate(m).is_empty());
    assert_eq!(m.cell_count(), 4 * n0);
    let mut per: std::collections::BTreeMap<String, f64> = Default::default();
    for c in 0..m.cell_count() {
        *per.entry(m.cell_class(CellIx::new(c)).entity.to_string()).or_default() += cell_volume(m, CellIx::new(c));
    }
    let want: Vec<f64> = vec![8.0, 8.0];
    let got: Vec<f64> = per.values().copied().collect();
    assert_eq!(got.len(), 2);
    for (g, w) in got.iter().zip(&want) {
        assert!((g - w).abs() < 1e-12, "{g} vs {w}");
    }
    let SetMembers::Faces(fs) = &m.set_by_name("xmin").expect("set").members else { panic!("faces") };
    assert_eq!(fs.len(), 24, "8 triangles become 24 quads");
    let a: f64 = fs.iter().map(|&f| face_area_vector(m, f).norm()).sum();
    assert!((a - 4.0).abs() < 1e-13, "side set area {a}");
}

/// Acceptance 6: the two faces of a periodic pair (x = 0 and x = 3 of a Kuhn block, equal
/// triangulations) are split identically: the quad centroids of the B side are those of the A side translated.
#[test]
fn c6_paired_faces_are_split_identically() {
    let fx = tet_block([3, 3, 3], [1.0; 3]).expect("block");
    let r = thex(&fx.mesh);
    let m = &r.finished.mesh;
    let quads_at = |x: f64| -> BTreeSet<[i64; 3]> {
        (0..m.face_count())
            .map(FaceIx::new)
            .filter(|&f| m.face_vertices(f).iter().all(|v| (m.point(v.get()).x - x).abs() < 1e-12))
            .map(|f| {
                let l = m.face_vertices(f);
                let k = 1.0 / l.len() as f64;
                let c = l.iter().fold([0.0; 3], |s, v| [s[0] + m.point(v.get()).y, s[1] + m.point(v.get()).z, 0.0 * s[2]]);
                [(c[0] * k * 1e9).round() as i64, (c[1] * k * 1e9).round() as i64, l.len() as i64]
            })
            .collect()
    };
    let (a, b) = (quads_at(0.0), quads_at(3.0));
    assert_eq!(a.len(), 3 * 18, "each of the 18 boundary triangles of a face gives 3 quads");
    assert_eq!(a, b);
}

/// Mixed and non-tet meshes are refused for the conversion rules.
#[test]
fn only_tets_convert_to_hexes_and_only_triangles_to_quads() {
    let hex = hex_block_2x2x2().expect("hex block");
    let Err(e) = pass(&hex.mesh, None, true, Rule::TetToHex, MeshBuilder::new(), &mut SerialAlloc::new()) else { panic!("refused") };
    assert!(matches!(e, RefineError::UnsupportedCell { .. }), "{e}");
    let tet = single_tet().expect("tet");
    let Err(e) = pass(&tet.mesh, None, true, Rule::TriToQuad, MeshBuilder::surface(), &mut SerialAlloc::new()) else { panic!("refused") };
    assert!(matches!(e, RefineError::UnsupportedCell { .. }), "{e}");
}

/// Two runs and 1 vs N threads give equal hashes.
#[test]
fn c7_thex_is_deterministic() {
    let fx = tet_block([3, 2, 2], [1.0, 0.5, 2.0]).expect("block");
    let h = |r: &Refined<sm_mesh::Mesh>| (sm_mesh_api::topology_hash(&r.finished.mesh), sm_mesh_api::geometry_hash(&r.finished.mesh));
    let a = h(&thex(&fx.mesh));
    assert_eq!(a, h(&thex(&fx.mesh)));
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    assert_eq!(a, one.install(|| h(&thex(&fx.mesh))));
}
