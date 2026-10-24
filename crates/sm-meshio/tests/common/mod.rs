//! Shared meshes and the round-trip comparison for the integration tests.
#![allow(dead_code)]

pub mod py;

use sm_core::{EntityId, EntityKind, Point3};
use sm_meshio::SmshMesh;
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh, hex_block, hex_block_with_edges};
use sm_mesh_api::*;
use sm_testkit::meshes::all_mesh_fixtures;

pub fn id(kind: EntityKind, n: u64) -> EntityId {
    EntityId::new(kind, n).unwrap()
}

pub fn on(kind: EntityKind, n: u64) -> Classification {
    Classification::on(id(kind, n))
}

fn mid(a: Point3, b: Point3) -> Point3 {
    Point3::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)
}

fn centroid_of(ps: &[Point3]) -> Point3 {
    let n = ps.len() as f64;
    Point3::new(ps.iter().map(|p| p.x).sum::<f64>() / n, ps.iter().map(|p| p.y).sum::<f64>() / n, ps.iter().map(|p| p.z).sum::<f64>() / n)
}

const HEX: [[f64; 3]; 8] = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];

/// A unit HEX27 (degree 2 Lagrange) with a classified edge table.
pub fn ho_hex() -> ReferenceMesh {
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = HEX
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + i as u64)).unwrap();
            v
        })
        .collect();
    let added = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
    b.set_class(MeshEntityRef::Cell(added.cell), on(EntityKind::TopoVolume, 1)).unwrap();
    for (i, [a, c]) in CellShape::Hex.local_edges().iter().enumerate() {
        b.set_class(MeshEntityRef::Edge(vs[*a as usize], vs[*c as usize]), on(EntityKind::TopoEdge, 1 + i as u64)).unwrap();
    }
    for (i, sf) in added.faces.iter().enumerate() {
        b.set_class(MeshEntityRef::Face(sf.face()), on(EntityKind::TopoFace, 1 + i as u64)).unwrap();
    }
    let p = |i: u8| Point3::new(HEX[i as usize][0], HEX[i as usize][1], HEX[i as usize][2]);
    let mut nodes = Vec::new();
    for s in &node_layout(CellShape::Hex, 2, Basis::LagrangeEquispaced).unwrap() {
        nodes.push(match *s {
            NodeSlot::Corner(i) => NodeRef::Vertex(vs[i as usize]),
            NodeSlot::Edge(a, c) => b.add_extra_node(mid(p(a), p(c))).unwrap(),
            NodeSlot::Face(q) => b.add_extra_node(centroid_of(&q.map(p))).unwrap(),
            NodeSlot::Interior => b.add_extra_node(centroid_of(&(0..8).map(p).collect::<Vec<_>>())).unwrap(),
            other => panic!("unexpected slot {other:?}"),
        });
    }
    b.set_high_order(added.cell, 2, Basis::LagrangeEquispaced, &nodes).unwrap();
    b.finish(&mut SerialAlloc::new()).unwrap().mesh
}

/// Two unit quads and a triangle in z = 0 (a dimension-2 mesh).
pub fn surface_strip() -> ReferenceMesh {
    let mut b = ReferenceBuilder::surface();
    let pts = [[0., 0.], [1., 0.], [2., 0.], [0., 1.], [1., 1.], [2., 1.], [3., 0.5]];
    let vs: Vec<VertexIx> = pts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], 0.0)).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + i as u64)).unwrap();
            v
        })
        .collect();
    let q1 = b.add_cell_from_vertices(CellShape::Quad, &[vs[0], vs[1], vs[4], vs[3]]).unwrap();
    let q2 = b.add_polygon(&[vs[1], vs[2], vs[5], vs[4]]).unwrap();
    let t = b.add_polygon(&[vs[2], vs[6], vs[5]]).unwrap();
    for (c, f) in [(q1.cell, 1), (q2.cell, 1), (t.cell, 2)] {
        b.set_class(MeshEntityRef::Cell(c), on(EntityKind::TopoFace, f)).unwrap();
    }
    let mut k = 1;
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        if b.face_degree(f) == 1 {
            b.set_class(MeshEntityRef::Face(f), on(EntityKind::TopoEdge, k)).unwrap();
            k += 1;
        }
    }
    let (f, _) = b.find_face(&[vs[2], vs[5]]).unwrap();
    b.set_class(MeshEntityRef::Face(f), on(EntityKind::TopoEdge, 99)).unwrap();
    b.add_tag("quality", MeshEntityKind::Cell, TagValues::F64(vec![0.5, 0.75, 1.0])).unwrap();
    b.finish(&mut SerialAlloc::new()).unwrap().mesh
}

/// Two unit hexes; cell 1 is marked as a child of cell 0 and cell 0 as inactive.
pub fn lineage_pair() -> ReferenceMesh {
    let mut b = ReferenceBuilder::new();
    let mut vs = Vec::new();
    for i in 0..3 {
        for j in 0..2 {
            for k in 0..2 {
                let v = b.add_vertex(Point3::new(i as f64, j as f64, k as f64)).unwrap();
                b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + vs.len() as u64)).unwrap();
                vs.push(v);
            }
        }
    }
    let at = |i: usize, j: usize, k: usize| vs[(i * 2 + j) * 2 + k];
    let mut cells = Vec::new();
    for i in 0..2 {
        let c = b
            .add_cell_from_vertices(
                CellShape::Hex,
                &[at(i, 0, 0), at(i + 1, 0, 0), at(i + 1, 1, 0), at(i, 1, 0), at(i, 0, 1), at(i + 1, 0, 1), at(i + 1, 1, 1), at(i, 1, 1)],
            )
            .unwrap();
        b.set_class(MeshEntityRef::Cell(c.cell), on(EntityKind::TopoVolume, 1)).unwrap();
        cells.push(c.cell);
    }
    for f in 0..b.face_count() {
        let fi = FaceIx::new(f);
        b.set_class(MeshEntityRef::Face(fi), if b.face_degree(fi) == 1 { on(EntityKind::TopoFace, 1 + f as u64) } else { on(EntityKind::TopoVolume, 1) }).unwrap();
    }
    b.set_cell_parent(cells[1], cells[0]).unwrap();
    b.set_cell_active(cells[0], false).unwrap();
    b.finish(&mut SerialAlloc::new()).unwrap().mesh
}

/// A 3x2x2 hex block with a classified edge table, cell / node / face sets with labels, side
/// set, tags, a cached geometry and a parametric vertex classification.
pub fn decorated_block() -> ReferenceMesh {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block_with_edges([3, 2, 2], [1.0, 0.5, 2.0], &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "fuel", SetMembers::Cells((0..6).map(CellIx).collect()), &mut alloc).unwrap();
    MeshEdit::add_set(&mut m, "clad", SetMembers::Cells((6..12).map(CellIx).collect()), &mut alloc).unwrap();
    MeshEdit::set_label(&mut m, "fuel", Some(7)).unwrap();
    MeshEdit::set_label(&mut m, "clad", Some(9)).unwrap();
    let bf: Vec<FaceIx> = (0..m.face_count()).map(FaceIx::new).filter(|f| m.is_boundary_face(*f)).collect();
    MeshEdit::add_set(&mut m, "wall", SetMembers::Faces(bf.clone()), &mut alloc).unwrap();
    MeshEdit::set_label(&mut m, "wall", Some(100)).unwrap();
    let sides: Vec<SignedFace> = bf.iter().map(|f| SignedFace::new(*f, Side::Left)).collect();
    MeshEdit::add_set(&mut m, "wall_sides", SetMembers::Sides(sides), &mut alloc).unwrap();
    MeshEdit::set_label(&mut m, "wall_sides", Some(101)).unwrap();
    MeshEdit::add_set(&mut m, "corner", SetMembers::Vertices(vec![VertexIx(0), VertexIx(5)]), &mut alloc).unwrap();
    MeshEdit::set_label(&mut m, "corner", Some(3)).unwrap();
    MeshEdit::add_set(&mut m, "pts", SetMembers::Points(vec![PointIx(1), PointIx(2)]), &mut alloc).unwrap();
    let nv = m.vertex_count();
    MeshEdit::set_tag(&mut m, "mat", MeshEntityKind::Cell, TagValues::I64((0..12).map(|c| c % 3).collect())).unwrap();
    MeshEdit::set_tag(&mut m, "temp", MeshEntityKind::Vertex, TagValues::F64((0..nv).map(|v| 300.0 + v as f64 * 0.25).collect())).unwrap();
    // A parametric classification on a vertex lying on a model face.
    let v = (0..nv).find(|&v| m.vertex_class(VertexIx::new(v)).entity.kind() == EntityKind::TopoFace).unwrap();
    let ent = m.vertex_class(VertexIx::new(v)).entity;
    MeshEdit::set_class(&mut m, MeshEntity::Vertex(VertexIx::new(v)), Classification { entity: ent, param: Param::Surface([0.25, 0.5]) }).unwrap();
    let cache = compute_geometry_cache(&m);
    MeshEdit::set_geometry_cache(&mut m, Some(cache)).unwrap();
    m
}

/// Every mesh the round-trip and export tests run on.
pub fn corpus() -> Vec<(String, ReferenceMesh)> {
    let mut v: Vec<(String, ReferenceMesh)> = all_mesh_fixtures().unwrap().into_iter().map(|f| (f.name.to_string(), f.mesh)).collect();
    v.push(("hex_block".into(), hex_block([4, 3, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap()));
    v.push(("decorated_block".into(), decorated_block()));
    v.push(("ho_hex".into(), ho_hex()));
    v.push(("surface_strip".into(), surface_strip()));
    v.push(("lineage_pair".into(), lineage_pair()));
    v
}

fn bits(a: &[f64]) -> Vec<u64> {
    a.iter().map(|x| x.to_bits()).collect()
}

/// Assert that the decoded file holds exactly the mesh `m`, bit for bit.
pub fn assert_matches(m: &dyn MeshRead, d: &SmshMesh) {
    let (nv, nf, nc) = (m.vertex_count(), m.face_count(), m.cell_count());
    assert_eq!(d.mesh_id, m.mesh_id());
    assert_eq!(d.dim, m.dim());
    assert_eq!(d.header.mesh.geometry_rule, m.geometry_rule());
    let co = m.coords();
    assert_eq!(bits(&d.x), bits(co.x));
    assert_eq!(bits(&d.y), bits(co.y));
    assert_eq!(bits(&d.z), bits(co.z));
    for (k, ids, n) in [(MeshEntityKind::Vertex, &d.vertex_ids, nv), (MeshEntityKind::Face, &d.face_ids, nf), (MeshEntityKind::Cell, &d.cell_ids, nc)] {
        assert_eq!(ids.len(), n);
        for (i, id) in ids.iter().enumerate() {
            assert_eq!(*id, m.id(k, i as u32));
        }
    }
    let cols = m.columns();
    assert_eq!(d.face_offsets, cols.face_vertex_offsets);
    assert_eq!(d.face_vertices, cols.face_vertices.iter().map(|v| v.0).collect::<Vec<_>>());
    assert_eq!(d.face_left, cols.face_left.iter().map(|v| v.0).collect::<Vec<_>>());
    assert_eq!(d.face_right, cols.face_right);
    assert_eq!(d.cell_face_offsets, cols.cell_face_offsets);
    assert_eq!(d.cell_faces, cols.cell_faces.iter().map(|s| s.to_raw()).collect::<Vec<_>>());
    for f in 0..nf {
        let fi = FaceIx::new(f);
        let kind = m.face_kind(fi).code();
        assert_eq!(d.face_kind[f], kind);
        assert_eq!(d.face_class[f], m.face_class(fi));
    }
    for v in 0..nv {
        assert_eq!(d.vertex_class[v], m.vertex_class(VertexIx::new(v)));
    }
    for c in 0..nc {
        let ci = CellIx::new(c);
        assert_eq!(d.cell_class[c], m.cell_class(ci));
        let a = d.cell_corner_offsets[c] as usize..d.cell_corner_offsets[c + 1] as usize;
        let want: Vec<u32> = m.cell_corners(ci).map_or(vec![], |s| s.iter().map(|v| v.0).collect());
        assert_eq!(&d.cell_corners[a], want.as_slice());
        assert_eq!(d.cell_parent[c], m.cell_parent(ci).map_or(NO_CELL, |p| p.0));
        assert_eq!(d.cell_active[c], m.cell_active(ci));
        let code = m.cell_shape(ci).code();
        assert_eq!(d.cell_shape[c], code);
    }
    assert!(d.class_entities.windows(2).all(|w| w[0] < w[1]));
    // Edge table.
    match (m.edges(), &d.edges) {
        (None, None) => {}
        (Some(t), Some(e)) => assert_eq!(e, t),
        _ => panic!("edge table presence differs"),
    }
    // High order.
    let want_ho: Vec<(CellIx, HighOrderCell)> = (0..nc).filter_map(|c| m.high_order_cell(CellIx::new(c)).map(|h| (CellIx::new(c), h.clone()))).collect();
    assert_eq!(d.high_order, want_ho);
    let want_fc: Vec<_> = (0..nf)
        .filter_map(|f| m.face_curve(FaceIx::new(f)).map(|c| (FaceIx::new(f), c.degree, c.basis, c.nodes.to_vec())))
        .collect();
    assert_eq!(d.face_curves, want_fc);
    // Cached geometry.
    match (m.geometry_cache(), &d.geometry_cache) {
        (None, None) => {}
        (Some(a), Some(b)) => assert_eq!(a, b),
        _ => panic!("geometry cache presence differs"),
    }
    // Sets and tags.
    assert_eq!(d.sets.len(), m.sets().len());
    for (a, b) in d.sets.iter().zip(m.sets()) {
        assert_eq!(a, b);
    }
    assert_eq!(d.tags.as_slice(), m.tags());
    // Hashes recorded in the header agree with the mesh.
    let h = d.header.mesh.hashes.as_ref().unwrap();
    assert_eq!(h.connectivity, format!("{:016x}", connectivity_hash(m)));
    assert_eq!(h.topology, format!("{:016x}", topology_hash(m)));
    assert_eq!(h.geometry, format!("{:016x}", geometry_hash(m)));
}
