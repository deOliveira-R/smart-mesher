//! W0f gaps 2: point sets, the edge-table rule for high-order meshes, `MeshRead::point_class`,
//! shell side sets and the dimension-2 shell orientation rule.

mod common;

use common::*;
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::*;

fn mid(a: Point3, b: Point3) -> Point3 {
    Point3::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)
}

/// A unit HEX27 (vertices on `TopoVertex#1..8`, faces on `TopoFace#1..6`, edges on
/// `TopoEdge#1..12`) with face 0 curved by a QUAD9 record whose extra nodes are fresh points.
fn hex27(classify_edges: bool) -> Result<(ReferenceMesh, Vec<NodeRef>)> {
    let mut b = ReferenceBuilder::new();
    let pts = template_points(CellShape::Hex);
    let vs: Vec<VertexIx> = pts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + i as u64)).unwrap();
            v
        })
        .collect();
    let added = b.add_cell_from_vertices(CellShape::Hex, &vs)?;
    b.set_class(MeshEntityRef::Cell(added.cell), vol())?;
    for (i, sf) in added.faces.iter().enumerate() {
        b.set_class(MeshEntityRef::Face(sf.face()), on(EntityKind::TopoFace, 1 + i as u64))?;
    }
    if classify_edges {
        classify_template_edges(&mut b, CellShape::Hex, &vs);
    }
    let p = |i: u8| Point3::new(pts[i as usize][0], pts[i as usize][1], pts[i as usize][2]);
    let mut nodes = Vec::new();
    for s in node_layout(CellShape::Hex, 2, Basis::LagrangeEquispaced).unwrap() {
        nodes.push(match s {
            NodeSlot::Corner(i) => NodeRef::Vertex(vs[i as usize]),
            NodeSlot::Edge(a, c) => b.add_extra_node(mid(p(a), p(c)))?,
            NodeSlot::Face(_) | NodeSlot::Interior => b.add_extra_node(Point3::new(0.5, 0.5, 0.5))?,
            other => panic!("{other:?}"),
        });
    }
    b.set_high_order(added.cell, 2, Basis::LagrangeEquispaced, &nodes)?;
    // Curve face 0 with its own edge nodes and centre.
    let f0 = added.faces[0].face();
    let lp = b.face_loop(f0).to_vec();
    let mut fnodes: Vec<NodeRef> = lp.iter().map(|&v| NodeRef::Vertex(v)).collect();
    for i in 0..4 {
        let pm = mid(b.vertex_point(lp[i]), b.vertex_point(lp[(i + 1) % 4]));
        fnodes.push(b.add_extra_node(Point3::new(pm.x - 0.05, pm.y, pm.z))?);
    }
    fnodes.push(b.add_extra_node(Point3::new(-0.1, 0.5, 0.5))?);
    b.set_face_curve(f0, 2, Basis::LagrangeEquispaced, &fnodes)?;
    Ok((b.finish(&mut SerialAlloc::new())?.mesh, nodes))
}

#[test]
fn high_order_meshes_carry_a_classified_edge_table() {
    // Brick edges lie on two model faces: without explicit classes finish cannot derive them.
    assert!(matches!(hex27(false), Err(MeshError::UnclassifiedEdge(..))));
    let (mut m, _) = hex27(true).unwrap();
    assert_valid(&m);
    assert_eq!(m.edges().unwrap().len(), 12, "derived at finish although no edge tag was asked for");
    // The table cannot be dropped while edge nodes exist; a linear copy can drop it.
    assert!(matches!(MeshEdit::drop_edges(&mut m), Err(MeshError::State(_))));
    // MeshEdit refuses new high-order records on a dimension-3 mesh without a table.
    let mut lin = single_cell(CellShape::Hex, &mut SerialAlloc::new());
    let rec = HighOrderCell { degree: 2, basis: Basis::Serendipity, nodes: vec![PointIx(0); 20] };
    assert!(matches!(MeshEdit::set_high_order(&mut lin, CellIx(0), Some(rec)), Err(MeshError::State(_))));
    MeshEdit::drop_edges(&mut lin).unwrap();
}

#[test]
fn point_class_follows_the_slot_owner() {
    let (m, _) = hex27(true).unwrap();
    let nv = m.vertex_count();
    let h = m.high_order_cell(CellIx(0)).unwrap().clone();
    let corners = m.cell_corners(CellIx(0)).unwrap().to_vec();
    let layout = node_layout(CellShape::Hex, 2, Basis::LagrangeEquispaced).unwrap();
    for (k, slot) in layout.iter().enumerate() {
        let got = m.point_class(h.nodes[k]).unwrap();
        let want = match *slot {
            NodeSlot::Corner(i) => m.vertex_class(corners[i as usize]),
            NodeSlot::Edge(a, b) => {
                let t = m.edges().unwrap();
                t.class[t.find(corners[a as usize], corners[b as usize]).unwrap().get()]
            }
            NodeSlot::Face(q) => {
                let f = m.cell_faces(CellIx(0)).iter().map(|sf| sf.face()).find(|f| q.iter().all(|i| m.face_vertices(*f).contains(&corners[*i as usize]))).unwrap();
                m.face_class(f)
            }
            NodeSlot::Interior => m.cell_class(CellIx(0)),
            _ => unreachable!(),
        };
        assert_eq!(got, want, "slot {slot:?}");
        if k >= 8 {
            assert!(h.nodes[k].get() >= nv);
        }
    }
    assert_eq!(m.point_class(h.nodes[8]).unwrap().entity.kind(), EntityKind::TopoEdge);
    assert_eq!(m.point_class(h.nodes[26]).unwrap(), vol());
    // Face-curve nodes: edge nodes on their edges, the centre on the face.
    let f = (0..m.face_count()).map(FaceIx::new).find(|f| m.face_curve(*f).is_some()).unwrap();
    let fc = m.face_curve(f).unwrap();
    let l = m.face_vertices(f);
    for i in 0..4 {
        let t = m.edges().unwrap();
        let e = t.find(l[i], l[(i + 1) % 4]).unwrap();
        assert_eq!(m.point_class(fc.nodes[4 + i]).unwrap(), t.class[e.get()]);
    }
    assert_eq!(m.point_class(fc.nodes[8]).unwrap(), m.face_class(f));
    assert_eq!(m.point_class(PointIx(0)).unwrap(), m.vertex_class(VertexIx(0)));
    assert!(m.point_class(PointIx(m.coords().len() as u32)).is_none());
}

#[test]
fn point_sets_address_the_coords_space() {
    let (mut m, _) = hex27(true).unwrap();
    let np = m.coords().len() as u32;
    let mut alloc = SerialAlloc::new();
    let s = SetMembers::Points(vec![PointIx(0), PointIx(9), PointIx(np - 1)]);
    assert_eq!((s.kind(), s.code(), s.len()), (MeshEntityKind::Vertex, 5, 3));
    assert!(s.contains(np - 1) && !s.contains(1));
    let h0 = topology_hash(&m);
    MeshEdit::add_set(&mut m, "nodes", s, &mut alloc).unwrap();
    assert_valid(&m);
    assert_ne!(topology_hash(&m), h0, "point sets are hashed");
    assert!(MeshEdit::add_set(&mut m, "oob", SetMembers::Points(vec![PointIx(np)]), &mut alloc).is_err());
    assert!(MeshEdit::add_set(&mut m, "unsorted", SetMembers::Points(vec![PointIx(3), PointIx(2)]), &mut alloc).is_err());
    // A point set and a vertex set with the same raw members hash differently (code).
    let (mut a, mut b) = (m.clone(), m.clone());
    MeshEdit::add_set(&mut a, "x", SetMembers::Points(vec![PointIx(1)]), &mut SerialAlloc::new()).unwrap();
    MeshEdit::add_set(&mut b, "x", SetMembers::Vertices(vec![VertexIx(1)]), &mut SerialAlloc::new()).unwrap();
    assert_ne!(topology_hash(&a), topology_hash(&b));
    // Builder path: extra nodes are numbered after the vertices.
    let mut bld = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = template_points(CellShape::Tet).iter().map(|p| bld.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap()).collect();
    let c = bld.add_cell_from_vertices(CellShape::Tet, &vs).unwrap();
    bld.set_class(MeshEntityRef::Cell(c.cell), vol()).unwrap();
    for sf in &c.faces {
        bld.set_class(MeshEntityRef::Face(sf.face()), on(EntityKind::TopoFace, 1)).unwrap();
    }
    for &v in &vs {
        bld.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoFace, 1)).unwrap();
    }
    bld.add_extra_node(Point3::new(0.1, 0.1, 0.1)).unwrap();
    bld.add_set("p", SetMembers::Points(vec![PointIx(4), PointIx(0)])).unwrap();
    let fm = bld.finish(&mut SerialAlloc::new()).unwrap().mesh;
    assert_eq!(fm.set_by_name("p").unwrap().members, SetMembers::Points(vec![PointIx(0), PointIx(4)]));
}

/// Two quads in z = 0 on `TopoFace#1`, counter-clockwise seen from +z.
fn quads() -> ReferenceMesh {
    let mut b = ReferenceBuilder::surface();
    let pts = [[0., 0.], [1., 0.], [2., 0.], [0., 1.], [1., 1.], [2., 1.]];
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
    let q2 = b.add_cell_from_vertices(CellShape::Quad, &[vs[1], vs[2], vs[5], vs[4]]).unwrap();
    for c in [q1.cell, q2.cell] {
        b.set_class(MeshEntityRef::Cell(c), on(EntityKind::TopoFace, 1)).unwrap();
    }
    let mut k = 1;
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        let cls = if b.face_degree(f) == 1 { on(EntityKind::TopoEdge, k) } else { on(EntityKind::TopoFace, 1) };
        b.set_class(MeshEntityRef::Face(f), cls).unwrap();
        k += 1;
    }
    b.add_set("top0", SetMembers::ShellSides(vec![0, 3])).unwrap();
    b.finish(&mut SerialAlloc::new()).unwrap().mesh
}

struct Normal(Vector3);

impl FaceNormals for Normal {
    fn normal(&self, entity: EntityId, _near: Point3) -> Option<Vector3> {
        (entity == id(EntityKind::TopoFace, 1)).then_some(self.0)
    }
}

#[test]
fn shell_cells_follow_the_classified_face_normal() {
    let m = quads();
    assert_valid(&m);
    let tol = GeomTol::default();
    assert!(validate_geometry_with(&m, &tol, &Normal(Vector3::new(0.0, 0.0, 1.0))).is_empty());
    let bad = validate_geometry_with(&m, &tol, &Normal(Vector3::new(0.0, 0.0, -1.0)));
    assert_eq!(bad.len(), 2);
    assert!(bad.iter().all(|v| v.code == ViolationCode::GeomShellOrientation && v.code.as_str() == "geom.shell_orientation"));
    // Without a normal source (or a carrier) the rule is not checked.
    struct None_;
    impl FaceNormals for None_ {
        fn normal(&self, _: EntityId, _: Point3) -> Option<Vector3> {
            None
        }
    }
    assert!(validate_geometry_with(&m, &tol, &None_).is_empty());
}

#[test]
fn shell_side_sets_pack_cell_and_side() {
    let mut m = quads();
    let s = m.set_by_name("top0").unwrap().members.clone();
    assert_eq!((s.kind(), s.code()), (MeshEntityKind::Cell, 4));
    assert_eq!(s, SetMembers::ShellSides(vec![0, 3]), "cell 0 top, cell 1 bottom");
    assert!(s.contains(0) && s.contains(1) && !s.contains(2));
    assert_eq!(s.max_index(), Some(1));
    let mut alloc = SerialAlloc::new();
    assert!(MeshEdit::add_set(&mut m, "oob", SetMembers::ShellSides(vec![4]), &mut alloc).is_err());
    assert!(MeshEdit::add_set(&mut m, "dup", SetMembers::ShellSides(vec![1, 1]), &mut alloc).is_err());
    // Shell sides belong to dimension-2 meshes only.
    let mut v = single_cell(CellShape::Hex, &mut SerialAlloc::new());
    assert!(MeshEdit::add_set(&mut v, "s", SetMembers::ShellSides(vec![0]), &mut alloc).is_err());
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = template_points(CellShape::Tet).iter().map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap()).collect();
    let c = b.add_cell_from_vertices(CellShape::Tet, &vs).unwrap();
    b.set_class(MeshEntityRef::Cell(c.cell), vol()).unwrap();
    b.add_set("s", SetMembers::ShellSides(vec![0])).unwrap();
    assert!(b.finish(&mut SerialAlloc::new()).is_err());
}

#[test]
fn point_class_in_a_surface_mesh_reads_the_facet() {
    let mut b = ReferenceBuilder::surface();
    let pts = [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]];
    let vs: Vec<VertexIx> = pts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], 0.0)).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + i as u64)).unwrap();
            v
        })
        .collect();
    let t = b.add_cell_from_vertices(CellShape::Tri, &vs).unwrap();
    b.set_class(MeshEntityRef::Cell(t.cell), on(EntityKind::TopoFace, 1)).unwrap();
    for (k, sf) in t.faces.iter().enumerate() {
        b.set_class(MeshEntityRef::Face(sf.face()), on(EntityKind::TopoEdge, 10 + k as u64)).unwrap();
    }
    let mut nodes: Vec<NodeRef> = vs.iter().map(|&v| NodeRef::Vertex(v)).collect();
    for [a, c] in CellShape::Tri.local_edges() {
        let (pa, pc) = (pts[*a as usize], pts[*c as usize]);
        nodes.push(b.add_extra_node(Point3::new((pa[0] + pc[0]) / 2.0, (pa[1] + pc[1]) / 2.0, 0.0)).unwrap());
    }
    b.set_high_order(t.cell, 2, Basis::LagrangeEquispaced, &nodes).unwrap();
    let m = b.finish(&mut SerialAlloc::new()).unwrap().mesh;
    assert!(m.edges().is_none(), "a surface mesh has no edge table");
    let h = m.high_order_cell(CellIx(0)).unwrap();
    let corners = m.cell_corners(CellIx(0)).unwrap();
    for (k, [a, c]) in CellShape::Tri.local_edges().iter().enumerate() {
        let f = (0..m.face_count()).map(FaceIx::new).find(|f| {
            let l = m.face_vertices(*f);
            l.contains(&corners[*a as usize]) && l.contains(&corners[*c as usize])
        });
        assert_eq!(m.point_class(h.nodes[3 + k]), Some(m.face_class(f.unwrap())));
    }
}
