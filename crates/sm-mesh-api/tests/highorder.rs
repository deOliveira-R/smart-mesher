//! High order: node counts and order, HEX27/HEX20, face curvature side table.

mod common;
use common::*;
use sm_core::{EntityKind, Point3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::*;

#[test]
fn node_counts_follow_the_shape_degree_basis_table() {
    use Basis::*;
    use CellShape::*;
    let table = [
        (Tet, 2, LagrangeEquispaced, Some(10)),
        (Tet, 2, Serendipity, Some(10)),
        (Hex, 2, LagrangeEquispaced, Some(27)),
        (Hex, 2, LagrangeGll, Some(27)),
        (Hex, 2, Serendipity, Some(20)),
        (Wedge, 2, LagrangeEquispaced, Some(18)),
        (Wedge, 2, Serendipity, Some(15)),
        (Pyramid, 2, LagrangeEquispaced, Some(14)),
        (Pyramid, 2, Serendipity, Some(13)),
        (Hex, 1, LagrangeEquispaced, Some(8)),
        (Tet, 1, LagrangeGll, Some(4)),
        (Hex, 0, LagrangeEquispaced, None),
        (Hex, 3, LagrangeEquispaced, None),
        (Polyhedron, 2, LagrangeEquispaced, None),
    ];
    for (s, d, b, want) in table {
        assert_eq!(node_count(s, d, b), want, "{s:?} d{d} {b:?}");
        if let Some(n) = want {
            let l = node_layout(s, d, b).unwrap();
            assert_eq!(l.len(), n);
            assert!(
                l.iter()
                    .take(s.vertex_count().unwrap())
                    .enumerate()
                    .all(|(i, x)| *x == NodeSlot::Corner(i as u8))
            );
        }
    }
    let l = node_layout(Hex, 2, LagrangeEquispaced).unwrap();
    let count = |f: fn(&NodeSlot) -> bool| l.iter().filter(|s| f(s)).count();
    assert_eq!(count(|s| matches!(s, NodeSlot::Edge(..))), 12);
    assert_eq!(count(|s| matches!(s, NodeSlot::Face(_))), 6);
    assert_eq!(count(|s| matches!(s, NodeSlot::Interior)), 1);
    assert_eq!(face_node_count(3, 2, Basis::LagrangeEquispaced), Some(6));
    assert_eq!(face_node_count(4, 2, Basis::LagrangeEquispaced), Some(9));
    assert_eq!(face_node_count(4, 2, Basis::Serendipity), Some(8));
    assert_eq!(face_node_count(5, 2, Basis::Serendipity), Some(10));
    assert_eq!(face_node_count(5, 2, Basis::LagrangeEquispaced), None);
    assert_eq!(face_node_count(4, 3, Basis::Serendipity), None);
}

fn mid(a: Point3, b: Point3) -> Point3 {
    Point3::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0)
}

fn centroid_of(ps: &[Point3]) -> Point3 {
    let n = ps.len() as f64;
    Point3::new(
        ps.iter().map(|p| p.x).sum::<f64>() / n,
        ps.iter().map(|p| p.y).sum::<f64>() / n,
        ps.iter().map(|p| p.z).sum::<f64>() / n,
    )
}

/// A unit hex with a high-order record of the given basis; extra nodes at their straight positions.
fn ho_hex(basis: Basis, degree: u8, drop: usize) -> Result<ReferenceMesh> {
    let mut b = ReferenceBuilder::new();
    let pts = template_points(CellShape::Hex);
    let vs: Vec<VertexIx> = pts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
            b.set_class(
                MeshEntityRef::Vertex(v),
                on(EntityKind::TopoVertex, 1 + i as u64),
            )
            .unwrap();
            v
        })
        .collect();
    let added = b.add_cell_from_vertices(CellShape::Hex, &vs)?;
    b.set_class(MeshEntityRef::Cell(added.cell), vol())?;
    classify_template_edges(&mut b, CellShape::Hex, &vs);
    for (i, sf) in added.faces.iter().enumerate() {
        b.set_class(
            MeshEntityRef::Face(sf.face()),
            on(EntityKind::TopoFace, 1 + i as u64),
        )?;
    }
    let p = |i: u8| Point3::new(pts[i as usize][0], pts[i as usize][1], pts[i as usize][2]);
    let layout = node_layout(CellShape::Hex, degree.max(2), basis).unwrap();
    let mut nodes = Vec::new();
    for s in &layout {
        nodes.push(match *s {
            NodeSlot::Corner(i) => NodeRef::Vertex(vs[i as usize]),
            NodeSlot::Edge(a, c) => b.add_extra_node(mid(p(a), p(c)))?,
            NodeSlot::Face(q) => b.add_extra_node(centroid_of(&q.map(p)))?,
            NodeSlot::Interior => {
                b.add_extra_node(centroid_of(&(0..8).map(p).collect::<Vec<_>>()))?
            }
            other => panic!("slot {other:?} not used by HEX27/HEX20"),
        });
    }
    nodes.truncate(nodes.len() - drop);
    b.set_high_order(added.cell, degree, basis, &nodes)?;
    Ok(b.finish(&mut SerialAlloc::new())?.mesh)
}

#[test]
fn hex27_and_hex20_build_and_validate() {
    let m = ho_hex(Basis::LagrangeEquispaced, 2, 0).unwrap();
    assert_valid(&m);
    let h = m.high_order_cell(CellIx(0)).unwrap();
    assert_eq!((h.degree, h.nodes.len()), (2, 27));
    assert_eq!(m.coords().len(), 8 + 19);
    assert_eq!(
        m.cell_geometry(CellIx(0)),
        CellGeometry {
            shape: CellShape::Hex,
            degree: 2,
            basis: Basis::LagrangeEquispaced
        }
    );
    // Analytic node positions: every edge node is the midpoint of its edge, face nodes the face centre.
    let layout = node_layout(CellShape::Hex, 2, Basis::LagrangeEquispaced).unwrap();
    let corners = m.cell_corners(CellIx(0)).unwrap();
    for (n, s) in h.nodes.iter().zip(&layout) {
        let p = m.point(n.get());
        match *s {
            NodeSlot::Edge(a, c) => {
                let (pa, pc) = (
                    m.point(corners[a as usize].get()),
                    m.point(corners[c as usize].get()),
                );
                assert!(
                    (p.x - (pa.x + pc.x) / 2.0).abs() < 1e-15
                        && (p.y - (pa.y + pc.y) / 2.0).abs() < 1e-15
                );
            }
            NodeSlot::Interior => {
                assert!((p.x - 0.5).abs() + (p.y - 0.5).abs() + (p.z - 0.5).abs() < 1e-15)
            }
            _ => {}
        }
    }
    let s = ho_hex(Basis::Serendipity, 2, 0).unwrap();
    assert_valid(&s);
    assert_eq!(s.high_order_cell(CellIx(0)).unwrap().nodes.len(), 20);
    let g = ho_hex(Basis::LagrangeGll, 2, 0).unwrap();
    assert_eq!(g.cell_geometry(CellIx(0)).basis, Basis::LagrangeGll);
}

#[test]
fn high_order_counts_degrees_and_shapes_are_checked() {
    // One node too few.
    assert!(matches!(
        ho_hex(Basis::LagrangeEquispaced, 2, 1),
        Err(MeshError::OutOfRange(_))
    ));
    // Serendipity needs 20, not 27 (extra 7 dropped from a 20-node layout is fine; 27 nodes is not).
    // Unsupported degrees.
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = template_points(CellShape::Tet)
        .iter()
        .map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap())
        .collect();
    let added = b.add_cell_from_vertices(CellShape::Tet, &vs).unwrap();
    let nodes: Vec<NodeRef> = vs.iter().map(|&v| NodeRef::Vertex(v)).collect();
    for d in [0u8, 1, 3] {
        assert!(
            matches!(
                b.set_high_order(added.cell, d, Basis::LagrangeEquispaced, &nodes),
                Err(MeshError::Unsupported(_))
            ),
            "degree {d}"
        );
    }
    // A cell built from loops has no supplied corners: high order is refused.
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = template_points(CellShape::Tet)
        .iter()
        .map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap())
        .collect();
    let loops: Vec<Vec<VertexIx>> = CellShape::Tet
        .local_faces()
        .iter()
        .map(|l| l.iter().map(|&i| vs[i as usize]).collect())
        .collect();
    let refs: Vec<&[VertexIx]> = loops.iter().map(Vec::as_slice).collect();
    let added = b.add_polyhedron(&refs).unwrap();
    let nodes: Vec<NodeRef> = (0..10)
        .map(|i| {
            if i < 4 {
                NodeRef::Vertex(vs[i])
            } else {
                b.add_extra_node(Point3::default()).unwrap()
            }
        })
        .collect();
    assert!(matches!(
        b.set_high_order(added.cell, 2, Basis::LagrangeEquispaced, &nodes),
        Err(MeshError::Unsupported(_))
    ));
    // Polyhedra (here a pentagonal prism) are rejected by the builder and by MeshEdit.
    let (m, cell) = prism();
    let mut m = m;
    let r = MeshEdit::set_high_order(
        &mut m,
        cell,
        Some(HighOrderCell::new(2, Basis::Serendipity, vec![PointIx(0); 15])),
    );
    assert!(r.is_err());
    // MeshEdit checks counts too.
    let mut h = ho_hex(Basis::LagrangeEquispaced, 2, 0).unwrap();
    let mut rec = h.high_order_cell(CellIx(0)).unwrap().clone();
    rec.nodes.pop();
    assert!(MeshEdit::set_high_order(&mut h, CellIx(0), Some(rec)).is_err());
    MeshEdit::set_high_order(&mut h, CellIx(0), None).unwrap();
    assert!(h.high_order_cell(CellIx(0)).is_none());
    assert_valid(&h);
}

fn prism() -> (ReferenceMesh, CellIx) {
    let mut b = ReferenceBuilder::new();
    let ring = |z: f64, b: &mut ReferenceBuilder, base: u64| -> Vec<VertexIx> {
        (0..5u64)
            .map(|i| {
                let a = 2.0 * std::f64::consts::PI * i as f64 / 5.0;
                let v = b.add_vertex(Point3::new(a.cos(), a.sin(), z)).unwrap();
                b.set_class(
                    MeshEntityRef::Vertex(v),
                    on(EntityKind::TopoVertex, base + i),
                )
                .unwrap();
                v
            })
            .collect()
    };
    let bot = ring(0.0, &mut b, 1);
    let top = ring(2.0, &mut b, 6);
    let mut loops: Vec<Vec<VertexIx>> = vec![bot.iter().rev().copied().collect(), top.clone()];
    for i in 0..5 {
        let j = (i + 1) % 5;
        loops.push(vec![bot[i], bot[j], top[j], top[i]]);
    }
    let refs: Vec<&[VertexIx]> = loops.iter().map(Vec::as_slice).collect();
    let added = b.add_polyhedron(&refs).unwrap();
    b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
    for (i, sf) in added.faces.iter().enumerate() {
        b.set_class(
            MeshEntityRef::Face(sf.face()),
            on(EntityKind::TopoFace, 1 + i as u64),
        )
        .unwrap();
    }
    // The builder refuses high order on the polyhedron.
    let nodes: Vec<NodeRef> = bot
        .iter()
        .chain(&top)
        .map(|&v| NodeRef::Vertex(v))
        .collect();
    assert!(matches!(
        b.set_high_order(added.cell, 2, Basis::Serendipity, &nodes),
        Err(MeshError::Unsupported(_))
    ));
    (b.finish(&mut SerialAlloc::new()).unwrap().mesh, added.cell)
}

#[test]
fn face_curvature_side_table_follows_loop_reorientation() {
    // Importer path: faces are given with inward loops, so finish reverses and rotates them.
    let mut b = ReferenceBuilder::new();
    let pts = template_points(CellShape::Hex);
    let vs: Vec<VertexIx> = pts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
            b.set_class(
                MeshEntityRef::Vertex(v),
                on(EntityKind::TopoVertex, 1 + i as u64),
            )
            .unwrap();
            v
        })
        .collect();
    classify_template_edges(&mut b, CellShape::Hex, &vs);
    let mut sfs = Vec::new();
    let mut inward: Vec<FaceIx> = Vec::new();
    for lf in CellShape::Hex.local_faces() {
        let l: Vec<VertexIx> = lf.iter().rev().map(|&i| vs[i as usize]).collect();
        let f = b.add_face(&l).unwrap();
        inward.push(f);
        sfs.push(SignedFace::new(f, Side::Right));
    }
    let cell = b.add_cell(&sfs, None).unwrap();
    b.set_class(MeshEntityRef::Cell(cell), vol()).unwrap();
    for (i, f) in inward.iter().enumerate() {
        b.set_class(
            MeshEntityRef::Face(*f),
            on(EntityKind::TopoFace, 1 + i as u64),
        )
        .unwrap();
    }
    // Curve face 3 (+x): loop corners, then edge midpoints, then the centre, relative to the builder loop.
    let target = inward[3];
    let lp = b.face_loop(target).to_vec();
    let p = |v: VertexIx| b.vertex_point(v);
    let mut nodes: Vec<NodeRef> = lp.iter().map(|&v| NodeRef::Vertex(v)).collect();
    let mut mids = Vec::new();
    for i in 0..4 {
        mids.push(mid(p(lp[i]), p(lp[(i + 1) % 4])));
    }
    let mut bulged = Vec::new();
    for m in &mids {
        bulged.push(Point3::new(m.x + 0.1, m.y, m.z));
    }
    for m in &bulged {
        nodes.push(b.add_extra_node(*m).unwrap());
    }
    let centre = Point3::new(1.2, 0.5, 0.5);
    nodes.push(b.add_extra_node(centre).unwrap());
    b.set_face_curve(target, 2, Basis::LagrangeEquispaced, &nodes)
        .unwrap();
    // Wrong counts, wrong prefix and unsupported degrees are rejected.
    assert!(
        b.set_face_curve(target, 2, Basis::LagrangeEquispaced, &nodes[..8])
            .is_err()
    );
    assert!(matches!(
        b.set_face_curve(target, 3, Basis::LagrangeEquispaced, &nodes),
        Err(MeshError::Unsupported(_))
    ));
    let mut swapped = nodes.clone();
    swapped.swap(0, 1);
    assert!(
        b.set_face_curve(inward[3], 2, Basis::LagrangeEquispaced, &swapped)
            .is_err()
    );
    let fin = b.finish(&mut SerialAlloc::new()).unwrap();
    let m = fin.mesh;
    assert_valid(&m);
    let f = fin.face_remap[target.get()];
    let fc = m.face_curve(f).unwrap();
    assert_eq!(
        (fc.degree, fc.basis, fc.nodes.len()),
        (2, Basis::LagrangeEquispaced, 9)
    );
    let l = m.face_vertices(f);
    // Prefix equals the final loop; edge node j sits on final edge j (bulged by +0.1 in x); centre kept.
    for j in 0..4 {
        assert_eq!(fc.nodes[j].get(), l[j].get());
        let (a, c) = (m.point(l[j].get()), m.point(l[(j + 1) % 4].get()));
        let e = m.point(fc.nodes[4 + j].get());
        assert!(
            (e.x - ((a.x + c.x) / 2.0 + 0.1)).abs() < 1e-15
                && (e.y - (a.y + c.y) / 2.0).abs() < 1e-15
                && (e.z - (a.z + c.z) / 2.0).abs() < 1e-15,
            "edge {j}"
        );
    }
    assert_eq!(m.point(fc.nodes[8].get()), centre);
    // Other faces stay flat; the cell carries no volume record.
    assert_eq!(
        (0..6)
            .filter(|&i| m.face_curve(FaceIx::new(i)).is_some())
            .count(),
        1
    );
    assert!(m.high_order_cell(CellIx(0)).is_none());
    // The hash sees the curve.
    let mut flat = m.clone();
    MeshEdit::set_face_curve(&mut flat, f, None).unwrap();
    assert_ne!(topology_hash(&flat), topology_hash(&m));
}

#[test]
fn a_finished_mesh_can_be_curved_with_mesh_edit() {
    let mut m = ho_hex(Basis::LagrangeEquispaced, 2, 0).unwrap();
    let f = FaceIx(0);
    let l: Vec<VertexIx> = m.face_vertices(f).to_vec();
    let mut nodes: Vec<PointIx> = l.iter().map(|&v| v.into()).collect();
    for i in 0..4 {
        let pm = mid(m.point(l[i].get()), m.point(l[(i + 1) % 4].get()));
        nodes.push(MeshEdit::add_extra_node(&mut m, pm).unwrap());
    }
    // Serendipity quad: 8 nodes.
    MeshEdit::set_face_curve(&mut m, f, Some((2, Basis::Serendipity, nodes.clone()))).unwrap();
    assert_eq!(m.face_curve(f).unwrap().nodes.len(), 8);
    assert_valid(&m);
    // Lagrange needs the centre node too.
    assert!(
        MeshEdit::set_face_curve(
            &mut m,
            f,
            Some((2, Basis::LagrangeEquispaced, nodes.clone()))
        )
        .is_err()
    );
    nodes.push(MeshEdit::add_extra_node(&mut m, Point3::new(0.5, 0.5, 0.0)).unwrap());
    MeshEdit::set_face_curve(&mut m, f, Some((2, Basis::LagrangeEquispaced, nodes))).unwrap();
    assert_valid(&m);
    // Nodes that are not the loop vertices first are rejected.
    let bad: Vec<PointIx> = (0..8).map(|i| PointIx(8 + i)).collect();
    assert!(MeshEdit::set_face_curve(&mut m, f, Some((2, Basis::Serendipity, bad))).is_err());
}
