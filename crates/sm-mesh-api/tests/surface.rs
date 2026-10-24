//! Dimension-2 (surface) meshes and classification onto decomposition (`Partition*`) entities.

mod common;

use common::{id, on};
use sm_core::{EntityKind, Point3, Vector3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh, hex_block};
use sm_mesh_api::*;

const TOL: f64 = 1e-12;

/// Two unit quads and a triangle in the plane z = 0 (total area 2.5), consistently oriented.
/// Boundary facets on `TopoEdge#(1 + i)`, cells on `TopoFace#1` (quads) and `TopoFace#2` (tri),
/// so the facet between the second quad and the triangle needs an explicit class.
fn strip(b: &mut ReferenceBuilder) -> Vec<CellAdded> {
    let pts = [
        [0., 0.],
        [1., 0.],
        [2., 0.],
        [0., 1.],
        [1., 1.],
        [2., 1.],
        [3., 0.5],
    ];
    let vs: Vec<VertexIx> = pts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], 0.0)).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + i as u64))
                .unwrap();
            v
        })
        .collect();
    let q1 = b
        .add_cell_from_vertices(CellShape::Quad, &[vs[0], vs[1], vs[4], vs[3]])
        .unwrap();
    let q2 = b.add_polygon(&[vs[1], vs[2], vs[5], vs[4]]).unwrap();
    let t = b.add_polygon(&[vs[2], vs[6], vs[5]]).unwrap();
    for (c, f) in [(q1.cell, 1), (q2.cell, 1), (t.cell, 2)] {
        b.set_class(MeshEntityRef::Cell(c), on(EntityKind::TopoFace, f))
            .unwrap();
    }
    let mut k = 1;
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        if b.face_degree(f) == 1 {
            b.set_class(MeshEntityRef::Face(f), on(EntityKind::TopoEdge, k))
                .unwrap();
            k += 1;
        }
    }
    // The facet between q2 and the triangle lies on the curve between the two surfaces.
    let (f, _) = b.find_face(&[vs[2], vs[5]]).unwrap();
    b.set_class(MeshEntityRef::Face(f), on(EntityKind::TopoEdge, 99))
        .unwrap();
    vec![q1, q2, t]
}

fn build() -> (ReferenceMesh, SerialAlloc) {
    let mut alloc = SerialAlloc::new();
    let mut b = ReferenceBuilder::surface();
    assert_eq!(b.dim(), MeshDim::Two);
    strip(&mut b);
    let m = b.finish(&mut alloc).unwrap().mesh;
    (m, alloc)
}

#[test]
fn surface_mesh_keeps_the_face_first_invariants_one_level_down() {
    let (m, _) = build();
    assert_eq!(m.dim(), MeshDim::Two);
    assert!(validate(&m).is_empty(), "{:?}", validate(&m));
    assert!(validate_geometry(&m, &GeomTol::default()).is_empty());
    assert_eq!(m.cell_count(), 3);
    // 4 + 4 + 3 edges minus 2 shared.
    assert_eq!(m.face_count(), 9);
    let shapes: Vec<CellShape> = (0..3).map(|c| m.cell_shape(CellIx(c))).collect();
    assert_eq!(shapes, [CellShape::Quad, CellShape::Quad, CellShape::Tri]);
    let mut interior = 0;
    for f in 0..m.face_count() {
        let fi = FaceIx::new(f);
        let l = m.face_vertices(fi);
        assert_eq!(l.len(), 2);
        assert_eq!(m.face_shape(fi), FaceShape::Segment);
        // The left cell traverses the facet forward, the right cell backward.
        let left = m.face_left(fi);
        let cyc = m.cell_corners(left).unwrap();
        let n = cyc.len();
        assert!((0..n).any(|i| cyc[i] == l[0] && cyc[(i + 1) % n] == l[1]));
        if let Some(r) = m.face_right(fi) {
            interior += 1;
            assert!(left < r);
            let rc = m.cell_corners(r).unwrap();
            let n = rc.len();
            assert!((0..n).any(|i| rc[i] == l[1] && rc[(i + 1) % n] == l[0]));
            assert_eq!(m.face_kind(fi), FaceKind::Interior);
        } else {
            assert_eq!(m.face_kind(fi), FaceKind::Boundary);
            assert_eq!(m.face_class(fi).entity.kind(), EntityKind::TopoEdge);
        }
        // Facet "area vector" is the segment vector.
        let a = face_area_vector(&m, fi);
        let (p, q) = (m.point(l[0].get()), m.point(l[1].get()));
        assert!((a - (q - p)).norm() < TOL);
    }
    assert_eq!(interior, 2);
    // The interior facet between the quads was derived from their common surface.
    let quads_shared = (0..m.face_count())
        .map(FaceIx::new)
        .find(|f| m.face_right(*f) == Some(CellIx(1)) && m.face_left(*f) == CellIx(0))
        .unwrap();
    assert_eq!(m.face_class(quads_shared).entity, id(EntityKind::TopoFace, 1));
    // Areas and centroids are analytic.
    let areas: Vec<f64> = (0..3).map(|c| cell_volume(&m, CellIx(c))).collect();
    assert!((areas[0] - 1.0).abs() < TOL && (areas[1] - 1.0).abs() < TOL);
    assert!((areas[2] - 0.5).abs() < TOL);
    let (_, g) = cell_volume_centroid(&m, CellIx(2));
    assert!((g - Point3::new(7.0 / 3.0, 0.5, 0.0)).norm() < TOL);
    let (_, g0) = cell_volume_centroid(&m, CellIx(0));
    assert!((g0 - Point3::new(0.5, 0.5, 0.0)).norm() < TOL);
    // Signed facet vectors of every cell sum to zero (closed loop).
    for c in 0..3 {
        let mut s = Vector3::default();
        for sf in m.cell_faces(CellIx(c)) {
            s = s + face_area_vector(&m, sf.face()) * sf.sign();
        }
        assert!(s.norm() < TOL);
    }
    // Geometry cache round trip.
    let mut m2 = m.clone();
    m2.set_geometry_cache(Some(compute_geometry_cache(&m))).unwrap();
    assert!(validate(&m2).is_empty());
}

#[test]
fn surface_mesh_rules_are_enforced() {
    let mut alloc = SerialAlloc::new();
    // A dimension-2 facet has exactly two vertices; a polygon needs three.
    let mut b = ReferenceBuilder::surface();
    let vs: Vec<VertexIx> = (0..4)
        .map(|i| b.add_vertex(Point3::new(f64::from(i), f64::from(i % 2), 0.0)).unwrap())
        .collect();
    assert!(b.add_face(&[vs[0], vs[1], vs[2]]).is_err());
    assert!(b.add_polygon(&[vs[0], vs[1]]).is_err());
    assert!(b.add_cell_from_vertices(CellShape::Hex, &vs).is_err());
    // Cells lie on face entities, not volumes; there is no edge table.
    let c = b.add_polygon(&[vs[0], vs[1], vs[2]]).unwrap();
    assert!(b
        .set_class(MeshEntityRef::Cell(c.cell), on(EntityKind::TopoVolume, 1))
        .is_err());
    assert!(b
        .set_class(MeshEntityRef::Cell(c.cell), on(EntityKind::PartitionFace, 1))
        .is_ok());
    assert!(b
        .set_class(MeshEntityRef::Edge(vs[0], vs[1]), on(EntityKind::TopoEdge, 1))
        .is_err());
    // Two cells traversing a shared facet the same way are inconsistent.
    let mut b2 = ReferenceBuilder::surface();
    let ws: Vec<VertexIx> = [[0., 0.], [1., 0.], [0., 1.], [1., 1.]]
        .iter()
        .map(|p| b2.add_vertex(Point3::new(p[0], p[1], 0.0)).unwrap())
        .collect();
    b2.add_polygon(&[ws[0], ws[1], ws[2]]).unwrap();
    assert!(matches!(
        b2.add_polygon(&[ws[1], ws[3], ws[2]]).map(|_| ()),
        Ok(())
    ));
    let bad = b2.add_polygon(&[ws[0], ws[1], ws[3]]);
    assert!(
        matches!(bad, Err(MeshError::InconsistentOrientation(_))),
        "{bad:?}"
    );
    // Boundary facets on a TopoFace are rejected by validation in dimension 2.
    let mut b3 = ReferenceBuilder::surface();
    let us: Vec<VertexIx> = [[0., 0.], [1., 0.], [0., 1.]]
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b3.add_vertex(Point3::new(p[0], p[1], 0.0)).unwrap();
            b3.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, i as u64))
                .unwrap();
            v
        })
        .collect();
    let t = b3.add_polygon(&us).unwrap();
    b3.set_class(MeshEntityRef::Cell(t.cell), on(EntityKind::TopoFace, 1))
        .unwrap();
    for sf in &t.faces {
        b3.set_class(MeshEntityRef::Face(sf.face()), on(EntityKind::TopoFace, 2))
            .unwrap();
    }
    let e = b3.finish(&mut alloc).unwrap_err();
    assert!(
        matches!(&e, MeshError::Invalid(v) if v.iter().any(|x| x.code == ViolationCode::ClassKind)),
        "{e:?}"
    );
}

#[test]
fn polygons_and_high_order_triangles() {
    let mut alloc = SerialAlloc::new();
    let mut b = ReferenceBuilder::surface();
    // A regular pentagon (one polygon cell) and a TRI6 next to it.
    let mut vs = Vec::new();
    for i in 0..5 {
        let a = std::f64::consts::TAU * f64::from(i) / 5.0;
        let v = b.add_vertex(Point3::new(a.cos(), a.sin(), 0.0)).unwrap();
        b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoEdge, 1))
            .unwrap();
        vs.push(v);
    }
    let p = b.add_polygon(&vs).unwrap();
    b.set_class(MeshEntityRef::Cell(p.cell), on(EntityKind::TopoFace, 1))
        .unwrap();
    let apex = b.add_vertex(Point3::new(1.5, 0.6, 0.0)).unwrap();
    b.set_class(MeshEntityRef::Vertex(apex), on(EntityKind::TopoEdge, 1))
        .unwrap();
    // The triangle traverses the shared edge 1 -> 0 (the pentagon runs 0 -> 1).
    let t = b
        .add_cell_from_vertices(CellShape::Tri, &[vs[1], vs[0], apex])
        .unwrap();
    b.set_class(MeshEntityRef::Cell(t.cell), on(EntityKind::TopoFace, 1))
        .unwrap();
    let mids: Vec<NodeRef> = [[1.2, 0.4], [1.3, 0.8], [0.6, 0.6]]
        .iter()
        .map(|q| b.add_extra_node(Point3::new(q[0], q[1], 0.0)).unwrap())
        .collect();
    let nodes = [
        NodeRef::Vertex(vs[1]),
        NodeRef::Vertex(vs[0]),
        NodeRef::Vertex(apex),
        mids[0],
        mids[1],
        mids[2],
    ];
    assert_eq!(node_count(CellShape::Tri, 2, Basis::LagrangeEquispaced), Some(6));
    assert_eq!(node_count(CellShape::Quad, 2, Basis::LagrangeEquispaced), Some(9));
    assert_eq!(node_count(CellShape::Quad, 2, Basis::Serendipity), Some(8));
    assert_eq!(node_count(CellShape::Polygon, 2, Basis::Serendipity), None);
    b.set_high_order(t.cell, 2, Basis::LagrangeEquispaced, &nodes)
        .unwrap();
    for a in [&p, &t] {
        for sf in &a.faces {
            if b.face_degree(sf.face()) == 1 {
                b.set_class(MeshEntityRef::Face(sf.face()), on(EntityKind::TopoEdge, 1))
                    .unwrap();
            }
        }
    }
    let m = b.finish(&mut alloc).unwrap().mesh;
    assert!(validate(&m).is_empty(), "{:?}", validate(&m));
    assert_eq!(m.cell_shape(CellIx(0)), CellShape::Polygon);
    assert_eq!(m.cell_corners(CellIx(0)).unwrap(), vs.as_slice());
    assert_eq!(m.cell_shape(CellIx(1)), CellShape::Tri);
    assert!(m.high_order_cell(CellIx(1)).is_some());
    // Area of the regular pentagon with circumradius 1: (5/2) sin(72 deg).
    let want = 2.5 * (std::f64::consts::TAU / 5.0).sin();
    assert!((cell_volume(&m, CellIx(0)) - want).abs() < 1e-12);
    // Dimension-3-only operations refuse a surface mesh.
    let mut m2 = m.clone();
    assert!(matches!(
        m2.ensure_edges(&mut alloc, &[]),
        Err(MeshError::Unsupported(_))
    ));
    let mut out = ReferenceBuilder::new();
    let r = append_transformed(
        &mut out,
        &m,
        id(EntityKind::MeshInstance, 0),
        &sm_core::Transform::IDENTITY,
        &FlattenOptions::default(),
    );
    assert!(matches!(r, Err(MeshError::Unsupported(_))));
}

#[test]
fn surface_meshes_are_deterministic() {
    let run = || {
        let (m, a) = build();
        (
            connectivity_hash(&m),
            topology_hash(&m),
            geometry_hash(&m),
            a.state().collect::<Vec<_>>(),
        )
    };
    let first = run();
    assert_eq!(first, run());
    let handles: Vec<_> = (0..4).map(|_| std::thread::spawn(run)).collect();
    for h in handles {
        assert_eq!(h.join().unwrap(), first);
    }
    // The dimension is part of the connectivity.
    let (m, _) = build();
    let mut alloc = SerialAlloc::new();
    let h3 = hex_block([1, 1, 1], [1.0, 1.0, 1.0], &mut alloc).unwrap();
    assert_ne!(connectivity_hash(&m), connectivity_hash(&h3));
}

#[test]
fn classification_onto_decomposition_entities() {
    // Entity dimensions and parameter fits for the Partition* kinds.
    assert_eq!(entity_dim(EntityKind::PartitionVertex), Some(0));
    assert_eq!(entity_dim(EntityKind::PartitionEdge), Some(1));
    assert_eq!(entity_dim(EntityKind::PartitionFace), Some(2));
    assert_eq!(entity_dim(EntityKind::PartitionCell), Some(3));
    assert_eq!(entity_dim(EntityKind::MeshCell), None);
    assert!(Param::Curve(0.5).fits(EntityKind::PartitionEdge));
    assert!(Param::Surface([0.1, 0.2]).fits(EntityKind::PartitionFace));
    assert!(!Param::Surface([0.1, 0.2]).fits(EntityKind::PartitionEdge));
    assert!(!Param::Volume([0.0; 3]).fits(EntityKind::PartitionCell));
    assert!(!Param::Curve(0.0).fits(EntityKind::PartitionVertex));
    assert!(MeshDim::Three.accepts_facet(FaceKind::Boundary, EntityKind::PartitionFace));
    assert!(!MeshDim::Three.accepts_facet(FaceKind::Boundary, EntityKind::PartitionCell));
    assert!(MeshDim::Two.accepts_facet(FaceKind::Boundary, EntityKind::PartitionEdge));
    assert!(MeshDim::Three.accepts_cell(EntityKind::PartitionCell));
    assert!(MeshDim::Two.accepts_cell(EntityKind::PartitionFace));

    // A hex whose cell lies on a piece, one face on a virtual cut face (with surface
    // parameters), its vertices on virtual vertices and edges: valid, dimension order holds.
    let mut alloc = SerialAlloc::new();
    let mut b = ReferenceBuilder::new();
    let pts = common::template_points(CellShape::Hex);
    let vs: Vec<VertexIx> = pts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
            let c = if p[2] == 0.0 {
                on(EntityKind::PartitionVertex, i as u64)
            } else {
                on(EntityKind::TopoVertex, i as u64)
            };
            b.set_class(MeshEntityRef::Vertex(v), c).unwrap();
            v
        })
        .collect();
    let a = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
    b.set_class(MeshEntityRef::Cell(a.cell), on(EntityKind::PartitionCell, 7))
        .unwrap();
    for (i, sf) in a.faces.iter().enumerate() {
        let c = if i == 0 {
            Classification {
                entity: id(EntityKind::PartitionFace, 3),
                param: Param::Surface([0.5, 0.5]),
            }
        } else {
            on(EntityKind::TopoFace, i as u64)
        };
        b.set_class(MeshEntityRef::Face(sf.face()), c).unwrap();
    }
    b.set_class(
        MeshEntityRef::Edge(vs[0], vs[1]),
        Classification {
            entity: id(EntityKind::PartitionEdge, 2),
            param: Param::Curve(0.25),
        },
    )
    .unwrap();
    for (x, y) in [(1, 2), (2, 3), (3, 0)] {
        b.set_class(MeshEntityRef::Edge(vs[x], vs[y]), on(EntityKind::PartitionEdge, 2))
            .unwrap();
    }
    for (x, y) in [(4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)] {
        b.set_class(MeshEntityRef::Edge(vs[x], vs[y]), on(EntityKind::TopoEdge, 1))
            .unwrap();
    }
    let m = b.finish(&mut alloc).unwrap().mesh;
    assert!(validate(&m).is_empty(), "{:?}", validate(&m));
    let f0 = m.classified_on(id(EntityKind::PartitionFace, 3)).unwrap();
    assert_eq!(f0.faces.len(), 1);
    assert!(m.is_boundary_face(f0.faces[0]));
    assert_eq!(m.classified_on(id(EntityKind::PartitionCell, 7)).unwrap().cells.len(), 1);
}
