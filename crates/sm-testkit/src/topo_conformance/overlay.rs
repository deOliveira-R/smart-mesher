//! Conformance scenarios: The decomposition overlay over a face-split host.
#![allow(clippy::too_many_arguments, clippy::type_complexity, clippy::needless_range_loop)]


use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{CurveDef, Interval, Line};
use sm_topo_api::fixtures::{self, BrickIds, with_op};
use super::{Mk, Scenario, Sys};
use sm_topo_api::*;
use std::sync::Arc;

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

fn assert_valid(v: &dyn TopologyView) {
    let bad = validate(v);
    assert!(bad.is_empty(), "violations: {bad:#?}");
    let bad = validate_geometry(v);
    assert!(bad.is_empty(), "geometry violations: {bad:#?}");
}

/// A unit brick whose top face is split into two halves by the corner-to-corner diagonal.
fn brick_with_split_top<E: Sys>(make: &Mk<E>) -> (E, BrickIds, [EntityId; 2], EntityId) {
    let mut t = make(None);
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let (halves, chord) = with_op(&mut t, OpId(2), |t| {
        let line = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 1.0, 0.0)).unwrap())).unwrap();
        let chord = t.make_edge(b.vertices[4], b.vertices[7], Some(CurveCarrier { curve: line, range: Interval::new(0.0, 2f64.sqrt()), sense: Sense::Forward }))?;
        Ok((t.split_face(b.faces[5], FaceCut::Chord { edge: chord }, InnerLoopAssign::None)?, chord))
    })
    .unwrap();
    (t, b, halves, chord)
}

pub(super) fn composite_hides_the_interior_without_touching_the_host_and_undo_restores_it<E: Sys>(make: &Mk<E>) {
    let (t, b, [f0, f1], chord) = brick_with_split_top(make);
    let host: Arc<dyn TopologyView> = Arc::new(t.clone());
    let h0 = host.canonical_hash();
    let mut ov = Overlay::new(host.clone());
    assert_eq!(ov.canonical_hash(), h0, "an empty overlay is indistinguishable from the host");
    assert_valid(&ov);

    let vf = ov.composite_faces(&[f1, f0]).unwrap();
    assert_eq!(vf.kind(), EntityKind::PartitionFace);
    assert_eq!(ov.dim_of(vf), Some(Dim::Face));
    assert_eq!(ov.state(vf), Some(EntityState::Live));
    assert_eq!(host.canonical_hash(), h0, "the host hash is unchanged");
    assert_eq!(t.canonical_hash(), h0);
    // Members and the interior edge are hidden; the corners stay (the boundary edges still use them).
    assert_eq!(ov.cells(Dim::Face).len(), 6);
    assert!(!ov.is_alive(f0) && !ov.is_alive(f1) && !ov.is_alive(chord));
    assert!(ov.boundary(chord).is_empty() && ov.coboundary(chord).is_empty(), "interior edge invisible");
    assert!(ov.is_alive(b.vertices[4]) && ov.is_alive(b.vertices[7]));
    assert_eq!(ov.cells(Dim::Edge).len(), 12);
    assert_eq!(euler_characteristic(&ov), 8 - 12 + 6);
    // The virtual face has a derived boundary, the sides of its members and a composite carrier.
    assert_eq!(ov.loops(vf).count(), 1);
    assert_eq!(ov.coedges(vf).count(), 4);
    assert_eq!(ov.carrier(vf), &Carrier::Composite([f0, f1].into_iter().collect()));
    assert_eq!(ov.resolve(vf), &[f0, f1]);
    assert_eq!(ov.virtual_of(f0), Some(vf));
    assert_eq!(ov.virtual_of(f1), Some(vf));
    let sides = ov.face_sides(vf).unwrap();
    assert_eq!((sides.left, sides.right), (Region::Volume(b.volume), Region::Exterior));
    assert!(ov.volume_faces(b.volume).contains(&vf) && !ov.volume_faces(b.volume).contains(&f0));
    assert_eq!(ov.face_edges(vf).len(), 4);
    assert!(close(ov.face_moments(vf).unwrap().area, 1.0, 1e-9), "composite moments are the sum of the members'");
    assert!(close(ov.volume_moments(b.volume).unwrap().volume, 1.0, 1e-9));
    assert_eq!(ov.face_ordinal(vf), Some(ov.cells(Dim::Face).iter().position(|f| *f == vf).unwrap()));
    assert_valid(&ov);
    // The boundary edges now list the virtual face (and keep their ring size).
    let boundary_edge = ov.boundary(vf)[0].cell;
    assert!(ov.coboundary(boundary_edge).iter().any(|e| e.cell == vf));
    assert!(ov.radial_uses(boundary_edge).iter().any(|u| u.face == vf), "ring maps the absorbed face to its virtual face");
    assert_eq!(ov.radial_uses(boundary_edge).len(), 2);

    // Undo: the overlay is the host again, and the id is not reused.
    ov.remove(vf).unwrap();
    assert_eq!(ov.canonical_hash(), h0);
    assert!(ov.is_alive(f0) && ov.is_alive(chord));
    assert!(matches!(ov.remove(vf), Err(TopoError::NoSuchCell(_))));
    let vf2 = ov.composite_faces(&[f0, f1]).unwrap();
    assert_ne!(vf2, vf);
    assert!(vf2 > vf, "monotonic ids");
    assert_eq!(ov.host_hash(), h0);
}

pub(super) fn composite_rejections<E: Sys>(make: &Mk<E>) {
    let (t, b, [f0, f1], _chord) = brick_with_split_top(make);
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let mut ov = Overlay::new(host);
    // Fewer than two faces, unknown/dead faces, faces that are not faces.
    assert!(ov.composite_faces(&[f0]).is_err());
    assert!(ov.composite_faces(&[f0, b.vertices[0]]).is_err());
    // Not adjacent: opposite faces share no edge.
    assert!(matches!(ov.composite_faces(&[b.faces[0], b.faces[1]]), Err(TopoError::InvalidParameter(_))));
    // Exactly one top half borders the -x face through a brick edge: that composite is fine.
    let adjacent = |f: EntityId| ov.face_edges(f).iter().any(|e| ov.face_edges(b.faces[0]).contains(e));
    let (near, far) = if adjacent(f0) { (f0, f1) } else { (f1, f0) };
    assert!(matches!(ov.composite_faces(&[far, b.faces[0]]), Err(TopoError::InvalidParameter(_))), "not adjacent");
    let vf = ov.composite_faces(&[near, b.faces[0]]).unwrap();
    // Absorbed faces cannot be used again; failed attempts do not consume ids.
    assert!(ov.composite_faces(&[near, b.faces[2]]).is_err());
    let next = ov.composite_faces(&[far, b.faces[1]]).unwrap();
    assert_eq!(next.serial(), vf.serial() + 1);
}

pub(super) fn composite_rejects_different_volumes_sides_and_foreign_edges<E: Sys>(make: &Mk<E>) {
    // Webcut halves belong to different volumes.
    let mut t = make(None);
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let w = with_op(&mut t, OpId(2), |t| fixtures::webcut_brick(t, &b, 0.5)).unwrap();
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let mut ov = Overlay::new(host.clone());
    let err = ov.composite_faces(&[w.split_faces[3][0], w.split_faces[3][1]]);
    assert!(matches!(err, Err(TopoError::Unsupported(_))), "{err:?}");
    // The cap (interface, two volumes) and a side half (one volume) have different parents.
    assert!(ov.composite_faces(&[w.cap, w.split_faces[0][0]]).is_err());
    assert_eq!(ov.ops().len(), 0, "rejections leave no trace");
    assert_eq!(ov.canonical_hash(), host.canonical_hash());

    // A face sharing an edge with a face outside the composite (non-manifold fin).
    let mut t2 = make(None);
    let (b2, fin) = with_op(&mut t2, OpId(1), fixtures::brick_with_fin).unwrap();
    let host2: Arc<dyn TopologyView> = Arc::new(t2);
    let mut ov2 = Overlay::new(host2);
    let err = ov2.composite_faces(&[b2.faces[0], b2.faces[2]]);
    assert!(matches!(err, Err(TopoError::Unsupported(_))), "{err:?} (the fin also uses the shared edge)");
    let _ = fin;
}

pub(super) fn partitions_use_real_cut_geometry_and_undo_in_dependency_order<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let h0 = host.canonical_hash();
    let mut ov = Overlay::new(host.clone());
    // Top face z = 1 (corners 4..8). Add a virtual vertex at the middle of the y-edge at x = 0
    // (edge 4 + 0 + 2 = 6 joins corners (0,0,1) and (0,1,1)), then cut from the opposite corner.
    let y_edge = b.edges[6];
    let (mid, halves) = ov.partition_edge(y_edge, 0.5).unwrap();
    assert_eq!(mid.kind(), EntityKind::PartitionVertex);
    assert!(halves.iter().all(|e| e.kind() == EntityKind::PartitionEdge));
    assert!(!ov.is_alive(y_edge));
    assert_eq!(ov.virtual_of(y_edge), Some(halves[0]));
    assert_eq!(ov.cells(Dim::Vertex).len(), 9);
    assert_eq!(ov.cells(Dim::Edge).len(), 13);
    assert_eq!(ov.edge_ends(halves[0]), Some((b.vertices[6 - 2], mid)).or_else(|| ov.edge_ends(halves[0])));
    let Carrier::Point(p) = ov.carrier(mid) else { panic!() };
    assert!(close(p.x, 0.0, 1e-12) && close(p.y, 0.5, 1e-12) && close(p.z, 1.0, 1e-12), "{p:?}");
    assert_valid(&ov);
    assert_eq!(euler_characteristic(&ov), 9 - 13 + 6);

    // Chord from the virtual vertex to the far corner (1,0,1) [vertex 5].
    let cut = VirtualCut {
        from: mid,
        to: b.vertices[5],
        curve: CurveDef::Line(Line::new(Point3::new(0.0, 0.5, 1.0), Vector3::new(1.0, -0.5, 0.0)).unwrap()),
        range: Interval::new(0.0, (1.0f64 + 0.25).sqrt()),
    };
    let [c0, c1] = ov.partition_face(b.faces[5], cut.clone()).unwrap();
    assert!(c0.kind() == EntityKind::PartitionFace && c1.kind() == EntityKind::PartitionFace);
    assert!(!ov.is_alive(b.faces[5]));
    assert_eq!(ov.cells(Dim::Face).len(), 7);
    assert_eq!(ov.cells(Dim::Edge).len(), 14);
    assert_eq!(ov.resolve(c0), &[b.faces[5]]);
    assert_eq!(ov.face_sides(c0).unwrap().left, Region::Volume(b.volume));
    assert_eq!(ov.face_sides(c1).unwrap().left, Region::Volume(b.volume));
    assert_eq!(euler_characteristic(&ov), 9 - 14 + 7);
    assert_valid(&ov);
    assert!(matches!(ov.face_moments(c0), Err(QueryError::Unsupported(_))));
    assert_eq!(host.canonical_hash(), h0);
    // The chord curve lives in the overlay's own geometry layer.
    let chord = ov.coedges(c0).chain(ov.coedges(c1)).map(|c| c.key.edge).find(|e| e.kind() == EntityKind::PartitionEdge && !halves.contains(e)).unwrap();
    let Carrier::Curve(cc) = ov.carrier(chord) else { panic!() };
    assert!(ov.shapes().curve(cc.curve).is_some() && host.shapes().curve(cc.curve).is_none());
    assert_eq!(ov.shapes().curve_count(), host.shapes().curve_count() + 1);

    // Persistence: the recorded operations replay to the same overlay.
    let data = ov.data();
    let replayed = Overlay::from_data(host.clone(), data.clone()).unwrap();
    assert_eq!(replayed.canonical_hash(), ov.canonical_hash());
    assert_eq!(replayed.ops(), ov.ops());

    // Undo in dependency order: the face cut depends on the virtual vertex.
    assert!(matches!(ov.remove(mid), Err(TopoError::StillReferenced(_))));
    ov.remove(c0).unwrap();
    assert!(ov.is_alive(b.faces[5]));
    ov.remove(mid).unwrap();
    assert_eq!(ov.canonical_hash(), h0, "everything undone");
    let (mid2, _) = ov.partition_edge(y_edge, 0.25).unwrap();
    assert!(mid2 > mid, "ids are never reused");
    // A failed partition leaves nothing behind.
    let bad = VirtualCut { from: b.vertices[0], to: b.vertices[0], ..cut };
    let before = ov.canonical_hash();
    assert!(ov.partition_face(b.faces[5], bad).is_err());
    assert_eq!(ov.canonical_hash(), before);
}

pub(super) fn stale_overlays_are_detected_and_can_be_rebased<E: Sys>(make: &Mk<E>) {
    let (mut t, b, [f0, f1], _chord) = brick_with_split_top(make);
    let host: Arc<dyn TopologyView> = Arc::new(t.clone());
    let mut ov = Overlay::new(host);
    let vf = ov.composite_faces(&[f0, f1]).unwrap();
    assert!(!ov.is_stale_against(&t));
    assert_eq!(ov.host_revision(), t.revision());
    // The host moves on (an unrelated brick is added): the overlay is stale until rebased.
    with_op(&mut t, OpId(3), |t| fixtures::brick(t, Point3::new(5.0, 0.0, 0.0), [1.0; 3])).unwrap();
    assert!(ov.is_stale_against(&t));
    assert!(matches!(Overlay::from_data(Arc::new(t.clone()), ov.data()), Err(TopoError::StaleOverlay { .. })));
    ov.rebase(Arc::new(t.clone())).unwrap();
    assert!(!ov.is_stale_against(&t));
    assert!(ov.is_alive(vf) && ov.cells(Dim::Volume).len() == 2);
    assert_valid(&ov);
    // A host change that invalidates an operation (the member faces die with their volume) makes
    // rebase fail and leaves the overlay untouched.
    with_op(&mut t, OpId(4), |t| Ok(t.kill_cascade(b.volume)?)).unwrap();
    let before = ov.canonical_hash();
    assert!(ov.rebase(Arc::new(t.clone())).is_err());
    assert_eq!(ov.canonical_hash(), before);
}

pub(super) fn overlay_is_object_safe_and_usable_as_a_view<E: Sys>(make: &Mk<E>) {
    let (t, _b, [f0, f1], _chord) = brick_with_split_top(make);
    let mut ov = Overlay::new(Arc::new(t));
    ov.composite_faces(&[f0, f1]).unwrap();
    let v: &dyn TopologyView = &ov;
    assert_eq!(v.cells(Dim::Face).len(), 6);
    let o: &dyn VirtualOverlay = &ov;
    assert_eq!(o.host_revision(), OpId(2));
}

pub(super) fn scenarios<E: Sys>() -> Vec<Scenario<E>> {
    vec![
        ("composite_hides_the_interior_without_touching_the_host_and_undo_restores_it", composite_hides_the_interior_without_touching_the_host_and_undo_restores_it::<E>),
        ("composite_rejections", composite_rejections::<E>),
        ("composite_rejects_different_volumes_sides_and_foreign_edges", composite_rejects_different_volumes_sides_and_foreign_edges::<E>),
        ("partitions_use_real_cut_geometry_and_undo_in_dependency_order", partitions_use_real_cut_geometry_and_undo_in_dependency_order::<E>),
        ("stale_overlays_are_detected_and_can_be_rebased", stale_overlays_are_detected_and_can_be_rebased::<E>),
        ("overlay_is_object_safe_and_usable_as_a_view", overlay_is_object_safe_and_usable_as_a_view::<E>),
    ]
}
