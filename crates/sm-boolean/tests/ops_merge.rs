//! Op-level `model.merge` (with the interface skip) and `model.imprint`.

mod common;

use common::rig::*;
use common::*;
use sm_boolean::ops::{Imprint, Merge};
use sm_core::EntityId;
use sm_topo_api::{Dim, Region, TopologyViewExt};

/// The face of `faces` whose centroid lies on the plane `x = x0`.
fn face_at_x(r: &Rig, faces: &[EntityId], x0: f64) -> EntityId {
    let t = r.topo();
    *faces.iter().find(|&&f| t.face_moments(f).unwrap().centroid().is_some_and(|c| (c.x - x0).abs() < 1e-9)).expect("face at x")
}

fn two_touching(r: &mut Rig) -> (Solid, Solid) {
    let a = r.brick([1.0; 3], [0.5, 0.5, 0.5]);
    let b = r.brick([1.0; 3], [1.5, 0.5, 0.5]);
    (a, b)
}

#[test]
fn merge_op_joins_coincident_faces() {
    let mut r = Rig::cc();
    two_touching(&mut r);
    assert_eq!(r.topo().cells(Dim::Face).len(), 12);
    let d = r.run(&Merge { entities: None, kinds: None, tol: None, check_only: None }).unwrap();
    assert_eq!(r.topo().cells(Dim::Face).len(), 11);
    assert_eq!(d.outputs["faces"].len(), 1);
    assert_valid(r.topo());
}

#[test]
fn merge_op_skips_faces_on_an_interface_record() {
    let mut r = Rig::cc();
    let (a, _b) = two_touching(&mut r);
    let f = face_at_x(&r, &a.faces, 1.0);
    r.run(&MarkInterface { faces: vec![f] }).unwrap();
    let d = r.run(&Merge { entities: None, kinds: None, tol: None, check_only: None }).unwrap();
    assert!(d.outputs.get("faces").is_none_or(|v| v.is_empty()));
    assert_eq!(r.topo().cells(Dim::Face).len(), 12, "the interface face stays unmerged");
    assert_valid(r.topo());
}

#[test]
fn imprint_op_splits_overlapping_faces_and_merge_joins_the_overlap() {
    let mut r = Rig::cc();
    let a = r.brick([2.0; 3], [1.0, 1.0, 1.0]);
    let b = r.brick([2.0; 3], [3.0, 2.0, 1.0]);
    let d = r.run(&Imprint { bodies: vec![a.body, b.body], merge: Some(true), tol: None }).unwrap();
    assert_valid(r.topo());
    assert!(!d.outputs["faces"].is_empty());
    // With merge on, the new edges coincide with the other brick's edges and are merged away.
    assert!(d.outputs.get("edges").is_none_or(|v| v.iter().all(|&e| r.topo().is_alive(e))));
    // The 2x1 overlap on x = 2 is now one shared face; each body keeps its own volume.
    let t = r.topo();
    assert!((volume_of(t, a.volume) - 8.0).abs() < 1e-12);
    assert!((volume_of(t, b.volume) - 8.0).abs() < 1e-12);
    assert_eq!(t.cells(Dim::Volume).len(), 2);
    let shared = t.cells(Dim::Face).iter().filter(|f| t.face_sides(**f).is_some_and(|s| matches!(s.left, Region::Volume(_)) && matches!(s.right, Region::Volume(_)))).count();
    assert_eq!(shared, 1);
}
