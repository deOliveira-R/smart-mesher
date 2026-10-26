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
    // Bullet 1 at op level: counts, area, and the structure of `merged_bricks` (bricks made by
    // `model.create_brick` get other ids than the fixture's, so compare id-free signatures; the
    // canonical hash is not compared because the model layer's bookkeeping differs).
    let t = r.topo();
    assert_eq!(counts(t), [12, 20, 11, 2, 2]);
    assert!((total_area(t) - 11.0).abs() < 1e-12, "{}", total_area(t));
    let want = sm_testkit::topo::merged_bricks().unwrap();
    assert_eq!(counts(t), want.counts);
    let sig = |t: &dyn sm_topo_api::TopologyView| -> Vec<(Dim, usize, usize)> {
        let mut v: Vec<_> = Dim::ALL.iter().flat_map(|&d| t.cells(d).iter().map(move |&c| (d, t.boundary(c).len(), t.coboundary(c).len())).collect::<Vec<_>>()).collect();
        v.sort();
        v
    };
    assert_eq!(sig(t), sig(&want.complex), "boundary/coboundary degrees per cell");
    let merged = |t: &dyn sm_topo_api::TopologyView| t.lineage().records().iter().filter(|r| matches!(r.kind, sm_topo_api::LineageKind::Merged { .. })).count();
    assert_eq!(merged(t), merged(&want.complex), "Merged lineage records");
    assert_eq!(merged(t), 9);
    let merged_pairs = |t: &dyn sm_topo_api::TopologyView| t.coincidences().iter().filter(|p| p.status == sm_topo_api::Coincidence::Merged).count();
    assert_eq!(merged_pairs(t), merged_pairs(&want.complex));
    assert_eq!(merged_pairs(t), 1);
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
