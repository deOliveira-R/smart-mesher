//! Acceptance 3: imprint and merge of partially overlapping bricks.

mod common;

use common::*;
use sm_boolean::{ImprintOptions, MergeKind, MergeOptions, MergeStatus, imprint_bodies, merge, merge_check};
use sm_topo_api::{CellComplex, Coincidence, Dim, TopologyView, TopologyViewExt};

#[test]
fn partial_overlap_imprint_then_merge() {
    let mut t = CellComplex::new();
    let a = brick(&mut t, 1, [0.0; 3], [2.0; 3]);
    let b = brick(&mut t, 2, [2.0, 1.0, 0.0], [2.0; 3]);
    let rep = merge_check(&t, &[], &[], &MergeOptions::default());
    let over: Vec<_> = rep.with_status(MergeKind::Face, MergeStatus::Overlapping).collect();
    assert_eq!(over.len(), 1, "{rep:#?}");
    assert!((over[0].overlap_area.unwrap() - 2.0).abs() < 1e-12);
    let out = in_op(&mut t, 3, |t| imprint_bodies(t, &[a.body, b.body], &ImprintOptions::default())).unwrap();
    assert_eq!(counts(&t), [20, 30, 14, 2, 2], "faces {:?}", out.faces);
    assert_valid(&t);
    let pairs: Vec<_> = t.coincidences().iter().filter(|p| p.status == Coincidence::ImprintedNotMerged).collect();
    assert_eq!(pairs.len(), 1, "{:?}", t.coincidences());
    // Total area is unchanged by imprinting.
    assert!((total_area(&t) - 48.0).abs() < 1e-12);
    // Merge: V16 E26 F13.
    in_op(&mut t, 4, |t| merge(t, &[], &[], &MergeOptions::default())).unwrap();
    assert_eq!(counts(&t), [16, 26, 13, 2, 2]);
    assert_eq!(euler(&t), 3);
    assert!((total_area(&t) - 46.0).abs() < 1e-12);
    assert_valid(&t);
    // The shared face has area 2 and two sides.
    let shared: Vec<_> = t.cells(Dim::Face).iter().copied().filter(|&f| t.is_interface_face(f)).collect();
    assert_eq!(shared.len(), 1);
    assert!((t.face_moments(shared[0]).unwrap().area - 2.0).abs() < 1e-12);
    // The L-shaped (here rectangular) remainders have area 2 each.
    let rest: Vec<f64> = t.cells(Dim::Face).iter().copied().filter(|&f| {
        let xs: Vec<f64> = t.face_outline(f).iter().map(|p| p.x).collect();
        xs.iter().all(|x| (x - 2.0).abs() < 1e-12) && !t.is_interface_face(f)
    }).map(|f| t.face_moments(f).unwrap().area).collect();
    assert_eq!(rest.len(), 2);
    assert!(rest.iter().all(|a| (a - 2.0).abs() < 1e-12), "{rest:?}");
}
