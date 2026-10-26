//! Acceptance 1 and 2: merge and merge_check on bricks.

mod common;

use common::*;
use sm_boolean::{MergeKind, MergeKinds, MergeOptions, MergeStatus, merge, merge_check};
use sm_core::EntityId;
use sm_topo_api::fixtures;
use sm_topo_api::{CellComplex, Coincidence, Dim, LineageKind, Side, TopologyEdit, TopologyView, TopologyViewExt};

fn two_bricks_gap<E: TopologyEdit + ?Sized>(t: &mut E, gap: f64) -> (fixtures::BrickIds, fixtures::BrickIds) {
    let a = brick(t, 1, [0.0; 3], [1.0; 3]);
    let b = brick(t, 2, [1.0 + gap, 0.0, 0.0], [1.0; 3]);
    (a, b)
}

fn run_merge_case<E: TopologyEdit + Default>() {
    let mut t = E::default();
    let (a, b) = {
        t.begin_op(sm_topo_api::OpId(1)).unwrap();
        let r = fixtures::two_bricks(&mut t).unwrap();
        t.end_op().unwrap();
        r
    };
    let rep = merge_check(&t, &[], &[], &MergeOptions::default());
    assert_eq!(rep.with_status(MergeKind::Vertex, MergeStatus::Mergeable).count(), 4);
    assert_eq!(rep.with_status(MergeKind::Edge, MergeStatus::Mergeable).count(), 4);
    assert_eq!(rep.with_status(MergeKind::Face, MergeStatus::Mergeable).count(), 1);
    let r = in_op(&mut t, 2, |t| merge(t, &[], &[], &MergeOptions::default())).unwrap();
    assert_eq!(r.merged.len(), 9);
    assert_eq!(counts(&t), [12, 20, 11, 2, 2]);
    assert_eq!(euler(&t), 3);
    assert!((total_area(&t) - 11.0).abs() < 1e-12);
    assert_valid(&t);
    // Shared face: left is the lower volume id.
    let face = a.faces[1];
    assert!(t.is_alive(face));
    assert!(!t.is_alive(b.faces[0]));
    let sides = t.face_sides(face).unwrap();
    assert_eq!(sides.left, sm_topo_api::Region::Volume(a.volume));
    assert_eq!(sides.right, sm_topo_api::Region::Volume(b.volume));
    assert!(t.coincident_faces(face).iter().any(|p| p.status == Coincidence::Merged));
    let dropped = t.lineage().records().iter().find(|r| r.entity == b.faces[0] && matches!(r.kind, LineageKind::Merged { .. }));
    assert!(dropped.is_some(), "lineage Merged on the dropped face");
    let _ = Side::Left;
}

#[test]
fn merge_two_bricks_cell_complex() {
    run_merge_case::<CellComplex>();
}

#[test]
fn merge_two_bricks_topo_store() {
    run_merge_case::<sm_topo::TopoStore>();
}

/// Acceptance 1: the merge reproduces `merged_bricks()`. Only the order of the vertex-merge
/// records differs (the fixture merges in (j, k) loop order, the op in ascending pair order, as
/// the `TopologyEdit` contract asks), so the hash differs; the structure and the multiset of
/// lineage records are identical.
#[test]
fn merged_bricks_matches_the_testkit_fixture_up_to_record_order() {
    let mut t = CellComplex::new();
    t.begin_op(sm_topo_api::OpId(1)).unwrap();
    fixtures::two_bricks(&mut t).unwrap();
    t.end_op().unwrap();
    in_op(&mut t, 2, |t| merge(t, &[], &[], &MergeOptions::default())).unwrap();
    let want = sm_testkit::topo::merged_bricks().unwrap();
    assert_eq!(counts(&t), want.counts);
    let key = |c: &CellComplex| {
        let mut v: Vec<String> = c.lineage().records().iter().map(|r| format!("{r:?}")).collect();
        v.sort();
        v
    };
    assert_eq!(key(&t), key(&want.complex));
    for d in Dim::ALL {
        assert_eq!(t.cells(d), want.complex.cells(d));
        for &id in t.cells(d) {
            assert_eq!(t.boundary(id), want.complex.boundary(id), "{id}");
        }
    }
}

#[test]
fn tolerance_near_then_merge_with_tol() {
    let mut t = CellComplex::new();
    let (a, b) = two_bricks_gap(&mut t, 1e-5);
    let rep = merge_check(&t, &[], &[], &MergeOptions::default());
    let faces: Vec<_> = rep.with_status(MergeKind::Face, MergeStatus::Near).collect();
    assert_eq!(faces.len(), 1, "{rep:#?}");
    assert!((faces[0].gap - 1e-5).abs() < 1e-12);
    assert_eq!((faces[0].a, faces[0].b), (a.faces[1], b.faces[0]));
    // Default merge leaves everything alone.
    let r = in_op(&mut t, 3, |t| merge(t, &[], &[], &MergeOptions::default())).unwrap();
    assert!(r.merged.is_empty());
    assert_eq!(counts(&t)[2], 12);
    // tol too large is invalid.
    let bad = MergeOptions { tol: Some(1.0), kinds: MergeKinds::default() };
    assert!(matches!(in_op(&mut t, 4, |t| merge(t, &[], &[], &bad)), Err(sm_boolean::BooleanError::Invalid(_))));
    assert!(merge_check(&t, &[], &[], &bad).error.is_some());
    // tol = 2e-5 merges and records the gap.
    let ok = MergeOptions { tol: Some(2e-5), kinds: MergeKinds::default() };
    let r = in_op(&mut t, 5, |t| merge(t, &[], &[], &ok)).unwrap();
    assert_eq!(r.merged.len(), 9);
    assert_eq!(counts(&t)[2], 11);
    let rec = t.lineage().records().iter().find(|r| r.entity == b.faces[0] && matches!(r.kind, LineageKind::Merged { .. })).unwrap();
    match rec.kind {
        LineageKind::Merged { gap, .. } => assert!((gap - 1e-5).abs() < 1e-12, "gap {gap}"),
        _ => unreachable!(),
    }
    assert_valid(&t);
}

#[test]
fn skipped_faces_are_reported_and_never_merged() {
    let mut t = CellComplex::new();
    t.begin_op(sm_topo_api::OpId(1)).unwrap();
    let (a, b) = fixtures::two_bricks(&mut t).unwrap();
    t.end_op().unwrap();
    let skip = [a.faces[1]];
    let rep = merge_check(&t, &[], &skip, &MergeOptions::default());
    let sk: Vec<_> = rep.with_status(MergeKind::Face, MergeStatus::Skipped).collect();
    assert_eq!(sk.len(), 1);
    assert_eq!(sk[0].reason, Some(sm_boolean::SkipReason::Interface));
    let r = in_op(&mut t, 2, |t| merge(t, &[], &skip, &MergeOptions::default())).unwrap();
    assert_eq!(r.merged.iter().filter(|m| m.kind == MergeKind::Face).count(), 0);
    assert!(t.is_alive(b.faces[0]));
    assert_eq!(t.cells(Dim::Face).len(), 12);
    let _: Option<EntityId> = None;
}
