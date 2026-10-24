//! Conformance scenarios: flag edits are checked at `end_op` like any other touched cell.
use super::{Mk, Scenario, Sys};
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::*;

/// Run `op` with a lone `set_flags(cell, flags)` and return the `end_op` result.
fn toggle<E: Sys>(t: &mut E, op: u64, cell: sm_core::EntityId, flags: CellFlags) -> Result<OpSummary, TopoError> {
    t.begin_op(OpId(op)).unwrap();
    t.set_flags(cell, flags).unwrap();
    t.end_op()
}

fn expect_invalid(r: Result<OpSummary, TopoError>, rules: &[Rule]) {
    match r {
        Err(TopoError::OpInvalid(issues)) => {
            assert!(issues.iter().any(|i| rules.contains(&i.rule)), "expected one of {rules:?}, got {issues:?}");
        }
        other => panic!("expected OpInvalid({rules:?}), got {other:?}"),
    }
}

pub(super) fn clearing_a_pole_flag_in_its_own_op_is_rejected_and_rolled_back<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    let s = with_op(&mut t, OpId(1), |t| fixtures::sphere(t, 2.0)).unwrap();
    let pole = s.edges[0];
    assert!(t.flags(pole).contains(CellFlags::DEGENERATE));
    let before = t.canonical_hash();
    // Only a flag changes: the edge, its face and its volume must still be checked.
    expect_invalid(toggle(&mut t, 2, pole, CellFlags::NONE), &[Rule::EdgeEnds, Rule::ShellOpen]);
    assert!(t.flags(pole).contains(CellFlags::DEGENERATE), "the flag edit was rolled back");
    assert_eq!(t.canonical_hash(), before, "state restored exactly");
    assert!(validate(&t).is_empty());
    // The same operation id is free again.
    let ok = toggle(&mut t, 2, pole, CellFlags::DEGENERATE);
    assert!(ok.is_ok(), "{ok:?}");
}

pub(super) fn setting_degenerate_on_an_open_edge_in_its_own_op_is_rejected<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, sm_core::Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let before = t.canonical_hash();
    expect_invalid(toggle(&mut t, 2, b.edges[0], CellFlags::DEGENERATE), &[Rule::EdgeEnds, Rule::ShellOpen]);
    assert_eq!(t.flags(b.edges[0]), CellFlags::NONE);
    assert_eq!(t.canonical_hash(), before);
    assert!(validate(&t).is_empty());
}

pub(super) fn a_valid_flag_edit_still_commits<E: Sys>(make: &Mk<E>) {
    let mut t = make(None);
    let s = with_op(&mut t, OpId(1), |t| fixtures::sphere(t, 1.0)).unwrap();
    // Setting the flag a pole edge already has is a no-op edit that must pass.
    let r = toggle(&mut t, 2, s.edges[0], CellFlags::DEGENERATE);
    assert!(r.is_ok(), "{r:?}");
    assert_eq!(t.revision(), OpId(2));
}

pub(super) fn scenarios<E: Sys>() -> Vec<Scenario<E>> {
    vec![
        ("clearing_a_pole_flag_in_its_own_op_is_rejected_and_rolled_back", clearing_a_pole_flag_in_its_own_op_is_rejected_and_rolled_back::<E>),
        ("setting_degenerate_on_an_open_edge_in_its_own_op_is_rejected", setting_degenerate_on_an_open_edge_in_its_own_op_is_rejected::<E>),
        ("a_valid_flag_edit_still_commits", a_valid_flag_edit_still_commits::<E>),
    ]
}
