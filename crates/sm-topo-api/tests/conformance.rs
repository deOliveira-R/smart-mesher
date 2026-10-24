//! The reference `CellComplex` against the shared topology conformance suite (the scenarios live in
//! `sm-testkit` so every implementation runs the same ones).

use sm_topo_api::CellComplex;

#[test]
fn reference_passes_the_conformance_suite() {
    sm_testkit::topo_conformance::run_topology_conformance("reference", &|t| t.map_or_else(CellComplex::new, CellComplex::with_tolerance));
}

#[test]
fn the_suite_is_not_empty() {
    assert!(sm_testkit::topo_conformance::scenario_count() > 50);
}
