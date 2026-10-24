//! The shared topology conformance suite (`sm_testkit::topo_conformance`), run against the
//! production store and against the paranoid twin that drives the reference and the store in lock
//! step and compares the complete observable state after every operation.
//!
//! Timing assertions are not in the suite; see `perf.rs`.

mod common;

use common::Dual;
use sm_geom_api::ModelTolerance;
use sm_testkit::topo_conformance::run_topology_conformance;
use sm_topo::TopoStore;

/// The store with the paranoid cross-checks on and small row thresholds, so the positional and
/// incremental paths run on small models.
fn make_store(tol: Option<ModelTolerance>) -> TopoStore {
    let mut s = tol.map_or_else(TopoStore::new, TopoStore::with_tolerance);
    s.set_paranoid(true);
    s.set_row_thresholds(6, 6);
    s
}

/// The shipped configuration: default thresholds, paranoid off.
fn make_production(tol: Option<ModelTolerance>) -> TopoStore {
    tol.map_or_else(TopoStore::new, TopoStore::with_tolerance)
}

fn make_dual(tol: Option<ModelTolerance>) -> Dual {
    tol.map_or_else(Dual::new, Dual::with_tolerance)
}

#[test]
fn store_passes_the_conformance_suite() {
    run_topology_conformance("sm-topo", &make_store);
}

#[test]
fn store_with_production_settings_passes_the_conformance_suite() {
    run_topology_conformance("sm-topo (production settings)", &make_production);
}

#[test]
fn dual_passes_the_conformance_suite() {
    run_topology_conformance("dual", &make_dual);
}
