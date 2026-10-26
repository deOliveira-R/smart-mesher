//! Shared helpers of the integration tests.
#![allow(dead_code)]

pub mod rig;

use sm_core::{EntityId, Point3};
use sm_topo_api::fixtures::{self, FixtureError};
use sm_topo_api::{CellComplex, Dim, OpId, TopologyEdit, TopologyView, validate, validate_geometry};

/// Run `f` as operation `op` (begin, f, end; abort on error).
pub fn in_op<E: TopologyEdit + ?Sized, T>(t: &mut E, op: u64, f: impl FnOnce(&mut E) -> Result<T, sm_boolean::BooleanError>) -> Result<T, sm_boolean::BooleanError> {
    t.begin_op(OpId(op))?;
    match f(t) {
        Ok(v) => {
            t.end_op()?;
            Ok(v)
        }
        Err(e) => {
            t.abort_op();
            Err(e)
        }
    }
}

/// `[V, E, F, Vol, Body]` of live cells, degenerate (pole) edges excluded from E.
pub fn counts<V: TopologyView + ?Sized>(t: &V) -> [usize; 5] {
    let edges = t.cells(Dim::Edge).iter().filter(|&&e| !t.flags(e).contains(sm_topo_api::CellFlags::DEGENERATE)).count();
    [t.cells(Dim::Vertex).len(), edges, t.cells(Dim::Face).len(), t.cells(Dim::Volume).len(), t.cells(Dim::Body).len()]
}

/// Euler characteristic `V - E + F`.
pub fn euler<V: TopologyView + ?Sized>(t: &V) -> i64 {
    let c = counts(t);
    c[0] as i64 - c[1] as i64 + c[2] as i64
}

/// Assert `validate` and `validate_geometry` report nothing.
pub fn assert_valid(t: &dyn TopologyView) {
    let issues = validate(t);
    assert!(issues.is_empty(), "validate: {issues:#?}");
    let g = validate_geometry(t);
    assert!(g.is_empty(), "validate_geometry: {g:#?}");
}

pub fn total_area<V: TopologyView + ?Sized>(t: &V) -> f64 {
    t.cells(Dim::Face).iter().map(|&f| t.face_moments(f).expect("face moments").area).sum()
}

pub fn volume_of<V: TopologyView + ?Sized>(t: &V, v: EntityId) -> f64 {
    t.volume_moments(v).expect("volume moments").volume
}

pub fn brick<E: TopologyEdit + ?Sized>(t: &mut E, op: u64, min: [f64; 3], size: [f64; 3]) -> fixtures::BrickIds {
    t.begin_op(OpId(op)).expect("begin");
    let b = fixtures::brick(t, Point3::new(min[0], min[1], min[2]), size).expect("brick");
    t.end_op().expect("end");
    b
}

pub fn fixture_err(e: FixtureError) -> sm_boolean::BooleanError {
    sm_boolean::BooleanError::Invalid(e.to_string())
}

pub fn new_cc() -> CellComplex {
    CellComplex::new()
}
