//! Differential-testing support: build the same history on the reference `CellComplex` and on
//! `TopoStore`, and demand identical results and identical observable state.
#![allow(dead_code, unused_imports)]

mod dual;
pub use dual::Dual;
use sm_core::EntityId;
use sm_topo::TopoStore;
use sm_topo_api::{CellComplex, TopologyEdit, TopologyView, validate};
use std::fmt::Debug;

/// Both implementations, driven in lock step.
pub struct Twin {
    pub a: CellComplex,
    pub b: TopoStore,
}

impl Twin {
    pub fn new() -> Self {
        let mut b = TopoStore::new();
        b.set_paranoid(true);
        // Small thresholds so the positional and incremental paths run on small models.
        b.set_row_thresholds(6, 6);
        Self { a: CellComplex::new(), b }
    }

    /// Production settings on the store: `TopoStore::new()`, paranoid off, default thresholds.
    pub fn production() -> Self {
        Self { a: CellComplex::new(), b: TopoStore::new() }
    }

    pub fn with_tolerance(tol: sm_geom_api::ModelTolerance) -> Self {
        let mut b = TopoStore::with_tolerance(tol);
        b.set_paranoid(true);
        b.set_row_thresholds(6, 6);
        Self { a: CellComplex::with_tolerance(tol), b }
    }

    /// Run `f` on both and demand equal results (errors included).
    pub fn run<R: PartialEq + Debug>(&mut self, ctx: &str, f: impl Fn(&mut dyn TopologyEdit) -> R) -> R {
        let ra = f(&mut self.a);
        let rb = f(&mut self.b);
        assert_eq!(ra, rb, "{ctx}: results differ (reference vs sm-topo)");
        ra
    }

    /// Full state comparison.
    pub fn same(&self, ctx: &str) {
        same_state(&self.a, &self.b, ctx);
        assert_eq!(self.b.validate_par(), validate(&self.b), "{ctx}: validate_par vs validate");
    }

    /// `begin_op`, `f`, `end_op` on both (abort if `f` fails), comparing every result and the
    /// final state. Returns `f`'s result.
    pub fn op<R: PartialEq + Debug>(&mut self, ctx: &str, op: u64, f: impl Fn(&mut dyn TopologyEdit) -> Result<R, sm_topo_api::TopoError>) -> Result<R, sm_topo_api::TopoError> {
        self.run(ctx, |t| t.begin_op(sm_topo_api::OpId(op))).expect("begin_op");
        let r = self.run(ctx, &f);
        if r.is_ok() {
            let end = self.run(ctx, |t| t.end_op());
            if let Err(e) = end {
                self.same(&format!("{ctx} (after failed end_op: {e})"));
                return Err(e);
            }
        } else {
            self.run(ctx, |t| {
                t.abort_op();
            });
        }
        self.same(ctx);
        r
    }
}

/// Full state comparison (see `sm_testkit::topo_differential::same_state`); panics with `ctx`.
pub fn same_state(a: &dyn TopologyView, b: &dyn TopologyView, ctx: &str) {
    if let Err(e) = sm_testkit::topo_differential::same_state(a, b) {
        panic!("{ctx}: {e}");
    }
}

pub use sm_testkit::topo_differential::Rng;
