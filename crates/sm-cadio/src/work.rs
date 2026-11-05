//! Work counters: deterministic measures of how much one read did, so tests can assert
//! complexity (linear growth, early refusal) without wall-clock time.
//!
//! Counting is per call: [`read_step_metered`](crate::read_step_metered) owns a private meter and
//! returns its totals as a [`Work`]. There is no process-wide state, so concurrent reads do not
//! mix and tests need no serialization. Totals are sums of increments, so they do not depend on
//! the thread count.

use std::sync::atomic::{AtomicU64, Ordering::Relaxed};

/// What one read did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Work {
    /// Entities in the parsed file (zero when the parse failed).
    pub entities_parsed: u64,
    /// Solid bodies decoded from the entity graph (shared bodies count once).
    pub solids_decoded: u64,
    /// Coedges visited by shell-closure checks (the split pass must keep this linear).
    pub closure_coedges: u64,
    /// Global (grid search) surface projections made by the closed-face split.
    pub global_projections: u64,
    /// Gauss-Newton runs made by the closed-face split.
    pub newton_runs: u64,
    /// Representations entered by the assembly instance walk.
    pub walk_steps: u64,
}

/// The per-call counters (atomics only because decode runs under rayon).
#[derive(Debug, Default)]
pub(crate) struct Meter {
    entities_parsed: AtomicU64,
    solids_decoded: AtomicU64,
    closure_coedges: AtomicU64,
    global_projections: AtomicU64,
    newton_runs: AtomicU64,
    walk_steps: AtomicU64,
}

impl Meter {
    pub(crate) fn parsed(&self, n: usize) {
        self.entities_parsed.fetch_add(n as u64, Relaxed);
    }
    pub(crate) fn solid_decoded(&self) {
        self.solids_decoded.fetch_add(1, Relaxed);
    }
    pub(crate) fn closure_coedges(&self, n: usize) {
        self.closure_coedges.fetch_add(n as u64, Relaxed);
    }
    pub(crate) fn global_projection(&self) {
        self.global_projections.fetch_add(1, Relaxed);
    }
    pub(crate) fn newton_run(&self) {
        self.newton_runs.fetch_add(1, Relaxed);
    }
    pub(crate) fn walk_step(&self) {
        self.walk_steps.fetch_add(1, Relaxed);
    }
    pub(crate) fn snapshot(&self) -> Work {
        Work {
            entities_parsed: self.entities_parsed.load(Relaxed),
            solids_decoded: self.solids_decoded.load(Relaxed),
            closure_coedges: self.closure_coedges.load(Relaxed),
            global_projections: self.global_projections.load(Relaxed),
            newton_runs: self.newton_runs.load(Relaxed),
            walk_steps: self.walk_steps.load(Relaxed),
        }
    }
}
