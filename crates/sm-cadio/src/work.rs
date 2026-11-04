//! Work counters: deterministic measures of how much the reader did, so tests can assert
//! complexity (linear growth, early refusal) without wall-clock time.
//!
//! The counters are process-wide atomics. Take a [`work`] snapshot before and after a read and
//! subtract with [`Work::since`]; tests that do so must not run concurrently with other reads.

use std::sync::atomic::{AtomicU64, Ordering::Relaxed};

pub(crate) static SOLIDS_DECODED: AtomicU64 = AtomicU64::new(0);
pub(crate) static CLOSURE_COEDGES: AtomicU64 = AtomicU64::new(0);
pub(crate) static GLOBAL_PROJECTIONS: AtomicU64 = AtomicU64::new(0);
pub(crate) static NEWTON_RUNS: AtomicU64 = AtomicU64::new(0);
pub(crate) static WALK_STEPS: AtomicU64 = AtomicU64::new(0);

/// A snapshot of the work counters (monotonic since process start).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Work {
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

impl Work {
    /// The work done since `earlier`.
    pub fn since(self, earlier: Work) -> Work {
        Work {
            solids_decoded: self.solids_decoded.saturating_sub(earlier.solids_decoded),
            closure_coedges: self.closure_coedges.saturating_sub(earlier.closure_coedges),
            global_projections: self.global_projections.saturating_sub(earlier.global_projections),
            newton_runs: self.newton_runs.saturating_sub(earlier.newton_runs),
            walk_steps: self.walk_steps.saturating_sub(earlier.walk_steps),
        }
    }
}

/// The current counter values.
pub fn work() -> Work {
    Work { solids_decoded: SOLIDS_DECODED.load(Relaxed), closure_coedges: CLOSURE_COEDGES.load(Relaxed), global_projections: GLOBAL_PROJECTIONS.load(Relaxed), newton_runs: NEWTON_RUNS.load(Relaxed), walk_steps: WALK_STEPS.load(Relaxed) }
}

pub(crate) fn add(c: &AtomicU64, n: usize) {
    c.fetch_add(n as u64, Relaxed);
}
