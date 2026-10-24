//! A rayon-backed [`Executor`] for the driver's generation waves.

use rayon::prelude::*;
use sm_scheme_api::Executor;

/// Runs the jobs of a generation wave on the global rayon pool.
///
/// The driver gives every job its own result slot and merges the slots in job order, so the
/// mesh is bit-identical for any pool size (`RAYON_NUM_THREADS=1` or the default). One job is
/// one rayon task (`with_max_len(1)`): waves hold few, heavy jobs (a volume or a face each).
#[derive(Clone, Copy, Debug, Default)]
pub struct RayonExec;

impl Executor for RayonExec {
    fn run(&self, n: usize, job: &(dyn Fn(usize) + Sync)) {
        (0..n).into_par_iter().with_max_len(1).for_each(job);
    }
}
