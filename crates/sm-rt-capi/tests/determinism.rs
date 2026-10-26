//! Determinism of the ABI (CLAUDE.md): the same calls give bit-identical results when repeated
//! and when run on 1 thread or many (every ray has its own context; nothing is shared but the
//! read-only tracker). Uses the testkit harness.
#![cfg(feature = "stubs")]

mod common;
use common::*;
use sm_rt_capi::ffi::test_hooks::*;
use sm_rt_capi::ffi::*;
use sm_rt_capi::*;

/// Trace `n` seeded rays through the lattice world with `step` and fold every output bit into a
/// vector of f64 (distances) plus ids; per-ray work is independent and joined in index order.
fn walk(threads: usize) -> Vec<f64> {
    let t = stub(STUB_LATTICE, 1.0);
    let tp = t as usize;
    let n = 2000;
    let per = sm_testkit::determinism::par_map(threads, n, |i| {
        let mut rng = Rng::named("det-walk");
        rng.0 = rng.0.wrapping_add(i as u64 * 0x9E37_79B9);
        let c = ctx_of(tp as *const SmrtTracker);
        let (p, d) = ([rng.unit() * 12.0, rng.unit() * 12.0, rng.unit() * 12.0], rng.dir());
        let mut acc = vec![];
        if init(c, p, d) == 0 {
            loop {
                let (_, s) = step(c, f64::INFINITY);
                acc.extend([s.dist, s.cell as f64, s.region as f64, f64::from(s.end), s.to_cell as f64, s.surface as f64]);
                if s.end == SMRT_END_BOUNDARY || s.end == SMRT_END_LOST || acc.len() > 600 {
                    break;
                }
            }
        }
        // SAFETY: freed once.
        unsafe { smrt_ctx_free(c) };
        acc
    });
    // SAFETY: freed once, after all contexts.
    unsafe { smrt_tracker_free(t) };
    per.into_iter().flatten().collect()
}

#[test]
fn step_streams_are_bit_identical_across_runs_and_thread_counts() {
    let v = sm_testkit::determinism::check_f64_bits(2, 4, walk).unwrap();
    assert!(v.len() > 5000, "{} values", v.len());
}
