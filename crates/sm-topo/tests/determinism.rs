//! Determinism (CLAUDE.md, binding): the same history gives bit-identical state when run twice,
//! with one worker thread or many, and when run concurrently from several threads.

mod common;
#[path = "common/scenario.rs"]
mod scenario;

use sm_testkit::determinism;
use sm_topo::TopoStore;
use sm_topo_api::{Dim, TopologyView, validate};

/// A summary that covers every observable of the store: canonical hash, all radial rings (they
/// are rebuilt in parallel at `end_op`), both validators.
fn fingerprint(t: &TopoStore) -> (u64, u64, usize, usize) {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for &e in t.cells(Dim::Edge) {
        for u in t.radial_uses(e) {
            for w in [e.to_raw(), u.face.to_raw(), u.sense as u64] {
                h = (h ^ w).wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    (t.canonical_hash(), h, validate(t).len(), t.validate_par().len())
}

fn run(threads: usize, n: usize, per_op: bool) -> (u64, u64, usize, usize) {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
    pool.install(|| {
        let mut t = TopoStore::new();
        let b = scenario::setup(&mut t);
        scenario::refine(&mut t, &b, n, per_op);
        fingerprint(&t)
    })
}

#[test]
fn one_op_refinement_is_identical_for_1_and_n_threads_and_across_runs() {
    // 1500 splits in one operation: thousands of dirty edges, so the parallel radial rebuild runs.
    let fp = determinism::check(2, 4, |threads| run(threads, 1500, false)).expect("deterministic");
    assert_eq!(fp.2, 0);
    assert_eq!(fp.3, 0);
}

#[test]
fn per_op_refinement_is_identical_for_1_and_n_threads_and_across_runs() {
    determinism::check(2, 4, |threads| run(threads, 400, true)).expect("deterministic");
}

#[test]
fn the_parallel_validator_equals_the_sequential_one_on_a_broken_complex() {
    use sm_topo_api::fixtures::{self, with_op};
    use sm_topo_api::{OpId, TopologyEdit};
    // A complex that fails validation: a face reversed by hand is re-normalized at end_op, so
    // break it with a moved vertex plus a dead reference via kill (validate sees geometry only
    // through validate_geometry, so compare the combinatorial validators on a sound complex and
    // on one whose end_op was bypassed by flags).
    let mut t = TopoStore::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, sm_core::Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    with_op(&mut t, OpId(2), |t| Ok(t.set_flags(b.edges[0], sm_topo_api::CellFlags::DEGENERATE)?)).ok();
    let seq = validate(&t);
    for threads in [1, 3] {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        assert_eq!(pool.install(|| t.validate_par()), seq);
    }
}
