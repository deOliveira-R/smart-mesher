//! Randomized differential drivers (`sm_testkit::topo_differential`): random operator sequences
//! (valid and invalid) run in lock step on the reference `CellComplex` and on `TopoStore`. Every
//! result, error, rollback and the full observable state must agree after every operation; the
//! store-specific extras (`validate_par`, counters) are checked here.

use proptest::prelude::*;
use sm_testkit::topo_differential::{Stats, run_biased_differential, run_random_differential_with};
use sm_topo::TopoStore;
use sm_topo_api::{CellComplex, validate};

/// The store with paranoid cross-checks and small thresholds, so the positional and incremental
/// paths run on small models.
fn paranoid_store() -> TopoStore {
    let mut s = TopoStore::new();
    s.set_paranoid(true);
    s.set_row_thresholds(6, 6);
    s
}

fn production_store() -> TopoStore {
    TopoStore::new()
}

/// Store-specific check run with every full-state comparison.
fn par_validate_agrees(_: &CellComplex, b: &TopoStore) {
    assert_eq!(b.validate_par(), validate(b), "validate_par vs validate");
}

fn report(stats: &Stats) {
    for (k, (ok, err)) in stats {
        println!("{k:>20}: ok {ok:5}  err {err:5}");
    }
}

#[test]
fn random_sequences_agree_with_the_reference() {
    let mut stats = Stats::new();
    let (mut sorted, mut full) = (0, 0);
    for seed in 1..=8u64 {
        let r = run_random_differential_with(seed, 60, &CellComplex::new, &paranoid_store, &par_validate_agrees);
        for (k, (ok, err)) in r.stats {
            let e = stats.entry(k).or_default();
            e.0 += ok;
            e.1 += err;
        }
        let c = r.b.counters();
        sorted += c.sorted_parent_rewrites;
        full += c.full_parent_rewrites;
    }
    report(&stats);
    println!("sorted rewrites {sorted}, full rewrites {full}");
    assert!(sorted > 0, "the positional parent rewrite never ran");
    // The driver must really exercise the operators (not only their error paths).
    for k in ["brick", "split_edge", "split_face", "merge_vertices", "unmerge_face", "kill_cascade", "reverse", "remove_face_use", "make_inner_loop", "imprint", "sheet", "tetra"] {
        let (ok, _) = stats.get(k).copied().unwrap_or_default();
        assert!(ok > 0, "operator {k} never succeeded");
    }
    assert!(stats["<end_op>"].0 > 0);
}

#[test]
fn long_random_sequences_agree_with_the_reference() {
    let mut stats = Stats::new();
    for seed in 100..102u64 {
        let r = run_random_differential_with(seed, 150, &CellComplex::new, &paranoid_store, &par_validate_agrees);
        for (k, (ok, err)) in r.stats {
            let e = stats.entry(k).or_default();
            e.0 += ok;
            e.1 += err;
        }
    }
    report(&stats);
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(40))]

    /// Shrinking differential test over biased op sequences.
    #[test]
    fn biased_sequences_agree_with_the_reference(ops in proptest::collection::vec((any::<u64>(), 0u8..12, any::<u8>().prop_map(|b| b % 8 == 0)), 1..24)) {
        run_biased_differential(&ops, &CellComplex::new, &paranoid_store, &par_validate_agrees);
    }

    /// The same driver with production settings on the store (paranoid off, default thresholds).
    #[test]
    fn biased_sequences_agree_with_the_reference_in_production_settings(ops in proptest::collection::vec((any::<u64>(), 0u8..12, any::<u8>().prop_map(|b| b % 8 == 0)), 1..24)) {
        run_biased_differential(&ops, &CellComplex::new, &production_store, &par_validate_agrees);
    }
}
