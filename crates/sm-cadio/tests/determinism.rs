//! Acceptance 13: bit-identical results for two runs, `RAYON_NUM_THREADS=1` against the default
//! and concurrent runs (`sm_testkit::determinism::check`), plus cold replay of the registered op.

mod common;

use std::sync::Arc;

use common::*;
use sm_cadio::{ImportStep, MemoryInputStore, ReadOptions, Source, read_step};
use sm_testkit::determinism::{Fnv, check};

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool").install(f)
}

fn debug_hash(m: &sm_cadio::BrepModel) -> u64 {
    use std::hash::Hasher;
    // `Debug` prints floats with their shortest round-trip form, so equal strings mean equal bits.
    let mut h = Fnv::default();
    h.write(format!("{m:?}").as_bytes());
    h.finish()
}

fn fixtures() -> Vec<(&'static str, Vec<u8>)> {
    let mut v: Vec<(&'static str, Vec<u8>)> = sm_testkit::step::ALL.iter().map(|f| (f.name, f.bytes.to_vec())).collect();
    for n in ["brick_2x3x4.stp", "brick_inch.stp", "two_bricks.stp", "brick_with_void.stp", "frustum_r2_r1_h3.stp", "sphere_seam_r3.stp", "block_fillet_nurbs.stp"] {
        v.push((n, data(n)));
    }
    v.push(("many_bricks", common::synth::bricks_step(40)));
    v
}

#[test]
fn read_step_is_bit_identical_across_runs_and_thread_counts() {
    for (name, bytes) in fixtures() {
        let got = check(2, 4, |threads| in_pool(threads, || debug_hash(&read_step(&bytes, &ReadOptions::default()).expect("read")))).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_ne!(got, 0);
    }
}

#[test]
fn the_import_state_hash_is_bit_identical_across_runs_and_thread_counts() {
    for (name, bytes) in fixtures() {
        let st = Arc::new(MemoryInputStore::new());
        let r = st.put(&bytes);
        // The production store on the small models only (the NURBS loft is slow in debug builds).
        let heavy = matches!(name, "loft_bspline" | "w3a_part" | "many_bricks");
        // Realizing these costs minutes in a debug build; `cargo test --release` (and the
        // `cold_replay` test, which imports the w3a part) still cover them.
        if heavy && cfg!(debug_assertions) {
            continue;
        }
        let factories: &[Factory] = if heavy { &[cell_complex] } else { &[cell_complex, topo_store] };
        for factory in factories.iter().copied() {
            let h = check(if heavy { 1 } else { 2 }, 3, |threads| {
                in_pool(threads, || {
                    let mut s = session(factory, Arc::clone(&st));
                    // The validators are read-only and deterministic; `check` only costs time here.
                    let op = ImportStep { source: Source::sha256(r.sha256.clone()), name: None, to_unit: None, assume_unit: None, weld_tol: None, solids: None, sheets: None, check: Some(false) };
                    run(&mut s, &op).expect("import");
                    (s.state().hash(), s.state().layer_hash("model"))
                })
            })
            .unwrap_or_else(|e| panic!("{name}: {e}"));
            let _ = h;
        }
    }
}

#[test]
fn cold_replay_from_the_root_does_not_diverge() {
    let st = Arc::new(MemoryInputStore::new());
    let r = st.put(sm_testkit::w3a::STEP);
    for factory in [cell_complex as Factory, topo_store] {
        let mut s = sm_op_api::Session::new(engine(factory, Arc::clone(&st), sm_op_api::DerivedCache::disabled()));
        let op = ImportStep { source: Source::sha256(r.sha256.clone()), name: None, to_unit: None, assume_unit: None, weld_tol: None, solids: None, sheets: None, check: None };
        let (h, _) = run(&mut s, &op).expect("import");
        let q = sm_op_api::QuietControl::new();
        let cold = s.state_at_from_root(h, &q.control()).expect("no Diverged");
        assert_eq!(cold.hash(), s.state().hash());
    }
}

#[test]
fn the_parallel_solid_path_matches_the_sequential_one() {
    // 40 solids take the rayon path; selecting 3 of them takes the sequential one: same solids.
    let bytes = common::synth::bricks_step(40);
    let all = read_step(&bytes, &ReadOptions::default()).expect("all");
    let some = read_step(&bytes, &ReadOptions { solids: Some(vec![3, 17, 39]), ..ReadOptions::default() }).expect("some");
    assert_eq!(some.solids.len(), 3);
    for (i, k) in [3usize, 17, 39].into_iter().enumerate() {
        assert_eq!(some.solids[i], all.solids[k]);
    }
}
