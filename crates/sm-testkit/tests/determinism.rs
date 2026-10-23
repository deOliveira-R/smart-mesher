//! Determinism harness self-test: passes on deterministic closures, fails on non-deterministic ones.

use sm_mesh_api::{CellIx, MeshRead, cell_volume};
use sm_testkit::check::{mesh_hash, topo_hash};
use sm_testkit::determinism::*;
use sm_testkit::{meshes, topo};
use std::sync::atomic::{AtomicUsize, Ordering};

/// Build every fixture (in parallel over fixtures) and hash the results.
fn pipeline(threads: usize) -> Vec<u64> {
    let mesh_hashes = par_map(threads, 10, |i| {
        let all = meshes::all_mesh_fixtures().unwrap();
        let f = &all[i];
        let vols: Vec<f64> = (0..f.mesh.cell_count()).map(|c| cell_volume(&f.mesh, CellIx::new(c))).collect();
        mesh_hash(&f.mesh) ^ hash_f64_bits(&vols)
    });
    let topo_hashes = par_map(threads, 6, |i| {
        let all = topo::all_topo_fixtures().unwrap();
        topo_hash(&all[i].complex)
    });
    let chamber = topo::mirror_chamber().unwrap();
    mesh_hashes.into_iter().chain(topo_hashes).chain([topo_hash(&chamber.complex), hash_f64_bits(&[chamber.volume])]).collect()
}

#[test]
fn the_fixture_pipeline_is_deterministic() {
    let h = check(2, 4, pipeline).unwrap();
    assert_eq!(h.len(), 18);
    // Different fixtures give different hashes.
    let mut d = h.clone();
    d.sort_unstable();
    d.dedup();
    assert!(d.len() >= 16);
}

#[test]
fn a_pure_closure_passes_and_returns_the_baseline() {
    assert_eq!(check(3, 8, |_| 42u64).unwrap(), 42);
    let sum = |threads: usize| par_map(threads, 1000, |i| (i as f64).sqrt()).iter().sum::<f64>();
    assert!(check(2, 7, sum).is_ok(), "sequential reduction over index-ordered results is thread-count independent");
}

#[test]
fn a_global_counter_is_detected() {
    static CALLS: AtomicUsize = AtomicUsize::new(0);
    let r = check(2, 2, |_| CALLS.fetch_add(1, Ordering::SeqCst));
    assert!(matches!(r, Err(DeterminismError::Mismatch { .. })), "{r:?}");
}

#[test]
fn a_thread_count_dependent_result_is_detected() {
    let r = check(1, 4, |threads| threads);
    match r {
        Err(DeterminismError::Mismatch { what, expected, actual }) => {
            assert!(what.contains("4 threads"), "{what}");
            assert_eq!((expected.as_str(), actual.as_str()), ("1", "4"));
        }
        other => panic!("{other:?}"),
    }
    // With one thread requested nothing differs.
    assert!(check(1, 1, |threads| threads).is_ok());
}

#[test]
fn a_panic_in_a_concurrent_call_is_reported() {
    // Only the spawned (unnamed) threads panic.
    let r = check(1, 2, |_| {
        assert!(std::thread::current().name().is_some(), "spawned thread");
        1u8
    });
    assert!(matches!(r, Err(DeterminismError::Panicked(_))), "{r:?}");
}

#[test]
fn bad_arguments_are_rejected() {
    assert_eq!(check(0, 2, |_| 1), Err(DeterminismError::BadArguments));
    assert_eq!(check(2, 0, |_| 1), Err(DeterminismError::BadArguments));
}

#[test]
fn par_map_keeps_index_order() {
    for threads in [0, 1, 2, 3, 8, 100] {
        for n in [0, 1, 2, 7, 64, 101] {
            let v = par_map(threads, n, |i| i * i);
            assert_eq!(v, (0..n).map(|i| i * i).collect::<Vec<_>>(), "threads={threads} n={n}");
        }
    }
}

#[test]
fn hashing_is_stable() {
    // FNV-1a of the little-endian bits of 1.0 and -0.0 versus 0.0.
    assert_eq!(hash_f64_bits(&[]), 0xcbf2_9ce4_8422_2325);
    assert_ne!(hash_f64_bits(&[0.0]), hash_f64_bits(&[-0.0]));
    assert_eq!(hash_f64_bits(&[1.0, 2.0]), hash_f64_bits(&[1.0, 2.0]));
    assert_ne!(hash_f64_bits(&[1.0, 2.0]), hash_f64_bits(&[2.0, 1.0]));
}
