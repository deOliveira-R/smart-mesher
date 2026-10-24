//! Determinism: repeated runs and 1 thread vs N threads give bit-identical results.
#![allow(missing_docs)]

mod common;
use common::*;
use sm_mesh_api::MeshRead;
use sm_quality::*;
use sm_testkit::determinism::{check, hash_f64_bits};
use sm_testkit::meshes::all_mesh_fixtures;

fn with_threads<R: Send>(t: usize, f: impl FnOnce() -> R + Send) -> R {
    rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap().install(f)
}

fn fingerprint(m: &dyn MeshRead) -> Vec<u64> {
    let metrics: Vec<Metric> = Metric::ALL.iter().copied().filter(|x| (0..m.cell_count()).any(|c| x.applies(m.cell_shape(sm_mesh_api::CellIx::new(c))))).collect();
    let opts = Options::default();
    let t = compute_many(m, &metrics, &opts).unwrap();
    let mut out = Vec::new();
    for (metric, col) in t.metrics.iter().zip(&t.columns) {
        out.push(hash_f64_bits(col));
        let s = summarize(col);
        out.push(hash_f64_bits(&[s.count as f64, s.undefined as f64, s.min, s.max, s.mean, s.std_dev, s.argmin.map_or(-1.0, |i| i as f64), s.argmax.map_or(-1.0, |i| i as f64)]));
        if s.count > 0 {
            let h = histogram_auto(col, 16).unwrap();
            out.push(h.counts.iter().fold(h.below ^ h.above, |a, &c| a.rotate_left(7) ^ c));
        }
        let _ = metric;
    }
    let v = validity(m, &opts).unwrap();
    out.push((v.n_valid * 1_000_003 + v.n_inverted * 1009 + v.n_degenerate) as u64);
    out
}

#[test]
fn perturbed_block_is_bit_identical_across_runs_and_thread_counts() {
    let m = perturbed_block(20, 0.3); // 8000 cells, two reduction chunks
    check(2, 4, |t| with_threads(t, || fingerprint(&m))).unwrap();
}

#[test]
fn every_fixture_is_bit_identical_across_thread_counts() {
    for fx in all_mesh_fixtures().unwrap() {
        check(2, 3, |t| with_threads(t, || fingerprint(&fx.mesh))).unwrap_or_else(|e| panic!("{}: {e}", fx.name));
    }
}

#[test]
fn ambient_pool_matches_single_thread() {
    // RAYON_NUM_THREADS=1 vs the default pool is exercised by running the suite twice (see CI);
    // here the ambient pool is compared with an explicit one-thread pool.
    let m = perturbed_block(8, 0.25);
    assert_eq!(fingerprint(&m), with_threads(1, || fingerprint(&m)));
}
