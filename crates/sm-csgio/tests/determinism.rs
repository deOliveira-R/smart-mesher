//! Determinism: two runs, and 1 thread vs N threads, give bit-identical hashes, findings and
//! chord tables (`tools/ci.sh` also runs the whole suite under `RAYON_NUM_THREADS=1`).
mod common;
use common::*;
use sm_csgio::{ImportError, ImportOptions, ImportedCsg};

fn in_pool<R: Send>(threads: usize, f: impl FnOnce() -> R + Send) -> R {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

/// Everything an import of `ir` produces that must not depend on threads, as comparable values.
fn fingerprint(ir: &sm_csgio::CsgIr) -> (Result<u128, ImportError>, Vec<u64>, Vec<String>) {
    match ImportedCsg::import(ir, &ImportOptions::default()) {
        Ok(m) => {
            let t = tracker(&m);
            let mut table = Vec::new();
            for i in 0..40 {
                for j in 0..40 {
                    let (y, z) = (-1.9 + 3.8 * f64::from(i) / 39.0, -1.9 + 3.8 * f64::from(j) / 39.0);
                    for (_, l) in chords(&t, [-2.0, y, z], [1.0, 0.0, 0.0]) {
                        table.push(l.to_bits());
                    }
                }
            }
            let warnings = m.report().warnings.iter().map(|w| format!("{} {:?}", w.code, w.point.map(|p| p.map(f64::to_bits)))).collect();
            (Ok(m.content_hash().0), table, warnings)
        }
        Err(e) => (Err(e), Vec::new(), Vec::new()),
    }
}

#[test]
fn imports_are_identical_across_runs_and_thread_counts() {
    for ir in [deck_o(), deck_o_overlap(), deck_o_gap()] {
        let a = fingerprint(&ir);
        let b = fingerprint(&ir);
        let one = in_pool(1, || fingerprint(&ir));
        let many = in_pool(4, || fingerprint(&ir));
        assert_eq!(a, b);
        assert_eq!(a, one);
        assert_eq!(a, many);
    }
}

#[test]
fn the_content_hash_sees_the_model() {
    let h = |ir: &sm_csgio::CsgIr| ImportedCsg::instantiate(ir, &ImportOptions::default()).unwrap().content_hash();
    assert_eq!(h(&deck_o()), h(&deck_o()));
    assert_ne!(h(&deck_o()), h(&deck_o_overlap()));
    let mut moved = deck_o();
    moved.surfaces.insert(1, sm_csgio::ir::Surf::sphere([0.0; 3], 1.0000001).unwrap());
    assert_ne!(h(&deck_o()), h(&moved));
}
