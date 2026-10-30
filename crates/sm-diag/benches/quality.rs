//! `mesh.quality` with the generic profile on hex blocks (record only, see DESIGN.md): a clean
//! 100^3 block, and a damaged block that exercises the cluster pass.
#![allow(missing_docs)]

use criterion::{Criterion, criterion_group, criterion_main};
use sm_diag::{QualityOptions, ValidateOptions, quality_report, validate_mesh};
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block;
use sm_testkit::broken::Tampered;

fn env_n(name: &str, default: u32) -> u32 {
    std::env::var(name).ok().and_then(|s| s.parse().ok()).unwrap_or(default)
}

fn bench(c: &mut Criterion) {
    let n = env_n("SM_DIAG_BENCH_N", 100);
    let m = sm_mesh::generate::hex_block_bulk([n; 3], [1.0; 3], &mut SerialAlloc::new()).expect("block").mesh;
    let mut g = c.benchmark_group("diag");
    g.sample_size(10);
    g.bench_function(format!("quality_generic_{n}^3"), |b| b.iter(|| quality_report(&m, &QualityOptions::default(), &|_| true).expect("report")));
    g.bench_function(format!("validate_{n}^3"), |b| b.iter(|| validate_mesh(&m, &ValidateOptions::default(), None)));

    // Damaged block: every 3rd vertex is displaced by a fixed pseudo-random offset (a hash of its
    // index, no RNG state), so many faces and cells are warn/fail and the cluster pass runs for
    // every graded metric.
    let nd = env_n("SM_DIAG_BENCH_DAMAGED_N", 50);
    let base = hex_block([nd; 3], [1.0; 3], &mut SerialAlloc::new()).expect("block");
    let mut t = Tampered::of(&base);
    let off = |v: usize, s: u64| ((((v as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ s).wrapping_mul(0xBF58_476D_1CE4_E5B9) >> 40) as f64 / (1u64 << 24) as f64 - 0.5) * 1.2;
    for v in (0..t.x.len()).step_by(3) {
        t.x[v] += off(v, 1);
        t.y[v] += off(v, 2);
        t.z[v] += off(v, 3);
    }
    let r = quality_report(&t, &QualityOptions::default(), &|_| true).expect("report");
    let bad: usize = r.metrics.iter().map(|m| m.n_warn + m.n_fail).sum();
    eprintln!("damaged {nd}^3: {} cells, {bad} bad entities over {} metrics, {} clusters", r.cells, r.metrics.len(), r.metrics.iter().map(|m| m.clusters.len()).sum::<usize>());
    g.bench_function(format!("quality_generic_damaged_{nd}^3"), |b| b.iter(|| quality_report(&t, &QualityOptions::default(), &|_| true).expect("report")));
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
