//! `mesh.quality` with the generic profile on a 100^3 hex block (record only, see DESIGN.md).
#![allow(missing_docs)]

use criterion::{Criterion, criterion_group, criterion_main};
use sm_diag::{QualityOptions, ValidateOptions, quality_report, validate_mesh};
use sm_mesh_api::SerialAlloc;

fn bench(c: &mut Criterion) {
    let n: u32 = std::env::var("SM_DIAG_BENCH_N").ok().and_then(|s| s.parse().ok()).unwrap_or(100);
    let m = sm_mesh::generate::hex_block_bulk([n; 3], [1.0; 3], &mut SerialAlloc::new()).expect("block").mesh;
    let mut g = c.benchmark_group("diag");
    g.sample_size(10);
    g.bench_function(format!("quality_generic_{n}^3"), |b| b.iter(|| quality_report(&m, &QualityOptions::default(), &|_| true).expect("report")));
    g.bench_function(format!("validate_{n}^3"), |b| b.iter(|| validate_mesh(&m, &ValidateOptions::default(), None)));
    g.finish();
}

criterion_group!(benches, bench);
criterion_main!(benches);
