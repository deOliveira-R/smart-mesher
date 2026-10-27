//! Criterion benches: `delaunay` and the full pipeline.
//!
//! The 200k-point and 1M-tet records of acceptance 12 are the `#[ignore]` tests in
//! `tests/perf.rs` (too long for criterion's repetition); these benches track the same code at a
//! size that repeats.

#![allow(missing_docs)]

use criterion::{Criterion, criterion_group, criterion_main};
use sm_scheme_tet::{SizeSpec, SurfaceInput, TetOptions, delaunay, tetrahedralize};
use sm_testkit::surf3::{self, SplitMix64};

fn bench_delaunay(c: &mut Criterion) {
    let mut r = SplitMix64(1);
    let pts: Vec<[f64; 3]> = (0..20_000).map(|_| [r.unit(), r.unit(), r.unit()]).collect();
    let mut g = c.benchmark_group("delaunay");
    g.sample_size(10);
    g.bench_function("20k_random_points", |b| b.iter(|| delaunay(&pts).map(|m| m.tets.len())));
    g.finish();
}

fn bench_pipeline(c: &mut Criterion) {
    let f = surf3::icosphere(1.0, 4);
    let inp = SurfaceInput { points: f.points.clone(), tris: f.oriented_tris(0) };
    let mut g = c.benchmark_group("tetrahedralize");
    g.sample_size(10);
    for opt in [0u8, 2] {
        let opts = TetOptions { size: SizeSpec::Constant(0.12), optimize: opt, ..TetOptions::default() };
        g.bench_function(format!("icosphere4_h0.12_optimize{opt}"), |b| b.iter(|| tetrahedralize(&inp, &opts).map(|m| m.tets.len())));
    }
    let cube = surf3::tri_cube(10);
    let inp = SurfaceInput { points: cube.points.clone(), tris: cube.oriented_tris(0) };
    g.bench_function("cube10_default", |b| b.iter(|| tetrahedralize(&inp, &TetOptions::default()).map(|m| m.tets.len())));
    g.finish();
}

criterion_group!(benches, bench_delaunay, bench_pipeline);
criterion_main!(benches);
