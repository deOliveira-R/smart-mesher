//! Scaling benchmark: a brick's top face refined by `n` splits (edge subdivision + chord, breadth
//! first), so one volume ends with `n + 6` faces. The reference `CellComplex` rewrites the whole
//! volume row (and re-sorts, re-links and re-checks it) on every split, so it is quadratic-ish
//! and is only run up to a size where it finishes in seconds; `TopoStore` runs to 10^5 faces.
//!
//! Run: `cargo bench -p sm-topo --bench scaling`.
#![allow(missing_docs)]

#[path = "../tests/common/scenario.rs"]
mod scenario;

use criterion::{BenchmarkId, Criterion, Throughput, criterion_group, criterion_main};
use sm_topo::TopoStore;
use sm_topo_api::{CellComplex, TopologyEdit, TopologyView, validate};
use std::hint::black_box;

fn run<E: TopologyEdit + Default>(n: usize, per_op: bool) -> E {
    let mut t = E::default();
    let b = scenario::setup(&mut t);
    black_box(scenario::refine(&mut t, &b, n, per_op));
    t
}

fn refine(c: &mut Criterion) {
    for (name, per_op) in [("refine_one_op", false), ("refine_op_per_split", true)] {
        let mut g = c.benchmark_group(name);
        g.sample_size(10);
        for n in [1_000usize, 4_000, 16_000, 100_000] {
            g.throughput(Throughput::Elements(n as u64));
            g.bench_with_input(BenchmarkId::new("sm-topo", n), &n, |b, &n| b.iter(|| run::<TopoStore>(n, per_op)));
            // The reference is quadratic here: stop where it still finishes in seconds.
            if n <= 4_000 {
                g.bench_with_input(BenchmarkId::new("reference", n), &n, |b, &n| b.iter(|| run::<CellComplex>(n, per_op)));
            }
        }
        g.finish();
    }
}

fn queries(c: &mut Criterion) {
    let t: TopoStore = run(16_000, false);
    let r: CellComplex = run(4_000, false);
    let mut g = c.benchmark_group("queries");
    g.sample_size(20);
    g.bench_function("sm-topo canonical_hash 16k faces", |b| b.iter(|| black_box(t.canonical_hash())));
    g.bench_function("reference canonical_hash 4k faces", |b| b.iter(|| black_box(r.canonical_hash())));
    g.bench_function("sm-topo validate_par 16k faces", |b| b.iter(|| black_box(t.validate_par().len())));
    g.bench_function("sm-topo validate (sequential) 16k faces", |b| b.iter(|| black_box(validate(&t).len())));
    g.finish();
}

criterion_group!(benches, refine, queries);
criterion_main!(benches);
