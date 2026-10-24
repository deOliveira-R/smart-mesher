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

/// Many small solids, one operation each: a brick webcut in two (11 faces, 2 volumes). Rows stay
/// short, so this shows the part of the win that does not come from long volume rows.
fn assembly<E: TopologyEdit + Default>(n: usize) -> E {
    use sm_core::Point3;
    use sm_topo_api::OpId;
    use sm_topo_api::fixtures::{self, with_op};
    let mut t = E::default();
    for i in 0..n {
        let b = with_op(&mut t, OpId(2 * i as u64 + 1), |t| fixtures::brick(t, Point3::new(3.0 * i as f64, 0.0, 0.0), [1.0; 3])).expect("brick");
        with_op(&mut t, OpId(2 * i as u64 + 2), |t| fixtures::webcut_brick(t, &b, 0.4)).expect("webcut");
    }
    t
}

fn assemblies(c: &mut Criterion) {
    let mut g = c.benchmark_group("assembly_of_webcut_bricks");
    g.sample_size(10);
    for n in [250usize, 1_000, 4_000] {
        g.throughput(Throughput::Elements(n as u64));
        g.bench_with_input(BenchmarkId::new("sm-topo", n), &n, |b, &n| b.iter(|| black_box(assembly::<TopoStore>(n))));
        g.bench_with_input(BenchmarkId::new("reference", n), &n, |b, &n| b.iter(|| black_box(assembly::<CellComplex>(n))));
    }
    g.finish();
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

/// Per-record cost of the lineage log: `n` records (a split every other record, over faces,
/// edges and vertices with dense serials like the store allocates), then a lookup by subject and
/// by target for each. Throughput is in records, so the report reads as time per record.
fn lineage_log(c: &mut Criterion) {
    use sm_core::{EntityId, EntityKind};
    use sm_topo_api::{LineageKind, LineageLog, LineageRecord, OpId};
    let kinds = [EntityKind::TopoFace, EntityKind::TopoEdge, EntityKind::TopoVertex];
    let id = |k: EntityKind, n: u64| EntityId::new(k, n).expect("serial");
    let fill = |n: usize| {
        let mut log = LineageLog::new();
        for i in 0..n {
            let k = kinds[i % 3];
            let s = (i / 3) as u64 + 1;
            let kind = if i % 2 == 0 { LineageKind::Split { children: [id(k, 2 * s + 1), id(k, 2 * s + 2)].into_iter().collect() } } else { LineageKind::Deleted };
            log.push(LineageRecord { op: OpId(i as u64 / 8 + 1), entity: id(k, s), kind });
        }
        log
    };
    let mut g = c.benchmark_group("lineage_log");
    g.sample_size(20);
    for n in [10_000usize, 100_000, 400_000] {
        g.throughput(Throughput::Elements(n as u64));
        g.bench_with_input(BenchmarkId::new("push", n), &n, |b, &n| b.iter(|| black_box(fill(n))));
        let log = fill(n);
        g.bench_with_input(BenchmarkId::new("lookup", n), &n, |b, &n| {
            b.iter(|| {
                let mut hits = 0usize;
                for i in 0..n {
                    let k = kinds[i % 3];
                    let s = (i / 3) as u64 + 1;
                    hits += log.of(id(k, s)).count() + log.produced(id(k, s + 1)).count();
                }
                black_box(hits)
            })
        });
    }
    g.finish();
}

criterion_group!(benches, refine, assemblies, queries, lineage_log);
criterion_main!(benches);
