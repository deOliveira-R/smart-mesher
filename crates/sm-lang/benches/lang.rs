//! Benchmarks of the hot paths: parsing, lowering a large script (plan mode, so no op cost), set algebra on
//! million-id sets and resolving `all <kind>` over a million allocated entities.

// `criterion_group!` generates an undocumented public function.
#![allow(missing_docs)]

use criterion::{BatchSize, Criterion, Throughput, criterion_group, criterion_main};
use sm_core::{EntityId, EntityKind};
use sm_lang::entity::{PAR_CHUNK, all_of};
use sm_lang::{EntitySet, Interpreter, Options, PlanHost, parse};
use sm_op_api::{OpRecord, QuietControl};

#[path = "../tests/common/mod.rs"]
mod common;

/// A script of `n` statements mixing lets, op calls with entity lists, loops-free arithmetic and a batch every 50.
fn script(n: usize) -> String {
    let mut s = String::from("let base = model.create_brick(x: 1, y: 1, z: 1)\n");
    for i in 0..n {
        match i % 5 {
            0 => s.push_str(&format!("let w{i} = {i} * 0.5 + sqrt({i} + 1)\n")),
            1 => s.push_str(&format!("model.create_brick(x: w{}, y: 2, z: {})\n", i - 1, i + 1)),
            2 => s.push_str(&format!("intent.set_size(ids: [Face#{}, Face#{}], size: 0.25 * {})\n", i % 6 + 1, i % 5 + 1, i + 1)),
            3 => s.push_str("mesh.run(targets: Volume#1, scheme: \"pave\", merge: true, order: 2)\n"),
            _ => s.push_str(&format!("model.note(text: \"note {i}\")\n")),
        }
    }
    s
}

fn bench_parse(c: &mut Criterion) {
    let text = script(2000);
    let mut g = c.benchmark_group("parse");
    g.throughput(Throughput::Bytes(text.len() as u64));
    g.bench_function("2000_statements", |b| b.iter(|| parse(0, std::hint::black_box(&text))));
    g.finish();
}

fn bench_lower(c: &mut Criterion) {
    let engine = common::engine();
    let state = engine.registry().new_state();
    let text = script(2000);
    c.bench_function("lower_2000_statements_plan_mode", |b| {
        b.iter_batched(
            || state.clone(),
            |st| {
                let mut h = PlanHost::new(engine.registry(), st);
                let rep = Interpreter::new(&mut h, Options::default()).run_source("bench", &text);
                assert!(rep.ok());
                rep.events.len()
            },
            BatchSize::SmallInput,
        )
    });
}

fn big_set(offset: u64, step: u64, n: u64) -> EntitySet {
    EntitySet::from_sorted_unique((0..n).map(|i| EntityId::new(EntityKind::TopoFace, offset + i * step).expect("serial fits")).collect())
}

fn bench_sets(c: &mut Criterion) {
    let a = big_set(1, 2, 1_000_000);
    let b = big_set(1, 3, 1_000_000);
    let mut g = c.benchmark_group("set_algebra_1M");
    g.sample_size(20);
    for (name, chunk) in [("sequential", usize::MAX), ("chunked_parallel", PAR_CHUNK)] {
        g.bench_function(format!("union/{name}"), |bch| bch.iter(|| a.union_chunked(&b, chunk).len()));
        g.bench_function(format!("intersection/{name}"), |bch| bch.iter(|| a.intersection_chunked(&b, chunk).len()));
        g.bench_function(format!("difference/{name}"), |bch| bch.iter(|| a.difference_chunked(&b, chunk).len()));
    }
    g.finish();
}

fn bench_resolve(c: &mut Criterion) {
    let engine = common::engine();
    let mut state = engine.registry().new_state();
    let q = QuietControl::new();
    let rec = OpRecord::new("test.alloc_many", 1, serde_json::json!({"kind": "TopoFace", "n": 1_000_000})).expect("record");
    engine.apply_record(&mut state, &rec, &q.control()).expect("alloc");
    c.bench_function("all_faces_of_1M_allocated", |b| b.iter(|| all_of(&state, EntityKind::TopoFace).len()));
}

criterion_group!(benches, bench_parse, bench_lower, bench_sets, bench_resolve);
criterion_main!(benches);
