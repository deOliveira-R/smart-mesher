//! Hot paths of the engine: autosaved apply, bulk save, and opening a large project.
#![allow(missing_docs)]

#[path = "../tests/common/mod.rs"]
mod common;

use std::hint::black_box;

use common::*;
use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use sm_engine::{Durability, Layout, ProjectConfig, Workspace, load_timeline, save_timeline};

/// A project with `n` nodes on main; returns its directory guard and path.
fn project_with(n: usize, tag: &str) -> (TempDir, std::path::PathBuf) {
    let dir = TempDir::new(tag);
    let p = dir.project("p");
    let (b, _) = builder();
    let mut ws = b
        .autosave(false)
        .create(&p, ProjectConfig::new("bench"))
        .unwrap();
    let id = create(&mut ws, "a", 0);
    for k in 1..n {
        add(&mut ws, id, k as i64 % 7);
    }
    ws.save().unwrap();
    ws.close().unwrap();
    (dir, p)
}

fn apply_with_autosave(c: &mut Criterion) {
    let mut g = c.benchmark_group("apply_autosave");
    for (name, dur) in [("fast", Durability::Fast), ("full_fsync", Durability::Full)] {
        let dir = TempDir::new("bench-apply");
        let (b, _) = builder();
        let mut ws = b
            .durability(dur)
            .create(dir.project("p"), ProjectConfig::new("b"))
            .unwrap();
        let id = create(&mut ws, "a", 0);
        g.bench_function(name, |b| b.iter(|| black_box(add(&mut ws, id, 1))));
    }
    g.finish();
}

fn apply_in_memory(c: &mut Criterion) {
    let (b, _) = builder();
    let mut ws = b.memory().unwrap();
    let id = create(&mut ws, "a", 0);
    c.bench_function("apply_in_memory", |b| {
        b.iter(|| black_box(add(&mut ws, id, 1)))
    });
}

fn bulk_save(c: &mut Criterion) {
    let mut g = c.benchmark_group("bulk_save");
    g.sample_size(10);
    for n in [1_000usize, 5_000] {
        let (b, _) = builder();
        let mut ws = b.memory().unwrap();
        let id = create(&mut ws, "a", 0);
        for k in 1..n {
            add(&mut ws, id, k as i64 % 7);
        }
        let tl = ws.timeline().clone();
        g.bench_function(format!("{n}_nodes_fast"), |b| {
            b.iter_batched(
                || TempDir::new("bench-save"),
                |dir| {
                    save_timeline(&Layout::new(dir.project("p")), &tl, &[], Durability::Fast)
                        .unwrap();
                    dir
                },
                BatchSize::PerIteration,
            )
        });
    }
    g.finish();
}

fn open_project(c: &mut Criterion) {
    let mut g = c.benchmark_group("open");
    g.sample_size(10);
    for n in [1_000usize, 5_000] {
        let (_dir, p) = project_with(n, "bench-open");
        g.bench_function(format!("load_timeline_{n}"), |b| {
            b.iter(|| black_box(load_timeline(&Layout::new(&p)).unwrap().0.len()))
        });
        g.bench_function(format!("open_replay_{n}"), |b| {
            b.iter(|| {
                let (bld, _) = builder();
                black_box(bld.open(&p).unwrap().state_hash())
            })
        });
    }
    // With a snapshot at the tip the replay disappears.
    let (_dir, p) = project_with(5_000, "bench-open-snap");
    let (bld, _) = builder();
    let mut ws = bld.open(&p).unwrap();
    assert!(ws.snapshot_now().unwrap());
    ws.close().unwrap();
    g.bench_function("open_snapshot_5000", |b| {
        b.iter(|| {
            let (bld, _) = builder();
            black_box(bld.open(&p).unwrap().state_hash())
        })
    });
    g.finish();
}

fn facade_json(c: &mut Criterion) {
    let (b, _) = builder();
    let mut ws: Workspace = b.memory().unwrap();
    let id = create(&mut ws, "a", 0);
    let params = serde_json::json!({"id": id, "delta": 1});
    c.bench_function("apply_json_in_memory", |b| {
        b.iter(|| {
            black_box(
                ws.apply_json("counter.add", params.clone(), &opts())
                    .unwrap()
                    .node,
            )
        })
    });
    c.bench_function("query_json", |b| {
        b.iter(|| black_box(ws.query("counter.total", &serde_json::json!({})).unwrap()))
    });
}

criterion_group!(
    benches,
    apply_in_memory,
    facade_json,
    apply_with_autosave,
    bulk_save,
    open_project
);
criterion_main!(benches);
