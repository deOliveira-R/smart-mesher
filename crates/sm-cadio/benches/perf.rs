//! Performance record (acceptance 15, record-only): parse a generated 50 MB file, import 1000
//! disjoint bricks, weld 1e5 vertices. The numbers are written to DESIGN.md, not gated.

use std::sync::Arc;

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use sm_cadio::{CadioPlugin, ImportStep, MemoryInputStore, ReadOptions, Source, StepInputs, part21, read_step, weld_vertices};
use sm_core::Point3;
use sm_layer_api::CoreLayers;
use sm_op_api::{DerivedCache, Engine, QuietControl, Registry, Services, Session};

#[path = "../tests/common/synth.rs"]
mod synth;

fn session(store: Arc<MemoryInputStore>) -> Session {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(|| Box::new(sm_topo::TopoStore::new()))).expect("layers");
    reg.install(&CadioPlugin).expect("plugin");
    let services = Services::new().with(StepInputs(store));
    Session::new(Engine::new(Arc::new(reg), services, Arc::new(DerivedCache::disabled())).expect("engine"))
}

fn parse_50mb(c: &mut Criterion) {
    // Size the file from a probe: bytes per brick.
    let probe = synth::bricks_step(100).len() as f64 / 100.0;
    let n = (50.0e6 / probe).ceil() as usize;
    let bytes = synth::bricks_step(n);
    eprintln!("generated {n} bricks, {:.1} MB", bytes.len() as f64 / 1e6);
    let mut g = c.benchmark_group("parse");
    g.sample_size(10).throughput(Throughput::Bytes(bytes.len() as u64));
    g.bench_function("part21_50MB", |b| b.iter(|| part21::parse(&bytes).expect("parse")));
    g.finish();
}

fn import_1000_bricks(c: &mut Criterion) {
    let bytes = synth::bricks_step(1000);
    let mut g = c.benchmark_group("import_1000_bricks");
    g.sample_size(10);
    g.bench_function("read_step", |b| b.iter(|| read_step(&bytes, &ReadOptions::default()).expect("read")));
    let store = Arc::new(MemoryInputStore::new());
    let r = store.put(&bytes);
    for (label, check) in [("op_check", None), ("op_nocheck", Some(false))] {
        g.bench_function(label, |b| {
            b.iter(|| {
                let mut s = session(Arc::clone(&store));
                let op = ImportStep { source: Source::sha256(r.sha256.clone()), name: None, to_unit: None, assume_unit: None, weld_tol: None, solids: None, sheets: None, check };
                let q = QuietControl::new();
                s.apply(&op, sm_op_api::Meta::new("bench", "", ""), &q.control()).expect("import")
            })
        });
    }
    g.finish();
}

fn weld_1e5(c: &mut Criterion) {
    let mut s = 7u64;
    let mut rnd = move || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (s >> 33) as f64 / (1u64 << 31) as f64
    };
    // 1e5 points, 10% duplicated within the tolerance of an earlier point.
    let mut pts: Vec<Point3> = Vec::with_capacity(100_000);
    for i in 0..100_000 {
        if i % 10 == 9 {
            let j = (rnd() * pts.len() as f64) as usize % pts.len();
            let p = pts[j];
            pts.push(Point3::new(p.x + 1e-8, p.y, p.z));
        } else {
            pts.push(Point3::new(rnd() * 100.0, rnd() * 100.0, rnd() * 100.0));
        }
    }
    let mut g = c.benchmark_group("weld");
    g.sample_size(10);
    g.bench_function("vertices_1e5", |b| b.iter(|| weld_vertices(&pts, 1e-6)));
    g.finish();
    let _ = Registry::new();
}

criterion_group!(benches, parse_50mb, import_1000_bricks, weld_1e5);
criterion_main!(benches);
