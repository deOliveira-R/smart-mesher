//! Record-only benchmarks: 64^3 hex Laplacian (both schedules) and 16^3 Optimize, 10 sweeps.
//!
//! Each case also runs with 1 sweep, so `(t10 - t1) / 9` is the cost of one sweep without the
//! fixed preparation and statistics cost. Node-updates per second = movable nodes / that.

#![allow(missing_docs)]

use criterion::measurement::WallTime;
use criterion::{BenchmarkGroup, Criterion, criterion_group, criterion_main};
use sm_smooth::{Method, NodeSelect, Objective, Schedule, SmoothInput, SmoothRequest, smooth};
use sm_testkit::lattice::{self, LatticeFixture};

fn req(method: Method, schedule: Schedule, iterations: u32) -> SmoothRequest {
    SmoothRequest { nodes: NodeSelect::All, method, iterations, tolerance: 0.0, schedule, fix_boundary: true, ..SmoothRequest::default() }
}

fn case(c: &mut BenchmarkGroup<'_, WallTime>, name: &str, f: &LatticeFixture, method: Method, schedule: Schedule) {
    let input = SmoothInput { mesh: &f.mesh, topo: &f.topo, model: &f.model };
    for iters in [1u32, 10] {
        let r = req(method, schedule, iters);
        c.bench_function(format!("{name}/{schedule:?}/{iters}_sweeps"), |b| b.iter(|| smooth(&input, &r).expect("smooth")));
    }
}

fn benches(c: &mut Criterion) {
    let mut g = c.benchmark_group("smooth");
    g.sample_size(10);
    let big = lattice::hex([64; 3], [64.0; 3]).expect("64^3").perturbed(1, 0.2).expect("perturb");
    eprintln!("64^3 lattice: {} nodes", sm_mesh_api::MeshRead::vertex_count(&big.mesh));
    for s in [Schedule::Colored, Schedule::Jacobi] {
        case(&mut g, "hex64_laplacian", &big, Method::Laplacian { relax: 1.0 }, s);
    }
    drop(big);
    let small = lattice::hex([16; 3], [16.0; 3]).expect("16^3").perturbed(1, 0.2).expect("perturb");
    case(&mut g, "hex16_optimize_p4", &small, Method::Optimize { objective: Objective::MeanRatio, p: 4.0 }, Schedule::Colored);
    g.finish();
}

criterion_group!(group, benches);
criterion_main!(group);
