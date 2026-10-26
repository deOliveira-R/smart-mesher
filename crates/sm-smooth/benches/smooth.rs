//! Record-only benchmarks: 64^3 hex Laplacian (both schedules) and 16^3 Optimize, 10 sweeps.
//!
//! Each case also runs with 1 sweep, so `(t10 - t1) / 9` is the cost of one sweep without the
//! fixed preparation and statistics cost. Node-updates per second = movable nodes / that.

#![allow(missing_docs)]

use criterion::measurement::WallTime;
use criterion::{BenchmarkGroup, Criterion, criterion_group, criterion_main};
use sm_core::{EntityKind, Point3};
use sm_mesh_api::{MeshEdit, MeshEntityKind, MeshRead, PointIx};
use sm_smooth::{Method, NodeSelect, Objective, Schedule, SmoothInput, SmoothRequest, smooth};
use sm_testkit::lattice::{self, LatticeFixture};
use sm_testkit::surf3::{SplitMix64, mix64};

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

/// Jitter the volume-classified nodes with `set_point` only. (`lattice::perturb` reclassifies
/// one node at a time, which rebuilds the reverse class index per call and is quadratic.)
fn jitter(f: &mut LatticeFixture, seed: u64, amp: f64) {
    for i in 0..f.mesh.vertex_count() {
        if f.mesh.classification(MeshEntityKind::Vertex, i as u32).entity.kind() != EntityKind::TopoVolume {
            continue;
        }
        let p = f.mesh.point(i);
        let mut rng = SplitMix64(mix64(seed) ^ mix64(i as u64));
        let q = Point3::new(p.x + amp * rng.unit(), p.y + amp * rng.unit(), p.z + amp * rng.unit());
        f.mesh.set_point(PointIx::new(i), q).expect("set_point");
    }
}

fn benches(c: &mut Criterion) {
    let mut g = c.benchmark_group("smooth");
    g.sample_size(10);
    let mut big = lattice::hex([64; 3], [64.0; 3]).expect("64^3");
    jitter(&mut big, 1, 0.2);
    eprintln!("64^3 lattice: {} nodes", big.mesh.vertex_count());
    for s in [Schedule::Colored, Schedule::Jacobi] {
        case(&mut g, "hex64_laplacian", &big, Method::Laplacian { relax: 1.0 }, s);
    }
    drop(big);
    let mut small = lattice::hex([16; 3], [16.0; 3]).expect("16^3");
    jitter(&mut small, 1, 0.2);
    case(&mut g, "hex16_optimize_p4", &small, Method::Optimize { objective: Objective::MeanRatio, p: 4.0 }, Schedule::Colored);
    g.finish();
}

criterion_group!(group, benches);
criterion_main!(group);
