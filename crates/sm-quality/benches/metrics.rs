//! Hot-path benchmarks: element kernels, the parallel mesh driver and the reductions.

#![allow(missing_docs)]

use criterion::{Criterion, black_box, criterion_group, criterion_main};
use sm_core::Point3;
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::{CellShape, MeshEdit, MeshRead, PointIx, SerialAlloc};
use sm_quality::{Context, Metric, Options, compute, compute_many, eval, histogram, summarize, validity};

fn block(n: u32) -> ReferenceMesh {
    let mut m = hex_block([n, n, n], [1.0; 3], &mut SerialAlloc::new()).expect("block");
    let mut s: u64 = 0x9E37_79B9_7F4A_7C15;
    for i in 0..m.vertex_count() {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let d = ((s >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 0.3;
        let p = m.point(i);
        let interior = [p.x, p.y, p.z].iter().all(|c| *c > 1e-9 && (*c - n as f64).abs() > 1e-9);
        if interior {
            m.set_point(PointIx::new(i), Point3::new(p.x + d, p.y - d, p.z + 0.5 * d)).expect("set_point");
        }
    }
    m
}

fn benches(c: &mut Criterion) {
    let hex: Vec<[f64; 3]> = vec![
        [0., 0., 0.], [1., 0., 0.], [1.1, 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.2], [0., 1., 1.],
    ];
    let ctx = Context::default();
    c.bench_function("element/hex_scaled_jacobian", |b| b.iter(|| eval(Metric::ScaledJacobian, CellShape::Hex, black_box(&hex), &ctx)));
    c.bench_function("element/hex_condition", |b| b.iter(|| eval(Metric::Condition, CellShape::Hex, black_box(&hex), &ctx)));
    c.bench_function("element/hex_distortion", |b| b.iter(|| eval(Metric::Distortion, CellShape::Hex, black_box(&hex), &ctx)));

    let m = block(40); // 64 000 hexahedra
    let opts = Options::default();
    c.bench_function("mesh/64k_hex_scaled_jacobian", |b| b.iter(|| compute(&m, Metric::ScaledJacobian, &opts).expect("compute")));
    c.bench_function("mesh/64k_hex_volume", |b| b.iter(|| compute(&m, Metric::Volume, &opts).expect("compute")));
    let set = [Metric::ScaledJacobian, Metric::Shape, Metric::Skew, Metric::Condition, Metric::RelativeSizeSquared];
    c.bench_function("mesh/64k_hex_five_metrics", |b| b.iter(|| compute_many(&m, &set, &opts).expect("compute")));
    c.bench_function("mesh/64k_hex_validity", |b| b.iter(|| validity(&m, &opts).expect("validity")));

    let col: Vec<f64> = (0..1_000_000).map(|i| (i % 1013) as f64 / 1013.0).collect();
    c.bench_function("stats/summarize_1M", |b| b.iter(|| summarize(black_box(&col))));
    c.bench_function("stats/histogram_1M_64", |b| b.iter(|| histogram(black_box(&col), 0.0, 1.0, 64).expect("hist")));
}

criterion_group!(group, benches);
criterion_main!(group);
