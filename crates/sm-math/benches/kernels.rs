//! Hot-path benchmarks: predicate filter hit, exact fallback, SoS tie-break, and parallel sums.

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use sm_core::Point3;
use sm_math::sos::{Tagged, insphere_sos};
use sm_math::sum::{par_sum, sum_neumaier};
use sm_math::{incircle, insphere, orient2d, orient3d};
use std::hint::black_box;

struct Lcg(u64);
impl Lcg {
    fn f(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
    fn p(&mut self) -> Point3 {
        Point3::new(self.f(), self.f(), self.f())
    }
}

fn predicates(c: &mut Criterion) {
    let mut r = Lcg(1);
    let pts: Vec<Point3> = (0..1024).map(|_| r.p()).collect();
    let mut g = c.benchmark_group("filter_hit");
    g.bench_function("orient2d", |b| {
        b.iter(|| {
            let mut s = 0;
            for w in pts.windows(3) {
                s += orient2d([w[0].x, w[0].y], [w[1].x, w[1].y], [w[2].x, w[2].y]).as_i32();
            }
            black_box(s)
        })
    });
    g.bench_function("orient3d", |b| {
        b.iter(|| {
            let mut s = 0;
            for w in pts.windows(4) {
                s += orient3d(w[0], w[1], w[2], w[3]).as_i32();
            }
            black_box(s)
        })
    });
    g.bench_function("incircle", |b| {
        b.iter(|| {
            let mut s = 0;
            for w in pts.windows(4) {
                s += incircle(
                    [w[0].x, w[0].y],
                    [w[1].x, w[1].y],
                    [w[2].x, w[2].y],
                    [w[3].x, w[3].y],
                )
                .as_i32();
            }
            black_box(s)
        })
    });
    g.bench_function("insphere", |b| {
        b.iter(|| {
            let mut s = 0;
            for w in pts.windows(5) {
                s += insphere(w[0], w[1], w[2], w[3], w[4]).as_i32();
            }
            black_box(s)
        })
    });
    g.finish();

    // Exactly degenerate inputs always take the exact stage.
    let mut g = c.benchmark_group("exact_fallback");
    let (a, b, cc) = ([0.0, 0.0], [1.0, 1.0], [3.0, 3.0]);
    g.bench_function("orient2d_collinear", |bn| {
        bn.iter(|| orient2d(black_box(a), black_box(b), black_box(cc)))
    });
    let (p0, p1, p2, p3) = (
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    );
    g.bench_function("orient3d_coplanar", |bn| {
        bn.iter(|| orient3d(black_box(p0), black_box(p1), black_box(p2), black_box(p3)))
    });
    g.bench_function("incircle_cocircular", |bn| {
        bn.iter(|| incircle([0.0, 0.0], [1.0, 0.0], [0.0, 1.0], black_box([1.0, 1.0])))
    });
    let cube = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
    ];
    g.bench_function("insphere_cospherical", |bn| {
        bn.iter(|| {
            insphere(
                cube[0],
                cube[1],
                cube[2],
                cube[3],
                black_box(Point3::new(1.0, 1.0, 0.0)),
            )
        })
    });
    g.bench_function("insphere_sos_cospherical", |bn| {
        bn.iter(|| {
            let t = |p: Point3, i: u64| Tagged::new(p, i);
            insphere_sos(
                t(cube[0], 0),
                t(cube[1], 1),
                t(cube[2], 2),
                t(cube[3], 3),
                t(black_box(Point3::new(1.0, 1.0, 0.0)), 4),
            )
        })
    });
    g.finish();
}

fn sums(c: &mut Criterion) {
    let mut r = Lcg(2);
    let xs: Vec<f64> = (0..1_000_000).map(|_| r.f() - 0.5).collect();
    let mut g = c.benchmark_group("sum_1e6");
    g.bench_function("naive", |b| b.iter(|| black_box(xs.iter().sum::<f64>())));
    g.bench_function("neumaier", |b| b.iter(|| sum_neumaier(black_box(&xs))));
    g.bench_function("par_sum", |b| {
        b.iter_batched(|| (), |_| par_sum(black_box(&xs)), BatchSize::SmallInput)
    });
    g.finish();
}

criterion_group!(benches, predicates, sums);
criterion_main!(benches);
