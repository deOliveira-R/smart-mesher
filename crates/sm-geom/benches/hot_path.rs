//! Hot paths: NURBS jets (prepared vs the contract's reference evaluator), closest-point
//! projection, batched evaluation, and surface-surface intersection.

use criterion::{BatchSize, Criterion, black_box, criterion_group, criterion_main};
use sm_core::{Point3, Vector3};
use sm_geom::exact;
use sm_geom::{PreparedCurve, PreparedSurface, SsiOptions, intersect_surfaces};
use sm_geom_api::{Circle, Cylinder, Frame, ModelTolerance, NurbsSurface, ParametricCurve, ParametricSurface, Sphere, SurfaceDef, Uv};

fn bicubic_net() -> NurbsSurface {
    let n = 12;
    let mut control = Vec::new();
    for i in 0..n {
        for j in 0..n {
            control.push(Point3::new(i as f64, j as f64, ((i * 7 + j * 13) % 11) as f64 * 0.2));
        }
    }
    let weights = (0..n * n).map(|i| 1.0 + 0.05 * (i % 7) as f64).collect();
    let mut k = vec![0.0; 4];
    for i in 1..n - 3 {
        k.push(i as f64 / (n - 3) as f64);
    }
    k.extend([1.0; 4]);
    NurbsSurface::new(3, 3, k.clone(), k, n, n, control, Some(weights)).unwrap()
}

fn params(n: usize) -> Vec<Uv> {
    (0..n).map(|i| Uv::new((i as f64 * 0.618_033_988_75).fract(), (i as f64 * 0.414_213_562_37).fract())).collect()
}

fn jets(c: &mut Criterion) {
    let s = bicubic_net();
    let ps = PreparedSurface::new(&s).unwrap();
    let uvs = params(10_000);
    let mut g = c.benchmark_group("surface_jet_1e4");
    g.bench_function("reference", |b| b.iter(|| uvs.iter().map(|q| s.jet(*q).p.x).sum::<f64>()));
    g.bench_function("prepared", |b| b.iter(|| uvs.iter().map(|q| ps.jet(*q).p.x).sum::<f64>()));
    g.finish();
    let mut g = c.benchmark_group("surface_eval_1e4");
    g.bench_function("reference", |b| b.iter(|| uvs.iter().map(|q| s.eval(*q).x).sum::<f64>()));
    g.bench_function("prepared", |b| b.iter(|| uvs.iter().map(|q| ps.eval(*q).x).sum::<f64>()));
    g.finish();
    let mut out = vec![Point3::default(); uvs.len()];
    c.bench_function("surface_eval_many_1e4_rayon", |b| b.iter(|| ps.eval_many(black_box(&uvs), &mut out)));
    let torus = PreparedSurface::new(&exact::torus_to_nurbs(&sm_geom_api::Torus::new(Frame::at(Point3::default()), 3.0, 1.0).unwrap()).unwrap()).unwrap();
    let tuv = params(10_000);
    let mut g = c.benchmark_group("rational_torus_jet_1e4");
    g.bench_function("prepared", |b| b.iter(|| tuv.iter().map(|q| torus.jet(Uv::new(q.u * 6.0, q.v * 6.0)).p.x).sum::<f64>()));
    g.finish();
}

fn projection(c: &mut Criterion) {
    let s = bicubic_net();
    let ps = PreparedSurface::new(&s).unwrap();
    let queries: Vec<Point3> = (0..64).map(|i| Point3::new((i as f64 * 0.37).sin() * 8.0 + 5.0, (i as f64 * 0.73).cos() * 8.0 + 5.0, 2.0 + (i % 5) as f64 * 0.3)).collect();
    let mut g = c.benchmark_group("surface_projection_64");
    g.sample_size(20);
    g.bench_function("reference", |b| b.iter(|| queries.iter().map(|q| s.project(*q, None).unwrap().distance).sum::<f64>()));
    g.bench_function("prepared", |b| b.iter(|| queries.iter().map(|q| ps.project(*q, None).unwrap().distance).sum::<f64>()));
    g.finish();
    let circle = PreparedCurve::new(&exact::circle_to_nurbs(&Circle::new(Frame::at(Point3::default()), 2.0).unwrap()).unwrap()).unwrap();
    c.bench_function("curve_projection_64", |b| b.iter(|| queries.iter().map(|q| circle.project(*q, None).unwrap().distance).sum::<f64>()));
}

fn intersection(c: &mut Criterion) {
    let a = SurfaceDef::Cylinder(Cylinder::new(Frame::at(Point3::default()), 1.0).unwrap());
    let b = SurfaceDef::Cylinder(Cylinder::new(Frame::from_z(Point3::default(), Vector3::new(1.0, 0.0, 0.0)).unwrap(), 0.6).unwrap());
    let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window([-3.0, -3.0, -3.0, 3.0, 3.0, 3.0]);
    let mut g = c.benchmark_group("ssi");
    g.sample_size(20);
    g.bench_function("cylinder_cylinder_marched", |bn| bn.iter_batched(|| (), |_| intersect_surfaces(&a, &b, &o).unwrap(), BatchSize::SmallInput));
    let sph = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::default()), 2.0).unwrap());
    let pl = SurfaceDef::Plane(sm_geom_api::Plane { frame: Frame::from_z(Point3::new(0.0, 0.0, 0.5), Vector3::new(0.3, 0.1, 1.0)).unwrap() });
    g.bench_function("plane_sphere_closed_form", |bn| bn.iter(|| intersect_surfaces(black_box(&pl), black_box(&sph), &o).unwrap()));
    g.finish();
}

criterion_group!(benches, jets, projection, intersection);
criterion_main!(benches);
