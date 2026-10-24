//! Determinism (CLAUDE.md): repeated runs, one thread versus many, and concurrent callers give
//! bit-identical results for evaluation, projection, intersection and the store.

use sm_core::{Point3, Vector3};
use sm_geom::exact;
use sm_geom::{CurveGeometry, KernelStore, PreparedCurve, PreparedSurface, SsiOptions, SsiResult, curve_surface, intersect_surfaces};
use sm_geom_api::{Circle, CurveDef, Cylinder, Frame, GeomAppend, GeomStore, ModelTolerance, Sphere, SurfaceDef, Torus, Uv};
use sm_testkit::determinism::{check_f64_bits, hash_f64_bits};

fn pool<R: Send>(threads: usize, f: impl FnOnce() -> R + Send) -> R {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool").install(f)
}

fn world() -> Frame {
    Frame::at(Point3::new(0.0, 0.0, 0.0))
}

fn flatten_ssi(r: &SsiResult, out: &mut Vec<f64>) {
    if let SsiResult::Curves(cs) = r {
        for c in cs {
            if let CurveGeometry::Marched(m) = &c.geometry {
                for p in &m.points {
                    out.extend([p.x, p.y, p.z]);
                }
                for q in m.uv_a.iter().chain(m.uv_b.iter()) {
                    out.extend([q.u, q.v]);
                }
            }
        }
    }
}

fn workload(threads: usize) -> Vec<f64> {
    pool(threads, || {
        let mut out = Vec::new();
        // Batch evaluation and projection on a NURBS surface and curve.
        let torus = exact::torus_to_nurbs(&Torus::new(world(), 3.0, 1.0).unwrap()).unwrap();
        let ps = PreparedSurface::new(&torus).unwrap();
        let uvs: Vec<Uv> = (0..800).map(|i| Uv::new(6.0 * (i as f64 * 0.37).sin().abs(), 6.0 * (i as f64 * 0.61).cos().abs())).collect();
        let mut pts = vec![Point3::default(); uvs.len()];
        ps.eval_many(&uvs, &mut pts);
        for p in &pts {
            out.extend([p.x, p.y, p.z]);
        }
        let queries: Vec<Point3> = pts.iter().take(24).map(|p| Point3::new(p.x * 1.2 + 0.1, p.y * 0.9, p.z + 0.4)).collect();
        for pr in ps.project_many(&queries) {
            out.extend([pr.uv.u, pr.uv.v, pr.distance]);
        }
        let cir = exact::circle_to_nurbs(&Circle::new(world(), 2.0).unwrap()).unwrap();
        let pc = PreparedCurve::new(&cir).unwrap();
        for pr in pc.project_many(&queries) {
            out.extend([pr.t, pr.distance]);
        }
        // Intersections (marched) and curve/surface hits.
        let a = SurfaceDef::Cylinder(Cylinder::new(world(), 1.0).unwrap());
        let b = SurfaceDef::Cylinder(Cylinder::new(Frame::from_z(Point3::default(), Vector3::new(1.0, 0.0, 0.0)).unwrap(), 0.6).unwrap());
        let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window([-3.0, -3.0, -3.0, 3.0, 3.0, 3.0]);
        flatten_ssi(&intersect_surfaces(&a, &b, &o).unwrap(), &mut out);
        let ns = SurfaceDef::Nurbs(exact::sphere_to_nurbs(&Sphere::new(world(), 2.0).unwrap()).unwrap());
        let pl = SurfaceDef::Plane(sm_geom_api::Plane { frame: Frame::from_z(Point3::new(0.0, 0.0, 0.5), Vector3::new(0.3, 0.1, 1.0)).unwrap() });
        flatten_ssi(&intersect_surfaces(&ns, &pl, &o).unwrap(), &mut out);
        let line = CurveDef::Line(sm_geom_api::Line::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.3, 0.2)).unwrap());
        if let sm_geom::CurveSurface::Points(h) = curve_surface(&line, None, &SurfaceDef::Nurbs(torus), &ModelTolerance::DEFAULT).unwrap() {
            for x in h {
                out.extend([x.t, x.point.x, x.point.y, x.point.z]);
            }
        }
        // Store ids and prepared evaluation.
        let mut s = KernelStore::new();
        let id = s.add_surface(ns).unwrap();
        let j = s.surface(id).unwrap().jet(Uv::new(1.0, 0.5));
        out.extend([j.p.x, j.p.y, j.p.z, j.du.x, j.dvv.z]);
        out
    })
}

#[test]
fn repeated_runs_one_thread_vs_many_and_concurrent_callers_are_bit_identical() {
    let base = check_f64_bits(2, 4, workload).expect("determinism");
    assert!(base.len() > 2000, "the workload produced output: {}", base.len());
    assert_eq!(hash_f64_bits(&base), hash_f64_bits(&workload(3)));
}
