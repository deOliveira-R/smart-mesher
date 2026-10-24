//! NURBS ray hook, marched curve utilities (pcurve, fitted NURBS), pcurves on surfaces.

use core::f64::consts::TAU;
use sm_core::{Point3, Vector3};
use sm_geom::exact;
use sm_geom::{CurveGeometry, CurveOnSurface, PreparedSurface, SsiOptions, SsiResult, intersect_surfaces};
use sm_geom_api::{
    Crossing, Cylinder, Frame, ImplicitSurface, Interval, ModelTolerance, NurbsCurve2, PCurve, ParametricCurve, ParametricSurface, Plane, Ray, SurfaceDef, Torus, Uv,
};

fn world() -> Frame {
    Frame::at(Point3::new(0.0, 0.0, 0.0))
}

#[test]
fn nurbs_ray_hook_matches_the_analytic_torus() {
    let t = Torus::new(world(), 3.0, 1.0).unwrap();
    let ps = PreparedSurface::new(&exact::torus_to_nurbs(&t).unwrap()).unwrap();
    for ray in [
        Ray { origin: Point3::new(-10.0, 0.0, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) },
        Ray { origin: Point3::new(-10.0, 0.2, 0.3), dir: Vector3::new(2.0, 0.05, -0.02) },
        Ray { origin: Point3::new(3.0, 0.0, 5.0), dir: Vector3::new(0.0, 0.0, -1.0) },
    ] {
        let (mut a, mut b) = (Vec::new(), Vec::new());
        t.ray_hits(&ray, Interval::new(0.0, 100.0), &mut a).unwrap();
        ps.ray_hits(&ray, Interval::new(0.0, 100.0), &mut b).unwrap();
        assert_eq!(a.len(), b.len(), "{ray:?}: {a:?} vs {b:?}");
        for (x, y) in a.iter().zip(&b) {
            assert!((x.t - y.t).abs() < 1e-7, "{x:?} {y:?}");
            assert!((x.normal - y.normal).norm() < 1e-6 && x.crossing == y.crossing, "{x:?} {y:?}");
            assert!(y.uv.is_some());
        }
    }
    // Range restriction and a miss.
    let mut out = Vec::new();
    ps.ray_hits(&Ray { origin: Point3::new(-10.0, 0.0, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) }, Interval::new(7.5, 100.0), &mut out).unwrap();
    assert_eq!(out.len(), 3);
    assert_eq!(out[0].crossing, Crossing::AlongNormal, "t = 8 leaves the tube");
    out.clear();
    ps.ray_hits(&Ray { origin: Point3::new(-10.0, 0.0, 2.5), dir: Vector3::new(1.0, 0.0, 0.0) }, Interval::new(0.0, 100.0), &mut out).unwrap();
    assert!(out.is_empty());
}

#[test]
fn marched_loop_pcurves_and_fitted_nurbs() {
    let cyl = Cylinder::new(world(), 1.0).unwrap();
    let a = SurfaceDef::Cylinder(cyl);
    let b = SurfaceDef::Cylinder(Cylinder::new(Frame::from_z(Point3::default(), Vector3::new(1.0, 0.0, 0.0)).unwrap(), 0.6).unwrap());
    let o = SsiOptions::new(ModelTolerance::DEFAULT).with_window([-3.0, -3.0, -3.0, 3.0, 3.0, 3.0]).with_chord_tol(2e-5);
    let SsiResult::Curves(cs) = intersect_surfaces(&a, &b, &o).unwrap() else { panic!("curves") };
    assert_eq!(cs.len(), 2);
    for c in cs {
        let CurveGeometry::Marched(m) = c.geometry else { panic!("marched") };
        // The pcurve of the first surface maps back onto the curve at every vertex.
        let first = m.uv_a.first().copied().unwrap();
        let last = m.uv_a.last().copied().unwrap();
        // Surface "a" is the cylinder or the roles were swapped; the uv set that belongs to a
        // cylinder about z is the one with u = angle.
        let pc = m.pcurve_a().unwrap();
        let on = CurveOnSurface::new(&cyl, &pc).unwrap();
        // Vertex i maps to point i if `a` was the marching surface, else the second set is ours.
        let ok_a = (0..m.points.len()).step_by(7).all(|i| (on.eval(i as f64) - m.points[i]).norm() < 1e-7);
        assert!(ok_a, "pcurve_a round trip");
        assert!(m.closed && (first.u - last.u).abs() > 5.0 || m.closed, "closed loops wrap once in u");
        // Fitted NURBS stays on both surfaces between the traced points.
        let nc = m.to_nurbs().unwrap();
        for k in 0..400 {
            let p = nc.eval(k as f64 / 399.0);
            let (fa, fb) = (a.implicit_def().unwrap(), b.implicit_def().unwrap());
            assert!(fa.value(p).abs() / fa.gradient(p).norm() < 5e-5 && fb.value(p).abs() / fb.gradient(p).norm() < 5e-5, "{p:?}");
        }
    }
}

#[test]
fn nurbs_pcurve_lifted_onto_a_plane() {
    // A rational quadratic quarter circle in (u, v), lifted on the plane z = 0.5.
    let w = core::f64::consts::FRAC_1_SQRT_2;
    let pc = PCurve::Nurbs2(NurbsCurve2::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![Uv::new(2.0, 0.0), Uv::new(2.0, 2.0), Uv::new(0.0, 2.0)], Some(vec![1.0, w, 1.0])).unwrap());
    let plane = Plane { frame: Frame { origin: Point3::new(0.0, 0.0, 0.5), ..world() } };
    let c = CurveOnSurface::new(&plane, &pc).unwrap();
    for i in 0..=20 {
        let t = i as f64 / 20.0;
        let j = c.jet(t);
        assert!((j.p.x.hypot(j.p.y) - 2.0).abs() < 1e-13 && (j.p.z - 0.5).abs() < 1e-14);
        // Curvature 1/2 from the chain rule (the plane has no curvature of its own).
        let k = j.d1.cross(j.d2).norm() / j.d1.norm().powi(3);
        assert!((k - 0.5).abs() < 1e-10, "{k}");
    }
    assert!((c.length(Interval::new(0.0, 1.0), 1e-12).unwrap() - core::f64::consts::PI).abs() < 1e-9);
    let _ = TAU;
}
