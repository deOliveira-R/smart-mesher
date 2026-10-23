//! Tests against analytic results.

use crate::predicates::{orient2d_exact, orient3d_exact};
use crate::*;
use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI, TAU};
use sm_core::{Point3, Transform, Vector3};
use std::sync::Arc;

fn tol() -> ModelTolerance {
    ModelTolerance::default()
}
fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}
fn near(a: f64, b: f64, e: f64) -> bool {
    (a - b).abs() <= e
}
fn near_p(a: Point3, b: Point3, e: f64) -> bool {
    (a - b).norm() <= e
}
fn frame(o: Point3, z: Vector3) -> Frame {
    Frame::from_z(o, z).unwrap()
}

fn curves() -> Vec<(&'static str, Box<dyn ParametricCurve>)> {
    let f = frame(p3(1.0, -2.0, 0.5), Vector3::new(1.0, 2.0, 3.0));
    vec![
        ("line", Box::new(Line::new(p3(1.0, 2.0, 3.0), Vector3::new(1.0, -1.0, 2.0)).unwrap())),
        ("circle", Box::new(Circle::new(f, 2.5).unwrap())),
        ("ellipse", Box::new(Ellipse::new(f, 3.0, 1.5).unwrap())),
    ]
}

fn surfaces() -> Vec<(&'static str, AnalyticSurface)> {
    let f = frame(p3(0.3, -0.2, 0.7), Vector3::new(0.2, -0.5, 1.0));
    vec![
        ("plane", Plane { frame: f }.into()),
        ("cylinder", Cylinder::new(f, 1.7).unwrap().into()),
        ("cone", Cone::new(f, 0.4).unwrap().into()),
        ("sphere", Sphere::new(f, 2.0).unwrap().into()),
        ("torus", Torus::new(f, 3.0, 0.8).unwrap().into()),
    ]
}

#[test]
fn objects_are_dyn_compatible() {
    let c = Line::new(p3(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
    let s = Sphere::new(Frame::from_z(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap(), 1.0).unwrap();
    let _: &dyn ParametricCurve = &c;
    let _: &dyn ParametricSurface = &s;
    let _: &dyn ImplicitSurface = &s;
    let _: &dyn RayIntersect = &s;
    let lib = ShapeLibrary::new();
    let _: &dyn ShapeStore = &lib;
}

#[test]
fn curve_derivatives_match_finite_differences() {
    let h = 1e-6;
    for (name, c) in curves() {
        for &t in &[0.3, 1.1, 4.0] {
            let fd1 = (c.eval(t + h) - c.eval(t - h)) * (0.5 / h);
            let fd2 = (c.d1(t + h) - c.d1(t - h)) * (0.5 / h);
            assert!((fd1 - c.d1(t)).norm() < 1e-6, "{name} d1");
            assert!((fd2 - c.d2(t)).norm() < 1e-6, "{name} d2");
        }
    }
}

#[test]
fn surface_derivatives_match_finite_differences() {
    let h = 1e-6;
    for (name, s) in surfaces() {
        let (u, v) = match s.kind() {
            SurfaceKind::Cone => (1.0, 2.0),
            SurfaceKind::Sphere => (1.0, 0.4),
            _ => (1.0, 0.7),
        };
        let (su, sv) = s.d1(u, v);
        let d = s.d2(u, v);
        assert!(((s.eval(u + h, v) - s.eval(u - h, v)) * (0.5 / h) - su).norm() < 1e-5, "{name} Su");
        assert!(((s.eval(u, v + h) - s.eval(u, v - h)) * (0.5 / h) - sv).norm() < 1e-5, "{name} Sv");
        assert!(((s.d1(u + h, v).0 - s.d1(u - h, v).0) * (0.5 / h) - d.uu).norm() < 1e-5, "{name} Suu");
        assert!(((s.d1(u, v + h).0 - s.d1(u, v - h).0) * (0.5 / h) - d.uv).norm() < 1e-5, "{name} Suv");
        assert!(((s.d1(u, v + h).1 - s.d1(u, v - h).1) * (0.5 / h) - d.vv).norm() < 1e-5, "{name} Svv");
    }
}

#[test]
fn normal_is_unit_agrees_with_cross_product_and_gradient() {
    for (name, s) in surfaces() {
        let (u, v) = match s.kind() {
            SurfaceKind::Cone => (2.0, 1.5),
            SurfaceKind::Sphere => (2.0, 0.3),
            _ => (2.0, 0.9),
        };
        let n = s.normal(u, v).unwrap();
        let (su, sv) = s.d1(u, v);
        let c = su.cross(sv);
        assert!(near(n.norm(), 1.0, 1e-12), "{name}");
        assert!(n.dot(c) > 0.0, "{name} sign vs Su x Sv");
        assert!((n * c.norm() - c).norm() < 1e-9 * (1.0 + c.norm()), "{name} parallel");
        let g = ImplicitSurface::gradient(&s, s.eval(u, v));
        assert!(near(g.dot(n) / g.norm(), 1.0, 1e-9), "{name} implicit gradient direction");
        assert!(ImplicitSurface::value(&s, s.eval(u, v)).abs() < 1e-9, "{name} on surface");
    }
}

#[test]
fn periodic_seams_and_poles() {
    for (name, s) in surfaces() {
        let d = s.domain();
        if let Some(p) = d.u_period {
            let v = if s.kind() == SurfaceKind::Cone { 1.0 } else { 0.3 };
            assert!(near_p(s.eval(d.u.lo, v), s.eval(d.u.lo + p, v), 1e-12), "{name} u seam");
            assert!(near(d.u.len(), p, 0.0));
        }
        if let Some(p) = d.v_period {
            assert!(near_p(s.eval(0.4, d.v.lo), s.eval(0.4, d.v.lo + p), 1e-12), "{name} v seam");
        }
    }
    let f = frame(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let sph = Sphere::new(f, 2.0).unwrap();
    let poles = sph.poles();
    assert_eq!(poles.len(), 2);
    for pole in &poles {
        let v = if pole.edge == DomainEdge::VMax { FRAC_PI_2 } else { -FRAC_PI_2 };
        for u in [0.0, 1.0, 4.0] {
            assert!(near_p(sph.eval(u, v), pole.point, 1e-12));
        }
        assert!(sph.d1(1.0, v).0.norm() < 1e-12, "Su vanishes at a pole");
        let n = sph.normal(1.0, v).unwrap();
        assert!(near(n.z.abs(), 1.0, 1e-12), "limit normal is the polar axis");
    }
    // the generic default normal reports the singularity
    #[derive(Debug)]
    struct Wrap(Sphere);
    impl ParametricSurface for Wrap {
        fn kind(&self) -> SurfaceKind {
            SurfaceKind::Other
        }
        fn domain(&self) -> SurfaceDomain {
            self.0.domain()
        }
        fn eval(&self, u: f64, v: f64) -> Point3 {
            self.0.eval(u, v)
        }
        fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3) {
            self.0.d1(u, v)
        }
        fn d2(&self, u: f64, v: f64) -> SecondDerivs {
            self.0.d2(u, v)
        }
        fn project(&self, p: Point3, h: Option<[f64; 2]>, t: &ModelTolerance) -> Result<SurfaceProjection, GeomError> {
            surface::project_by_newton(self, p, h, t)
        }
    }
    let w = Wrap(sph);
    assert!(matches!(w.normal(0.5, FRAC_PI_2), Err(GeomError::Singular { .. })));
    // cone apex
    let cone = Cone::new(f, 0.5).unwrap();
    assert_eq!(cone.poles().len(), 1);
    assert!(near_p(cone.eval(2.0, 0.0), f.origin, 1e-15));
}

#[test]
fn projection_round_trips() {
    for (name, s) in surfaces() {
        let (u, v) = match s.kind() {
            SurfaceKind::Cone => (5.5, 2.0),
            SurfaceKind::Sphere => (5.5, 0.4),
            SurfaceKind::Torus => (5.5, 5.0),
            _ => (5.5, 0.7),
        };
        let n = s.normal(u, v).unwrap();
        let off = 0.1;
        let p = s.eval(u, v) + n * off;
        let r = s.project(p, None, &tol()).unwrap();
        assert!(near(r.uv[0], u, 1e-9) && near(r.uv[1], v, 1e-9), "{name} uv {:?} vs {:?}", r.uv, (u, v));
        assert!(near(r.distance, off, 1e-9), "{name} distance");
        // a hint on another period branch is honoured
        if s.domain().u_period.is_some() {
            let r2 = s.project(p, Some([u - TAU, v]), &tol()).unwrap();
            assert!(near(r2.uv[0], u - TAU, 1e-9), "{name} hint branch");
        }
    }
}

#[test]
fn generic_newton_projection_agrees_with_closed_forms() {
    let f = frame(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let cyl = Cylinder::new(f, 1.0).unwrap();
    let p = p3(2.0, 1.0, 0.5);
    let a = cyl.project(p, None, &tol()).unwrap();
    let b = surface::project_by_newton(&cyl, p, None, &tol()).unwrap();
    assert!(near_p(a.point, b.point, 1e-9));
    let ell = Ellipse::new(f, 3.0, 1.5).unwrap();
    let q = ell.project(p3(10.0, 0.0, 0.0), None, &tol()).unwrap();
    assert!(near(q.distance, 7.0, 1e-9) && (near(q.t, 0.0, 1e-9) || near(q.t, TAU, 1e-9)));
    let c = Circle::new(f, 2.0).unwrap().project(p3(0.0, 5.0, 1.0), None, &tol()).unwrap();
    assert!(near(c.t, FRAC_PI_2, 1e-12) && near(c.distance, (9.0f64 + 1.0).sqrt(), 1e-12));
}

#[test]
fn principal_curvatures_of_sphere_and_cylinder() {
    let f = frame(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let (k1, k2) = Sphere::new(f, 2.0).unwrap().principal_curvatures(1.0, 0.3).unwrap();
    assert!(near(k1, -0.5, 1e-12) && near(k2, -0.5, 1e-12));
    let (k1, k2) = Cylinder::new(f, 4.0).unwrap().principal_curvatures(1.0, 0.3).unwrap();
    assert!(near(k1, 0.0, 1e-12) && near(k2, -0.25, 1e-12));
}

#[test]
fn lengths_match_closed_forms() {
    let f = frame(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let circle_as_ellipse = Ellipse::new(f, 2.0, 2.0).unwrap();
    assert!(near(circle_as_ellipse.length(0.0, TAU, &tol()), 4.0 * PI, 1e-10));
    assert!(near(Circle::new(f, 2.0).unwrap().length(0.0, PI, &tol()), 2.0 * PI, 1e-14));
    // quarter of an a=3, b=1.5 ellipse via default integration vs a high-resolution sum
    let e = Ellipse::new(f, 3.0, 1.5).unwrap();
    let n = 200_000;
    let mut s = 0.0;
    let mut prev = e.eval(0.0);
    for i in 1..=n {
        let q = e.eval(FRAC_PI_2 * f64::from(i) / f64::from(n));
        s += (q - prev).norm();
        prev = q;
    }
    assert!(near(e.length(0.0, FRAC_PI_2, &tol()), s, 1e-8));
}

fn hits(s: &dyn RayIntersect, o: Point3, d: Vector3) -> Vec<RayHit> {
    let mut v = Vec::new();
    s.intersect_ray(&Ray { origin: o, dir: d }, 0.0, f64::INFINITY, &tol(), &mut v);
    v
}

#[test]
fn ray_hits_on_analytic_surfaces() {
    let f = frame(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let x = Vector3::new(1.0, 0.0, 0.0);
    let o = p3(-10.0, 0.0, 0.0);
    // sphere R=2
    let h = hits(&Sphere::new(f, 2.0).unwrap(), o, x);
    assert_eq!(h.len(), 2);
    assert!(near(h[0].t, 8.0, 1e-12) && near(h[1].t, 12.0, 1e-12));
    assert_eq!(h[0].crossing, NormalCrossing::AgainstNormal);
    assert_eq!(h[1].crossing, NormalCrossing::AlongNormal);
    assert!(near_p(h[0].point, p3(-2.0, 0.0, 0.0), 1e-12));
    // tangent ray
    let h = hits(&Sphere::new(f, 2.0).unwrap(), p3(-10.0, 2.0, 0.0), x);
    assert_eq!(h.len(), 1);
    assert_eq!(h[0].crossing, NormalCrossing::Tangent);
    // torus R=3, r=1: x = -4,-2,2,4
    let h = hits(&Torus::new(f, 3.0, 1.0).unwrap(), o, x);
    let ts: Vec<f64> = h.iter().map(|h| h.t).collect();
    assert_eq!(ts.len(), 4, "{ts:?}");
    for (t, e) in ts.iter().zip([6.0, 8.0, 12.0, 14.0]) {
        assert!(near(*t, e, 1e-9));
    }
    assert!(h.iter().all(|h| h.uv.is_some()));
    // cylinder R=1 hit sideways
    let h = hits(&Cylinder::new(f, 1.0).unwrap(), o, x);
    assert!(near(h[0].t, 9.0, 1e-12) && near(h[1].t, 11.0, 1e-12));
    // plane z=0 from above
    let h = hits(&Plane { frame: f }, p3(0.3, 0.4, 5.0), Vector3::new(0.0, 0.0, -1.0));
    assert!(near(h[0].t, 5.0, 1e-12) && h[0].crossing == NormalCrossing::AgainstNormal);
    assert_eq!(h[0].uv, Some([0.3, 0.4]));
    // cone alpha=45deg opening to +z: ray along x at z=2 hits x=+-2; lower nappe is excluded
    let cone = Cone::new(f, FRAC_PI_4).unwrap();
    let h = hits(&cone, p3(-10.0, 0.0, 2.0), x);
    assert_eq!(h.len(), 2);
    assert!(near(h[0].point.x, -2.0, 1e-9) && near(h[1].point.x, 2.0, 1e-9));
    assert!(hits(&cone, p3(-10.0, 0.0, -2.0), x).is_empty());
    // ray_hook is wired through the trait objects
    let s: &dyn ParametricSurface = &cone;
    assert!(s.ray_hook().is_some());
    // t window
    let mut v = Vec::new();
    Sphere::new(f, 2.0).unwrap().intersect_ray(&Ray { origin: o, dir: x }, 9.0, 20.0, &tol(), &mut v);
    assert_eq!(v.len(), 1);
}

#[test]
fn quadric_csg_halfspaces() {
    let t = tol();
    let sph = Quadric::sphere(p3(1.0, 2.0, 3.0), 2.0);
    assert!(near(sph.value(p3(3.0, 2.0, 3.0)), 0.0, 1e-12));
    assert_eq!(sph.classify(p3(1.0, 2.0, 3.0), &t), Sidedness::Negative);
    assert_eq!(sph.classify(p3(3.0, 2.0, 3.0), &t), Sidedness::On);
    assert_eq!(sph.classify(p3(9.0, 2.0, 3.0), &t), Sidedness::Positive);
    let cyl = Quadric::cylinder(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 2.0), 1.5).unwrap();
    assert!(near(cyl.value(p3(1.5, 0.0, 100.0)), 0.0, 1e-9));
    let cone = Quadric::cone(p3(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0), FRAC_PI_4).unwrap();
    assert!(near(cone.value(p3(2.0, 0.0, 3.0)), 0.0, 1e-12));
    assert!(near(cone.value(p3(2.0, 0.0, -1.0)), 0.0, 1e-12), "double cone");
    let pl = Quadric::plane(p3(0.0, 0.0, 2.0), Vector3::new(0.0, 0.0, 5.0)).unwrap();
    assert!(near(pl.value(p3(7.0, 7.0, 3.0)), 1.0, 1e-12));
    // translation + mirror (improper) keep the surface consistent
    let mirror = Transform { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]], t: Vector3::new(0.0, 0.0, 10.0) };
    assert!(mirror.is_improper());
    let ms = sph.transformed(&mirror).unwrap();
    let p = p3(3.0, 2.0, 3.0);
    assert!(near(ms.value(mirror.apply_point(p)), 0.0, 1e-9));
    assert_eq!(ms.classify(mirror.apply_point(p3(1.0, 2.0, 3.0)), &t), Sidedness::Negative);
    let tr = Transform { t: Vector3::new(5.0, 0.0, 0.0), ..Transform::IDENTITY };
    assert!(near(sph.transformed(&tr).unwrap().value(p3(8.0, 2.0, 3.0)), 0.0, 1e-9));
    // half-space containment
    let hs = HalfSpace { surface: Arc::new(sph), sense: Sense::Forward };
    assert_eq!(hs.contains(p3(1.0, 2.0, 3.0), &t), Containment::Inside);
    let out = HalfSpace { surface: hs.surface.clone(), sense: Sense::Reversed };
    assert_eq!(out.contains(p3(1.0, 2.0, 3.0), &t), Containment::Outside);
    assert_eq!(out.contains(p3(3.0, 2.0, 3.0), &t), Containment::Boundary);
    // quadric ray hits agree with the analytic sphere
    let mut v = Vec::new();
    Quadric::sphere(p3(0.0, 0.0, 0.0), 2.0).intersect_ray(&Ray { origin: p3(-10.0, 0.0, 0.0), dir: Vector3::new(1.0, 0.0, 0.0) }, 0.0, 100.0, &t, &mut v);
    assert!(v.len() == 2 && near(v[0].t, 8.0, 1e-12) && near(v[1].t, 12.0, 1e-12));
    // torus as an implicit CSG surface: point on the tube is on the surface, the centre is outside
    let tor = Torus::new(frame(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)), 3.0, 1.0).unwrap();
    assert_eq!(ImplicitSurface::classify(&tor, p3(4.0, 0.0, 0.0), &t), Sidedness::On);
    assert_eq!(ImplicitSurface::classify(&tor, p3(3.0, 0.0, 0.0), &t), Sidedness::Negative);
    assert_eq!(ImplicitSurface::classify(&tor, p3(0.0, 0.0, 0.0), &t), Sidedness::Positive);
}

#[test]
fn exact_predicates() {
    assert_eq!(orient2d([0.0, 0.0], [1.0, 0.0], [0.0, 1.0]), Sign::Positive);
    assert_eq!(orient2d([0.0, 0.0], [1.0, 0.0], [0.0, -1.0]), Sign::Negative);
    assert_eq!(orient2d([0.1, 0.1], [0.3, 0.3], [0.7, 0.7]), Sign::Zero);
    let c = [0.7, 0.7f64.next_up()];
    assert_eq!(orient2d([0.1, 0.1], [0.3, 0.3], c), Sign::Positive);
    let (a, b, c) = (p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0));
    assert_eq!(orient3d(a, b, c, p3(0.0, 0.0, -1.0)), Sign::Positive);
    assert_eq!(orient3d(a, b, c, p3(0.0, 0.0, 1.0)), Sign::Negative);
    assert_eq!(orient3d(a, b, c, p3(0.3, 0.3, 0.0)), Sign::Zero);
    assert_eq!(orient3d(a, b, c, p3(0.3, 0.3, f64::MIN_POSITIVE)), Sign::Negative);
    // exact path vs i128 reference on small integer points (deterministic LCG)
    let mut s: u64 = 12345;
    let mut next = || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((s >> 33) % 21) as i64 - 10
    };
    for _ in 0..500 {
        let q: Vec<[i64; 3]> = (0..4).map(|_| [next(), next(), next()]).collect();
        let f = |i: usize| p3(q[i][0] as f64, q[i][1] as f64, q[i][2] as f64);
        let d = |i: usize, k: usize| i128::from(q[i][k] - q[3][k]);
        let det = d(0, 0) * (d(1, 1) * d(2, 2) - d(1, 2) * d(2, 1)) - d(0, 1) * (d(1, 0) * d(2, 2) - d(1, 2) * d(2, 0)) + d(0, 2) * (d(1, 0) * d(2, 1) - d(1, 1) * d(2, 0));
        let want = match det.signum() {
            1 => Sign::Positive,
            -1 => Sign::Negative,
            _ => Sign::Zero,
        };
        assert_eq!(orient3d_exact(f(0), f(1), f(2), f(3)), want);
        assert_eq!(orient3d(f(0), f(1), f(2), f(3)), want);
        let det2 = i128::from(q[0][0] - q[2][0]) * i128::from(q[1][1] - q[2][1]) - i128::from(q[0][1] - q[2][1]) * i128::from(q[1][0] - q[2][0]);
        let want2 = match det2.signum() {
            1 => Sign::Positive,
            -1 => Sign::Negative,
            _ => Sign::Zero,
        };
        let p2 = |i: usize| [q[i][0] as f64, q[i][1] as f64];
        assert_eq!(orient2d_exact(p2(0), p2(1), p2(2)), want2);
    }
    // a case where the naive float determinant is wrong: huge cancellation
    let big = 1e16;
    assert_eq!(orient2d([big, big], [big + 2.0, big + 2.0], [big + 4.0, big + 4.0]), Sign::Zero);
    assert_eq!(orient2d([big, big], [big + 2.0, big + 2.0], [big + 4.0, big + 6.0]), Sign::Positive);
}

#[test]
fn nurbs_validation() {
    // quarter circle, rational quadratic
    let w = 0.5f64.sqrt();
    let good = NurbsCurve {
        degree: 2,
        knots: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        control: vec![p3(1.0, 0.0, 0.0), p3(1.0, 1.0, 0.0), p3(0.0, 1.0, 0.0)],
        weights: Some(vec![1.0, w, 1.0]),
    };
    assert!(good.validate().is_ok());
    assert_eq!(good.domain(), ParamRange::new(0.0, 1.0));
    let mut bad = good.clone();
    bad.knots.pop();
    assert!(matches!(bad.validate(), Err(GeomError::InvalidNurbs(_))));
    let mut bad = good.clone();
    bad.weights = Some(vec![1.0, -1.0, 1.0]);
    assert!(bad.validate().is_err());
    let mut bad = good;
    bad.knots = vec![0.0, 0.0, 0.0, 1.0, 0.5, 1.0];
    assert!(bad.validate().is_err());
    let surf = NurbsSurface {
        degree_u: 1,
        degree_v: 1,
        knots_u: vec![0.0, 0.0, 1.0, 1.0],
        knots_v: vec![0.0, 0.0, 1.0, 1.0],
        count_u: 2,
        count_v: 2,
        control: vec![p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0), p3(0.0, 1.0, 0.0), p3(1.0, 1.0, 0.0)],
        weights: None,
    };
    assert!(surf.validate().is_ok());
    let mut bad = surf;
    bad.control.pop();
    assert!(bad.validate().is_err());
}

#[test]
fn shape_library_resolves_handles() {
    let f = frame(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let mut lib = ShapeLibrary::new();
    let c = lib.add_curve(Circle::new(f, 1.0).unwrap()).unwrap();
    let s = lib.add_analytic_surface(Sphere::new(f, 1.0).unwrap().into()).unwrap();
    let q = lib.add_implicit_surface(Arc::new(Quadric::sphere(p3(0.0, 0.0, 0.0), 1.0))).unwrap();
    assert!(lib.curve(c).is_some() && lib.curve(s).is_none());
    assert!(lib.surface(s).is_some() && lib.implicit(s).is_some());
    assert!(lib.surface(q).is_none() && lib.implicit(q).is_some());
    assert_eq!(c.serial(), 0);
    assert_eq!(s.serial(), 0);
    assert_eq!(q.serial(), 1);
}

#[test]
fn tolerance_policy() {
    let t = ModelTolerance::for_extent(1000.0);
    assert!(near(t.resolution, 1e-6, 1e-18));
    assert!(t.same_point(p3(0.0, 0.0, 0.0), p3(5e-7, 0.0, 0.0)));
    assert!(!t.same_point(p3(0.0, 0.0, 0.0), p3(2e-6, 0.0, 0.0)));
    assert!(t.hash_quantum() < t.resolution);
}

#[test]
fn frames_are_right_handed_orthonormal() {
    for z in [Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 2.0, 3.0), Vector3::new(0.0, -1.0, 0.0)] {
        let f = frame(p3(0.0, 0.0, 0.0), z);
        assert!(near(f.x.dot(f.y), 0.0, 1e-15) && near(f.x.dot(f.z), 0.0, 1e-15));
        assert!((f.x.cross(f.y) - f.z).norm() < 1e-14);
    }
    assert!(Frame::from_z(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0)).is_err());
}

#[test]
fn deterministic_repeat() {
    let f = frame(p3(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let t = Torus::new(f, 3.0, 1.0).unwrap();
    let run = || {
        let h = hits(&t, p3(-10.0, 0.3, 0.1), Vector3::new(1.0, 0.0, 0.0));
        h.iter().map(|h| h.t.to_bits()).collect::<Vec<_>>()
    };
    assert_eq!(run(), run());
}
