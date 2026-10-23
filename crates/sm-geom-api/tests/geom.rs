//! Tests against analytic results.

use core::f64::consts::{FRAC_PI_2, PI, TAU};
use smallvec::SmallVec;
use sm_core::{Point3, Transform, Vector3};
use sm_geom_api::*;

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}
fn v3(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z)
}
fn close(a: f64, b: f64, tol: f64) {
    assert!((a - b).abs() <= tol, "{a} vs {b} (tol {tol})");
}
fn vclose(a: Vector3, b: Vector3, tol: f64) {
    assert!((a - b).norm() <= tol, "{a:?} vs {b:?}");
}

fn frame() -> Frame {
    Frame::from_zx(p3(1.0, -2.0, 0.5), v3(0.3, 0.4, 1.0), v3(1.0, 0.2, 0.0)).unwrap()
}

fn mirror_x() -> Transform {
    Transform { m: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: v3(0.0, 0.0, 0.0) }
}

fn quarter_arc_nurbs() -> NurbsCurve {
    let w = 0.5f64.sqrt();
    NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![p3(2.0, 0.0, 0.0), p3(2.0, 2.0, 0.0), p3(0.0, 2.0, 0.0)], Some(vec![1.0, w, 1.0])).unwrap()
}

fn nurbs_patch() -> NurbsSurface {
    // Quarter cylinder of radius 2 about z, height 3. Poles: arc (u, 3) x height (v, 2).
    let w = 0.5f64.sqrt();
    let arc = [(2.0, 0.0, 1.0), (2.0, 2.0, w), (0.0, 2.0, 1.0)];
    let mut poles = Vec::new();
    let mut weights = Vec::new();
    for (x, y, wi) in arc {
        for z in [0.0, 3.0] {
            poles.push(p3(x, y, z));
            weights.push(wi);
        }
    }
    NurbsSurface { degree_u: 2, degree_v: 1, knots_u: vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], knots_v: vec![0.0, 0.0, 1.0, 1.0], nu: 3, nv: 2, poles, weights: Some(weights) }
}

fn surfaces() -> Vec<SurfaceDef> {
    let f = frame();
    vec![
        SurfaceDef::Plane(Plane { frame: f }),
        SurfaceDef::Cylinder(Cylinder::new(f, 1.5).unwrap()),
        SurfaceDef::Cone(Cone::new(f, 1.0, 0.4).unwrap()),
        SurfaceDef::Sphere(Sphere::new(f, 2.0).unwrap()),
        SurfaceDef::Torus(Torus::new(f, 3.0, 0.8).unwrap()),
        SurfaceDef::Nurbs(nurbs_patch()),
    ]
}

fn sample_uv(s: &dyn ParametricSurface) -> Vec<(f64, f64)> {
    let d = s.domain();
    let (u0, u1) = (d.u.lo.max(-2.0), d.u.hi.min(2.0 * PI));
    let (v0, v1) = (d.v.lo.max(-2.0), d.v.hi.min(2.0 * PI));
    let mut out = Vec::new();
    for i in 0..7 {
        for j in 0..7 {
            // stay off the exact boundaries (poles / seams)
            let (a, b) = ((i as f64 + 0.37) / 7.0, (j as f64 + 0.41) / 7.0);
            out.push((u0 + (u1 - u0) * a, v0 + (v1 - v0) * b));
        }
    }
    out
}

#[test]
fn analytic_points_lie_on_implicit_form_with_matching_normal() {
    for s in surfaces() {
        let (Some(par), Some(imp)) = (s.as_parametric(), s.to_implicit()) else { continue };
        for (u, v) in sample_uv(par) {
            let p = par.eval(u, v);
            close(imp.value(p), 0.0, 1e-9);
            let n = par.normal(u, v).unwrap();
            let g = imp.gradient(p);
            let gn = g * (1.0 / g.norm());
            // oriented the same way
            assert!(n.dot(gn) > 1.0 - 1e-9, "{:?} n.g = {}", par.kind(), n.dot(gn));
        }
    }
}

#[test]
fn surface_derivatives_match_finite_differences() {
    let h = 1e-6;
    for s in surfaces() {
        let par = s.as_parametric().unwrap();
        for (u, v) in sample_uv(par) {
            let d = par.derivs(u, v);
            let fd_u = (par.eval(u + h, v) - par.eval(u - h, v)) * (0.5 / h);
            let fd_v = (par.eval(u, v + h) - par.eval(u, v - h)) * (0.5 / h);
            vclose(d.du, fd_u, 1e-6);
            vclose(d.dv, fd_v, 1e-6);
            let (au, av) = par.d1(u, v);
            let fd_uu = (par.d1(u + h, v).0 - au) * (1.0 / h);
            let fd_uv = (par.d1(u, v + h).0 - au) * (1.0 / h);
            let fd_vv = (par.d1(u, v + h).1 - av) * (1.0 / h);
            vclose(d.duu, fd_uu, 1e-4);
            vclose(d.duv, fd_uv, 1e-4);
            vclose(d.dvv, fd_vv, 1e-4);
        }
    }
}

#[test]
fn projection_recovers_parameters_and_distance() {
    for s in surfaces() {
        let par = s.as_parametric().unwrap();
        for (u, v) in sample_uv(par).into_iter().step_by(5) {
            let n = par.normal(u, v).unwrap();
            let on = par.eval(u, v);
            let q = on + n * 0.05;
            let pr = par.project(q, Some((u, v)));
            close(pr.distance, 0.05, 1e-7);
            assert!((pr.point - on).norm() < 1e-6, "{:?}", par.kind());
            if let Some(pu) = par.period_u() {
                let du = (pr.u - u).rem_euclid(pu);
                assert!(du.min(pu - du) < 1e-6);
            } else {
                close(pr.u, u, 1e-6);
            }
        }
    }
}

#[test]
fn periodic_seam_and_poles() {
    let sph = Sphere::new(Frame::WORLD, 2.0).unwrap();
    assert_eq!(sph.period_u(), Some(TAU));
    close(sph.eval(0.0, 0.3).x, sph.eval(TAU, 0.3).x, 1e-12);
    let sing = sph.singularities();
    assert_eq!(sing.len(), 2);
    assert_eq!(sing[1].locus, SingularLocus::V(FRAC_PI_2));
    vclose(sph.d1(1.0, FRAC_PI_2).0, v3(0.0, 0.0, 0.0), 1e-12);
    // normal is defined at the pole by continuity
    vclose(sph.normal(0.7, FRAC_PI_2).unwrap(), v3(0.0, 0.0, 1.0), 1e-12);
    // projecting a point on the axis uses the hint for the longitude
    let pr = sph.project(p3(0.0, 0.0, 5.0), Some((1.25, 0.0)));
    close(pr.v, FRAC_PI_2, 1e-12);
    close(pr.u, 1.25, 1e-12);
    close(pr.distance, 3.0, 1e-12);
    // cone apex: normal undefined
    let cone = Cone::new(Frame::WORLD, 1.0, 0.5).unwrap();
    let apex = cone.singularities()[0];
    let SingularLocus::V(v0) = apex.locus else { panic!() };
    assert!(cone.normal(0.3, v0).is_err());
    close(cone.eval(1.0, v0).x, apex.point.x, 1e-12);
    // torus is periodic in both directions
    let t = Torus::new(Frame::WORLD, 3.0, 1.0).unwrap();
    assert_eq!((t.period_u(), t.period_v()), (Some(TAU), Some(TAU)));
}

#[test]
fn mirror_reverses_parametric_orientation_and_stays_consistent() {
    let sph = Sphere::new(frame(), 2.0).unwrap();
    let m = sph.transformed(&mirror_x()).unwrap();
    assert!(!m.frame.is_direct());
    let c = m.frame.origin;
    for (u, v) in [(0.4, 0.2), (2.0, -0.8), (5.0, 1.0), (1.0, FRAC_PI_2)] {
        let p = m.eval(u, v);
        close((p - c).norm(), 2.0, 1e-12);
        // An improper map reverses the orientation of du x dv: the normal now points inward.
        assert!(m.normal(u, v).unwrap().dot(p - c) < 0.0);
    }
    let (du, dv) = m.d1(0.4, 0.2);
    assert!(du.cross(dv).dot(m.normal(0.4, 0.2).unwrap()) > 0.0);
    // Implicit form stays "positive outside" regardless of handedness.
    let cyl = SurfaceDef::Cylinder(Cylinder::new(frame(), 1.0).unwrap()).transformed(&mirror_x()).unwrap();
    let (par, imp) = (cyl.as_parametric().unwrap(), cyl.to_implicit().unwrap());
    let p = par.eval(1.0, 0.3);
    close(imp.value(p), 0.0, 1e-9);
    assert!(imp.value(p + (p - par.project(p, None).point) * 0.0 + par.normal(1.0, 0.3).unwrap() * -0.1) > 0.0);
}

#[test]
fn similarity_transform_scales_radius() {
    let s = Transform { m: [[0.0, -2.0, 0.0], [2.0, 0.0, 0.0], [0.0, 0.0, 2.0]], t: v3(1.0, 0.0, 0.0) };
    let c = Circle::new(Frame::WORLD, 1.5).unwrap().transformed(&s).unwrap();
    close(c.radius, 3.0, 1e-12);
    let shear = Transform { m: [[1.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: v3(0.0, 0.0, 0.0) };
    assert_eq!(Circle::new(Frame::WORLD, 1.0).unwrap().transformed(&shear), Err(GeomError::NotSimilarity));
}

#[test]
fn curve_derivatives_and_lengths() {
    let f = frame();
    let curves = vec![
        CurveDef::Line(Line::new(p3(0.0, 1.0, 2.0), v3(1.0, 2.0, 3.0)).unwrap()),
        CurveDef::Circle(Circle::new(f, 2.5).unwrap()),
        CurveDef::Ellipse(Ellipse::new(f, 2.0, 1.0).unwrap()),
        CurveDef::Nurbs(quarter_arc_nurbs()),
    ];
    let h = 1e-6;
    for c in &curves {
        let r = c.range();
        let (lo, hi) = (r.lo.max(-1.0), r.hi.min(2.0));
        for i in 1..6 {
            let t = lo + (hi - lo) * i as f64 / 6.0;
            vclose(c.d1(t), (c.eval(t + h) - c.eval(t - h)) * (0.5 / h), 1e-6);
            vclose(c.d2(t), (c.d1(t + h) - c.d1(t - h)) * (0.5 / h), 1e-5);
        }
    }
    close(curves[1].length(0.0, TAU, 1e-12), TAU * 2.5, 1e-12);
    // Generic quadrature path on the circle, vs the analytic circumference.
    close(sm_geom_api::curve::integrate_speed(&Circle::new(f, 2.5).unwrap(), 0.0, TAU, 1e-12), TAU * 2.5, 1e-9);
    // Ellipse a=2, b=1: perimeter 9.688448220547672 (elliptic integral).
    close(curves[2].length(0.0, TAU, 1e-10), 9.688_448_220_547_672, 1e-8);
    // Rational quarter arc of radius 2 has length pi.
    close(curves[3].length(0.0, 1.0, 1e-12), PI, 1e-10);
    assert!(curves[1].is_closed() && !curves[0].is_closed());
}

#[test]
fn nurbs_reproduces_circle_exactly() {
    let c = quarter_arc_nurbs();
    for i in 0..=20 {
        let t = i as f64 / 20.0;
        close((c.eval(t) - p3(0.0, 0.0, 0.0)).norm(), 2.0, 1e-13);
    }
    vclose(c.eval(0.0) - p3(2.0, 0.0, 0.0), v3(0.0, 0.0, 0.0), 1e-14);
    vclose(c.eval(1.0) - p3(0.0, 2.0, 0.0), v3(0.0, 0.0, 0.0), 1e-14);
    // the patch lies on the radius-2 cylinder
    let s = nurbs_patch();
    s.validate().unwrap();
    for (u, v) in [(0.2, 0.3), (0.5, 0.9), (0.9, 0.1)] {
        let p = s.eval(u, v);
        close(p.x.hypot(p.y), 2.0, 1e-13);
        close(p.z, 3.0 * v, 1e-13);
    }
    // bad data is rejected
    assert!(NurbsCurve::new(2, vec![0.0, 0.0, 1.0, 1.0], vec![p3(0.0, 0.0, 0.0); 3], None).is_err());
    assert!(NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 1.0], vec![p3(0.0, 0.0, 0.0); 2], Some(vec![1.0, -1.0])).is_err());
}

#[test]
fn closest_point_on_curves() {
    let circ = Circle::new(Frame::WORLD, 2.0).unwrap();
    let pr = circ.closest(p3(0.0, 5.0, 1.0));
    close(pr.t, FRAC_PI_2, 1e-12);
    close(pr.distance, (9.0f64 + 1.0).sqrt(), 1e-12);
    // restricted to an arc that excludes the unconstrained optimum: endpoint wins
    let pr = circ.closest_in(p3(0.0, 5.0, 0.0), Interval::new(0.0, 0.5));
    close(pr.t, 0.5, 1e-12);
    // ellipse: on-axis points have known closest points
    let ell = Ellipse::new(Frame::WORLD, 2.0, 1.0).unwrap();
    let pr = ell.closest(p3(5.0, 0.0, 0.0));
    close(pr.t.min(TAU - pr.t), 0.0, 1e-9);
    close(pr.distance, 3.0, 1e-9);
    let nb = quarter_arc_nurbs();
    let pr = nb.closest(p3(3.0, 3.0, 0.0));
    close(pr.t, 0.5, 1e-8);
    close(pr.distance, 18.0f64.sqrt() - 2.0, 1e-8);
    let line = Line::new(p3(0.0, 0.0, 0.0), v3(0.0, 0.0, 2.0)).unwrap();
    close(line.closest(p3(1.0, 0.0, 4.0)).t, 4.0, 1e-12);
}

fn hits(s: &dyn RayIntersect, ray: &Ray) -> SmallVec<[RayHit; 4]> {
    let mut h = SmallVec::new();
    s.intersect_ray(ray, Interval::new(0.0, f64::INFINITY), &TolerancePolicy::default(), &mut h).unwrap();
    h
}

#[test]
fn ray_hits_quadrics_and_torus() {
    let tol = TolerancePolicy::default();
    let ray = Ray::new(p3(-5.0, 0.0, 0.0), v3(2.0, 0.0, 0.0)).unwrap();
    let sphere = Quadric::sphere(p3(0.0, 0.0, 0.0), 2.0).unwrap();
    let h = hits(&sphere, &ray);
    assert_eq!(h.len(), 2);
    close(h[0].t, 3.0, 1e-12);
    close(h[1].t, 7.0, 1e-12);
    vclose(h[0].normal, v3(-1.0, 0.0, 0.0), 1e-12);
    // tangent ray is a single grazing hit
    let tangent = Ray::new(p3(-5.0, 2.0, 0.0), v3(1.0, 0.0, 0.0)).unwrap();
    let h = hits(&sphere, &tangent);
    assert_eq!(h.len(), 1);
    assert!(h[0].grazing);
    // miss
    assert!(hits(&sphere, &Ray::new(p3(-5.0, 3.0, 0.0), v3(1.0, 0.0, 0.0)).unwrap()).is_empty());
    // t_range excludes the surface just crossed
    let mut h = SmallVec::new();
    sphere.intersect_ray(&ray, Interval::new(3.5, f64::INFINITY), &tol, &mut h).unwrap();
    assert_eq!(h.len(), 1);
    close(h[0].t, 7.0, 1e-12);
    // cylinder about z, radius 1.5; cone of half-angle 45 deg at the origin
    let cyl = Quadric::cylinder(p3(0.0, 0.0, 0.0), v3(0.0, 0.0, 1.0), 1.5).unwrap();
    let h = hits(&cyl, &Ray::new(p3(-4.0, 0.0, 9.0), v3(1.0, 0.0, 0.0)).unwrap());
    close(h[0].t, 2.5, 1e-12);
    let cone = Quadric::cone(p3(0.0, 0.0, 0.0), v3(0.0, 0.0, 1.0), PI / 4.0).unwrap();
    let h = hits(&cone, &Ray::new(p3(-4.0, 0.0, 2.0), v3(1.0, 0.0, 0.0)).unwrap());
    assert_eq!(h.len(), 2);
    close(h[0].t, 2.0, 1e-12);
    close(h[1].t, 6.0, 1e-12);
    // torus R=3, r=1 along the x axis: crossings at x = -4, -2, 2, 4
    let torus = ImplicitTorus::new(Frame::WORLD, 3.0, 1.0).unwrap();
    let h = hits(&torus, &Ray::new(p3(-6.0, 0.0, 0.0), v3(1.0, 0.0, 0.0)).unwrap());
    assert_eq!(h.len(), 4);
    for (hit, x) in h.iter().zip([-4.0, -2.0, 2.0, 4.0]) {
        close(hit.point.x, x, 1e-9);
        close(torus.value(hit.point), 0.0, 1e-8);
    }
    // parametric surface hits carry (u, v) and the oriented normal; NURBS is unsupported
    let sd = SurfaceDef::Sphere(Sphere::new(Frame::WORLD, 2.0).unwrap());
    let h = hits(&sd, &ray);
    assert!(h[0].uv.is_some());
    vclose(h[0].normal, v3(-1.0, 0.0, 0.0), 1e-9);
    let mut out = SmallVec::new();
    let err = SurfaceDef::Nurbs(nurbs_patch()).intersect_ray(&ray, Interval::new(0.0, 1e9), &tol, &mut out);
    assert!(matches!(err, Err(GeomError::Unsupported(_))));
}

#[test]
fn polynomial_roots() {
    // (t-1)(t-2)(t+3)(t-5) = t^4 - 5t^3 - 7t^2 + 41t - 30
    let r = ray::poly_roots_in(&[-30.0, 41.0, -7.0, -5.0, 1.0], -10.0, 10.0);
    let want = [-3.0, 1.0, 2.0, 5.0];
    assert_eq!(r.len(), 4);
    for (a, b) in r.iter().zip(want) {
        close(*a, b, 1e-10);
    }
    // double root: (t-1)^2 (t-4)(t+2)
    let r = ray::poly_roots_in(&[-8.0, 14.0, -3.0, -4.0, 1.0], -10.0, 10.0);
    assert_eq!(r.len(), 3);
    for (a, b) in r.iter().zip([-2.0, 1.0, 4.0]) {
        close(*a, b, 1e-6);
    }
}

#[test]
fn quadric_transform_and_halfspace() {
    let s = Quadric::sphere(p3(0.0, 0.0, 0.0), 2.0).unwrap();
    let rot = Transform { m: [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], t: v3(5.0, 1.0, -1.0) };
    let moved = s.transformed(&rot).unwrap();
    close(moved.value(p3(5.0, 1.0, -1.0)), -4.0, 1e-12);
    close(moved.value(p3(7.0, 1.0, -1.0)), 0.0, 1e-12);
    // an off-axis cylinder transforms consistently with point mapping
    let cyl = Quadric::cylinder(p3(1.0, 0.0, 0.0), v3(0.0, 1.0, 1.0), 0.5).unwrap();
    let cm = cyl.transformed(&rot).unwrap();
    for p in [p3(0.3, 0.2, 0.9), p3(2.0, -1.0, 4.0)] {
        close(cm.value(rot.apply_point(p)), cyl.value(p), 1e-12);
    }
    let hs = HalfSpace { surface: ImplicitDef::Quadric(s), positive: false };
    assert!(hs.contains(p3(1.0, 0.0, 0.0), 1e-9));
    assert!(hs.contains(p3(2.0, 0.0, 0.0), 1e-9));
    assert!(!hs.contains(p3(2.1, 0.0, 0.0), 1e-9));
    // GQ coefficient order: x^2 + y^2 + z^2 - 4
    let gq = Quadric::from_gq([1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -4.0]);
    close(gq.value(p3(0.0, 2.0, 0.0)), 0.0, 1e-15);
    assert_eq!(gq.side(p3(0.0, 0.0, 0.0), 1e-9), Sidedness::Negative);
}

#[test]
fn tolerance_policy_rules() {
    let p = TolerancePolicy::default();
    assert!(p.is_valid());
    assert!(p.coincident(1.5e-7, 1e-7, 1e-7));
    assert!(!p.coincident(3e-7, 1e-7, 1e-7));
    close(p.grown(1e-7, 2e-7, 1e-7), 3e-7, 1e-20);
    assert!(!p.within_cap(1e-2));
    assert_eq!(p.clamp_linear(1.0), p.linear_max);
}

#[test]
fn object_safety_and_store() {
    let mut store = SimpleGeomStore::new();
    let c = store.add_curve(CurveDef::Circle(Circle::new(Frame::WORLD, 1.0).unwrap())).unwrap();
    let s = store.add_surface(SurfaceDef::Plane(Plane { frame: Frame::WORLD })).unwrap();
    let gs: &dyn GeomStore = &store;
    let curve: &dyn ParametricCurve = gs.curve(c).unwrap().as_parametric();
    let surf: &dyn ParametricSurface = gs.surface(s).unwrap().as_parametric().unwrap();
    let ray: &dyn RayIntersect = gs.surface(s).unwrap();
    let imp: &dyn ImplicitSurface = &Quadric::sphere(p3(0.0, 0.0, 0.0), 1.0).unwrap();
    close(curve.eval(0.0).x, 1.0, 1e-15);
    close(surf.eval(2.0, 3.0).y, 3.0, 1e-15);
    close(imp.value(p3(2.0, 0.0, 0.0)), 3.0, 1e-15);
    assert_eq!(c.id().to_string(), "Curve#0");
    assert_eq!(s.id().to_string(), "Surface#0");
    let h = hits(ray, &Ray::new(p3(0.0, 0.0, 4.0), v3(0.0, 0.0, -1.0)).unwrap());
    close(h[0].t, 4.0, 1e-12);
}

/// Same inputs -> bit-identical outputs, run twice (no threads or time involved).
#[test]
fn determinism() {
    fn digest() -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        let mut mix = |x: f64| {
            for b in x.to_bits().to_le_bytes() {
                h = (h ^ u64::from(b)).wrapping_mul(0x100_0000_01b3);
            }
        };
        for s in surfaces() {
            let par = s.as_parametric().unwrap();
            for (u, v) in sample_uv(par) {
                let d = par.derivs(u, v);
                [d.p.x, d.p.y, d.p.z, d.du.x, d.dv.y, d.duv.z].into_iter().for_each(&mut mix);
                let pr = par.project(d.p + v3(0.01, 0.02, 0.03), Some((u, v)));
                mix(pr.u);
                mix(pr.v);
            }
        }
        h
    }
    assert_eq!(digest(), digest());
}
