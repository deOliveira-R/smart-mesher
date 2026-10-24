//! W0f gaps4: ellipse and elliptic-cone projection, periodic NURBS.

use core::f64::consts::{PI, TAU};
use sm_core::{Point3, Vector3};
use sm_geom_api::*;

fn tilted() -> Frame {
    Frame::from_zx(Point3::new(0.3, -0.7, 1.1), Vector3::new(1.0, 2.0, -0.5), Vector3::new(0.0, 0.0, 1.0)).unwrap()
}

#[test]
fn ellipse_projection_is_exact() {
    let e = Ellipse::new(tilted(), 3.0, 1.0).unwrap();
    // Analytic: a point beyond the major-axis end projects onto it; one above the plane over a
    // curve point projects onto that point with the height as distance.
    let pr = e.project(e.frame.from_local(Vector3::new(5.0, 0.0, 0.0)), None).unwrap();
    assert!(pr.t.abs() < 1e-12 && (pr.distance - 2.0).abs() < 1e-12);
    let pr = e.project(e.frame.from_local(Vector3::new(0.0, -1.5, 0.0)), None).unwrap();
    assert!((pr.t - 1.5 * PI).abs() < 1e-12 && (pr.distance - 0.5).abs() < 1e-12);
    let on = e.eval(0.7);
    let pr = e.project(on + e.frame.z * 0.25, None).unwrap();
    assert!((pr.point - on).norm() < 1e-9 && (pr.distance - 0.25).abs() < 1e-9);
    // Brute force over the curve for general queries.
    for (x, y, z) in [(3.5, 1.2, 0.3), (-0.2, 0.1, -1.0), (0.0, 0.0, 0.0), (-4.0, -4.0, 2.0), (2.9, 0.05, 0.0)] {
        let p = e.frame.from_local(Vector3::new(x, y, z));
        let pr = e.project(p, None).unwrap();
        let best = (0..40000).map(|i| (e.eval(TAU * i as f64 / 40000.0) - p).norm()).fold(f64::INFINITY, f64::min);
        assert!(pr.distance <= best + 1e-9, "({x},{y},{z}): {} vs {best}", pr.distance);
        assert!(((pr.point - p).norm() - pr.distance).abs() < 1e-12);
    }
    // a = b agrees with the circle.
    let c = Circle::new(tilted(), 2.0).unwrap();
    let ec = Ellipse::new(tilted(), 2.0, 2.0).unwrap();
    let q = Point3::new(1.0, 4.0, -2.0);
    assert!((c.project(q, None).unwrap().distance - ec.project(q, None).unwrap().distance).abs() < 1e-12);
}

#[test]
fn elliptic_cone_projection_is_the_closest_point() {
    let k = EllipticCone::new(tilted(), 2.0, 0.6, 0.5).unwrap();
    let mut queries: Vec<Point3> = [(3.0, 1.0, 0.2), (-0.1, 0.05, 1.0), (0.0, -2.0, -3.0), (0.0, 0.0, 5.0), (1.0, 1.0, -6.0)]
        .iter()
        .map(|&(x, y, z)| k.frame.from_local(Vector3::new(x, y, z)))
        .collect();
    queries.push(k.apex() + k.frame.z * -1.0); // below the apex: closest is the apex
    for p in queries {
        let pr = k.project(p, None).unwrap();
        // Brute force over a fine (u, v) grid on the nappe.
        let mut best = f64::INFINITY;
        for i in 0..720 {
            for j in 0..=600 {
                let uv = Uv::new(TAU * i as f64 / 720.0, k.apex_v() + 12.0 * j as f64 / 600.0);
                best = best.min((k.eval(uv) - p).norm());
            }
        }
        assert!(pr.distance <= best + 1e-9, "{p:?}: {} vs {best}", pr.distance);
        assert!(((pr.point - p).norm() - pr.distance).abs() < 1e-12);
        assert!(pr.uv.v >= k.apex_v());
    }
    // Below the apex along the axis the apex is the answer.
    let pr = k.project(k.apex() + k.frame.z * -1.0, None).unwrap();
    assert!((pr.point - k.apex()).norm() < 1e-12 && (pr.distance - 1.0).abs() < 1e-12);
    // a = b agrees with the circular cone's analytic projection.
    let ec = EllipticCone::new(tilted(), 1.5, 1.5, 0.4).unwrap();
    let cc = Cone::new(tilted(), 1.5, 0.4).unwrap();
    for q in [Point3::new(2.0, 0.5, 1.0), Point3::new(-1.0, 3.0, 2.0), Point3::new(0.4, -0.2, 3.0)] {
        let (a, b) = (ec.project(q, None).unwrap(), cc.project(q, None).unwrap());
        assert!((a.distance - b.distance).abs() < 1e-9, "{q:?}: {} vs {}", a.distance, b.distance);
    }
}

/// Uniform periodic cubic B-spline through 6 unique control points on a hexagon (poles 0..2
/// repeated at the end), knots -3..=6 (spacing 1), domain [0, 6).
fn periodic_curve() -> NurbsCurve {
    let mut ctrl: Vec<Point3> = (0..6).map(|i| { let a = TAU * i as f64 / 6.0; Point3::new(a.cos(), a.sin(), 0.0) }).collect();
    ctrl.extend_from_within(0..3);
    let knots: Vec<f64> = (-3..=9).map(f64::from).collect();
    NurbsCurve::new_periodic(3, knots, ctrl, None).unwrap()
}

#[test]
fn periodic_curve_wraps_and_validates() {
    let c = periodic_curve();
    let d = c.domain();
    assert_eq!(d.period, Some(6.0));
    assert_eq!(d.range, Interval::new(0.0, 6.0));
    for t in [0.0, 0.3, 2.5, 5.99] {
        let (a, b, m) = (c.jet(t), c.jet(t + 6.0), c.jet(t - 12.0));
        assert!((a.p - b.p).norm() < 1e-12 && (a.p - m.p).norm() < 1e-12);
        assert!((a.d1 - b.d1).norm() < 1e-12 && (a.d2 - b.d2).norm() < 1e-12);
    }
    // C2 across the seam: the jet just before 6 matches the jet at 0.
    let (lo, hi) = (c.jet(0.0), c.jet(6.0 - 1e-9));
    assert!((lo.p - hi.p).norm() < 1e-8 && (lo.d1 - hi.d1).norm() < 1e-7);
    // Uniform cubic: the point at t = 0 is (P_0 + 4 P_1 + P_2) / 6 (the curve starts at span 3).
    let q = |i: usize| c.control[i];
    let want = Point3::new((q(0).x + 4.0 * q(1).x + q(2).x) / 6.0, (q(0).y + 4.0 * q(1).y + q(2).y) / 6.0, 0.0);
    assert!((c.eval(0.0) - want).norm() < 1e-12);
    // Projection near the seam picks the wrapped parameter.
    let pr = c.project({ let j = c.jet(5.999); let n = Vector3::new(j.d1.y, -j.d1.x, 0.0); j.p + n * (0.2 / n.norm()) }, Some(0.01)).unwrap();
    assert!((c.eval(pr.t) - c.eval(5.999)).norm() < 1e-6, "t = {}", pr.t);

    // Validation.
    let mut bad = periodic_curve();
    bad.control[7] = Point3::new(9.0, 9.0, 9.0);
    assert!(bad.validate().is_err(), "poles must repeat");
    let mut bad = periodic_curve();
    bad.knots[0] = -3.5;
    assert!(bad.validate().is_err(), "knot spacing must repeat");
    // A clamped closed curve is fine non-periodic and rejected as periodic.
    let clamped = NurbsCurve::new(1, vec![0.0, 0.0, 1.0, 2.0, 2.0], vec![Point3::default(), Point3::new(1.0, 0.0, 0.0), Point3::default()], None).unwrap();
    assert_eq!(clamped.domain().period, None);
    let mut flagged = clamped.clone();
    flagged.periodic = true;
    assert!(flagged.validate().is_err());
    // Linear periodic polygon: knots [0, 1, 2, 3, 4], 3 poles (first repeated).
    let tri = NurbsCurve::new_periodic(1, vec![0.0, 1.0, 2.0, 3.0, 4.0], vec![Point3::default(), Point3::new(1.0, 0.0, 0.0), Point3::default()], None).unwrap();
    assert_eq!(tri.domain().period, Some(2.0));
    assert!((tri.eval(3.5) - tri.eval(1.5)).norm() < 1e-15);
}

#[test]
fn periodic_flag_changes_the_geometry_hash() {
    let c = periodic_curve();
    let tol = ModelTolerance::default();
    let codes = |d: &CurveDef| {
        let mut v = Vec::new();
        d.quantized_values(&tol, |x| v.push(x));
        v
    };
    let p = codes(&CurveDef::Nurbs(c.clone()));
    let mut unflagged = c.clone();
    unflagged.periodic = false;
    let np = codes(&CurveDef::Nurbs(unflagged));
    assert_ne!(p, np);
    assert_eq!(p.len(), np.len());
    // Pcurves too.
    let control2: Vec<Uv> = c.control.iter().map(|q| Uv::new(q.x, q.y)).collect();
    let mk = |periodic: bool| {
        PCurve::Nurbs2(if periodic { NurbsCurve2::new_periodic(3, c.knots.clone(), control2.clone(), None) } else { NurbsCurve2::new(3, c.knots.clone(), control2.clone(), None) }.unwrap())
    };
    let vals = |pc: &PCurve| {
        let mut v = Vec::new();
        pc.for_each_word(|w| v.push(format!("{w:?}")));
        v
    };
    assert_ne!(vals(&mk(true)), vals(&mk(false)));
    let PCurve::Nurbs2(n2) = mk(true) else { unreachable!() };
    n2.validate().unwrap();
    assert!((n2.eval_d1(6.5).0.u - n2.eval_d1(0.5).0.u).abs() < 1e-12);
}

#[test]
fn periodic_surface_wraps_in_u() {
    // Extrude the periodic hexagon spline along z: periodic in u, clamped linear in v.
    let c = periodic_curve();
    let nu = c.control.len();
    let mut control = Vec::new();
    for q in &c.control {
        control.push(*q);
        control.push(*q + Vector3::new(0.0, 0.0, 2.0));
    }
    let s = NurbsSurface::new_periodic(3, 1, c.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], nu, 2, control, None, [true, false]).unwrap();
    s.validate().unwrap();
    let d = s.domain();
    assert_eq!(d.u.period, Some(6.0));
    assert_eq!(d.v.period, None);
    let a = s.jet(Uv::new(0.4, 0.5));
    let b = s.jet(Uv::new(6.4, 0.5));
    assert!((a.p - b.p).norm() < 1e-12 && (a.du - b.du).norm() < 1e-12);
    assert!((a.p - (c.eval(0.4) + Vector3::new(0.0, 0.0, 1.0))).norm() < 1e-12);
    let target = s.eval(Uv::new(5.995, 0.3));
    let pr = s.project(target + s.normal(Uv::new(5.995, 0.3)).unwrap() * 0.1, Some(Uv::new(0.01, 0.3))).unwrap();
    assert!((pr.point - target).norm() < 1e-6, "{pr:?}");
    // Breaking a repeated row is rejected; so is a periodic flag on the clamped v direction.
    let mut bad = s.clone();
    bad.control[2 * (nu - 1)] = Point3::new(5.0, 5.0, 5.0);
    assert!(bad.validate().is_err());
    let mut both = s.clone();
    both.periodic = [true, true];
    assert!(both.validate().is_err());
}
