//! Regressions from the W1 review: Bezier extraction with C^-1 knots, tolerance-aware NURBS
//! contacts, and NURBS-surface hits for circles and ellipses over shifted and multi-period
//! ranges.

use core::f64::consts::{PI, TAU};
use sm_core::{Point3, Vector3};
use sm_geom::exact;
use sm_geom::{CurveSurface, PreparedCurve, PreparedSurface, curve_plane, curve_surface};
use sm_geom_api::{Circle, CurveDef, Ellipse, Frame, Interval, ModelTolerance, NurbsCurve, NurbsSurface, ParametricCurve, ParametricSurface, Plane, Ray, Sphere, SurfaceDef};

fn tol() -> ModelTolerance {
    ModelTolerance::DEFAULT
}

fn ts(r: CurveSurface) -> Vec<f64> {
    match r {
        CurveSurface::Points(p) => p.into_iter().map(|h| h.t).collect(),
        other => panic!("expected points, got {other:?}"),
    }
}

/// A quadratic curve that is discontinuous at t = 0.5 (interior knot of multiplicity p + 1):
/// two separate parabolic arcs, x in [0, 2] and x in [3, 5].
fn broken_curve() -> NurbsCurve {
    let ctrl = [[0.0, 0.0], [1.0, 1.0], [2.0, 0.0], [3.0, 0.0], [4.0, 1.0], [5.0, 0.0]];
    NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0], ctrl.iter().map(|p| Point3::new(p[0], p[1], 0.0)).collect(), None).unwrap()
}

#[test]
fn interior_knot_of_multiplicity_degree_plus_one_curve() {
    let c = broken_curve();
    let pc = PreparedCurve::new(&c).unwrap();
    // Evaluation matches the reference on both arcs.
    for i in 0..=40 {
        let t = i as f64 / 40.0;
        assert!((pc.eval(t) - c.eval(t)).norm() < 1e-12, "t={t}");
    }
    // x = 3.5 is on the second arc only (x = 3 + 4 s ... the arc is x(s) = 3 + 2 s).
    let plane = Plane { frame: Frame::from_z(Point3::new(3.5, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() };
    let hits = ts(curve_plane(&CurveDef::Nurbs(c.clone()), None, &plane, &tol()).unwrap());
    assert_eq!(hits.len(), 1, "{hits:?}");
    assert!((hits[0] - 0.625).abs() < 1e-9, "{hits:?}");
    assert!((pc.eval(hits[0]).x - 3.5).abs() < 1e-9);
    // x = 2.5 falls in the jump: no hit (the old extraction produced a false hit at t = 0.5).
    let gap = Plane { frame: Frame::from_z(Point3::new(2.5, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() };
    assert!(ts(curve_plane(&CurveDef::Nurbs(c), None, &gap, &tol()).unwrap()).is_empty());
}

#[test]
fn interior_knot_of_multiplicity_degree_plus_one_surface() {
    // Degree 1 in u with knots [0,0,.5,.5,1,1]: two unit quads, x in [0,1] and [2,3].
    let mut control = Vec::new();
    for x in [0.0, 1.0, 2.0, 3.0] {
        for y in [0.0, 1.0] {
            control.push(Point3::new(x, y, 0.0));
        }
    }
    let s = NurbsSurface::new(1, 1, vec![0.0, 0.0, 0.5, 0.5, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], 4, 2, control, None).unwrap();
    let ps = PreparedSurface::new(&s).unwrap();
    let ray = |x: f64| Ray { origin: Point3::new(x, 0.5, 1.0), dir: Vector3::new(0.0, 0.0, -1.0) };
    let hits = |x: f64| {
        let mut out = Vec::new();
        ps.ray_hits(&ray(x), Interval::new(0.0, 2.0), &mut out).unwrap();
        out
    };
    let h = hits(2.5);
    assert_eq!(h.len(), 1);
    assert!((h[0].uv.unwrap().u - 0.75).abs() < 1e-9, "{:?}", h[0].uv);
    assert_eq!(hits(0.25).len(), 1);
    assert!(hits(1.5).is_empty(), "the jump between the quads is not part of the surface");
}

#[test]
fn nurbs_curve_just_above_a_plane_is_tangent_within_tolerance() {
    let plane = Plane { frame: Frame::at(Point3::new(0.0, 0.0, 0.0)) };
    let parabola = |h: f64| NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![Point3::new(0.0, 0.0, 1.0 + h), Point3::new(1.0, 0.0, -1.0 + h), Point3::new(2.0, 0.0, 1.0 + h)], None).unwrap();
    let cubic = |h: f64| {
        NurbsCurve::new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], vec![Point3::new(0.0, 0.0, 1.0 + h), Point3::new(1.0, 0.0, -1.0 / 3.0 + h), Point3::new(2.0, 0.0, -1.0 / 3.0 + h), Point3::new(3.0, 0.0, 1.0 + h)], None).unwrap()
    };
    for h in [0.0, 1e-9, 1e-7, 5e-7] {
        for c in [parabola(h), cubic(h)] {
            let r = curve_plane(&CurveDef::Nurbs(c), None, &plane, &tol()).unwrap();
            let CurveSurface::Points(p) = r else { panic!("h={h}: {r:?}") };
            assert_eq!(p.len(), 1, "h={h}: {p:?}");
            assert!(p[0].tangent && (p[0].t - 0.5).abs() < 1e-3, "h={h}: {p:?}");
        }
        // Below the surface by the same amount it is a pair of crossings.
        let r = curve_plane(&CurveDef::Nurbs(parabola(-h.max(1e-3))), None, &plane, &tol()).unwrap();
        assert_eq!(ts(r).len(), 2);
    }
    for c in [parabola(2e-6), cubic(2e-6)] {
        assert!(ts(curve_plane(&CurveDef::Nurbs(c), None, &plane, &tol()).unwrap()).is_empty());
    }
}

/// A long straight cubic through tiny spheres: two hits, not one (the merge window is
/// geometric, not a fraction of the parameter range).
#[test]
fn nurbs_curve_through_a_tiny_sphere_has_two_hits() {
    let line = NurbsCurve::new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], (0..4).map(|i| Point3::new(i as f64 / 3.0, 0.0, 0.0)).collect(), None).unwrap();
    for r in [3e-4, 1e-4, 3e-5, 1e-5] {
        let sphere = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(0.4, 0.0, 0.0)), r).unwrap());
        let h = ts(curve_surface(&CurveDef::Nurbs(line.clone()), None, &sphere, &tol()).unwrap());
        assert_eq!(h.len(), 2, "radius {r}: {h:?}");
        assert!((h[0] - (0.4 - r)).abs() < 1e-7 && (h[1] - (0.4 + r)).abs() < 1e-7, "radius {r}: {h:?}");
    }
    // A sphere smaller than the tolerance is one contact.
    let tiny = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(0.4, 0.0, 0.0)), 1e-7).unwrap());
    assert_eq!(ts(curve_surface(&CurveDef::Nurbs(line), None, &tiny, &tol()).unwrap()).len(), 1);
}

/// Arc length over a huge periodic range is the per-period length times the count, in bounded
/// time.
#[test]
fn periodic_length_over_many_periods_is_analytic_in_the_count() {
    let u = [[0.0, 0.0], [2.0, 0.5], [3.0, 2.0], [1.5, 3.0], [-0.5, 2.0], [-1.0, 0.7]];
    let (m, p) = (u.len(), 3usize);
    let control: Vec<Point3> = (0..m + p).map(|i| Point3::new(u[i % m][0], u[i % m][1], 0.0)).collect();
    let knots: Vec<f64> = (0..m + 2 * p + 1).map(|i| i as f64 - p as f64).collect();
    let c = NurbsCurve::new_periodic(p, knots, control, None).unwrap();
    let pc = PreparedCurve::new(&c).unwrap();
    let one = pc.length(Interval::new(0.0, m as f64), 1e-9).unwrap();
    assert!(one > 5.0);
    let n = 1_000_000.0;
    let many = pc.length(Interval::new(0.25, 0.25 + n * m as f64), 1e-9).unwrap();
    assert!((many - n * one).abs() < 1e-6 * n * one, "{many} vs {}", n * one);
    // A range spanning a fraction of three periods agrees with the sum of its parts.
    let whole = pc.length(Interval::new(-2.5, 2.0 * m as f64 + 1.5), 1e-9).unwrap();
    let parts = pc.length(Interval::new(-2.5, 0.0), 1e-9).unwrap() + 2.0 * one + pc.length(Interval::new(2.0 * m as f64, 2.0 * m as f64 + 1.5), 1e-9).unwrap();
    assert!((whole - parts).abs() < 1e-6, "{whole} vs {parts}");
}

fn sorted(mut v: Vec<f64>) -> Vec<f64> {
    v.sort_by(|a, b| a.total_cmp(b));
    v
}

fn assert_same(a: &[f64], b: &[f64], what: &str) {
    assert_eq!(a.len(), b.len(), "{what}: {a:?} vs {b:?}");
    for (x, y) in a.iter().zip(b) {
        assert!((x - y).abs() < 1e-6, "{what}: {a:?} vs {b:?}");
    }
}

/// `curve_prepared_surface` (through a NURBS surface) agrees with the analytic path over
/// ranges that are shifted, negative and several periods long.
#[test]
fn circle_and_ellipse_against_nurbs_surfaces_match_analytic_over_any_range() {
    let circle = CurveDef::Circle(Circle::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap());
    let ellipse = CurveDef::Ellipse(Ellipse::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.5, 1.0).unwrap());
    let plane = Plane { frame: Frame::from_z(Point3::new(0.5, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() };
    let plane_nurbs = SurfaceDef::Nurbs(exact::plane_to_nurbs(&plane, (-3.0, 3.0), (-3.0, 3.0)).unwrap());
    let sphere = Sphere::new(Frame::at(Point3::new(0.8, 0.0, 0.3)), 1.0).unwrap();
    let sphere_nurbs = SurfaceDef::Nurbs(exact::sphere_to_nurbs(&sphere).unwrap());
    let ranges = [None, Some(Interval::new(-1.5, 1.5)), Some(Interval::new(-PI, PI)), Some(Interval::new(0.0, 2.0 * TAU)), Some(Interval::new(2.5, 9.0)), Some(Interval::new(-7.0, -1.0)), Some(Interval::new(0.3, 0.9))];
    let cases: [(&CurveDef, SurfaceDef, SurfaceDef, &str); 4] = [
        (&circle, SurfaceDef::Plane(plane), plane_nurbs.clone(), "circle/plane"),
        (&ellipse, SurfaceDef::Plane(plane), plane_nurbs, "ellipse/plane"),
        (&circle, SurfaceDef::Sphere(sphere), sphere_nurbs.clone(), "circle/sphere"),
        (&ellipse, SurfaceDef::Sphere(sphere), sphere_nurbs, "ellipse/sphere"),
    ];
    for (curve, analytic, nurbs, name) in cases {
        for r in ranges {
            let a = sorted(ts(curve_surface(curve, r, &analytic, &tol()).unwrap()));
            let b = sorted(ts(curve_surface(curve, r, &nurbs, &tol()).unwrap()));
            assert!(r.is_some() || !a.is_empty(), "{name}: the default range must have hits");
            assert_same(&a, &b, &format!("{name} range {r:?}"));
        }
    }
    // The headline numbers: x = 0.5 over [-1.5, 1.5] has both crossings.
    let r = Some(Interval::new(-1.5, 1.5));
    let nurbs_plane = SurfaceDef::Nurbs(exact::plane_to_nurbs(&plane, (-3.0, 3.0), (-3.0, 3.0)).unwrap());
    assert_eq!(ts(curve_surface(&circle, r, &nurbs_plane, &tol()).unwrap()).len(), 2);
    // Four hits over two periods against the sphere.
    let four = ts(curve_surface(&circle, Some(Interval::new(0.0, 2.0 * TAU)), &sphere_nurbs_def(&sphere), &tol()).unwrap());
    assert_eq!(four.len(), 4, "{four:?}");
}

fn sphere_nurbs_def(s: &Sphere) -> SurfaceDef {
    SurfaceDef::Nurbs(exact::sphere_to_nurbs(s).unwrap())
}
