//! Closed-form surface-surface intersections, checked against analytic values and by
//! evaluating both surfaces' implicit forms on the returned curves.

use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, TAU};
use sm_core::{Point3, Vector3};
use sm_geom::{CurveGeometry, IntersectionCurve, SsiOptions, SsiResult, intersect_surfaces};
use sm_geom_api::{Circle, Cone, CurveDef, Cylinder, EllipticCylinder, Frame, ImplicitSurface, Interval, ModelTolerance, Plane, Quadric, Sphere, SurfaceDef, Torus};

fn world() -> Frame {
    Frame::at(Point3::new(0.0, 0.0, 0.0))
}

fn plane(o: Point3, n: Vector3) -> SurfaceDef {
    SurfaceDef::Plane(Plane { frame: Frame::from_z(o, n).unwrap() })
}

fn opts() -> SsiOptions {
    SsiOptions::new(ModelTolerance::DEFAULT)
}

fn curves(r: SsiResult) -> Vec<IntersectionCurve> {
    match r {
        SsiResult::Curves(c) => c,
        other => panic!("expected curves, got {other:?}"),
    }
}

/// Distance of `p` to the surface through its implicit form (first-order).
fn dist(s: &SurfaceDef, p: Point3) -> f64 {
    let f = s.implicit_def().expect("implicit form");
    f.value(p).abs() / f.gradient(p).norm().max(1e-300)
}

fn sample(c: &IntersectionCurve) -> Vec<Point3> {
    match &c.geometry {
        CurveGeometry::Analytic { curve, range } => {
            let r = Interval::new(range.lo.max(-6.0), range.hi.min(6.0));
            let r = if curve.as_curve().domain().is_periodic() { *range } else { r };
            (0..=40).map(|i| curve.as_curve().eval(r.lo + r.len() * i as f64 / 40.0)).collect()
        }
        CurveGeometry::Marched(m) => m.points.clone(),
        _ => unreachable!(),
    }
}

fn assert_on_both(cs: &[IntersectionCurve], a: &SurfaceDef, b: &SurfaceDef, tol: f64) {
    for c in cs {
        for p in sample(c) {
            assert!(dist(a, p) < tol && dist(b, p) < tol, "{p:?}: {} {}", dist(a, p), dist(b, p));
        }
    }
}

#[test]
fn plane_plane_cases() {
    let a = plane(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let b = plane(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
    let cs = curves(intersect_surfaces(&a, &b, &opts()).unwrap());
    assert_eq!(cs.len(), 1);
    match &cs[0].geometry {
        CurveGeometry::Analytic { curve: CurveDef::Line(l), .. } => {
            assert!(l.dir.cross(Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-14);
            assert!(Vector3::new(l.origin.x, l.origin.y, 0.0).norm() < 1e-14);
        }
        other => panic!("{other:?}"),
    }
    assert_on_both(&cs, &a, &b, 1e-13);
    // Oblique planes: x + y = 1 and y + z = 1 meet along (t, 1 - t, t)... check membership.
    let c = plane(Point3::new(0.5, 0.5, 0.0), Vector3::new(1.0, 1.0, 0.0));
    let d = plane(Point3::new(0.0, 0.5, 0.5), Vector3::new(0.0, 1.0, 1.0));
    assert_on_both(&curves(intersect_surfaces(&c, &d, &opts()).unwrap()), &c, &d, 1e-13);
    // Parallel and coincident.
    let e = plane(Point3::new(1.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(intersect_surfaces(&a, &e, &opts()).unwrap(), SsiResult::Empty);
    let f = plane(Point3::new(0.0, 5.0, 3.0), Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(intersect_surfaces(&a, &f, &opts()).unwrap(), SsiResult::Coincident);
    // Window clipping of the line.
    let w = opts().with_window([-1.0, -1.0, -2.0, 1.0, 1.0, 3.0]);
    let cs = curves(intersect_surfaces(&a, &b, &w).unwrap());
    match &cs[0].geometry {
        CurveGeometry::Analytic { range, .. } => assert!((range.len() - 5.0).abs() < 1e-5, "{range:?}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn plane_sphere() {
    let s = SurfaceDef::Sphere(Sphere::new(world(), 2.0).unwrap());
    let p = plane(Point3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0));
    let cs = curves(intersect_surfaces(&p, &s, &opts()).unwrap());
    match &cs[0].geometry {
        CurveGeometry::Analytic { curve: CurveDef::Circle(c), .. } => {
            assert!((c.radius - 3.0f64.sqrt()).abs() < 1e-14 && (c.frame.origin.z - 1.0).abs() < 1e-14);
        }
        other => panic!("{other:?}"),
    }
    assert!(cs[0].closed);
    assert_on_both(&cs, &p, &s, 1e-13);
    // Order of arguments does not matter.
    assert_eq!(curves(intersect_surfaces(&s, &p, &opts()).unwrap()).len(), 1);
    let tangent = plane(Point3::new(0.0, 0.0, 2.0), Vector3::new(0.0, 0.0, 1.0));
    match intersect_surfaces(&tangent, &s, &opts()).unwrap() {
        SsiResult::Point(q) => assert!((q - Point3::new(0.0, 0.0, 2.0)).norm() < 1e-6),
        other => panic!("{other:?}"),
    }
    let far = plane(Point3::new(0.0, 0.0, 3.0), Vector3::new(0.0, 0.0, 1.0));
    assert_eq!(intersect_surfaces(&far, &s, &opts()).unwrap(), SsiResult::Empty);
}

#[test]
fn plane_cylinder_circle_ellipse_lines() {
    let cyl = SurfaceDef::Cylinder(Cylinder::new(world(), 1.5).unwrap());
    // Perpendicular: circle.
    let p = plane(Point3::new(0.0, 0.0, 0.7), Vector3::new(0.0, 0.0, 1.0));
    let cs = curves(intersect_surfaces(&p, &cyl, &opts()).unwrap());
    assert!(matches!(&cs[0].geometry, CurveGeometry::Analytic { curve: CurveDef::Circle(c), .. } if (c.radius - 1.5).abs() < 1e-13));
    // 45 degrees: ellipse a = r sqrt 2, b = r.
    let n = Vector3::new(1.0, 0.0, 1.0);
    let p = plane(Point3::new(0.0, 0.0, 0.0), n);
    let cs = curves(intersect_surfaces(&p, &cyl, &opts()).unwrap());
    match &cs[0].geometry {
        CurveGeometry::Analytic { curve: CurveDef::Ellipse(e), .. } => {
            let (mut a, mut b) = (e.a, e.b);
            if a < b {
                core::mem::swap(&mut a, &mut b);
            }
            assert!((a - 1.5 * 2.0f64.sqrt()).abs() < 1e-12 && (b - 1.5).abs() < 1e-12, "{a} {b}");
        }
        other => panic!("{other:?}"),
    }
    assert_on_both(&cs, &p, &cyl, 1e-12);
    // Parallel to the axis: two lines at x = +-0.9 (circle x^2 + y^2 = r^2 -> y = +-sqrt(r^2 - 0.81)).
    let p = plane(Point3::new(0.9, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let cs = curves(intersect_surfaces(&p, &cyl, &opts()).unwrap());
    assert_eq!(cs.len(), 2);
    assert_on_both(&cs, &p, &cyl, 1e-12);
    // Tangent plane: one line.
    let p = plane(Point3::new(1.5, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let cs = curves(intersect_surfaces(&p, &cyl, &opts()).unwrap());
    assert_eq!(cs.len(), 1);
    // Missed.
    let p = plane(Point3::new(2.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    assert_eq!(intersect_surfaces(&p, &cyl, &opts()).unwrap(), SsiResult::Empty);
}

#[test]
fn plane_cone_all_conics() {
    let h = 0.5; // half angle 0.5 rad
    let cone = SurfaceDef::Cone(Cone::new(world(), 0.0, h).unwrap());
    let w = opts().with_window([-8.0, -8.0, -1.0, 8.0, 8.0, 8.0]);
    // Ellipse: plane z = 2 (circle).
    let p = plane(Point3::new(0.0, 0.0, 2.0), Vector3::new(0.0, 0.0, 1.0));
    let cs = curves(intersect_surfaces(&p, &cone, &w).unwrap());
    assert!(matches!(&cs[0].geometry, CurveGeometry::Analytic { curve: CurveDef::Circle(c), .. } if (c.radius - 2.0 * h.tan()).abs() < 1e-12));
    // Same plane below the apex hits only the other nappe: empty for the parametric cone.
    let below = plane(Point3::new(0.0, 0.0, -2.0), Vector3::new(0.0, 0.0, 1.0));
    assert_eq!(intersect_surfaces(&below, &cone, &w).unwrap(), SsiResult::Empty);
    // Tilted ellipse.
    let tilted = plane(Point3::new(0.0, 0.0, 3.0), Vector3::new(0.3, 0.0, 1.0));
    let cs = curves(intersect_surfaces(&tilted, &cone, &w).unwrap());
    assert!(matches!(&cs[0].geometry, CurveGeometry::Analytic { curve: CurveDef::Ellipse(_), .. }));
    assert_on_both(&cs, &tilted, &cone, 1e-10);
    // Hyperbola: a plane x = 1 (parallel to the axis) cuts one branch on the nappe.
    let hyper = plane(Point3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let cs = curves(intersect_surfaces(&hyper, &cone, &w).unwrap());
    assert_eq!(cs.len(), 1, "one branch lies on the nappe");
    assert_on_both(&cs, &hyper, &cone, 1e-9);
    for p in sample(&cs[0]) {
        assert!(p.z > 0.0, "nappe: {p:?}");
    }
    // Parabola: plane parallel to a generator. Generator direction (sin h, 0, cos h); normal
    // perpendicular to it and to y: (cos h, 0, -sin h).
    let par = plane(Point3::new(0.0, 0.0, 1.0), Vector3::new(h.cos(), 0.0, -h.sin()));
    let cs = curves(intersect_surfaces(&par, &cone, &w).unwrap());
    assert_eq!(cs.len(), 1);
    assert_on_both(&cs, &par, &cone, 1e-8);
    // Through the apex: two generator rays.
    let thru = plane(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let cs = curves(intersect_surfaces(&thru, &cone, &w).unwrap());
    assert_eq!(cs.len(), 2);
    for c in &cs {
        match &c.geometry {
            CurveGeometry::Analytic { range, curve: CurveDef::Line(l) } => {
                assert_eq!(range.lo, 0.0);
                assert!(l.dir.z > 0.0);
            }
            other => panic!("{other:?}"),
        }
    }
    // An open conic without a window is an error, not a guess.
    assert!(intersect_surfaces(&hyper, &cone, &opts()).is_err());
}

#[test]
fn plane_torus_symmetric_cuts_and_sphere_sphere_and_quadrics() {
    let tor = SurfaceDef::Torus(Torus::new(world(), 3.0, 1.0).unwrap());
    // z = 0.6: circles of radius 3 +- 0.8.
    let p = plane(Point3::new(0.0, 0.0, 0.6), Vector3::new(0.0, 0.0, 1.0));
    let cs = curves(intersect_surfaces(&p, &tor, &opts()).unwrap());
    let mut rs: Vec<f64> = cs
        .iter()
        .map(|c| match &c.geometry {
            CurveGeometry::Analytic { curve: CurveDef::Circle(c), .. } => c.radius,
            other => panic!("{other:?}"),
        })
        .collect();
    rs.sort_by(f64::total_cmp);
    assert!((rs[0] - 2.2).abs() < 1e-12 && (rs[1] - 3.8).abs() < 1e-12);
    assert_on_both(&cs, &p, &tor, 1e-10);
    // Through the axis (y = 0): two circles of radius 1 at x = +-3.
    let p = plane(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0));
    let cs = curves(intersect_surfaces(&p, &tor, &opts()).unwrap());
    assert_eq!(cs.len(), 2);
    assert_on_both(&cs, &p, &tor, 1e-10);
    // Sphere-sphere: centres 0 and 2 on x, radii 2 and 2: circle at x = 1 radius sqrt 3.
    let a = SurfaceDef::Sphere(Sphere::new(world(), 2.0).unwrap());
    let b = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(2.0, 0.0, 0.0)), 2.0).unwrap());
    let cs = curves(intersect_surfaces(&a, &b, &opts()).unwrap());
    assert!(matches!(&cs[0].geometry, CurveGeometry::Analytic { curve: CurveDef::Circle(c), .. } if (c.radius - 3.0f64.sqrt()).abs() < 1e-13 && (c.frame.origin.x - 1.0).abs() < 1e-13));
    assert_on_both(&cs, &a, &b, 1e-12);
    // Elliptic cylinder and a general quadric (ellipsoid) against planes.
    let ec = SurfaceDef::EllipticCylinder(EllipticCylinder::new(world(), 2.0, 1.0).unwrap());
    let p = plane(Point3::new(0.0, 0.0, 0.3), Vector3::new(0.0, 0.0, 1.0));
    let cs = curves(intersect_surfaces(&p, &ec, &opts()).unwrap());
    assert!(matches!(&cs[0].geometry, CurveGeometry::Analytic { curve: CurveDef::Ellipse(e), .. } if (e.a.max(e.b) - 2.0).abs() < 1e-12 && (e.a.min(e.b) - 1.0).abs() < 1e-12));
    let ellipsoid = SurfaceDef::Implicit(sm_geom_api::ImplicitDef::Quadric(Quadric::from_local(&world(), [0.25, 1.0, 4.0], [0.0; 3], -1.0)));
    let p = plane(Point3::new(0.0, 0.0, 0.2), Vector3::new(0.2, 0.1, 1.0));
    let cs = curves(intersect_surfaces(&p, &ellipsoid, &opts()).unwrap());
    assert_on_both(&cs, &p, &ellipsoid, 1e-10);
    // Quarter-turn and full-turn parameter sanity for circles: range is the full period.
    for c in &cs {
        if let CurveGeometry::Analytic { range, .. } = &c.geometry {
            assert!((range.len() - TAU).abs() < 1e-14);
        }
    }
    let _ = (FRAC_PI_2, FRAC_PI_4, Circle::new(world(), 1.0).unwrap());
}
