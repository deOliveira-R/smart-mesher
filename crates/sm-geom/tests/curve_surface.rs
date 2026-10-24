//! Curve-plane and curve-surface intersections against hand-computed values.

use core::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI, TAU};
use sm_core::{Point3, Vector3};
use sm_geom::exact;
use sm_geom::{CurveSurface, CurveSurfaceHit, curve_plane, curve_surface};
use sm_geom_api::{Circle, Cone, CurveDef, Cylinder, Ellipse, Frame, Interval, Line, ModelTolerance, NurbsCurve, Plane, Sphere, SurfaceDef, Torus};

fn tol() -> ModelTolerance {
    ModelTolerance::DEFAULT
}

fn pts(r: CurveSurface) -> Vec<CurveSurfaceHit> {
    match r {
        CurveSurface::Points(p) => p,
        other => panic!("expected points, got {other:?}"),
    }
}

fn world() -> Frame {
    Frame::at(Point3::new(0.0, 0.0, 0.0))
}

#[test]
fn line_plane() {
    let plane = Plane { frame: Frame::at(Point3::new(0.0, 0.0, 2.0)) };
    let line = CurveDef::Line(Line::new(Point3::new(1.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 2.0)).unwrap());
    let h = pts(curve_plane(&line, None, &plane, &tol()).unwrap());
    assert_eq!(h.len(), 1);
    assert!((h[0].t - 2.0).abs() < 1e-14 && (h[0].point - Point3::new(1.0, 1.0, 2.0)).norm() < 1e-14);
    assert!(!h[0].tangent);
    // Range excludes it.
    assert!(pts(curve_plane(&line, Some(Interval::new(0.0, 1.0)), &plane, &tol()).unwrap()).is_empty());
    // Parallel and in-plane.
    let par = CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 2.0), Vector3::new(1.0, 0.0, 0.0)).unwrap());
    assert_eq!(curve_plane(&par, None, &plane, &tol()).unwrap(), CurveSurface::Coincident);
    let off = CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 3.0), Vector3::new(1.0, 0.0, 0.0)).unwrap());
    assert!(pts(curve_plane(&off, None, &plane, &tol()).unwrap()).is_empty());
}

#[test]
fn circle_and_ellipse_plane() {
    // Unit circle in z = 0 against the plane x = 0.5: angles +-60 degrees.
    let c = CurveDef::Circle(Circle::new(world(), 1.0).unwrap());
    let plane = Plane { frame: Frame::from_z(Point3::new(0.5, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() };
    let h = pts(curve_plane(&c, None, &plane, &tol()).unwrap());
    assert_eq!(h.len(), 2);
    assert!((h[0].t - FRAC_PI_3).abs() < 1e-12 && (h[1].t - (TAU - FRAC_PI_3)).abs() < 1e-12, "{h:?}");
    assert!(h.iter().all(|x| (x.point.x - 0.5).abs() < 1e-14));
    // Tangent plane x = 1.
    let tp = Plane { frame: Frame::from_z(Point3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() };
    let h = pts(curve_plane(&c, None, &tp, &tol()).unwrap());
    assert_eq!(h.len(), 1);
    assert!(h[0].tangent && h[0].t.abs() < 1e-6);
    // Missed.
    let far = Plane { frame: Frame::from_z(Point3::new(2.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() };
    assert!(pts(curve_plane(&c, None, &far, &tol()).unwrap()).is_empty());
    // Ellipse a=2, b=1 against y = 0.5: sin t = 0.5 -> t = pi/6, 5 pi/6.
    let e = CurveDef::Ellipse(Ellipse::new(world(), 2.0, 1.0).unwrap());
    let py = Plane { frame: Frame::from_z(Point3::new(0.0, 0.5, 0.0), Vector3::new(0.0, 1.0, 0.0)).unwrap() };
    let h = pts(curve_plane(&e, None, &py, &tol()).unwrap());
    assert_eq!(h.len(), 2);
    assert!((h[0].t - PI / 6.0).abs() < 1e-12 && (h[1].t - 5.0 * PI / 6.0).abs() < 1e-12);
    // A circle in the plane itself is coincident.
    let pz = Plane { frame: world() };
    assert_eq!(curve_plane(&c, None, &pz, &tol()).unwrap(), CurveSurface::Coincident);
}

#[test]
fn nurbs_curve_plane() {
    // Rational quarter circle: x = cos, plane x = 0.6 -> y = 0.8.
    let w = FRAC_PI_2.cos().max(0.0) + core::f64::consts::FRAC_1_SQRT_2;
    let arc = NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)], Some(vec![1.0, w, 1.0])).unwrap();
    let plane = Plane { frame: Frame::from_z(Point3::new(0.6, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap() };
    let h = pts(curve_plane(&CurveDef::Nurbs(arc), None, &plane, &tol()).unwrap());
    assert_eq!(h.len(), 1);
    assert!((h[0].point - Point3::new(0.6, 0.8, 0.0)).norm() < 1e-12, "{:?}", h[0]);
    // The full NURBS circle against y = 0: hits at angle 0 (seam, once) and pi.
    let full = exact::circle_to_nurbs(&Circle::new(world(), 2.0).unwrap()).unwrap();
    let py = Plane { frame: Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)).unwrap() };
    let h = pts(curve_plane(&CurveDef::Nurbs(full), None, &py, &tol()).unwrap());
    let xs: Vec<f64> = h.iter().map(|x| x.point.x).collect();
    assert_eq!(h.len(), 2, "the seam is one hit, not the start and the end of the knot range: {xs:?}");
    assert!(h.iter().all(|x| !x.tangent), "{h:?}");
    // Tangent: plane y = 2.
    let full = exact::circle_to_nurbs(&Circle::new(world(), 2.0).unwrap()).unwrap();
    let pt = Plane { frame: Frame::from_z(Point3::new(0.0, 2.0, 0.0), Vector3::new(0.0, 1.0, 0.0)).unwrap() };
    let h = pts(curve_plane(&CurveDef::Nurbs(full), None, &pt, &tol()).unwrap());
    assert_eq!(h.len(), 1, "{h:?}");
    assert!(h[0].tangent);
}

#[test]
fn line_against_quadrics_and_torus() {
    let sph = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(0.0, 0.0, 10.0)), 2.0).unwrap());
    let l = CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap());
    let h = pts(curve_surface(&l, None, &sph, &tol()).unwrap());
    assert_eq!(h.len(), 2);
    assert!((h[0].t - 8.0).abs() < 1e-12 && (h[1].t - 12.0).abs() < 1e-12);
    assert!(h[0].uv.is_some());
    // Torus R=3, r=1 along x: t = 2, 4, -4, -2 relative to a line through the centre.
    let tor = SurfaceDef::Torus(Torus::new(world(), 3.0, 1.0).unwrap());
    let l = CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap());
    let h = pts(curve_surface(&l, None, &tor, &tol()).unwrap());
    let ts: Vec<f64> = h.iter().map(|x| x.t).collect();
    assert_eq!(ts.len(), 4);
    for (a, b) in ts.iter().zip([-4.0, -2.0, 2.0, 4.0]) {
        assert!((a - b).abs() < 1e-10, "{ts:?}");
    }
}

#[test]
fn circle_against_sphere_cylinder_torus_cone() {
    // Unit circle in z=0 vs sphere centre (1,0,0) radius 1: |p - c|^2 = 2 - 2cos t = 1 -> t = +-60 deg.
    let c = CurveDef::Circle(Circle::new(world(), 1.0).unwrap());
    let sph = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(1.0, 0.0, 0.0)), 1.0).unwrap());
    let h = pts(curve_surface(&c, None, &sph, &tol()).unwrap());
    assert_eq!(h.len(), 2);
    assert!((h[0].t - FRAC_PI_3).abs() < 1e-9 && (h[1].t - (TAU - FRAC_PI_3)).abs() < 1e-9, "{h:?}");
    // Cylinder about the x axis shifted: unit circle vs cylinder radius 0.5 about z axis at x=1:
    // a cylinder with axis z centred at (1,0): distance^2 to axis = 2 - 2cos t = 0.25.
    let cyl = SurfaceDef::Cylinder(Cylinder::new(Frame::at(Point3::new(1.0, 0.0, 0.0)), 0.5).unwrap());
    let h = pts(curve_surface(&c, None, &cyl, &tol()).unwrap());
    assert_eq!(h.len(), 2);
    let want = (1.0f64 - 0.125).acos();
    assert!((h[0].t - want).abs() < 1e-9, "{h:?}");
    // Circle radius 3 in the plane z = 0 against the torus R=3, r=1: points with rho=3 (the
    // tube centre circle): the whole circle is inside the tube (g<0 everywhere): no hits.
    let tor = SurfaceDef::Torus(Torus::new(world(), 3.0, 1.0).unwrap());
    let c3 = CurveDef::Circle(Circle::new(world(), 3.0).unwrap());
    assert!(pts(curve_surface(&c3, None, &tor, &tol()).unwrap()).is_empty());
    // A circle of radius 3 centred off-axis hits the torus four times.
    let off = CurveDef::Circle(Circle::new(Frame::at(Point3::new(3.0, 0.0, 0.0)), 3.0).unwrap());
    let h = pts(curve_surface(&off, None, &tor, &tol()).unwrap());
    assert!(!h.is_empty() && h.len().is_multiple_of(2), "{h:?}");
    for x in &h {
        let l = x.point;
        let rho = l.x.hypot(l.y);
        assert!(((rho - 3.0).powi(2) + l.z * l.z - 1.0).abs() < 1e-9, "{x:?}");
    }
    // Ellipse vs cone: points must lie on the nappe only.
    let cone = SurfaceDef::Cone(Cone::new(world(), 0.0, FRAC_PI_3 / 2.0).unwrap());
    let e = CurveDef::Ellipse(Ellipse::new(Frame::at(Point3::new(0.0, 0.0, 1.0)), 2.0, 1.0).unwrap());
    let h = pts(curve_surface(&e, None, &cone, &tol()).unwrap());
    assert!(h.iter().all(|x| x.point.z > 0.0));
    let below = CurveDef::Ellipse(Ellipse::new(Frame::at(Point3::new(0.0, 0.0, -1.0)), 2.0, 1.0).unwrap());
    assert!(pts(curve_surface(&below, None, &cone, &tol()).unwrap()).is_empty(), "other nappe is not part of the parametric cone");
}

#[test]
fn nurbs_curve_against_implicit_and_nurbs_surfaces() {
    // NURBS circle (radius 2) vs sphere centre (2,0,0) radius 2: angle where |p-c| = 2:
    // 8 - 8 cos t = 4 -> t = +-60 deg.
    let nc = CurveDef::Nurbs(exact::circle_to_nurbs(&Circle::new(world(), 2.0).unwrap()).unwrap());
    let sph = SurfaceDef::Sphere(Sphere::new(Frame::at(Point3::new(2.0, 0.0, 0.0)), 2.0).unwrap());
    let h = pts(curve_surface(&nc, None, &sph, &tol()).unwrap());
    assert_eq!(h.len(), 2, "{h:?}");
    for x in &h {
        assert!(((x.point - Point3::new(2.0, 0.0, 0.0)).norm() - 2.0).abs() < 1e-9);
        assert!((x.point.x - 1.0).abs() < 1e-9);
    }
    // The same sphere as a NURBS surface: hits agree.
    let ns = SurfaceDef::Nurbs(exact::sphere_to_nurbs(&Sphere::new(Frame::at(Point3::new(2.0, 0.0, 0.0)), 2.0).unwrap()).unwrap());
    let h2 = pts(curve_surface(&nc, None, &ns, &tol()).unwrap());
    assert_eq!(h2.len(), 2, "{h2:?}");
    for (a, b) in h.iter().zip(h2.iter()) {
        assert!((a.point - b.point).norm() < 1e-7, "{a:?} {b:?}");
    }
    // Analytic circle vs the NURBS sphere maps the parameter back to the analytic angle.
    let c = CurveDef::Circle(Circle::new(world(), 2.0).unwrap());
    let h3 = pts(curve_surface(&c, None, &ns, &tol()).unwrap());
    assert_eq!(h3.len(), 2);
    assert!((h3[0].t - FRAC_PI_3).abs() < 1e-7 && (h3[1].t - (TAU - FRAC_PI_3)).abs() < 1e-7, "{h3:?}");
    // A line through a NURBS torus crosses it four times.
    let nt = SurfaceDef::Nurbs(exact::torus_to_nurbs(&Torus::new(world(), 3.0, 1.0).unwrap()).unwrap());
    let l = CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap());
    let h = pts(curve_surface(&l, None, &nt, &tol()).unwrap());
    let ts: Vec<f64> = h.iter().map(|x| x.t).collect();
    assert_eq!(ts.len(), 4, "{ts:?}");
    for (a, b) in ts.iter().zip([-4.0, -2.0, 2.0, 4.0]) {
        assert!((a - b).abs() < 1e-7, "{ts:?}");
    }
}
