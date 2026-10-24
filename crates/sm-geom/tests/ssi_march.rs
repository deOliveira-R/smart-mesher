//! Marched surface-surface intersections: every returned point must satisfy both surfaces, loops
//! must close, and lengths must match closed forms.

use core::f64::consts::{FRAC_PI_2, TAU};
use sm_core::{Point3, Vector3};
use sm_geom::exact;
use sm_geom::{CurveGeometry, IntersectionCurve, MarchedCurve, SsiOptions, SsiResult, intersect_surfaces};
use sm_geom_api::{Cylinder, Frame, ImplicitSurface, ModelTolerance, Plane, Sphere, SurfaceDef, Torus};

fn world() -> Frame {
    Frame::at(Point3::new(0.0, 0.0, 0.0))
}

fn opts(w: f64) -> SsiOptions {
    SsiOptions::new(ModelTolerance::DEFAULT).with_window([-w, -w, -w, w, w, w])
}

fn marched(r: SsiResult) -> Vec<MarchedCurve> {
    match r {
        SsiResult::Curves(cs) => cs
            .into_iter()
            .map(|c: IntersectionCurve| match c.geometry {
                CurveGeometry::Marched(m) => m,
                other => panic!("expected marched, got {other:?}"),
            })
            .collect(),
        other => panic!("expected curves, got {other:?}"),
    }
}

fn dist(s: &SurfaceDef, p: Point3) -> f64 {
    let f = s.implicit_def().expect("implicit");
    f.value(p).abs() / f.gradient(p).norm().max(1e-300)
}

fn length(m: &MarchedCurve) -> f64 {
    m.points.windows(2).map(|w| (w[1] - w[0]).norm()).sum()
}

fn check_on_both(ms: &[MarchedCurve], a: &SurfaceDef, b: &SurfaceDef, tol: f64) {
    for m in ms {
        assert!(m.points.len() >= 3);
        for p in &m.points {
            assert!(dist(a, *p) < tol && dist(b, *p) < tol, "{p:?}: {} {}", dist(a, *p), dist(b, *p));
        }
    }
}

fn cyl(frame: Frame, r: f64) -> SurfaceDef {
    SurfaceDef::Cylinder(Cylinder::new(frame, r).unwrap())
}

fn x_axis_frame() -> Frame {
    Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap()
}

#[test]
fn perpendicular_cylinders_unequal_radii_give_two_closed_loops() {
    let a = cyl(world(), 1.0);
    let b = cyl(x_axis_frame(), 0.6);
    let ms = marched(intersect_surfaces(&a, &b, &opts(3.0).with_chord_tol(2e-5)).unwrap());
    assert_eq!(ms.len(), 2, "two loops: {}", ms.len());
    check_on_both(&ms, &a, &b, 1e-8);
    assert!(ms.iter().all(|m| m.closed), "both loops close");
    // Exact length of x = +-sqrt(1 - 0.36 cos^2 t), y = 0.6 cos t, z = 0.6 sin t by quadrature.
    let n = 20000;
    let mut exact_len = 0.0;
    for i in 0..n {
        let t = TAU * (i as f64 + 0.5) / n as f64;
        let (s, c) = t.sin_cos();
        let x = (1.0 - 0.36 * c * c).sqrt();
        let dx = 0.36 * c * s / x;
        exact_len += (dx * dx + 0.36 * s * s + 0.36 * c * c).sqrt() * TAU / n as f64;
    }
    for m in &ms {
        let l = length(m);
        assert!((l - exact_len).abs() / exact_len < 2e-5, "loop length {l} vs {exact_len}");
    }
}

#[test]
fn steinmetz_equal_radii_branches_end_at_the_crossings() {
    let a = cyl(world(), 1.0);
    let b = cyl(x_axis_frame(), 1.0);
    let ms = marched(intersect_surfaces(&a, &b, &opts(3.0)).unwrap());
    check_on_both(&ms, &a, &b, 1e-8);
    // The union of the branches covers the two ellipses x = +-z: sample the exact curve.
    let mut covered = 0;
    let total = 400;
    for i in 0..total {
        let t = TAU * (i as f64 + 0.37) / total as f64;
        for sgn in [1.0, -1.0] {
            // On the plane x = sgn * z: (cos t, sin t, sgn cos t).
            let p = Point3::new(t.cos(), t.sin(), sgn * t.cos());
            // Skip the singular crossing neighbourhood (0, +-1, 0).
            if t.cos().abs() < 0.05 {
                continue;
            }
            let near = ms.iter().any(|m| m.points.windows(2).any(|w| {
                let ab = w[1] - w[0];
                let tt = ((p - w[0]).dot(ab) / ab.dot(ab).max(1e-300)).clamp(0.0, 1.0);
                (p - (w[0] + ab * tt)).norm() < 2e-3
            }));
            if near {
                covered += 1;
            } else {
                panic!("exact point {p:?} (t={t}) not covered by any branch");
            }
        }
    }
    assert!(covered > 600);
}

#[test]
fn oblique_plane_torus_gives_closed_loops() {
    let tor = SurfaceDef::Torus(Torus::new(world(), 3.0, 1.0).unwrap());
    // Tilted plane through the centre: z = 0.2 x -> a Villarceau-like oval pair.
    let p = SurfaceDef::Plane(Plane { frame: Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vector3::new(-0.2, 0.0, 1.0)).unwrap() });
    let ms = marched(intersect_surfaces(&p, &tor, &opts(6.0)).unwrap());
    check_on_both(&ms, &p, &tor, 1e-8);
    assert!(ms.iter().all(|m| m.closed), "{:?}", ms.iter().map(|m| (m.closed, m.start, m.end, m.points.len())).collect::<Vec<_>>());
    assert_eq!(ms.len(), 2, "outer and inner loop");
    // Plane z = 0 handled in closed form; the tilted cut's inner loop is shorter than the outer.
    let (l0, l1) = (length(&ms[0]), length(&ms[1]));
    assert!(l0.min(l1) > 2.0 * core::f64::consts::PI * 2.0 * 0.95 && l0.max(l1) < 2.0 * core::f64::consts::PI * 4.2, "{l0} {l1}");
}

#[test]
fn sphere_cylinder_offset_axis() {
    let s = SurfaceDef::Sphere(Sphere::new(world(), 2.0).unwrap());
    let c = cyl(Frame::at(Point3::new(0.9, 0.0, 0.0)), 0.8);
    let ms = marched(intersect_surfaces(&s, &c, &opts(4.0)).unwrap());
    check_on_both(&ms, &s, &c, 1e-8);
    assert_eq!(ms.len(), 2, "upper and lower closed curves");
    assert!(ms.iter().all(|m| m.closed));
}

#[test]
fn nurbs_sphere_with_plane_and_nurbs_pair() {
    let sph = Sphere::new(world(), 2.0).unwrap();
    let ns = SurfaceDef::Nurbs(exact::sphere_to_nurbs(&sph).unwrap());
    let p = SurfaceDef::Plane(Plane { frame: Frame::from_z(Point3::new(0.0, 0.0, 0.5), Vector3::new(0.3, 0.1, 1.0)).unwrap() });
    let ms = marched(intersect_surfaces(&ns, &p, &opts(4.0)).unwrap());
    assert_eq!(ms.len(), 1);
    assert!(ms[0].closed);
    // Compare with the exact sphere/plane circle: radius sqrt(4 - d^2), d = distance of centre.
    let nrm = Vector3::new(0.3, 0.1, 1.0);
    let nrm = nrm * (1.0 / nrm.norm());
    let d = nrm.dot(Vector3::new(0.0, 0.0, 0.5));
    let rho = (4.0 - d * d).sqrt();
    assert!((length(&ms[0]) - TAU * rho).abs() / (TAU * rho) < 1e-4, "{} vs {}", length(&ms[0]), TAU * rho);
    let sphere_def = SurfaceDef::Sphere(sph);
    check_on_both(&ms, &sphere_def, &p, 1e-7);
    // NURBS sphere against NURBS cylinder (both rational): points must satisfy the analytic forms.
    let cy = Cylinder::new(Frame::at(Point3::new(0.9, 0.0, 0.0)), 0.8).unwrap();
    let nc = SurfaceDef::Nurbs(exact::cylinder_to_nurbs(&cy, -3.0, 3.0).unwrap());
    let ms = marched(intersect_surfaces(&ns, &nc, &opts(4.0)).unwrap());
    check_on_both(&ms, &sphere_def, &SurfaceDef::Cylinder(cy), 1e-7);
    assert_eq!(ms.len(), 2);
    assert!(ms.iter().all(|m| !m.uv_a.is_empty() && !m.uv_b.is_empty()));
    let _ = FRAC_PI_2;
}

#[test]
fn deterministic_output() {
    let a = cyl(world(), 1.0);
    let b = cyl(x_axis_frame(), 0.6);
    let r1 = intersect_surfaces(&a, &b, &opts(3.0)).unwrap();
    let r2 = intersect_surfaces(&a, &b, &opts(3.0)).unwrap();
    assert_eq!(r1, r2);
}
