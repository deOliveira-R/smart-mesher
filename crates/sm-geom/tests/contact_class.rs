//! Contact classification is path independent: a circle or ellipse touching a sphere, cylinder,
//! torus or plane gives the same topology (hit count and `tangent` flag) on every route,
//! wherever the contact sits on either parametrization: at the closed curve's seam, at Bezier
//! knots, at seam +- 1e-9, or at a random angle, under random rigid motions and in windows that
//! run across the seam.

use core::f64::consts::{FRAC_PI_2, PI, TAU};
use proptest::prelude::*;
use sm_core::{Point3, Transform, Vector3};
use sm_geom::exact;
use sm_geom::{CurveSurface, CurveSurfaceHit, curve_surface};
use sm_geom_api::{Circle, Cylinder, CurveDef, Ellipse, Frame, Interval, ModelTolerance, Plane, Sphere, SurfaceDef, Torus};
use sm_testkit::strategies::{config, proper_transform};

struct Scene {
    name: String,
    c: CurveDef,
    cn: CurveDef,
    s: SurfaceDef,
    sn: SurfaceDef,
    /// Curve parameter of the (nominal) contact and its point.
    t0: f64,
    p0: Point3,
}

fn hits(c: &CurveDef, r: Option<Interval>, s: &SurfaceDef) -> Vec<CurveSurfaceHit> {
    match curve_surface(c, r, s, &ModelTolerance::DEFAULT).unwrap() {
        CurveSurface::Points(p) => p,
        other => panic!("expected points, got {other:?}"),
    }
}

/// `kind`: 0 sphere, 1 cylinder, 2 torus (axis y, R = 2, r = 1), 3 plane. The curve is fixed (a
/// unit circle or the ellipse a = 2, b = 1 in z = 0); the contact sits at curve parameter `t0`,
/// wherever that is, and the surface is placed to touch there with gap `g` (negative: it cuts
/// the curve). `psi` turns the surface frame about its axis away from the contact direction
/// (`psi = 0`: the contact is on the surface's seam). Everything is moved by `tf`.
fn scene(kind: usize, ell: bool, g: f64, t0: f64, psi: f64, tf: &Transform) -> Scene {
    let (a, b) = if ell { (2.0, 1.0) } else { (1.0, 1.0) };
    let (ex, ey, ez) = (Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
    let cdef = if ell { CurveDef::Ellipse(Ellipse::new(Frame::at(Point3::default()), a, b).unwrap()) } else { CurveDef::Circle(Circle::new(Frame::at(Point3::default()), 1.0).unwrap()) };
    let p = Point3::new(a * t0.cos(), b * t0.sin(), 0.0);
    let n = Vector3::new(t0.cos() / a, t0.sin() / b, 0.0);
    let n = n * (1.0 / n.norm());
    // Rotate `v` by `psi` about `axis`.
    let turn = |v: Vector3, axis: Vector3| v * psi.cos() + axis.cross(v) * psi.sin();
    let q = p + n * (1.0 + g);
    let (sname, sdef) = match kind {
        0 | 1 => {
            let x = turn(n * -1.0, ez);
            let f = Frame { origin: q, x, y: ez.cross(x), z: ez };
            if kind == 0 { ("sphere", SurfaceDef::Sphere(Sphere::new(f, 1.0).unwrap())) } else { ("cylinder", SurfaceDef::Cylinder(Cylinder::new(f, 1.0).unwrap())) }
        }
        2 => {
            // The tube circle through the contact is centred at q; the other one lies on the
            // far side of the tangent line.
            let m = if n.x >= 0.0 { ex } else { ex * -1.0 };
            let x = turn(m * -1.0, ey);
            ("torus", SurfaceDef::Torus(Torus::new(Frame { origin: q + m * 2.0, x, y: ey.cross(x), z: ey }, 2.0, 1.0).unwrap()))
        }
        _ => ("plane", SurfaceDef::Plane(Plane { frame: Frame::from_zx(p + n * g, n, turn(n.cross(ez), n)).unwrap() })),
    };
    let s = sdef.transformed(tf).unwrap().0;
    let c = cdef.transformed(tf).unwrap();
    let cn = match &c {
        CurveDef::Circle(x) => CurveDef::Nurbs(exact::circle_to_nurbs(x).unwrap()),
        CurveDef::Ellipse(x) => CurveDef::Nurbs(exact::ellipse_to_nurbs(x).unwrap()),
        _ => unreachable!(),
    };
    let sn = match &s {
        SurfaceDef::Sphere(x) => SurfaceDef::Nurbs(exact::sphere_to_nurbs(x).unwrap()),
        SurfaceDef::Cylinder(x) => SurfaceDef::Nurbs(exact::cylinder_to_nurbs(x, -1.0, 1.0).unwrap()),
        SurfaceDef::Torus(x) => SurfaceDef::Nurbs(exact::torus_to_nurbs(x).unwrap()),
        SurfaceDef::Plane(x) => SurfaceDef::Nurbs(exact::plane_to_nurbs(x, (-3.0, 3.0), (-3.0, 3.0)).unwrap()),
        _ => unreachable!(),
    };
    let p0 = c.as_curve().eval(t0);
    Scene { name: format!("{}/{sname} g={g} t0={t0} psi={psi}", if ell { "ellipse" } else { "circle" }), c, cn, s, sn, t0, p0 }
}

/// Expected topology for gap `g`: nothing past `tol.linear`, one tangent hit within it, two
/// transversal hits for a real penetration. Hits must sit near the nominal contact.
fn check(what: &str, h: &[CurveSurfaceHit], g: f64, p0: Point3) {
    if g >= 1e-5 {
        assert!(h.is_empty(), "{what}: {h:?}");
    } else if g <= -1e-4 {
        assert_eq!(h.len(), 2, "{what}: {h:?}");
        assert!(h.iter().all(|x| !x.tangent), "{what}: {h:?}");
        assert!(h.iter().all(|x| (x.point - p0).norm() < 0.25), "{what}: {h:?}");
    } else {
        assert_eq!(h.len(), 1, "{what}: {h:?}");
        assert!(h[0].tangent, "{what}: {h:?}");
        assert!((h[0].point - p0).norm() < 1e-2, "{what}: {h:?}");
    }
}

fn check_scene(sc: &Scene, g: f64) {
    check(&format!("{} analytic/analytic", sc.name), &hits(&sc.c, None, &sc.s), g, sc.p0);
    check(&format!("{} nurbs curve/analytic", sc.name), &hits(&sc.cn, None, &sc.s), g, sc.p0);
    check(&format!("{} analytic/nurbs surface", sc.name), &hits(&sc.c, None, &sc.sn), g, sc.p0);
    check(&format!("{} nurbs/nurbs", sc.name), &hits(&sc.cn, None, &sc.sn), g, sc.p0);
}

const GAPS: [f64; 14] = [0.0, 1e-9, -1e-9, 5e-7, -5e-7, 1e-5, -1e-3, -2.5e-7, 5.5e-7, 7e-7, 8e-7, 9e-7, 9.9e-7, -9e-7];

/// Parameters on the seam (0), the knots of the 9-pole circle (multiples of pi / 2) and their
/// +- 1e-9 and +- 3e-6 shifts, plus the seam from below.
fn special_angles() -> Vec<f64> {
    let mut v = Vec::new();
    for k in 0..4 {
        for d in [0.0, 1e-9, -1e-9, 3e-6, -3e-6] {
            v.push((k as f64 * FRAC_PI_2 + d).rem_euclid(TAU));
        }
    }
    v
}

#[test]
fn contacts_at_seams_and_knots_agree_on_every_path() {
    let id = Transform::IDENTITY;
    for kind in 0..4 {
        for ell in [false, true] {
            for t0 in special_angles() {
                for g in GAPS {
                    check_scene(&scene(kind, ell, g, t0, 0.7, &id), g);
                }
            }
        }
    }
}

/// Gaps inside the band `(-tol.linear, tol.linear)` near its edge, at knots and seam.
#[test]
fn band_edge_gaps_agree_on_every_path() {
    let id = Transform::IDENTITY;
    for kind in 0..4 {
        for ell in [false, true] {
            for t0 in special_angles() {
                for g in [5.5e-7, 6e-7, 7e-7, 8e-7, 9e-7, 9.9e-7, -9.9e-7, -8e-7] {
                    check_scene(&scene(kind, ell, g, t0, 0.7, &id), g);
                    check_scene(&scene(kind, ell, g, t0, 0.0, &id), g);
                }
            }
        }
    }
}

#[test]
fn surface_seams_and_knots_agree_on_every_path() {
    let id = Transform::IDENTITY;
    for kind in 0..4 {
        for psi in special_angles() {
            for g in [0.0, 1e-9, -5e-7, 5e-7, 1e-5, -1e-3] {
                check_scene(&scene(kind, false, g, 0.9, psi, &id), g);
            }
        }
    }
}

proptest! {
    #![proptest_config(config(96))]

    /// Gap drawn uniformly inside the tolerance band.
    #[test]
    fn random_in_band_gaps_agree(kind in 0usize..4, ell in any::<bool>(), g in -9.9e-7f64..9.9e-7, t0 in 0.0f64..TAU, special in 0usize..20, use_special in any::<bool>(), psi in 0.0f64..TAU, tf in proper_transform()) {
        let t0 = if use_special { special_angles()[special] } else { t0 };
        check_scene(&scene(kind, ell, g, t0, psi, &tf), g);
    }

    #[test]
    fn random_contacts_agree_on_every_path(kind in 0usize..4, ell in any::<bool>(), gi in 0usize..GAPS.len(), t0 in 0.0f64..TAU, psi in 0.0f64..TAU, tf in proper_transform()) {
        let g = GAPS[gi];
        check_scene(&scene(kind, ell, g, t0, psi, &tf), g);
    }

    #[test]
    fn random_windows_across_the_seam_agree(kind in 0usize..4, ell in any::<bool>(), gi in 0usize..GAPS.len(), t0 in 0.0f64..TAU, a in 0.2f64..1.0, b in 0.2f64..1.0, k in -2i32..3, tf in proper_transform()) {
        let g = GAPS[gi];
        let sc = scene(kind, ell, g, t0, 0.3, &tf);
        let r = Interval::new(sc.t0 - a + k as f64 * TAU, sc.t0 + b + k as f64 * TAU);
        for (what, s) in [("analytic", &sc.s), ("nurbs surface", &sc.sn)] {
            let h = hits(&sc.c, Some(r), s);
            check(&format!("{} window {r:?} {what}", sc.name), &h, g, sc.p0);
            prop_assert!(h.iter().all(|x| x.t >= r.lo - 1e-9 && x.t <= r.hi + 1e-9), "{}: hit outside the window {h:?}", sc.name);
        }
    }

    /// A window that contains the seam (t = 0) and a contact on it, at any gap.
    #[test]
    fn windows_containing_the_seam_agree(kind in 0usize..4, ell in any::<bool>(), gi in 0usize..GAPS.len(), a in 0.3f64..1.0, b in 0.3f64..1.0, tf in proper_transform()) {
        let g = GAPS[gi];
        let sc = scene(kind, ell, g, 0.0, 0.3, &tf);
        let r = Interval::new(-a, b);
        for (what, s) in [("analytic", &sc.s), ("nurbs surface", &sc.sn)] {
            let h = hits(&sc.c, Some(r), s);
            check(&format!("{} window {r:?} {what}", sc.name), &h, g, sc.p0);
            prop_assert!(h.iter().all(|x| r.contains(x.t, 1e-9)), "{}: hit outside the window {h:?}", sc.name);
        }
    }
}

/// The seam case from review: unit sphere, circle r = 1 centred (2 + g, 0, 0), frame flipped so
/// the contact is at `t = 0` (here `t0 = pi` of the standard frame is the same contact).
#[test]
fn review_seam_case() {
    let id = Transform::IDENTITY;
    for g in GAPS {
        check_scene(&scene(0, false, g, PI, 0.0, &id), g);
        check_scene(&scene(2, true, g, PI, 0.0, &id), g);
    }
}
