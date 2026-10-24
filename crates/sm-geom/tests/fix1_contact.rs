//! Fix round 1: analytic curves against analytic surfaces near tangency follow the same
//! `tol.linear` contact rule as the NURBS paths; one tangent contact is one hit.

use core::f64::consts::PI;
use sm_core::{Point3, Vector3};
use sm_geom::exact;
use sm_geom::{CurveSurface, CurveSurfaceHit, curve_surface};
use sm_geom_api::{Circle, Cylinder, CurveDef, Ellipse, Frame, ModelTolerance, Sphere, SurfaceDef, Torus};

fn hits(c: &CurveDef, s: &SurfaceDef) -> Vec<CurveSurfaceHit> {
    match curve_surface(c, None, s, &ModelTolerance::DEFAULT).unwrap() {
        CurveSurface::Points(p) => p,
        other => panic!("expected points, got {other:?}"),
    }
}

type Case = (&'static str, CurveDef, CurveDef, SurfaceDef, SurfaceDef);

/// (name, analytic curve, its NURBS form, analytic surface, its NURBS form); `g` is the gap.
fn cases(g: f64) -> Vec<Case> {
    let sphere = Sphere::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap();
    let cyl = Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap();
    let torus = Torus::new(Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)).unwrap(), 2.0, 1.0).unwrap();
    let circ = |x: f64| Circle::new(Frame::at(Point3::new(x, 0.0, 0.0)), 1.0).unwrap();
    let ell = Ellipse::new(Frame::at(Point3::new(3.0 + g, 0.0, 0.0)), 2.0, 1.0).unwrap();
    let nc = |c: &Circle| CurveDef::Nurbs(exact::circle_to_nurbs(c).unwrap());
    let ne = CurveDef::Nurbs(exact::ellipse_to_nurbs(&ell).unwrap());
    let ns = SurfaceDef::Nurbs(exact::sphere_to_nurbs(&sphere).unwrap());
    let nz = SurfaceDef::Nurbs(exact::cylinder_to_nurbs(&cyl, -1.0, 1.0).unwrap());
    let nt = SurfaceDef::Nurbs(exact::torus_to_nurbs(&torus).unwrap());
    vec![
        ("circle/sphere", CurveDef::Circle(circ(2.0 + g)), nc(&circ(2.0 + g)), SurfaceDef::Sphere(sphere), ns.clone()),
        ("ellipse/sphere", CurveDef::Ellipse(ell), ne, SurfaceDef::Sphere(sphere), ns),
        ("circle/cylinder", CurveDef::Circle(circ(2.0 + g)), nc(&circ(2.0 + g)), SurfaceDef::Cylinder(cyl), nz),
        ("circle/torus", CurveDef::Circle(circ(4.0 + g)), nc(&circ(4.0 + g)), SurfaceDef::Torus(torus), nt),
    ]
}

#[test]
fn analytic_and_nurbs_agree_on_contact_within_tolerance() {
    for g in [0.0, 1e-9, 1e-8, 5e-7, -1e-9, -5e-7] {
        for (name, c, cn, s, sn) in cases(g) {
            let a = hits(&c, &s);
            assert_eq!(a.len(), 1, "{name} g={g}: {a:?}");
            assert!(a[0].tangent, "{name} g={g}: {a:?}");
            assert!((a[0].t - PI).abs() < 1e-3, "{name} g={g}: {a:?}");
            // NURBS curve vs analytic surface, analytic curve vs NURBS surface.
            assert_eq!(hits(&cn, &s).len(), 1, "{name} g={g}: nurbs curve");
            let b = hits(&c, &sn);
            assert_eq!(b.len(), 1, "{name} g={g}: nurbs surface {b:?}");
            assert!(b[0].tangent, "{name} g={g}: nurbs surface {b:?}");
        }
    }
}

#[test]
fn analytic_and_nurbs_agree_on_a_gap_beyond_tolerance() {
    for (name, c, cn, s, sn) in cases(1e-5) {
        assert!(hits(&c, &s).is_empty(), "{name}");
        assert!(hits(&cn, &s).is_empty(), "{name}: nurbs curve");
        assert!(hits(&c, &sn).is_empty(), "{name}: nurbs surface");
    }
}

#[test]
fn real_crossings_stay_two_hits() {
    // Penetration of 1e-3 (well past tol.linear): two transversal hits, not tangent.
    for (name, c, _, s, _) in cases(-1e-3) {
        let h = hits(&c, &s);
        assert_eq!(h.len(), 2, "{name}: {h:?}");
        assert!(h.iter().all(|x| !x.tangent), "{name}: {h:?}");
    }
}
