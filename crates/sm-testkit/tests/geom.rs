//! Every analytic expectation is checked against the `sm-geom-api` implementation.

use core::f64::consts::{PI, TAU};
use sm_core::Vector3;
use sm_geom_api::{Cone, Cylinder, Ellipse, Interval, ParametricCurve, ParametricSurface, Plane, Sphere, Torus, Uv};
use sm_testkit::geom::*;

fn close(a: Vector3, b: Vector3, tol: f64) -> bool {
    (a - b).norm() <= tol
}

#[test]
fn normals_and_curvatures_match_the_jets() {
    for s in surface_fixtures() {
        for uv in s.sample_uvs() {
            let n = s.exact_normal(uv);
            assert!((n.norm() - 1.0).abs() < 1e-12, "{}", s.label());
            let api = s.normal(uv).unwrap();
            assert!(close(n, api, 1e-12), "{} normal at {uv:?}", s.label());
            let (k, h) = curvatures_from_jet(&s.jet(uv), n).unwrap();
            assert!((k - s.gaussian_curvature(uv)).abs() < 1e-9, "{} K at {uv:?}: {k} vs {}", s.label(), s.gaussian_curvature(uv));
            assert!((h - s.mean_curvature(uv)).abs() < 1e-9, "{} H at {uv:?}: {h} vs {}", s.label(), s.mean_curvature(uv));
        }
    }
}

#[test]
fn closest_points_match_offsets_along_the_normal_and_project() {
    for s in surface_fixtures() {
        for uv in s.sample_uvs() {
            let n = s.exact_normal(uv);
            let on = s.eval(uv);
            for d in [0.1, -0.1] {
                let q = on + n * d;
                let cp = s.closest_point(q).unwrap();
                assert!((cp - on).norm() < 1e-9, "{} closest at {uv:?} d={d}", s.label());
                let pr = s.project(q, None).unwrap();
                assert!((pr.distance - 0.1).abs() < 1e-9, "{} distance", s.label());
                assert!((pr.point - on).norm() < 1e-9, "{} project point", s.label());
            }
        }
    }
}

#[test]
fn cone_closest_point_clamps_at_the_apex() {
    let c = Cone::new(world_frame(), 0.0, 0.5).unwrap();
    // Below the apex, closest is the apex itself.
    let q = sm_core::Point3::new(0.1, 0.0, -1.0);
    assert!((c.closest_point(q).unwrap() - c.apex()).norm() < 1e-12);
    assert!((c.project(q, None).unwrap().point - c.apex()).norm() < 1e-9);
}

fn numeric_area(s: &dyn ParametricSurface, u: Interval, v: Interval, n: usize) -> f64 {
    let (hu, hv) = (u.len() / n as f64, v.len() / n as f64);
    let mut sum = 0.0;
    for i in 0..n {
        for j in 0..n {
            let uv = Uv::new(u.lo + (i as f64 + 0.5) * hu, v.lo + (j as f64 + 0.5) * hv);
            let jet = s.jet(uv);
            sum += jet.du.cross(jet.dv).norm();
        }
    }
    sum * hu * hv
}

#[test]
fn areas_match_closed_forms_and_quadrature() {
    for s in surface_fixtures() {
        let (u, v) = match s.label() {
            "plane" => (Interval::new(-1.0, 2.0), Interval::new(0.5, 3.0)),
            "cylinder" => (Interval::new(0.2, 5.0), Interval::new(-1.0, 2.0)),
            "cone" => (Interval::new(0.0, TAU), Interval::new(0.0, 3.0)),
            "sphere" => (Interval::new(0.0, TAU), Interval::new(-PI / 2.0, PI / 2.0)),
            _ => (Interval::new(0.0, TAU), Interval::new(0.0, TAU)),
        };
        let exact = s.area(u, v);
        let num = numeric_area(&*s, u, v, 300);
        assert!(((exact - num) / exact).abs() < 1e-4, "{}: {exact} vs {num}", s.label());
    }
}

#[test]
fn full_surface_areas() {
    let full = Interval::new(0.0, TAU);
    let sph = Sphere::new(world_frame(), 2.0).unwrap();
    assert!((sph.area(full, Interval::new(-PI / 2.0, PI / 2.0)) - sphere_area(2.0)).abs() < 1e-12);
    let tor = Torus::new(world_frame(), 3.0, 1.0).unwrap();
    assert!((tor.area(full, full) - torus_area(3.0, 1.0)).abs() < 1e-10);
    let cyl = Cylinder::new(world_frame(), 1.5).unwrap();
    assert!((cyl.area(full, Interval::new(0.0, 4.0)) - cylinder_lateral_area(1.5, 4.0)).abs() < 1e-12);
    // Cone frustum: pi (r1 + r2) slant length.
    let cone = Cone::new(world_frame(), 1.0, 0.3).unwrap();
    let (v1, v2) = (0.0, 2.0);
    let (r1, r2) = (1.0 + v1 * 0.3f64.sin(), 1.0 + v2 * 0.3f64.sin());
    assert!((cone.area(full, Interval::new(v1, v2)) - PI * (r1 + r2) * (v2 - v1)).abs() < 1e-12);
    let plane = Plane { frame: tilted_frame() };
    assert_eq!(plane.area(Interval::new(0.0, 2.0), Interval::new(0.0, 3.0)), 6.0);
}

#[test]
fn curve_tangents_curvatures_lengths_and_closest_points() {
    for c in curve_fixtures() {
        for t in c.sample_ts() {
            let jet = c.jet(t);
            let tan = c.exact_tangent(t);
            assert!(close(tan, jet.d1 * (1.0 / jet.d1.norm()), 1e-12), "{} tangent", c.label());
            let kappa = jet.d1.cross(jet.d2).norm() / jet.d1.norm().powi(3);
            assert!((kappa - c.curvature(t)).abs() < 1e-10, "{} curvature at {t}", c.label());
            if let Some(cp) = c.closest_point(jet.p) {
                assert!((cp - jet.p).norm() < 1e-9, "{} closest of an on-curve point", c.label());
            }
        }
        let range = if c.domain().is_periodic() { Interval::new(0.0, TAU) } else { Interval::new(-1.0, 2.5) };
        if let Some(exact) = c.arc_length(range) {
            let num = c.length(range, 1e-11).unwrap();
            assert!((exact - num).abs() < 1e-9, "{} length {exact} vs {num}", c.label());
        }
        if c.label() == "ellipse" {
            assert!(c.arc_length(Interval::new(0.0, 1.0)).is_none());
        }
    }
}

#[test]
fn circle_and_line_closest_points_off_the_curve() {
    for c in curve_fixtures().into_iter().filter(|c| c.label() != "ellipse") {
        for t in c.sample_ts() {
            let on = c.eval(t);
            // Offset perpendicular to the tangent.
            let tan = c.exact_tangent(t);
            let off = if tan.x.abs() < 0.9 { Vector3::new(1.0, 0.0, 0.0) } else { Vector3::new(0.0, 1.0, 0.0) };
            let perp = off - tan * off.dot(tan);
            let perp = perp * (1.0 / perp.norm());
            let q = on + perp * 0.05;
            if let Some(cp) = c.closest_point(q) {
                let pr = c.project(q, None).unwrap();
                assert!((pr.point - cp).norm() < 1e-9, "{}: projected {:?} vs closed form {:?}", c.label(), pr.point, cp);
            }
        }
    }
}

#[test]
fn ellipse_perimeter_closed_form() {
    // Degenerate inputs are defined: a flat ellipse is a doubled segment.
    assert_eq!(ellipse_perimeter(2.0, 0.0), 8.0);
    assert_eq!(ellipse_perimeter(0.0, 3.0), 12.0);
    assert_eq!(ellipse_perimeter(0.0, 0.0), 0.0);
    assert!(ellipse_perimeter(-1.0, 1.0).is_nan() && ellipse_perimeter(1.0, f64::INFINITY).is_nan());
    assert!((ellipse_perimeter(1.0, 1.0) - TAU).abs() < 1e-14);
    assert!((ellipse_perimeter(3.0, 3.0) - 6.0 * PI).abs() < 1e-13);
    assert!((ellipse_perimeter(2.0, 1.0) - 9.688_448_220_547_672).abs() < 1e-12);
    assert!((ellipse_perimeter(1.0, 2.0) - ellipse_perimeter(2.0, 1.0)).abs() < 1e-15, "symmetric in the axes");
    // Ramanujan's approximation agrees to ~1e-6 relative for a moderately eccentric ellipse.
    let (a, b) = (5.0f64, 3.0f64);
    let h = ((a - b) / (a + b)).powi(2);
    let ram = PI * (a + b) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()));
    assert!((ellipse_perimeter(a, b) - ram).abs() / ram < 1e-6);
}

#[test]
fn ellipse_projection_is_exact_in_the_contract() {
    // W0f gaps4: the contract projects onto ellipses (Eberly bisection).
    let e = Ellipse::new(world_frame(), 2.0, 1.0).unwrap();
    let q = sm_core::Point3::new(1.0, 1.0, 0.0);
    let pr = e.project(q, None).unwrap();
    let best = (0..20000).map(|i| (e.eval(2.0 * PI * f64::from(i) / 20000.0) - q).norm()).fold(f64::INFINITY, f64::min);
    assert!(pr.distance <= best + 1e-9, "{} vs {best}", pr.distance);
}
