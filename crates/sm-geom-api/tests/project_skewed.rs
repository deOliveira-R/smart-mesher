//! Regression: `NurbsSurface::project` on a strongly twisted bilinear patch.
//!
//! Points on the surface must project to themselves. Before the fix, an undamped Newton step
//! overshot on this skewed patch, the loop gave up, and the result was the nearest sample of
//! the 1/16 seed grid (up to 6e-2 away), which moved swept-mesh nodes off the surface iso-lines.

use sm_core::Point3;
use sm_geom_api::nurbs::NurbsSurface;
use sm_geom_api::{ParametricSurface, Uv};

/// Lateral face 0 of `sm_testkit::topo::twisted_prism` for the failing CI inputs.
fn twisted_patch() -> NurbsSurface {
    let (twist, side, h) = (120.1275823096648f64.to_radians(), 2.447990487719083f64, 0.5f64);
    let q = 0.5 * side;
    let (c, s) = (twist.cos(), twist.sin());
    let rot = |x: f64, y: f64| Point3::new(c * x - s * y, s * x + c * y, h);
    let ctrl = vec![Point3::new(-q, -q, 0.0), rot(-q, -q), Point3::new(q, -q, 0.0), rot(q, -q)];
    NurbsSurface::new(1, 1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 0.0, 1.0, 1.0], 2, 2, ctrl, None).expect("patch")
}

#[test]
fn on_surface_points_project_to_themselves() {
    let srf = twisted_patch();
    let n = 20;
    for i in 0..=n {
        for k in 0..=n {
            let uv = Uv::new(f64::from(i) / f64::from(n), f64::from(k) / f64::from(n));
            let p = srf.jet(uv).p;
            for hint in [None, Some(Uv::new(0.5, 0.5))] {
                let pr = srf.project(p, hint).expect("project");
                assert!(pr.distance < 1e-12, "{uv:?} hint {hint:?}: distance {}", pr.distance);
                assert!((pr.uv.u - uv.u).abs() < 1e-12 && (pr.uv.v - uv.v).abs() < 1e-12, "{uv:?} -> {:?}", pr.uv);
            }
        }
    }
}

#[test]
fn off_surface_points_reach_an_orthogonal_foot() {
    let srf = twisted_patch();
    for &(u, v) in &[(0.25, 0.6), (0.5, 0.4), (0.75, 0.4)] {
        let j = srf.jet(Uv::new(u, v));
        let n = j.du.cross(j.dv);
        let p = j.p + n * (0.01 / n.norm());
        let pr = srf.project(p, None).expect("project");
        let f = srf.jet(pr.uv);
        let r = p - f.p;
        // Stationarity: the residual is orthogonal to both tangents.
        assert!(r.dot(f.du).abs() < 1e-10 && r.dot(f.dv).abs() < 1e-10, "({u},{v}): not orthogonal");
        assert!(pr.distance <= 0.01 + 1e-12, "({u},{v}): distance {}", pr.distance);
    }
}
