//! W4-BOOL-WEBCUT2: a sphere tool against a cylinder target does not depend on the user's frame;
//! an untouched target adds no surface; the typed error of the degenerate branch; cone tools.

mod common;

use common::rig::*;
use common::volume_of;
use core::f64::consts::PI;
use sm_boolean::{CutTool, WebcutOptions, webcut};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Cone, EllipticCone, Frame, Sphere, SurfaceDef};
use sm_topo_api::TopologyView;

fn frame(o: [f64; 3], z: [f64; 3]) -> Frame {
    Frame::from_z(Point3::new(o[0], o[1], o[2]), Vector3::new(z[0], z[1], z[2])).unwrap()
}

fn vol(t: &dyn TopologyView, bs: &[EntityId]) -> f64 {
    bs.iter().map(|&b| t.boundary(b).iter().map(|e| volume_of(t, e.cell)).sum::<f64>()).sum()
}

/// Volume of a ball (R) centred on the axis of a cylinder (r, taller than the ball): the ball
/// inside the tube, `2 pi (zc r^2 + R^2 (R - zc) - (R^3 - zc^3) / 3)` with `zc = sqrt(R^2 - r^2)`.
fn ball_in_cylinder(big: f64, r: f64) -> f64 {
    let zc = (big * big - r * r).sqrt();
    2.0 * PI * (zc * r * r + big * big * (big - zc) - (big.powi(3) - zc.powi(3)) / 3.0)
}

/// Cut a z-cylinder (r = 1, h = 4, centred at the origin) by a sphere (R = 1.5) at `c` with axis `z`.
fn cyl_cut(c: [f64; 3], z: [f64; 3]) -> Result<(f64, f64), String> {
    let mut r = Rig::cc();
    let cy = r.cylinder(4.0, 1.0, [0.0; 3]);
    let def = SurfaceDef::Sphere(Sphere::new(frame(c, z), 1.5).unwrap());
    let (host, out) = r.lib(|t| webcut(t, &[cy.body], &CutTool::Surface { surface: def, window: None }, &WebcutOptions::default()))?;
    let t: &dyn TopologyView = &*host;
    assert!(sm_topo_api::validate(t).is_empty(), "validate: {:?}", sm_topo_api::validate(t));
    Ok((vol(t, &out.negative), vol(t, &out.positive)))
}

/// The sphere's frame is the user's choice and must not matter: x, y, z and tilted axes, the
/// sphere coaxial with the cylinder and offset from it.
#[test]
fn sphere_against_a_cylinder_in_any_frame() {
    let total = PI * 4.0;
    let coaxial = ball_in_cylinder(1.5, 1.0);
    let axes: [[f64; 3]; 6] = [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 1.0], [0.3, -0.7, 0.2], [0.0, 0.2, 1.0]];
    for z in axes {
        let (i, o) = cyl_cut([0.0; 3], z).unwrap_or_else(|e| panic!("coaxial {z:?}: {e}"));
        // Curved-on-curved pieces come from quadrature: about 4e-8 (DESIGN.md).
        assert!((i - coaxial).abs() < 1e-7, "coaxial {z:?}: {i} vs {coaxial}");
        assert!((i + o - total).abs() < 1e-6, "coaxial {z:?}: sum {}", i + o);
    }
    // Offsets: no closed form here, but the pieces sum to the cylinder and the result does not
    // depend on the frame (curved-on-curved quadrature, about 1e-6).
    for off in [[0.8, 0.0, 0.0], [0.0, 0.8, 0.0], [0.5, -0.4, 0.3]] {
        let mut first = None;
        for z in axes {
            let (i, o) = cyl_cut(off, z).unwrap_or_else(|e| panic!("offset {off:?} {z:?}: {e}"));
            assert!((i + o - total).abs() < 1e-5, "offset {off:?} {z:?}: sum {}", i + o);
            let f = *first.get_or_insert(i);
            assert!((i - f).abs() < 1e-5, "offset {off:?} {z:?}: {i} vs {f}");
            assert!(i > 0.0 && o > 0.0);
        }
    }
}

/// Run `webcut` of one brick by `def` and return the typed outcome with the host.
fn brick_cut(size: [f64; 3], def: SurfaceDef) -> (Box<dyn sm_layer_api::TopoHost>, Result<sm_boolean::WebcutOutcome, sm_boolean::BooleanError>) {
    let mut r = Rig::cc();
    let b = r.brick(size, [0.0; 3]);
    let mut host = r.layer().topo_host().clone_host();
    host.begin_op(sm_topo_api::OpId(r.layer().ops_run() + 1)).unwrap();
    let out = webcut(&mut *host, &[b.body], &CutTool::Surface { surface: def, window: None }, &WebcutOptions::default());
    (host, out)
}

/// A sphere that cannot touch the target, or holds it whole, adds no surface to the store.
#[test]
fn untouched_target_adds_no_surface() {
    // Far away: the target is outside. Around: the target is inside. Crossing: a re-framed
    // surface beside the tool's own handle (`webcut` always adds that one).
    for (centre, rad, adds) in [([20.0, 0.0, 0.0], 1.5, 1), ([0.0; 3], 20.0, 1), ([0.0; 3], 1.5, 2)] {
        let mut r = Rig::cc();
        let b = r.brick([6.0; 3], [0.0; 3]);
        let before = r.topo().shapes().surface_count();
        let def = SurfaceDef::Sphere(Sphere::new(frame(centre, [1.0, 0.3, 0.2]), rad).unwrap());
        let (host, out) = r.lib(|t| webcut(t, &[b.body], &CutTool::Surface { surface: def, window: None }, &WebcutOptions::default())).unwrap();
        let after = host.shapes().surface_count();
        assert_eq!(after - before, adds, "centre {centre:?} R {rad}");
        assert_eq!(out.negative.len(), usize::from(rad == 20.0 || adds == 2));
    }
}

/// A cone whose apex lies on a face of the target is a typed `ResultInvalid` (DESIGN.md limit).
#[test]
fn cone_apex_on_a_target_face_is_result_invalid() {
    let cone = Cone::new(frame([0.0, 0.0, 3.0], [0.0, 0.0, -1.0]), 0.0, 0.3).unwrap();
    let (mut host, out) = brick_cut([6.0; 3], SurfaceDef::Cone(cone));
    host.abort_op();
    assert!(matches!(out, Err(sm_boolean::BooleanError::ResultInvalid { rule: "webcut", .. })), "{out:?}");
}

fn rot_frame(o: [f64; 3], s: f64, phi: f64) -> Frame {
    let x = Vector3::new(phi.cos(), phi.sin(), 0.0);
    Frame { origin: Point3::new(o[0], o[1], o[2]), x, y: Vector3::new(-s * phi.sin(), s * phi.cos(), 0.0), z: Vector3::new(0.0, 0.0, s) }
}

/// Volume of a circular cone nappe `rho = k (w - wa)` between `a` and `b` along its axis.
fn nappe(k: f64, wa: f64, a: f64, b: f64) -> f64 {
    PI * k * k * ((b - wa).powi(3) - (a - wa).powi(3)) / 3.0
}

mod prop {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #![proptest_config(sm_testkit::strategies::config(16))]

        /// A cone with a random offset, radius, half angle, direction (up or down) and frame
        /// rotation in a brick `z in [-2, 2]`: the piece inside is the frustum (or the full cone)
        /// between the apex or the brick face behind it and the brick face ahead.
        #[test]
        fn cone_frustum_volumes(x in -1.0f64..1.0, y in -1.0f64..1.0, z0 in -1.5f64..1.5, r0 in 0.0f64..0.4, half in 0.15f64..0.5, up in proptest::bool::ANY, phi in 0.0f64..core::f64::consts::TAU) {
            let s = if up { 1.0 } else { -1.0 };
            let k = half.tan();
            let wa = -r0 / k; // the apex, along the axis from the frame origin
            // The brick spans w in [wlo, whi] along the axis.
            let (wlo, whi) = if up { (-2.0 - z0, 2.0 - z0) } else { (z0 - 2.0, z0 + 2.0) };
            prop_assume!(wa < whi - 0.1 && (wa - wlo).abs() > 0.1);
            let want = nappe(k, wa, wa.max(wlo), whi);
            let def = SurfaceDef::Cone(Cone::new(rot_frame([x, y, z0], s, phi), r0, half).unwrap());
            let (host, out) = brick_cut([12.0, 12.0, 4.0], def);
            let mut host = host;
            let out = out.unwrap();
            host.end_op().unwrap();
            let t: &dyn TopologyView = &*host;
            prop_assert!(sm_topo_api::validate(t).is_empty());
            let (vi, vo) = (vol(t, &out.negative), vol(t, &out.positive));
            prop_assert!((vi - want).abs() < 1e-7 * want.max(1.0), "{vi} vs {want}");
            prop_assert!((vi + vo - 576.0).abs() < 1e-6);
        }

        /// The same for an elliptic cone: its sections scale the circular cone's by `b / a`.
        #[test]
        fn elliptic_cone_frustum_volumes(x in -1.0f64..1.0, y in -1.0f64..1.0, z0 in -1.5f64..1.5, a in 0.4f64..1.0, ratio in 0.4f64..1.0, half in 0.15f64..0.5, up in proptest::bool::ANY, phi in 0.0f64..core::f64::consts::TAU) {
            let s = if up { 1.0 } else { -1.0 };
            let (k, b) = (half.tan(), a * ratio);
            let wa = -a / k;
            let (wlo, whi) = if up { (-2.0 - z0, 2.0 - z0) } else { (z0 - 2.0, z0 + 2.0) };
            prop_assume!(wa < whi - 0.1 && (wa - wlo).abs() > 0.1);
            let want = b / a * nappe(k, wa, wa.max(wlo), whi);
            let def = SurfaceDef::EllipticCone(EllipticCone::new(rot_frame([x, y, z0], s, phi), a, b, half).unwrap());
            let (host, out) = brick_cut([12.0, 12.0, 4.0], def);
            let mut host = host;
            let out = out.unwrap();
            host.end_op().unwrap();
            let t: &dyn TopologyView = &*host;
            prop_assert!(sm_topo_api::validate(t).is_empty());
            let (vi, vo) = (vol(t, &out.negative), vol(t, &out.positive));
            prop_assert!((vi - want).abs() < 1e-7 * want.max(1.0), "{vi} vs {want}");
            prop_assert!((vi + vo - 576.0).abs() < 1e-6);
        }
    }
}
