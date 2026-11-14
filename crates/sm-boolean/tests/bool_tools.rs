//! W4-BOOL-TOOLS: kernel limits found by the W4a reviews, each fixed with a test or pinned as a
//! typed limit: coaxial cylinder cuts that failed at certain radii, plane cuts near a face or
//! giving a thin slab, `locate_point` in thin annuli.

mod common;

use common::rig::*;
use common::volume_of;
use core::f64::consts::PI;
use sm_boolean::ops::{AxisPlane, PlaneAxis, PlaneSpec, Subtract, WebcutCylinder, WebcutPlane};
use sm_boolean::{PointLocation, locate_point};
use sm_core::Point3;
use sm_op_api::Delta;

fn vol_sum(r: &Rig, d: &Delta, role: &str) -> f64 {
    d.outputs.get(role).map(|v| v.iter().map(|&b| r.topo().boundary(b).iter().map(|e| volume_of(r.topo(), e.cell)).sum::<f64>()).sum()).unwrap_or(0.0)
}

fn cyl_cut(r_tgt: f64, r_cut: f64) -> Result<(f64, f64), String> {
    let mut r = Rig::cc();
    let c = r.cylinder(1.0, r_tgt, [0.0, 0.0, 0.0]);
    let op = WebcutCylinder { targets: vec![c.body], radius: r_cut, axis: [0.0, 0.0, 1.0], center: [0.0, 0.0, 0.0], keep: None, merge: None, imprint: None };
    let d = r.run(&op).map_err(|e| e.to_string())?;
    let t = r.topo();
    if !sm_topo_api::validate(t).is_empty() || !sm_topo_api::validate_geometry(t).is_empty() {
        return Err("invalid topology".into());
    }
    Ok((vol_sum(&r, &d, "negative"), vol_sum(&r, &d, "positive")))
}

/// The W4a REGIONS-CUT finding: a 0.475 cylinder cut at 0.418 or 0.476 with the default tool
/// frame. The cause was not the seam: `interior_point` chose a point of the annular cap that
/// lies inside the chord sagitta of the inner circle (on the wrong side of it), so the fragment
/// was put on the wrong side of the tool and the shells did not close. Both cut positions, and
/// the radii around them, now give analytic volumes with the default frame. (A tool radius of
/// 0.476 does not cross a 0.475 target; the crossing case is a 0.5 target.)
#[test]
fn coaxial_cylinder_cuts_at_the_failing_radii_are_exact() {
    let r_tgt = 0.475;
    for r_cut in [0.3, 0.41, 0.417, 0.418, 0.42, 0.442, 0.45, 0.466, 0.47] {
        let (inside, outside) = cyl_cut(r_tgt, r_cut).unwrap_or_else(|e| panic!("cut at {r_cut}: {e}"));
        assert!((inside - PI * r_cut * r_cut).abs() < 1e-9, "inside at {r_cut}: {inside}");
        assert!((outside - PI * (r_tgt * r_tgt - r_cut * r_cut)).abs() < 1e-9, "outside at {r_cut}: {outside}");
    }
    // The other radius of the finding, 0.476, crossed for real: a 0.5 target, so the wall is 0.024.
    let (inside, outside) = cyl_cut(0.5, 0.476).unwrap();
    assert!((inside - PI * 0.476 * 0.476).abs() < 1e-9, "inside {inside}");
    assert!((outside - PI * (0.25 - 0.476 * 0.476)).abs() < 1e-9, "outside {outside}");
}

/// Documented limit: a wall thinner than the sagitta of the 32-sample boundary polygon (about
/// 1 % of the radius, here 0.471..0.474 on 0.475) can still put the fragment point on the
/// wrong side, and the cut is refused with the typed `ResultInvalid` (rolled back).
#[test]
fn a_wall_thinner_than_one_percent_of_the_radius_is_a_typed_refusal() {
    let e = cyl_cut(0.475, 0.473).unwrap_err();
    assert!(e.contains("webcut result volume") && e.contains("invalid"), "{e}");
}

fn plane_x(size: f64, x: f64) -> Result<(f64, f64), String> {
    let mut r = Rig::cc();
    let b = r.brick([size; 3], [size / 2.0; 3]);
    let op = WebcutPlane { targets: vec![b.body], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: x }), keep: None, merge: None, imprint: None };
    let d = r.run(&op).map_err(|e| e.to_string())?;
    let t = r.topo();
    if !sm_topo_api::validate(t).is_empty() || !sm_topo_api::validate_geometry(t).is_empty() {
        return Err("invalid topology".into());
    }
    Ok((vol_sum(&r, &d, "negative"), vol_sum(&r, &d, "positive")))
}

/// A plane within 1e-3 (relative) of a face made a thin slab whose shell was probed with a step
/// larger than the slab, so the slab was taken for a cavity wall. The probe now steps down a
/// decade ladder. Volumes are exact.
#[test]
fn a_plane_cut_near_a_face_gives_the_exact_slab() {
    for x in [0.02, 0.01, 0.001, 1e-4, 9.99, 9.999, 9.9999] {
        let (neg, pos) = plane_x(10.0, x).unwrap_or_else(|e| panic!("x = {x}: {e}"));
        assert!((neg - 100.0 * x).abs() < 1e-9 && (pos - 100.0 * (10.0 - x)).abs() < 1e-9, "x = {x}: {neg} {pos}");
    }
}

/// A slab between two planes 1e-3 and 1e-4 thick, in the middle of the brick (the CSG-CHECK gap
/// and sliver rows): both cuts in sequence.
#[test]
fn thin_slabs_in_the_middle_are_exact() {
    for w in [1e-3, 1e-4] {
        let mut r = Rig::cc();
        let b = r.brick([10.0; 3], [5.0; 3]);
        let cut = |r: &mut Rig, body, x| {
            r.run(&WebcutPlane { targets: vec![body], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: x }), keep: None, merge: None, imprint: None }).unwrap()
        };
        let d = cut(&mut r, b.body, 5.0 - 0.5 * w);
        let right = d.outputs["positive"][0];
        let d = cut(&mut r, right, 5.0 + 0.5 * w);
        let slab = vol_sum(&r, &d, "negative");
        assert!((slab - 100.0 * w).abs() < 1e-9, "w = {w}: {slab}");
        assert!(sm_topo_api::validate(r.topo()).is_empty());
    }
}

fn tube_by_subtract(tgt: f64, tool_h: f64, rc: f64) -> Result<f64, String> {
    let mut r = Rig::cc();
    let a = r.cylinder(1.0, tgt, [0.0, 0.0, 0.0]);
    let b = r.cylinder(tool_h, rc, [0.0, 0.0, 0.0]);
    let d = r.run(&Subtract { targets: vec![a.body], tools: vec![b.body], keep: None, regularize: None, tol: None }).map_err(|e| e.to_string())?;
    let t = r.topo();
    if !sm_topo_api::validate(t).is_empty() || !sm_topo_api::validate_geometry(t).is_empty() {
        return Err("invalid topology".into());
    }
    Ok(d.outputs["volumes"].iter().map(|&v| volume_of(t, v)).sum())
}

/// `model.subtract` of a coaxial cylinder (through, and with the same height) at the radii that
/// failed in the W4a review, for the 0.475 target and a 0.476 target (the other radius named in
/// the finding); thin walls are the typed limit of the test above.
#[test]
fn coaxial_subtract_at_the_failing_radii_is_exact() {
    for tgt in [0.475, 0.476] {
        for h in [2.0, 1.0] {
            for rc in [0.3, 0.417, 0.418, 0.442, 0.466, 0.47] {
                let v = tube_by_subtract(tgt, h, rc).unwrap_or_else(|e| panic!("target {tgt}, tool height {h}, cut {rc}: {e}"));
                assert!((v - PI * (tgt * tgt - rc * rc)).abs() < 1e-9, "target {tgt}, tool height {h}, cut {rc}: {v}");
            }
        }
    }
    let e = tube_by_subtract(0.475, 2.0, 0.473).unwrap_err();
    assert!(e.contains("subtract result volume") && e.contains("invalid"), "{e}");
    // The webcut with a 0.476 target.
    let (inside, outside) = cyl_cut(0.476, 0.418).unwrap();
    assert!((inside - PI * 0.418 * 0.418).abs() < 1e-9 && (outside - PI * (0.476 * 0.476 - 0.418 * 0.418)).abs() < 1e-9);
}

/// The a3 finding: `locate_point` gave `In` for a thin annulus at a point 0.2 outside it (seen in
/// `ref_mini3x3_h2`, direction dependent). The ring there (radii 0.4096 and 0.418, height 2, at
/// pin centres of pitch 1.26) is rebuilt here by a coaxial cut, and 4 x 360 x 6 points (outside
/// at 0.2 and more, in the hole, inside the wall) are located. The deck's own ring was not
/// reproduced: no false `In` was found with the final kernel (the cut itself fails on the old
/// kernel), so this is a regression guard, not a proof of the cause: the false `In` is recorded as
/// "not reproduced, guarded", and the CSG worker re-probes it on the deck.
#[test]
fn locate_point_in_the_gap_ring_at_pin_centres() {
    let (ri, ro) = (0.4096, 0.418);
    let mut bad = Vec::new();
    let mut n = 0;
    for centre in [[0.0, 0.0], [1.26, 0.0], [-1.26, 1.26], [0.63, 0.63]] {
        let mut r = Rig::cc();
        let c = r.cylinder(2.0, ro, [centre[0], centre[1], 1.0]);
        let op = WebcutCylinder { targets: vec![c.body], radius: ri, axis: [0.0, 0.0, 1.0], center: [centre[0], centre[1], 1.0], keep: Some(sm_boolean::KeepSide::Positive), merge: None, imprint: None };
        let d = r.run(&op).unwrap();
        let tube = d.outputs["volumes"][0];
        let t = r.topo();
        for k in 0..360 {
            let a = k as f64 * PI / 180.0 + 0.001;
            for (rad, z, want) in [(0.618, 1.0, PointLocation::Out), (0.5, 0.7, PointLocation::Out), (0.35, 1.3, PointLocation::Out), (0.4138, 0.9, PointLocation::In), (0.9, 1.7, PointLocation::Out), (0.7, 0.3, PointLocation::Out)] {
                let p = Point3::new(centre[0] + rad * a.cos(), centre[1] + rad * a.sin(), z);
                n += 1;
                match locate_point(t, tube, p) {
                    Ok(l) if l == want => {}
                    other => bad.push(format!("{centre:?} a {a:.4} rad {rad} z {z}: {other:?}")),
                }
            }
        }
    }
    assert!(bad.is_empty(), "{} of {n} wrong, first: {:?}", bad.len(), &bad[..bad.len().min(5)]);
}

/// `locate_point` in a thin annulus (the outer ring of a coaxial cylinder cut): wall points are
/// `In`, hole and outside points `Out`, for walls from 0.1 down to 0.005 of a 0.5 radius.
#[test]
fn locate_point_in_thin_annuli() {
    for wall in [0.1, 0.02, 0.01, 0.005] {
        let (ro, ri) = (0.5, 0.5 - wall);
        let mut r = Rig::cc();
        let c = r.cylinder(1.0, ro, [0.0, 0.0, 0.0]);
        let op = WebcutCylinder { targets: vec![c.body], radius: ri, axis: [0.0, 0.0, 1.0], center: [0.0, 0.0, 0.0], keep: Some(sm_boolean::KeepSide::Positive), merge: None, imprint: None };
        let d = r.run(&op).unwrap_or_else(|e| panic!("wall {wall}: {e}"));
        let tube = d.outputs["volumes"][0];
        let t = r.topo();
        assert!((volume_of(t, tube) - PI * (ro * ro - ri * ri)).abs() < 1e-9);
        for k in 0..16 {
            let a = 0.37 + k as f64 * PI / 8.0;
            let at = |rad: f64, z: f64| Point3::new(rad * a.cos(), rad * a.sin(), z);
            let mid = 0.5 * (ro + ri);
            assert_eq!(locate_point(t, tube, at(mid, 0.1)).unwrap(), PointLocation::In, "wall {wall} angle {a}");
            assert_eq!(locate_point(t, tube, at(ri - 0.3 * wall.max(0.01), 0.1)).unwrap(), PointLocation::Out, "hole, wall {wall} angle {a}");
            assert_eq!(locate_point(t, tube, at(0.0, 0.1)).unwrap(), PointLocation::Out, "axis, wall {wall}");
            // Close to either wall surface, on each side (inside the chord sagitta of the polygon).
            assert_eq!(locate_point(t, tube, at(ri + 0.25 * wall, 0.1)).unwrap(), PointLocation::In, "inner side, wall {wall} angle {a}");
            assert_eq!(locate_point(t, tube, at(ro - 0.25 * wall, 0.1)).unwrap(), PointLocation::In, "outer side, wall {wall} angle {a}");
            assert_eq!(locate_point(t, tube, at(ri - 0.25 * wall, 0.1)).unwrap(), PointLocation::Out, "hole side, wall {wall} angle {a}");
            assert_eq!(locate_point(t, tube, at(ro + 0.25 * wall, 0.1)).unwrap(), PointLocation::Out, "beyond, wall {wall} angle {a}");
            assert_eq!(locate_point(t, tube, at(ro + 0.3 * wall.max(0.01), 0.1)).unwrap(), PointLocation::Out, "outside, wall {wall} angle {a}");
        }
    }
}

// ---- body tools: the native webcut tools and the composition of booleans ----

#[derive(Clone, Copy, Debug)]
enum Tool {
    Sphere,
    Frustum,
    Torus,
}

/// Splits a brick by a closed body tool with the existing booleans: the inside piece is
/// `intersect(keep)`, the outside piece `subtract(keep)`, both inputs are then deleted and the
/// two pieces merged. Returns (inside, outside, faces shared by both pieces). The shared count
/// is only informative: the two booleans build separate copies of the cut face, and `merge`
/// joins them for the frustum and torus but not for the sphere (poles and seam).
fn cut_by_body(tool: Tool, crossing: bool) -> Result<(f64, f64, usize), String> {
    use sm_boolean::ops::{Intersect, Merge};
    use sm_model::{CreateFrustum, CreateTorus, Delete};
    let mut r = Rig::cc();
    let z0 = if crossing { 0.0 } else { -3.0 };
    let b = r.brick([6.0; 3], [0.0, 0.0, z0 + 3.0]);
    let (t, full, half) = match tool {
        Tool::Sphere => (r.sphere(1.5, [0.0; 3]).body, 4.0 / 3.0 * PI * 1.5f64.powi(3), 2.0 / 3.0 * PI * 1.5f64.powi(3)),
        Tool::Frustum => {
            let d = r.run(&CreateFrustum { height: 4.0, radius: 2.0, minor: None, top: Some(1.0), center: Some([0.0; 3]) }).map_err(|e| e.to_string())?;
            let f = |h: f64, a: f64, b: f64| PI * h * (a * a + a * b + b * b) / 3.0;
            (d.outputs["body"][0], f(4.0, 2.0, 1.0), f(2.0, 1.5, 1.0))
        }
        Tool::Torus => {
            let d = r.run(&CreateTorus { major: 2.0, minor: 0.5, center: Some([0.0; 3]) }).map_err(|e| e.to_string())?;
            (d.outputs["body"][0], 2.0 * PI * PI * 2.0 * 0.25, PI * PI * 2.0 * 0.25)
        }
    };
    let inside_want = if crossing { half } else { full };
    let i = r.run(&Intersect { bodies: vec![b.body, t], keep: Some(true), regularize: None, tol: None }).map_err(|e| e.to_string())?;
    let o = r.run(&Subtract { targets: vec![b.body], tools: vec![t], keep: Some(true), regularize: None, tol: None }).map_err(|e| e.to_string())?;
    let (ib, ob) = (i.outputs["bodies"].clone(), o.outputs["bodies"].clone());
    r.run(&Delete { bodies: vec![b.body, t] }).map_err(|e| e.to_string())?;
    r.run(&Merge { entities: None, kinds: None, tol: None, check_only: None }).map_err(|e| e.to_string())?;
    let tp = r.topo();
    if !sm_topo_api::validate(tp).is_empty() || !sm_topo_api::validate_geometry(tp).is_empty() {
        return Err("invalid topology".into());
    }
    let vol_of_bodies = |bs: &[sm_core::EntityId]| -> f64 { bs.iter().map(|&b| tp.boundary(b).iter().map(|e| volume_of(tp, e.cell)).sum::<f64>()).sum() };
    let (vi, vo) = (vol_of_bodies(&ib), vol_of_bodies(&ob));
    assert!((vi - inside_want).abs() < 1e-9, "{tool:?} crossing={crossing}: inside {vi} want {inside_want}");
    // Faces used by a volume of each piece (the cut face is one cell with two uses).
    let faces_of = |bs: &[sm_core::EntityId]| -> std::collections::BTreeSet<sm_core::EntityId> {
        bs.iter().flat_map(|&b| tp.boundary(b).iter().flat_map(|e| tp.volume_faces(e.cell).to_vec()).collect::<Vec<_>>()).collect()
    };
    let shared = faces_of(&ib).intersection(&faces_of(&ob)).count();
    Ok((vi, vo, shared))
}

/// The composition of the existing booleans gives exact volumes and valid topology for the
/// three body tools. It is the cross-check of the native tools (`tests/webcut_tools.rs`), which
/// also share their cut face for the sphere, where the composition cannot.
#[test]
fn body_tools_by_boolean_composition() {
    for tool in [Tool::Sphere, Tool::Frustum, Tool::Torus] {
        for crossing in [false, true] {
            let (vi, vo, shared) = cut_by_body(tool, crossing).unwrap_or_else(|e| panic!("{tool:?} crossing={crossing}: {e}"));
            assert!((vi + vo - 216.0).abs() < 1e-9, "{tool:?} crossing={crossing}: {vi} + {vo}");
            println!("{tool:?} crossing={crossing}: inside {vi:.9} outside {vo:.9} shared faces {shared}");
        }
    }
}

/// The native sphere tool: the same split as the composition, with the cut face shared (the
/// temporary `shared == 0` pin of the composition is gone).
#[test]
fn native_sphere_tool_shares_its_cut_face() {
    use sm_boolean::{CutTool, WebcutOptions, webcut};
    use sm_geom_api::{Frame, Sphere, SurfaceDef};
    for crossing in [false, true] {
        let mut r = Rig::cc();
        let z0 = if crossing { 0.0 } else { -3.0 };
        let b = r.brick([6.0; 3], [0.0, 0.0, z0 + 3.0]);
        let sph = Sphere::new(Frame::from_z(Point3::new(0.0, 0.0, 0.0), sm_core::Vector3::new(0.0, 0.0, 1.0)).unwrap(), 1.5).unwrap();
        let (host, out) = r.lib(|t| webcut(t, &[b.body], &CutTool::Surface { surface: SurfaceDef::Sphere(sph), window: None }, &WebcutOptions::default())).unwrap();
        let t: &dyn sm_topo_api::TopologyView = &*host;
        let faces = |bs: &[sm_core::EntityId]| -> std::collections::BTreeSet<sm_core::EntityId> { bs.iter().flat_map(|&b| t.boundary(b).iter().flat_map(|e| t.volume_faces(e.cell).to_vec()).collect::<Vec<_>>()).collect() };
        let shared = faces(&out.negative).intersection(&faces(&out.positive)).count();
        assert_eq!(shared, 1, "crossing={crossing}");
        let vi: f64 = out.negative.iter().map(|&b| t.boundary(b).iter().map(|e| volume_of(t, e.cell)).sum::<f64>()).sum();
        let want = if crossing { 2.0 / 3.0 * PI * 1.5f64.powi(3) } else { 4.0 / 3.0 * PI * 1.5f64.powi(3) };
        assert!((vi - want).abs() < 1e-9, "crossing={crossing}: {vi}");
    }
}

/// Surfaces without a closed-solid form stay typed refusals.
#[test]
fn webcut_with_an_implicit_tool_is_a_typed_unsupported() {
    use sm_boolean::{CutTool, WebcutOptions, webcut};
    use sm_geom_api::{Frame, ImplicitDef, Quadric, SurfaceDef};
    let mut r = Rig::cc();
    let b = r.brick([6.0; 3], [0.0; 3]);
    let f = Frame::from_z(Point3::new(0.0, 0.0, 0.0), sm_core::Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let q = Quadric::from_local(&f, [1.0, 1.0, 1.0], [0.0; 3], -1.0);
    let res = r.lib(|t| webcut(t, &[b.body], &CutTool::Surface { surface: SurfaceDef::Implicit(ImplicitDef::Quadric(q)), window: None }, &WebcutOptions::default()));
    let e = res.err().expect("refused");
    assert!(e.contains("webcut supports plane, cylinder, sphere"), "{e}");
}
