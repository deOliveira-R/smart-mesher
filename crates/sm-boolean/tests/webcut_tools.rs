//! W4-BOOL-WEBCUT: native sphere, cone, torus and elliptic webcut tools. A brick is split in two
//! volumes whose sum is the brick and whose inside piece has the analytic volume; both pieces
//! use one shared cut face; the cut is deterministic.

mod common;

use common::rig::*;
use common::volume_of;
use core::f64::consts::PI;
use sm_boolean::{CutTool, KeepSide, WebcutOptions, WebcutOutcome, webcut};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{Cone, EllipticCone, EllipticCylinder, Frame, Sphere, SurfaceDef, Torus};
use sm_topo_api::TopologyView;

fn frame(o: [f64; 3], z: [f64; 3]) -> Frame {
    Frame::from_z(Point3::new(o[0], o[1], o[2]), Vector3::new(z[0], z[1], z[2])).unwrap()
}

struct Cut {
    inside: f64,
    outside: f64,
    shared: usize,
    out: WebcutOutcome,
    hash: u64,
}

fn fnv(h: &mut u64, x: u64) {
    *h = (*h ^ x).wrapping_mul(0x100000001b3);
}

/// Cut a brick (`size`, centred on `c`) by `def`; check validity and return the volumes.
fn cut(def: SurfaceDef, size: [f64; 3], c: [f64; 3], merge: bool) -> Result<Cut, String> {
    let mut r = Rig::cc();
    let b = r.brick(size, c);
    let opts = WebcutOptions { merge, ..WebcutOptions::default() };
    let (host, out) = r.lib(|t| webcut(t, &[b.body], &CutTool::Surface { surface: def, window: None }, &opts))?;
    let t: &dyn TopologyView = &*host;
    assert!(sm_topo_api::validate(t).is_empty(), "validate: {:?}", sm_topo_api::validate(t));
    assert!(sm_topo_api::validate_geometry(t).is_empty(), "geometry: {:?}", sm_topo_api::validate_geometry(t));
    let vol = |bs: &[EntityId]| -> f64 { bs.iter().map(|&b| t.boundary(b).iter().map(|e| volume_of(t, e.cell)).sum::<f64>()).sum() };
    let faces_of = |bs: &[EntityId]| -> std::collections::BTreeSet<EntityId> { bs.iter().flat_map(|&b| t.boundary(b).iter().flat_map(|e| t.volume_faces(e.cell).to_vec()).collect::<Vec<_>>()).collect() };
    let shared = faces_of(&out.negative).intersection(&faces_of(&out.positive)).count();
    let mut h = 0xcbf29ce484222325u64;
    for &v in &out.volumes {
        fnv(&mut h, volume_of(t, v).to_bits());
    }
    fnv(&mut h, shared as u64);
    Ok(Cut { inside: vol(&out.negative), outside: vol(&out.positive), shared, out, hash: h })
}

fn check(name: &str, def: SurfaceDef, size: [f64; 3], c: [f64; 3], inside: f64) -> Cut {
    let k = cut(def, size, c, true).unwrap_or_else(|e| panic!("{name}: {e}"));
    let total = size[0] * size[1] * size[2];
    assert!((k.inside - inside).abs() < 1e-9 * inside.max(1.0), "{name}: inside {} want {inside}", k.inside);
    assert!((k.inside + k.outside - total).abs() < 1e-9 * total, "{name}: sum {} want {total}", k.inside + k.outside);
    assert!(k.shared >= 1, "{name}: no shared cut face");
    assert_eq!(k.out.negative.len(), 1, "{name}");
    assert_eq!(k.out.positive.len(), 1, "{name}");
    k
}

#[test]
fn sphere_inside_a_brick_splits_it_in_two_with_one_shared_face() {
    let r = 1.5;
    let k = check("sphere inside", SurfaceDef::Sphere(Sphere::new(frame([0.0; 3], [0.0, 0.0, 1.0]), r).unwrap()), [6.0; 3], [0.0; 3], 4.0 / 3.0 * PI * r * r * r);
    assert_eq!(k.shared, 1);
    assert_eq!(k.out.cut_faces.len(), 1);
}

#[test]
fn sphere_crossing_the_brick() {
    // A sphere centred on a face: a hemisphere is inside the brick.
    let r = 1.5;
    let k = check("sphere crossing", SurfaceDef::Sphere(Sphere::new(frame([0.0, 0.0, -3.0], [0.3, 0.2, 1.0]), r).unwrap()), [6.0; 3], [0.0; 3], 2.0 / 3.0 * PI * r * r * r);
    assert_eq!(k.shared, 1);
}

/// A sphere centred on a face, an edge and a corner of the brick: its great circles meet at
/// vertices of the target; the inside is 1/2, 1/4 and 1/8 of the ball.
#[test]
fn sphere_centred_on_a_face_an_edge_and_a_corner() {
    let r = 1.5;
    let ball = 4.0 / 3.0 * PI * r * r * r;
    for (centre, share) in [([0.0, 0.0, -3.0], 0.5), ([0.0, -3.0, -3.0], 0.25), ([-3.0, -3.0, -3.0], 0.125)] {
        let def = SurfaceDef::Sphere(Sphere::new(frame(centre, [0.0, 0.0, 1.0]), r).unwrap());
        let k = cut(def, [6.0; 3], [0.0; 3], true).unwrap_or_else(|e| panic!("{centre:?}: {e}"));
        assert!((k.inside - ball * share).abs() < 1e-9, "{centre:?}: {} want {}", k.inside, ball * share);
        assert!((k.inside + k.outside - 216.0).abs() < 1e-9);
        assert_eq!(k.shared, 1, "{centre:?}");
    }
}

#[test]
fn sphere_cut_keep_sides() {
    let r = 1.5;
    let def = SurfaceDef::Sphere(Sphere::new(frame([0.0; 3], [0.0, 0.0, 1.0]), r).unwrap());
    for (keep, want) in [(KeepSide::Negative, 4.0 / 3.0 * PI * r * r * r), (KeepSide::Positive, 216.0 - 4.0 / 3.0 * PI * r * r * r)] {
        let mut rig = Rig::cc();
        let b = rig.brick([6.0; 3], [0.0; 3]);
        let (host, out) = rig.lib(|t| webcut(t, &[b.body], &CutTool::Surface { surface: def.clone(), window: None }, &WebcutOptions { keep, ..WebcutOptions::default() })).unwrap();
        let t: &dyn TopologyView = &*host;
        let all: Vec<EntityId> = out.positive.iter().chain(&out.negative).copied().collect();
        assert_eq!(all.len(), 1, "{keep:?}");
        let v: f64 = all.iter().map(|&b| t.boundary(b).iter().map(|e| volume_of(t, e.cell)).sum::<f64>()).sum();
        assert!((v - want).abs() < 1e-9, "{keep:?}: {v} want {want}");
        assert!(sm_topo_api::validate(t).is_empty());
    }
}

#[test]
fn sphere_outside_and_around_are_untouched() {
    let mut rig = Rig::cc();
    let b = rig.brick([2.0; 3], [0.0; 3]);
    for (c, rad, positive) in [([10.0, 0.0, 0.0], 1.0, true), ([0.0; 3], 5.0, false)] {
        let def = SurfaceDef::Sphere(Sphere::new(frame(c, [0.0, 0.0, 1.0]), rad).unwrap());
        let (_, out) = rig.lib(|t| webcut(t, &[b.body], &CutTool::Surface { surface: def, window: None }, &WebcutOptions::default())).unwrap();
        assert_eq!(out.positive.len(), usize::from(positive));
        assert_eq!(out.negative.len(), usize::from(!positive));
    }
}

#[test]
fn torus_in_a_brick() {
    let (big, small) = (2.0, 0.5);
    let k = check("torus", SurfaceDef::Torus(Torus::new(frame([0.0; 3], [0.0, 0.0, 1.0]), big, small).unwrap()), [6.0; 3], [0.0; 3], 2.0 * PI * PI * big * small * small);
    assert_eq!(k.shared, 1);
    // Tilted torus.
    check("tilted torus", SurfaceDef::Torus(Torus::new(frame([0.2, -0.1, 0.3], [0.2, 0.5, 1.0]), big, small).unwrap()), [6.0; 3], [0.0; 3], 2.0 * PI * PI * big * small * small);
}

#[test]
fn torus_crossing_a_face() {
    // The torus ring in the plane z = 0 is the brick's bottom face: half the torus is inside.
    let (big, small) = (2.0, 0.5);
    check("torus half", SurfaceDef::Torus(Torus::new(frame([0.0; 3], [0.0, 0.0, 1.0]), big, small).unwrap()), [6.0, 6.0, 6.0], [0.0, 0.0, 3.0], PI * PI * big * small * small);
}

#[test]
fn cone_nappe_in_a_brick() {
    // Cone with its apex at the origin widening along +z: a frustum between z = 1 and z = 3.
    let half: f64 = 0.4;
    let cone = Cone::new(frame([0.0; 3], [0.0, 0.0, 1.0]), 0.0, half).unwrap();
    let (z0, z1) = (1.0, 3.0);
    let (r0, r1) = (z0 * half.tan(), z1 * half.tan());
    let want = PI * (z1 - z0) * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0;
    let k = check("cone", SurfaceDef::Cone(cone), [8.0, 8.0, 2.0], [0.0, 0.0, 2.0], want);
    assert_eq!(k.shared, 1);
    // Apex inside the brick: a full cone to z = 3.
    let want = PI * r1 * r1 * z1 / 3.0;
    check("cone apex", SurfaceDef::Cone(cone), [8.0, 8.0, 4.0], [0.0, 0.0, 1.0], want);
}

#[test]
fn cone_with_a_base_radius() {
    let half: f64 = 0.3;
    let cone = Cone::new(frame([0.0; 3], [0.0, 0.0, 1.0]), 0.3, half).unwrap();
    let v1 = 2.0f64;
    let z1 = v1 * half.cos();
    let (r0, r1) = (0.3, 0.3 + v1 * half.sin());
    let want = PI * z1 * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0;
    check("cone radius", SurfaceDef::Cone(cone), [8.0, 8.0, z1], [0.0, 0.0, 0.5 * z1], want);
}

#[test]
fn elliptic_cylinder_through_a_brick() {
    let (a, b, h) = (1.5, 0.8, 4.0);
    let k = check("elliptic cylinder", SurfaceDef::EllipticCylinder(EllipticCylinder::new(frame([0.3, 0.2, 0.0], [0.0, 0.0, 1.0]), a, b).unwrap()), [6.0, 6.0, h], [0.0; 3], PI * a * b * h);
    assert_eq!(k.shared, 1);
}

#[test]
fn elliptic_cone_in_a_brick() {
    let (a, b, half): (f64, f64, f64) = (1.0, 0.6, 0.3);
    let c = EllipticCone::new(frame([0.0; 3], [0.0, 0.0, 1.0]), a, b, half).unwrap();
    // Section at v: semi-axes a k, b k, k = 1 + v sin / a; the slab 0 <= v <= 2 is a frustum.
    let (v0, v1) = (0.0f64, 2.0f64);
    let k = |v: f64| 1.0 + v * half.sin() / a;
    let (s0, s1) = (PI * a * b * k(v0).powi(2), PI * a * b * k(v1).powi(2));
    let h = (v1 - v0) * half.cos();
    let want = h * (s0 + (s0 * s1).sqrt() + s1) / 3.0;
    check("elliptic cone", SurfaceDef::EllipticCone(c), [8.0, 8.0, h], [0.0, 0.0, 0.5 * h], want);
}

#[test]
fn unmerged_cut_gives_each_piece_its_own_face() {
    let def = SurfaceDef::Sphere(Sphere::new(frame([0.0; 3], [0.0, 0.0, 1.0]), 1.5).unwrap());
    let k = cut(def, [6.0; 3], [0.0; 3], false).unwrap();
    assert_eq!(k.shared, 0);
    assert!((k.inside - 4.0 / 3.0 * PI * 1.5f64.powi(3)).abs() < 1e-9);
}

/// Regression of the seam-loop orientation in `arrange`: a sphere tilted by -10 or -17 degrees
/// about x, cut through its centre by a plane, failed with an open shell when the sample of the
/// closing loop nearest to the probe was the last one (the tangent wrapped by a period).
#[test]
fn tilted_sphere_through_a_plane_booleans_and_webcut() {
    use sm_boolean::ops::{Intersect, Subtract};
    use sm_model::Rotate;
    for deg in [-5.0, -10.0, -17.0, -25.0] {
        let mut rig = Rig::cc();
        let b = rig.brick([12.0, 12.0, 6.0], [0.0, 0.0, 3.0]);
        let s = rig.sphere(1.0, [0.0; 3]);
        rig.run(&Rotate { entities: vec![s.body], axis: [1.0, 0.0, 0.0], angle_deg: deg, origin: None }).unwrap();
        let i = rig.run(&Intersect { bodies: vec![b.body, s.body], keep: Some(true), regularize: None, tol: None }).unwrap_or_else(|e| panic!("intersect {deg}: {e}"));
        let o = rig.run(&Subtract { targets: vec![b.body], tools: vec![s.body], keep: Some(true), regularize: None, tol: None }).unwrap_or_else(|e| panic!("subtract {deg}: {e}"));
        let t = rig.topo();
        let vol = |d: &sm_op_api::Delta| -> f64 { d.outputs["bodies"].iter().map(|&b| t.boundary(b).iter().map(|e| volume_of(t, e.cell)).sum::<f64>()).sum() };
        assert!((vol(&i) - 2.0 / 3.0 * PI).abs() < 1e-8, "{deg}: {}", vol(&i));
        let vo = vol(&o) - 432.0; // the kept target is listed beside the result
        assert!((vo - (432.0 - 2.0 / 3.0 * PI)).abs() < 1e-7, "{deg}: {vo}");
    }
    // The same through the native tool, tilted the other way.
    for ty in [0.15, 0.2, 0.3, 0.4] {
        let k = cut(SurfaceDef::Sphere(Sphere::new(frame([0.0; 3], [0.0, ty, 1.0]), 1.0).unwrap()), [12.0, 12.0, 6.0], [0.0, 0.0, 3.0], true).unwrap_or_else(|e| panic!("ty {ty}: {e}"));
        assert!((k.inside - 2.0 / 3.0 * PI).abs() < 1e-8, "ty {ty}: {}", k.inside);
    }
}

/// The CSG-CHECK lens: two equal spheres (radius R, centre distance d), the ball of the first cut
/// by the second. The inside piece is the lens `pi (4R + d)(2R - d)^2 / 12`, the other piece the
/// ball minus it, and they share the cut face (one TopoFace, two uses).
#[test]
fn sphere_lens_by_a_second_sphere_cut() {
    let mut r = Rig::cc();
    let b = r.brick([20.0; 3], [0.0; 3]);
    let (rad, d) = (2.0f64, 1.5f64);
    let sph = |x: f64| SurfaceDef::Sphere(Sphere::new(frame([x, 0.0, 0.0], [0.0, 0.0, 1.0]), rad).unwrap());
    let (host, first) = r.lib(|t| webcut(t, &[b.body], &CutTool::Surface { surface: sph(0.0), window: None }, &WebcutOptions::default())).unwrap();
    // Cut the ball by the second sphere in a fresh transaction on the edited topology.
    let mut host = host;
    let ball = first.negative[0];
    host.begin_op(sm_topo_api::OpId(99)).unwrap();
    let second = webcut(&mut *host, &[ball], &CutTool::Surface { surface: sph(d), window: None }, &WebcutOptions::default()).unwrap();
    host.end_op().unwrap();
    let t: &dyn TopologyView = &*host;
    let vol = |bs: &[EntityId]| -> f64 { bs.iter().map(|&b| t.boundary(b).iter().map(|e| volume_of(t, e.cell)).sum::<f64>()).sum() };
    let lens = PI * (4.0 * rad + d) * (2.0 * rad - d).powi(2) / 12.0;
    assert_eq!(second.negative.len(), 1);
    assert_eq!(second.positive.len(), 1);
    assert!((vol(&second.negative) - lens).abs() < 1e-9 * lens, "lens {} want {lens}", vol(&second.negative));
    assert!((vol(&second.negative) + vol(&second.positive) - 4.0 / 3.0 * PI * rad.powi(3)).abs() < 1e-9);
    let faces = |bs: &[EntityId]| -> std::collections::BTreeSet<EntityId> { bs.iter().flat_map(|&b| t.boundary(b).iter().flat_map(|e| t.volume_faces(e.cell).to_vec()).collect::<Vec<_>>()).collect() };
    assert_eq!(faces(&second.negative).intersection(&faces(&second.positive)).count(), 1, "one shared cut face");
    assert!(sm_topo_api::validate(t).is_empty());
}

/// A sphere tool on both halves of a brick cut through the sphere's centre (the pattern of the CSG
/// partition: one tool applied to every piece): each half gets its own cut face, and `merge`
/// welds the two hemispherical caps along the circle on the cut plane. The result is conformal
/// and valid, volumes are exact.
#[test]
fn a_sphere_tool_on_two_halves_merges_conformally() {
    use sm_boolean::ops::{AxisPlane, Merge, PlaneAxis, PlaneSpec, WebcutPlane, WebcutSurface};
    // Centres off the cut plane, on it (the circle runs through the poles), and on the other side.
    for centre in [[0.4, 0.2, 0.1], [0.0, 0.0, 0.0], [-0.4, 0.2, 0.1]] {
        let mut r = Rig::cc();
        let b = r.brick([6.0; 3], [0.0; 3]);
        let halves = r.run(&WebcutPlane { targets: vec![b.body], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: 0.0 }), keep: None, merge: None, imprint: None }).unwrap();
        let both: Vec<EntityId> = halves.outputs["positive"].iter().chain(&halves.outputs["negative"]).copied().collect();
        let s = r.sphere(1.5, centre);
        let d = r.run(&WebcutSurface { targets: both, face: s.faces[0], keep: None, merge: None, imprint: None }).unwrap_or_else(|e| panic!("{centre:?}: {e}"));
        r.run(&Merge { entities: None, kinds: None, tol: None, check_only: None }).unwrap();
        let t = r.topo();
        assert!(sm_topo_api::validate(t).is_empty() && sm_topo_api::validate_geometry(t).is_empty(), "{centre:?}");
        let vol = |role: &str| -> f64 { d.outputs[role].iter().map(|&b| t.boundary(b).iter().map(|e| volume_of(t, e.cell)).sum::<f64>()).sum() };
        assert!((vol("negative") - 4.0 / 3.0 * PI * 1.5f64.powi(3)).abs() < 1e-9, "{centre:?}: {}", vol("negative"));
        assert!((vol("negative") + vol("positive") - 216.0).abs() < 1e-9, "{centre:?}");
        assert_eq!(d.outputs["negative"].len(), 2, "{centre:?}");
        assert_eq!(d.outputs["positive"].len(), 2, "{centre:?}");
    }
}

/// Op-level scenario: a sphere, a torus and a cone face used as webcut tools on three bricks.
fn tools_scenario(r: &mut Rig) -> sm_op_api::Hash256 {
    use sm_boolean::ops::WebcutSurface;
    use sm_model::{CreateFrustum, CreateTorus};
    let b1 = r.brick([6.0; 3], [0.0; 3]);
    let s = r.sphere(1.5, [0.2, 0.0, 0.0]);
    r.run(&WebcutSurface { targets: vec![b1.body], face: s.faces[0], keep: None, merge: None, imprint: None }).unwrap();
    let b2 = r.brick([6.0; 3], [20.0, 0.0, 0.0]);
    let t = r.run(&CreateTorus { major: 2.0, minor: 0.5, center: Some([20.0, 0.0, 0.0]) }).unwrap();
    let tf = t.outputs["faces"][0];
    r.run(&WebcutSurface { targets: vec![b2.body], face: tf, keep: Some(KeepSide::Negative), merge: None, imprint: None }).unwrap();
    let b3 = r.brick([6.0; 3], [40.0, 0.0, 0.0]);
    let c = r.run(&CreateFrustum { height: 4.0, radius: 2.0, minor: None, top: Some(1.0), center: Some([40.0, 0.0, 0.0]) }).unwrap();
    let lat = *c.outputs["faces"].iter().find(|&&f| matches!(r.topo().carrier(f), sm_topo_api::Carrier::Surface(sc) if matches!(r.topo().shapes().surface_def(sc.surface), Some(SurfaceDef::Cone(_))))).unwrap();
    r.run(&sm_boolean::ops::WebcutSurface { targets: vec![b3.body], face: lat, keep: None, merge: Some(false), imprint: None }).unwrap();
    r.hash()
}

/// Two runs, one and several threads, and both topology stores give one state hash.
#[test]
fn native_tools_are_deterministic_across_runs_threads_and_hosts() {
    let run = |threads: usize, factory: Factory| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let mut r = Rig::new(factory);
            let h = tools_scenario(&mut r);
            (h, r.s.state().layer_hash("model").unwrap())
        })
    };
    let a = sm_testkit::determinism::check(2, 4, |t| run(t, cell_complex)).expect("deterministic");
    let b = sm_testkit::determinism::check(2, 4, |t| run(t, topo_store)).expect("deterministic");
    assert_eq!(a, b);
}

#[test]
fn deterministic() {
    let def = SurfaceDef::Torus(Torus::new(frame([0.1, 0.2, 0.0], [0.3, 0.1, 1.0]), 2.0, 0.5).unwrap());
    let a = cut(def.clone(), [6.0; 3], [0.0; 3], true).unwrap().hash;
    let b = cut(def, [6.0; 3], [0.0; 3], true).unwrap().hash;
    assert_eq!(a, b);
}

#[test]
fn webcut_surface_takes_a_sphere_face_and_a_cylinder_face() {
    use sm_boolean::ops::WebcutSurface;
    let mut r = Rig::cc();
    let brick = r.brick([6.0; 3], [0.0; 3]);
    let sph = r.sphere(1.5, [0.5, 0.0, 0.0]);
    let d = r.run(&WebcutSurface { targets: vec![brick.body], face: sph.faces[0], keep: None, merge: None, imprint: None }).unwrap();
    let t = r.topo();
    let vol = |role: &str| -> f64 { d.outputs[role].iter().map(|&b| t.boundary(b).iter().map(|e| volume_of(t, e.cell)).sum::<f64>()).sum() };
    assert!((vol("negative") - 4.0 / 3.0 * PI * 1.5f64.powi(3)).abs() < 1e-9);
    assert!((vol("positive") - (216.0 - 4.0 / 3.0 * PI * 1.5f64.powi(3))).abs() < 1e-9);
    assert!(sm_topo_api::validate(t).is_empty());
    // The lateral face of a cylinder: its outside is the positive side.
    let mut r = Rig::cc();
    let brick = r.brick([6.0; 3], [0.0; 3]);
    let cyl = r.cylinder(2.0, 1.0, [0.0, 0.0, -1.0]);
    let lat = *cyl.faces.iter().find(|&&f| matches!(r.topo().carrier(f), sm_topo_api::Carrier::Surface(sc) if matches!(r.topo().shapes().surface_def(sc.surface), Some(SurfaceDef::Cylinder(_))))).unwrap();
    let d = r.run(&WebcutSurface { targets: vec![brick.body], face: lat, keep: Some(KeepSide::Negative), merge: None, imprint: None }).unwrap();
    let t = r.topo();
    let v: f64 = d.outputs["negative"].iter().map(|&b| t.boundary(b).iter().map(|e| volume_of(t, e.cell)).sum::<f64>()).sum();
    assert!((v - PI * 1.0 * 6.0).abs() < 1e-9, "{v}");
    assert!(d.outputs.get("positive").is_none_or(|p| p.is_empty()));
}

#[test]
fn interior_points_are_inside() {
    use sm_boolean::{PointLocation, locate_point, volume_interior_point};
    let mut r = Rig::cc();
    let b = r.brick([6.0, 2.0, 1.0], [1.0, 2.0, 3.0]);
    let s = r.sphere(1.5, [10.0, 0.0, 0.0]);
    for v in [b.volume, s.volume] {
        let p = volume_interior_point(r.topo(), v).unwrap();
        assert_eq!(locate_point(r.topo(), v, p).unwrap(), PointLocation::In);
    }
    // A thin-walled tube.
    let mut r = Rig::cc();
    let c = r.cylinder(2.0, 0.5, [0.0; 3]);
    let d = r.run(&sm_boolean::ops::WebcutCylinder { targets: vec![c.body], radius: 0.49, axis: [0.0, 0.0, 1.0], center: [0.0; 3], keep: Some(KeepSide::Positive), merge: None, imprint: None }).unwrap();
    let tube = d.outputs["volumes"][0];
    let p = volume_interior_point(r.topo(), tube).unwrap();
    assert_eq!(locate_point(r.topo(), tube, p).unwrap(), PointLocation::In);
}

mod prop {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #![proptest_config(sm_testkit::strategies::config(24))]

        /// A random sphere crossing the bottom face of a wide brick: the part inside the brick is
        /// the spherical cap of height `r + c` (c is the centre height).
        #[test]
        fn sphere_cap_volumes(r in 0.5f64..2.0, c in -0.8f64..0.8, x in -2.0f64..2.0, y in -2.0f64..2.0, tx in -4.0f64..4.0, ty in -4.0f64..4.0) {
            // Any axis direction: loops that wrap the axis and loops that do not (the sphere's seam
            // and poles end up anywhere relative to the cut).
            let c = c * r;
            let def = SurfaceDef::Sphere(Sphere::new(frame([x, y, c], [tx, ty, 1.0]), r).unwrap());
            let h = r + c;
            let want = PI * h * h * (3.0 * r - h) / 3.0;
            let k = cut(def, [12.0, 12.0, 6.0], [0.0, 0.0, 3.0], true).unwrap();
            // The sphere's frame is re-chosen away from the cut circles, so any axis gives 1e-9.
            prop_assert!((k.inside - want).abs() < 1e-9 * want.max(1.0), "{} vs {}", k.inside, want);
            prop_assert!((k.inside + k.outside - 864.0).abs() < 1e-8);
            prop_assert!(k.shared >= 1);
        }

        /// A random torus fully inside a brick, any orientation.
        #[test]
        fn torus_volumes(big in 1.0f64..2.0, ratio in 0.15f64..0.6, tx in -1.0f64..1.0, ty in -1.0f64..1.0, x in -1.0f64..1.0, y in -1.0f64..1.0, z in -1.0f64..1.0) {
            let small = big * ratio;
            let def = SurfaceDef::Torus(Torus::new(frame([x, y, z], [tx, ty, 1.0]), big, small).unwrap());
            let want = 2.0 * PI * PI * big * small * small;
            let k = cut(def, [12.0; 3], [0.0; 3], true).unwrap();
            prop_assert!((k.inside - want).abs() < 1e-9 * want, "{} vs {}", k.inside, want);
            prop_assert!((k.inside + k.outside - 1728.0).abs() < 1e-7);
            prop_assert_eq!(k.shared, 1);
        }

        /// A tilted elliptic cylinder crossing the top and bottom faces: volume pi a b L / cos(tilt).
        #[test]
        fn elliptic_cylinder_volumes(a in 0.3f64..1.0, b in 0.3f64..1.0, tx in -0.4f64..0.4, ty in -0.4f64..0.4, x in -1.0f64..1.0, y in -1.0f64..1.0) {
            let axis = Vector3::new(tx, ty, 1.0);
            let cos = axis.z / axis.norm();
            let h = 4.0;
            let want = PI * a * b * h / cos;
            let def = SurfaceDef::EllipticCylinder(EllipticCylinder::new(frame([x, y, 0.0], [tx, ty, 1.0]), a, b).unwrap());
            let k = cut(def, [30.0, 30.0, h], [0.0; 3], true).unwrap();
            prop_assert!((k.inside - want).abs() < 1e-9 * want, "{} vs {}", k.inside, want);
            prop_assert!((k.inside + k.outside - 30.0 * 30.0 * h).abs() < 1e-7);
        }
    }
}
