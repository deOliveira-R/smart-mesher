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

#[test]
fn deterministic() {
    let def = SurfaceDef::Torus(Torus::new(frame([0.1, 0.2, 0.0], [0.3, 0.1, 1.0]), 2.0, 0.5).unwrap());
    let a = cut(def.clone(), [6.0; 3], [0.0; 3], true).unwrap().hash;
    let b = cut(def, [6.0; 3], [0.0; 3], true).unwrap().hash;
    assert_eq!(a, b);
}
