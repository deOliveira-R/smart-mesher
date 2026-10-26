//! Booleans with quadrics through the ops (acceptance 6-8).

mod common;

use common::rig::*;
use common::*;
use core::f64::consts::PI;
use sm_boolean::ops::{Intersect, Subtract, Unite};
use sm_topo_api::TopologyViewExt;

#[test]
fn brick_minus_through_cylinder() {
    let mut r = Rig::cc();
    let b = r.brick([4.0; 3], [2.0; 3]);
    let c = r.cylinder(6.0, 1.0, [2.0, 2.0, 2.0]);
    let d = r.run(&Subtract { targets: vec![b.body], tools: vec![c.body], keep: None, regularize: None, tol: None }).unwrap();
    let t = r.topo();
    assert_valid(t);
    assert_eq!(d.outputs["volumes"].len(), 1);
    let v = d.outputs["volumes"][0];
    assert!((volume_of(t, v) - (64.0 - 4.0 * PI)).abs() < 1e-8 * 64.0, "{}", volume_of(t, v));
    assert!((total_area(t) - (96.0 + 6.0 * PI)).abs() < 1e-8 * 100.0);
    assert_eq!(counts(t), [10, 15, 7, 1, 1]);
    assert_eq!(euler(t), 2);
    // Top and bottom carry an inner loop (area 16 - pi); the lateral face points at the axis.
    let holed: Vec<_> = t.volume_faces(v).iter().filter(|&&f| t.loops(f).count() == 2).collect();
    assert_eq!(holed.len(), 2);
    for &&f in &holed {
        assert!((t.face_moments(f).unwrap().area - (16.0 - PI)).abs() < 1e-8);
    }
    let lateral = t.volume_faces(v).iter().copied().find(|f| t.loops(*f).count() == 1 && matches!(t.carrier(*f), sm_topo_api::Carrier::Surface(sc) if matches!(t.shapes().surface_def(sc.surface), Some(sm_geom_api::SurfaceDef::Cylinder(_))))).expect("lateral face");
    let sm_topo_api::Carrier::Surface(sc) = t.carrier(lateral).clone() else { unreachable!() };
    let surf = t.shapes().surface(sc.surface).unwrap();
    let probe = sm_core::Point3::new(3.0, 2.0, 2.0);
    let uv = surf.project(probe, None).unwrap().uv;
    let use_sense = t.face_uses(v).find(|u| u.face == lateral).expect("use").sense;
    let outward = surf.normal(uv).unwrap() * (sc.sense.sign() * use_sense.sign());
    assert!(outward.dot(sm_core::Vector3::new(-1.0, 0.0, 0.0)) > 0.99, "outward normal of the solid points toward the axis: {outward:?}");
}

fn sub(r: &mut Rig, t: &[sm_core::EntityId], tools: &[sm_core::EntityId]) -> Result<sm_op_api::Delta, sm_op_api::TimelineError> {
    r.run(&Subtract { targets: t.to_vec(), tools: tools.to_vec(), keep: None, regularize: None, tol: None })
}

#[test]
fn flush_cylinder_gives_the_same_result() {
    let mut r = Rig::cc();
    let b = r.brick([4.0; 3], [2.0; 3]);
    let c = r.cylinder(4.0, 1.0, [2.0, 2.0, 2.0]);
    let d = sub(&mut r, &[b.body], &[c.body]).unwrap();
    let t = r.topo();
    assert_valid(t);
    let v = d.outputs["volumes"][0];
    assert!((volume_of(t, v) - (64.0 - 4.0 * PI)).abs() < 1e-8 * 64.0);
    assert!((total_area(t) - (96.0 + 6.0 * PI)).abs() < 1e-8 * 100.0);
    assert_eq!(counts(t), [10, 15, 7, 1, 1]);
}

#[test]
fn brick_minus_interior_sphere() {
    let mut r = Rig::cc();
    let b = r.brick([10.0; 3], [5.0; 3]);
    let s = r.sphere(2.0, [5.0; 3]);
    let d = sub(&mut r, &[b.body], &[s.body]).unwrap();
    let t = r.topo();
    assert_valid(t);
    let v = d.outputs["volumes"][0];
    assert!((volume_of(t, v) - (1000.0 - 32.0 * PI / 3.0)).abs() < 1e-8 * 1000.0, "{}", volume_of(t, v));
    assert_eq!(counts(t), [10, 13, 7, 1, 1]);
    assert_eq!(euler(t), 4);
    use sm_topo_api::ShellKind;
    assert_eq!(t.shell_kind(v, 0), Some(ShellKind::Outer));
    assert_eq!(t.shell_kind(v, 1), Some(ShellKind::Void));
}

#[test]
fn intersect_cylinder_inside_brick_returns_the_cylinder() {
    let mut r = Rig::cc();
    let b = r.brick([4.0; 3], [2.0; 3]);
    let c = r.cylinder(2.0, 1.0, [2.0, 2.0, 2.0]);
    let d = r.run(&Intersect { bodies: vec![b.body, c.body], keep: None, regularize: None, tol: None }).unwrap();
    let t = r.topo();
    assert_valid(t);
    let v = d.outputs["volumes"][0];
    assert!((volume_of(t, v) - 2.0 * PI).abs() < 1e-8 * 7.0);
    assert_eq!(counts(t)[3], 1);
}

#[test]
fn unite_brick_and_sphere_half() {
    let mut r = Rig::cc();
    let b = r.brick([2.0; 3], [1.0; 3]);
    let s = r.sphere(1.0, [2.0, 1.0, 1.0]);
    let res = r.run(&Unite { bodies: vec![b.body, s.body], keep: None, regularize: None, tol: None });
    let d = res.unwrap();
    let t = r.topo();
    assert_valid(t);
    let v = d.outputs["volumes"][0];
    assert!((volume_of(t, v) - (8.0 + 2.0 * PI / 3.0)).abs() < 1e-8 * 9.0);
    assert!((total_area(t) - (24.0 + PI)).abs() < 1e-8 * 30.0);
}

fn plane_z(z: f64) -> sm_boolean::ops::PlaneSpec {
    sm_boolean::ops::PlaneSpec::Axis(sm_boolean::ops::AxisPlane { axis: sm_boolean::ops::PlaneAxis::Z, offset: z })
}

fn cut(r: &mut Rig, body: sm_core::EntityId, spec: sm_boolean::ops::PlaneSpec) -> sm_op_api::Delta {
    r.run(&sm_boolean::ops::WebcutPlane { targets: vec![body], plane: spec, keep: None, merge: None, imprint: None }).unwrap()
}

#[test]
fn webcut_cylinder_at_height() {
    let mut r = Rig::cc();
    let c = r.cylinder(2.0, 1.0, [0.0, 0.0, 1.0]);
    let d = cut(&mut r, c.body, plane_z(0.5));
    let t = r.topo();
    assert_valid(t);
    let vols = &d.outputs["volumes"];
    assert_eq!(vols.len(), 2);
    let mut v: Vec<f64> = vols.iter().map(|&x| volume_of(t, x)).collect();
    v.sort_by(f64::total_cmp);
    assert!((v[0] - PI * 0.5).abs() < 1e-8 && (v[1] - PI * 1.5).abs() < 1e-8, "{v:?}");
    let cap = d.outputs["cut_faces"][0];
    assert!((t.face_moments(cap).unwrap().area - PI).abs() < 1e-8);
    let mut lat: Vec<f64> = t.cells(sm_topo_api::Dim::Face).iter().filter(|&&f| t.carrier(f) != t.carrier(cap) && t.face_moments(f).unwrap().area > 0.0).filter(|&&f| matches!(t.shapes().surface_def(match t.carrier(f) { sm_topo_api::Carrier::Surface(s) => s.surface, _ => unreachable!() }), Some(sm_geom_api::SurfaceDef::Cylinder(_)))).map(|&f| t.face_moments(f).unwrap().area).collect();
    lat.sort_by(f64::total_cmp);
    assert!((lat[0] - 2.0 * PI * 0.5).abs() < 1e-8 && (lat[1] - 2.0 * PI * 1.5).abs() < 1e-8, "{lat:?}");
}

#[test]
fn webcut_sphere_at_equator() {
    let mut r = Rig::cc();
    let s = r.sphere(1.0, [0.0; 3]);
    let d = cut(&mut r, s.body, plane_z(0.0));
    let t = r.topo();
    assert_valid(t);
    let vols = &d.outputs["volumes"];
    assert_eq!(vols.len(), 2);
    for &v in vols {
        assert!((volume_of(t, v) - 2.0 * PI / 3.0).abs() < 1e-8);
    }
    let cap = d.outputs["cut_faces"][0];
    assert!((t.face_moments(cap).unwrap().area - PI).abs() < 1e-8);
}

#[test]
fn webcut_cylinder_through_brick() {
    let mut r = Rig::cc();
    let b = r.brick([4.0; 3], [2.0; 3]);
    let d = r.run(&sm_boolean::ops::WebcutCylinder { targets: vec![b.body], radius: 1.0, axis: [0.0, 0.0, 1.0], center: [2.0, 2.0, 2.0], keep: None, merge: None, imprint: None }).unwrap();
    let t = r.topo();
    assert_valid(t);
    let vols = &d.outputs["volumes"];
    assert_eq!(vols.len(), 2);
    let mut v: Vec<f64> = vols.iter().map(|&x| volume_of(t, x)).collect();
    v.sort_by(f64::total_cmp);
    assert!((v[0] - 4.0 * PI).abs() < 1e-8 && (v[1] - (64.0 - 4.0 * PI)).abs() < 1e-8, "{v:?}");
    assert_eq!(d.outputs["cut_faces"].len(), 1);
    let cap = d.outputs["cut_faces"][0];
    assert!((t.face_moments(cap).unwrap().area - 8.0 * PI).abs() < 1e-8);
    assert_eq!(counts(t)[..3], [10, 15, 9]);
    // Positive is the outside of the cylinder.
    let pos = d.outputs["positive"][0];
    let pv = t.boundary(pos)[0].cell;
    assert!((volume_of(t, pv) - (64.0 - 4.0 * PI)).abs() < 1e-8);
}

#[test]
fn unite_brick_and_sphere_half_unregularized() {
    let mut r = Rig::cc();
    let b = r.brick([2.0; 3], [1.0; 3]);
    let s = r.sphere(1.0, [2.0, 1.0, 1.0]);
    let d = r.run(&Unite { bodies: vec![b.body, s.body], keep: None, regularize: Some(false), tol: None }).unwrap();
    let t = r.topo();
    assert_valid(t);
    let v = d.outputs["volumes"][0];
    assert!((volume_of(t, v) - (8.0 + 2.0 * PI / 3.0)).abs() < 1e-8 * 9.0);
}
