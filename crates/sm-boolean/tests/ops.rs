//! Op-level acceptance: webcut plane forms, roles, lineage through the model tables, failure
//! atomicity.

mod common;

use common::rig::*;
use common::*;
use sm_boolean::KeepSide;
use sm_boolean::ops::{AxisPlane, Intersect, PlaneAxis, PlaneSpec, PointNormalPlane, Subtract, Unite, VerticesPlane, WebcutPlane, WebcutSurface};
use sm_core::{EntityId, EntityKind};
use sm_op_api::{Delta, OpError, TimelineError};
use sm_topo_api::{Carrier, Dim, TopologyViewExt};

fn cube(r: &mut Rig) -> Solid {
    r.brick([1.0; 3], [0.5; 3])
}

fn cut(r: &mut Rig, body: EntityId, plane: PlaneSpec, keep: Option<KeepSide>, merge: Option<bool>) -> Delta {
    r.run(&WebcutPlane { targets: vec![body], plane, keep, merge, imprint: None }).unwrap()
}

fn z(v: f64) -> PlaneSpec {
    PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::Z, offset: v })
}

fn vol_of_body(r: &Rig, b: EntityId) -> f64 {
    let t = r.topo();
    volume_of(t, t.boundary(b)[0].cell)
}

fn failed(e: TimelineError) -> OpError {
    match e {
        TimelineError::Apply(ap) => match ap {
            sm_op_api::ApplyError::Op { source, .. } => source,
            other => panic!("{other:?}"),
        },
        other => panic!("{other:?}"),
    }
}

#[test]
fn webcut_plane_every_form_gives_the_same_cut() {
    let origin_normal = PlaneSpec::PointNormal(PointNormalPlane { origin: [0.0, 0.0, 0.25], normal: [0.0, 0.0, 1.0] });
    let slanted = PlaneSpec::PointNormal(PointNormalPlane { origin: [0.3, 0.1, 0.25], normal: [0.0, 0.0, 7.0] });
    let forms = vec![z(0.25), origin_normal, slanted];
    for spec in forms {
        let mut r = Rig::cc();
        let c = cube(&mut r);
        let d = cut(&mut r, c.body, spec, None, None);
        let t = r.topo();
        assert_valid(t);
        assert_eq!(counts(t), [12, 20, 11, 2, 2]);
        assert!((total_area(t) - 7.0).abs() < 1e-12);
        assert_eq!(d.outputs["cut_faces"].len(), 1);
        assert_eq!(d.outputs["negative"].len(), 1);
        assert_eq!(d.outputs["positive"].len(), 1);
        assert!(!d.outputs.contains_key("bodies"));
        assert!((vol_of_body(&r, d.outputs["negative"][0]) - 0.25).abs() < 1e-12);
        assert!((vol_of_body(&r, d.outputs["positive"][0]) - 0.75).abs() < 1e-12);
    }
}

#[test]
fn webcut_with_three_vertices_and_reversed_order() {
    // A second brick supplies vertices at z = 0.25 only through a webcut of itself.
    let mut r = Rig::cc();
    let helper = r.brick([1.0; 3], [10.5, 0.5, 0.5]);
    let h = cut(&mut r, helper.body, z(0.25), None, None);
    let t = r.topo();
    let pts: Vec<(EntityId, [f64; 3])> = t.cells(Dim::Vertex).iter().filter_map(|&v| match t.carrier(v) { Carrier::Point(p) if (p.z - 0.25).abs() < 1e-12 => Some((v, [p.x, p.y, p.z])), _ => None }).collect();
    assert!(pts.len() >= 4, "{pts:?}");
    let _ = h;
    // v0 = (10, 0, .25), v1 = (11, 0, .25), v2 = (10, 1, .25): normal +z.
    let find = |x: f64, y: f64| pts.iter().find(|(_, p)| (p[0] - x).abs() < 1e-12 && (p[1] - y).abs() < 1e-12).map(|p| p.0).unwrap();
    let (v0, v1, v2) = (find(10.0, 0.0), find(11.0, 0.0), find(10.0, 1.0));
    let c = cube(&mut r);
    let d = cut(&mut r, c.body, PlaneSpec::Vertices(VerticesPlane { vertices: [v0, v1, v2] }), None, None);
    assert!((vol_of_body(&r, d.outputs["negative"][0]) - 0.25).abs() < 1e-12);
    // Reversed order flips only the roles.
    let mut r2 = Rig::cc();
    let helper = r2.brick([1.0; 3], [10.5, 0.5, 0.5]);
    cut(&mut r2, helper.body, z(0.25), None, None);
    let c2 = cube(&mut r2);
    let d2 = cut(&mut r2, c2.body, PlaneSpec::Vertices(VerticesPlane { vertices: [v0, v2, v1] }), None, None);
    assert!((vol_of_body(&r2, d2.outputs["positive"][0]) - 0.25).abs() < 1e-12);
    assert!((vol_of_body(&r2, d2.outputs["negative"][0]) - 0.75).abs() < 1e-12);
}

#[test]
fn webcut_surface_with_a_planar_face_matches_the_plane() {
    let mut r = Rig::cc();
    let c = cube(&mut r);
    // A thin brick far away whose top face lies in z = 0.25 with its normal along +z.
    let tool = r.brick([2.0, 2.0, 0.5], [10.0, 0.0, 0.0]);
    let top = tool.faces[5];
    let d = r.run(&WebcutSurface { targets: vec![c.body], face: top, keep: None, merge: None, imprint: None }).unwrap();
    let t = r.topo();
    assert_valid(t);
    assert_eq!(counts(t)[..3], [12 + 8, 20 + 12, 11 + 6]);
    assert!((vol_of_body(&r, d.outputs["negative"][0]) - 0.25).abs() < 1e-12);
    assert!((vol_of_body(&r, d.outputs["positive"][0]) - 0.75).abs() < 1e-12);
    assert!((r.topo().face_moments(d.outputs["cut_faces"][0]).unwrap().area - 1.0).abs() < 1e-12);
    // A curved face is refused with a typed error.
    let cyl = r.cylinder(2.0, 1.0, [20.0, 0.0, 0.0]);
    let side = cyl.faces[2];
    let e = r.run(&WebcutSurface { targets: vec![c.body], face: side, keep: None, merge: None, imprint: None });
    assert!(e.is_err());
}

#[test]
fn keep_positive_and_nomerge_through_the_op() {
    let mut r = Rig::cc();
    let c = cube(&mut r);
    let d = cut(&mut r, c.body, z(0.25), Some(KeepSide::Positive), None);
    assert_eq!(d.outputs["positive"].len(), 1);
    assert!(!d.outputs.contains_key("negative") || d.outputs["negative"].is_empty());
    let t = r.topo();
    assert_valid(t);
    assert_eq!(counts(t)[3], 1);
    assert!(t.is_boundary_face(d.outputs["cut_faces"][0]));
    let mut r = Rig::cc();
    let c = cube(&mut r);
    let d = cut(&mut r, c.body, z(0.25), None, Some(false));
    assert_eq!(counts(r.topo())[2], 12);
    assert_eq!(d.outputs["cut_faces"].len(), 2);
    assert_valid(r.topo());
}

#[test]
fn colors_and_groups_follow_the_lineage_of_a_unite() {
    let mut r = Rig::cc();
    let a = r.brick([2.0; 3], [1.0; 3]);
    let b = r.brick([2.0; 3], [2.0, 1.0, 1.0]);
    r.run(&Tag { volumes: vec![a.volume], label: 5 }).unwrap();
    let before = r.layer().model().volume_color(a.volume).expect("colored");
    let d = r.run(&Unite { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap();
    let v = d.outputs["volumes"][0];
    let m = r.layer().model();
    assert_eq!(m.volume_color(v), Some(before), "the color of A's volume reaches the result");
    let g = m.group_by_name("g5").unwrap();
    assert!(m.group(g).unwrap().members.contains(&v), "the group member follows the lineage");
    assert_eq!(d.removed.iter().filter(|e| e.kind() == EntityKind::TopoVolume).count(), 2);
}

#[test]
fn a_color_on_the_outside_volume_survives_the_unite() {
    let mut r = Rig::cc();
    let x = r.brick([1.0; 3], [0.5, 0.5, 0.5]);
    let y = r.brick([1.0; 3], [1.5, 0.5, 0.5]);
    let z3 = r.brick([1.0; 3], [2.5, 0.5, 0.5]);
    r.run(&sm_boolean::ops::Merge { entities: None, kinds: None, tol: None, check_only: None }).unwrap();
    r.run(&Tag { volumes: vec![z3.volume], label: 9 }).unwrap();
    let col = r.layer().model().volume_color(z3.volume).unwrap();
    let d = r.run(&Unite { bodies: vec![x.body, y.body], keep: None, regularize: None, tol: None }).unwrap();
    let u = d.outputs["volumes"][0];
    let t = r.topo();
    assert_valid(t);
    assert!((volume_of(t, u) - 2.0).abs() < 1e-12);
    assert_eq!(r.layer().model().volume_color(z3.volume), Some(col));
    // The union keeps the interface with Z two-sided.
    let shared: Vec<_> = t.cells(Dim::Face).iter().copied().filter(|&f| t.is_interface_face(f)).collect();
    assert_eq!(shared.len(), 1);
    assert_eq!(t.volume_faces(u).len(), 6);
    assert!(t.volume_faces(z3.volume).contains(&shared[0]));
}

#[test]
fn failing_ops_leave_the_state_hash_unchanged() {
    // Disjoint intersect.
    let mut r = Rig::cc();
    let a = r.brick([1.0; 3], [0.5; 3]);
    let b = r.brick([1.0; 3], [5.5, 0.5, 0.5]);
    let h = r.hash();
    let e = r.run(&Intersect { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap_err();
    assert!(failed(e).to_string().contains("empty"), "EmptyResult text");
    assert_eq!(r.hash(), h);
    // Disjoint unite.
    let e = r.run(&Unite { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap_err();
    assert!(matches!(failed(e), OpError::Invalid(_)));
    assert_eq!(r.hash(), h);
    // A tool that is only tangent: a sphere touching a brick face at a point.
    let s = r.sphere(0.5, [0.5, 0.5, 1.5]);
    let h = r.hash();
    let e = r.run(&Unite { bodies: vec![a.body, s.body], keep: None, regularize: None, tol: None }).unwrap_err();
    assert!(matches!(failed(e), OpError::Unsupported(_)), "tangent-only contact is Unsupported");
    assert_eq!(r.hash(), h);
    // An input with a mesh.
    r.run(&FakeMesh { volume: a.volume }).unwrap();
    let h = r.hash();
    let e = r.run(&Subtract { targets: vec![a.body], tools: vec![b.body], keep: None, regularize: None, tol: None }).unwrap_err();
    assert!(failed(e).to_string().contains("mesh.delete first"));
    assert_eq!(r.hash(), h);
    let e = r.run(&WebcutPlane { targets: vec![a.body], plane: z(0.5), keep: None, merge: None, imprint: None }).unwrap_err();
    assert!(failed(e).to_string().contains("mesh.delete first"));
    assert_eq!(r.hash(), h);
}

#[test]
fn without_keep_the_inputs_are_retired_and_with_keep_they_are_not() {
    let mut r = Rig::cc();
    let a = r.brick([2.0; 3], [1.0; 3]);
    let b = r.brick([2.0; 3], [2.0, 1.0, 1.0]);
    let d = r.run(&Unite { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap();
    for id in [a.volume, b.volume, a.body, b.body] {
        assert!(d.removed.contains(&id), "{id} retired");
    }
    let mut r = Rig::cc();
    let a = r.brick([2.0; 3], [1.0; 3]);
    let b = r.brick([2.0; 3], [2.0, 1.0, 1.0]);
    let d = r.run(&Unite { bodies: vec![a.body, b.body], keep: Some(true), regularize: None, tol: None }).unwrap();
    for id in [a.volume, b.volume, a.body, b.body] {
        assert!(!d.removed.contains(&id), "{id} kept");
    }
    assert!(d.outputs["volumes"].iter().all(|v| *v != a.volume && *v != b.volume));
}
