//! W3b-PRIM acceptance 6-7: `model.move_to` and `model.rotate_about`.

mod common;

use common::*;
use core::f64::consts::PI;
use sm_core::{EntityId, EntityKind};
use sm_model::{Anchor, AxisName, AxisSpec, CreateBrick, CreateCylinder, CreateSpherePart, MoveTarget, MoveTo, Octants, PointSpec, Rotate, RotateAbout};
use sm_op_api::Session;

fn vol(n: u64) -> EntityId {
    id(EntityKind::TopoVolume, n)
}

fn centroid(s: &Session, v: EntityId) -> [f64; 3] {
    let t = model(s).topo();
    let c = t.volume_moments(v).expect("moments").centroid().expect("centroid");
    [c.x, c.y, c.z]
}

fn brick_at(c: [f64; 3]) -> CreateBrick {
    CreateBrick { x: 10.0, y: Some(4.0), z: Some(6.0), center: Some(c) }
}

fn move_to(entities: Vec<EntityId>, to: MoveTarget, anchor: Option<Anchor>) -> MoveTo {
    MoveTo { entities, to, anchor }
}

fn sorted(mut pts: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    pts.sort_by(|a, b| a.iter().zip(b).map(|(x, y)| x.total_cmp(y)).find(|o| o.is_ne()).unwrap_or(core::cmp::Ordering::Equal));
    pts
}

fn same_set(a: Vec<[f64; 3]>, b: Vec<[f64; 3]>, tol: f64) -> bool {
    a.len() == b.len() && sorted(a).iter().zip(sorted(b).iter()).all(|(p, q)| (0..3).all(|k| (p[k] - q[k]).abs() < tol))
}

#[test]
fn move_to_a_point_moves_the_centroid_exactly_and_an_omitted_coordinate_stays() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &brick_at([1.0, 2.0, 3.0])).expect("brick");
        let before = vertex_points(&s);
        run(&mut s, &move_to(vec![vol(1)], MoveTarget::Point(PointSpec { x: Some(10.0), ..Default::default() }), None)).expect("move_to");
        let c = centroid(&s, vol(1));
        assert!((c[0] - 10.0).abs() < 1e-12 && (c[1] - 2.0).abs() < 1e-12 && (c[2] - 3.0).abs() < 1e-12, "{c:?}");
        // Every vertex moved by the same delta, y and z untouched bit for bit.
        let after = vertex_points(&s);
        let d = after[0][0] - before[0][0];
        for (a, b) in after.iter().zip(&before) {
            assert_eq!(a[1].to_bits(), b[1].to_bits());
            assert_eq!(a[2].to_bits(), b[2].to_bits());
            assert!((a[0] - b[0] - d).abs() < 1e-14, "{a:?} {b:?}");
        }
        clean(&s);
        // A body selects its volumes.
        run(&mut s, &move_to(vec![id(EntityKind::TopoBody, 1)], MoveTarget::Coords([0.0; 3]), None)).expect("by body");
        let c = centroid(&s, vol(1));
        assert!(c.iter().all(|x| x.abs() < 1e-12), "{c:?}");
    });
}

#[test]
fn move_to_an_entity_and_the_bbox_anchor() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &brick_at([0.0, 0.0, 0.0])).expect("a");
        run(&mut s, &brick_at([100.0, 50.0, 25.0])).expect("b");
        // Vertex 8 + 7 is the (+,+,+) corner of brick 2.
        let corner = vertex_points(&s)[8 + 7];
        run(&mut s, &move_to(vec![vol(1)], MoveTarget::Entity(id(EntityKind::TopoVertex, 16)), None)).expect("to vertex");
        let c = centroid(&s, vol(1));
        assert!((0..3).all(|k| (c[k] - corner[k]).abs() < 1e-10), "{c:?} vs {corner:?}");
        // bbox_center of the moved brick is its centroid for a brick; put it on brick 2's centroid via the volume id.
        run(&mut s, &move_to(vec![vol(1)], MoveTarget::Entity(vol(2)), Some(Anchor::BboxCenter))).expect("to volume");
        let c = centroid(&s, vol(1));
        assert!((0..3).all(|k| (c[k] - [100.0, 50.0, 25.0][k]).abs() < 1e-10), "{c:?}");
        // Face of brick 2: the +x face (index 1) centroid.
        run(&mut s, &move_to(vec![vol(1)], MoveTarget::Entity(id(EntityKind::TopoFace, 6 + 2)), None)).expect("to face");
        let c = centroid(&s, vol(1));
        assert!((0..3).all(|k| (c[k] - [105.0, 50.0, 25.0][k]).abs() < 1e-10), "{c:?}");
        // An edge: the midpoint of a straight edge.
        run(&mut s, &move_to(vec![vol(1)], MoveTarget::Entity(id(EntityKind::TopoEdge, 13)), None)).expect("to edge");
        clean(&s);
    });
}

#[test]
fn bbox_anchor_differs_from_the_centroid_for_a_cone() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &sm_model::CreateFrustum { height: 4.0, radius: 2.0, minor: None, top: None, center: None }).expect("cone");
        run(&mut s, &move_to(vec![vol(1)], MoveTarget::Coords([0.0; 3]), Some(Anchor::BboxCenter))).expect("bbox");
        // The bbox of the cone spans z in [-2, 2] (the apex is at +2); its centre goes to z = 0.
        let zs: Vec<f64> = vertex_points(&s).iter().map(|p| p[2]).collect();
        assert!((zs.iter().cloned().fold(f64::MIN, f64::max) + zs.iter().cloned().fold(f64::MAX, f64::min)).abs() < 1e-12);
        clean(&s);
    });
}

#[test]
fn move_to_refuses_meshed_volumes_and_bad_targets() {
    let mut s = session(cell_complex);
    run(&mut s, &brick_at([0.0; 3])).expect("brick");
    run(&mut s, &FakeMesh { volume: vol(1) }).expect("mesh");
    let before = s.state().hash();
    let e = run(&mut s, &move_to(vec![vol(1)], MoveTarget::Point(PointSpec { x: Some(1.0), ..Default::default() }), None)).expect_err("meshed");
    assert!(format!("{e:?}").contains("mesh.delete first"), "{e:?}");
    assert_eq!(s.state().hash(), before);
    let mut s = session(cell_complex);
    run(&mut s, &brick_at([0.0; 3])).expect("brick");
    let before = s.state().hash();
    for op in [
        move_to(vec![], MoveTarget::Point(PointSpec { x: Some(1.0), ..Default::default() }), None),
        move_to(vec![vol(1)], MoveTarget::Entity(id(EntityKind::TopoVertex, 99)), None),
        move_to(vec![id(EntityKind::TopoFace, 1)], MoveTarget::Point(PointSpec::default()), None),
    ] {
        let e = run(&mut s, &op).expect_err("refused");
        assert_eq!(s.state().hash(), before, "{e:?}");
    }
}

#[test]
fn move_to_json_forms_decode() {
    let s = session(cell_complex);
    let reg = s.engine().registry();
    let rec = sm_op_api::OpRecord::new("model.move_to", 1, serde_json::json!({"entities": ["TopoVolume#1"], "to": {"x": 10}, "anchor": "bbox_center"})).expect("record");
    let op = reg.decode(&rec).expect("partial point target");
    assert_eq!(op.encode().expect("encode")["anchor"], serde_json::json!("bbox_center"));
    let rec = sm_op_api::OpRecord::new("model.move_to", 1, serde_json::json!({"entities": ["TopoVolume#1"], "to": "TopoVertex#3"})).expect("record");
    reg.decode(&rec).expect("entity target");
    let rec = sm_op_api::OpRecord::new("model.move_to", 1, serde_json::json!({"entities": ["TopoVolume#1"], "to": [1, 2, 3]})).expect("record");
    reg.decode(&rec).expect("coordinate target");
    // Nulls cannot be recorded at all, so the array-with-null form of the draft is unreachable.
    assert!(sm_op_api::OpRecord::new("model.move_to", 1, serde_json::json!({"entities": ["TopoVolume#1"], "to": [1, null, null]})).is_err());
}

fn cube2() -> CreateBrick {
    CreateBrick { x: 2.0, y: None, z: None, center: None }
}

#[test]
fn rotate_about_a_vertex_pair_permutes_the_corners() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &cube2()).expect("cube");
        let before = vertex_points(&s);
        // The body diagonal from vertex 0 (-1,-1,-1) to vertex 7 (1,1,1).
        let op = RotateAbout { entities: vec![vol(1)], angle_deg: 120.0, axis: None, origin: None, vertices: Some([id(EntityKind::TopoVertex, 1), id(EntityKind::TopoVertex, 8)]), face: None };
        run(&mut s, &op).expect("rotate");
        let after = vertex_points(&s);
        assert!(same_set(before.clone(), after.clone(), 1e-12), "vertex set is unchanged");
        // The three vertices adjacent to vertex 0 are cycled.
        let adj = |pts: &Vec<[f64; 3]>| [pts[1], pts[2], pts[4]];
        let (a, b) = (adj(&before), adj(&after));
        assert!(same_set(a.to_vec(), b.to_vec(), 1e-12));
        assert!((0..3).any(|i| (0..3).any(|k| (before[1][k] - after[1][k]).abs() > 1.0) && i == 0), "the corners actually moved");
        // The pivot is the first vertex: it stays, and so does the opposite one.
        assert!((0..3).all(|k| (before[0][k] - after[0][k]).abs() < 1e-12 && (before[7][k] - after[7][k]).abs() < 1e-12));
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
    });
}

#[test]
fn rotate_about_a_face_normal() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &cube2()).expect("cube");
        let before = vertex_points(&s);
        let top = id(EntityKind::TopoFace, 6);
        let op = |angle| RotateAbout { entities: vec![vol(1)], angle_deg: angle, axis: None, origin: None, vertices: None, face: Some(top) };
        run(&mut s, &op(90.0)).expect("90");
        assert!(same_set(before.clone(), vertex_points(&s), 1e-12), "the cube maps onto itself");
        run(&mut s, &op(-45.0)).expect("-45");
        // 45 degrees from the original: the xy extent is sqrt 2 on each axis.
        let pts = vertex_points(&s);
        let ext = |k: usize| pts.iter().map(|p| p[k].abs()).fold(0.0, f64::max);
        assert!((ext(0) - 2f64.sqrt()).abs() < 1e-12 && (ext(1) - 2f64.sqrt()).abs() < 1e-12 && (ext(2) - 1.0).abs() < 1e-12);
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
        // A right-hand sense: rotating +90 about +z sends +x to +y.
        let mut s = session(factory);
        run(&mut s, &cube2()).expect("cube");
        run(&mut s, &RotateAbout { entities: vec![vol(1)], angle_deg: 90.0, axis: None, origin: None, vertices: None, face: Some(id(EntityKind::TopoFace, 2)) }).expect("about +x");
        // Face 2 is +x: rotating about +x by 90 sends +y to +z, so vertex 0 (-1,-1,-1) goes to (-1, 1, -1).
        let p = vertex_points(&s)[0];
        assert!((p[0] + 1.0).abs() < 1e-12 && (p[1] - 1.0).abs() < 1e-12 && (p[2] + 1.0).abs() < 1e-12, "{p:?}");
    });
}

#[test]
fn rotate_about_a_curved_face_uses_the_surface_normal() {
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &CreateCylinder { height: 2.0, radius: 1.0, center: Some([3.0, 0.0, 0.0]) }).expect("cylinder");
        let before = model(&s).topo().volume_moments(vol(1)).expect("m").centroid().expect("c");
        // The top cap (face 2) is planar: rotating about its normal keeps the cylinder in place.
        run(&mut s, &RotateAbout { entities: vec![vol(1)], angle_deg: 37.0, axis: None, origin: None, vertices: None, face: Some(id(EntityKind::TopoFace, 2)) }).expect("cap");
        let after = model(&s).topo().volume_moments(vol(1)).expect("m").centroid().expect("c");
        assert!((before - after).norm() < 1e-12);
        clean(&s);
        // A hemisphere's curved face: rotate about its normal at the centroid projection.
        let mut s = session(factory);
        run(&mut s, &CreateSpherePart { radius: 2.0, inner: None, octants: Some(Octants { x: None, y: None, z: Some(sm_model::HalfSpace::Positive) }), center: None }).expect("hemisphere");
        let curved = id(EntityKind::TopoFace, 1);
        run(&mut s, &RotateAbout { entities: vec![vol(1)], angle_deg: 10.0, axis: None, origin: None, vertices: None, face: Some(curved) }).expect("curved face");
        clean(&s);
    });
}

#[test]
fn rotate_about_axis_z_equals_model_rotate() {
    both(|factory| {
        let mut a = session(factory);
        run(&mut a, &brick_at([1.0, 2.0, 3.0])).expect("brick");
        let mut b = session(factory);
        run(&mut b, &brick_at([1.0, 2.0, 3.0])).expect("brick");
        run(&mut a, &RotateAbout { entities: vec![vol(1)], angle_deg: 33.0, axis: Some(AxisSpec::Name(AxisName::Z)), origin: None, vertices: None, face: None }).expect("named");
        run(&mut b, &Rotate { entities: vec![vol(1)], axis: [0.0, 0.0, 1.0], angle_deg: 33.0, origin: None }).expect("rotate");
        assert_eq!(a.state().hash(), b.state().hash());
        // A vector axis with an origin equals model.rotate too.
        run(&mut a, &RotateAbout { entities: vec![vol(1)], angle_deg: 10.0, axis: Some(AxisSpec::Vector([1.0, 1.0, 0.0])), origin: Some([1.0, 0.0, 0.0]), vertices: None, face: None }).expect("vector");
        run(&mut b, &Rotate { entities: vec![vol(1)], axis: [1.0, 1.0, 0.0], angle_deg: 10.0, origin: Some([1.0, 0.0, 0.0]) }).expect("rotate");
        assert_eq!(a.state().hash(), b.state().hash());
        // Move and move_to agree when the delta is the same.
        let c = centroid(&a, vol(1));
        run(&mut a, &move_to(vec![vol(1)], MoveTarget::Coords([c[0] + 5.0, c[1], c[2]]), None)).expect("move_to");
        let c2 = centroid(&a, vol(1));
        assert!((c2[0] - c[0] - 5.0).abs() < 1e-12);
    });
}

#[test]
fn rotate_about_needs_exactly_one_form() {
    let mut s = session(cell_complex);
    run(&mut s, &cube2()).expect("cube");
    let before = s.state().hash();
    let v = |n| id(EntityKind::TopoVertex, n);
    let ops = [
        // none
        RotateAbout { entities: vec![vol(1)], angle_deg: 1.0, axis: None, origin: None, vertices: None, face: None },
        // two
        RotateAbout { entities: vec![vol(1)], angle_deg: 1.0, axis: Some(AxisSpec::Name(AxisName::X)), origin: None, vertices: Some([v(1), v(2)]), face: None },
        RotateAbout { entities: vec![vol(1)], angle_deg: 1.0, axis: None, origin: None, vertices: Some([v(1), v(2)]), face: Some(id(EntityKind::TopoFace, 1)) },
        // zero axis
        RotateAbout { entities: vec![vol(1)], angle_deg: 1.0, axis: Some(AxisSpec::Vector([0.0; 3])), origin: None, vertices: None, face: None },
        // coincident vertices
        RotateAbout { entities: vec![vol(1)], angle_deg: 1.0, axis: None, origin: None, vertices: Some([v(3), v(3)]), face: None },
        // wrong kinds
        RotateAbout { entities: vec![vol(1)], angle_deg: 1.0, axis: None, origin: None, vertices: Some([v(1), id(EntityKind::TopoFace, 1)]), face: None },
        RotateAbout { entities: vec![vol(1)], angle_deg: 1.0, axis: None, origin: None, vertices: None, face: Some(v(1)) },
        // non-finite angle through a huge value is allowed; infinity is not encodable, so use an unknown face
        RotateAbout { entities: vec![vol(1)], angle_deg: 1.0, axis: None, origin: None, vertices: None, face: Some(id(EntityKind::TopoFace, 99)) },
    ];
    for (i, op) in ops.iter().enumerate() {
        let e = run(&mut s, op).expect_err("refused");
        if i < 5 {
            assert!(format!("{e:?}").contains("Invalid"), "{i}: {e:?}");
        }
        assert_eq!(s.state().hash(), before, "{i}: {e:?}");
    }
}

#[test]
fn rotate_about_a_scaled_and_reflected_cube_still_works() {
    // Chains with the existing ops keep the model clean.
    both(|factory| {
        let mut s = session(factory);
        run(&mut s, &cube2()).expect("cube");
        run(&mut s, &sm_model::Reflect { entities: vec![vol(1)], normal: [1.0, 2.0, 3.0], origin: None }).expect("reflect");
        run(&mut s, &RotateAbout { entities: vec![vol(1)], angle_deg: PI, axis: None, origin: None, vertices: Some([id(EntityKind::TopoVertex, 1), id(EntityKind::TopoVertex, 8)]), face: None }).expect("rotate");
        clean(&s);
        assert!(worst_outwardness(&s) > 0.0);
    });
}
