//! T1 acceptance: move, rotate, scale, reflect and the meshed-volume refusal.

mod common;

use common::*;
use core::f64::consts::PI;
use sm_core::EntityKind;
use sm_model::{CreateBrick, CreateCylinder, CreateSphere, Move, Reflect, Rotate, Scale};
use sm_op_api::{Op, OpError, Session, TimelineError};

fn brick10() -> CreateBrick {
    CreateBrick { x: 10.0, y: None, z: None, center: None }
}

fn vol1() -> sm_core::EntityId {
    id(EntityKind::TopoVolume, 1)
}

fn body1() -> sm_core::EntityId {
    id(EntityKind::TopoBody, 1)
}

fn volume_of(s: &Session) -> f64 {
    let t = model(s).topo();
    t.volume_moments(t.cells(sm_topo_api::Dim::Volume)[0]).expect("moments").volume
}

fn hosts() -> [Factory; 2] {
    [cell_complex, topo_store]
}

fn is_invalid(r: Result<(sm_op_api::Hash256, sm_op_api::Delta), TimelineError>) -> bool {
    matches!(r, Err(TimelineError::Apply(sm_op_api::ApplyError::Op { source: OpError::Invalid(_), .. })))
}

#[test]
fn move_translates_every_vertex_bit_exactly_and_keeps_ids() {
    let mut hashes = Vec::new();
    for factory in hosts() {
        let mut s = session(factory);
        run(&mut s, &brick10()).expect("brick");
        let before = vertex_points(&s);
        let (_, d) = run(&mut s, &Move { entities: vec![vol1()], delta: [1.0, 2.0, 3.0] }).expect("move");
        let after = vertex_points(&s);
        for (a, b) in before.iter().zip(&after) {
            assert_eq!(b.map(f64::to_bits), [a[0] + 1.0, a[1] + 2.0, a[2] + 3.0].map(f64::to_bits));
        }
        assert!(d.created.is_empty() && d.removed.is_empty());
        assert_eq!(d.outputs["volumes"], vec![vol1()]);
        assert!((volume_of(&s) - 1000.0).abs() < 1e-9);
        assert_eq!(issues(&s), Vec::<String>::new());
        assert!(worst_outwardness(&s) > 0.0);
        hashes.push(s.state().layer_hash("model").expect("hash"));
    }
    assert_eq!(hashes[0], hashes[1], "reference and production hosts agree");
}

#[test]
fn move_by_body_moves_its_volume() {
    let mut s = session(cell_complex);
    run(&mut s, &brick10()).expect("brick");
    let (_, d) = run(&mut s, &Move { entities: vec![body1()], delta: [0.5, 0.0, 0.0] }).expect("move body");
    assert_eq!(d.outputs["volumes"], vec![vol1()]);
    assert_eq!(vertex_points(&s)[0], [-4.5, -5.0, -5.0]);
}

#[test]
fn rotate_90_about_z_maps_vertices_and_keeps_outward_normals() {
    for factory in hosts() {
        let mut s = session(factory);
        run(&mut s, &CreateBrick { x: 2.0, y: Some(4.0), z: Some(6.0), center: Some([1.0, 0.0, 0.0]) }).expect("brick");
        let before = vertex_points(&s);
        run(&mut s, &Rotate { entities: vec![vol1()], axis: [0.0, 0.0, 1.0], angle_deg: 90.0, origin: None }).expect("rotate");
        let after = vertex_points(&s);
        for (a, b) in before.iter().zip(&after) {
            // (x, y, z) -> (-y, x, z)
            let want = [-a[1], a[0], a[2]];
            for k in 0..3 {
                assert!((b[k] - want[k]).abs() < 1e-12, "{b:?} vs {want:?}");
            }
        }
        assert!((volume_of(&s) - 48.0).abs() < 1e-9 * 48.0);
        assert!(worst_outwardness(&s) > 0.0);
        assert_eq!(issues(&s), Vec::<String>::new());
    }
}

#[test]
fn rotate_about_an_offset_axis() {
    let mut s = session(cell_complex);
    run(&mut s, &CreateBrick { x: 2.0, y: None, z: None, center: Some([1.0, 0.0, 0.0]) }).expect("brick");
    run(&mut s, &Rotate { entities: vec![vol1()], axis: [0.0, 0.0, 2.0], angle_deg: 180.0, origin: Some([1.0, 0.0, 0.0]) }).expect("rotate");
    // A cube centred on the axis maps onto itself (vertex set unchanged).
    let mut got = vertex_points(&s);
    got.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    let mut want = Vec::new();
    for i in 0..2 {
        for j in 0..2 {
            for k in 0..2 {
                want.push([i as f64 * 2.0, j as f64 * 2.0 - 1.0, k as f64 * 2.0 - 1.0]);
            }
        }
    }
    want.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    for (g, w) in got.iter().zip(&want) {
        for k in 0..3 {
            assert!((g[k] - w[k]).abs() < 1e-12);
        }
    }
}

#[test]
fn scale_brick_cylinder_and_sphere_about_a_point() {
    let mut s = session(cell_complex);
    run(&mut s, &brick10()).expect("brick");
    run(&mut s, &Scale { entities: vec![vol1()], factor: 2.0, origin: Some([5.0, 5.0, 5.0]) }).expect("scale");
    let pts = vertex_points(&s);
    assert_eq!(pts[0], [-15.0, -15.0, -15.0]);
    assert_eq!(pts[7], [5.0, 5.0, 5.0]);
    assert!((volume_of(&s) - 8000.0).abs() < 1e-9 * 8000.0);
    assert_eq!(issues(&s), Vec::<String>::new());

    // A cylinder carries pcurves: area/volume scale as f^2 / f^3 and the pcurve check passes.
    for factory in hosts() {
        let mut s = session(factory);
        let (r, h, f) = (1.5, 4.0, 3.0);
        run(&mut s, &CreateCylinder { height: h, radius: r, center: None }).expect("cyl");
        run(&mut s, &Scale { entities: vec![vol1()], factor: f, origin: None }).expect("scale cyl");
        let want = PI * (r * f) * (r * f) * (h * f);
        assert!((volume_of(&s) - want).abs() < 1e-9 * want, "{} vs {want}", volume_of(&s));
        assert!(worst_outwardness(&s) > 0.0);
        assert_eq!(issues(&s), Vec::<String>::new());
    }
    for factory in hosts() {
        let mut s = session(factory);
        run(&mut s, &CreateSphere { radius: 1.0, center: Some([1.0, 0.0, 0.0]) }).expect("sphere");
        run(&mut s, &Scale { entities: vec![vol1()], factor: 2.0, origin: Some([0.0, 1.0, 0.0]) }).expect("scale sphere");
        assert_eq!(issues(&s), Vec::<String>::new());
        // The centre maps to (2, -1, 0) and the radius doubles.
        let m = measure(&s, vol1());
        let want = 4.0 / 3.0 * PI * 8.0;
        assert!((m.volume.expect("v") - want).abs() < 1e-12 * want);
        assert!((m.area.expect("a") - 16.0 * PI).abs() < 1e-12 * 16.0 * PI);
        let c = m.centroid.expect("c");
        for (g, w) in c.iter().zip([2.0, -1.0, 0.0]) {
            assert!((g - w).abs() < 1e-12, "{c:?}");
        }
        for k in 0..3 {
            assert!((m.bbox.lo[k] - ([2.0, -1.0, 0.0][k] - 2.0)).abs() < 1e-12 && (m.bbox.hi[k] - ([2.0, -1.0, 0.0][k] + 2.0)).abs() < 1e-12, "{:?}", m.bbox);
        }
        check_sphere(&s, [2.0, -1.0, 0.0], 2.0);
    }
}

#[test]
fn reflect_brick_keeps_positive_volume_outward_normals_and_validates() {
    let mut hashes = Vec::new();
    for factory in hosts() {
        let mut s = session(factory);
        run(&mut s, &CreateBrick { x: 2.0, y: Some(4.0), z: Some(6.0), center: Some([3.0, 1.0, -2.0]) }).expect("brick");
        let before = vertex_points(&s);
        let before_hash = s.state().layer_hash("model").expect("hash");
        let (_, d) = run(&mut s, &Reflect { entities: vec![vol1()], normal: [1.0, 0.0, 0.0], origin: None }).expect("reflect");
        let after = vertex_points(&s);
        for (a, b) in before.iter().zip(&after) {
            assert_eq!(b.map(f64::to_bits), [-a[0], a[1], a[2]].map(f64::to_bits));
        }
        assert_eq!(d.outputs["volumes"], vec![vol1()]);
        let v = volume_of(&s);
        assert!((v - 48.0).abs() < 1e-9 * 48.0, "volume {v}");
        assert!(worst_outwardness(&s) > 0.0, "every face points outward");
        assert_eq!(issues(&s), Vec::<String>::new());
        assert_ne!(s.state().layer_hash("model").expect("hash"), before_hash);
        // A second reflection restores the geometry.
        run(&mut s, &Reflect { entities: vec![vol1()], normal: [1.0, 0.0, 0.0], origin: None }).expect("reflect back");
        assert_eq!(vertex_points(&s), before);
        assert!(worst_outwardness(&s) > 0.0);
        assert_eq!(issues(&s), Vec::<String>::new());
        hashes.push(s.state().layer_hash("model").expect("hash"));
    }
    assert_eq!(hashes[0], hashes[1], "both hosts agree after two reflections");
}

#[test]
fn reflect_cylinder_and_sphere_keep_pcurves_and_orientation() {
    for factory in hosts() {
        let mut s = session(factory);
        run(&mut s, &CreateCylinder { height: 4.0, radius: 1.5, center: Some([1.0, 2.0, 3.0]) }).expect("cyl");
        let want = PI * 1.5 * 1.5 * 4.0;
        run(&mut s, &Reflect { entities: vec![vol1()], normal: [0.0, 0.0, 1.0], origin: None }).expect("reflect cyl z");
        assert!((volume_of(&s) - want).abs() < 1e-9 * want);
        assert!(worst_outwardness(&s) > 0.0);
        assert_eq!(issues(&s), Vec::<String>::new());
        run(&mut s, &Reflect { entities: vec![vol1()], normal: [1.0, 1.0, 0.0], origin: Some([0.5, 0.0, 0.0]) }).expect("reflect cyl diag");
        assert!((volume_of(&s) - want).abs() < 1e-9 * want);
        assert!(worst_outwardness(&s) > 0.0);
        assert_eq!(issues(&s), Vec::<String>::new());

        let mut s = session(factory);
        run(&mut s, &CreateSphere { radius: 1.0, center: Some([1.0, 0.0, 0.0]) }).expect("sphere");
        run(&mut s, &Reflect { entities: vec![vol1()], normal: [1.0, 0.0, 0.0], origin: None }).expect("reflect sphere");
        assert_eq!(issues(&s), Vec::<String>::new());
        // The orientation is also checked from the geometry (closed form, independent of the
        // numeric moments): sense stays Forward, the face normal is
        // radial, the divergence integral is +4/3 pi r^3, and the loop winding agrees.
        let m = measure(&s, vol1());
        let want = 4.0 / 3.0 * PI;
        assert!((m.volume.expect("v") - want).abs() < 1e-12 * want);
        let c = m.centroid.expect("c");
        for (g, w) in c.iter().zip([-1.0, 0.0, 0.0]) {
            assert!((g - w).abs() < 1e-12, "{c:?}");
        }
        assert_eq!(m.bbox.lo, [-2.0, -1.0, -1.0]);
        assert_eq!(m.bbox.hi, [0.0, 1.0, 1.0]);
        check_sphere(&s, [-1.0, 0.0, 0.0], 1.0);
        // A diagonal mirror (tilted frame).
        run(&mut s, &Reflect { entities: vec![vol1()], normal: [1.0, 1.0, 1.0], origin: Some([0.0, 0.0, 1.0]) }).expect("reflect sphere diag");
        assert_eq!(issues(&s), Vec::<String>::new());
        // c' = c - 2 (n.(c - o)) n, n = (1,1,1)/sqrt3, o = (0,0,1): n.(c-o) = -2/sqrt3.
        let k = 4.0 / 3.0;
        check_sphere(&s, [-1.0 + k, k, k], 1.0);
    }
}

/// Direction vectors with a norm that overflows (or underflows) when squared must give the same
/// transform as their unit versions: `sm_core::Transform` takes `sqrt(dot)` of the raw vector.
#[test]
fn huge_and_tiny_direction_vectors_act_like_their_unit_directions() {
    for factory in hosts() {
        let rot = |axis: [f64; 3]| {
            let mut s = session(factory);
            run(&mut s, &brick10()).expect("brick");
            run(&mut s, &Rotate { entities: vec![vol1()], axis, angle_deg: 60.0, origin: None }).expect("rotate");
            s
        };
        let want = rot([1.0, 0.0, 0.0]);
        for axis in [[1e200, 0.0, 0.0], [1e-300, 0.0, 0.0], [f64::MAX, 0.0, 0.0], [5e-324, 0.0, 0.0]] {
            let s = rot(axis);
            for (a, b) in vertex_points(&s).iter().zip(vertex_points(&want).iter()) {
                for k in 0..3 {
                    assert!((a[k] - b[k]).abs() < 1e-12, "axis {axis:?}: {a:?} vs {b:?}");
                }
            }
            assert!((volume_of(&s) - 1000.0).abs() < 1e-9 * 1000.0, "axis {axis:?}: volume {}", volume_of(&s));
            assert_eq!(issues(&s), Vec::<String>::new(), "axis {axis:?}");
            assert!(worst_outwardness(&s) > 0.0);
        }
        let (a, b) = (rot([1e300, 1e300, 0.0]), rot([1.0, 1.0, 0.0]));
        for (p, q) in vertex_points(&a).iter().zip(vertex_points(&b).iter()) {
            for k in 0..3 {
                assert!((p[k] - q[k]).abs() < 1e-12, "{p:?} vs {q:?}");
            }
        }

        let refl = |normal: [f64; 3]| {
            let mut s = session(factory);
            run(&mut s, &CreateBrick { x: 2.0, y: Some(4.0), z: Some(6.0), center: Some([3.0, 1.0, -2.0]) }).expect("brick");
            run(&mut s, &Reflect { entities: vec![vol1()], normal, origin: Some([1.0, 0.0, 0.0]) }).expect("reflect");
            s
        };
        let want = refl([1.0, 0.0, 0.0]);
        for normal in [[1e200, 0.0, 0.0], [1e-300, 0.0, 0.0], [-f64::MAX, 0.0, 0.0]] {
            let s = refl(normal);
            // x -> 2 - x about the plane x = 1: the box 2..4 maps to -2..0.
            assert_eq!(vertex_points(&s), vertex_points(&want), "normal {normal:?}");
            assert!(vertex_points(&s).iter().all(|p| p[0] <= 1e-12), "normal {normal:?} must move the model");
            assert!(worst_outwardness(&s) > 0.0);
            assert_eq!(issues(&s), Vec::<String>::new());
        }
    }
}

#[test]
fn invalid_requests_are_refused_without_changing_the_state() {
    let mut s = session(cell_complex);
    run(&mut s, &brick10()).expect("brick");
    run(&mut s, &CreateBrick { x: 1.0, y: None, z: None, center: Some([20.0, 0.0, 0.0]) }).expect("second brick");
    let before = s.state().hash();
    let bad: Vec<Box<dyn Op>> = vec![
        Box::new(Move { entities: vec![], delta: [0.0; 3] }),
        Box::new(Move { entities: vec![vol1()], delta: [f64::NAN, 0.0, 0.0] }),
        Box::new(Rotate { entities: vec![vol1()], axis: [0.0; 3], angle_deg: 10.0, origin: None }),
        Box::new(Rotate { entities: vec![vol1()], axis: [0.0, 0.0, 1.0], angle_deg: f64::INFINITY, origin: None }),
        Box::new(Scale { entities: vec![vol1()], factor: 0.0, origin: None }),
        Box::new(Scale { entities: vec![vol1()], factor: -2.0, origin: None }),
        Box::new(Reflect { entities: vec![vol1()], normal: [0.0; 3], origin: None }),
        // Overflow while the transform is built: classified as bad parameters.
        Box::new(Scale { entities: vec![vol1()], factor: 1e200, origin: None }),
        Box::new(Scale { entities: vec![vol1()], factor: 1e-200, origin: None }),
        Box::new(Reflect { entities: vec![vol1()], normal: [1.0, 1.0, 0.0], origin: Some([f64::MAX, f64::MAX, 0.0]) }),
        Box::new(Reflect { entities: vec![vol1()], normal: [f64::NAN, 0.0, 1.0], origin: None }),
        Box::new(Rotate { entities: vec![vol1()], axis: [f64::INFINITY, 0.0, 0.0], angle_deg: 10.0, origin: None }),
        Box::new(Move { entities: vec![id(EntityKind::TopoFace, 1)], delta: [1.0, 0.0, 0.0] }),
        Box::new(Move { entities: vec![id(EntityKind::TopoVolume, 99)], delta: [1.0, 0.0, 0.0] }),
    ];
    for op in &bad {
        assert!(run(&mut s, op.as_ref()).is_err(), "{op:?}");
        assert_eq!(s.state().hash(), before, "{op:?} must not change the state");
    }
}

#[test]
fn transform_of_a_meshed_volume_is_invalid_and_mesh_delete_hint_is_in_the_message() {
    let mut s = session(cell_complex);
    run(&mut s, &brick10()).expect("brick");
    run(&mut s, &common::FakeMesh { volume: vol1() }).expect("mesh");
    let before = s.state().hash();
    let ops: Vec<Box<dyn Op>> = vec![
        Box::new(Move { entities: vec![vol1()], delta: [1.0, 0.0, 0.0] }),
        Box::new(Rotate { entities: vec![vol1()], axis: [0.0, 0.0, 1.0], angle_deg: 1.0, origin: None }),
        Box::new(Scale { entities: vec![vol1()], factor: 2.0, origin: None }),
        Box::new(Reflect { entities: vec![body1()], normal: [1.0, 0.0, 0.0], origin: None }),
    ];
    for op in &ops {
        let r = run(&mut s, op.as_ref());
        let msg = format!("{r:?}");
        assert!(is_invalid(r), "{op:?}: {msg}");
        assert!(msg.contains("mesh.delete first"), "{msg}");
        assert_eq!(s.state().hash(), before);
    }
}

#[test]
fn transform_without_a_meshes_layer_is_allowed() {
    // A registry with only the model layer: the meshed-volume check tolerates the missing layer.
    use std::sync::Arc;
    let mut reg = sm_op_api::Registry::new();
    reg.register_layer_with::<sm_layer_api::ModelLayer>(|| sm_layer_api::ModelLayer::new(cell_complex())).expect("model layer");
    reg.install(&sm_model::ModelPlugin).expect("plugin");
    let engine = sm_op_api::Engine::new(Arc::new(reg), sm_op_api::Services::new(), Arc::new(sm_op_api::DerivedCache::default())).expect("engine");
    let mut s = Session::new(engine);
    run(&mut s, &brick10()).expect("brick");
    run(&mut s, &Move { entities: vec![vol1()], delta: [1.0, 0.0, 0.0] }).expect("move without a mesh layer");
}

fn deck_plane(s: &Session) -> (sm_core::Vector3, f64) {
    use sm_model_api::{BoundaryKind, Deck};
    let l = model(s);
    let row = l.model().boundaries().find(|r| r.kind.as_deck().is_some()).expect("a deck boundary");
    match &row.kind {
        BoundaryKind::Deck(Deck::Mirror { plane }) => (plane.normal, plane.offset),
        other => panic!("{other:?}"),
    }
}

#[test]
fn mirror_deck_stays_consistent_through_every_transform() {
    let top = id(EntityKind::TopoFace, 6);
    let mk = || {
        let mut s = session(topo_store);
        run(&mut s, &brick10()).expect("brick");
        run(&mut s, &common::MirrorDeck { face: top, normal: [0.0, 0.0, 1.0], offset: 5.0 }).expect("deck");
        assert_eq!(model_errors(&s), Vec::<String>::new(), "deck consistent before");
        s
    };

    let mut s = mk();
    run(&mut s, &Move { entities: vec![vol1()], delta: [1.0, 2.0, 3.0] }).expect("move");
    assert_eq!(model_errors(&s), Vec::<String>::new(), "after move");
    let (n, d) = deck_plane(&s);
    assert!((n.z - 1.0).abs() < 1e-12 && (d - 8.0).abs() < 1e-12, "{n:?} {d}");

    let mut s = mk();
    run(&mut s, &Rotate { entities: vec![vol1()], axis: [0.0, 1.0, 0.0], angle_deg: 90.0, origin: None }).expect("rotate");
    assert_eq!(model_errors(&s), Vec::<String>::new(), "after rotate");

    let mut s = mk();
    run(&mut s, &Scale { entities: vec![vol1()], factor: 2.0, origin: None }).expect("scale");
    assert_eq!(model_errors(&s), Vec::<String>::new(), "after scale");
    assert!((deck_plane(&s).1 - 10.0).abs() < 1e-12);

    let mut s = mk();
    run(&mut s, &Reflect { entities: vec![vol1()], normal: [0.0, 0.0, 1.0], origin: None }).expect("reflect");
    assert_eq!(model_errors(&s), Vec::<String>::new(), "after reflect");
    assert!(worst_outwardness(&s) > 0.0);
    // The deck face now lies on the image plane z = -5.
    let t = model(&s).topo();
    let m = t.face_moments(top).expect("moments");
    assert!((m.centroid().expect("centroid").z + 5.0).abs() < 1e-12);
    let (n, d) = deck_plane(&s);
    assert!((n.dot(sm_core::Vector3::new(0.0, 0.0, -5.0)) - d).abs() < 1e-12, "image plane contains the deck face: {n:?} {d}");

    // A diagonal mirror too.
    let mut s = mk();
    run(&mut s, &Reflect { entities: vec![vol1()], normal: [1.0, 0.0, 1.0], origin: Some([1.0, 0.0, 0.0]) }).expect("reflect diag");
    assert_eq!(model_errors(&s), Vec::<String>::new(), "after diagonal reflect");
}

#[test]
fn a_shared_face_refuses_a_partial_move_but_allows_the_whole_assembly() {
    let mut s = session(cell_complex);
    run(&mut s, &common::MergedBricks {}).expect("merged bricks");
    let (v1, v2) = (id(EntityKind::TopoVolume, 1), id(EntityKind::TopoVolume, 2));
    let before = s.state().hash();
    let r = run(&mut s, &Move { entities: vec![v1], delta: [0.0, 0.0, 1.0] });
    let msg = format!("{r:?}");
    assert!(is_invalid(r) && msg.contains("shared"), "{msg}");
    assert_eq!(s.state().hash(), before);
    let (_, d) = run(&mut s, &Rotate { entities: vec![v1, v2], axis: [0.0, 0.0, 1.0], angle_deg: 30.0, origin: None }).expect("both");
    assert_eq!(d.outputs["volumes"], vec![v1, v2]);
    assert_eq!(issues(&s), Vec::<String>::new());
    run(&mut s, &Reflect { entities: vec![v1, v2], normal: [0.0, 1.0, 0.0], origin: None }).expect("reflect both");
    assert_eq!(issues(&s), Vec::<String>::new());
    assert!(worst_outwardness(&s) > 0.0);
}

#[test]
fn overflowing_transform_inputs_are_invalid_not_failed() {
    let mut s = session(cell_complex);
    run(&mut s, &brick10()).expect("brick");
    let before = s.state().hash();
    let ops: Vec<Box<dyn Op>> = vec![
        Box::new(Scale { entities: vec![vol1()], factor: 1e200, origin: None }),
        Box::new(Scale { entities: vec![vol1()], factor: 1e-200, origin: None }),
        Box::new(Reflect { entities: vec![vol1()], normal: [1.0, 1.0, 0.0], origin: Some([f64::MAX, f64::MAX, 0.0]) }),
    ];
    for op in &ops {
        let msg = format!("{:?}", run(&mut s, op.as_ref()).expect_err("refused"));
        assert!(!msg.contains("op failed") && !msg.contains("not a similarity"), "{op:?}: {msg}");
        assert_eq!(s.state().hash(), before);
    }
}
