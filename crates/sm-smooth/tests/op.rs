//! Acceptance 7: the `mesh.smooth` op and the `mesh.smooth_preview` query.

#![allow(clippy::needless_range_loop)]

mod common;

use common::*;
use serde_json::json;
use sm_core::EntityKind;
use sm_layer_api::MeshLayer;
use sm_mesh_api::{GeomTol, validate, validate_geometry};
use sm_op_api::{OpRecord, QuietControl};

fn smooth_params(mesh: sm_core::EntityId) -> serde_json::Value {
    json!({"mesh": mesh.to_string(), "method": {"kind": "laplacian", "relax": 1.0}, "iterations": 100, "tolerance": 0.0})
}

fn bits(p: &[[f64; 3]]) -> Vec<[u64; 3]> {
    p.iter().map(|q| [q[0].to_bits(), q[1].to_bits(), q[2].to_bits()]).collect()
}

#[test]
fn a7_smoothing_a_jittered_brick_restores_the_lattice() {
    let (mut s, mesh) = meshed_brick([10.0; 3], 1.0);
    let lattice = mesh_points(&s, mesh);
    {
        let layer = s.state().layer::<MeshLayer>().expect("layer");
        let m = layer.get(mesh).expect("mesh").mesh();
        assert!(validate(m).is_empty(), "mesh.run output validates");
        assert!(validate_geometry(m, &GeomTol::default()).is_empty());
    }
    apply(&mut s, &JitterMesh { mesh, seed: 3, amp: 0.1, volume_only: true }).expect("jitter");
    assert!(max_dev(&mesh_points(&s, mesh), &lattice) > 0.05);
    let before_hash = s.state().hash();
    let preview = query(&s, "mesh.smooth_preview", smooth_params(mesh)).expect("preview");
    assert_eq!(s.state().hash(), before_hash, "preview does not change the state");

    let (h, d) = run_json(&mut s, "mesh.smooth", smooth_params(mesh)).expect("mesh.smooth");
    assert!(d.created.is_empty(), "no id is minted: {:?}", d.created);
    assert!(d.removed.is_empty());
    assert_eq!(d.touched_layers, vec!["meshes".to_owned()]);
    let report = d.diagnostics.iter().find(|x| x.code == "smooth.report").expect("smooth.report diagnostic");
    let from_op: serde_json::Value = serde_json::from_str(&report.message).expect("report JSON");
    assert_eq!(from_op, preview, "preview returns the report the op produces");

    let pts = mesh_points(&s, mesh);
    let dev = max_dev(&pts, &lattice);
    eprintln!("a7 deviation after 100 sweeps: {dev:e}");
    assert!(dev <= 1e-6, "deviation {dev:e}");
    let info = query(&s, "mesh.info", json!({"mesh": mesh.to_string()})).expect("info");
    let vol = info["volume"].as_f64().expect("volume");
    assert!((vol - 1000.0).abs() <= 1e-9, "volume {vol}");
    {
        let layer = s.state().layer::<MeshLayer>().expect("layer");
        let m = layer.get(mesh).expect("mesh").mesh();
        assert!(validate(m).is_empty());
        assert!(validate_geometry(m, &GeomTol::default()).is_empty());
    }

    // Undo, redo and cold replay agree.
    let after_hash = s.state().hash();
    s.undo().expect("undo");
    assert_eq!(s.state().hash(), before_hash);
    s.redo().expect("redo");
    assert_eq!(s.state().hash(), after_hash);
    let q = QuietControl::new();
    let cold = s.state_at_from_root(h, &q.control()).expect("cold replay, no Diverged");
    assert_eq!(cold.hash(), after_hash);
}

#[test]
fn a7_entities_restrict_the_moved_nodes() {
    let (mut s, mesh) = meshed_brick([4.0; 3], 1.0);
    apply(&mut s, &JitterMesh { mesh, seed: 8, amp: 0.2, volume_only: false }).expect("jitter");
    let start = mesh_points(&s, mesh);
    let face = {
        let layer = s.state().layer::<MeshLayer>().expect("layer");
        let m = layer.get(mesh).expect("mesh").mesh();
        m.classified_entities().iter().copied().find(|e| e.kind() == EntityKind::TopoFace).expect("a face")
    };
    let on_face: Vec<usize> = {
        let layer = s.state().layer::<MeshLayer>().expect("layer");
        let m = layer.get(mesh).expect("mesh").mesh();
        m.classified_on(face).expect("nodes").vertices.iter().map(|v| v.get()).collect()
    };
    run_json(
        &mut s,
        "mesh.smooth",
        json!({"mesh": mesh.to_string(), "entities": [face.to_string()], "method": {"kind": "laplacian"}, "iterations": 20}),
    )
    .expect("smooth face");
    let end = mesh_points(&s, mesh);
    let (bs, be) = (bits(&start), bits(&end));
    let mut moved_on_face = 0;
    for i in 0..start.len() {
        if on_face.contains(&i) {
            moved_on_face += usize::from(bs[i] != be[i]);
        } else {
            assert_eq!(bs[i], be[i], "node {i} outside the face moved");
        }
    }
    assert!(moved_on_face > 0, "the face nodes moved");
}

#[test]
fn a7_failures_leave_the_state_hash_unchanged() {
    let (mut s, mesh) = meshed_brick([10.0; 3], 1.0);
    // A second brick, never meshed.
    apply(&mut s, &CreateBrick { origin: [20.0, 0.0, 0.0], size: [1.0; 3] }).expect("brick 2");
    let h0 = s.state().hash();
    let fail = |s: &mut sm_op_api::Session, params: serde_json::Value| {
        assert!(run_json(s, "mesh.smooth", params).is_err());
        assert_eq!(s.state().hash(), h0);
    };
    // unknown mesh
    fail(&mut s, json!({"mesh": id(EntityKind::Mesh, 99).to_string(), "method": {"kind": "laplacian"}}));
    // a TopoFace with no nodes in the mesh (a face of the unmeshed brick)
    fail(&mut s, json!({"mesh": mesh.to_string(), "entities": ["TopoFace#7"], "method": {"kind": "laplacian"}}));
    // bad parameters
    fail(&mut s, json!({"mesh": mesh.to_string(), "method": {"kind": "laplacian", "relax": 2.5}}));
    fail(&mut s, json!({"mesh": mesh.to_string(), "method": {"kind": "optimize", "p": 40}}));
    fail(&mut s, json!({"mesh": mesh.to_string(), "method": {"kind": "laplacian"}, "iterations": 0}));
    fail(&mut s, json!({"mesh": mesh.to_string(), "method": {"kind": "laplacian"}, "entities": []}));
    // a high-order mesh
    apply(&mut s, &AddExtraNode { mesh }).expect("extra node");
    let h1 = s.state().hash();
    assert!(run_json(&mut s, "mesh.smooth", json!({"mesh": mesh.to_string(), "method": {"kind": "laplacian"}})).is_err());
    assert_eq!(s.state().hash(), h1);
}

#[test]
fn a7_nothing_to_smooth_fails_without_a_change() {
    let (mut s, mesh) = meshed_brick([1.0; 3], 1.0); // one hex: eight vertex nodes only
    let h = s.state().hash();
    let e = run_json(&mut s, "mesh.smooth", json!({"mesh": mesh.to_string(), "method": {"kind": "laplacian"}})).expect_err("nothing to smooth");
    assert!(format!("{e}").contains("nothing to smooth"), "{e}");
    assert_eq!(s.state().hash(), h);
}

#[test]
fn a7_params_round_trip_through_the_schema() {
    let s = session();
    let reg = s.engine().registry();
    let params = json!({
        "mesh": "Mesh#1",
        "entities": ["TopoVolume#1", "TopoFace#2"],
        "method": {"kind": "optimize", "objective": "mean_ratio", "p": 4.0},
        "iterations": 25,
        "tolerance": 1e-8,
        "schedule": "jacobi",
        "fix_boundary": false,
        "guard_floor": 0.01
    });
    let rec = OpRecord::new("mesh.smooth", 1, params).expect("record");
    let op = reg.decode(&rec).expect("decode");
    let back = OpRecord::of(op.as_ref()).expect("encode");
    assert_eq!(back.params, rec.params);
    // Method defaults fill in and unknown fields are refused.
    let rec = OpRecord::new("mesh.smooth", 1, json!({"mesh": "Mesh#1", "method": {"kind": "laplacian"}})).expect("record");
    let op = reg.decode(&rec).expect("decode");
    assert_eq!(OpRecord::of(op.as_ref()).expect("encode").params["method"], json!({"kind": "laplacian", "relax": 1.0}));
    let bad = OpRecord::new("mesh.smooth", 1, json!({"mesh": "Mesh#1", "method": {"kind": "laplacian"}, "bogus": 1})).expect("record");
    assert!(reg.decode(&bad).is_err());
    let schema = serde_json::to_string(&reg.schema("mesh.smooth").expect("schema")).expect("json");
    assert!(schema.contains("x-entity-kind"), "{schema}");
    assert!(schema.contains("laplacian") && schema.contains("optimize"));
}

#[test]
fn a7_diagnostics_name_the_problem() {
    let (mut s, mesh) = meshed_brick([6.0; 3], 1.0);
    // A translation deck pair on two opposite faces: TopoFace#1 / #2 (x-faces of a brick).
    apply(&mut s, &PairFaces { a: id(EntityKind::TopoFace, 1), b: id(EntityKind::TopoFace, 2), vector: [6.0, 0.0, 0.0] }).expect("pair");
    // A heavy volume-node jitter inverts some hexes.
    apply(&mut s, &JitterMesh { mesh, seed: 1, amp: 0.6, volume_only: true }).expect("jitter");
    let (_, d) = run_json(
        &mut s,
        "mesh.smooth",
        json!({"mesh": mesh.to_string(), "method": {"kind": "laplacian"}, "iterations": 2, "fix_boundary": false}),
    )
    .expect("smooth");
    let codes: Vec<&str> = d.diagnostics.iter().map(|x| x.code.as_str()).collect();
    assert!(codes.contains(&"smooth.report"), "{codes:?}");
    assert!(codes.contains(&"smooth.inverted_input"), "{codes:?}");
    assert!(codes.contains(&"smooth.not_converged"), "{codes:?}");
    let deck = d.diagnostics.iter().find(|x| x.code == "smooth.pinned_deck").expect("pinned_deck");
    assert!(deck.entities.contains(&id(EntityKind::TopoFace, 1)) && deck.entities.contains(&id(EntityKind::TopoFace, 2)), "{:?}", deck.entities);
}
