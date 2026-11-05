//! W3b-DIAG: the two-step smooth suggestion, `mesh.unflag_cells`, and the opt-in
//! two-boundary-face metric.
#![allow(missing_docs)]

mod common;
use common::*;
use serde_json::{Value, json};
use sm_core::EntityId;
use sm_mesh_api::{MeshRead, SerialAlloc};

/// A 5x5x5 block with every interior vertex jittered by a seeded LCG.
fn jittered_moves(amp: f64) -> Vec<(usize, [f64; 3])> {
    let m = block([5, 5, 5], [1.0; 3], &[], &mut SerialAlloc::new());
    let mut s: u64 = 987654321;
    let mut next = || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((s >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 2.0 * amp
    };
    let mut moves = Vec::new();
    for v in 0..m.vertex_count() {
        let p = m.point(v);
        if [p.x, p.y, p.z].iter().all(|c| *c > 0.0 && *c < 5.0) {
            let (a, b, c) = (next(), next(), next());
            moves.push((v, [p.x + a, p.y + b, p.z + c]));
        }
    }
    moves
}

/// Mesh content: topology hash (covers the set list) and geometry hash.
fn content(rig: &Rig, mesh: EntityId) -> (u64, u64, usize) {
    let layer = rig.state().layer::<sm_layer_api::MeshLayer>().unwrap();
    let m = layer.get(mesh).unwrap().mesh();
    (sm_mesh_api::topology_hash(m), sm_mesh_api::geometry_hash(m), m.sets().len())
}

fn quality(rig: &Rig, mesh: EntityId) -> Value {
    rig.query("mesh.quality", json!({"mesh": mesh, "metrics": ["scaled_jacobian"]})).unwrap()
}

fn min_sj(rig: &Rig, mesh: EntityId) -> f64 {
    quality(rig, mesh)["metrics"][0]["summary"]["min"].as_f64().unwrap()
}

fn suggestions(rig: &Rig, mesh: EntityId) -> Vec<Value> {
    quality(rig, mesh)["suggestions"].as_array().unwrap().clone()
}

fn with_mesh_arg(mesh: EntityId, args: &Value) -> Value {
    let mut a = args.clone();
    a["mesh"] = json!(mesh);
    a
}

#[test]
fn smooth_suggestion_is_a_two_step_pair_that_improves_min_sj() {
    let (mut rig, mesh) = Rig::with_block([5, 5, 5], &jittered_moves(0.32));
    let sg = suggestions(&rig, mesh);
    let kinds: Vec<&str> = sg.iter().map(|s| s["kind"].as_str().unwrap()).collect();
    assert_eq!(kinds, ["inspect", "smooth"], "{sg:#?}");
    let (inspect, smooth) = (&sg[0], &sg[1]);
    assert_eq!(inspect["op"], "mesh.flag_cells");
    assert_eq!(inspect["args"], json!({"name": "diag.scaled_jacobian", "metrics": ["scaled_jacobian"], "level": "warn"}));
    assert_eq!(smooth["op"], "mesh.smooth");
    assert_eq!(smooth["args"], json!({"set": "diag.scaled_jacobian"}));
    assert!(smooth["reason"].as_str().unwrap().contains("preceding `inspect` step"));
    assert!(inspect["available"].as_bool().unwrap() && smooth["available"].as_bool().unwrap());

    let before = min_sj(&rig, mesh);
    rig.json("mesh.flag_cells", with_mesh_arg(mesh, &inspect["args"])).unwrap();
    // With the set present only `smooth` remains.
    let sg2 = suggestions(&rig, mesh);
    let kinds2: Vec<&str> = sg2.iter().map(|s| s["kind"].as_str().unwrap()).collect();
    assert_eq!(kinds2, ["smooth"], "{sg2:#?}");
    assert_eq!(sg2[0]["args"], smooth["args"]);
    assert!(!sg2[0]["reason"].as_str().unwrap().contains("preceding"));

    let mut args = with_mesh_arg(mesh, &smooth["args"]);
    args["method"] = json!({"kind": "laplacian", "relax": 1.0});
    rig.json("mesh.smooth", args).unwrap();
    let after = min_sj(&rig, mesh);
    assert!(after > before, "min SJ {before} -> {after}");
}

#[test]
fn unflag_restores_content_and_set_list() {
    let (mut rig, mesh) = Rig::with_sm_block([3, 3, 3], &[(21, [1.9, 1.7, 1.6])]);
    let sets = |rig: &Rig| -> Vec<(EntityId, String)> {
        let layer = rig.state().layer::<sm_layer_api::MeshLayer>().unwrap();
        layer.get(mesh).unwrap().mesh().sets().iter().map(|s| (s.id, s.name.clone())).collect()
    };
    // A first set, so the state to restore is not the bare mesh.
    rig.json("mesh.flag_cells", json!({"name": "first", "level": "warn"})).unwrap();
    let s0 = sets(&rig);
    let h0 = rig.state().hash();
    let content0 = content(&rig, mesh);

    let (_, d) = rig.json("mesh.flag_cells", json!({"name": "second", "level": "warn", "profile": "openfoam", "thresholds": {"non_orthogonality": {"warn": 0.1}}})).unwrap();
    let created = d.outputs.get("set").map(|v| v[0]);
    assert!(created.is_some());
    assert_ne!(rig.state().hash(), h0);
    let (_, d) = rig.json("mesh.unflag_cells", json!({"set": "second"})).unwrap();
    assert_eq!(d.removed, vec![created.unwrap()]);
    assert_eq!(sets(&rig), s0);
    // The mesh content and set list are back. The state and `meshes` layer hashes are not: both
    // cover allocator counters (`next` serial, retired set) that never roll back, so flag +
    // unflag moves them. See the DIAG report (needs_ruling).
    assert_eq!(content(&rig, mesh), content0);
    let h_after = rig.state().hash();
    assert_ne!(h_after, h0);

    // Unknown set: typed error listing the names, nothing changes.
    let e = rig.json("mesh.unflag_cells", json!({"set": "nope"})).unwrap_err();
    assert!(e.contains("nope") && e.contains("first"), "{e}");
    assert_eq!(rig.state().hash(), h_after);

    // Undo / redo of the removal.
    rig.json("mesh.unflag_cells", json!({"set": "first"})).unwrap();
    assert!(sets(&rig).is_empty());
    rig.s.undo().unwrap();
    assert_eq!(rig.state().hash(), h_after);
    rig.s.redo().unwrap();
    assert!(sets(&rig).is_empty());
}

#[test]
fn unflag_from_the_bare_mesh_restores_the_content() {
    let (mut rig, mesh) = Rig::with_sm_block([3, 3, 3], &[(21, [1.9, 1.7, 1.6])]);
    let c0 = content(&rig, mesh);
    rig.json("mesh.flag_cells", json!({"name": "n", "level": "warn"})).unwrap();
    assert_ne!(content(&rig, mesh), c0);
    rig.json("mesh.unflag_cells", json!({"set": "n"})).unwrap();
    assert_eq!(content(&rig, mesh), c0);
}

#[test]
fn two_boundary_face_cells_are_flagged_and_located_under_transport() {
    // 3x3x1 slab: 9 cells, all with at least 2 boundary faces except none interior: the
    // centre cell has 2 (top and bottom), edge cells 3, corners 4.
    let (rig, mesh) = Rig::with_block([3, 3, 1], &[]);
    // Default transport report keeps its five metrics.
    let r = rig.query("mesh.quality", json!({"mesh": mesh, "profile": "transport"})).unwrap();
    let names: Vec<&str> = r["metrics"].as_array().unwrap().iter().map(|m| m["metric"].as_str().unwrap()).collect();
    assert!(!names.contains(&"boundary_face_count"), "{names:?}");
    let r = rig.query("mesh.quality", json!({"mesh": mesh, "profile": "transport", "metrics": ["boundary_face_count"], "worst_n": 3})).unwrap();
    let mr = &r["metrics"][0];
    assert_eq!(mr["n_warn"], 9);
    assert_eq!(mr["n_fail"], 0);
    // Analytic distribution: 1 cell with 2 boundary faces, 4 with 3, 4 with 4.
    assert_eq!(mr["summary"]["max"].as_f64().unwrap(), 4.0);
    assert_eq!(mr["summary"]["min"].as_f64().unwrap(), 2.0);
    assert_eq!(mr["worst"].as_array().unwrap().len(), 3);
    for w in mr["worst"].as_array().unwrap() {
        assert!(w["location"].is_array() && w["owner"].is_string(), "{w}");
        assert_eq!(w["value"].as_f64().unwrap(), 4.0);
    }
    let mut rig = rig;
    let (_, d) = rig.json("mesh.flag_cells", json!({"name": "wall", "profile": "transport", "metrics": ["boundary_face_count"], "level": "warn", "include_invalid": false})).unwrap();
    assert_eq!(d.outputs["set"].len(), 1);
}

#[test]
fn non_generic_profile_inspect_args_carry_the_profile_and_grade_the_same_cells() {
    let (mut rig, mesh) = Rig::with_block([5, 5, 5], &jittered_moves(0.45));
    let q = rig.query("mesh.quality", json!({"mesh": mesh, "profile": "openfoam"})).unwrap();
    let sg = q["suggestions"].as_array().unwrap().clone();
    let inspect = sg.iter().find(|s| s["kind"] == "inspect").unwrap_or_else(|| panic!("{sg:#?}"));
    assert_eq!(inspect["args"]["profile"], "openfoam");
    let metric = inspect["metric"].as_str().unwrap();
    let report = q["metrics"].as_array().unwrap().iter().find(|m| m["metric"] == metric).unwrap();
    let n_bad = inspect["params"]["n_bad"].as_u64().unwrap();
    assert!(n_bad > 0 && n_bad <= 10);
    // Cells touching the report's bad entities (all within the default worst list).
    let mut expect: Vec<String> = report["worst"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|w| w["grade"] != "ok")
        .flat_map(|w| w["cells"].as_array().unwrap().iter().map(|c| c.as_str().unwrap().to_string()))
        .collect();
    expect.sort();
    expect.dedup();
    assert!(!expect.is_empty());
    // The suggestion's args keep the default `include_invalid`; switch it off to isolate grading.
    let mut args = with_mesh_arg(mesh, &inspect["args"]);
    args["include_invalid"] = json!(false);
    rig.json("mesh.flag_cells", args).unwrap();
    let layer = rig.state().layer::<sm_layer_api::MeshLayer>().unwrap();
    let m = layer.get(mesh).unwrap().mesh();
    let set = m.set_by_name(&format!("diag.{metric}")).unwrap();
    let sm_mesh_api::SetMembers::Cells(cells) = &set.members else { panic!("not a cell set") };
    assert_eq!(cells.len(), expect.len(), "{expect:?}");
}
