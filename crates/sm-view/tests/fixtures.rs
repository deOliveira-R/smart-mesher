//! Regenerates `web/test/fixtures/*.json` from real replies and asserts they equal the committed
//! files (golden). `SM_BLESS=1` rewrites them.
#![allow(missing_docs)]

mod common;

use std::path::PathBuf;

use common::*;
use serde_json::{Value, json};
use sm_view::b64;

fn fixture_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("web/test/fixtures")
}

/// The model box `[0,4]x[0,2]x[0,2]` as a `view.tessellate` reply (faces TopoFace#1..6 =
/// -x, +x, -y, +y, -z, +z, as `hex_block` classifies them).
fn tessellation() -> Value {
    let pts: Vec<[f32; 3]> = (0..8).map(|i| [if i & 1 == 1 { 4.0 } else { 0.0 }, if i & 2 == 2 { 2.0 } else { 0.0 }, if i & 4 == 4 { 2.0 } else { 0.0 }]).collect();
    let loops: [[u32; 4]; 6] = [[0, 4, 6, 2], [1, 3, 7, 5], [0, 1, 5, 4], [2, 6, 7, 3], [0, 2, 3, 1], [4, 5, 7, 6]];
    let mut indices: Vec<u32> = Vec::new();
    let mut faces = Vec::new();
    for (i, l) in loops.iter().enumerate() {
        faces.push(json!({"face": format!("TopoFace#{}", i + 1), "first_index": indices.len(), "index_count": 6, "boundary_only": false}));
        indices.extend([l[0], l[1], l[2], l[0], l[2], l[3]]);
    }
    let edge_pairs: [[u32; 2]; 12] = [[0, 1], [2, 3], [4, 5], [6, 7], [0, 2], [1, 3], [4, 6], [5, 7], [0, 4], [1, 5], [2, 6], [3, 7]];
    let mut edge_indices: Vec<u32> = Vec::new();
    let mut edges = Vec::new();
    for (i, e) in edge_pairs.iter().enumerate() {
        edges.push(json!({"edge": format!("TopoEdge#{}", i + 1), "first": edge_indices.len(), "count": 2}));
        edge_indices.extend(e);
    }
    json!({
        "vertex_count": 8, "triangle_count": 12,
        "positions": b64::encode(&b64::f32_bytes(&pts.iter().flatten().copied().collect::<Vec<_>>())),
        "indices": b64::encode(&b64::u32_bytes(&indices)),
        "faces": faces,
        "edge_indices": b64::encode(&b64::u32_bytes(&edge_indices)),
        "edges": edges,
        "chord_tol": 0.0, "angle_tol": 0.3, "max_edge": null
    })
}

fn entry(m: Value, reply: Value) -> Value {
    json!({"match": m, "reply": reply})
}

fn build_hexbox() -> Value {
    let mut r = Rig::new();
    let d = r.apply(CreateBrick { size: [4.0, 2.0, 2.0] });
    let _ = d;
    let node1 = r.s.position().expect("node").to_hex();
    let state1 = r.s.state().hash().to_hex();
    let scene1 = r.query("view.scene", json!({})).expect("scene1");
    r.apply(InsertMesh { kind: "block".into(), n: [4, 2, 2], moves: vec![(22, [2.3, 1.2, 1.1])] });
    let node2 = r.s.position().expect("node").to_hex();
    let state2 = r.s.state().hash().to_hex();
    let scene = r.query("view.scene", json!({})).expect("scene");
    let mesh = scene["meshes"][0]["id"].as_str().expect("mesh id").to_owned();

    let mut skins = Vec::new();
    // Clip replies first: the first matching entry wins and `{}` matches anything.
    let bb = scene["bbox"].clone();
    let lo: Vec<f64> = (0..3).map(|k| bb[0][k].as_f64().expect("lo")).collect();
    let hi: Vec<f64> = (0..3).map(|k| bb[1][k].as_f64().expect("hi")).collect();
    for axis in 0..3 {
        for k in 0..4 {
            let mut origin = lo.clone();
            origin[axis] = lo[axis] + (hi[axis] - lo[axis]) * f64::from(k) / 4.0;
            let mut normal = [0.0; 3];
            normal[axis] = 1.0;
            let clip = json!({"origin": origin, "normal": normal});
            let reply = r.query("view.mesh_skin", json!({"mesh": mesh, "clip": clip})).expect("clip skin");
            skins.push(entry(json!({"clip": clip}), reply));
        }
    }
    skins.push(entry(json!({}), r.query("view.mesh_skin", json!({"mesh": mesh})).expect("skin")));

    let mut scalars = Vec::new();
    for metric in ["scaled_jacobian", "edge_ratio"] {
        let reply = r.query("view.mesh_scalar", json!({"mesh": mesh, "metric": metric, "bad_below": 0.9})).expect("scalar");
        scalars.push(entry(json!({"metric": metric}), reply));
    }
    let sj = scalars[0]["reply"].clone();
    let worst = sj["worst"][0].clone();
    let loc = worst["location"].clone();

    let validate = json!({
        "schema_version": 1, "mesh": mesh, "valid": true, "geometry_checked": true, "total": 1,
        "violations": [{
            "code": "geom.face_nonplanar", "severity": "warning",
            "entity": {"id": worst["cell"]["id"], "index": worst["cell"]["index"]},
            "detail": "a face of the cell next to the displaced node is not planar",
            "owner": "TopoVolume#1", "near": [], "location": loc
        }],
        "truncated": false
    });
    let quality = json!({
        "schema_version": 1, "mesh": mesh, "profile": "generic", "cells": 16, "dim": 3, "verdict": "warn",
        "metrics": [{
            "metric": "scaled_jacobian", "entity": "cell", "worst": [{
                "entity": {"id": worst["cell"]["id"], "index": worst["cell"]["index"]},
                "value": worst["value"], "grade": "warn", "location": loc, "owner": "TopoVolume#1", "near": []
            }]
        }],
        "suggestions": [{
            "id": format!("smooth:{mesh}:scaled_jacobian"), "kind": "smooth", "severity": "warning",
            "metric": "scaled_jacobian", "targets": [mesh], "op": "mesh.smooth",
            "args": {"mesh": mesh, "iterations": 10},
            "params": {}, "reason": "bad cells cluster around one displaced node; smoothing relaxes it", "available": true
        }]
    });
    let log = json!([
        {"node": node2, "parent": node1, "depth": 2, "op": "test.insert_mesh", "author": "test", "time": "", "message": ""},
        {"node": node1, "parent": null, "depth": 1, "op": "test.create_brick", "author": "test", "time": "", "message": ""}
    ]);
    let status = json!({"head": {"ref": "main"}, "position": node2, "depth": 2, "state": state2, "nodes": 2,
        "branches": {"main": node2}, "redo": 0, "dirty": false, "project": null});
    let at1 = json!({"@at": {"node": node1}});
    json!({
        "format": "smview-fixture/1",
        "name": "hexbox",
        "replies": {
            "view.scene": [entry(at1.clone(), scene1), entry(json!({}), scene)],
            "view.tessellate": [entry(json!({}), tessellation())],
            "view.mesh_skin": skins,
            "view.mesh_scalar": scalars,
            "mesh.validate": [entry(json!({}), validate)],
            "mesh.quality": [entry(json!({}), quality)],
            "timeline.log": [entry(json!({}), log)],
            "timeline.status": [entry(json!({}), status)],
            "session.hash": [
                entry(json!({"at": {"node": node1}}), json!({"state": state1, "layers": {}})),
                entry(json!({}), json!({"state": state2, "layers": {}}))
            ],
            "script.run": [entry(json!({}), json!({"state": state2, "output": null, "diagnostics": []}))]
        }
    })
}

fn golden(name: &str, actual: &Value) {
    let path = fixture_dir().join(name);
    let text = serde_json::to_string_pretty(actual).expect("json") + "\n";
    if std::env::var_os("SM_BLESS").is_some() {
        std::fs::create_dir_all(fixture_dir()).expect("mkdir");
        std::fs::write(&path, &text).expect("write");
    }
    let want = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("{}: {e} (SM_BLESS=1 creates it)", path.display()));
    assert!(want == text, "{name} is stale: run `SM_BLESS=1 cargo test -p sm-view --test fixtures`");
}

#[test]
fn hexbox_fixture_is_current() {
    golden("hexbox.json", &build_hexbox());
}

#[test]
fn fixture_generation_is_deterministic() {
    assert_eq!(build_hexbox(), build_hexbox());
}
