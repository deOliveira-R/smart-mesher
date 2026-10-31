//! W3 front-end commands (`stage`, `validate`, `quality`, new export formats) against the
//! recording fake endpoint.
mod common;

use common::{canned, go, go_with};
use serde_json::{Value, json};
use sm_protocol::{ExportFormat, Request};

fn query_of(o: &common::Outcome) -> (String, Value) {
    match &o.requests[0] {
        Request::Query { name, input, at } => {
            assert!(at.is_none());
            (name.clone(), input.clone())
        }
        r => panic!("not a query: {r:?}"),
    }
}

fn validate_reply(valid: bool) -> common::Reply {
    Box::new(move |req| match req {
        Request::Query { .. } => Ok(json!({
            "schema_version": 1, "mesh": "Mesh#1", "valid": valid,
            "geometry_checked": true, "geometry_skipped_reason": null,
            "total": 1, "by_code": {"quality.inverted": 1}, "by_owner": [],
            "violations": [{
                "code": "quality.inverted", "severity": "error",
                "entity": {"id": "MeshCell#3", "index": 3}, "detail": "cell is inverted",
                "owner": "TopoVolume#1", "near": ["TopoFace#2", "TopoEdge#5"],
                "location": [0.5, 0.25, 1.0]
            }],
            "truncated": false
        })),
        r => canned(r),
    })
}

#[test]
fn export_exodus_and_openfoam_formats() {
    for (arg, want) in [
        ("exodus", ExportFormat::Exodus),
        ("openfoam", ExportFormat::Openfoam),
    ] {
        let o = go(&["export", "out", "--format", arg, "--memory"]);
        assert_eq!(o.code, 0, "{}", o.err);
        assert!(matches!(&o.requests[0], Request::Export { format: Some(f), .. } if *f == want));
    }
    let o = go(&["export", "a.exo", "--memory"]);
    assert!(matches!(
        &o.requests[0],
        Request::Export {
            format: Some(ExportFormat::Exodus),
            ..
        }
    ));
    // A directory name cannot be inferred.
    let o = go(&["export", "case", "--memory"]);
    assert_eq!(o.code, 2);
    assert!(o.requests.is_empty());
}

#[test]
fn export_lists_the_files_of_a_multi_file_result() {
    let reply: common::Reply = Box::new(|req| match req {
        Request::Export { .. } => Ok(json!({
            "path": "case", "format": "openfoam", "bytes": 30, "sha256": common::h("a").to_string(),
            "files": [{"name": "constant/polyMesh/points", "bytes": 10, "sha256": common::h("p").to_string()}]
        })),
        r => canned(r),
    });
    let o = go_with(
        &["export", "case", "--format", "openfoam", "--memory"],
        "",
        reply,
    );
    assert_eq!(o.code, 0, "{}", o.err);
    assert!(o.out.contains("wrote case (1 files)"), "{}", o.out);
    assert!(
        o.out.contains("constant/polyMesh/points (10 bytes"),
        "{}",
        o.out
    );
}

#[test]
fn stage_runs_session_stage_and_prints_sha_and_bytes() {
    let sha = "cd".repeat(32);
    let sha2 = sha.clone();
    let reply: common::Reply = Box::new(move |req| match req {
        Request::ScriptRun { .. } => Ok(json!({
            "state": common::h("s").to_string(), "output": "", "diagnostics": [],
            "events": [{"event": "session", "command": "stage", "applied": true,
                        "result": {"sha256": sha2, "bytes": 7}}]
        })),
        r => canned(r),
    });
    let o = go_with(&["stage", "part.step", "--memory", "--json"], "", reply);
    assert_eq!(o.code, 0, "{}", o.err);
    assert_eq!(o.json(), json!({"sha256": sha, "bytes": 7}));
    let Request::ScriptRun { text, .. } = &o.requests[0] else {
        panic!()
    };
    assert!(
        text.starts_with("session.stage(\"") && text.contains("part.step"),
        "{text}"
    );
    assert_eq!(o.requests.len(), 1);

    let o = go_with(
        &["stage", "a\"b.step", "--memory"],
        "",
        Box::new(|_| Ok(json!({"events": []}))),
    );
    assert_eq!(o.code, 1, "no session event is a malformed reply");
}

#[test]
fn stage_server_error_is_exit_1() {
    let reply: common::Reply = Box::new(|_| {
        Err(sm_protocol::RpcError::new(
            -32000,
            "script-error",
            "reading nope.step: not found",
        ))
    });
    let o = go_with(&["stage", "nope.step", "--memory"], "", reply);
    assert_eq!(o.code, 1);
    assert!(o.err.contains("script-error"), "{}", o.err);
}

#[test]
fn validate_builds_the_query_and_prints_violations() {
    let o = go_with(
        &["validate", "--mesh", "Mesh#1", "--memory"],
        "",
        validate_reply(true),
    );
    assert_eq!(o.code, 0, "{}", o.err);
    assert_eq!(
        query_of(&o),
        ("mesh.validate".to_owned(), json!({"mesh": "Mesh#1"}))
    );
    assert!(
        o.out.starts_with("Mesh#1: valid (1 violations)"),
        "{}",
        o.out
    );
    for needle in [
        "error quality.inverted",
        "entity=MeshCell#3",
        "owner=TopoVolume#1",
        "near=TopoFace#2,TopoEdge#5",
        "at=(",
    ] {
        assert!(o.out.contains(needle), "{needle} in {}", o.out);
    }
    let o = go_with(&["validate", "--memory"], "", validate_reply(true));
    assert_eq!(query_of(&o).1, json!({}));
}

#[test]
fn validate_exits_1_when_invalid_and_still_prints() {
    let o = go_with(&["validate", "--memory"], "", validate_reply(false));
    assert_eq!(o.code, 1);
    assert!(o.out.contains("INVALID"), "{}", o.out);
    assert!(o.err.is_empty(), "{}", o.err);
    let o = go_with(
        &["validate", "--memory", "--json"],
        "",
        validate_reply(false),
    );
    assert_eq!(o.code, 1);
    assert_eq!(o.json()["valid"], json!(false));
    assert_eq!(o.out.trim().lines().count(), 1);
}

#[test]
fn validate_rejects_a_bad_mesh_id() {
    let o = go(&["validate", "--mesh", "nonsense", "--memory"]);
    assert_eq!(o.code, 2);
    assert!(o.requests.is_empty());
}

fn quality_reply() -> common::Reply {
    Box::new(|req| match req {
        Request::Query { .. } => Ok(json!({
            "schema_version": 1, "mesh": "Mesh#1", "profile": "openfoam", "cells": 8, "dim": 3,
            "verdict": "warn", "validity": {"n_valid": 8, "n_degenerate": 0, "n_inverted": 0},
            "metrics": [{
                "metric": "non_orthogonality", "entity": "face", "unit": "deg", "sense": "higher_is_worse",
                "count": 36, "undefined": 24, "n_sentinel": 0,
                "summary": {"min": 0.0, "max": 55.5, "mean": 10.25, "std_dev": 1.0},
                "threshold": {"warn": 50.0, "fail": 70.0, "fail_boundary": null},
                "n_warn": 2, "n_fail": 0, "histogram": {"lo": 0.0, "hi": 1.0, "counts": [], "below": 0, "above": 0},
                "worst": [{"entity": {"id": "MeshFace#9", "index": 9}, "value": 55.5, "grade": "warn",
                           "location": [0.0, 0.5, 0.5], "owner": "TopoFace#1", "near": []}],
                "clusters": []
            }],
            "skipped": [],
            "suggestions": [
                {"id": "smooth:x:y", "kind": "smooth", "severity": "warning", "metric": "non_orthogonality",
                 "targets": ["TopoVolume#1"], "op": "mesh.smooth", "args": null, "params": {}, "reason": "bad faces are spread", "available": true},
                {"id": "inspect:x:y", "kind": "inspect", "severity": "warning", "metric": "non_orthogonality",
                 "targets": [], "op": "mesh.flag_cells", "args": null, "params": {}, "reason": "look at them", "available": false}
            ]
        })),
        r => canned(r),
    })
}

#[test]
fn quality_builds_the_query_with_profile_and_worst() {
    let o = go_with(
        &[
            "quality",
            "--mesh",
            "Mesh#1",
            "--profile",
            "openfoam",
            "--worst",
            "3",
            "--memory",
        ],
        "",
        quality_reply(),
    );
    assert_eq!(o.code, 0, "{}", o.err);
    assert_eq!(
        query_of(&o),
        (
            "mesh.quality".to_owned(),
            json!({"mesh": "Mesh#1", "profile": "openfoam", "worst_n": 3})
        )
    );
    for needle in [
        "verdict warn",
        "non_orthogonality [face]: min 0.000000e0 max 5.550000e1 mean 1.025000e1 n_warn 2 n_fail 0",
        "worst MeshFace#9",
        "smooth (available op=mesh.smooth)",
        "inspect (unavailable op=mesh.flag_cells)",
    ] {
        assert!(o.out.contains(needle), "{needle} in {}", o.out);
    }
    for p in ["generic", "fluent", "transport"] {
        let o = go_with(&["quality", "--profile", p, "--memory"], "", quality_reply());
        assert_eq!(query_of(&o).1, json!({"profile": p}));
    }
    let o = go(&["quality", "--profile", "bogus", "--memory"]);
    assert_eq!(o.code, 2);
}

#[test]
fn quality_json_is_the_server_value_and_exit_0_even_on_fail() {
    let o = go_with(&["quality", "--memory", "--json"], "", quality_reply());
    assert_eq!(o.code, 0);
    assert_eq!(o.json()["suggestions"][1]["available"], json!(false));
    assert_eq!(o.out.trim().lines().count(), 1);
}
