//! Each `smesh` command against a recording endpoint: the exact protocol requests it builds,
//! its stdout/stderr, and its exit code. The endpoint is a fake because this crate must stay
//! testable independent of the server crates' progress; the real binary is covered in
//! `tests/binary.rs` and `tests/cli_full.rs`.
mod common;

use std::path::PathBuf;

use common::{Outcome, canned, go, go_with, h, status_value};
use serde_json::{Value, json};
use sm_cli::{Settings, discover_project};
use sm_op_api::Rev;
use sm_protocol::{
    ApplyResult, ExportFormat, ExportResult, HashResult, Request, RpcError, ScriptResult,
    StatusResult,
};

fn tmp(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("smesh-cli-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).unwrap();
    d
}

fn methods(o: &Outcome) -> Vec<&'static str> {
    o.requests.iter().map(Request::method).collect()
}

// ---- exit codes and usage -------------------------------------------------------------

#[test]
fn no_arguments_is_a_usage_error_that_prints_usage() {
    let o = go(&[]);
    assert_eq!(o.code, 2);
    assert!(o.err.starts_with("usage:"), "{}", o.err);
    assert!(
        o.settings.is_none(),
        "no endpoint is built for a usage error"
    );
}

#[test]
fn unknown_flag_and_unknown_command_are_exit_2() {
    assert_eq!(go(&["hash", "--bogus"]).code, 2);
    assert_eq!(go(&["frobnicate"]).code, 2);
    assert_eq!(go(&["checkout"]).code, 2, "missing argument");
}

#[test]
fn version_and_help_exit_0_without_connecting() {
    let v = go(&["--version"]);
    assert_eq!(
        (v.code, v.out.trim().to_owned()),
        (0, sm_cli::version_line())
    );
    let hlp = go(&["--help"]);
    assert_eq!(hlp.code, 0);
    assert!(
        hlp.out.contains("export") && hlp.out.contains("serve") && hlp.out.contains("--fixed-time"),
        "{}",
        hlp.out
    );
    let sub = go(&["run", "--help"]);
    assert_eq!(sub.code, 0);
    assert!(sub.settings.is_none() && v.settings.is_none() && hlp.settings.is_none());
}

#[test]
fn server_errors_are_exit_1_with_the_contract_kind() {
    let reply = |_: &Request| Err(RpcError::application("op-invalid", "size must be positive"));
    let o = go_with(
        &["apply", "mesh.size", "{}", "--memory"],
        "",
        Box::new(reply),
    );
    assert_eq!(o.code, 1);
    assert!(
        o.err.contains("error[op-invalid]: size must be positive"),
        "{}",
        o.err
    );
    assert!(o.out.is_empty(), "human errors go to stderr only");
}

#[test]
fn server_error_data_is_printed_for_humans() {
    let reply = |_: &Request| {
        Err(RpcError::application("script-failed", "assert failed").with_data(json!({"line": 3})))
    };
    let o = go_with(
        &["run", "/nonexistent-but-checked-later"],
        "",
        Box::new(reply),
    );
    assert_eq!(
        o.code, 2,
        "unreadable script is a usage error before any call"
    );
    let dir = tmp("errdata");
    let s = dir.join("a.smesh");
    std::fs::write(&s, "assert false\n").unwrap();
    let reply = |_: &Request| {
        Err(RpcError::application("script-failed", "assert failed").with_data(json!({"line": 3})))
    };
    let o = go_with(
        &["run", s.to_str().unwrap(), "--memory"],
        "",
        Box::new(reply),
    );
    assert_eq!(o.code, 1);
    assert!(
        o.err.contains("script-failed") && o.err.contains("\"line\": 3"),
        "{}",
        o.err
    );
}

#[test]
fn json_errors_go_to_stdout_as_an_error_object() {
    let reply = |_: &Request| Err(RpcError::application("unknown-entity", "no such id"));
    let o = go_with(&["--json", "hash", "--memory"], "", Box::new(reply));
    assert_eq!(o.code, 1);
    let e: RpcError = serde_json::from_value(o.json()["error"].clone()).unwrap();
    assert_eq!(
        (e.kind.as_str(), e.code),
        ("unknown-entity", sm_protocol::APPLICATION_ERROR)
    );
    // A usage error detected after parsing is also an object, with kind `usage`.
    let u = go(&["--json", "apply", "x.y", "[1]", "--memory"]);
    assert_eq!(u.code, 2);
    assert_eq!(u.json()["error"]["kind"], "usage");
}

// ---- session options ---------------------------------------------------------------------

#[test]
fn memory_sessions_do_not_open_or_save() {
    let o = go(&["apply", "model.create_brick", r#"{"x":10}"#, "--memory"]);
    assert_eq!((o.code, methods(&o)), (0, vec!["apply"]));
    assert_eq!(o.settings, Some(Settings::default()));
}

#[test]
fn project_sessions_open_first_and_save_after_changes_only() {
    let o = go(&["apply", "model.create_brick", r#"{"x":10}"#, "-p", "proj"]);
    assert_eq!(methods(&o), ["session.open", "apply", "session.save"]);
    assert_eq!(
        o.requests[0],
        Request::Open {
            path: "proj".into(),
            create: false
        }
    );
    assert_eq!(o.settings.unwrap().project, Some("proj".into()));
    // -p works before or after the subcommand and read-only commands never save.
    for cmd in [["-p", "proj", "hash"], ["-p", "proj", "status"]] {
        let o = go(&cmd);
        assert_eq!(methods(&o)[0], "session.open");
        assert!(!methods(&o).contains(&"session.save"), "{:?}", methods(&o));
    }
    let o = go(&["log", "-p", "proj"]);
    assert_eq!(
        methods(&o),
        ["session.open", "timeline.status", "timeline.log"]
    );
}

#[test]
fn run_under_p_creates_a_missing_project_and_others_do_not() {
    let dir = tmp("runcreate");
    let s = dir.join("a.smesh");
    std::fs::write(&s, "print 1\n").unwrap();
    let o = go(&["run", s.to_str().unwrap(), "-p", "fresh/dir"]);
    assert_eq!(methods(&o), ["session.open", "script.run", "session.save"]);
    assert_eq!(
        o.requests[0],
        Request::Open {
            path: "fresh/dir".into(),
            create: true
        }
    );
    let o = go(&["fork", "alt", "-p", "fresh/dir"]);
    assert_eq!(
        o.requests[0],
        Request::Open {
            path: "fresh/dir".into(),
            create: false
        }
    );
}

#[test]
fn new_with_a_different_project_flag_is_a_usage_error() {
    let o = go(&["new", "d", "-p", "other"]);
    assert_eq!((o.code, o.requests.len()), (2, 0));
}

#[test]
fn failed_save_prints_only_the_error_object() {
    let reply = |r: &Request| match r {
        Request::Save => Err(RpcError::application("io", "disk full")),
        r => canned(r),
    };
    let o = go_with(
        &[
            "--json",
            "apply",
            "model.create_brick",
            r#"{"x":10}"#,
            "-p",
            "proj",
        ],
        "",
        Box::new(reply),
    );
    assert_eq!(o.code, 1);
    assert_eq!(o.out.lines().count(), 1, "{}", o.out);
    assert_eq!(o.json()["error"]["message"], "disk full");
}

#[test]
fn log_tolerates_non_ascii_node_strings() {
    let reply = |r: &Request| match r {
        Request::Log { .. } => Ok(json!([{"node": "ééééééééééééé", "op": "x"}])),
        r => canned(r),
    };
    let o = go_with(&["log", "--memory"], "", Box::new(reply));
    assert_eq!(o.code, 0);
    assert!(o.out.contains("éééééééééééé x"), "{}", o.out);
}

#[test]
fn project_and_memory_are_mutually_exclusive() {
    assert_eq!(go(&["status", "-p", "d", "--memory"]).code, 2);
}

#[test]
fn author_and_fixed_time_reach_the_connector_and_are_validated() {
    let o = go(&[
        "hash",
        "--memory",
        "--author",
        "it",
        "--fixed-time",
        "2026-10-24T00:00:00Z",
    ]);
    let s = o.settings.unwrap();
    assert_eq!(
        (s.author.as_deref(), s.fixed_time.as_deref()),
        (Some("it"), Some("2026-10-24T00:00:00Z"))
    );
    for bad in [
        "yesterday",
        "2026-10-24 00:00:00",
        "2026-10-24T00:00:00",
        "2026-10-24T00:00:00+01:00",
        "2026-99-99T99:99:99Z",
        "2026-02-29T00:00:00Z",
        "2026-04-31T00:00:00Z",
        "2026-10-24T24:00:00Z",
        "2026-10-24T00:60:00Z",
        "2026-00-10T00:00:00Z",
    ] {
        let o = go(&["hash", "--memory", "--fixed-time", bad]);
        assert_eq!(o.code, 2, "{bad}");
        assert!(
            o.settings.is_none(),
            "an invalid time is rejected before connecting"
        );
    }
}

#[test]
fn project_is_discovered_from_an_enclosing_directory() {
    let root = tmp("discover");
    std::fs::write(root.join("project.toml"), "").unwrap();
    let deep = root.join("a/b");
    std::fs::create_dir_all(&deep).unwrap();
    assert_eq!(discover_project(&deep), Some(root.clone()));
    let lone = tmp("nodiscover");
    // A temp directory may itself sit under a project in odd setups; only assert the positive case
    // when the ancestors are clean.
    if lone.ancestors().all(|a| !a.join("project.toml").exists()) {
        assert_eq!(discover_project(&lone), None);
    }
}

// ---- commands --------------------------------------------------------------------------

#[test]
fn new_creates_the_project_and_saves() {
    let o = go(&["new", "some/dir"]);
    assert_eq!(o.code, 0);
    assert_eq!(
        o.requests[0],
        Request::Open {
            path: "some/dir".into(),
            create: true
        }
    );
    assert_eq!(
        methods(&o),
        ["session.open", "timeline.status", "session.save"]
    );
    assert!(o.out.contains("created project some/dir"), "{}", o.out);
}

#[test]
fn run_sends_the_script_and_prints_its_output() {
    let dir = tmp("run");
    let s = dir.join("a.smesh");
    std::fs::write(&s, "print 1\n").unwrap();
    let o = go(&["run", s.to_str().unwrap(), "--memory"]);
    assert_eq!(o.code, 0);
    assert_eq!(o.out, "hello\n");
    let Request::ScriptRun {
        text,
        path,
        includes,
    } = &o.requests[0]
    else {
        panic!("{:?}", o.requests)
    };
    assert_eq!(
        (text.as_str(), path.as_deref(), includes.len()),
        ("print 1\n", s.to_str(), 0)
    );
}

#[test]
fn run_print_hash_appends_the_state_hash_as_the_last_line() {
    let dir = tmp("runhash");
    let s = dir.join("a.smesh");
    std::fs::write(&s, "print 1\n").unwrap();
    let o = go(&["run", s.to_str().unwrap(), "--memory", "--print-hash"]);
    let last = o.out.lines().last().unwrap();
    assert_eq!(last, h("s").to_string());
    assert_eq!(o.out.lines().next(), Some("hello"));
}

#[test]
fn run_json_is_the_script_result() {
    let dir = tmp("runjson");
    let s = dir.join("a.smesh");
    std::fs::write(&s, "print 1\n").unwrap();
    let o = go(&["run", s.to_str().unwrap(), "--memory", "--json"]);
    let r: ScriptResult = serde_json::from_value(o.json()).unwrap();
    assert_eq!((r.state, r.output.as_str()), (h("s"), "hello\n"));
}

#[test]
fn run_sends_included_files_read_next_to_the_script() {
    let dir = tmp("include");
    std::fs::create_dir_all(dir.join("lib")).unwrap();
    std::fs::write(
        dir.join("lib/mat.smesh"),
        "include \"lib/deep.smesh\"\nlet a = 1\n",
    )
    .unwrap();
    std::fs::write(dir.join("lib/deep.smesh"), "let z = 2\n").unwrap();
    std::fs::write(
        dir.join("main.smesh"),
        "// include \"commented.smesh\"\ninclude \"lib/mat.smesh\"\ninclude \"missing.smesh\"\n",
    )
    .unwrap();
    let o = go(&["run", dir.join("main.smesh").to_str().unwrap(), "--memory"]);
    let Request::ScriptRun { includes, .. } = &o.requests[0] else {
        panic!()
    };
    let keys: Vec<&str> = includes.keys().map(String::as_str).collect();
    assert_eq!(
        keys,
        ["lib/deep.smesh", "lib/mat.smesh"],
        "nested includes found, comments and missing files skipped"
    );
    assert_eq!(includes["lib/deep.smesh"], "let z = 2\n");
}

#[test]
fn run_of_a_missing_script_is_a_usage_error() {
    let o = go(&["run", "/no/such/file.smesh", "--memory"]);
    assert_eq!(o.code, 2);
    assert!(o.requests.is_empty());
}

#[test]
fn apply_builds_the_request_and_prints_node_state_and_roles() {
    let o = go(&["apply", "model.create_brick", r#"{"x": 10}"#, "--memory"]);
    assert_eq!(o.code, 0);
    assert_eq!(
        o.requests,
        [Request::Apply {
            op: "model.create_brick".into(),
            params: json!({"x": 10}),
            version: None
        }]
    );
    assert!(o.out.starts_with("model.create_brick: node "), "{}", o.out);
    assert!(o.out.contains("volume: TopoVolume#1"), "{}", o.out);
    let o = go(&[
        "apply",
        "mesh.size",
        "{}",
        "--payload-version",
        "3",
        "--memory",
    ]);
    assert!(matches!(
        &o.requests[0],
        Request::Apply {
            version: Some(3),
            ..
        }
    ));
}

#[test]
fn apply_json_is_the_apply_result_and_reads_params_from_stdin() {
    let o = go_with(
        &["apply", "mesh.size", "-", "--json", "--memory"],
        r#"{"size": 1.0}"#,
        Box::new(canned),
    );
    let r: ApplyResult = serde_json::from_value(o.json()).unwrap();
    assert_eq!(r.node, h("n"));
    assert_eq!(r.delta.outputs["volume"][0].to_string(), "TopoVolume#1");
    assert!(
        matches!(&o.requests[0], Request::Apply { params, .. } if params == &json!({"size": 1.0}))
    );
}

#[test]
fn apply_rejects_malformed_params_locally() {
    for bad in ["{x:", "[1,2]", "7", ""] {
        let o = go(&["apply", "mesh.size", bad, "--memory"]);
        assert_eq!(o.code, 2, "{bad:?}");
        assert!(o.requests.is_empty(), "{bad:?} must not reach the server");
    }
}

#[test]
fn query_defaults_to_null_input_and_parses_revisions() {
    let o = go(&["query", "mesh.info", "--memory"]);
    assert_eq!(
        o.requests,
        [Request::Query {
            name: "mesh.info".into(),
            input: Value::Null,
            at: None
        }]
    );
    assert_eq!(
        serde_json::from_str::<Value>(&o.out).unwrap(),
        json!({"cells": 1000}),
        "pretty JSON in human mode"
    );
    let hex = h("n1").to_string();
    for (arg, rev) in [
        ("main", Rev::Ref("main".into())),
        ("empty", Rev::Empty),
        (hex.as_str(), Rev::Node(h("n1"))),
    ] {
        let o = go(&[
            "query",
            "mesh.info",
            r#"{"mesh":"Mesh#1"}"#,
            "--at",
            arg,
            "--memory",
        ]);
        assert_eq!(
            o.requests,
            [Request::Query {
                name: "mesh.info".into(),
                input: json!({"mesh": "Mesh#1"}),
                at: Some(rev)
            }],
            "--at {arg}"
        );
    }
    let json = go(&["query", "mesh.info", "--json", "--memory"]);
    assert_eq!(json.out, "{\"cells\":1000}\n", "one compact line");
}

#[test]
fn status_prints_head_node_and_branches_and_json_is_verbatim() {
    let o = go(&["status", "--memory"]);
    assert_eq!(o.requests, [Request::Status]);
    for want in [
        "head:     main",
        "state:    ",
        "branch:   alt ",
        "branch:   main ",
        "redo: 1",
    ] {
        assert!(o.out.contains(want), "{want}: {}", o.out);
    }
    let j = go(&["status", "--json", "--memory"]);
    let s: StatusResult = serde_json::from_value(j.json()).unwrap();
    assert_eq!((s.depth, s.nodes, s.redo), (2, 2, 1));
    assert_eq!(j.json(), status_value());
}

#[test]
fn log_asks_for_status_and_log_and_tags_branch_tips() {
    let o = go(&["log", "--memory"]);
    assert_eq!(
        o.requests,
        [
            Request::Status,
            Request::Log {
                from: None,
                limit: 20
            }
        ]
    );
    let lines: Vec<&str> = o.out.lines().collect();
    assert_eq!(lines[0], "HEAD main");
    assert!(
        lines[1].starts_with(&h("n2").to_string()[..12])
            && lines[1].contains("(main)")
            && lines[1].contains("mesh.size"),
        "{}",
        lines[1]
    );
    assert!(
        lines[2].contains("(alt)") && lines[2].contains("make a brick"),
        "{}",
        lines[2]
    );
    let o = go(&["log", "-n", "5", "--from", &h("n1").to_string(), "--memory"]);
    assert_eq!(
        o.requests[1],
        Request::Log {
            from: Some(h("n1")),
            limit: 5
        }
    );
}

#[test]
fn log_json_is_one_object_with_head_branches_and_nodes() {
    let o = go(&["log", "--json", "--memory"]);
    let v = o.json();
    assert_eq!(v["head"], json!({"ref": "main"}));
    assert_eq!(
        v["branches"]
            .as_object()
            .unwrap()
            .keys()
            .collect::<Vec<_>>(),
        ["alt", "main"]
    );
    assert_eq!(v["nodes"].as_array().unwrap().len(), 2);
    assert_eq!(v["nodes"][0]["op"], "mesh.size");
}

#[test]
fn hash_prints_the_state_hash_and_optionally_layers() {
    let o = go(&["hash", "--memory"]);
    assert_eq!(
        (o.requests.clone(), o.out.clone()),
        (vec![Request::Hash { at: None }], format!("{}\n", h("s")))
    );
    let o = go(&["hash", "--layers", "--memory"]);
    let lines: Vec<&str> = o.out.lines().collect();
    assert_eq!(lines.len(), 3);
    assert!(
        lines[1].starts_with("  meshes: ") && lines[2].starts_with("  model: "),
        "layers sorted: {lines:?}"
    );
    let o = go(&["hash", "--at", "main", "--json", "--memory"]);
    assert_eq!(
        o.requests,
        [Request::Hash {
            at: Some(Rev::Ref("main".into()))
        }]
    );
    let r: HashResult = serde_json::from_value(o.json()).unwrap();
    assert_eq!(r.state, h("s"));
}

#[test]
fn checkout_fork_undo_redo_map_to_timeline_requests() {
    assert_eq!(
        go(&["checkout", "alt", "--memory"]).requests,
        [Request::Checkout {
            rev: Rev::Ref("alt".into())
        }]
    );
    assert_eq!(
        go(&["checkout", "empty", "--memory"]).requests,
        [Request::Checkout { rev: Rev::Empty }]
    );
    let hex = h("n1").to_string();
    assert_eq!(
        go(&["checkout", &hex, "--memory"]).requests,
        [Request::Checkout {
            rev: Rev::Node(h("n1"))
        }]
    );
    assert_eq!(
        go(&["fork", "alt", "--memory"]).requests,
        [Request::Branch {
            name: "alt".into(),
            at: None
        }]
    );
    assert_eq!(
        go(&["fork", "alt", "--at", &hex, "--memory"]).requests,
        [Request::Branch {
            name: "alt".into(),
            at: Some(h("n1"))
        }]
    );
    assert_eq!(go(&["undo", "--memory"]).requests, [Request::Undo]);
    assert_eq!(go(&["redo", "--memory"]).requests, [Request::Redo]);
    // All of them print the status afterwards and change a project, so they save under -p.
    let o = go(&["checkout", "alt", "-p", "d"]);
    assert_eq!(
        methods(&o),
        ["session.open", "timeline.checkout", "session.save"]
    );
    assert!(o.out.contains("head:     main"));
}

#[test]
fn fork_at_rejects_a_bad_hash() {
    for bad in ["abc", &"G".repeat(64), &h("x").to_string().to_uppercase()] {
        let o = go(&["fork", "x", "--at", bad, "--memory"]);
        assert_eq!(o.code, 2, "{bad}");
        assert!(o.requests.is_empty());
    }
}

#[test]
fn export_infers_the_format_from_the_extension() {
    let o = go(&["export", "out/a.smsh", "--memory"]);
    assert_eq!(
        o.requests,
        [Request::Export {
            path: Some("out/a.smsh".into()),
            format: Some(ExportFormat::Smsh),
            mesh: None,
            provenance: false,
            inline: false
        }]
    );
    assert!(
        o.out.starts_with("wrote out/a.smsh (42 bytes, sha256 "),
        "{}",
        o.out
    );
    let o = go(&["export", "a.vtu", "--memory"]);
    assert!(matches!(
        &o.requests[0],
        Request::Export {
            format: Some(ExportFormat::Vtu),
            ..
        }
    ));
}

#[test]
fn export_flags_mesh_format_and_provenance() {
    let o = go(&[
        "export",
        "dump.bin",
        "--format",
        "vtu",
        "--mesh",
        "Mesh#7",
        "--provenance",
        "--memory",
    ]);
    let Request::Export {
        path,
        format,
        mesh,
        provenance,
        inline,
    } = &o.requests[0]
    else {
        panic!()
    };
    assert_eq!(
        (
            path.as_deref(),
            *format,
            mesh.map(|m| m.to_string()),
            *provenance,
            *inline
        ),
        (
            Some("dump.bin"),
            Some(ExportFormat::Vtu),
            Some("Mesh#7".to_owned()),
            true,
            false
        )
    );
}

#[test]
fn export_json_is_the_export_result_and_bad_inputs_are_usage_errors() {
    let o = go(&["export", "a.smsh", "--json", "--memory"]);
    let r: ExportResult = serde_json::from_value(o.json()).unwrap();
    assert_eq!((r.bytes, r.format), (42, ExportFormat::Smsh));
    for args in [
        &["export", "a.txt", "--memory"][..],
        &["export", "a.smsh", "--mesh", "nope", "--memory"],
        &["export", "a.smsh", "--format", "stl", "--memory"],
    ] {
        let o = go(args);
        assert_eq!(o.code, 2, "{args:?}");
        assert!(o.requests.is_empty());
    }
}

// ---- transports ----------------------------------------------------------------------------

#[test]
fn serve_stdio_answers_one_line_per_request_and_reports_parse_errors() {
    let input = "{\"jsonrpc\":\"2.0\",\"id\":7,\"method\":\"timeline.status\"}\n\n{not json\n{\"jsonrpc\":\"2.0\",\"id\":8,\"method\":\"session.hash\",\"params\":{}}\n";
    let o = go_with(&["serve", "--stdio", "--memory"], input, Box::new(canned));
    assert_eq!(o.code, 0);
    let replies: Vec<Value> = o
        .out
        .lines()
        .map(|l| serde_json::from_str(l).unwrap())
        .collect();
    assert_eq!(
        replies.len(),
        3,
        "blank lines are skipped, bad lines are answered"
    );
    assert_eq!(
        (
            replies[0]["id"].clone(),
            replies[0]["result"]["depth"].clone()
        ),
        (json!(7), json!(2))
    );
    assert_eq!(
        (
            replies[1]["id"].clone(),
            replies[1]["error"]["code"].clone()
        ),
        (Value::Null, json!(sm_protocol::PARSE_ERROR))
    );
    assert_eq!(replies[2]["result"]["state"], h("s").to_string());
    assert_eq!(methods(&o), ["timeline.status", "session.hash"]);
}

#[test]
fn serve_defaults_to_stdio_and_ws_is_reported_unavailable() {
    let o = go_with(
        &["serve", "--memory"],
        "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"timeline.status\"}\n",
        Box::new(canned),
    );
    assert_eq!((o.code, o.out.lines().count()), (0, 1));
    let o = go(&["serve", "--ws", "127.0.0.1:0", "--memory"]);
    assert_eq!(o.code, 2);
    assert!(o.err.contains("WebSocket"), "{}", o.err);
    assert_eq!(go(&["serve", "--stdio", "--ws", "x", "--memory"]).code, 2);
}

#[test]
fn mcp_runs_the_mcp_loop_over_stdio() {
    let input = "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\",\"params\":{\"protocolVersion\":\"2025-06-18\",\"capabilities\":{},\"clientInfo\":{\"name\":\"t\",\"version\":\"0\"}}}\n";
    let o = go_with(&["mcp", "--memory"], input, Box::new(canned));
    assert_eq!(o.code, 0);
    let first: Value = serde_json::from_str(o.out.lines().next().unwrap()).unwrap();
    assert_eq!(first["id"], 1);
    assert_eq!(first["result"]["protocolVersion"], "2025-06-18");
    // Under -p the project is opened first and saved after the session.
    let o = go_with(&["mcp", "-p", "d"], input, Box::new(canned));
    assert_eq!(methods(&o).first(), Some(&"session.open"));
    assert_eq!(methods(&o).last(), Some(&"session.save"));
}

// ---- help --------------------------------------------------------------------------------

#[test]
fn help_lists_ops_and_queries_sorted_after_the_usage() {
    let o = go(&["help", "--memory"]);
    assert_eq!((o.code, o.requests.clone()), (0, vec![Request::Catalog]));
    assert!(o.out.contains("Usage: smesh"), "{}", o.out);
    assert!(
        o.out.contains("mesh.size") && o.out.contains("Set the target size."),
        "{}",
        o.out
    );
    assert!(o.out.contains("mesh.info") && o.out.contains("Mesh counts."));
    assert!(
        !o.out.contains("Second line."),
        "only the first description line is listed"
    );
}

#[test]
fn help_op_describes_params_kinds_and_outputs() {
    for name in ["mesh.size", "mesh__size"] {
        let o = go(&["help", name]);
        assert_eq!(o.code, 0, "{name}");
        for want in [
            "mesh.size (op, version 1, tool mesh__size)",
            "Second line.",
            "targets: array<string> [required, positional, ids of TopoVolume]  what to size",
            "size: number  edge length",
            "sized: many of TopoVolume  the sized volumes",
        ] {
            assert!(o.out.contains(want), "{want}\n{}", o.out);
        }
    }
    let q = go(&["help", "mesh.info"]);
    assert!(
        q.out.starts_with("mesh.info (query, tool mesh__info)"),
        "{}",
        q.out
    );
}

#[test]
fn help_unknown_name_is_a_usage_error_with_suggestions() {
    let o = go(&["help", "mesh.siz"]);
    assert_eq!(o.code, 2);
    assert!(
        o.err.contains("no op or query named `mesh.siz`"),
        "{}",
        o.err
    );
    let o = go(&["help", "size"]);
    assert!(o.err.contains("did you mean mesh.size"), "{}", o.err);
}

#[test]
fn help_json_is_the_catalog_or_one_entry() {
    let all = go(&["help", "--json"]).json();
    assert_eq!(all["ops"][0]["name"], "mesh.size");
    let one = go(&["help", "mesh.size", "--json"]).json();
    assert_eq!(
        (one["kind"].clone(), one["tool_name"].clone()),
        (json!("op"), json!("mesh__size"))
    );
    let q = go(&["help", "mesh.info", "--json"]).json();
    assert_eq!(q["kind"], "query");
}

#[test]
fn clap_errors_are_json_on_stdout_when_json_is_requested() {
    let o = go(&["--json", "apply", "mesh.size"]);
    assert_eq!(o.code, 2);
    assert!(o.err.is_empty(), "{}", o.err);
    let v = o.json();
    assert_eq!(v["error"]["kind"], "usage");
    assert!(
        v["error"]["message"].as_str().unwrap().contains("required"),
        "{v}"
    );
    // Help and version stay plain text.
    assert!(!go(&["--json", "--version"]).out.starts_with('{'));
}
