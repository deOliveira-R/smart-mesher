//! MCP server tests against an in-process mock `Endpoint`.
#![allow(missing_docs)]

mod common;

use common::*;
use serde_json::{Value, json};
use sm_protocol::{ExportFormat, Request};

fn tools(mock: &mut Mock, version: &str) -> Vec<Value> {
    let r = run(mock, &[init(version), list(1)]);
    r[1]["result"]["tools"].as_array().unwrap().clone()
}

fn names(tools: &[Value]) -> Vec<String> {
    tools.iter().map(|t| t["name"].as_str().unwrap().to_owned()).collect()
}

// ---- golden transcript + determinism ----

fn transcript() -> String {
    let msgs = [
        init("2025-06-18"),
        json!({"jsonrpc":"2.0","method":"notifications/initialized"}),
        list(2),
        call(3, "model__create_brick", json!({"x": 10})),
        call(4, "mesh__run", json!({"targets": ["TopoVolume#1"]})),
        call(5, "demo__count", json!({})),
        call(6, "topo__related", json!({"within": "TopoVolume#1"})),
        call(7, "session_export", json!({"path": "out.smsh"})),
        call(8, "session_export", json!({"path": "out.smsh", "mesh": "Mesh#1"})),
        call(9, "timeline_checkout", json!({"rev": "main"})),
        call(10, "run_script", json!({"text": "fail"})),
        call(11, "session_hash", json!({})),
        json!({"jsonrpc":"2.0","id":"p","method":"ping"}),
        call(12, "nope", json!({})),
        json!({"jsonrpc":"2.0","id":13,"method":"resources/list"}),
    ];
    msgs.iter().map(|m| format!("{m}\n")).collect::<String>() + "{broken\n"
}

fn golden_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden")
}

#[test]
fn golden_session_transcript() {
    let input = transcript();
    let out = run_raw(&mut Mock::new(), &input);
    let out_name = if cfg!(feature = "respell") { "session.out.respell.jsonl" } else { "session.out.jsonl" };
    let (fi, fo) = (golden_dir().join("session.in.jsonl"), golden_dir().join(out_name));
    if std::env::var_os("SM_MCP_BLESS").is_some() {
        std::fs::write(&fi, &input).unwrap();
        std::fs::write(&fo, &out).unwrap();
    }
    assert_eq!(input, std::fs::read_to_string(&fi).unwrap(), "input golden drifted");
    assert_eq!(out, std::fs::read_to_string(&fo).unwrap(), "output golden drifted");
}

#[test]
fn determinism_same_bytes_across_runs_and_threads() {
    let input = transcript();
    let first = run_raw(&mut Mock::new(), &input);
    assert_eq!(first, run_raw(&mut Mock::new(), &input));
    let t = std::thread::spawn(move || run_raw(&mut Mock::new(), &transcript()));
    assert_eq!(first, t.join().unwrap());
    // One reply per request: 13 requests + the broken line; the notification gets none.
    assert_eq!(first.lines().count(), 15);
}

// ---- tools/list ----

#[test]
fn tools_list_sorted_stable_valid() {
    let a = tools(&mut Mock::new(), "2025-06-18");
    let b = tools(&mut Mock::new(), "2025-06-18");
    assert_eq!(a, b);
    let n = names(&a);
    let mut sorted = n.clone();
    sorted.sort();
    sorted.dedup();
    assert_eq!(n, sorted, "sorted and unique");
    for t in &a {
        let name = t["name"].as_str().unwrap();
        assert!(!name.is_empty() && name.len() <= 64, "{name}");
        assert!(name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-'));
        assert_eq!(t["inputSchema"]["type"], "object", "{name}");
        assert!(t["description"].as_str().is_some_and(|d| !d.is_empty()), "{name}");
    }
}

#[test]
fn tools_cover_catalog_batch_and_fixed() {
    let n = names(&tools(&mut Mock::new(), "2025-06-18"));
    for want in [
        "core__batch", "model__create_brick", "mesh__run", "demo__count", "topo__related",
        "run_script", "session_hash", "session_export", "session_status", "timeline_branch",
        "timeline_checkout", "timeline_undo", "timeline_redo", "timeline_log", "help",
    ] {
        assert!(n.iter().any(|x| x == want), "missing {want}: {n:?}");
    }
    // Fixed tools never contain `__`, op and query tools always do.
    for x in &n {
        let fixed = !x.contains("__");
        let is_fixed = [
            "run_script", "session_hash", "session_export", "session_status", "timeline_branch",
            "timeline_checkout", "timeline_undo", "timeline_redo", "timeline_log", "help",
        ]
        .contains(&x.as_str());
        assert_eq!(fixed, is_fixed, "{x}");
    }
    assert_eq!(n.len(), 18);
}

#[test]
fn schemas_keep_extensions_and_describe_outputs() {
    let ts = tools(&mut Mock::new(), "2025-06-18");
    let by = |n: &str| ts.iter().find(|t| t["name"] == n).unwrap().clone();
    let run_t = by("mesh__run");
    assert_eq!(
        run_t["inputSchema"]["properties"]["targets"]["x-entity-kind"],
        json!(["TopoVolume"])
    );
    let brick = by("model__create_brick");
    assert_eq!(brick["inputSchema"]["properties"]["x"]["x-positional"], json!(true));
    let d = brick["description"].as_str().unwrap();
    assert!(d.contains("- volume (TopoVolume, one): the brick volume"), "{d}");
    assert!(d.contains("- faces (TopoFace, many): the six faces"), "{d}");
    assert_eq!(brick["title"], "model.create_brick");
    assert!(brick.get("annotations").is_none(), "ops are not read-only");
    assert_eq!(by("demo__count")["annotations"], json!({"readOnlyHint": true}));
    // core__batch carries the registry's batch schema.
    let batch = by("core__batch");
    assert_eq!(batch["inputSchema"]["title"], "core.batch");
    assert!(batch["inputSchema"]["properties"]["ops"].is_object());
    // Keys are sorted at every level.
    let s = serde_json::to_string(&brick).unwrap();
    assert!(s.find("\"description\"").unwrap() < s.find("\"inputSchema\"").unwrap());
}

#[test]
fn hostile_catalog_entries_are_dropped() {
    let mut m = Mock::new();
    let tpl = |name: &str, tool: &str| {
        json!({"name": name, "tool_name": tool, "version": 1,
               "schema": {"type": "object"}, "output_schema": {}, "outputs": [], "positional": null})
    };
    m.extra_ops = vec![
        tpl("x.long", &format!("x__{}", "a".repeat(70))),
        tpl("x.space", "x__sp ace"),
        tpl("x.empty", ""),
        tpl("x.dup", "mesh__run"),
        tpl("x.shadow", "help"),
        tpl("x.good", "x__good"),
    ];
    let ts = tools(&mut m, "2025-06-18");
    let n = names(&ts);
    assert!(n.contains(&"x__good".to_owned()));
    assert!(n.iter().all(|x| x.len() <= 64 && !x.contains(' ') && !x.is_empty()));
    assert_eq!(n.iter().filter(|x| *x == "mesh__run").count(), 1);
    assert_eq!(n.iter().filter(|x| *x == "help").count(), 1);
    // The fixed `help` wins over the shadowing op.
    let help = ts.iter().find(|t| t["name"] == "help").unwrap();
    assert!(help["description"].as_str().unwrap().starts_with("List the available tools"));
}

// ---- negotiation ----

#[test]
fn negotiation_echoes_supported_and_answers_latest_otherwise() {
    for (ask, got) in [
        ("2025-06-18", "2025-06-18"),
        ("2025-03-26", "2025-03-26"),
        ("2024-11-05", "2024-11-05"),
        ("1.0.0", "2025-06-18"),
        ("2099-01-01", "2025-06-18"),
    ] {
        let r = run(&mut Mock::new(), &[init(ask)]);
        assert_eq!(r[0]["result"]["protocolVersion"], got, "asked {ask}");
        assert_eq!(r[0]["result"]["capabilities"]["tools"]["listChanged"], false);
        assert_eq!(r[0]["result"]["serverInfo"]["name"], "smesh");
    }
    let bad = json!({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":5}});
    assert_eq!(run(&mut Mock::new(), &[bad])[0]["error"]["code"], -32602);
    // No params: latest.
    let none = json!({"jsonrpc":"2.0","id":1,"method":"initialize"});
    assert_eq!(run(&mut Mock::new(), &[none])[0]["result"]["protocolVersion"], "2025-06-18");
}

#[test]
fn old_versions_omit_newer_fields() {
    // 2025-03-26: annotations yes, title and structuredContent no.
    let mut m = Mock::new();
    let r = run(&mut m, &[init("2025-03-26"), list(1), call(2, "model__create_brick", json!({"x": 1}))]);
    let ts = r[1]["result"]["tools"].as_array().unwrap();
    assert!(ts.iter().all(|t| t.get("title").is_none()));
    let count = ts.iter().find(|t| t["name"] == "demo__count").unwrap();
    assert_eq!(count["annotations"]["readOnlyHint"], true);
    assert!(r[0]["result"]["serverInfo"].get("title").is_none());
    assert!(r[2]["result"].get("structuredContent").is_none());
    let text = r[2]["result"]["content"][0]["text"].as_str().unwrap();
    assert!(serde_json::from_str::<Value>(text).unwrap()["delta"]["outputs"]["volume"].is_array());
    assert_eq!(r[2]["result"]["isError"], false);

    // 2024-11-05: no annotations either.
    let r = run(&mut Mock::new(), &[init("2024-11-05"), list(1)]);
    let ts = r[1]["result"]["tools"].as_array().unwrap();
    assert!(ts.iter().all(|t| t.get("title").is_none() && t.get("annotations").is_none()));
    assert!(ts.iter().all(|t| t.get("outputSchema").is_none()));

    // 2025-06-18: all of it.
    let r = run(&mut Mock::new(), &[init("2025-06-18"), list(1)]);
    let t = &r[1]["result"]["tools"][0];
    assert!(t.get("title").is_some());
}

// ---- protocol errors ----

#[test]
fn unknown_tool_and_method_and_bad_params() {
    let mut m = Mock::new();
    let r = run(
        &mut m,
        &[
            call(1, "no_such_tool", json!({})),
            call(2, "no__such", json!({})),
            json!({"jsonrpc":"2.0","id":3,"method":"nope/nope"}),
            json!({"jsonrpc":"2.0","id":4,"method":"tools/call","params":{}}),
            json!({"jsonrpc":"2.0","id":5,"method":"tools/call","params":{"name":7}}),
            json!({"jsonrpc":"2.0","id":6,"method":"tools/call","params":{"name":"help","arguments":[1]}}),
            json!({"jsonrpc":"2.0","id":7,"method":"tools/list","params":{"cursor":"abc"}}),
            json!({"jsonrpc":"2.0","id":8,"method":"tools/call"}),
            json!({"jsonrpc":"2.0","id":9,"method":"resources/list"}),
        ],
    );
    assert_eq!(r[0]["error"]["code"], -32602);
    assert_eq!(r[0]["error"]["message"], "Unknown tool: no_such_tool");
    assert_eq!(r[1]["error"]["code"], -32602);
    assert_eq!(r[2]["error"]["code"], -32601);
    for i in [3, 4, 5, 6, 7] {
        assert_eq!(r[i]["error"]["code"], -32602, "reply {i}: {}", r[i]);
    }
    assert_eq!(r[8]["error"]["code"], -32601);
    for (i, rep) in r.iter().enumerate() {
        assert_eq!(rep["id"], json!([1, 2, 3, 4, 5, 6, 7, 8, 9][i]));
    }
}

#[test]
fn framing_and_malformed_input() {
    let mut m = Mock::new();
    let input = [
        "{not json",                                        // parse error
        "[1,2]",                                            // batch
        "[]",
        "42",
        "\"str\"",
        "{\"jsonrpc\":\"2.0\",\"id\":1}",                   // no method
        "{\"jsonrpc\":\"2.0\",\"id\":null,\"method\":\"ping\"}", // null id
        "{\"jsonrpc\":\"2.0\",\"id\":{},\"method\":\"ping\"}",
        "{\"jsonrpc\":\"1.0\",\"id\":2,\"method\":\"ping\"}",
        "{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":5}",
        "{\"jsonrpc\":\"2.0\",\"method\":\"tools/list\"}",    // notification: silent
        "{\"jsonrpc\":\"2.0\",\"method\":\"notifications/cancelled\",\"params\":{\"requestId\":1}}",
        "{\"jsonrpc\":\"2.0\",\"id\":9,\"result\":{}}",     // client response: silent
        "",
        "   ",
        "{\"jsonrpc\":\"2.0\",\"id\":\"s\",\"method\":\"ping\"}\r", // CRLF
    ]
    .join("\n");
    let out = run_raw(&mut m, &input); // no trailing newline on the last line
    let r: Vec<Value> = out.lines().map(|l| serde_json::from_str(l).unwrap()).collect();
    let codes: Vec<(Value, i64)> = r
        .iter()
        .take(10)
        .map(|v| (v["id"].clone(), v["error"]["code"].as_i64().unwrap()))
        .collect();
    assert_eq!(
        codes,
        vec![
            (Value::Null, -32700),
            (Value::Null, -32600),
            (Value::Null, -32600),
            (Value::Null, -32600),
            (Value::Null, -32600),
            (Value::Null, -32600),
            (Value::Null, -32600),
            (Value::Null, -32600),
            (json!(2), -32600),
            (json!(3), -32600),
        ]
    );
    assert_eq!(r.len(), 11);
    assert_eq!(r[10], json!({"jsonrpc":"2.0","id":"s","result":{}}));
    assert!(m.log.is_empty(), "nothing reached the endpoint");
}

#[test]
fn invalid_utf8_is_a_parse_error_not_a_crash() {
    let mut out = Vec::new();
    let mut input = b"\xff\xfe\n".to_vec();
    input.extend_from_slice(b"{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"ping\"}\n");
    sm_mcp::run_stdio(&mut Mock::new(), std::io::Cursor::new(input), &mut out).unwrap();
    let lines: Vec<Value> =
        String::from_utf8(out).unwrap().lines().map(|l| serde_json::from_str(l).unwrap()).collect();
    assert_eq!(lines[0]["error"]["code"], -32700);
    assert_eq!(lines[1]["result"], json!({}));
}

#[test]
fn ping_works_before_initialize_and_eof_ends_cleanly() {
    let r = run(&mut Mock::new(), &[json!({"jsonrpc":"2.0","id":"123","method":"ping"})]);
    assert_eq!(r, vec![json!({"jsonrpc":"2.0","id":"123","result":{}})]);
    assert_eq!(run_raw(&mut Mock::new(), ""), "");
}

// ---- tools/call ----

#[test]
fn op_call_maps_to_apply_and_returns_structured_and_text() {
    let mut m = Mock::new();
    let r = run(&mut m, &[call(1, "model__create_brick", json!({"x": 10}))]);
    assert_eq!(
        m.log.last().unwrap(),
        &Request::Apply { op: "model.create_brick".into(), params: json!({"x": 10}), version: None }
    );
    let res = &r[0]["result"];
    assert_eq!(res["isError"], false);
    assert_eq!(res["structuredContent"]["delta"]["outputs"]["volume"], json!(["TopoVolume#1"]));
    let text = res["content"][0]["text"].as_str().unwrap();
    assert!(!text.contains('\n'));
    let parsed = serde_json::from_str::<Value>(text).unwrap();
    if cfg!(not(feature = "respell")) {
        assert_eq!(parsed, res["structuredContent"]);
    }
    assert_eq!(res["content"][0]["type"], "text");
}

#[test]
fn missing_arguments_default_to_empty_object() {
    let mut m = Mock::new();
    run(&mut m, &[json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"mesh__run"}})]);
    assert_eq!(
        m.log.last().unwrap(),
        &Request::Apply { op: "mesh.run".into(), params: json!({}), version: None }
    );
}

#[test]
fn query_call_maps_to_query_and_wraps_non_objects() {
    let mut m = Mock::new();
    let r = run(&mut m, &[call(1, "demo__count", json!({})), call(2, "topo__related", json!({"within":"TopoVolume#1"}))]);
    assert_eq!(
        m.log[1],
        Request::Query { name: "demo.count".into(), input: json!({}), at: None }
    );
    assert_eq!(r[0]["result"]["structuredContent"], json!({"result": 7}));
    assert_eq!(
        r[1]["result"]["structuredContent"],
        json!({"result": ["TopoFace#1", "TopoFace#2"]})
    );
    assert_eq!(r[1]["result"]["isError"], false);
}

#[test]
fn application_errors_are_iserror_results() {
    let mut m = Mock::new();
    let r = run(&mut m, &[call(1, "mesh__run", json!({"targets":["TopoVolume#1"]})), call(2, "run_script", json!({"text":"fail"}))]);
    let e = &r[0]["result"];
    assert_eq!(r[0].get("error"), None);
    assert_eq!(e["isError"], true);
    assert_eq!(e["structuredContent"]["error"]["kind"], "op-invalid");
    assert_eq!(e["structuredContent"]["error"]["code"], -32000);
    let text = e["content"][0]["text"].as_str().unwrap();
    assert!(text.contains("has no scheme"), "{text}");
    // Error data survives.
    assert_eq!(r[1]["result"]["structuredContent"]["error"]["data"], json!({"line": 1}));
    assert_eq!(r[1]["result"]["isError"], true);
}

#[test]
fn non_application_endpoint_errors_are_protocol_errors() {
    let mut m = Mock::new();
    let r = run(&mut m, &[call(1, "demo__badparams", json!({})), call(2, "demo__internal", json!({}))]);
    assert_eq!(r[0]["error"]["code"], -32602);
    assert_eq!(r[0]["error"]["data"]["kind"], "invalid-params");
    assert_eq!(r[1]["error"]["code"], -32603);
    assert!(r[1]["error"]["message"].as_str().unwrap().contains("boom"));
}

#[test]
fn large_results_are_one_line() {
    let mut m = Mock::new();
    m.big = 200_000;
    let out = run_raw(&mut m, &format!("{}\n", call(1, "demo__big", json!({}))));
    assert!(out.len() > 2_000_000, "{}", out.len());
    assert_eq!(out.matches('\n').count(), 1);
    assert!(out.ends_with('\n'));
    let v: Value = serde_json::from_str(out.trim_end()).unwrap();
    assert_eq!(v["result"]["structuredContent"]["result"].as_array().unwrap().len(), 200_000);
}

#[test]
fn catalog_is_fetched_once_and_fixed_tools_never_need_it() {
    let mut m = Mock::new();
    run(&mut m, &[call(1, "session_status", json!({})), call(2, "session_hash", json!({}))]);
    assert_eq!(m.catalog_calls(), 0);
    run(&mut m, &[list(1), list(2), call(3, "mesh__run", json!({"targets":[]})), call(4, "help", json!({}))]);
    assert_eq!(m.catalog_calls(), 1);
}

#[test]
fn core_batch_is_applied_as_core_batch() {
    let mut m = Mock::new();
    let ops = json!({"ops": [{"name": "model.create_brick", "version": 1, "params": {"x": 1}}]});
    run(&mut m, &[call(1, "core__batch", ops.clone())]);
    assert_eq!(
        m.log.last().unwrap(),
        &Request::Apply { op: "core.batch".into(), params: ops, version: None }
    );
}

// ---- fixed tools ----

#[test]
fn fixed_tools_map_to_protocol_requests() {
    let node = "ab".repeat(32);
    let mut m = Mock::new();
    let r = run(
        &mut m,
        &[
            call(1, "run_script", json!({"text": "print 1", "path": "a.sm", "includes": {"b.sm": "x"}})),
            call(2, "session_hash", json!({})),
            call(3, "session_hash", json!({"at": "empty"})),
            call(4, "session_hash", json!({"at": node})),
            call(5, "session_export", json!({"path": "o.vtu", "format": "vtu", "mesh": "Mesh#1", "provenance": true, "inline": true})),
            call(6, "session_status", json!({})),
            call(7, "timeline_branch", json!({"name": "alt", "at": node})),
            call(8, "timeline_checkout", json!({"rev": "main"})),
            call(9, "timeline_checkout", json!({"rev": {"ref": "x"}})),
            call(10, "timeline_undo", json!({})),
            call(11, "timeline_redo", json!({})),
            call(12, "timeline_log", json!({})),
            call(13, "timeline_log", json!({"limit": 3, "from": node})),
        ],
    );
    assert!(r.iter().all(|x| x["result"]["isError"] == false), "{r:?}");
    let reqs = &m.log;
    assert!(matches!(&reqs[0], Request::ScriptRun { text, path: Some(p), includes } if text == "print 1" && p == "a.sm" && includes.len() == 1));
    assert!(matches!(&reqs[1], Request::Hash { at: None }));
    assert!(matches!(&reqs[2], Request::Hash { at: Some(sm_op_api::Rev::Empty) }));
    assert!(matches!(&reqs[3], Request::Hash { at: Some(sm_op_api::Rev::Node(_)) }));
    assert!(matches!(&reqs[4], Request::Export { path: Some(p), format: Some(ExportFormat::Vtu), mesh: Some(_), provenance: true, inline: true } if p == "o.vtu"));
    assert!(matches!(&reqs[5], Request::Status));
    assert!(matches!(&reqs[6], Request::Branch { name, at: Some(_) } if name == "alt"));
    assert!(matches!(&reqs[7], Request::Checkout { rev: sm_op_api::Rev::Ref(b) } if b == "main"));
    assert!(matches!(&reqs[8], Request::Checkout { rev: sm_op_api::Rev::Ref(b) } if b == "x"));
    assert!(matches!(&reqs[9], Request::Undo));
    assert!(matches!(&reqs[10], Request::Redo));
    assert!(matches!(&reqs[11], Request::Log { from: None, limit: 20 }));
    assert!(matches!(&reqs[12], Request::Log { from: Some(_), limit: 3 }));
    assert_eq!(reqs.len(), 13, "fixed tools never fetch the catalog");
}

#[test]
fn fixed_tool_arguments_are_validated_before_the_endpoint_is_called() {
    let mut m = Mock::new();
    let r = run(
        &mut m,
        &[
            call(1, "run_script", json!({})),                        // text missing
            call(2, "run_script", json!({"text": 5})),
            call(3, "session_status", json!({"extra": 1})),          // unit method, args
            call(4, "session_export", json!({"typo": true})),
            call(5, "timeline_branch", json!({})),
            call(6, "timeline_checkout", json!({})),
            call(7, "timeline_log", json!({"limit": -1})),
            call(8, "session_hash", json!({"at": 5})),
            call(9, "help", json!({"bogus": 1})),
        ],
    );
    for (i, rep) in r.iter().enumerate() {
        assert_eq!(rep["error"]["code"], -32602, "case {}: {rep}", i + 1);
    }
    assert!(m.log.iter().all(|q| matches!(q, Request::Catalog)), "{:?}", m.log);
}

#[test]
fn fixed_tool_schemas_match_the_protocol_request_fields() {
    let ts = tools(&mut Mock::new(), "2025-06-18");
    let props = |n: &str| -> Vec<String> {
        let t = ts.iter().find(|t| t["name"] == n).unwrap();
        t["inputSchema"]["properties"].as_object().unwrap().keys().cloned().collect()
    };
    assert_eq!(props("session_export"), ["format", "inline", "mesh", "path", "provenance"]);
    assert_eq!(props("run_script"), ["includes", "path", "text"]);
    assert_eq!(props("timeline_log"), ["from", "limit"]);
    assert_eq!(props("timeline_branch"), ["at", "name"]);
    assert!(props("session_status").is_empty());
    let t = ts.iter().find(|t| t["name"] == "run_script").unwrap();
    assert_eq!(t["inputSchema"]["required"], json!(["text"]));
}

#[test]
fn help_lists_and_describes() {
    let mut m = Mock::new();
    let r = run(
        &mut m,
        &[
            call(1, "help", json!({})),
            call(2, "help", json!({"topic": "mesh.run"})),
            call(3, "help", json!({"topic": "mesh__run"})),
            call(4, "help", json!({"topic": "nothing"})),
            call(5, "help", json!({"topic": "help"})),
        ],
    );
    let list = r[0]["result"]["structuredContent"]["tools"].as_array().unwrap();
    assert_eq!(list.len(), 18);
    assert_eq!(r[1]["result"]["structuredContent"]["name"], "mesh__run");
    assert_eq!(r[1]["result"]["structuredContent"], r[2]["result"]["structuredContent"]);
    assert!(r[1]["result"]["structuredContent"]["inputSchema"].is_object());
    assert_eq!(r[3]["result"]["isError"], true);
    assert_eq!(r[4]["result"]["structuredContent"]["name"], "help");
    // help is answered locally.
    assert!(m.log.iter().all(|q| matches!(q, Request::Catalog)));
}

#[test]
fn string_and_integer_ids_are_echoed() {
    let r = run(
        &mut Mock::new(),
        &[
            json!({"jsonrpc":"2.0","id":"abc-1","method":"ping"}),
            json!({"jsonrpc":"2.0","id":42,"method":"ping"}),
            json!({"jsonrpc":"2.0","id":-3,"method":"ping"}),
        ],
    );
    assert_eq!(r[0]["id"], "abc-1");
    assert_eq!(r[1]["id"], 42);
    assert_eq!(r[2]["id"], -3);
}

#[cfg(feature = "respell")]
#[test]
fn respell_feature_changes_text_only() {
    let mut m = Mock::new();
    let r = run(&mut m, &[call(1, "topo__related", json!({"within": "TopoVolume#1"}))]);
    let res = &r[0]["result"];
    assert_eq!(res["structuredContent"], json!({"result": ["TopoFace#1", "TopoFace#2"]}));
    let text = res["content"][0]["text"].as_str().unwrap();
    assert!(text.contains("Face#1") && !text.contains("TopoFace"), "{text}");
}

#[cfg(not(feature = "respell"))]
#[test]
fn text_keeps_wire_ids_by_default() {
    let mut m = Mock::new();
    let r = run(&mut m, &[call(1, "topo__related", json!({"within": "TopoVolume#1"}))]);
    let text = r[0]["result"]["content"][0]["text"].as_str().unwrap();
    assert!(text.contains("TopoFace#1"), "{text}");
}

/// Every `Kind#serial` token in `s`.
#[cfg(feature = "respell")]
fn id_tokens(s: &str) -> Vec<String> {
    let b = s.as_bytes();
    let mut out = Vec::new();
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'#' {
            let mut a = i;
            while a > 0 && b[a - 1].is_ascii_alphabetic() {
                a -= 1;
            }
            let mut e = i + 1;
            while e < b.len() && b[e].is_ascii_digit() {
                e += 1;
            }
            if a < i && e > i + 1 {
                out.push(s[a..e].to_owned());
            }
            i = e;
        } else {
            i += 1;
        }
    }
    out
}

#[cfg(feature = "respell")]
#[test]
fn respelled_ids_feed_back_as_arguments() {
    let mut m = Mock::new();
    let r = run(&mut m, &[call(1, "model__create_brick", json!({"x": 10})), call(2, "topo__related", json!({"within": "TopoVolume#1"}))]);
    let mut seen = 0;
    for resp in &r[..2] {
        let text = resp["result"]["content"][0]["text"].as_str().unwrap();
        for id in id_tokens(text) {
            seen += 1;
            assert!(!id.starts_with("Topo"), "text should be respelled: {id}");
            let r2 = run(&mut m, &[call(9, "topo__related", json!({"within": id}))]);
            assert!(r2[0].get("error").is_none() && r2[0]["result"]["isError"] != json!(true), "{id}: {}", r2[0]);
            let r3 = run(&mut m, &[call(10, "mesh__run", json!({"targets": [id]}))]);
            assert!(r3[0].get("error").is_none(), "{id}: {}", r3[0]);
        }
    }
    assert!(seen > 0, "no ids found in tool text");
}
