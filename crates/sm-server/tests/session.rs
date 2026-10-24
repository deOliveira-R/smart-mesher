//! Server behaviour with the demo plugins: scripts, apply, timeline, hashing, errors, export.
mod common;

use common::*;
use serde_json::{Value, json};
use sm_engine::Workspace;
use sm_op_api::{Hash256, Rev};
use sm_protocol::{ApplyResult, Endpoint, ExportFormat, HashResult, Request, RpcError, ScriptResult, StatusResult};
use sm_server::{InProcess, Server};

fn server() -> Server {
    Server::new(demo_builder()).expect("server")
}

fn call(s: &mut Server, r: Request) -> Value {
    s.call(r).unwrap_or_else(|e| panic!("request failed: {e}"))
}

fn apply(s: &mut Server, op: &str, params: Value) -> ApplyResult {
    let v = call(s, Request::Apply { op: op.into(), params, version: None });
    serde_json::from_value(v).expect("ApplyResult")
}

fn err(s: &mut Server, r: Request) -> RpcError {
    s.call(r).expect_err("must fail")
}

fn log_nodes(s: &mut Server) -> Vec<Hash256> {
    let v = call(s, Request::Log { from: None, limit: 100 });
    let mut nodes: Vec<Hash256> = v.as_array().expect("array").iter().map(|e| serde_json::from_value(e["node"].clone()).expect("node")).collect();
    nodes.reverse();
    nodes
}

fn hash(s: &mut Server, at: Option<Rev>) -> HashResult {
    serde_json::from_value(call(s, Request::Hash { at })).expect("HashResult")
}

const SCRIPT: &str = "let a = demo.add_item(\"alpha\")\nlet b = demo.add_item(name: \"beta\")\ndemo.rename(item: a.item, name: \"gamma\")\nprint \"done\"\n";

#[test]
fn script_run_equals_the_equivalent_apply_calls() {
    let mut s1 = server();
    let r: ScriptResult = serde_json::from_value(call(&mut s1, Request::ScriptRun { text: SCRIPT.into(), path: None, includes: Default::default() })).expect("ScriptResult");
    assert_eq!(r.output, "done\n");
    assert_eq!(r.events.iter().filter(|e| e["event"] == "op").count(), 3);

    let mut s2 = server();
    apply(&mut s2, "demo.add_item", json!({"name": "alpha"}));
    apply(&mut s2, "demo.add_item", json!({"name": "beta"}));
    apply(&mut s2, "demo.rename", json!({"item": "TopoBody#1", "name": "gamma"}));

    assert_eq!(r.state, hash(&mut s2, None).state, "same state hash");
    assert_eq!(log_nodes(&mut s1), log_nodes(&mut s2), "same node sequence");
    assert_eq!(hash(&mut s1, None), hash(&mut s2, None), "same layer hashes");
}

#[test]
fn script_events_carry_op_params_and_delta() {
    let mut s = server();
    let r: ScriptResult = serde_json::from_value(call(&mut s, Request::ScriptRun { text: SCRIPT.into(), path: Some("t.smesh".into()), includes: Default::default() })).expect("ScriptResult");
    let e = &r.events[0];
    assert_eq!(e["op"], "demo.add_item");
    assert_eq!(e["params"], json!({"name": "alpha"}));
    assert_eq!(e["delta"]["outputs"]["item"], json!(["TopoBody#1"]));
    assert!(e["at"].as_str().is_some_and(|a| a.starts_with("t.smesh:1:")), "{e}");
    assert_eq!(r.diagnostics, json!([]));
}

#[test]
fn failing_script_reports_script_error_and_keeps_applied_ops() {
    let mut s = server();
    let e = err(&mut s, Request::ScriptRun { text: "demo.add_item(\"a\")\ndemo.no_such_op(1)\n".into(), path: None, includes: Default::default() });
    assert_eq!(e.kind, "script-error");
    assert_eq!(e.code, sm_protocol::APPLICATION_ERROR);
    assert_eq!(e.data["diagnostics"].as_array().map(Vec::len), Some(1), "{}", e.data);
    assert!(e.data["rendered"].as_str().is_some_and(|r| r.contains("error[")));
    assert_eq!(e.data["events"].as_array().map(Vec::len), Some(1), "the applied op is reported");
    assert_eq!(call(&mut s, Request::Status)["nodes"], 1, "the op before the failure stays applied");
    // The reported state is the real state.
    assert_eq!(e.data["state"], json!(hash(&mut s, None).state));
}

#[test]
fn script_syntax_error_applies_nothing() {
    let mut s = server();
    let e = err(&mut s, Request::ScriptRun { text: "demo.add_item(\"a\"\n".into(), path: None, includes: Default::default() });
    assert_eq!(e.kind, "script-error");
    assert_eq!(call(&mut s, Request::Status)["nodes"], 0);
}

#[test]
fn fork_then_checkout_back_is_bit_identical() {
    let mut s = server();
    let n1 = apply(&mut s, "demo.add_item", json!({"name": "a"}));
    apply(&mut s, "demo.add_item", json!({"name": "b"}));
    let before = hash(&mut s, None);
    // Fork at the earlier node: creates the branch and attaches HEAD.
    let st: StatusResult = serde_json::from_value(call(&mut s, Request::Branch { name: "alt".into(), at: Some(n1.node) })).expect("status");
    assert_eq!(st.position, Some(n1.node));
    assert_eq!(st.branches["alt"], n1.node);
    assert_eq!(serde_json::to_value(&st.head).expect("head"), json!({"ref": "alt"}));
    assert_eq!(hash(&mut s, None).state, n1.state, "state moved to the fork point");
    apply(&mut s, "demo.add_item", json!({"name": "c"}));
    assert_ne!(hash(&mut s, None), before);
    call(&mut s, Request::Checkout { rev: Rev::Ref("main".into()) });
    assert_eq!(hash(&mut s, None), before, "checkout back is bit-identical");
    // Undo / redo round trip.
    call(&mut s, Request::Undo);
    assert_ne!(hash(&mut s, None), before);
    let st: StatusResult = serde_json::from_value(call(&mut s, Request::Redo)).expect("status");
    assert_eq!(st.state, before.state);
    // Detached checkout of a node, and of the empty state.
    call(&mut s, Request::Checkout { rev: Rev::Node(n1.node) });
    assert_eq!(hash(&mut s, None).state, n1.state);
    call(&mut s, Request::Checkout { rev: Rev::Empty });
    assert_eq!(call(&mut s, Request::Status)["position"], Value::Null);
}

#[test]
fn script_timeline_commands_map_to_the_workspace() {
    let mut s = server();
    let script = "demo.add_item(\"a\")\nlet h0 = timeline.head().node\ndemo.add_item(\"b\")\nlet main_hash = session.hash()\ntimeline.branch(\"alt\", at: h0)\nassert timeline.head().branch == \"alt\"\ndemo.add_item(\"b\")\nassert session.hash() == main_hash\ntimeline.checkout(\"main\")\nassert session.hash() == main_hash\n";
    let r = s.call(Request::ScriptRun { text: script.into(), path: None, includes: Default::default() });
    // `session.*` statements are parsed by sm-lang (T9); until it lands the statement is a
    // diagnostic, not a crash. Timeline commands work either way.
    match r {
        Ok(v) => {
            let st = call(&mut s, Request::Status);
            assert_eq!(st["head"], json!({"ref": "main"}), "{v}");
        }
        Err(e) => assert_eq!(e.kind, "script-error", "{e}"),
    }
    let mut s = server();
    let script = "demo.add_item(\"a\")\nlet h0 = timeline.head().node\ndemo.add_item(\"b\")\ntimeline.branch(\"alt\", at: h0)\nassert timeline.head().branch == \"alt\"\ndemo.add_item(\"c\")\ntimeline.checkout(\"main\")\nassert timeline.head().branch == \"main\"\ntimeline.undo()\ntimeline.redo()\n";
    let v = call(&mut s, Request::ScriptRun { text: script.into(), path: None, includes: Default::default() });
    assert!(v["events"].as_array().is_some_and(|e| e.iter().filter(|e| e["event"] == "timeline").count() == 4), "{v}");
    let st = call(&mut s, Request::Status);
    assert_eq!(st["head"], json!({"ref": "main"}));
    assert_eq!(st["branches"].as_object().map(|b| b.len()), Some(2));
}

#[test]
fn session_hash_at_rev_equals_state_at() {
    let mut ws: Workspace = demo_builder().memory().expect("ws");
    let o = sm_engine::ApplyOptions::default();
    let a = ws.apply_json("demo.add_item", json!({"name": "a"}), &o).expect("a");
    ws.apply_json("demo.add_item", json!({"name": "b"}), &o).expect("b");
    let mut srv = Server::from_workspace(ws);
    let revs = [Rev::Node(a.node), Rev::Ref("main".into()), Rev::Empty];
    let got: Vec<HashResult> = revs.iter().map(|r| hash(&mut srv, Some(r.clone()))).collect();
    let mut ws = srv.into_workspace();
    for (rev, got) in revs.iter().zip(&got) {
        let st = ws.state_at(rev).expect("state_at");
        assert_eq!(got.state, st.hash(), "session.hash at {rev:?}");
        assert_eq!(got.layers, st.layer_hashes(), "layer hashes at {rev:?}");
    }
    assert_eq!(got[0].state, a.state);
    assert_eq!(got[1].state, ws.state_hash(), "a ref at HEAD is the current state");
}

#[test]
fn provenance_embeds_the_current_node() {
    let mut s = server();
    let m = apply(&mut s, "demo.add_hex", json!({"n": 1}));
    let plain = call(&mut s, Request::Export { path: None, format: Some(ExportFormat::Smsh), mesh: None, provenance: false, inline: true });
    let prov = call(&mut s, Request::Export { path: None, format: Some(ExportFormat::Smsh), mesh: None, provenance: true, inline: true });
    assert_ne!(plain["sha256"], prov["sha256"]);
    // The node hash is present in the container, and absent without provenance.
    let find = |v: &Value| -> bool {
        let b = v["base64"].as_str().expect("b64").to_owned();
        let bytes = decode_b64(&b);
        let hex = m.node.to_hex();
        bytes.windows(hex.len()).any(|w| w == hex.as_bytes())
    };
    assert!(find(&prov));
    assert!(!find(&plain));
}

fn decode_b64(s: &str) -> Vec<u8> {
    let val = |c: u8| -> u32 {
        match c {
            b'A'..=b'Z' => u32::from(c - b'A'),
            b'a'..=b'z' => u32::from(c - b'a') + 26,
            b'0'..=b'9' => u32::from(c - b'0') + 52,
            b'+' => 62,
            _ => 63,
        }
    };
    let mut out = Vec::new();
    for chunk in s.as_bytes().chunks(4) {
        let pad = chunk.iter().filter(|&&c| c == b'=').count();
        let n = chunk.iter().take(4 - pad).fold(0u32, |a, &c| (a << 6) | val(c)) << (6 * pad as u32);
        let bytes = [(n >> 16) as u8, (n >> 8) as u8, n as u8];
        out.extend_from_slice(&bytes[..3 - pad]);
    }
    out
}

#[test]
fn errors_carry_contract_code_strings() {
    let mut s = server();
    assert_eq!(err(&mut s, Request::Apply { op: "demo.nope".into(), params: json!({}), version: None }).kind, "unknown-op");
    let e = err(&mut s, Request::Apply { op: "demo.add_item".into(), params: json!({"nme": "x"}), version: None });
    assert_eq!(e.code, sm_protocol::APPLICATION_ERROR);
    assert_eq!(e.kind, "decode");
    assert_eq!(err(&mut s, Request::Apply { op: "demo.rename".into(), params: json!({"item": "TopoBody#9", "name": "x"}), version: None }).kind, "unknown-entity");
    assert_eq!(err(&mut s, Request::Query { name: "demo.nope".into(), input: json!({}), at: None }).kind, "unknown-query");
    assert_eq!(err(&mut s, Request::Checkout { rev: Rev::Ref("nope".into()) }).kind, "unknown-ref");
    assert_eq!(err(&mut s, Request::Branch { name: "x".into(), at: None }).kind, "empty-position");
    assert_eq!(err(&mut s, Request::Save).kind, "no-project");
    assert_eq!(err(&mut s, Request::Export { path: Some("x.smsh".into()), format: None, mesh: None, provenance: false, inline: false }).kind, "no-mesh");
    // Protocol-level errors keep their JSON-RPC kinds.
    let mut ip = InProcess::new(server());
    let r: Value = serde_json::from_str(&ip.send_text(r#"{"jsonrpc":"2.0","id":3,"method":"nope"}"#)).expect("json");
    assert_eq!(r["error"]["kind"], "method-not-found");
    assert_eq!(r["id"], 3);
}

#[test]
fn query_at_a_revision_and_current() {
    let mut s = server();
    let n1 = apply(&mut s, "demo.add_item", json!({"name": "a"}));
    apply(&mut s, "demo.add_item", json!({"name": "b"}));
    assert_eq!(call(&mut s, Request::Query { name: "demo.count".into(), input: json!({}), at: None }), json!(2));
    assert_eq!(call(&mut s, Request::Query { name: "demo.count".into(), input: Value::Null, at: Some(Rev::Node(n1.node)) }), json!(1));
}

#[test]
fn catalog_lists_ops_with_outputs_and_tool_names() {
    let mut s = server();
    let cat = call(&mut s, Request::Catalog);
    let ops = cat["ops"].as_array().expect("ops");
    let add = ops.iter().find(|o| o["name"] == "demo.add_item").expect("demo.add_item");
    assert_eq!(add["tool_name"], "demo__add_item");
    assert_eq!(add["outputs"][0]["name"], "item");
    assert_eq!(add["positional"], "name");
    assert!(ops.iter().any(|o| o["name"] == "core.batch"));
    assert!(cat["queries"].as_array().is_some_and(|q| q.iter().any(|q| q["name"] == "topo.related")));
}

#[test]
fn batch_through_apply_and_script_agree() {
    let mut s1 = server();
    apply(&mut s1, "core.batch", json!({"ops": [{"name": "demo.add_item", "version": 1, "params": {"name": "a"}}, {"name": "demo.rename", "version": 1, "params": {"item": "$0.item[0]", "name": "z"}}]}));
    let mut s2 = server();
    call(&mut s2, Request::ScriptRun { text: "batch {\n let a = demo.add_item(\"a\")\n demo.rename(item: a.item[0], name: \"z\")\n}\n".into(), path: None, includes: Default::default() });
    assert_eq!(hash(&mut s1, None).state, hash(&mut s2, None).state);
}

#[test]
fn save_persists_and_reopen_replays_to_the_same_hash() {
    let dir = TempDir::new("save");
    let proj = dir.path().join("p");
    let mut s = Server::create(demo_builder(), &proj, "p").expect("create");
    apply(&mut s, "demo.add_item", json!({"name": "a"}));
    apply(&mut s, "demo.add_item", json!({"name": "b"}));
    let st = call(&mut s, Request::Save);
    assert_eq!(st["dirty"], false);
    assert_eq!(st["project"], "p");
    let h = hash(&mut s, None);
    drop(s);
    let mut s = Server::open(demo_builder(), &proj).expect("reopen (strict replay)");
    assert_eq!(hash(&mut s, None), h);
}

#[test]
fn session_open_switches_projects_and_refuses_to_discard_a_memory_session() {
    let dir = TempDir::new("open");
    let proj = dir.path().join("proj");
    let mut s = Server::from_factory(demo_builder).expect("server");
    // Empty in-memory session: opening (creating) is fine.
    let path = proj.to_string_lossy().into_owned();
    let e = err(&mut s, Request::Open { path: path.clone(), create: false });
    assert_eq!(e.kind, "not-a-project");
    let st = call(&mut s, Request::Open { path: path.clone(), create: true });
    assert_eq!(st["project"], "proj");
    apply(&mut s, "demo.add_item", json!({"name": "a"}));
    // Opening the same project again is a no-op.
    let st = call(&mut s, Request::Open { path: path.clone(), create: false });
    assert_eq!(st["nodes"], 1);
    // Another project: the first one is saved, then replaced.
    let other = dir.path().join("other").to_string_lossy().into_owned();
    call(&mut s, Request::Open { path: other, create: true });
    assert_eq!(call(&mut s, Request::Status)["nodes"], 0);
    drop(s);
    let mut back = Server::open(demo_builder(), &proj).expect("saved on switch");
    assert_eq!(call(&mut back, Request::Status)["nodes"], 1);

    // A memory session with nodes is not discarded.
    let mut m = Server::from_factory(demo_builder).expect("server");
    apply(&mut m, "demo.add_item", json!({"name": "a"}));
    let e = err(&mut m, Request::Open { path: dir.path().join("third").to_string_lossy().into_owned(), create: true });
    assert_eq!(e.kind, "workspace-not-empty");
    // And a server without a factory cannot open at all.
    let mut n = server();
    assert_eq!(err(&mut n, Request::Open { path: "x".into(), create: true }).kind, "open-unsupported");
}

#[test]
fn exports_smsh_and_vtu_with_the_path_rule() {
    let dir = TempDir::new("export");
    let proj = dir.path().join("p");
    let mut s = Server::create(demo_builder(), &proj, "p").expect("create");
    apply(&mut s, "demo.add_hex", json!({"n": 2}));
    // Relative path under a project resolves against the project root.
    let v = call(&mut s, Request::Export { path: Some("out.smsh".into()), format: None, mesh: None, provenance: false, inline: false });
    let file = proj.join("out.smsh");
    let bytes = std::fs::read(&file).expect("written below the project root");
    assert_eq!(v["path"], json!(file.to_string_lossy()));
    assert_eq!(v["bytes"], bytes.len());
    assert_eq!(v["sha256"], json!(Hash256::of(&bytes)));
    assert_eq!(v["format"], "smsh");
    let decoded = sm_meshio::SmshFile::parse(&bytes).expect("parse").decode().expect("decode");
    assert_eq!(decoded.cell_ids.len(), 8);
    assert!(!proj.join("out.smsh.part").exists(), "temp file is gone");
    // Absolute path, explicit format overriding the extension, and the VTU is XML.
    let abs = dir.path().join("abs.dat");
    call(&mut s, Request::Export { path: Some(abs.to_string_lossy().into_owned()), format: Some(ExportFormat::Vtu), mesh: None, provenance: false, inline: false });
    assert!(String::from_utf8_lossy(&std::fs::read(&abs).expect("abs")).contains("VTKFile"));
    // Repeated exports are byte-identical.
    call(&mut s, Request::Export { path: Some("again.smsh".into()), format: None, mesh: None, provenance: false, inline: false });
    assert_eq!(std::fs::read(proj.join("again.smsh")).expect("again"), bytes);
    // Unknown extension without a format is a bad request; nothing is written.
    let e = err(&mut s, Request::Export { path: Some("x.bin".into()), format: None, mesh: None, provenance: false, inline: false });
    assert_eq!(e.kind, "bad-request");
    assert!(!proj.join("x.bin").exists());
    // Neither path nor inline.
    assert_eq!(err(&mut s, Request::Export { path: None, format: Some(ExportFormat::Smsh), mesh: None, provenance: false, inline: false }).kind, "bad-request");
}

#[test]
fn memory_export_resolves_against_the_working_directory() {
    let s = demo_builder().memory().expect("ws");
    let mut ws = s;
    let host = sm_server::SessionHost::new(&mut ws);
    let cwd = std::env::current_dir().expect("cwd");
    assert_eq!(host.resolve_path("m/out.smsh").expect("rel"), cwd.join("m/out.smsh"));
    let abs = cwd.join("abs.smsh");
    assert_eq!(host.resolve_path(abs.to_str().expect("utf8")).expect("abs"), abs);
    assert_eq!(host.resolve_path("").expect_err("empty").code, "bad-request");
}

#[test]
fn inline_export_returns_base64_of_the_bytes() {
    let mut s = server();
    apply(&mut s, "demo.add_hex", json!({"n": 1}));
    let v = call(&mut s, Request::Export { path: None, format: Some(ExportFormat::Smsh), mesh: None, provenance: false, inline: true });
    let b64 = v["base64"].as_str().expect("base64");
    assert_eq!(v["path"], Value::Null);
    // Length of padded base64 for `bytes` bytes.
    let n = v["bytes"].as_u64().expect("bytes") as usize;
    assert_eq!(b64.len(), n.div_ceil(3) * 4);
    // Inline without a format cannot be inferred.
    assert_eq!(err(&mut s, Request::Export { path: None, format: None, mesh: None, provenance: false, inline: true }).kind, "bad-request");
}

#[test]
fn export_picks_the_only_mesh_and_rejects_ambiguity() {
    let mut s = server();
    let m1 = apply(&mut s, "demo.add_hex", json!({"n": 1}));
    let id: String = m1.delta.outputs["mesh"][0].to_string();
    let one = call(&mut s, Request::Export { path: None, format: Some(ExportFormat::Vtu), mesh: None, provenance: false, inline: true });
    apply(&mut s, "demo.add_hex", json!({"n": 2}));
    let e = err(&mut s, Request::Export { path: None, format: Some(ExportFormat::Vtu), mesh: None, provenance: false, inline: true });
    assert_eq!(e.kind, "ambiguous-mesh");
    assert_eq!(e.data["meshes"].as_array().map(Vec::len), Some(2));
    let named = call(&mut s, Request::Export { path: None, format: Some(ExportFormat::Vtu), mesh: Some(id.parse().expect("id")), provenance: false, inline: true });
    assert_eq!(named["sha256"], one["sha256"], "naming the first mesh gives the same bytes as the only-mesh default");
    assert_eq!(err(&mut s, Request::Export { path: None, format: Some(ExportFormat::Vtu), mesh: Some("Mesh#99".parse().expect("id")), provenance: false, inline: true }).kind, "unknown-mesh");
}

#[test]
fn session_commands_on_the_host() {
    use sm_lang::{Host, SessionCommand};
    let dir = TempDir::new("sess");
    let proj = dir.path().join("p");
    let mut ws = demo_builder().create(&proj, sm_engine::ProjectConfig::new("p")).expect("create");
    let mut host = sm_server::SessionHost::new(&mut ws);
    host.apply(&sm_op_api::OpRecord::new("demo.add_hex", 1, json!({"n": 1})).expect("rec")).expect("apply");
    let h = host.session(&SessionCommand::Hash).expect("hash");
    assert_eq!(h, json!(host.workspace().state_hash().to_hex()));
    let st = host.session(&SessionCommand::Status).expect("status");
    assert_eq!(st["nodes"], 1);
    assert!(st.get("position").is_some_and(|p| p.is_string()), "{st}");
    let r = host
        .session(&SessionCommand::Export { path: "m.vtu".into(), format: None, mesh: None, provenance: false })
        .expect("export");
    assert_eq!(r["format"], "vtu");
    assert!(proj.join("m.vtu").exists());
    let e = host.session(&SessionCommand::Export { path: "m.x".into(), format: Some("stl".into()), mesh: None, provenance: false }).expect_err("format");
    assert_eq!(e.code, "bad-request");
    let st = host.session(&SessionCommand::Save).expect("save");
    assert_eq!(st["dirty"], false);
    // The head is attached to the default branch.
    let head = host.timeline_head().expect("head");
    assert_eq!(head.branch.as_deref(), Some("main"));
    assert_eq!(head.node.as_deref().map(str::len), Some(64));
}

#[test]
fn save_without_a_project_is_a_typed_host_error() {
    use sm_lang::{Host, SessionCommand};
    let mut ws = demo_builder().memory().expect("ws");
    let mut host = sm_server::SessionHost::new(&mut ws);
    assert_eq!(host.session(&SessionCommand::Save).expect_err("no project").code, "no-project");
}

#[test]
fn includes_come_from_the_request_then_the_project_and_never_escape_it() {
    use sm_lang::Host;
    let dir = TempDir::new("inc");
    let proj = dir.path().join("p");
    let mut ws = demo_builder().create(&proj, sm_engine::ProjectConfig::new("p")).expect("create");
    std::fs::create_dir_all(proj.join("lib")).expect("mkdir");
    std::fs::write(proj.join("lib/a.smesh"), "demo.add_item(\"from-file\")\n").expect("write");
    std::fs::write(dir.path().join("secret.smesh"), "secret\n").expect("write");
    let mut inc = std::collections::BTreeMap::new();
    inc.insert("virtual.smesh".to_owned(), "demo.add_item(\"from-map\")\n".to_owned());
    let mut host = sm_server::SessionHost::new(&mut ws).with_includes(inc);
    assert_eq!(host.load("virtual.smesh").expect("map"), "demo.add_item(\"from-map\")\n");
    assert_eq!(host.load("lib/a.smesh").expect("file"), "demo.add_item(\"from-file\")\n");
    assert!(host.load("../secret.smesh").is_err(), "parent traversal");
    assert!(host.load(dir.path().join("secret.smesh").to_str().expect("utf8")).is_err(), "absolute path");
    assert!(host.load("missing.smesh").is_err());
    // End to end through the interpreter.
    let mut srv = Server::from_workspace(ws);
    let r = srv.call(Request::ScriptRun { text: "include \"lib/a.smesh\"\n".into(), path: None, includes: Default::default() });
    assert!(r.is_ok(), "{r:?}");
    assert_eq!(call(&mut srv, Request::Query { name: "demo.count".into(), input: json!({}), at: None }), json!(1));
}

#[test]
fn relations_follow_the_registered_query() {
    let mut s = server();
    let v = call(&mut s, Request::ScriptRun { text: "let a = demo.add_item(\"a\")\nprint count(bodies in a.item)\n".into(), path: None, includes: Default::default() });
    // The point is that `kind in X` has adjacency available (no "host has no relations" error).
    assert!(v["diagnostics"].as_array().is_some_and(|d| d.iter().all(|d| d["severity"] != "error")), "{v}");
}
