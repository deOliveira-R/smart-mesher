//! W3b-SERVER-HOST acceptance 2: `script.record` on a milestone-shaped timeline (model, `mesh.auto`,
//! `mesh.run`, a fork) replays to equal per-ref tips, through the protocol, the CLI and MCP.

use std::cell::RefCell;
use std::io::Cursor;
use std::rc::Rc;

use serde_json::{Value, json};
use sm_op_api::Rev;
use sm_protocol::{Endpoint, RecordCheck, RecordForks, Request, RpcError, ScriptRecordResult};
use sm_server::{Server, standard_with};

const TIME: &str = "2026-10-24T00:00:00Z";

fn builder() -> sm_engine::WorkspaceBuilder {
    standard_with(Some("tester".into()), Some(TIME.into()))
}

/// Two merged bricks, automatic schemes recorded as explicit assignments, a mesh, then a fork that
/// meshes at another size.
const MILESTONE: &str = r#"
let a = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [1.0, 1.0, 1.0])
let b = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [3.0, 1.0, 1.0])
model.imprint(bodies: a.body | b.body)
model.merge(entities: a.body | b.body)
fem.block_add(block: 1, volumes: a.volume, name: "fuel")
fem.block_add(block: 2, volumes: b.volume, name: "water")
mesh.size(targets: a.volume | b.volume, size: 1.0)
mesh.auto(strategy: "default")
let m = mesh.run(targets: a.volume | b.volume)
timeline.branch("fine")
mesh.size(targets: a.volume | b.volume, size: 0.5)
"#;

fn milestone() -> Server {
    let mut s = Server::new(builder()).expect("server");
    s.call(Request::ScriptRun { text: MILESTONE.into(), path: None, includes: Default::default() })
        .unwrap_or_else(|e| panic!("fixture: {e} {}", e.data));
    s
}

fn record(s: &mut Server, forks: RecordForks, check: RecordCheck, verify: bool) -> ScriptRecordResult {
    let v = s.call(Request::ScriptRecord { forks, check, verify, raw: false }).unwrap_or_else(|e| panic!("record: {e} {}", e.data));
    serde_json::from_value(v).expect("ScriptRecordResult")
}

fn tip_state(s: &mut Server, name: &str) -> Value {
    s.call(Request::Hash { at: Some(Rev::Ref(name.into())) }).expect("hash at ref")["state"].clone()
}

fn replay(text: &str) -> Server {
    let mut t = Server::new(builder()).expect("fresh server");
    t.call(Request::ScriptRun { text: text.into(), path: None, includes: Default::default() })
        .unwrap_or_else(|e| panic!("replay: {e} {}", e.data));
    t
}

#[test]
fn recorded_script_replays_to_equal_per_ref_tips() {
    let mut s = milestone();
    let rec = record(&mut s, RecordForks::Refs, RecordCheck::Tips, true);
    assert_eq!(rec.verified, Some(true), "{:?}", rec.notes);
    assert!(rec.refs.contains_key("main") && rec.refs.contains_key("fine"), "{:?}", rec.refs);
    assert!(rec.text.contains("mesh.auto(assignments: ["), "{}", rec.text);
    assert!(rec.nodes >= 10, "{}", rec.nodes);

    let mut t = replay(&rec.text);
    for r in rec.refs.keys() {
        assert_eq!(tip_state(&mut s, r), tip_state(&mut t, r), "tip of `{r}`");
    }
    // Recording the replay gives the same text: the record is a fixed point.
    let again = record(&mut t, RecordForks::Refs, RecordCheck::Tips, false);
    assert_eq!(again.text, rec.text);
}

#[test]
fn head_only_recording_and_check_modes() {
    let mut s = milestone();
    let head = record(&mut s, RecordForks::Head, RecordCheck::None, true);
    assert_eq!(head.verified, Some(true));
    assert!(!head.text.contains("assert session.hash()"), "{}", head.text);
    let every = record(&mut s, RecordForks::Head, RecordCheck::Every, false);
    let tips = record(&mut s, RecordForks::Head, RecordCheck::Tips, false);
    let asserts = |t: &str| t.matches("assert session.hash()").count();
    assert_eq!(asserts(&tips.text), 1);
    assert!(asserts(&every.text) > asserts(&tips.text));
    let mut t = replay(&every.text);
    assert_eq!(t.call(Request::Hash { at: None }).expect("hash")["state"], s.call(Request::Hash { at: None }).expect("hash")["state"]);
}

#[test]
fn recording_is_deterministic_across_runs_and_thread_counts() {
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    let a = one.install(|| record(&mut milestone(), RecordForks::Refs, RecordCheck::Every, true));
    let b = record(&mut milestone(), RecordForks::Refs, RecordCheck::Every, true);
    let c = record(&mut milestone(), RecordForks::Refs, RecordCheck::Every, true);
    assert_eq!(a.text, b.text);
    assert_eq!(b.text, c.text);
    assert_eq!(a.refs, b.refs);
}

/// A server shared between the in-process calls and the CLI.
struct Shared(Rc<RefCell<Server>>);

impl Endpoint for Shared {
    fn call(&mut self, req: Request) -> Result<Value, RpcError> {
        self.0.borrow_mut().call(req)
    }
}

fn cli(server: &Rc<RefCell<Server>>, args: &[&str]) -> (i32, String, String) {
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let mut input = Cursor::new(Vec::new());
    let mut io = sm_cli::Io { stdin: &mut input, stdout: &mut out, stderr: &mut err };
    let mut connect = |_: &sm_cli::Settings| -> Result<Box<dyn Endpoint>, String> { Ok(Box::new(Shared(Rc::clone(server)))) };
    let argv: Vec<&str> = std::iter::once("smesh").chain(args.iter().copied()).collect();
    let code = sm_cli::run(argv, &mut connect, &mut io);
    (code, String::from_utf8_lossy(&out).into_owned(), String::from_utf8_lossy(&err).into_owned())
}

fn mcp(server: &mut Server, tool: &str, args: Value) -> Value {
    let msgs = [
        json!({"jsonrpc":"2.0","id":0,"method":"initialize","params":{"protocolVersion":"2025-06-18"}}),
        json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":tool,"arguments":args}}),
    ];
    let input: String = msgs.iter().map(|m| format!("{m}\n")).collect();
    let mut out = Vec::new();
    sm_mcp::run_stdio(server, Cursor::new(input.into_bytes()), &mut out).expect("run");
    let replies: Vec<Value> = String::from_utf8(out).expect("utf8").lines().map(|l| serde_json::from_str(l).expect("json")).collect();
    replies[1].clone()
}

#[test]
fn cli_and_mcp_record_the_same_text_and_it_replays() {
    let mut direct = milestone();
    let want = record(&mut direct, RecordForks::Refs, RecordCheck::Tips, true);
    let shared = Rc::new(RefCell::new(milestone()));

    // CLI: the script on stdout, equal to the protocol's.
    let (code, out, err) = cli(&shared, &["--memory", "record", "--forks", "refs", "--verify"]);
    assert_eq!(code, 0, "{err}");
    assert_eq!(out, want.text);
    let cli_text = out;
    assert!(err.contains("replay reproduces the hashes"), "{err}");
    // `--json` prints the result object.
    let (code, out, _) = cli(&shared, &["--json", "--memory", "record", "--forks", "refs"]);
    assert_eq!(code, 0);
    let v: Value = serde_json::from_str(out.trim()).expect("json");
    assert_eq!(v["text"], json!(want.text));
    assert_eq!(v["refs"], serde_json::to_value(&want.refs).expect("refs"));

    // MCP: the structured result is the protocol's.
    let r = mcp(&mut shared.borrow_mut(), "record_script", json!({"forks": "refs", "verify": true}));
    let r = &r["result"];
    assert_ne!(r["isError"], json!(true), "{r}");
    assert_eq!(r["structuredContent"]["text"], json!(want.text), "{r}");
    assert_eq!(r["structuredContent"]["verified"], json!(true));

    // The CLI text replays to equal per-ref tips.
    let mut t = replay(&cli_text);
    for r in want.refs.keys() {
        assert_eq!(tip_state(&mut direct, r), tip_state(&mut t, r), "tip of `{r}`");
    }
    // Bad flag values are usage errors.
    assert_eq!(cli(&shared, &["--memory", "record", "--forks", "all"]).0, 2);
}
