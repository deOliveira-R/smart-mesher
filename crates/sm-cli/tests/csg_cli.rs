//! W4-CSG-CLI: `smesh import-openmc`, the front ends of `model.import_openmc`, `export --source`
//! and `export --regions`.
//!
//! The Python leg of the four-front-end hash row is `crates/sm-py/tests/test_import_openmc.py`,
//! which runs the same deck through `smartmesh.Session` and compares with this binary.
#![allow(missing_docs)]

mod common;
mod w3a_support;

use std::path::{Path, PathBuf};

use common::Scratch;
use serde_json::{Value, json};
use sm_protocol::{Endpoint, ExportFormat, ExportSource, Request};
use sm_server::{Server, standard_with};
use w3a_support::*;

fn fixture(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../sm-csgio/tests/fixtures/openmc")
        .join(name)
        .canonicalize()
        .expect("fixture dir")
}

fn s(p: &Path) -> String {
    p.to_string_lossy().into_owned()
}

/// Two boxes side by side along x in a 10-cube: cell 1 is `[0, a]`, cell 2 is `[b, 10]`.
fn two_boxes(a: &str, b: &str) -> String {
    format!(
        "<?xml version=\"1.0\"?><geometry>\
         <cell id=\"1\" material=\"1\" region=\"1 -2 3 -4 5 -6\" universe=\"1\"/>\
         <cell id=\"2\" material=\"2\" region=\"7 -8 3 -4 5 -6\" universe=\"1\"/>\
         <surface id=\"1\" type=\"x-plane\" boundary=\"vacuum\" coeffs=\"0\"/><surface id=\"2\" type=\"x-plane\" coeffs=\"{a}\"/>\
         <surface id=\"7\" type=\"x-plane\" coeffs=\"{b}\"/><surface id=\"8\" type=\"x-plane\" boundary=\"vacuum\" coeffs=\"10\"/>\
         <surface id=\"3\" type=\"y-plane\" boundary=\"vacuum\" coeffs=\"0\"/><surface id=\"4\" type=\"y-plane\" boundary=\"vacuum\" coeffs=\"10\"/>\
         <surface id=\"5\" type=\"z-plane\" boundary=\"vacuum\" coeffs=\"0\"/><surface id=\"6\" type=\"z-plane\" boundary=\"vacuum\" coeffs=\"10\"/></geometry>"
    )
}

fn diag_codes(v: &Value) -> Vec<String> {
    v["delta"]["diagnostics"]
        .as_array()
        .map(|a| a.iter().filter_map(|d| d["code"].as_str().map(str::to_owned)).collect())
        .unwrap_or_default()
}

#[test]
fn overlap_exits_1_by_default_and_0_with_report_and_both_outputs_carry_the_diagnostic() {
    let dir = Scratch::new("csg_cli_overlap");
    let geo = dir.join("geometry.xml");
    std::fs::write(&geo, two_boxes("5.5", "5")).unwrap();
    let g = s(&geo);

    // Default policy: exit 1, the error names both cells, in text and in JSON.
    let o = spawn(&["--memory", "import-openmc", &g], None, Threads::Inherit, None).unwrap();
    assert_eq!(o.status.code(), Some(1), "{}", describe(&["import-openmc"], &o));
    let err = String::from_utf8_lossy(&o.stderr);
    assert!(err.contains("csg.overlap"), "{err}");
    let (code, v) = any_json(&["--memory", "--json", "import-openmc", &g], None, Threads::Inherit).unwrap();
    assert_eq!(code, 1);
    assert!(v["error"]["message"].as_str().unwrap_or("").contains("csg.overlap"), "{v}");

    // `--overlap report`: exit 0, one `import.overlap` warning on stderr (text) and in the delta (JSON).
    let o = spawn(&["--memory", "import-openmc", &g, "--overlap", "report"], None, Threads::Inherit, None).unwrap();
    assert_eq!(o.status.code(), Some(0), "{}", describe(&["import-openmc"], &o));
    let err = String::from_utf8_lossy(&o.stderr);
    assert!(err.contains("warning: import.overlap:"), "{err}");
    let v = ok_json(&["--memory", "--json", "import-openmc", &g, "--overlap", "report"], None, Threads::Inherit).unwrap();
    assert_eq!(diag_codes(&v).iter().filter(|c| *c == "import.overlap").count(), 1, "{:?}", diag_codes(&v));
    // Analytic: the double-owned slab is 0.5 x 10 x 10 = 50 and it is named in the message.
    let msg = v["delta"]["diagnostics"][0]["message"].as_str().unwrap();
    assert!(msg.contains("5e1") || msg.contains("50"), "{msg}");

    // A gap is dropped with `import.gap` and does not fail the import.
    std::fs::write(&geo, two_boxes("4.95", "5.05")).unwrap();
    let v = ok_json(&["--memory", "--json", "import-openmc", &g], None, Threads::Inherit).unwrap();
    assert!(diag_codes(&v).contains(&"import.gap".to_owned()), "{:?}", diag_codes(&v));
}

#[test]
fn import_openmc_is_one_history_node_and_the_options_reach_the_op() {
    let dir = Scratch::new("csg_cli_node");
    let proj = dir.join("proj");
    let pin = fixture("pin");
    let (g, m) = (s(&pin.join("geometry.xml")), s(&pin.join("materials.xml")));
    ok_text(&["new", &s(&proj)], None, Threads::Inherit).unwrap();
    let out = ok_text(
        &["-p", &s(&proj), "import-openmc", &g, "--materials", &m, "--max-cells", "50", "--no-reuse-lattices"],
        None,
        Threads::Inherit,
    )
    .unwrap();
    assert!(out.starts_with("model.import_openmc: node "), "{out}");
    let rec = ok_text(&["-p", &s(&proj), "record", "--raw"], None, Threads::Inherit).unwrap();
    assert_eq!(rec.matches("model.import_openmc").count(), 1, "{rec}");
    assert!(rec.contains("max_cells: 50") && rec.contains("reuse_lattices: false"), "{rec}");
    let log = ok_json(&["-p", &s(&proj), "--json", "log"], None, Threads::Inherit).unwrap();
    assert_eq!(log["nodes"].as_array().map_or(0, Vec::len), 1, "{log}");
    // A missing geometry file is an op failure reported by the server, not a usage error: exit 1.
    let o = spawn(&["--memory", "import-openmc", "/nonexistent/geometry.xml"], None, Threads::Inherit, None).unwrap();
    assert_eq!(o.status.code(), Some(1), "{}", describe(&["import-openmc"], &o));
}

/// The pin deck's state hash from the library, the CLI and MCP (Python: sm-py's pytest).
#[test]
fn the_pin_deck_gives_one_state_hash_in_process_cli_and_mcp() {
    let pin = fixture("pin");
    let (g, m) = (s(&pin.join("geometry.xml")), s(&pin.join("materials.xml")));

    // In process: stage through `script.run`, apply the op with `{sha256}` sources.
    let mut srv = Server::new(standard_with(None, None)).expect("server");
    let mut stage = |p: &str| -> Value {
        let text = format!("session.stage({})\n", serde_json::to_string(p).unwrap());
        let v = srv.call(Request::ScriptRun { text, path: None, includes: Default::default() }).expect("stage");
        let ev = v["events"].as_array().unwrap().iter().find(|e| e["event"] == "session").expect("stage event");
        json!({"sha256": ev["result"]["sha256"]})
    };
    let (gs, ms) = (stage(&g), stage(&m));
    let r = srv
        .call(Request::Apply { op: "model.import_openmc".into(), params: json!({"geometry": gs.clone(), "materials": ms.clone()}), version: None })
        .expect("apply");
    let lib = r["state"].as_str().unwrap().to_owned();
    let h = srv.call(Request::Hash { at: None }).expect("hash");
    assert_eq!(h["state"].as_str(), Some(lib.as_str()));

    // CLI (one process, JSON): the apply result's state, and `hash` of a project run.
    let v = ok_json(&["--memory", "--json", "import-openmc", &g, "--materials", &m], None, Threads::Inherit).unwrap();
    assert_eq!(v["state"].as_str(), Some(lib.as_str()), "CLI vs library");

    // 1 thread equals the default pool.
    let v1 = ok_json(&["--memory", "--json", "import-openmc", &g, "--materials", &m], None, Threads::One).unwrap();
    assert_eq!(v1["state"].as_str(), Some(lib.as_str()), "1 thread");

    // MCP: session_stage twice, the op tool, session_hash.
    // The stores are per process, so stage and apply must share one MCP session.
    let mut lines = mcp_init();
    lines.push(tool_call(2, "session_stage", json!({"path": g})));
    lines.push(tool_call(3, "session_stage", json!({"path": m})));
    lines.push(tool_call(
        4,
        "model__import_openmc",
        json!({"geometry": gs.clone(), "materials": ms.clone()}),
    ));
    lines.push(tool_call(5, "session_hash", json!({})));
    let rep = mcp_session(&[], &lines, Threads::Inherit, None).unwrap();
    let applied = tool_ok(&rep, 4).unwrap();
    assert_eq!(applied["state"].as_str(), Some(lib.as_str()), "MCP apply");
    assert_eq!(tool_ok(&rep, 5).unwrap()["state"].as_str(), Some(lib.as_str()), "MCP hash");
    // The op is in the MCP tool list.
    let mut lines = mcp_init();
    lines.push(json!({"jsonrpc":"2.0","id":9,"method":"tools/list","params":{}}));
    let rep = mcp_session(&[], &lines, Threads::Inherit, None).unwrap();
    let tools = rep[&9]["result"]["tools"].as_array().unwrap();
    assert!(tools.iter().any(|t| t["name"] == "model__import_openmc"));
}

#[test]
fn export_source_model_reaches_the_server() {
    let dir = Scratch::new("csg_cli_source");
    let pin = fixture("pin");
    let (g, m) = (s(&pin.join("geometry.xml")), s(&pin.join("materials.xml")));
    let proj = dir.join("proj");
    ok_text(&["new", &s(&proj)], None, Threads::Inherit).unwrap();
    ok_text(&["-p", &s(&proj), "import-openmc", &g, "--materials", &m], None, Threads::Inherit).unwrap();
    let out = dir.join("pin.h5m");
    let o = spawn(
        &["-p", &s(&proj), "--json", "export", &s(&out), "--source", "model", "--option", "chord_tol=0.05"],
        None,
        Threads::Inherit,
        None,
    )
    .unwrap();
    assert_eq!(o.status.code(), Some(0), "{}", describe(&["export"], &o));
    let v: Value = serde_json::from_slice(&o.stdout).unwrap();
    assert_eq!(v["format"], "dagmc");
    assert!(std::fs::metadata(&out).unwrap().len() > 0);
    // Without `--source model` the same export has no mesh to write and fails.
    let o = spawn(&["-p", &s(&proj), "export", &s(&dir.join("none.h5m"))], None, Threads::Inherit, None).unwrap();
    assert_eq!(o.status.code(), Some(1));
}

/// `regions` reaches the server's option check from the CLI flag and from MCP `options`: both
/// refuse an unknown part with the server's own message.
#[test]
fn regions_pass_through_to_the_server_from_cli_and_mcp() {
    let dir = Scratch::new("csg_cli_regions");
    let pin = fixture("pin");
    let g = s(&pin.join("geometry.xml"));
    let proj = dir.join("proj");
    ok_text(&["new", &s(&proj)], None, Threads::Inherit).unwrap();
    ok_text(&["-p", &s(&proj), "import-openmc", &g], None, Threads::Inherit).unwrap();
    let out = s(&dir.join("a.smsh"));
    let (code, v) = any_json(&["-p", &s(&proj), "--json", "export", &out, "--regions", "bogus"], None, Threads::Inherit).unwrap();
    assert_eq!(code, 1, "{v}");
    assert!(v["error"]["message"].as_str().unwrap_or("").contains("unknown part `bogus`"), "{v}");
    let mut lines = mcp_init();
    lines.push(tool_call(2, "session_export", json!({"path": out, "options": {"regions": {"split": ["bogus"]}}})));
    let rep = mcp_session(&[], &lines, Threads::Inherit, None).unwrap();
    let text = rep[&2]["result"]["content"].to_string();
    assert_eq!(rep[&2]["result"]["isError"], Value::Bool(true), "{text}");
    assert!(text.contains("unknown part"), "{text}");
}

/// The requests of `export --source` and `--regions`, against a recording fake.
#[test]
fn export_regions_and_source_build_the_request() {
    use std::cell::RefCell;
    use std::rc::Rc;
    let run = |args: &[&str]| -> (i32, Vec<Request>) {
        let log = Rc::new(RefCell::new(Vec::new()));
        let l2 = log.clone();
        let mut connect = move |_: &sm_cli::Settings| -> Result<Box<dyn Endpoint>, String> {
            Ok(Box::new(common::Fake {
                log: l2.clone(),
                reply: Box::new(|req| match req {
                    Request::Export { .. } => Ok(json!({"path": "a.smsh", "format": "smsh", "bytes": 1, "sha256": "0".repeat(64)})),
                    _ => Ok(common::status_value()),
                }),
            }))
        };
        let mut stdin = std::io::Cursor::new(Vec::new());
        let (mut out, mut err) = (Vec::new(), Vec::new());
        let mut io = sm_cli::Io { stdin: &mut stdin, stdout: &mut out, stderr: &mut err };
        let mut argv = vec!["smesh", "--memory"];
        argv.extend_from_slice(args);
        let code = sm_cli::run(argv, &mut connect, &mut io);
        let reqs = log.borrow().clone();
        (code, reqs)
    };
    let (code, reqs) = run(&["export", "a.smsh", "--regions", "lattice,ring"]);
    assert_eq!(code, 0);
    let Request::Export { options, source, format, .. } = &reqs[0] else { panic!("{reqs:?}") };
    assert_eq!(options.as_ref().unwrap(), &json!({"regions": {"split": ["lattice", "ring"]}}));
    assert_eq!(*source, ExportSource::Mesh);
    assert_eq!(*format, Some(ExportFormat::Smsh));
    // Merged with another `--option`; conflicting spellings are a usage error.
    let (_, reqs) = run(&["export", "a.smsh", "--regions", "sector", "--option", "x=1"]);
    let Request::Export { options, .. } = &reqs[0] else { panic!() };
    assert_eq!(options.as_ref().unwrap(), &json!({"x": 1, "regions": {"split": ["sector"]}}));
    let (code, reqs) = run(&["export", "a.smsh", "--regions", "lattice", "--option", "regions={}"]);
    assert_eq!(code, 2);
    assert!(reqs.is_empty());
    let (_, reqs) = run(&["export", "a.h5m", "--source", "model"]);
    let Request::Export { source, .. } = &reqs[0] else { panic!() };
    assert_eq!(*source, ExportSource::Model);
}
