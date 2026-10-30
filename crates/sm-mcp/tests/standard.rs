//! sm-mcp over the real standard server: the tool list, tool names and `session_stage`.
#![allow(missing_docs)]

use std::io::Cursor;

use serde_json::{Value, json};
use sm_server::{Server, standard};

fn run(server: &mut Server, msgs: &[Value]) -> Vec<Value> {
    let input: String = msgs.iter().map(|m| format!("{m}\n")).collect();
    let mut out = Vec::new();
    sm_mcp::run_stdio(server, Cursor::new(input.into_bytes()), &mut out).expect("run");
    String::from_utf8(out)
        .expect("utf8")
        .lines()
        .map(|l| serde_json::from_str(l).expect("json"))
        .collect()
}

fn init() -> Value {
    json!({"jsonrpc":"2.0","id":0,"method":"initialize","params":{"protocolVersion":"2025-06-18"}})
}

fn tool_names() -> Vec<String> {
    let mut s = Server::new(standard()).expect("server");
    let r = run(&mut s, &[init(), json!({"jsonrpc":"2.0","id":1,"method":"tools/list"})]);
    r[1]["result"]["tools"]
        .as_array()
        .expect("tools")
        .iter()
        .map(|t| t["name"].as_str().expect("name").to_owned())
        .collect()
}

#[test]
fn golden_standard_tool_list() {
    let names = tool_names();
    let text: String = names.iter().map(|n| format!("{n}\n")).collect();
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/golden/tools.standard.txt");
    if std::env::var_os("SM_MCP_BLESS").is_some() {
        std::fs::write(&path, &text).expect("bless");
    }
    assert_eq!(text, std::fs::read_to_string(&path).expect("golden"), "tool list drifted");
}

#[test]
fn every_tool_name_is_valid_and_new_plugins_are_present() {
    let names = tool_names();
    for n in &names {
        assert!(n.len() <= 64, "{n}");
        assert!(n.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-'), "{n}");
    }
    for want in [
        "session_stage", "rt__info", "rt__check", "rt__locate", "mesh__validate", "mesh__quality",
        "mesh__flag_cells", "mesh__smooth", "mesh__to_poly", "model__import_step",
    ] {
        assert!(names.iter().any(|n| n == want), "missing {want}");
    }
}

#[test]
fn session_stage_stages_a_file() {
    let dir = std::env::temp_dir().join(format!("sm-mcp-stage-{}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("mkdir");
    let f = dir.join("part $1.step");
    std::fs::write(&f, b"hello").expect("write");
    let mut s = Server::new(standard()).expect("server");
    let call = |id: u64, path: &str| json!({"jsonrpc":"2.0","id":id,"method":"tools/call",
        "params":{"name":"session_stage","arguments":{"path":path}}});
    let r = run(&mut s, &[init(), call(1, &f.to_string_lossy()), call(2, "/definitely/not/here.step")]);
    let ok = &r[1]["result"];
    assert_eq!(ok["isError"], false, "{ok}");
    assert_eq!(ok["structuredContent"]["bytes"], 5);
    assert_eq!(ok["structuredContent"]["sha256"].as_str().map(str::len), Some(64));
    assert_eq!(r[2]["result"]["isError"], true);
    let _ = std::fs::remove_dir_all(&dir);
}
