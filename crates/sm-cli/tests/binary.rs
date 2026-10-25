//! The real `smesh` binary against the standard in-process server. Only behaviour that does
//! not depend on the not-yet-merged server crates is asserted here (catalog, hash, usage,
//! transports, error mapping); the full flows live in `tests/cli_full.rs`.

use std::io::Write;
use std::process::{Command, Output, Stdio};

use serde_json::{Value, json};
use sm_protocol::HashResult;

fn smesh(args: &[&str], stdin: &str, rayon_threads: Option<usize>) -> Output {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_smesh"));
    // Relative exports of a `run --memory` script land in a directory private to this call, so
    // concurrent processes never share an export path (or its `.part` temp file).
    static CALLS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    let n = CALLS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("{}-{n}", std::process::id()));
    std::fs::create_dir_all(&dir).expect("per-call tmpdir");
    cmd.args(args)
        .current_dir(&dir)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    match rayon_threads {
        Some(n) => cmd.env("RAYON_NUM_THREADS", n.to_string()),
        None => cmd.env_remove("RAYON_NUM_THREADS"),
    };
    let mut child = cmd.spawn().expect("spawn smesh");
    child
        .stdin
        .take()
        .expect("stdin")
        .write_all(stdin.as_bytes())
        .expect("write stdin");
    child.wait_with_output().expect("wait")
}

fn text(o: &Output) -> String {
    String::from_utf8(o.stdout.clone()).expect("utf8 stdout")
}

fn code(o: &Output) -> i32 {
    o.status.code().expect("exit code")
}

#[test]
fn version_and_usage_exit_codes() {
    let v = smesh(&["--version"], "", None);
    assert_eq!(
        (code(&v), text(&v).trim().to_owned()),
        (0, sm_cli::version_line())
    );
    let none = smesh(&[], "", None);
    assert_eq!(code(&none), 2);
    assert!(String::from_utf8_lossy(&none.stderr).starts_with("usage:"));
    assert_eq!(code(&smesh(&["hash", "--bogus"], "", None)), 2);
    assert_eq!(code(&smesh(&["hash", "--memory", "-p", "x"], "", None)), 2);
    assert_eq!(
        code(&smesh(
            &["hash", "--memory", "--fixed-time", "noon"],
            "",
            None
        )),
        2
    );
}

#[test]
fn help_lists_the_catalog_and_describes_an_op() {
    let all = smesh(&["help", "--memory"], "", None);
    assert_eq!(code(&all), 0, "{}", String::from_utf8_lossy(&all.stderr));
    assert!(text(&all).contains("core.batch"), "{}", text(&all));
    let one = smesh(&["help", "core.batch"], "", None);
    assert_eq!(code(&one), 0);
    assert!(
        text(&one).starts_with("core.batch (op, version "),
        "{}",
        text(&one)
    );
    let json = smesh(&["help", "core.batch", "--json"], "", None);
    assert_eq!(
        serde_json::from_str::<Value>(&text(&json)).unwrap()["kind"],
        "op"
    );
    assert_eq!(code(&smesh(&["help", "no.such_op"], "", None)), 2);
}

#[test]
fn hash_json_is_a_hash_result() {
    let o = smesh(&["hash", "--memory", "--json"], "", None);
    assert_eq!(code(&o), 0, "{}", String::from_utf8_lossy(&o.stderr));
    let r: HashResult = serde_json::from_str(&text(&o)).expect("HashResult");
    assert!(r.layers.is_empty() || r.layers.keys().all(|k| !k.is_empty()));
    let human = smesh(&["hash", "--memory"], "", None);
    assert_eq!(
        text(&human).trim(),
        r.state.to_string(),
        "the empty state hashes the same in both forms"
    );
}

#[test]
fn an_unknown_op_fails_with_exit_1_and_a_typed_error() {
    let o = smesh(
        &["apply", "no.such_op", "{}", "--memory", "--json"],
        "",
        None,
    );
    assert_eq!(code(&o), 1);
    let v: Value = serde_json::from_str(&text(&o)).expect("json error object");
    assert!(
        v["error"]["kind"].as_str().is_some_and(|k| !k.is_empty()),
        "{v}"
    );
    assert!(v["error"]["code"].as_i64().is_some());
    let h = smesh(&["apply", "no.such_op", "{}", "--memory"], "", None);
    assert!(String::from_utf8_lossy(&h.stderr).starts_with("error["));
}

#[test]
fn serve_stdio_speaks_json_rpc_lines() {
    let input = "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"catalog\"}\n{broken\n{\"jsonrpc\":\"2.0\",\"id\":3,\"method\":\"session.hash\"}\n";
    let o = smesh(&["serve", "--stdio", "--memory"], input, None);
    assert_eq!(code(&o), 0);
    let lines: Vec<Value> = text(&o)
        .lines()
        .map(|l| serde_json::from_str(l).expect("json line"))
        .collect();
    assert_eq!(lines.len(), 3);
    assert_eq!(lines[0]["id"], 1);
    assert_eq!(
        lines[0]["result"]["protocol"],
        sm_protocol::PROTOCOL_VERSION
    );
    assert_eq!(
        (lines[1]["id"].clone(), lines[1]["error"]["code"].clone()),
        (Value::Null, json!(sm_protocol::PARSE_ERROR))
    );
    assert!(
        lines[2]["result"]["state"]
            .as_str()
            .is_some_and(|s| s.len() == 64)
    );
}

#[test]
fn mcp_initializes_over_stdio() {
    let input = "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"initialize\",\"params\":{\"protocolVersion\":\"2025-06-18\",\"capabilities\":{},\"clientInfo\":{\"name\":\"t\",\"version\":\"0\"}}}\n{\"jsonrpc\":\"2.0\",\"method\":\"notifications/initialized\"}\n";
    let o = smesh(&["mcp", "--memory"], input, None);
    assert_eq!(code(&o), 0, "{}", String::from_utf8_lossy(&o.stderr));
    let first: Value = serde_json::from_str(text(&o).lines().next().expect("a reply")).unwrap();
    assert_eq!(first["id"], 1);
    assert_eq!(first["result"]["protocolVersion"], "2025-06-18");
}

/// Determinism: the same invocation gives byte-identical stdout run twice, with one rayon thread
/// versus the default, and from concurrent processes.
#[test]
fn output_is_identical_across_runs_and_thread_counts() {
    let run = |threads: usize| -> Vec<Vec<u8>> {
        [
            smesh(&["hash", "--memory", "--json"], "", Some(threads)).stdout,
            smesh(&["help", "--memory", "--json"], "", Some(threads)).stdout,
            smesh(&["help", "core.batch"], "", Some(threads)).stdout,
            smesh(
                &[
                    "run",
                    &format!("{}/tests/data/milestone.smesh", env!("CARGO_MANIFEST_DIR")),
                    "--memory",
                    "--print-hash",
                    "--author",
                    "it",
                    "--fixed-time",
                    "2026-10-24T00:00:00Z",
                ],
                "",
                Some(threads),
            )
            .stdout,
            smesh(
                &["serve", "--memory"],
                "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"catalog\"}\n",
                Some(threads),
            )
            .stdout,
        ]
        .to_vec()
    };
    let baseline = sm_testkit::determinism::check(2, 3, run).expect("deterministic");
    assert!(baseline.iter().all(|b| !b.is_empty()));
}
