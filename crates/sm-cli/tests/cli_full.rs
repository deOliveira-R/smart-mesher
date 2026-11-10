//! Full `smesh` flows against the real standard distribution. They need the W2 crates
//! (`sm-server` endpoint methods, `sm-model`, `sm-mesher`, ...). The integration phase's `milestone*.rs` files cover §4 of the plan;
//! these exercise each command once.

mod common;

use std::path::Path;
use std::process::{Command, Output};

use common::Scratch;
use serde_json::Value;

const PIN: [&str; 4] = ["--author", "it", "--fixed-time", "2026-10-24T00:00:00Z"];

fn tmp(name: &str) -> Scratch {
    Scratch::new(&format!("smesh-full-{}-{name}", std::process::id()))
}

fn smesh(args: &[&str], cwd: Option<&Path>) -> Output {
    let mut c = Command::new(env!("CARGO_BIN_EXE_smesh"));
    c.args(args);
    if let Some(d) = cwd {
        c.current_dir(d);
    }
    c.output().expect("spawn")
}

fn ok_json(args: &[&str], cwd: Option<&Path>) -> Value {
    let o = smesh(args, cwd);
    assert_eq!(
        o.status.code(),
        Some(0),
        "{args:?}: {}",
        String::from_utf8_lossy(&o.stderr)
    );
    serde_json::from_slice(&o.stdout).expect("json stdout")
}

fn data(name: &str) -> String {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
        .to_string_lossy()
        .into_owned()
}

#[test]
fn timeline_commands_round_trip_in_a_project() {
    let d = tmp("timeline");
    let p = d.join("proj");
    let p = p.to_str().expect("utf8");
    assert_eq!(smesh(&["new", p], None).status.code(), Some(0));
    let a = ok_json(
        &[
            "apply",
            "model.create_brick",
            r#"{"x":10}"#,
            "-p",
            p,
            "--json",
        ],
        None,
    );
    let s1 = a["state"].as_str().expect("state").to_owned();
    ok_json(
        &[
            "apply",
            "model.move",
            r#"{"entities":["TopoBody#1"],"delta":[1,0,0]}"#,
            "-p",
            p,
            "--json",
        ],
        None,
    );
    let st = ok_json(&["status", "-p", p, "--json"], None);
    assert_eq!(st["depth"], 2);
    assert_eq!(
        ok_json(&["undo", "-p", p, "--json"], None)["state"],
        s1.as_str(),
        "undo returns to the brick"
    );
    assert_eq!(ok_json(&["redo", "-p", p, "--json"], None)["depth"], 2);
    let node = st["position"].as_str().expect("node").to_owned();
    ok_json(&["fork", "alt", "--at", &node, "-p", p, "--json"], None);
    let log = ok_json(&["log", "-p", p, "--json"], None);
    assert!(log["branches"]["alt"].is_string() && log["branches"]["main"].is_string());
    ok_json(&["checkout", "main", "-p", p, "--json"], None);
    // Strict cold replay on reopen gives the same hash.
    let h = ok_json(&["hash", "-p", p, "--json"], None);
    assert_eq!(h["state"], st["state"]);
}

#[test]
fn fixed_time_and_author_make_node_hashes_reproducible() {
    let d = tmp("pinned");
    let mut nodes = Vec::new();
    for n in ["a", "b"] {
        let p = d.join(n);
        let p = p.to_str().expect("utf8");
        assert_eq!(smesh(&["new", p], None).status.code(), Some(0));
        let mut args = vec![
            "apply",
            "model.create_brick",
            r#"{"x":10}"#,
            "-p",
            p,
            "--json",
        ];
        args.extend(PIN);
        nodes.push(ok_json(&args, None)["node"].clone());
    }
    assert_eq!(nodes[0], nodes[1]);
}

#[test]
fn run_fork_script_in_a_fresh_project_without_new() {
    let d = tmp("forkfresh");
    let p = d.join("fork");
    let p = p.to_str().expect("utf8");
    let r = ok_json(&["run", &data("fork.smesh"), "-p", p, "--json"], None);
    assert!(r["state"].is_string());
    assert!(d.join("fork/project.toml").is_file());
}

#[test]
fn unpinned_runs_differ_in_node_hashes_but_not_in_state() {
    let d = tmp("unpinned");
    let mut out = Vec::new();
    for n in ["a", "b"] {
        let p = d.join(n);
        let p = p.to_str().expect("utf8");
        assert_eq!(smesh(&["new", p], None).status.code(), Some(0));
        std::thread::sleep(std::time::Duration::from_millis(1100));
        out.push(ok_json(
            &[
                "apply",
                "model.create_brick",
                r#"{"x":10}"#,
                "-p",
                p,
                "--json",
            ],
            None,
        ));
    }
    assert_eq!(out[0]["state"], out[1]["state"]);
    assert_ne!(
        out[0]["node"], out[1]["node"],
        "the pin is what makes nodes equal"
    );
}

#[test]
fn run_exports_and_hash_agree_with_print_hash() {
    let d = tmp("run");
    let script = data("milestone.smesh");
    let mut printed = vec!["run", script.as_str(), "--memory", "--print-hash"];
    printed.extend(PIN);
    let o = smesh(&printed, Some(&d));
    assert_eq!(
        o.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&o.stderr)
    );
    assert!(
        d.join("out.smsh").is_file() && d.join("out.vtu").is_file(),
        "relative exports land in the working directory under --memory"
    );
    let stdout = String::from_utf8(o.stdout).expect("utf8");
    let last = stdout.lines().last().expect("a last line");
    assert!(
        last.len() == 64
            && last
                .bytes()
                .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b)),
        "last line is 64 lowercase hex: {last:?}"
    );
    let mut json = vec!["run", script.as_str(), "--memory", "--json"];
    json.extend(PIN);
    let r = ok_json(&json, Some(&d));
    assert_eq!(
        r["state"], last,
        "print-hash is the state of the --json run"
    );
}

/// `new` on a directory that already holds a project is an error and leaves it untouched.
#[test]
fn new_on_an_existing_project_is_an_error() {
    let d = tmp("newtwice");
    let p = d.join("proj");
    let p = p.to_str().expect("utf8");
    assert_eq!(smesh(&["new", p], None).status.code(), Some(0));
    let before = std::fs::read(Path::new(p).join("project.toml")).expect("toml");
    let o = smesh(&["new", p], None);
    assert_eq!(o.status.code(), Some(1));
    let err = String::from_utf8_lossy(&o.stderr);
    assert!(err.contains("already holds a project"), "{err}");
    assert!(!String::from_utf8_lossy(&o.stdout).contains("created project"));
    assert_eq!(
        std::fs::read(Path::new(p).join("project.toml")).expect("toml"),
        before
    );
}

#[test]
fn export_command_writes_the_requested_file() {
    let d = tmp("export");
    let p = d.join("proj");
    let p = p.to_str().expect("utf8");
    assert_eq!(smesh(&["new", p], None).status.code(), Some(0));
    let mut args = vec![
        "run".to_owned(),
        data("milestone.smesh"),
        "-p".to_owned(),
        p.to_owned(),
    ];
    args.extend(PIN.map(str::to_owned));
    let refs: Vec<&str> = args.iter().map(String::as_str).collect();
    assert_eq!(smesh(&refs, None).status.code(), Some(0));
    let target = d.join("re.smsh");
    let r = ok_json(
        &["export", target.to_str().expect("utf8"), "-p", p, "--json"],
        None,
    );
    assert!(target.is_file() && r["bytes"].as_u64().is_some_and(|b| b > 0));
}

/// Regression: the connector must build a server that can `session.open` (not `open-unsupported`).
#[test]
fn new_creates_a_project_instead_of_open_unsupported() {
    let d = tmp("open");
    let p = d.join("x");
    let o = smesh(&["new", p.to_str().expect("utf8")], None);
    assert_eq!(
        o.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&o.stderr)
    );
    assert!(p.join("project.toml").is_file());
}

/// W4-CLI-DIAG: the enclosed pin falls back to tets in two volumes. Text mode prints one
/// `warning: mesh.auto_tet: ...` line per volume on stderr, stdout is the script's own output,
/// and the lines equal the `delta.diagnostics` of the `--json` result.
#[test]
fn run_prints_per_op_diagnostics_on_stderr_and_they_match_json() {
    let d = tmp("diag");
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/w3c_pin_enclosed.smesh");
    let script = script.to_str().expect("utf8");
    let text = smesh(&["run", script, "--memory"], Some(&d));
    assert_eq!(text.status.code(), Some(0), "{}", String::from_utf8_lossy(&text.stderr));
    let json = smesh(&["run", script, "--memory", "--json"], Some(&d));
    assert_eq!(json.status.code(), Some(0));
    assert!(json.stderr.is_empty(), "--json prints nothing on stderr: {}", String::from_utf8_lossy(&json.stderr));
    let v: Value = serde_json::from_slice(&json.stdout).expect("json stdout");
    let want: Vec<String> = v["events"]
        .as_array()
        .expect("events")
        .iter()
        .flat_map(|e| e["delta"]["diagnostics"].as_array().cloned().unwrap_or_default())
        .map(|x| format!("{}: {}: {}", x["severity"].as_str().unwrap(), x["code"].as_str().unwrap(), x["message"].as_str().unwrap()))
        .collect();
    let got: Vec<String> = String::from_utf8_lossy(&text.stderr).lines().map(str::to_owned).collect();
    assert_eq!(got, want);
    assert_eq!(want.len(), 2, "one per tet-fallback volume: {want:?}");
    assert!(want.iter().all(|l| l.starts_with("warning: mesh.auto_tet: ")), "{want:?}");
    // stdout is the script's printed text, as before.
    assert_eq!(String::from_utf8_lossy(&text.stdout), v["output"].as_str().expect("output"));
}

/// A script with no diagnostics prints nothing on stderr.
#[test]
fn run_of_a_clean_script_prints_no_extra_lines() {
    let d = tmp("diag_clean");
    let s = d.join("clean.smesh");
    std::fs::write(&s, "let b = model.create_brick(10.0)\nprint 1\n").expect("write");
    let o = smesh(&["run", s.to_str().expect("utf8"), "--memory"], Some(&d));
    assert_eq!(o.status.code(), Some(0));
    assert!(o.stderr.is_empty(), "{}", String::from_utf8_lossy(&o.stderr));
}
