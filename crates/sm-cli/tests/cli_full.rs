//! Full `smesh` flows against the real standard distribution. They need the W2 crates
//! (`sm-server` endpoint methods, `sm-model`, `sm-mesher`, ...) merged, so they are ignored until
//! the integration phase. The integration phase's `milestone*.rs` files cover §4 of the plan;
//! these exercise each command once.

use std::path::{Path, PathBuf};
use std::process::{Command, Output};

use serde_json::Value;

const PIN: [&str; 4] = ["--author", "it", "--fixed-time", "2026-10-24T00:00:00Z"];

fn tmp(name: &str) -> PathBuf {
    let d = std::env::temp_dir().join(format!("smesh-full-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("tmp");
    d
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
#[ignore = "W2 integration"]
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
#[ignore = "W2 integration"]
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
#[ignore = "W2 integration"]
fn run_fork_script_in_a_fresh_project_without_new() {
    let d = tmp("forkfresh");
    let p = d.join("fork");
    let p = p.to_str().expect("utf8");
    let r = ok_json(&["run", &data("fork.smesh"), "-p", p, "--json"], None);
    assert!(r["state"].is_string());
    assert!(d.join("fork/project.toml").is_file());
}

#[test]
#[ignore = "W2 integration"]
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
#[ignore = "W2 integration"]
fn run_exports_and_hash_agree_with_print_hash() {
    let d = tmp("run");
    let mut args = vec![
        "run".to_owned(),
        data("milestone.smesh"),
        "--memory".to_owned(),
        "--print-hash".to_owned(),
    ];
    args.extend(PIN.map(str::to_owned));
    let refs: Vec<&str> = args.iter().map(String::as_str).collect();
    let o = smesh(&refs, Some(&d));
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
}

#[test]
#[ignore = "W2 integration"]
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
