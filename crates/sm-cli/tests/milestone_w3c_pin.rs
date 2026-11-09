//! W3c round-pin milestone (plan `docs/design/w3c/plan.md` section 5, item 1; rulings C1, C4, C10).
//!
//! `tools/w3c_pin.smesh` builds a through pin (H = 1.0: a moderator brick minus a consumed copy of
//! the fuel cylinder, imprint, merge) and its 3x3 lattice with `model.copy` (18 volumes), lets
//! `mesh.auto { strategy: "default" }` choose the schemes (all-hex, 432 hexes), meshes, validates,
//! and writes the `.smrt` and an OpenFOAM case. `tools/w3c_pin_enclosed.smesh` is the enclosed pin
//! (H = 0.8, all-tet by C1). `tools/w3c_pin_leftout.smesh` leaves one volume out of the blocks (the
//! R10 warning) and `tools/w3c_warped.smesh` is the warped-hexahedron fixture for
//! `split_warped_faces` (R11). Collect-all style like `milestone_w3b`: every row is evaluated and
//! the test fails once at the end with the whole report. Run it as
//! `cargo test -p sm-cli --test milestone_w3c_pin -- --nocapture` (and with `RAYON_NUM_THREADS=1`).
//!
//! | row | what |
//! |---|---|
//! | P1 | through pin and 3x3 lattice by `model.copy`: 18 volumes, volume 9 x 1.26^2, 432 hexes (script asserts) |
//! | P2 | `mesh.auto` stores explicit assignments; replay under an unregistered strategy reaches the same hash |
//! | P3 | `validate` is empty (conformal interfaces) |
//! | P4 | blocks per material; `.smrt` with no warning, loaded by `sm_rt::read_smrt` with regions = blocks |
//! | P5 | a volume left out of the blocks: warning in CLI stderr, MCP text, Python `ExportWarning` (export-opts acceptance 6, 8(d)) |
//! | P6 | OpenFOAM with and without `split_warped_faces` through CLI and Python (acceptance 6, 8(c)) |
//! | P7 | `script.record` and replay equal; MCP `record_script` equals the CLI record |
//! | P8 | enclosed pin (H = 0.8) is all-tet with a `mesh.auto_tet` warning per volume (C1) |
//! | P9 | CLI, MCP, in-process and Python state hashes and files equal |
//! | P10 | two runs and `RAYON_NUM_THREADS=1` vs default equal |
//! | P11 | the state hashes pinned below (new hashes, blessed on macOS arm64 on the INT commit) |

mod w3a_support;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::time::Instant;

use serde_json::{Value, json};
use sm_protocol::{Endpoint, ExportFormat, ExportResult, Request};
use sm_rt_api::Tracker;
use sm_server::{Server, standard_with};

use w3a_support::*;

/// Pinned state hashes, `(h_model, h_auto, h_mesh)` of each script, new in W3c (not re-blessed).
/// Measured on macOS arm64; per-platform identity is `tools/platform/expected.txt`'s business.
const PIN_HASHES: [&str; 3] = [
    "f238f0340fc123ef799f241e7f9ff99e3d5d7ef848113f9a6c20f768d47b4e9d",
    "1f4b0912569abb477044a9d2bf42f024b5936a30220888ea5bfd28f9234fdbe1",
    "6953b909cfbaf576efe1828577f6daf73039b0fa4934677a99dab0172f1a36f6",
];
const ENCLOSED_HASHES: [&str; 3] = [
    "8cdcb65bfef688ff4804606968a1be43f8261b7b5bd9a683bbd22ffcef958bfe",
    "5dd202e512ebe0b55bed8c6f88a1e1449e625da091ba4c6d94a0841d0a6ffc47",
    "0804bbbffc493a1a399145c657edbb7554381df129af432830f312a4a72204b2",
];
const LEFTOUT_H_MESH: &str = "87055d625319ddbce42e6a913115c380151972a7294f215a7144b17eb6340256";
const WARPED_H_MESH: &str = "45de87c0fddd5b56bdc26eddf66e416fd69db68987a4e26b9cc2c9aa9d3880b8";

const KEYS: [&str; 3] = ["h_model", "h_auto", "h_mesh"];
const PIN_FILES: [&str; 2] = ["pin.smrt", "pin_foam"];
const ENCLOSED_FILES: [&str; 1] = ["enclosed.smrt"];

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("repo root")
}

fn tools() -> PathBuf {
    repo().join("tools")
}

fn script_path(name: &str) -> PathBuf {
    tools().join(name)
}

fn script_text(name: &str) -> String {
    read_text(&script_path(name))
}

fn base() -> PathBuf {
    Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("milestone_w3c_pin-{}", std::process::id()))
}

fn scratch(name: &str) -> PathBuf {
    let d = base().join(name);
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("scratch dir");
    d
}

fn ts(p: &Path) -> String {
    p.to_string_lossy().into_owned()
}

/// Removes this process's scratch tree when the test ends, pass or fail.
struct Cleanup;

impl Drop for Cleanup {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(base());
    }
}

type Prints = BTreeMap<String, Vec<String>>;

fn prints_of(text: &str) -> Prints {
    let mut m: Prints = BTreeMap::new();
    for l in text.lines() {
        if let Some((k, v)) = l.split_once(' ')
            && k.chars().all(|c| c.is_ascii_lowercase() || c == '_')
        {
            m.entry(k.to_owned()).or_default().push(v.trim().to_owned());
        }
    }
    m
}

fn one(p: &Prints, k: &str) -> R<String> {
    p.get(k).and_then(|v| v.first()).cloned().ok_or_else(|| format!("no `{k}` line in the output"))
}

/// sha256 of a file, or of a directory: sha256 over the sorted `relpath sha256` lines of its files
/// (the Python driver computes the same).
fn digest(p: &Path) -> R<String> {
    if p.is_dir() {
        let lines: String = tree(p)?.iter().map(|(k, v)| format!("{k} {}\n", sha_hex(v))).collect();
        Ok(sha_hex(lines.as_bytes()))
    } else {
        Ok(sha_hex(&read_bytes(p)?))
    }
}

/// What one front end produced for one script.
struct Front {
    name: String,
    prints: Prints,
    files: BTreeMap<String, String>,
}

impl Front {
    fn new(name: &str, output: &str, dir: &Path, files: &[&str]) -> R<Self> {
        let files = files.iter().map(|f| Ok(((*f).to_owned(), digest(&dir.join(f))?))).collect::<R<_>>()?;
        Ok(Self { name: name.to_owned(), prints: prints_of(output), files })
    }
}

// ------------------------------------------------------------------ front ends

fn cli_memory(script: &str, name: &str, threads: Threads, files: &[&str]) -> R<Front> {
    let dir = scratch(name);
    let args = ["--memory", "run", &ts(&script_path(script)), "--author", AUTHOR, "--fixed-time", TIME];
    let out = ok_text(&args, Some(&dir), threads)?;
    Front::new(name, &out, &dir, files)
}

/// The project run: files land in the project root and the timeline is on disk for `record` and `export`.
fn cli_project(script: &str, name: &str, files: &[&str]) -> R<(Front, PathBuf)> {
    let dir = scratch(name);
    let proj = dir.join("proj");
    ok_text(&["new", &ts(&proj)], Some(&dir), Threads::Inherit)?;
    let out = ok_text(&["-p", &ts(&proj), "run", &ts(&script_path(script)), "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit)?;
    Ok((Front::new(name, &out, &proj, files)?, proj))
}

/// `smesh -p proj export ...`: exit code, stdout and stderr.
fn cli_export(proj: &Path, args: &[&str]) -> R<(i32, String, String)> {
    let mut a = vec!["-p", "PROJ", "export"];
    let p = ts(proj);
    a[1] = &p;
    a.extend_from_slice(args);
    let o: Output = spawn(&a, Some(proj), Threads::Inherit, None)?;
    Ok((o.status.code().unwrap_or(-1), String::from_utf8_lossy(&o.stdout).into_owned(), String::from_utf8_lossy(&o.stderr).into_owned()))
}

struct Mcp {
    front: Front,
    record: Value,
    export_text: String,
}

/// One MCP session: run the script, hash, record, then export `<export>` and keep its text content.
fn mcp_front(script: &str, name: &str, files: &[&str], export: &str) -> R<Mcp> {
    let dir = scratch(name);
    let mut lines = mcp_init();
    lines.push(tool_call(2, "run_script", json!({"text": script_text(script)})));
    lines.push(tool_call(3, "session_hash", json!({})));
    lines.push(tool_call(4, "record_script", json!({"forks": "refs"})));
    lines.push(tool_call(5, "session_export", json!({"path": export})));
    let replies = mcp_session(&PIN, &lines, Threads::Inherit, Some(&dir))?;
    let run = tool_ok(&replies, 2)?;
    let hash = tool_ok(&replies, 3)?;
    let record = tool_ok(&replies, 4)?;
    tool_ok(&replies, 5)?;
    let export_text = replies[&5]["result"]["content"][0]["text"].as_str().ok_or("session_export has no text content")?.to_owned();
    let out = run["output"].as_str().ok_or_else(|| format!("run_script has no output: {run}"))?;
    let front = Front::new(name, out, &dir, files)?;
    if hash["state"].as_str() != Some(one(&front.prints, "h_mesh")?.as_str()) {
        return Err(format!("MCP session_hash {hash} != printed h_mesh"));
    }
    Ok(Mcp { front, record, export_text })
}

/// An in-process server with the script run in it.
fn in_process_server(script: &str, name: &str) -> R<(Server, Value, PathBuf)> {
    let dir = scratch(name);
    let proj = dir.join("proj");
    let mut s = Server::create(standard_with(Some(AUTHOR.into()), Some(TIME.into())), &proj, "w3c").map_err(e)?;
    let v = s
        .call(Request::ScriptRun { text: script_text(script), path: None, includes: Default::default() })
        .map_err(|x| format!("script.run: {x} {}", x.data["rendered"].as_str().unwrap_or("")))?;
    Ok((s, v, proj))
}

fn in_process(script: &str, name: &str, files: &[&str]) -> R<(Front, Value)> {
    let (mut s, v, proj) = in_process_server(script, name)?;
    let out = v["output"].as_str().ok_or_else(|| format!("script.run has no output: {v}"))?;
    let f = Front::new(name, out, &proj, files)?;
    let h = s.call(Request::Hash { at: None }).map_err(e)?;
    if h["state"].as_str() != Some(one(&f.prints, "h_mesh")?.as_str()) {
        return Err(format!("in-process session.hash {} != printed h_mesh", h["state"]));
    }
    Ok((f, v))
}

fn export_req(path: &Path, format: Option<ExportFormat>, warp: bool) -> Request {
    Request::Export {
        path: Some(ts(path)),
        format,
        mesh: None,
        provenance: false,
        inline: false,
        options: None,
        split_small_determinant: None,
        empty_patches: vec![],
        wedge_patches: vec![],
        split_warped_faces: warp,
        source: Default::default(),
    }
}

fn export_in_process(s: &mut Server, path: &Path, format: Option<ExportFormat>, warp: bool) -> R<ExportResult> {
    let v = s.call(export_req(path, format, warp)).map_err(|x| format!("export {}: {x} {}", path.display(), x.data))?;
    serde_json::from_value(v).map_err(e)
}

/// The venv python with the wheel installed, or the reason to skip (maturin unavailable).
fn python() -> R<Result<PathBuf, String>> {
    for var in ["W3C_PYTHON", "W3B_PYTHON"] {
        if let Some(p) = std::env::var_os(var) {
            return Ok(Ok(PathBuf::from(p)));
        }
    }
    let venv = Path::new(env!("CARGO_TARGET_TMPDIR")).join("w3c-venv");
    let o = Command::new("bash")
        .arg(tools().join("w3b_python.sh"))
        .arg(&venv)
        .env("CARGO_TARGET_DIR", Path::new(env!("CARGO_TARGET_TMPDIR")).parent().expect("target dir"))
        .output()
        .map_err(|x| format!("run w3b_python.sh: {x}"))?;
    match o.status.code() {
        Some(0) => {
            let out = String::from_utf8_lossy(&o.stdout);
            Ok(Ok(PathBuf::from(out.lines().last().ok_or("w3b_python.sh printed nothing")?.trim())))
        }
        Some(3) => Ok(Err(String::from_utf8_lossy(&o.stderr).trim().to_owned())),
        _ => Err(format!("w3b_python.sh failed ({}): {}", o.status, String::from_utf8_lossy(&o.stderr))),
    }
}

/// The Python driver's `key value` lines (all of them, keys repeat) and its output directory.
fn python_run(py: &Path) -> R<(Vec<String>, PathBuf)> {
    let dir = scratch("python");
    let o = Command::new(py).arg("-I").arg(tools().join("w3c_driver.py")).arg(tools()).arg(&dir).output().map_err(|x| format!("run python driver: {x}"))?;
    let out = String::from_utf8_lossy(&o.stdout).into_owned();
    if o.status.code() != Some(0) || !out.lines().any(|l| l == "OK") {
        return Err(format!("python driver exited {:?}:\n{out}\n{}", o.status.code(), String::from_utf8_lossy(&o.stderr)));
    }
    Ok((out.lines().map(str::to_owned).collect(), dir))
}

/// The value after `prefix ` on the first driver line that starts with it.
fn line_after(lines: &[String], prefix: &str) -> R<String> {
    lines.iter().find_map(|l| l.strip_prefix(prefix).and_then(|r| r.strip_prefix(' '))).map(str::to_owned).ok_or_else(|| format!("python driver printed no `{prefix}` line"))
}

// ------------------------------------------------------------------ helpers over results

fn region_names(smrt: &Path) -> R<Vec<String>> {
    let image = sm_rt::read_smrt(&read_bytes(smrt)?).map_err(e)?;
    let loaded = sm_rt::load_image(&image).map_err(e)?;
    Ok(loaded.regions().0.iter().map(|r| r.name.clone()).collect())
}

fn auto_tet_warnings(run: &Value) -> usize {
    run["events"]
        .as_array()
        .map(|ev| ev.iter().filter(|e| e["op"] == "mesh.auto").flat_map(|e| e["delta"]["diagnostics"].as_array().cloned().unwrap_or_default()).filter(|d| d["code"] == "mesh.auto_tet").count())
        .unwrap_or(0)
}

fn floats(p: &Prints, k: &str) -> R<f64> {
    let s = one(p, k)?;
    s.parse::<f64>().map_err(|x| format!("{k}: {s}: {x}"))
}

// ------------------------------------------------------------------ the milestone

#[test]
fn milestone_w3c_pin() {
    let _cleanup = Cleanup;
    let mut rows = Rows::default();
    let t0 = Instant::now();
    let mut timings: Vec<(String, f64)> = Vec::new();
    macro_rules! timed {
        ($label:expr, $e:expr) => {{
            let t = Instant::now();
            let r = $e;
            timings.push(($label.to_owned(), t.elapsed().as_secs_f64()));
            r
        }};
    }

    // ---------------- CLI: default threads twice, one thread, and the project run
    let cli = timed!("CLI run (default threads)", cli_memory("w3c_pin.smesh", "cli", Threads::Default, &PIN_FILES));
    let cli_again = cli_memory("w3c_pin.smesh", "cli_again", Threads::Default, &PIN_FILES);
    let cli1 = cli_memory("w3c_pin.smesh", "cli_1thread", Threads::One, &PIN_FILES);
    let proj = timed!("CLI project run", cli_project("w3c_pin.smesh", "cli_project", &PIN_FILES));
    let enc = cli_memory("w3c_pin_enclosed.smesh", "enc", Threads::Default, &ENCLOSED_FILES);
    let enc_again = cli_memory("w3c_pin_enclosed.smesh", "enc_again", Threads::Default, &ENCLOSED_FILES);
    let enc1 = cli_memory("w3c_pin_enclosed.smesh", "enc_1thread", Threads::One, &ENCLOSED_FILES);
    for (label, r) in [("CLI", cli.as_ref().err()), ("CLI again", cli_again.as_ref().err()), ("CLI 1 thread", cli1.as_ref().err()), ("CLI project", proj.as_ref().err())] {
        if let Some(x) = r {
            for row in [1, 3, 4] {
                rows.fail(row, format!("{label} run failed: {x}"));
            }
        }
    }
    let Ok(cli) = cli else {
        report(&rows, &timings, t0);
        return;
    };
    let h = |f: &Front, k: &str| one(&f.prints, k);

    // P1 and P3: the script's own assertions passed (exit 0); read back what it printed.
    rows.note(1, "script asserts: 2 volumes per pin cell, 18 in the lattice, 9 fuel + 9 moderator, volume 9 x 1.26^2, 432 hexes");
    match (floats(&cli.prints, "model_volume"), floats(&cli.prints, "mesh_volume"), one(&cli.prints, "cells"), one(&cli.prints, "shapes")) {
        (Ok(v), Ok(mv), Ok(c), Ok(sh)) => {
            rows.near(1, "lattice volume", v, 9.0 * 1.26 * 1.26, 1e-9);
            rows.near(1, "mesh volume", mv, 9.0 * 1.26 * 1.26, 1e-9);
            rows.eq(1, "hex cells", &c, &"432".to_owned());
            rows.eq(1, "cell shapes", &sh, &"{Hex: 432}".to_owned());
        }
        other => rows.fail(1, format!("model_volume/mesh_volume/cells/shapes: {other:?}")),
    }
    rows.eq(3, "validate violations", &one(&cli.prints, "violations").unwrap_or_default(), &"0".to_owned());
    rows.eq(1, "script ended on main", &one(&cli.prints, "head").unwrap_or_default(), &"main".to_owned());

    // P4: the .smrt of the fully blocked lattice: regions are the blocks, and the export had no note.
    let dir = base().join("cli").join("pin.smrt");
    match region_names(&dir) {
        Ok(names) => rows.eq(4, "regions of pin.smrt are the blocks", &names, &vec!["fuel".to_owned(), "moderator".to_owned()]),
        Err(x) => rows.fail(4, format!("pin.smrt: {x}")),
    }

    // ---------------- project exports: no warning on stderr (P4), OpenFOAM with and without the flag (P6)
    let mut pin_foam: Option<(String, String)> = None;
    if let Ok((_, pdir)) = &proj {
        let out = scratch("cli_exports");
        let o = |n: &str| ts(&out.join(n));
        match cli_export(pdir, &[&o("blocked.smrt")]) {
            Ok((0, stdout, stderr)) => {
                rows.check(4, stderr.is_empty(), || format!("export of a fully blocked model printed on stderr: {stderr}"));
                rows.check(4, stdout.starts_with("wrote ") && stdout.contains("sha256"), || format!("stdout: {stdout}"));
            }
            other => rows.fail(4, format!("smesh export blocked.smrt: {other:?}")),
        }
        for (name, flag) in [("foam_off", false), ("foam_on", true)] {
            let mut args = vec![o(name), "--format".to_owned(), "openfoam".to_owned()];
            if flag {
                args.push("--split-warped-faces".into());
            }
            let a: Vec<&str> = args.iter().map(String::as_str).collect();
            match cli_export(pdir, &a) {
                Ok((0, _, stderr)) => rows.check(6, !stderr.contains("warped"), || format!("pin OpenFOAM export mentions warped faces: {stderr}")),
                other => rows.fail(6, format!("smesh export {name}: {other:?}")),
            }
        }
        match (digest(&out.join("foam_off")), digest(&out.join("foam_on"))) {
            (Ok(off), Ok(on)) => {
                // The pin's faces are planar (vertical sides of a swept polygon), so the pass leaves the case as is.
                rows.eq(6, "pin OpenFOAM case: flag on equals flag off (planar faces)", &on, &off);
                rows.eq(6, "CLI export of the OpenFOAM case equals the script's", &off, cli.files.get("pin_foam").unwrap_or(&String::new()));
                pin_foam = Some((off, on));
            }
            (a, b) => rows.fail(6, format!("foam digests: {a:?} {b:?}")),
        }
    }

    // ---------------- P5 and P6: the left-out volume and the warped hexahedron, CLI project
    let leftout = cli_project("w3c_pin_leftout.smesh", "cli_leftout", &[]);
    let warped = cli_project("w3c_warped.smesh", "cli_warped", &[]);
    let mut cli_left_smrt = None;
    match &leftout {
        Ok((f, pdir)) => {
            rows.eq(5, "left-out h_mesh", &h(f, "h_mesh").unwrap_or_default(), &LEFTOUT_H_MESH.to_owned());
            let out = scratch("cli_leftout_out");
            let target = out.join("leftout.smrt");
            match cli_export(pdir, &[&ts(&target)]) {
                Ok((0, stdout, stderr)) => {
                    rows.check(5, stderr.lines().any(|l| l.starts_with("warning: smrt-region-names: ") && l.contains("cells are in no block")), || format!("CLI stderr lacks the warning: {stderr}"));
                    rows.check(5, stdout.starts_with("wrote ") && !stdout.contains("warning"), || format!("CLI stdout is not unchanged: {stdout}"));
                    rows.note(5, format!("CLI stderr: {}", stderr.trim()));
                }
                other => rows.fail(5, format!("smesh export leftout.smrt: {other:?}")),
            }
            match region_names(&target) {
                Ok(n) => rows.check(5, !n.is_empty() && n.iter().all(|x| x.starts_with("TopoVolume#")), || format!("regions of the left-out export: {n:?}")),
                Err(x) => rows.fail(5, format!("leftout.smrt: {x}")),
            }
            // --json: stdout is the raw result, stderr still carries the warning.
            let jt = ts(&out.join("j.smrt"));
            match spawn(&["--json", "-p", &ts(pdir), "export", &jt], Some(pdir), Threads::Inherit, None) {
                Ok(o) => {
                    let stderr = String::from_utf8_lossy(&o.stderr);
                    let v: Result<Value, _> = serde_json::from_slice(&o.stdout);
                    rows.check(5, o.status.code() == Some(0) && stderr.contains("warning: smrt-region-names"), || format!("--json export: {:?} {stderr}", o.status.code()));
                    rows.check(5, v.as_ref().is_ok_and(|v| v["notes"][0]["severity"] == "warning"), || format!("--json stdout: {:?}", String::from_utf8_lossy(&o.stdout)));
                }
                Err(x) => rows.fail(5, format!("--json export: {x}")),
            }
            cli_left_smrt = digest(&target).ok();
        }
        Err(x) => rows.fail(5, format!("left-out project run: {x}")),
    }

    let mut cli_warped: Option<(String, String)> = None;
    match &warped {
        Ok((f, pdir)) => {
            rows.eq(6, "warped h_mesh", &h(f, "h_mesh").unwrap_or_default(), &WARPED_H_MESH.to_owned());
            let out = scratch("cli_warped_out");
            let o = |n: &str| ts(&out.join(n));
            let off = cli_export(pdir, &[&o("w_off"), "--format", "openfoam"]);
            let on = cli_export(pdir, &[&o("w_on"), "--format", "openfoam", "--split-warped-faces"]);
            match (&off, &on) {
                (Ok((0, _, e_off)), Ok((0, _, e_on))) => {
                    rows.check(6, !e_off.contains("foam.warped_faces_split"), || format!("flag off mentions a split: {e_off}"));
                    rows.check(6, e_on.contains("foam.warped_faces_split"), || format!("flag on has no foam.warped_faces_split note on stderr: {e_on}"));
                    rows.note(6, format!("CLI stderr with the flag: {}", e_on.trim()));
                }
                other => rows.fail(6, format!("warped CLI exports: {other:?}")),
            }
            match (digest(&out.join("w_off")), digest(&out.join("w_on"))) {
                (Ok(a), Ok(b)) => {
                    rows.check(6, a != b, || "the warped case is unchanged by split_warped_faces".into());
                    cli_warped = Some((a, b));
                }
                (a, b) => rows.fail(6, format!("warped digests: {a:?} {b:?}")),
            }
            // A non-OpenFOAM format refuses the flag.
            match cli_export(pdir, &[&o("w.msh"), "--format", "gmsh", "--split-warped-faces"]) {
                Ok((code, _, stderr)) => rows.check(6, code != 0 && stderr.contains("split_warped_faces"), || format!("gmsh with the flag: exit {code}: {stderr}")),
                Err(x) => rows.fail(6, format!("gmsh with the flag: {x}")),
            }
        }
        Err(x) => rows.fail(6, format!("warped project run: {x}")),
    }

    // ---------------- P2 and P7: record and replay
    if let Ok((_, pdir)) = &proj {
        let rec_path = scratch("record").join("rec.smesh");
        match ok_text(&["-p", &ts(pdir), "record", "--forks", "refs", "--verify", "-o", &ts(&rec_path)], Some(pdir), Threads::Inherit) {
            Ok(_) => record_rows(&mut rows, &read_text(&rec_path), &cli),
            Err(x) => {
                rows.fail(2, format!("smesh record --verify: {x}"));
                rows.fail(7, format!("smesh record --verify: {x}"));
            }
        }
    }

    // ---------------- the other front ends
    let mcp = timed!("MCP run", mcp_front("w3c_pin.smesh", "mcp", &PIN_FILES, "mcp_blocked.smrt"));
    let mcp_left = mcp_front("w3c_pin_leftout.smesh", "mcp_leftout", &[], "mcp_leftout.smrt");
    let inproc = timed!("in-process run", in_process("w3c_pin.smesh", "inproc", &PIN_FILES));
    let inproc_enc = in_process("w3c_pin_enclosed.smesh", "inproc_enc", &ENCLOSED_FILES);
    let py = python();
    let pyr = match &py {
        Ok(Ok(p)) => Some(timed!("Python driver", python_run(p))),
        Ok(Err(reason)) => {
            println!("SKIP P5/P6/P9 Python rows (wheel): {reason}");
            for row in [5, 6, 9] {
                rows.note(row, format!("Python skipped: {reason}"));
            }
            None
        }
        Err(x) => {
            for row in [5, 6, 9] {
                rows.fail(row, x.clone());
            }
            None
        }
    };

    // P5: MCP text, in-process note, Python ExportWarning.
    match &mcp_left {
        Ok(m) => {
            let first = m.export_text.lines().next().unwrap_or("");
            rows.check(5, first.starts_with("warning: smrt-region-names: ") && first.contains("cells are in no block"), || format!("MCP text does not lead with the warning: {}", m.export_text));
            rows.note(5, format!("MCP text starts: {first}"));
        }
        Err(x) => rows.fail(5, format!("MCP left-out session: {x}")),
    }
    match &mcp {
        Ok(m) => rows.check(4, !m.export_text.contains("warning:") && m.export_text.starts_with('{'), || format!("MCP text of the fully blocked export: {}", m.export_text)),
        Err(x) => rows.fail(4, format!("MCP pin session: {x}")),
    }
    let mut inproc_left_smrt = None;
    match in_process_server("w3c_pin_leftout.smesh", "inproc_leftout") {
        Ok((mut s, _, proj)) => match export_in_process(&mut s, &proj.join("l.smrt"), None, false) {
            Ok(r) => {
                rows.check(5, r.notes.len() == 1 && r.notes[0].is_warning() && r.notes[0].code == "smrt-region-names", || format!("in-process notes: {:?}", r.notes));
                inproc_left_smrt = digest(&proj.join("l.smrt")).ok();
            }
            Err(x) => rows.fail(5, x),
        },
        Err(x) => rows.fail(5, format!("in-process left-out: {x}")),
    }
    // P4 in process: a fully blocked export has no notes.
    match in_process_server("w3c_pin.smesh", "inproc_blocked") {
        Ok((mut s, _, proj)) => match export_in_process(&mut s, &proj.join("b.smrt"), None, false) {
            Ok(r) => rows.check(4, r.notes.is_empty(), || format!("in-process notes on the fully blocked pin: {:?}", r.notes)),
            Err(x) => rows.fail(4, x),
        },
        Err(x) => rows.fail(4, format!("in-process pin: {x}")),
    }
    // P6 in process: the protocol-level export of the warped case equals the CLI's.
    let mut inproc_warped: Option<(String, String)> = None;
    match in_process_server("w3c_warped.smesh", "inproc_warped") {
        Ok((mut s, _, proj)) => {
            let off = export_in_process(&mut s, &proj.join("w_off"), Some(ExportFormat::Openfoam), false);
            let on = export_in_process(&mut s, &proj.join("w_on"), Some(ExportFormat::Openfoam), true);
            match (off, on) {
                (Ok(off), Ok(on)) => {
                    rows.check(6, off.notes.iter().all(|n| n.code != "foam.warped_faces_split"), || format!("flag off: {:?}", off.notes));
                    let n = on.notes.iter().find(|n| n.code == "foam.warped_faces_split").map(|n| n.count);
                    rows.check(6, n.is_some_and(|n| n > 0), || format!("flag on has no foam.warped_faces_split note: {:?}", on.notes));
                    if let (Ok(a), Ok(b)) = (digest(&proj.join("w_off")), digest(&proj.join("w_on"))) {
                        inproc_warped = Some((a, b));
                    }
                }
                (a, b) => rows.fail(6, format!("in-process warped exports: {:?} {:?}", a.err(), b.err())),
            }
        }
        Err(x) => rows.fail(6, format!("in-process warped: {x}")),
    }
    if let (Some(c), Some(i)) = (&cli_warped, &inproc_warped) {
        rows.eq(6, "warped OpenFOAM case, CLI vs protocol (acceptance 6)", c, i);
    }

    // Python rows.
    if let Some(r) = &pyr {
        match r {
            Err(x) => {
                for row in [5, 6, 9] {
                    rows.fail(row, format!("Python driver: {x}"));
                }
            }
            Ok((lines, pdir)) => {
                for key in KEYS {
                    rows.eq(9, &format!("python {key} vs CLI"), &line_after(lines, key).unwrap_or_default(), &h(&cli, key).unwrap_or_default());
                }
                for k in ["cells", "shapes", "violations", "mesh_volume", "model_volume"] {
                    rows.eq(9, &format!("python {k} vs CLI"), &line_after(lines, k).unwrap_or_default(), &h(&cli, k).unwrap_or_default());
                }
                for f in PIN_FILES {
                    let kind = if f.ends_with("_foam") { "tree" } else { "file" };
                    rows.eq(9, &format!("python {f} vs CLI"), &line_after(lines, &format!("{kind} {f}")).unwrap_or_default(), cli.files.get(f).unwrap_or(&String::new()));
                }
                if let Ok(e) = &enc {
                    for key in KEYS {
                        rows.eq(9, &format!("python enclosed {key} vs CLI"), &line_after(lines, &format!("enclosed_{key}")).unwrap_or_default(), &h(e, key).unwrap_or_default());
                    }
                    rows.eq(9, "python enclosed.smrt vs CLI", &line_after(lines, "file enclosed.smrt").unwrap_or_default(), e.files.get("enclosed.smrt").unwrap_or(&String::new()));
                    rows.eq(9, "python py_enclosed.smrt vs CLI", &line_after(lines, "file py_enclosed.smrt").unwrap_or_default(), e.files.get("enclosed.smrt").unwrap_or(&String::new()));
                }
                // P4: the typed export raised no warning (the driver ran it under `-W error` semantics) and wrote the same bytes.
                rows.eq(4, "python Session.export(pin.smrt) bytes vs script's", &line_after(lines, "file py.smrt").unwrap_or_default(), cli.files.get("pin.smrt").unwrap_or(&String::new()));
                // P5: one ExportWarning, same file bytes as the other front ends.
                match line_after(lines, "leftout_warning") {
                    Ok(w) => {
                        rows.check(5, w.starts_with("smrt-region-names: ") && w.contains("cells are in no block"), || format!("python warning: {w}"));
                        rows.note(5, format!("Python ExportWarning: {w}"));
                    }
                    Err(x) => rows.fail(5, x),
                }
                rows.eq(5, "python left-out h_mesh", &line_after(lines, "leftout_h_mesh").unwrap_or_default(), &LEFTOUT_H_MESH.to_owned());
                let py_left = digest(&pdir.join("leftout.smrt")).ok();
                rows.eq(5, "left-out .smrt: python vs CLI", &py_left, &cli_left_smrt);
                rows.eq(5, "left-out .smrt: in-process vs CLI", &inproc_left_smrt, &cli_left_smrt);
                // P6: Python on/off against the CLI.
                rows.eq(6, "python pin OpenFOAM flag on vs off", &line_after(lines, "tree py_foam_on").unwrap_or_default(), &line_after(lines, "tree py_foam_off").unwrap_or_default());
                if let Some((off, on)) = &pin_foam {
                    rows.eq(6, "python pin OpenFOAM off vs CLI", &line_after(lines, "tree py_foam_off").unwrap_or_default(), off);
                    rows.eq(6, "python pin OpenFOAM on vs CLI", &line_after(lines, "tree py_foam_on").unwrap_or_default(), on);
                }
                rows.eq(6, "python warped h_mesh", &line_after(lines, "warped_h_mesh").unwrap_or_default(), &WARPED_H_MESH.to_owned());
                if let Some((off, on)) = &cli_warped {
                    rows.eq(6, "python warped OpenFOAM off vs CLI", &line_after(lines, "tree w_off").unwrap_or_default(), off);
                    rows.eq(6, "python warped OpenFOAM on vs CLI", &line_after(lines, "tree w_on").unwrap_or_default(), on);
                }
                let notes = line_after(lines, "warped_notes").unwrap_or_default();
                let (n_off, n_on) = notes.split_once(" on ").unwrap_or(("", ""));
                rows.check(6, !n_off.contains("warped") && n_on.contains("('foam.warped_faces_split', "), || format!("python warped notes: {notes}"));
                rows.check(6, lines.iter().any(|l| l == "gmsh_refused bad-request"), || "python: split_warped_faces on gmsh was not refused".into());
                rows.note(6, format!("Python: {notes}"));
            }
        }
    }

    // P8: the enclosed pin.
    match (&enc, &inproc_enc) {
        (Ok(f), Ok((_, run))) => {
            rows.eq(8, "enclosed cells", &h(f, "cells").unwrap_or_default(), &"733".to_owned());
            rows.eq(8, "enclosed shapes (all-tet)", &h(f, "shapes").unwrap_or_default(), &"{Tet: 733}".to_owned());
            rows.eq(8, "enclosed violations", &h(f, "violations").unwrap_or_default(), &"0".to_owned());
            rows.near(8, "enclosed volume", floats(&f.prints, "mesh_volume").unwrap_or(0.0), 1.26 * 1.26, 1e-9);
            rows.eq(8, "mesh.auto_tet warnings (one per volume)", &auto_tet_warnings(run), &2);
            match region_names(&base().join("enc").join("enclosed.smrt")) {
                Ok(n) => rows.eq(8, "enclosed.smrt regions are the blocks", &n, &vec!["fuel".to_owned(), "moderator".to_owned()]),
                Err(x) => rows.fail(8, format!("enclosed.smrt: {x}")),
            }
            for k in KEYS {
                rows.eq(8, &format!("enclosed {k} pinned"), &h(f, k).unwrap_or_default(), &ENCLOSED_HASHES[KEYS.iter().position(|x| *x == k).unwrap_or(0)].to_owned());
            }
        }
        (a, b) => rows.fail(8, format!("enclosed runs: {:?} {:?}", a.as_ref().err(), b.as_ref().err())),
    }

    // P9 and P10: hashes and files, front end against the default-thread CLI run.
    let mut fronts: Vec<(&Front, &Front, u8)> = Vec::new();
    if let Ok(f) = &cli_again {
        fronts.push((f, &cli, 10));
    }
    if let Ok(f) = &cli1 {
        fronts.push((f, &cli, 10));
    }
    if let Ok((f, _)) = &proj {
        fronts.push((f, &cli, 9));
    }
    if let Ok(m) = &mcp {
        fronts.push((&m.front, &cli, 9));
    }
    if let Ok((f, _)) = &inproc {
        fronts.push((f, &cli, 9));
    }
    if let (Ok(e), Ok(a)) = (&enc, &enc_again) {
        fronts.push((a, e, 10));
    }
    if let (Ok(e), Ok(a)) = (&enc, &enc1) {
        fronts.push((a, e, 10));
    }
    if let (Ok(e), Ok((a, _))) = (&enc, &inproc_enc) {
        fronts.push((a, e, 9));
    }
    for (label, r) in [("CLI again", cli_again.as_ref().err()), ("CLI 1 thread", cli1.as_ref().err()), ("MCP", mcp.as_ref().err()), ("in-process", inproc.as_ref().err()), ("enclosed CLI", enc.as_ref().err())] {
        if let Some(x) = r {
            rows.fail(if label.contains("again") || label.contains("1 thread") { 10 } else { 9 }, format!("{label}: {x}"));
        }
    }
    for (f, reference, row) in fronts {
        for k in KEYS {
            match (h(reference, k), h(f, k)) {
                (Ok(a), Ok(b)) => rows.eq(row, &format!("{k}: {} vs {}", f.name, reference.name), &b, &a),
                (a, b) => rows.fail(row, format!("{k} of {}: {:?} {:?}", f.name, a.err(), b.err())),
            }
        }
        rows.eq(row, &format!("files of {} vs {}", f.name, reference.name), &f.files, &reference.files);
        if row == 10 {
            rows.eq(row, &format!("all printed lines, {} vs {}", f.name, reference.name), &f.prints, &reference.prints);
        }
    }
    if let Ok(m) = &mcp {
        if let Some(text) = m.record["text"].as_str() {
            rows.check(7, text.contains("mesh.auto(assignments:"), || "MCP record_script lacks the explicit mesh.auto record".into());
            if let Ok(cli_rec) = std::fs::read_to_string(base().join("record").join("rec.smesh")) {
                rows.eq(7, "MCP record_script vs CLI smesh record (refs)", &text.to_owned(), &cli_rec);
            }
        } else {
            rows.fail(7, format!("MCP record_script: {}", m.record));
        }
    }

    // P11: the pinned hashes.
    for (label, f, want) in [("pin", Some(&cli), &PIN_HASHES), ("enclosed", enc.as_ref().ok(), &ENCLOSED_HASHES)] {
        if let Some(f) = f {
            for (i, k) in KEYS.iter().enumerate() {
                rows.eq(11, &format!("{label} {k}"), &h(f, k).unwrap_or_default(), &want[i].to_owned());
            }
        }
    }
    report(&rows, &timings, t0);
}

/// P2 and P7 on the text of `smesh record --forks refs`.
fn record_rows(rows: &mut Rows, rec: &str, cli: &Front) {
    let auto: Vec<&str> = rec.lines().filter(|l| l.starts_with("mesh.auto(")).collect();
    rows.eq(2, "one mesh.auto statement in the record", &auto.len(), &1);
    if let Some(l) = auto.first() {
        let n = l.matches("scheme:").count();
        // One assignment per volume (18) and per face the strategy had to decide.
        rows.check(2, l.starts_with("mesh.auto(assignments: [") && n >= 18, || format!("mesh.auto record has {n} explicit assignments: {}", &l[..l.len().min(160)]));
        rows.check(2, l.ends_with("strategy: \"default\")"), || "the recorded mesh.auto does not end with the strategy name".into());
        rows.note(2, format!("history: mesh.auto stored {n} explicit scheme assignments; a record with `assignments` applies them as given and calls no strategy"));
    }
    let tips = rec.lines().filter(|l| l.starts_with("assert session.hash() == ")).count();
    rows.check(7, tips >= 1, || format!("only {tips} tip checks in the recorded script"));
    let dir = scratch("replay");
    let path = dir.join("rec.smesh");
    if std::fs::write(&path, rec).is_err() {
        rows.fail(7, "cannot write the recorded script");
        return;
    }
    let want = one(&cli.prints, "h_mesh").unwrap_or_default();
    match ok_text(&["--memory", "run", &ts(&path), "--print-hash", "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit) {
        Ok(out) => {
            let last = out.lines().last().unwrap_or("").to_owned();
            rows.eq(7, "replayed head hash vs h_mesh", &last, &want);
            rows.eq(2, "replay (explicit assignments) reaches the same state hash", &last, &want);
            rows.note(7, format!("{tips} tip checks passed in the replay"));
        }
        Err(x) => {
            rows.fail(7, format!("replaying the recorded script: {x}"));
            rows.fail(2, format!("replaying the recorded script: {x}"));
        }
    }
    // No strategy is consulted on replay: an unregistered strategy name still reaches the same state.
    let ghost = dir.join("ghost.smesh");
    let text = rec.replace("strategy: \"default\")", "strategy: \"no_such_strategy\")");
    if text == rec || std::fs::write(&ghost, &text).is_err() {
        rows.fail(2, "cannot build the unregistered-strategy twin of the record");
        return;
    }
    match ok_text(&["--memory", "run", &ts(&ghost), "--print-hash", "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit) {
        Ok(out) => rows.eq(2, "replay under an unregistered strategy name reaches h_mesh", &out.lines().last().unwrap_or("").to_owned(), &want),
        Err(x) => rows.fail(2, format!("replay under an unregistered strategy name: {x}")),
    }
}

fn report(rows: &Rows, timings: &[(String, f64)], t0: Instant) {
    println!("---- W3c pin milestone report");
    for row in 1..=11u8 {
        let fails = rows.fails.get(&row).map_or(0, Vec::len);
        println!("row P{row}: {}", if fails == 0 { "PASS" } else { "FAIL" });
        for n in rows.notes.get(&row).into_iter().flatten() {
            println!("    note: {n}");
        }
        for f in rows.fails.get(&row).into_iter().flatten() {
            println!("    FAIL: {f}");
        }
    }
    println!("---- timings (record only)");
    for (k, v) in timings {
        println!("    {k}: {v:.2} s");
    }
    println!("    total: {:.2} s", t0.elapsed().as_secs_f64());
    assert!(rows.fails.values().all(Vec::is_empty), "milestone rows failed: {:?}", rows.fails);
}
