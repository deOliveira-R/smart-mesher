//! W3b integration milestone (plan `docs/design/w3b/plan.md` section 5.1).
//!
//! `tools/w3b_milestone.smesh` builds a pin cell and a 3x3 lattice of it with `model.copy`,
//! round-trips the model through STEP (`model.export`), runs a face-only `mesh.run` with
//! `mesh.plan`, lets `mesh.auto { strategy: "default" }` choose the schemes, meshes the lattice
//! and writes `.smsh`, `.msh`, `.h5m` and the `.smrt` handover image. This test runs it through
//! every front end and checks the rows below. Collect-all style like `milestone_w3a`: every row
//! is evaluated, and the test fails once at the end with the whole report. Run it as
//! `cargo test -p sm-cli --test milestone_w3b -- --nocapture` (and with `RAYON_NUM_THREADS=1`).
//!
//! | row | what |
//! |---|---|
//! | M1 | pin cell and 3x3 lattice by `model.copy` (script asserts: 5 and 45 volumes, volume sum) |
//! | M2 | `mesh.auto` stores explicit assignments; the recorded script replays them, no strategy |
//! | M3 | face-only `mesh.run` and `mesh.plan` (will_run / ran) on a side branch |
//! | M4 | `.h5m` (with graveyard), `.msh` ASCII and binary, `.smsh` |
//! | M5 | `model.export` STEP: re-import with equal volumes, state hash and node unchanged |
//! | M6 | the C driver through libsmrt: names and sets, chords, flux, understated majorant |
//! | M7 | `script.record --forks refs` and replay equal (also SERVER-HOST acceptance 2) |
//! | M8 | the same slice from the installed wheel, typed ops equal |
//! | M9 | CLI, MCP, in-process and Python state hashes and files equal |
//! | M10 | default threads and `RAYON_NUM_THREADS=1` equal |

mod common;
mod w3a_support;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::Instant;

use serde_json::{Value, json};
use sm_protocol::{Endpoint, Request};
use sm_server::{Server, standard_with};

use w3a_support::*;

const FILES: [&str; 5] = ["milestone.step", "milestone.smsh", "milestone.msh", "milestone.h5m", "milestone.smrt"];
/// The state hash every front end must reach (also pinned by the Python driver's typed replay).
const KEYS: [&str; 3] = ["h_model", "h_auto", "h_mesh"];

fn repo() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().expect("repo root")
}

fn script_path() -> PathBuf {
    repo().join("tools/w3b_milestone.smesh")
}

fn script_text() -> String {
    read_text(&script_path())
}

fn scratch(name: &str) -> PathBuf {
    let d = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("milestone_w3b-{}", std::process::id())).join(name);
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("scratch dir");
    d
}

/// An existing scratch directory (no wipe).
fn existing(name: &str) -> PathBuf {
    Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("milestone_w3b-{}", std::process::id())).join(name)
}

fn ts(p: &Path) -> String {
    p.to_string_lossy().into_owned()
}

type Prints = BTreeMap<String, Vec<String>>;

/// `key value...` lines of a script's printed output.
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

/// sha256 of each milestone file in `dir`.
fn files_of(dir: &Path) -> R<BTreeMap<String, String>> {
    FILES.iter().map(|f| Ok(((*f).to_owned(), sha_hex(&read_bytes(&dir.join(f))?)))).collect()
}

/// What one front end produced.
struct Front {
    name: &'static str,
    prints: Prints,
    files: BTreeMap<String, String>,
    secs: f64,
}

impl Front {
    fn new(name: &'static str, output: &str, dir: &Path, t: Instant) -> R<Self> {
        Ok(Self { name, prints: prints_of(output), files: files_of(dir)?, secs: t.elapsed().as_secs_f64() })
    }
}

// ------------------------------------------------------------------ front ends

fn cli_memory(name: &'static str, threads: Threads) -> R<Front> {
    let dir = scratch(name);
    let t = Instant::now();
    let args = ["--memory", "run", &ts(&script_path()), "--author", AUTHOR, "--fixed-time", TIME];
    let out = ok_text(&args, Some(&dir), threads)?;
    Front::new(name, &out, &dir, t)
}

/// The project run: files land in the project root, and the timeline is on disk for `record`.
fn cli_project() -> R<(Front, PathBuf)> {
    let dir = scratch("cli_project");
    let proj = dir.join("proj");
    ok_text(&["new", &ts(&proj)], Some(&dir), Threads::Inherit)?;
    let t = Instant::now();
    let out = ok_text(&["-p", &ts(&proj), "run", &ts(&script_path()), "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit)?;
    Ok((Front::new("cli_project", &out, &proj, t)?, proj))
}

fn mcp_front() -> R<(Front, Value)> {
    let dir = scratch("mcp");
    let t = Instant::now();
    let mut lines = mcp_init();
    lines.push(tool_call(2, "run_script", json!({"text": script_text()})));
    lines.push(tool_call(3, "session_hash", json!({})));
    lines.push(tool_call(4, "record_script", json!({"forks": "refs"})));
    let replies = mcp_session(&PIN, &lines, Threads::Inherit, Some(&dir))?;
    let run = tool_ok(&replies, 2)?;
    let hash = tool_ok(&replies, 3)?;
    let rec = tool_ok(&replies, 4)?;
    let out = run["output"].as_str().ok_or_else(|| format!("run_script has no output: {run}"))?;
    let f = Front::new("mcp", out, &dir, t)?;
    let state = hash["state"].as_str().ok_or_else(|| format!("session_hash: {hash}"))?;
    if state != one(&f.prints, "h_mesh")? {
        return Err(format!("MCP session_hash {state} != printed h_mesh"));
    }
    Ok((f, rec))
}

fn in_process() -> R<Front> {
    let dir = scratch("inproc");
    let proj = dir.join("proj");
    let t = Instant::now();
    let mut s = Server::create(standard_with(Some(AUTHOR.into()), Some(TIME.into())), &proj, "w3b").map_err(e)?;
    let v = s
        .call(Request::ScriptRun { text: script_text(), path: None, includes: Default::default() })
        .map_err(|x| format!("script.run: {x} {}", x.data["rendered"].as_str().unwrap_or("")))?;
    let out = v["output"].as_str().ok_or_else(|| format!("script.run has no output: {v}"))?;
    let f = Front::new("in_process", out, &proj, t)?;
    let h = s.call(Request::Hash { at: None }).map_err(e)?;
    if h["state"].as_str() != Some(one(&f.prints, "h_mesh")?.as_str()) {
        return Err(format!("in-process session.hash {} != printed h_mesh", h["state"]));
    }
    Ok(f)
}

/// The venv python with the wheel installed, or the reason to skip (maturin unavailable).
fn python() -> R<Result<PathBuf, String>> {
    if let Some(p) = std::env::var_os("W3B_PYTHON") {
        return Ok(Ok(PathBuf::from(p)));
    }
    let venv = Path::new(env!("CARGO_TARGET_TMPDIR")).join("w3b-venv");
    let o = Command::new("bash")
        .arg(repo().join("tools/w3b_python.sh"))
        .arg(&venv)
        .env("CARGO_TARGET_DIR", target_dir())
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

fn target_dir() -> PathBuf {
    Path::new(env!("CARGO_TARGET_TMPDIR")).parent().expect("target dir").to_path_buf()
}

fn python_front(py: &Path) -> R<Front> {
    let dir = scratch("python");
    let t = Instant::now();
    let o = Command::new(py)
        .arg("-I")
        .arg(repo().join("tools/w3b_driver.py"))
        .arg(script_path())
        .arg(&dir)
        .output()
        .map_err(|x| format!("run python driver: {x}"))?;
    let out = String::from_utf8_lossy(&o.stdout).into_owned();
    if o.status.code() != Some(0) || !out.lines().any(|l| l == "OK") {
        return Err(format!("python driver exited {:?}:\n{out}\n{}", o.status.code(), String::from_utf8_lossy(&o.stderr)));
    }
    let f = Front::new("python", &out, &dir, t)?;
    // The driver prints its own file digests; they must be the ones on disk.
    for l in out.lines().filter(|l| l.starts_with("file ")) {
        let mut it = l.split_whitespace().skip(1);
        let (name, sha) = (it.next().unwrap_or(""), it.next().unwrap_or(""));
        if f.files.get(name).map(String::as_str) != Some(sha) {
            return Err(format!("driver digest of {name} differs from the file"));
        }
    }
    if !out.lines().any(|l| l.starts_with("typed ")) {
        return Err("python driver did not run the typed replay".into());
    }
    Ok(f)
}

// ------------------------------------------------------------------ the C driver

/// `libsm_rt_capi` built in its own target directory (the test's cargo may hold a different
/// feature set), and the C driver compiled against it.
fn build_c_driver(dir: &Path) -> R<PathBuf> {
    let capi_target = target_dir().join("w3b-capi");
    let profile = if cfg!(debug_assertions) { "debug" } else { "release" };
    let mut cargo = Command::new(std::env::var_os("CARGO").unwrap_or_else(|| "cargo".into()));
    cargo.args(["build", "-p", "sm-rt-capi", "--target-dir"]).arg(&capi_target).current_dir(repo());
    if !cfg!(debug_assertions) {
        cargo.arg("--release");
    }
    let o = cargo.output().map_err(|x| format!("cargo build sm-rt-capi: {x}"))?;
    if !o.status.success() {
        return Err(format!("cargo build -p sm-rt-capi failed: {}", String::from_utf8_lossy(&o.stderr)));
    }
    let libdir = capi_target.join(profile);
    let lib = libdir.join(format!("{}sm_rt_capi{}", std::env::consts::DLL_PREFIX, std::env::consts::DLL_SUFFIX));
    if !lib.exists() {
        return Err(format!("{} not found", lib.display()));
    }
    let exe = dir.join("w3b_driver");
    let cc = std::env::var("CC").unwrap_or_else(|_| "cc".into());
    let o = Command::new(&cc)
        .args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-D_GNU_SOURCE", "-ffp-contract=off", "-O1"])
        .arg("-I")
        .arg(repo().join("crates/sm-rt-capi/include"))
        .arg(repo().join("tools/w3b_driver.c"))
        .arg("-o")
        .arg(&exe)
        .arg(&lib)
        .arg(format!("-Wl,-rpath,{}", libdir.display()))
        .arg("-lm")
        .output()
        .map_err(|x| format!("run {cc}: {x}"))?;
    if !o.status.success() {
        return Err(format!("compiling tools/w3b_driver.c failed: {}", String::from_utf8_lossy(&o.stderr)));
    }
    Ok(exe)
}

/// The `ROW id PASS|FAIL detail` lines of the driver (and the whole text).
/// One `ROW` line of the C driver: id, passed, detail.
type DriverRow = (String, bool, String);

fn run_c_driver(smrt: &Path) -> R<(Vec<DriverRow>, String)> {
    let dir = scratch("cdriver");
    let exe = build_c_driver(&dir)?;
    let o = Command::new(&exe).arg(smrt).output().map_err(|x| format!("run w3b_driver: {x}"))?;
    let out = String::from_utf8_lossy(&o.stdout).into_owned();
    let rows = out
        .lines()
        .filter_map(|l| l.strip_prefix("ROW "))
        .map(|l| {
            let mut it = l.splitn(3, ' ');
            let (id, verdict, rest) = (it.next().unwrap_or(""), it.next().unwrap_or(""), it.next().unwrap_or(""));
            (id.to_owned(), verdict == "PASS", rest.to_owned())
        })
        .collect::<Vec<_>>();
    if o.status.code() != Some(0) && rows.iter().all(|r| r.1) {
        return Err(format!("w3b_driver exited {:?}: {out}\n{}", o.status.code(), String::from_utf8_lossy(&o.stderr)));
    }
    Ok((rows, out))
}

// ------------------------------------------------------------------ the milestone

fn floats(p: &Prints, k: &str) -> R<Vec<f64>> {
    p.get(k).ok_or_else(|| format!("no `{k}` lines"))?.iter().map(|s| s.parse::<f64>().map_err(|x| format!("{k}: {s}: {x}"))).collect()
}

#[test]
fn milestone_w3b() {
    let _root = common::Scratch::guard(Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("milestone_w3b-{}", std::process::id())));
    let mut rows = Rows::default();
    let t0 = Instant::now();
    let mut timings: Vec<(String, f64)> = Vec::new();

    // ---------------- the CLI runs: default threads, one thread, and the project run
    let cli = cli_memory("cli", Threads::Default);
    let cli1 = cli_memory("cli_1thread", Threads::One);
    let proj = cli_project();
    for (label, r) in [("CLI", cli.as_ref().err()), ("CLI 1 thread", cli1.as_ref().err()), ("CLI project", proj.as_ref().err())] {
        if let Some(x) = r {
            for row in [1, 3, 5] {
                rows.fail(row, format!("{label} run failed: {x}"));
            }
        }
    }
    let Ok(cli) = cli else {
        report(&rows, &timings, t0);
        return;
    };
    timings.push(("CLI run (default threads)".into(), cli.secs));
    let h = |f: &Front, k: &str| one(&f.prints, k);

    // M1, M3, M5: the script's own assertions passed (exit 0); read back what it printed.
    rows.note(1, "script asserts: 5 volumes per pin cell, 45 in the lattice, 9 fuel + 36 moderator, volume 9 x 1.26^2");
    match (floats(&cli.prints, "model_volume"), one(&cli.prints, "cells")) {
        (Ok(v), Ok(c)) => {
            rows.near(1, "lattice volume", v[0], 9.0 * 1.26 * 1.26, 1e-12);
            rows.eq(1, "hex cells", &c, &"675".to_owned());
        }
        (a, b) => rows.fail(1, format!("model_volume/cells: {a:?} {b:?}")),
    }
    match one(&cli.prints, "face_cells").map(|s| s.parse::<u64>()) {
        Ok(Ok(n)) => {
            rows.check(3, n > 1, || format!("face-only run made {n} cells"));
            rows.note(3, format!("face-only mesh.run on one face: {n} quads; mesh.plan will_run and ran asserted in the script"));
        }
        other => rows.fail(3, format!("face_cells: {other:?}")),
    }
    // STEP: volumes of the re-imported model equal, sorted, relative 1e-9.
    match (floats(&cli.prints, "vol_a"), floats(&cli.prints, "vol_b")) {
        (Ok(mut a), Ok(mut b)) => {
            a.sort_by(f64::total_cmp);
            b.sort_by(f64::total_cmp);
            rows.eq(5, "volume count", &a.len(), &b.len());
            rows.eq(5, "volume count is 45", &a.len(), &45);
            let worst = a.iter().zip(&b).map(|(x, y)| (x - y).abs() / x.abs().max(1e-300)).fold(0.0, f64::max);
            rows.check(5, worst < 1e-9, || format!("re-imported volumes differ by {worst:e} (relative)"));
            rows.note(5, format!("{} volumes, worst relative difference {worst:.2e}; script asserted the state hash and node unchanged by model.export", a.len()));
        }
        (a, b) => rows.fail(5, format!("vol_a/vol_b: {a:?} {b:?}")),
    }
    match one(&cli.prints, "step_bytes").map(|s| s.parse::<usize>()) {
        Ok(Ok(n)) => {
            let on_disk = std::fs::metadata(existing("cli").join("milestone.step")).map(|m| m.len() as usize).unwrap_or(0);
            rows.eq(5, "STEP bytes reported vs on disk", &n, &on_disk);
        }
        other => rows.fail(5, format!("step_bytes: {other:?}")),
    }
    rows.eq(1, "script ended on main", &one(&cli.prints, "head").unwrap_or_default(), &"main".to_owned());

    // M4: the exported files, and the graveyard / binary variants through the CLI options.
    let dir = existing("cli");
    match (read_bytes(&dir.join("milestone.msh")), read_bytes(&dir.join("milestone.h5m"))) {
        (Ok(msh), Ok(h5m)) => {
            rows.check(4, msh.starts_with(b"$MeshFormat\n4.1 0 8\n"), || "milestone.msh is not ASCII msh 4.1".into());
            rows.check(4, h5m.starts_with(b"\x89HDF\r\n\x1a\n"), || "milestone.h5m has no HDF5 signature".into());
            rows.check(4, contains(&h5m, b"mat:fuel") && contains(&h5m, b"mat:water"), || "h5m lacks mat:<set> names".into());
            rows.check(4, !contains(&h5m, b"mat:Graveyard"), || "default h5m must not carry the graveyard".into());
        }
        (a, b) => rows.fail(4, format!("exports unreadable: {:?} {:?}", a.err(), b.err())),
    }
    match read_bytes(&dir.join("milestone.smsh")).and_then(|b| sm_meshio::SmshFile::parse(&b).map_err(e)?.decode().map_err(e).map(|d| d.cell_ids.len())) {
        Ok(n) => rows.eq(4, ".smsh decodes with 675 cells", &n, &675),
        Err(x) => rows.fail(4, format!(".smsh: {x}")),
    }
    let proj_result = proj.as_ref().ok();
    if let Some((pf, pdir)) = proj_result {
        timings.push(("CLI project run".into(), pf.secs));
        let outdir = scratch("cli_options");
        let export = |file: &str, fmt: &str, opt: &str| ok_text(&["-p", &ts(pdir), "export", &ts(&outdir.join(file)), "--format", fmt, "--option", opt], Some(&outdir), Threads::Inherit);
        for (file, fmt, opt) in [("gy.h5m", "dagmc", "graveyard=true"), ("bin.msh", "gmsh", "binary=true")] {
            if let Err(x) = export(file, fmt, opt) {
                rows.fail(4, format!("export {file}: {x}"));
            }
        }
        match (read_bytes(&outdir.join("gy.h5m")), read_bytes(&dir.join("milestone.h5m"))) {
            (Ok(gy), Ok(plain)) => {
                rows.check(4, gy.starts_with(b"\x89HDF\r\n\x1a\n") && contains(&gy, b"mat:Graveyard"), || "graveyard h5m lacks mat:Graveyard".into());
                rows.check(4, gy.len() > plain.len(), || format!("graveyard h5m ({}) is not larger than the plain one ({})", gy.len(), plain.len()));
                rows.note(4, format!("h5m {} bytes, with graveyard {} bytes", plain.len(), gy.len()));
            }
            (a, b) => rows.fail(4, format!("graveyard export: {:?} {:?}", a.err(), b.err())),
        }
        match (read_bytes(&outdir.join("bin.msh")), read_bytes(&dir.join("milestone.msh"))) {
            (Ok(bin), Ok(ascii)) => {
                rows.check(4, bin.starts_with(b"$MeshFormat\n4.1 1 8\n"), || "binary msh header is not `4.1 1 8`".into());
                rows.note(4, format!("msh ASCII {} bytes, binary {} bytes", ascii.len(), bin.len()));
            }
            (a, b) => rows.fail(4, format!("binary msh: {:?} {:?}", a.err(), b.err())),
        }
        // M2, M7: the recorded script carries explicit assignments and replays to the same hashes.
        let rec_path = scratch("record").join("rec.smesh");
        match ok_text(&["-p", &ts(pdir), "record", "--forks", "refs", "--verify", "-o", &ts(&rec_path)], Some(&outdir), Threads::Inherit) {
            Ok(_) => {
                let rec = read_text(&rec_path);
                record_rows(&mut rows, &rec, &cli, &mut timings);
            }
            Err(x) => {
                rows.fail(2, format!("smesh record --verify: {x}"));
                rows.fail(7, format!("smesh record --verify: {x}"));
            }
        }
    }

    // ---------------- the other front ends
    let mcp = mcp_front();
    let inproc = in_process();
    let py = python();
    let pyf = match &py {
        Ok(Ok(p)) => Some(python_front(p)),
        Ok(Err(reason)) => {
            println!("SKIP M8 (Python wheel): {reason}");
            rows.note(8, format!("skipped: {reason}"));
            None
        }
        Err(x) => {
            rows.fail(8, x.clone());
            None
        }
    };

    // M9 and M10: hashes and files, front end against the default-thread CLI run.
    let mut fronts: Vec<&Front> = Vec::new();
    if let Ok(f) = &cli1 {
        fronts.push(f);
    }
    if let Some((f, _)) = proj_result {
        fronts.push(f);
    }
    let mcp_ok = mcp.as_ref().ok();
    if let Some((f, _)) = mcp_ok {
        fronts.push(f);
    }
    if let Ok(f) = &inproc {
        fronts.push(f);
    }
    if let Some(Ok(f)) = &pyf {
        fronts.push(f);
    }
    for (what, r) in [("MCP", mcp.as_ref().err()), ("in-process", inproc.as_ref().err()), ("Python", pyf.as_ref().and_then(|r| r.as_ref().err()))] {
        if let Some(x) = r {
            rows.fail(9, format!("{what} front end: {x}"));
            if what == "Python" {
                rows.fail(8, x.to_string());
            }
        }
    }
    if let Err(x) = &cli1 {
        rows.fail(10, format!("CLI with RAYON_NUM_THREADS=1: {x}"));
    }
    for f in fronts {
        let row = if f.name == "cli_1thread" { 10 } else { 9 };
        for k in KEYS {
            match (h(&cli, k), h(f, k)) {
                (Ok(a), Ok(b)) => rows.eq(row, &format!("{k}: {} vs CLI", f.name), &b, &a),
                (a, b) => rows.fail(row, format!("{k} of {}: {:?} {:?}", f.name, a.err(), b.err())),
            }
        }
        rows.eq(row, &format!("files of {} vs CLI", f.name), &f.files, &cli.files);
        if f.name == "cli_1thread" {
            rows.eq(row, "all printed lines, 1 thread vs default", &f.prints, &cli.prints);
        }
        timings.push((format!("{} run", f.name), f.secs));
        if f.name == "python" {
            rows.note(8, format!("wheel ran the script; hashes {} {} {}; files equal", f.prints["h_model"][0], f.prints["h_auto"][0], f.prints["h_mesh"][0]));
        }
    }
    if let Some((_, rec)) = mcp_ok {
        // MCP `record_script` gives the same text as the CLI's `smesh record` of the same history
        // (the CLI project run has the same timeline).
        if let Some(text) = rec["text"].as_str() {
            rows.check(7, text.contains("mesh.auto(assignments:"), || "MCP record_script lacks the explicit mesh.auto record".into());
        } else {
            rows.fail(7, format!("MCP record_script: {rec}"));
        }
    }
    if let (Some((_, pdir)), Some((_, rec))) = (proj_result, mcp_ok) {
        let rec_path = existing("record").join("rec.smesh");
        if let (Ok(cli_rec), Some(mcp_rec)) = (std::fs::read_to_string(&rec_path), rec["text"].as_str()) {
            rows.eq(7, "MCP record_script vs CLI smesh record (refs)", &mcp_rec.to_owned(), &cli_rec);
        }
        let _ = pdir;
    }

    // M6: the C driver on the file the CLI's session.export wrote.
    let t = Instant::now();
    match run_c_driver(&dir.join("milestone.smrt")) {
        Ok((crows, out)) => {
            timings.push(("C driver (build + run)".into(), t.elapsed().as_secs_f64()));
            for id in ["names", "i.surface", "i.delta", "i.no_lost", "ii.flux_per_region", "iii.understated"] {
                match crows.iter().find(|r| r.0 == id) {
                    Some((_, true, detail)) => rows.note(6, format!("{id}: {detail}")),
                    Some((_, false, detail)) => rows.fail(6, format!("{id} FAILED: {detail}")),
                    None => rows.fail(6, format!("the driver printed no row `{id}`")),
                }
            }
            for l in out.lines().filter(|l| l.starts_with("INFO ")) {
                rows.note(6, l.to_owned());
            }
            // The same on the .smrt of the one-thread run: identical bytes imply identical results.
            rows.eq(6, "smrt of the 1-thread CLI run", &cli1.as_ref().map(|f| f.files["milestone.smrt"].clone()).unwrap_or_default(), &cli.files["milestone.smrt"]);
        }
        Err(x) => rows.fail(6, format!("C driver: {x}")),
    }

    report(&rows, &timings, t0);
}

fn contains(hay: &[u8], needle: &[u8]) -> bool {
    hay.windows(needle.len()).any(|w| w == needle)
}

/// M2 and M7 on the text of `smesh record --forks refs`.
fn record_rows(rows: &mut Rows, rec: &str, cli: &Front, timings: &mut Vec<(String, f64)>) {
    let auto: Vec<&str> = rec.lines().filter(|l| l.starts_with("mesh.auto(")).collect();
    rows.eq(2, "one mesh.auto statement in the record", &auto.len(), &1);
    if let Some(l) = auto.first() {
        let n = l.matches("scheme:").count();
        // One assignment per volume (45) and per face the strategy had to decide (18 interface faces).
        rows.check(2, l.starts_with("mesh.auto(assignments: [") && n >= 45, || format!("mesh.auto record has {n} explicit assignments: {}", &l[..l.len().min(160)]));
        rows.check(2, l.ends_with("strategy: \"default\")"), || "the recorded mesh.auto does not end with the strategy name".into());
        let sweeps = l.matches("scheme: \"sweep\"").count();
        rows.note(2, format!("history: mesh.auto stored {n} explicit scheme assignments ({sweeps} sweep); a record with `assignments` applies them as given and calls no strategy"));
    }
    // The tips carry state-hash asserts; replaying the script in a fresh session passes them.
    let tips = rec.lines().filter(|l| l.starts_with("assert session.hash() == ")).count();
    rows.check(7, tips >= 3, || format!("only {tips} tip checks in the recorded script"));
    let dir = scratch("replay");
    let path = dir.join("rec.smesh");
    if std::fs::write(&path, rec).is_err() {
        rows.fail(7, "cannot write the recorded script");
        return;
    }
    let t = Instant::now();
    match ok_text(&["--memory", "run", &ts(&path), "--print-hash", "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit) {
        Ok(out) => {
            timings.push(("replay of the recorded script".into(), t.elapsed().as_secs_f64()));
            let last = out.lines().last().unwrap_or("").to_owned();
            rows.eq(7, "replayed head hash vs the milestone's h_mesh", &last, &one(&cli.prints, "h_mesh").unwrap_or_default());
            rows.eq(2, "replay (explicit assignments) reaches the same state hash", &last, &one(&cli.prints, "h_mesh").unwrap_or_default());
            rows.note(7, format!("{tips} tip checks passed in the replay; verify: replay reproduces the hashes"));
        }
        Err(x) => {
            rows.fail(7, format!("replaying the recorded script: {x}"));
            rows.fail(2, format!("replaying the recorded script: {x}"));
        }
    }
    // No strategy is consulted on replay: the same record under a strategy name nobody registered
    // still applies its assignments and reaches the same state.
    let ghost = dir.join("ghost.smesh");
    let text = rec.replace("strategy: \"default\")", "strategy: \"no_such_strategy\")");
    if text == rec || std::fs::write(&ghost, &text).is_err() {
        rows.fail(2, "cannot build the unregistered-strategy twin of the record");
        return;
    }
    match ok_text(&["--memory", "run", &ts(&ghost), "--print-hash", "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit) {
        Ok(out) => {
            rows.eq(2, "replay under an unregistered strategy name reaches h_mesh", &out.lines().last().unwrap_or("").to_owned(), &one(&cli.prints, "h_mesh").unwrap_or_default());
            rows.note(2, "replay with strategy renamed to `no_such_strategy`: same state hash, so no strategy ran");
        }
        Err(x) => rows.fail(2, format!("replay under an unregistered strategy name: {x}")),
    }
}

fn report(rows: &Rows, timings: &[(String, f64)], t0: Instant) {
    println!("---- W3b milestone report");
    for row in 1..=10u8 {
        let fails = rows.fails.get(&row).map_or(0, Vec::len);
        println!("row M{row}: {}", if fails == 0 { "PASS" } else { "FAIL" });
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

// ------------------------------------------------------------------ SUBMAP version question

/// W3b-2 SUBMAP changed the output of `submap` on periodic faces and on graded neighbours (cut
/// fractions) without a `mesh.run` or mesher service version bump; twelve ungraded fixtures kept
/// their hashes. INT decision (W3b-INT, 2026-11-05): no bump. The graded case is pinned here
/// instead: an L-slab (sweep with a `submap` source face and `pave`/`trimesh` neighbours) whose
/// source loop has one edge refined to 0.2 against 0.5 elsewhere, so the submap sees unequal
/// interval counts along the loop. Blessed on 2026-11-05 at the integration tip; a change of
/// these hashes means the submap output moved again and needs a version-bump ruling.
const SUBMAP_GRADED: &str = "let a = model.create_brick(x: 2.0, y: 2.0, z: 1.0, center: [1.0, 1.0, 0.5])\n\
    let t = model.create_brick(x: 2.0, y: 2.0, z: 3.0, center: [2.0, 2.0, 0.5])\n\
    let r = model.subtract(targets: a.body, tools: t.body)\n\
    mesh.size(targets: r.volumes, size: 0.5)\n\
    mesh.size(targets: Edge#9, size: 0.2)\n\
    mesh.auto(strategy: \"default\")\n\
    let m = mesh.run(targets: r.volumes)\n\
    print \"mesh\", m.mesh[0]\n";
const SUBMAP_GRADED_TOPOLOGY_HASH: &str = "c99bcaa8d93e9a5e";
const SUBMAP_GRADED_GEOMETRY_HASH: &str = "43ecc0a4b59b3715";
const SUBMAP_GRADED_STATE_HASH: &str = "7b71fd6dd5c3dd7136a6a1b78a40c8fa3f851877e9f069a73f164f731b0c0b1f";

fn graded_submap_run() -> R<(Value, String, usize)> {
    let mut s = Server::new(standard_with(Some(AUTHOR.into()), Some(TIME.into()))).map_err(e)?;
    let run = s
        .call(Request::ScriptRun { text: SUBMAP_GRADED.into(), path: None, includes: Default::default() })
        .map_err(|x| format!("script.run: {x} {}", x.data["rendered"].as_str().unwrap_or("")))?;
    let mesh = prints_of(run["output"].as_str().unwrap_or(""));
    let id = one(&mesh, "mesh")?;
    let info = s.call(Request::Query { name: "mesh.info".into(), input: json!({"mesh": id}), at: None }).map_err(e)?;
    let plan = s.call(Request::Query { name: "mesh.plan".into(), input: json!({"mesh": id}), at: None }).map_err(e)?;
    let submaps = plan["entities"].as_array().map_or(0, |v| v.iter().filter(|x| x["scheme"] == "submap").count());
    let h = s.call(Request::Hash { at: None }).map_err(e)?;
    Ok((info, h["state"].as_str().unwrap_or("").to_owned(), submaps))
}

#[test]
fn submap_graded_input_is_pinned() {
    let (info, state, submaps) = graded_submap_run().expect("graded submap run");
    assert!(submaps >= 1, "the fixture must exercise submap, plan had {submaps} submap rows");
    assert_eq!(info["cells"], 36);
    assert!((info["volume"].as_f64().expect("volume") - 3.0).abs() < 1e-9);
    assert_eq!(info["topology_hash"], SUBMAP_GRADED_TOPOLOGY_HASH, "submap output on a graded input moved");
    assert_eq!(info["geometry_hash"], SUBMAP_GRADED_GEOMETRY_HASH, "submap output on a graded input moved");
    assert_eq!(state, SUBMAP_GRADED_STATE_HASH, "state hash of the graded submap fixture moved");
    // One thread gives the same bytes.
    let pool = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    let (info1, state1, _) = pool.install(|| graded_submap_run().expect("1 thread"));
    assert_eq!((info1["topology_hash"].clone(), info1["geometry_hash"].clone(), state1), (info["topology_hash"].clone(), info["geometry_hash"].clone(), state));
}

// ------------------------------------------------------------------ viewer live smoke (plan 5.3)

#[cfg(feature = "ws")]
mod viewer {
    use super::*;
    use std::net::{TcpListener, TcpStream};
    use std::process::{Child, Stdio};
    use std::time::Duration;
    use tungstenite::client::IntoClientRequest;
    use tungstenite::http::HeaderValue;

    /// A running `smesh serve --ws`, killed on drop.
    struct Serve(Child);

    impl Drop for Serve {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }

    fn free_addr() -> String {
        TcpListener::bind("127.0.0.1:0").expect("bind").local_addr().expect("addr").to_string()
    }

    fn connect_retry(addr: &str) -> TcpStream {
        for _ in 0..400 {
            if let Ok(s) = TcpStream::connect(addr) {
                return s;
            }
            std::thread::sleep(Duration::from_millis(25));
        }
        panic!("smesh serve did not start on {addr}");
    }

    /// The WebSocket upgrade with an optional `Origin`: the reply or the HTTP status that refused it.
    fn upgrade(addr: &str, origin: Option<&str>) -> Result<tungstenite::WebSocket<TcpStream>, u16> {
        let mut req = format!("ws://{addr}").into_client_request().expect("request");
        if let Some(o) = origin {
            req.headers_mut().insert("Origin", HeaderValue::from_str(o).expect("origin"));
        }
        match tungstenite::client(req, connect_retry(addr)) {
            Ok((ws, _)) => Ok(ws),
            Err(tungstenite::HandshakeError::Failure(tungstenite::Error::Http(resp))) => Err(resp.status().as_u16()),
            Err(x) => panic!("upgrade failed unexpectedly: {x}"),
        }
    }

    fn rpc(ws: &mut tungstenite::WebSocket<TcpStream>, id: u64, method: &str, params: Value) -> Value {
        ws.send(tungstenite::Message::text(json!({"jsonrpc": "2.0", "id": id, "method": method, "params": params}).to_string())).expect("send");
        loop {
            if let tungstenite::Message::Text(t) = ws.read().expect("read") {
                let v: Value = serde_json::from_str(t.as_str()).expect("json");
                assert_eq!(v["id"], id, "{v}");
                assert!(v.get("error").is_none_or(Value::is_null), "{method}: {v}");
                return v["result"].clone();
            }
        }
    }

    fn tool(name: &str) -> Option<String> {
        Command::new(name).arg("--version").output().ok().filter(|o| o.status.success()).map(|o| String::from_utf8_lossy(&o.stdout).trim().to_owned())
    }

    /// A directory with `node_modules/playwright-core`, installed on demand; the reason when it cannot be.
    fn playwright_dir() -> Result<PathBuf, String> {
        if let Some(d) = std::env::var_os("W3B_PLAYWRIGHT_DIR") {
            return Ok(PathBuf::from(d));
        }
        let dir = target_dir().join("w3b-viewer");
        if dir.join("node_modules/playwright-core").exists() {
            return Ok(dir);
        }
        std::fs::create_dir_all(&dir).map_err(e)?;
        let o = Command::new("npm")
            .args(["install", "--prefix"])
            .arg(&dir)
            .args(["playwright-core@1.49.1", "--no-audit", "--no-fund", "--no-package-lock", "--ignore-scripts"])
            .output()
            .map_err(|x| format!("npm: {x}"))?;
        if o.status.success() && dir.join("node_modules/playwright-core").exists() {
            Ok(dir)
        } else {
            Err(format!("npm install playwright-core failed: {}", String::from_utf8_lossy(&o.stderr).lines().last().unwrap_or("")))
        }
    }

    /// Row M11: the viewer against a live `smesh serve --ws`. A foreign `Origin` is refused (403)
    /// and the served address is accepted, both with and without an `Origin` header; the page
    /// itself runs in Chromium when playwright-core and a Chromium or Chrome binary exist, else the
    /// Node protocol path runs alone and the skip is printed.
    #[test]
    fn viewer_live_smoke() {
        let dir = common::Scratch::new(&format!("milestone_w3b-{}-viewer", std::process::id()));
        let proj = dir.join("proj");
        ok_text(&["new", &ts(&proj)], Some(&dir), Threads::Inherit).expect("new");
        let out = ok_text(&["-p", &ts(&proj), "run", &ts(&script_path()), "--author", AUTHOR, "--fixed-time", TIME], Some(&dir), Threads::Inherit).expect("run");
        let h_mesh = one(&prints_of(&out), "h_mesh").expect("h_mesh");

        let addr = free_addr();
        let child = Command::new(env!("CARGO_BIN_EXE_smesh"))
            .args(["-p", &ts(&proj), "serve", "--ws", &addr])
            .stdin(Stdio::null())
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .spawn()
            .expect("spawn smesh serve --ws");
        let _serve = Serve(child);
        connect_retry(&addr);

        // The Origin rule.
        let foreign = upgrade(&addr, Some("http://evil.example"));
        assert_eq!(foreign.err(), Some(403), "a foreign Origin must be refused with 403");
        let (host, port) = addr.rsplit_once(':').expect("host:port");
        let other = port.parse::<u16>().expect("port") - 1;
        assert_eq!(upgrade(&addr, Some(&format!("http://{host}:{other}"))).err(), Some(403), "same host, other port is foreign");
        let mut same = upgrade(&addr, Some(&format!("http://{addr}"))).expect("the served origin is accepted");
        assert_eq!(rpc(&mut same, 1, "session.hash", json!({}))["state"], h_mesh.as_str());
        let mut none = upgrade(&addr, None).expect("a client without Origin is accepted");
        let scene = rpc(&mut none, 2, "query", json!({"name": "view.scene", "input": {}}));
        assert_eq!(scene["volumes"].as_array().map(Vec::len), Some(45), "the live session serves the lattice");
        println!("viewer: foreign Origin refused (403), served Origin and no Origin accepted, session.hash == h_mesh");

        // The page.
        let Some(node) = tool("node") else {
            println!("SKIP viewer browser path: node is not installed");
            return;
        };
        let major: u32 = node.trim_start_matches('v').split('.').next().and_then(|s| s.parse().ok()).unwrap_or(0);
        if major < 22 {
            println!("SKIP viewer smoke: node {node} has no global WebSocket (needs 22+)");
            return;
        }
        let mut cmd = Command::new("node");
        cmd.arg(repo().join("tools/w3b_viewer_smoke.mjs")).arg(format!("http://{addr}/")).arg(dir.join("viewer-out"));
        match playwright_dir() {
            Ok(d) => {
                cmd.env("W3B_PLAYWRIGHT_DIR", d);
            }
            Err(x) => println!("viewer: no playwright ({x}); the browser path will be skipped"),
        }
        let o = cmd.output().expect("run node");
        let text = String::from_utf8_lossy(&o.stdout).into_owned();
        println!("{text}");
        assert!(o.status.success() && text.lines().any(|l| l == "OK"), "viewer smoke failed:\n{text}\n{}", String::from_utf8_lossy(&o.stderr));
        if let Some(l) = text.lines().find(|l| l.starts_with("SKIP browser")) {
            println!("{l}");
        } else {
            assert!(text.lines().any(|l| l.starts_with("page_triangles ")), "{text}");
            assert!(text.lines().any(|l| l.starts_with("followed_redo ")), "{text}");
        }
    }
}
