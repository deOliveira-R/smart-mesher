//! Cross-platform identity-hash pins (W3c-INT, `docs/design/w3c/drafts/linux-hash.md` S4, ruling C12).
//!
//! `tools/platform/expected.txt` holds `script|key|value` lines: the identity hashes of the
//! milestone scripts (state hash, layer hashes, branch tips, the memory-run hash, and the
//! hash-like and count prints), measured on macOS arm64 and equal on Linux arm64 (glibc 2.36 and
//! 2.39), see `docs/design/w3c/platform-hashes.md`. This test recomputes them through the `smesh`
//! binary with the same pinning as `tools/platform/collect.py` and requires equality, so the
//! Linux x86_64 GitHub gate enforces them. It runs twice, with default threads and with
//! `RAYON_NUM_THREADS=1`; both must equal the file. A moved value is never re-blessed per
//! platform (C3, C12): find the first diverging op instead (`tools/platform/compare.py`).
//!
//! `PLATFORM_PINS_EMIT=<file>` writes the computed lines instead of checking them (how
//! `expected.txt` was produced, and how to refresh it after a ruled change).

mod w3a_support;

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde_json::Value;

const PIN: [&str; 4] = ["--author", "it", "--fixed-time", "2026-10-24T00:00:00Z"];

type R<T> = Result<T, String>;

/// Removes this process's scratch tree when the test ends, pass or fail.
struct Cleanup(PathBuf);

impl Drop for Cleanup {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// A print line is pinned when it is a hash or a count (floats and ids are not).
fn keep_print(name: &str) -> bool {
    name.starts_with("h_") || name.contains("hash") || name == "cells" || name == "face_cells"
}

fn smesh(args: &[&str], cwd: &Path, threads: Option<&str>) -> R<String> {
    let mut c = Command::new(env!("CARGO_BIN_EXE_smesh"));
    c.args(args).current_dir(cwd);
    match threads {
        Some(t) => c.env("RAYON_NUM_THREADS", t),
        None => c.env_remove("RAYON_NUM_THREADS"),
    };
    let o = c.output().map_err(|e| format!("spawn smesh {args:?}: {e}"))?;
    if !o.status.success() {
        return Err(format!("smesh {args:?} failed: {}", String::from_utf8_lossy(&o.stderr)));
    }
    Ok(String::from_utf8_lossy(&o.stdout).into_owned())
}

fn json(args: &[&str], cwd: &Path, threads: Option<&str>) -> R<Value> {
    serde_json::from_str(&smesh(args, cwd, threads)?).map_err(|e| format!("smesh {args:?}: bad json: {e}"))
}

fn s(v: &Value) -> R<String> {
    v.as_str().map(str::to_owned).ok_or_else(|| format!("not a string: {v}"))
}

/// The pinned lines of one script, same keys as `tools/platform/collect.py` minus the per-node rows.
fn collect(name: &str, script: &Path, scratch: &Path, threads: Option<&str>) -> R<BTreeSet<String>> {
    let dir = scratch.join(format!("{name}-{}", threads.unwrap_or("default")));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(dir.join("mem")).map_err(|e| e.to_string())?;
    let (proj, script) = (dir.join("proj"), script.to_string_lossy().into_owned());
    let (ps, mut rows) = (proj.to_string_lossy().into_owned(), BTreeSet::new());
    let mut add = |k: &str, v: String| rows.insert(format!("{name}|{k}|{v}"));
    smesh(&["new", &ps], &dir, threads)?;
    let mut a = vec!["run", script.as_str(), "-p", ps.as_str()];
    a.extend(PIN);
    a.push("--json");
    let out = json(&a, &dir, threads)?;
    for line in out["output"].as_str().unwrap_or("").lines() {
        if let Some((k, v)) = line.split_once(char::is_whitespace)
            && keep_print(k)
        {
            add(&format!("print.{k}"), v.trim().to_owned());
        }
    }
    let h = json(&["hash", "-p", &ps, "--json"], &dir, threads)?;
    add("state", s(&h["state"])?);
    for (k, v) in h["layers"].as_object().ok_or("no layers")? {
        add(&format!("layer.{k}"), s(v)?);
    }
    let log = json(&["log", "-p", &ps, "-n", "1000", "--json"], &dir, threads)?;
    for b in log["branches"].as_object().ok_or("no branches")?.keys() {
        let hb = json(&["hash", "--at", b, "-p", &ps, "--json"], &dir, threads)?;
        add(&format!("branch.{b}"), s(&hb["state"])?);
    }
    // memory run in its own cwd (some scripts write files relative to it)
    let mut m = vec!["run", script.as_str(), "--memory", "--print-hash"];
    m.extend(PIN);
    let mem = smesh(&m, &dir.join("mem"), threads)?;
    add("mem.state", mem.trim().lines().last().ok_or("empty memory run")?.to_owned());
    Ok(rows)
}

fn all(threads: Option<&str>, scratch: &Path) -> R<BTreeSet<String>> {
    let t = root().join("tools");
    let data = root().join("crates/sm-cli/tests/data");
    let emit = scratch.join("emit");
    std::fs::create_dir_all(&emit).map_err(|e| e.to_string())?;
    for (f, text) in [("w3a_hybrid.smesh", w3a_support::emit_hybrid()), ("w3a_tet.smesh", w3a_support::emit_tet())] {
        std::fs::write(emit.join(f), text).map_err(|e| e.to_string())?;
    }
    let scripts: [(&str, PathBuf); 7] = [
        ("w2_milestone", data.join("milestone.smesh")),
        ("w2_fork", data.join("fork.smesh")),
        ("w3a_hybrid", emit.join("w3a_hybrid.smesh")),
        ("w3a_tet", emit.join("w3a_tet.smesh")),
        ("w3b_milestone", t.join("w3b_milestone.smesh")),
        ("w3b_submap_graded", t.join("platform/submap_graded.smesh")),
        ("w3c_pin", t.join("w3c_pin.smesh")),
    ];
    let mut rows = BTreeSet::new();
    for (name, path) in &scripts {
        rows.extend(collect(name, path, scratch, threads)?);
    }
    Ok(rows)
}

fn expected() -> R<BTreeSet<String>> {
    let p = root().join("tools/platform/expected.txt");
    let text = std::fs::read_to_string(&p).map_err(|e| format!("{}: {e}", p.display()))?;
    Ok(text.lines().filter(|l| !l.is_empty() && !l.starts_with('#')).map(str::to_owned).collect())
}

fn diff(want: &BTreeSet<String>, got: &BTreeSet<String>) -> String {
    let mut out = String::new();
    for l in want.difference(got) {
        out.push_str(&format!("  expected, not produced: {l}\n"));
    }
    for l in got.difference(want) {
        out.push_str(&format!("  produced, not expected: {l}\n"));
    }
    out
}

#[test]
fn identity_hashes_match_expected_on_every_platform() {
    let scratch = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("platform_pins-{}", std::process::id()));
    let _guard = Cleanup(scratch.clone());
    let default = all(None, &scratch).expect("default-thread run");
    if let Some(p) = std::env::var_os("PLATFORM_PINS_EMIT") {
        let mut text = String::from(
            "# Identity hashes of the milestone scripts (see tests/platform_pins.rs, docs/design/w3c/platform-hashes.md).\n",
        );
        for l in &default {
            text.push_str(l);
            text.push('\n');
        }
        std::fs::write(p, text).expect("write emit file");
        return;
    }
    let want = expected().expect("expected.txt");
    let d = diff(&want, &default);
    assert!(d.is_empty(), "identity hashes moved on this platform (C3/C12: do not re-bless):\n{d}");
    let one = all(Some("1"), &scratch).expect("1-thread run");
    let d = diff(&default, &one);
    assert!(d.is_empty(), "RAYON_NUM_THREADS=1 differs from default threads:\n{d}");
}
