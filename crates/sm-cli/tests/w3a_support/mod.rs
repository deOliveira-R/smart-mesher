//! Shared plumbing of the W3a milestone (`tests/milestone_w3a.rs`): the per-row report, process
//! and MCP helpers, the script assembly, and the mesh-content checks in the sibling modules.
#![allow(dead_code)]

pub mod exo;
pub mod foam;
pub mod meshchk;
pub mod track;

use std::collections::BTreeMap;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Output, Stdio};

use serde_json::{Value, json};

pub const AUTHOR: &str = "it";
pub const TIME: &str = "2026-10-30T00:00:00Z";
pub const PIN: [&str; 4] = ["--author", AUTHOR, "--fixed-time", TIME];
/// The layers row 1 compares one by one (a new layer is reported as a deviation).
pub const LAYERS: [&str; 4] = ["model", "intent", "meshes", "fem"];

pub type R<T> = Result<T, String>;

pub fn e<E: std::fmt::Display>(e: E) -> String {
    e.to_string()
}

// ------------------------------------------------------------------ report

/// Failures and notes per row, each message prefixed with the path (`[A]`, `[B]`).
#[derive(Default)]
pub struct Rows {
    pub fails: BTreeMap<u8, Vec<String>>,
    pub notes: BTreeMap<u8, Vec<String>>,
}

impl Rows {
    pub fn fail(&mut self, row: u8, msg: impl Into<String>) {
        self.fails.entry(row).or_default().push(msg.into());
    }
    pub fn note(&mut self, row: u8, msg: impl Into<String>) {
        self.notes.entry(row).or_default().push(msg.into());
    }
    pub fn check(&mut self, row: u8, ok: bool, msg: impl FnOnce() -> String) {
        if !ok {
            self.fail(row, msg());
        }
    }
    /// The value of a step, or a failure recorded against every row in `rows`.
    pub fn need<'a, T>(&mut self, rows: &[u8], what: &str, r: &'a R<T>) -> Option<&'a T> {
        match r {
            Ok(v) => Some(v),
            Err(x) => {
                for &row in rows {
                    self.fail(row, format!("{what}: {x}"));
                }
                None
            }
        }
    }
    pub fn eq<T: PartialEq + std::fmt::Debug>(&mut self, row: u8, what: &str, a: &T, b: &T) {
        self.check(row, a == b, || format!("{what}: {a:?} != {b:?}"));
    }
    /// `|a - b| <= tol`.
    pub fn near(&mut self, row: u8, what: &str, a: f64, b: f64, tol: f64) {
        self.check(row, (a - b).abs() <= tol, || format!("{what}: {a} vs {b} (|d| = {:e} > {tol:e})", (a - b).abs()));
    }
}

/// `Rows` that prefixes every message with a path tag.
pub struct Tagged<'a> {
    pub rows: &'a mut Rows,
    pub tag: &'static str,
}

impl Tagged<'_> {
    pub fn fail(&mut self, row: u8, msg: impl Into<String>) {
        let m = msg.into();
        self.rows.fail(row, format!("[{}] {m}", self.tag));
    }
    pub fn note(&mut self, row: u8, msg: impl Into<String>) {
        let m = msg.into();
        self.rows.note(row, format!("[{}] {m}", self.tag));
    }
    pub fn check(&mut self, row: u8, ok: bool, msg: impl FnOnce() -> String) {
        if !ok {
            self.fail(row, msg());
        }
    }
    pub fn need<'a, T>(&mut self, rows: &[u8], what: &str, r: &'a R<T>) -> Option<&'a T> {
        match r {
            Ok(v) => Some(v),
            Err(x) => {
                for &row in rows {
                    self.fail(row, format!("{what}: {x}"));
                }
                None
            }
        }
    }
    pub fn eq<T: PartialEq + std::fmt::Debug>(&mut self, row: u8, what: &str, a: &T, b: &T) {
        self.check(row, a == b, || format!("{what}: {a:?} != {b:?}"));
    }
    pub fn near(&mut self, row: u8, what: &str, a: f64, b: f64, tol: f64) {
        self.check(row, (a - b).abs() <= tol, || format!("{what}: {a} vs {b} (|d| = {:e} > {tol:e})", (a - b).abs()));
    }
}

// ------------------------------------------------------------------ files

pub fn data(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/data/w3a").join(name)
}

pub fn read_text(p: &Path) -> String {
    std::fs::read_to_string(p).unwrap_or_else(|x| panic!("{}: {x}", p.display()))
}

pub fn read_bytes(p: &Path) -> R<Vec<u8>> {
    std::fs::read(p).map_err(|x| format!("{}: {x}", p.display()))
}

/// Copy a directory tree (files and directories only).
pub fn copy_dir(from: &Path, to: &Path) -> R<()> {
    std::fs::create_dir_all(to).map_err(|x| format!("{}: {x}", to.display()))?;
    let mut entries: Vec<_> = std::fs::read_dir(from)
        .map_err(|x| format!("{}: {x}", from.display()))?
        .collect::<Result<_, _>>()
        .map_err(e)?;
    entries.sort_by_key(std::fs::DirEntry::file_name);
    for d in entries {
        let p = d.path();
        let q = to.join(d.file_name());
        if p.is_dir() {
            copy_dir(&p, &q)?;
        } else {
            std::fs::copy(&p, &q).map_err(|x| format!("{}: {x}", p.display()))?;
        }
    }
    Ok(())
}

/// Every regular file under `dir`, as (relative path with `/`, bytes), sorted by path.
pub fn tree(dir: &Path) -> R<BTreeMap<String, Vec<u8>>> {
    fn walk(base: &Path, d: &Path, out: &mut BTreeMap<String, Vec<u8>>) -> R<()> {
        for ent in std::fs::read_dir(d).map_err(|x| format!("{}: {x}", d.display()))? {
            let p = ent.map_err(e)?.path();
            if p.is_dir() {
                walk(base, &p, out)?;
            } else {
                let rel = p.strip_prefix(base).map_err(e)?.to_string_lossy().replace('\\', "/");
                out.insert(rel, read_bytes(&p)?);
            }
        }
        Ok(())
    }
    let mut out = BTreeMap::new();
    walk(dir, dir, &mut out)?;
    Ok(out)
}

pub fn sha_hex(b: &[u8]) -> String {
    sm_op_api::Hash256::of(b).to_string()
}

// ------------------------------------------------------------------ processes

#[derive(Clone, Copy)]
pub enum Threads {
    /// Whatever the test process has.
    Inherit,
    /// `RAYON_NUM_THREADS=1`.
    One,
    /// `RAYON_NUM_THREADS` removed: rayon's default.
    Default,
}

pub fn spawn(args: &[&str], cwd: Option<&Path>, threads: Threads, stdin: Option<&str>) -> R<Output> {
    let mut c = Command::new(env!("CARGO_BIN_EXE_smesh"));
    c.args(args);
    if let Some(d) = cwd {
        c.current_dir(d);
    }
    match threads {
        Threads::Inherit => {}
        Threads::One => {
            c.env("RAYON_NUM_THREADS", "1");
        }
        Threads::Default => {
            c.env_remove("RAYON_NUM_THREADS");
        }
    }
    c.stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::piped());
    let mut child = c.spawn().map_err(|x| format!("spawn smesh {args:?}: {x}"))?;
    if let Some(mut si) = child.stdin.take() {
        if let Some(text) = stdin {
            si.write_all(text.as_bytes()).map_err(|x| format!("stdin: {x}"))?;
        }
        drop(si);
    }
    child.wait_with_output().map_err(|x| format!("wait smesh {args:?}: {x}"))
}

pub fn describe(args: &[&str], o: &Output) -> String {
    let cut = |s: String| if s.len() > 4000 { format!("{}...", &s[..4000]) } else { s };
    format!(
        "`smesh {}` exited {:?}; stdout: {}; stderr: {}",
        args.join(" "),
        o.status.code(),
        cut(String::from_utf8_lossy(&o.stdout).trim().to_owned()),
        cut(String::from_utf8_lossy(&o.stderr).trim().to_owned())
    )
}

/// Run and require exit 0; stdout as text.
pub fn ok_text(args: &[&str], cwd: Option<&Path>, threads: Threads) -> R<String> {
    let o = spawn(args, cwd, threads, None)?;
    if o.status.code() != Some(0) {
        return Err(describe(args, &o));
    }
    String::from_utf8(o.stdout).map_err(e)
}

/// Run with `--json` in `args`; require exit 0 and one JSON value on stdout.
pub fn ok_json(args: &[&str], cwd: Option<&Path>, threads: Threads) -> R<Value> {
    let o = spawn(args, cwd, threads, None)?;
    if o.status.code() != Some(0) {
        return Err(describe(args, &o));
    }
    serde_json::from_slice(&o.stdout).map_err(|x| format!("{}: stdout is not JSON: {x}", describe(args, &o)))
}

/// Run with `--json`; the exit code and the JSON on stdout (any exit code).
pub fn any_json(args: &[&str], cwd: Option<&Path>, threads: Threads) -> R<(i32, Value)> {
    let o = spawn(args, cwd, threads, None)?;
    let v = serde_json::from_slice(&o.stdout).map_err(|x| format!("{}: stdout is not JSON: {x}", describe(args, &o)))?;
    Ok((o.status.code().unwrap_or(-1), v))
}

pub fn hash_of(v: &Value, what: &str) -> R<String> {
    v.as_str().map(str::to_owned).ok_or_else(|| format!("{what}: not a string: {v}"))
}

/// `layers` of a `HashResult` as name -> hex.
pub fn layers_of(v: &Value) -> R<BTreeMap<String, String>> {
    v["layers"]
        .as_object()
        .ok_or_else(|| format!("no `layers` in {v}"))?
        .iter()
        .map(|(k, h)| Ok((k.clone(), hash_of(h, k)?)))
        .collect()
}

/// `print "<key>", value...` lines of a script output, as key -> rest.
pub fn printed(output: &str) -> BTreeMap<String, String> {
    output
        .lines()
        .filter_map(|l| l.split_once(' ').map(|(k, v)| (k.to_owned(), v.trim().to_owned())))
        .collect()
}

// ------------------------------------------------------------------ MCP

pub fn mcp_session(extra: &[&str], lines: &[Value], threads: Threads, cwd: Option<&Path>) -> R<BTreeMap<i64, Value>> {
    let mut args = vec!["mcp", "--memory"];
    args.extend_from_slice(extra);
    let text: String = lines.iter().map(|l| format!("{l}\n")).collect();
    let o = spawn(&args, cwd, threads, Some(&text))?;
    if o.status.code() != Some(0) {
        return Err(describe(&args, &o));
    }
    let mut out = BTreeMap::new();
    for l in String::from_utf8_lossy(&o.stdout).lines().filter(|l| !l.trim().is_empty()) {
        let v: Value = serde_json::from_str(l).map_err(|x| format!("MCP reply is not JSON ({x}): {l}"))?;
        if let Some(id) = v["id"].as_i64() {
            out.insert(id, v);
        }
    }
    Ok(out)
}

pub fn mcp_init() -> Vec<Value> {
    vec![
        json!({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"it","version":"0"}}}),
        json!({"jsonrpc":"2.0","method":"notifications/initialized"}),
    ]
}

pub fn tool_call(id: i64, name: &str, arguments: Value) -> Value {
    json!({"jsonrpc":"2.0","id":id,"method":"tools/call","params":{"name":name,"arguments":arguments}})
}

/// `structuredContent` of a successful `tools/call` reply.
pub fn tool_ok(replies: &BTreeMap<i64, Value>, id: i64) -> R<Value> {
    let r = replies.get(&id).ok_or_else(|| format!("no MCP reply for id {id}"))?;
    let res = r.get("result").ok_or_else(|| format!("id {id}: JSON-RPC error {}", r["error"]))?;
    if res["isError"] != Value::Bool(false) {
        let t = res["content"].to_string();
        return Err(format!("id {id}: isError: {}", if t.len() > 2000 { &t[..2000] } else { &t }));
    }
    Ok(res["structuredContent"].clone())
}

// ------------------------------------------------------------------ scripts

/// The two heads of the milestone (plan §4.1).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Head {
    /// STEP import.
    A,
    /// Bottom-up webcut + unite.
    B,
}

impl Head {
    pub fn tag(self) -> &'static str {
        match self {
            Head::A => "A",
            Head::B => "B",
        }
    }
    pub fn lower(self) -> &'static str {
        match self {
            Head::A => "a",
            Head::B => "b",
        }
    }
}

fn frag(name: &str, size: &str) -> String {
    read_text(&data(name)).replace("@SIZE@", size)
}

/// Head + the volume checks (defines `v1`, `v2`, `v3`, `h0`).
pub fn head(h: Head, step: &Path) -> String {
    let mut s = match h {
        Head::A => read_text(&data("head_a.smesh")).replace("@STEP@", &step.to_string_lossy()),
        Head::B => read_text(&data("head_b.smesh")),
    };
    s.push_str(&read_text(&data("checks.smesh")));
    s
}

pub fn tail_hybrid(size: &str) -> String {
    frag("tail_hybrid.smesh", size)
}

pub fn tail_tet(size: &str) -> String {
    frag("tail_tet.smesh", size)
}

pub const HYBRID_EXPORTS: &str = r#"session.export("out.exo")
session.export("out.smsh", provenance: true)
session.export("out.vtu")
session.export("case", format: "openfoam")
"#;

pub const TET_EXPORTS: &str = r#"session.export("tet.exo", mesh: mt.mesh[0])
session.export("tetcase", format: "openfoam", mesh: mt.mesh[0])
session.export("poly.exo", mesh: pm.mesh[0])
session.export("poly.vtu", mesh: pm.mesh[0])
session.export("poly.smsh", mesh: pm.mesh[0])
session.export("polycase", format: "openfoam", mesh: pm.mesh[0])
"#;

/// `w3a_{a,b}.smesh`: head, hybrid tail and its exports on `main`, the tet branch at `h0` and
/// its exports, then back to `main`. Prints `h0`, `hybrid`, `tet`, `meshes`.
pub fn main_script(h: Head, step: &Path) -> String {
    let mut s = head(h, step);
    s.push_str("print \"h0\", h0\n");
    s.push_str(&tail_hybrid("1.0"));
    s.push_str("let hm = session.hash()\nprint \"hybrid\", hm\nprint \"mesh\", m.mesh[0]\n");
    s.push_str(HYBRID_EXPORTS);
    s.push_str("timeline.branch(\"tet\", at: h0)\n");
    s.push_str(&tail_tet("1.0"));
    s.push_str("let ht = session.hash()\nprint \"tet\", ht\nprint \"meshes\", mt.mesh[0], pm.mesh[0]\n");
    s.push_str(TET_EXPORTS);
    s.push_str("timeline.checkout(\"main\")\nassert session.hash() == hm, \"back on main\"\n");
    s
}

/// `w3a_fork_{a,b}.smesh` (plan §4.2): alt == main, coarse != main, checkout back, tet2 == tet,
/// and a sub-branch with `POLYHEDRON` declared that exports the poly mesh. Every tail runs in its
/// own block so no name is rebound. Prints `hybrid` and `tet`.
pub fn fork_script(h: Head, step: &Path) -> String {
    let block = |body: &str, after: &str| format!("if true {{\n{body}{after}}}\n");
    let mut s = head(h, step);
    s.push_str("let hm = h0\nlet ht = h0\n");
    s.push_str(&block(&tail_hybrid("1.0"), "set hm = session.hash()\n"));
    s.push_str("timeline.branch(\"alt\", at: h0)\n");
    s.push_str(&block(&tail_hybrid("1.0"), "assert session.hash() == hm, \"alt == main\"\n"));
    s.push_str("timeline.branch(\"coarse\", at: h0)\n");
    s.push_str(&block(&tail_hybrid("2.0"), "assert session.hash() != hm, \"coarse != main\"\n"));
    s.push_str("timeline.checkout(\"main\")\nassert session.hash() == hm, \"checkout main\"\n");
    s.push_str("timeline.branch(\"tet\", at: h0)\n");
    s.push_str(&block(&tail_tet("1.0"), "set ht = session.hash()\n"));
    s.push_str("timeline.branch(\"tet2\", at: h0)\n");
    s.push_str(&block(&tail_tet("1.0"), "assert session.hash() == ht, \"tet2 == tet\"\n"));
    s.push_str("timeline.branch(\"tetpoly\", at: h0)\n");
    s.push_str(&block(
        &tail_tet("1.0"),
        "fem.block_element_type(block: 1, element_type: \"POLYHEDRON\")\n\
         fem.block_element_type(block: 2, element_type: \"POLYHEDRON\")\n\
         fem.block_element_type(block: 3, element_type: \"POLYHEDRON\")\n\
         session.export(\"polyblocks.exo\", mesh: pm.mesh[0])\n\
         session.export(\"polyblocks.smsh\", mesh: pm.mesh[0])\n",
    ));
    s.push_str("timeline.checkout(\"main\")\nprint \"hybrid\", hm\nprint \"tet\", ht\n");
    s
}

/// Conformance fixture `w3a_hybrid` (path B, no input file): the hybrid state. Prints one
/// `check <label> <mesh id>` line per mesh `tools/conformance.sh` checks.
pub fn emit_hybrid() -> String {
    let mut s = head(Head::B, Path::new(""));
    s.push_str(&tail_hybrid("1.0"));
    s.push_str("print \"check\", \"w3a_hybrid\", m.mesh[0]\n");
    s
}

/// Conformance fixture `w3a_tet` (path B): the tet branch, its tet and poly meshes.
pub fn emit_tet() -> String {
    let mut s = head(Head::B, Path::new(""));
    s.push_str(&tail_tet("1.0"));
    s.push_str("print \"check\", \"w3a_tet\", mt.mesh[0]\nprint \"check\", \"w3a_poly\", pm.mesh[0]\n");
    s
}

// ------------------------------------------------------------------ recorded ops

/// The op records of a `smesh run --json` result, split into the head (leading `model.*` ops),
/// the hybrid tail (until the `tet` branch) and the tet tail.
#[derive(Clone, Debug, Default)]
pub struct Ops {
    pub head: Vec<(String, Value)>,
    pub hybrid: Vec<(String, Value)>,
    pub tet: Vec<(String, Value)>,
}

pub fn split_ops(run: &Value) -> R<Ops> {
    let events = run["events"].as_array().ok_or("script result has no events")?;
    let mut ops = Ops::default();
    let mut phase = 0;
    for ev in events {
        match ev["event"].as_str() {
            Some("op") => {
                let name = ev["op"].as_str().ok_or("op event without name")?.to_owned();
                if phase == 0 && !name.starts_with("model.") {
                    phase = 1;
                }
                let p = ev["params"].clone();
                match phase {
                    0 => ops.head.push((name, p)),
                    1 => ops.hybrid.push((name, p)),
                    2 => ops.tet.push((name, p)),
                    _ => return Err(format!("op {name} after the tet branch was left")),
                }
            }
            Some("batch") => return Err("unexpected batch event".into()),
            Some("timeline") => {
                let verb = ev["command"].as_str().unwrap_or("");
                phase = match (phase, verb) {
                    (1, "branch") => 2,
                    (2, "checkout") => 3,
                    _ => return Err(format!("unexpected timeline event {ev}")),
                };
            }
            _ => {}
        }
    }
    if ops.head.is_empty() || ops.hybrid.is_empty() || ops.tet.is_empty() {
        return Err(format!("op split failed: {} head, {} hybrid, {} tet", ops.head.len(), ops.hybrid.len(), ops.tet.len()));
    }
    Ok(ops)
}

/// MCP tool name of an op (`model.webcut_plane` -> `model__webcut_plane`).
pub fn tool_name(op: &str) -> String {
    op.replace('.', "__")
}

/// First id of an output role in an `ApplyReport`/delta JSON.
pub fn role0(delta_outputs: &Value, role: &str) -> Option<String> {
    delta_outputs.get(role).and_then(|v| v.get(0)).and_then(Value::as_str).map(str::to_owned)
}
