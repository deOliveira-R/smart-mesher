//! W2 integration milestone (plan `docs/design/w2/plan.md` §4): one brick, sized, mapped,
//! blocked and exported through three front ends of the same `smesh` binary (CLI §4.3, MCP
//! §4.4) and in process (§4.5), then every assertion row of §4.6.
//!
//! The test collects: each step that fails is recorded against the rows that need it, every row
//! is evaluated, and the test fails once at the end with the full per-row report. Run it as
//! `cargo test -p sm-cli --test milestone -- --nocapture` (and with `RAYON_NUM_THREADS=1`).

mod vtu_check;

use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::{Command, Output, Stdio};

use serde_json::{Value, json};
use sm_engine::{ApplyOptions, FixedClock, Workspace};
use sm_op_api::{Hash256, Rev};

const AUTHOR: &str = "it";
const TIME: &str = "2026-10-24T00:00:00Z";
const PIN: [&str; 4] = ["--author", AUTHOR, "--fixed-time", TIME];
/// The layers row 1 compares one by one.
const LAYERS: [&str; 4] = ["model", "intent", "meshes", "fem"];

type R<T> = Result<T, String>;

// ------------------------------------------------------------------ report

#[derive(Default)]
struct Rows {
    fails: BTreeMap<u8, Vec<String>>,
    notes: BTreeMap<u8, Vec<String>>,
}

impl Rows {
    fn fail(&mut self, row: u8, msg: impl Into<String>) {
        self.fails.entry(row).or_default().push(msg.into());
    }
    fn note(&mut self, row: u8, msg: impl Into<String>) {
        self.notes.entry(row).or_default().push(msg.into());
    }
    fn check(&mut self, row: u8, ok: bool, msg: impl FnOnce() -> String) {
        if !ok {
            self.fail(row, msg());
        }
    }
    /// The value of a step, or a failure recorded against every row in `rows`.
    fn need<'a, T>(&mut self, rows: &[u8], what: &str, r: &'a R<T>) -> Option<&'a T> {
        match r {
            Ok(v) => Some(v),
            Err(e) => {
                for &row in rows {
                    self.fail(row, format!("{what}: {e}"));
                }
                None
            }
        }
    }
    fn eq<T: PartialEq + std::fmt::Debug>(&mut self, row: u8, what: &str, a: &T, b: &T) {
        self.check(row, a == b, || format!("{what}: {a:?} != {b:?}"));
    }
}

// ------------------------------------------------------------------ process helpers

fn data(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/data")
        .join(name)
}

fn read_text(p: &Path) -> String {
    std::fs::read_to_string(p).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
}

/// A fresh scratch directory (removed first).
fn scratch() -> PathBuf {
    let d =
        Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("milestone-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("scratch dir");
    d
}

#[derive(Clone, Copy)]
enum Threads {
    /// Whatever the test process has.
    Inherit,
    /// `RAYON_NUM_THREADS=1`.
    One,
    /// `RAYON_NUM_THREADS` removed: rayon's default.
    Default,
}

fn spawn(args: &[&str], cwd: Option<&Path>, threads: Threads, stdin: Option<&str>) -> R<Output> {
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
    c.stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = c
        .spawn()
        .map_err(|e| format!("spawn smesh {args:?}: {e}"))?;
    if let Some(mut si) = child.stdin.take() {
        if let Some(text) = stdin {
            si.write_all(text.as_bytes())
                .map_err(|e| format!("stdin: {e}"))?;
        }
        drop(si);
    }
    child
        .wait_with_output()
        .map_err(|e| format!("wait smesh {args:?}: {e}"))
}

fn describe(args: &[&str], o: &Output) -> String {
    format!(
        "`smesh {}` exited {:?}; stdout: {}; stderr: {}",
        args.join(" "),
        o.status.code(),
        String::from_utf8_lossy(&o.stdout).trim(),
        String::from_utf8_lossy(&o.stderr).trim()
    )
}

/// Run and require exit 0; stdout as text.
fn ok_text(args: &[&str], cwd: Option<&Path>, threads: Threads) -> R<String> {
    let o = spawn(args, cwd, threads, None)?;
    if o.status.code() != Some(0) {
        return Err(describe(args, &o));
    }
    String::from_utf8(o.stdout).map_err(|e| e.to_string())
}

/// Run with `--json` already in `args`; require exit 0 and one JSON value on stdout.
fn ok_json(args: &[&str], cwd: Option<&Path>, threads: Threads) -> R<Value> {
    let o = spawn(args, cwd, threads, None)?;
    if o.status.code() != Some(0) {
        return Err(describe(args, &o));
    }
    serde_json::from_slice(&o.stdout)
        .map_err(|e| format!("{}: stdout is not JSON: {e}", describe(args, &o)))
}

fn read_bytes(p: &Path) -> R<Vec<u8>> {
    std::fs::read(p).map_err(|e| format!("{}: {e}", p.display()))
}

fn hash_of(v: &Value, what: &str) -> R<String> {
    v.as_str()
        .map(str::to_owned)
        .ok_or_else(|| format!("{what}: not a string: {v}"))
}

/// `layers` of a `HashResult` as name -> hex.
fn layers_of(v: &Value) -> R<BTreeMap<String, String>> {
    v["layers"]
        .as_object()
        .ok_or_else(|| format!("no `layers` in {v}"))?
        .iter()
        .map(|(k, h)| Ok((k.clone(), hash_of(h, k)?)))
        .collect()
}

/// The state hash printed by `--print-hash` (the last stdout line).
fn last_line(s: &str) -> R<String> {
    s.lines()
        .last()
        .map(str::to_owned)
        .ok_or_else(|| "empty stdout".to_owned())
}

// ------------------------------------------------------------------ .smsh header (dep-free)

/// Split a `.smsh` file into its JSON header and the array payload after it.
/// Decode a `.smsh` with sm_meshio and rebuild it as a `ReferenceMesh` (the decoded form is not
/// itself a `MeshRead`), keeping vertices, faces, cells and their classifications.
fn smsh_rebuild(bytes: &[u8]) -> R<sm_mesh_api::reference::ReferenceMesh> {
    use sm_mesh_api::{
        CellIx, FaceIx, MeshBuild, MeshEntityRef, SerialAlloc, SignedFace, VertexIx,
    };
    let file = sm_meshio::SmshFile::parse(bytes).map_err(|e| e.to_string())?;
    file.verify_checksums().map_err(|e| e.to_string())?;
    let d = file.decode().map_err(|e| e.to_string())?;
    let mut b = sm_mesh_api::reference::ReferenceBuilder::with_dim(d.dim);
    let e = |e: sm_mesh_api::MeshError| e.to_string();
    for i in 0..d.x.len() {
        b.add_vertex(sm_core::Point3 {
            x: d.x[i],
            y: d.y[i],
            z: d.z[i],
        })
        .map_err(e)?;
    }
    for f in 0..d.face_left.len() {
        let l: Vec<VertexIx> = d
            .face_loop(f)
            .iter()
            .map(|&v| VertexIx::new(v as usize))
            .collect();
        b.add_face(&l).map_err(e)?;
    }
    for c in 0..d.cell_ids.len() {
        let faces: Vec<SignedFace> = d.cell_face_list(c).collect();
        let r = d
            .cell_corner_offsets
            .get(c..c + 2)
            .map(|w| w[0] as usize..w[1] as usize)
            .unwrap_or(0..0);
        let corners: Vec<VertexIx> = d.cell_corners[r]
            .iter()
            .map(|&v| VertexIx::new(v as usize))
            .collect();
        b.add_cell(
            &faces,
            if corners.is_empty() {
                None
            } else {
                Some(&corners)
            },
        )
        .map_err(e)?;
    }
    for (i, cl) in d.vertex_class.iter().enumerate() {
        b.set_class(MeshEntityRef::Vertex(VertexIx::new(i)), *cl)
            .map_err(e)?;
    }
    for (i, cl) in d.face_class.iter().enumerate() {
        b.set_class(MeshEntityRef::Face(FaceIx::new(i)), *cl)
            .map_err(e)?;
    }
    for (i, cl) in d.cell_class.iter().enumerate() {
        b.set_class(MeshEntityRef::Cell(CellIx::new(i)), *cl)
            .map_err(e)?;
    }
    b.finish(&mut SerialAlloc::new()).map(|f| f.mesh).map_err(e)
}

fn smsh_split(bytes: &[u8]) -> R<(Value, Vec<u8>)> {
    if bytes.len() < 16 || &bytes[..8] != b"SMSH\r\n\x1a\n" {
        return Err("not a .smsh file (magic)".into());
    }
    let n = u64::from_le_bytes(bytes[8..16].try_into().map_err(|_| "header length")?) as usize;
    let head = bytes.get(16..16 + n).ok_or("short header")?;
    let v: Value = serde_json::from_slice(head).map_err(|e| format!("header JSON: {e}"))?;
    Ok((v, bytes[16 + n..].to_vec()))
}

// ------------------------------------------------------------------ MCP

fn mcp_session(extra: &[&str], lines: &[Value], threads: Threads) -> R<BTreeMap<i64, Value>> {
    let mut args = vec!["mcp", "--memory"];
    args.extend_from_slice(extra);
    let text: String = lines.iter().map(|l| format!("{l}\n")).collect();
    let o = spawn(&args, None, threads, Some(&text))?;
    if o.status.code() != Some(0) {
        return Err(describe(&args, &o));
    }
    let mut out = BTreeMap::new();
    for l in String::from_utf8_lossy(&o.stdout)
        .lines()
        .filter(|l| !l.trim().is_empty())
    {
        let v: Value =
            serde_json::from_str(l).map_err(|e| format!("MCP reply is not JSON ({e}): {l}"))?;
        let id = v["id"]
            .as_i64()
            .ok_or_else(|| format!("MCP reply without integer id: {l}"))?;
        out.insert(id, v);
    }
    Ok(out)
}

fn tool_call(id: i64, name: &str, arguments: Value) -> Value {
    json!({"jsonrpc":"2.0","id":id,"method":"tools/call","params":{"name":name,"arguments":arguments}})
}

/// `structuredContent` of a successful `tools/call` reply; also checks that the text content is
/// exactly the JSON of `structuredContent`.
fn tool_ok(replies: &BTreeMap<i64, Value>, id: i64) -> R<Value> {
    let r = replies
        .get(&id)
        .ok_or_else(|| format!("no MCP reply for id {id}"))?;
    let res = r
        .get("result")
        .ok_or_else(|| format!("id {id}: JSON-RPC error {}", r["error"]))?;
    if res["isError"] != Value::Bool(false) {
        return Err(format!("id {id}: isError: {}", res["content"]));
    }
    let sc = res["structuredContent"].clone();
    let text = res["content"][0]["text"]
        .as_str()
        .ok_or_else(|| format!("id {id}: no text content"))?;
    let parsed: Value = serde_json::from_str(text)
        .map_err(|e| format!("id {id}: text content is not JSON: {e}"))?;
    if parsed != sc {
        return Err(format!(
            "id {id}: text content differs from structuredContent: {text} vs {sc}"
        ));
    }
    Ok(sc)
}

fn mcp_milestone_lines(dir: &Path) -> Vec<Value> {
    let p = |n: &str| dir.join(n).to_string_lossy().into_owned();
    vec![
        json!({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"it","version":"0"}}}),
        json!({"jsonrpc":"2.0","method":"notifications/initialized"}),
        json!({"jsonrpc":"2.0","id":2,"method":"tools/list"}),
        tool_call(3, "model__create_brick", json!({"x":10,"y":10,"z":10})),
        tool_call(
            4,
            "mesh__size",
            json!({"targets":["TopoVolume#1"],"size":1.0}),
        ),
        tool_call(
            5,
            "mesh__scheme",
            json!({"targets":["TopoVolume#1"],"scheme":"map"}),
        ),
        tool_call(6, "mesh__run", json!({"targets":["TopoVolume#1"]})),
        tool_call(7, "fem__block_add", json!({"block":1,"volumes":["TopoVolume#1"]})),
        tool_call(8, "fem__block_element_type", json!({"block":1,"element_type":"HEX8"})),
        tool_call(9, "session_export", json!({"path": p("mcp.smsh")})),
        tool_call(10, "session_export", json!({"path": p("mcp.vtu")})),
        tool_call(11, "session_hash", json!({})),
    ]
}

/// (node, state) of the six op calls 3..=8.
fn mcp_nodes(replies: &BTreeMap<i64, Value>) -> R<Vec<(String, String)>> {
    (3..=8)
        .map(|id| {
            let sc = tool_ok(replies, id)?;
            Ok((
                hash_of(&sc["node"], "node")?,
                hash_of(&sc["state"], "state")?,
            ))
        })
        .collect()
}

// ------------------------------------------------------------------ in process

/// The milestone ops as `(op, params)`, with `TopoVolume#1` taken from the brick's outputs.
fn api_ops(vol: &str, size: f64) -> Vec<(&'static str, Value)> {
    vec![
        ("mesh.size", json!({"targets":[vol],"size":size})),
        ("mesh.scheme", json!({"targets":[vol],"scheme":"map"})),
        ("mesh.run", json!({"targets":[vol]})),
        ("fem.block_add", json!({"block":1,"volumes":[vol]})),
        ("fem.block_element_type", json!({"block":1,"element_type":"HEX8"})),
    ]
}

struct Api {
    state: String,
    layers: BTreeMap<String, String>,
    nodes: Vec<(String, String)>,
    replay_root: String,
    replay_state_at: String,
    replay_layers: BTreeMap<String, String>,
    fork_alt: String,
    fork_coarse: String,
    coarse_cells: Value,
    back_on_main: String,
}

fn hexes(m: &BTreeMap<String, Hash256>) -> BTreeMap<String, String> {
    m.iter().map(|(k, v)| (k.clone(), v.to_string())).collect()
}

fn in_process() -> R<Api> {
    let e = |e: sm_engine::EngineError| e.to_string();
    let mut ws: Workspace = sm_server::standard()
        .author(AUTHOR)
        .clock(FixedClock(TIME.to_owned()))
        .memory()
        .map_err(e)?;
    let opts = ApplyOptions::default();
    let brick = ws
        .apply_json("model.create_brick", json!({"x":10,"y":10,"z":10}), &opts)
        .map_err(e)?;
    let vol = brick
        .delta
        .outputs
        .get("volume")
        .and_then(|v| v.first())
        .ok_or("create_brick has no `volume` output")?
        .to_string();
    let n0 = brick.node;
    let mut nodes = vec![(brick.node.to_string(), brick.state.to_string())];
    for (op, p) in api_ops(&vol, 1.0) {
        let r = ws.apply_json(op, p, &opts).map_err(e)?;
        nodes.push((r.node.to_string(), r.state.to_string()));
    }
    let head = ws.session().position().ok_or("no head")?;
    let state = ws.state().hash();
    if state != ws.state_hash() {
        return Err(format!(
            "State::hash {state} != Workspace::state_hash {}",
            ws.state_hash()
        ));
    }
    let layers = hexes(&ws.state().layer_hashes());
    let root = ws.state_from_root(head).map_err(e)?;
    let at = ws.state_at(&Rev::Node(head)).map_err(e)?;
    // Fork in process (row 2 from Rust): re-apply on `alt`, coarser on `coarse`, back to main.
    ws.fork("alt", Some(n0)).map_err(e)?;
    for (op, p) in api_ops(&vol, 1.0) {
        ws.apply_json(op, p, &opts).map_err(e)?;
    }
    let fork_alt = ws.state().hash().to_string();
    ws.fork("coarse", Some(n0)).map_err(e)?;
    let mut mesh = None;
    for (op, p) in api_ops(&vol, 2.0) {
        let r = ws.apply_json(op, p, &opts).map_err(e)?;
        if op == "mesh.run" {
            mesh = r
                .delta
                .outputs
                .get("mesh")
                .and_then(|v| v.first())
                .map(ToString::to_string);
        }
    }
    let fork_coarse = ws.state().hash().to_string();
    let mesh = mesh.ok_or("coarse mesh.run has no `mesh` output")?;
    let coarse_cells = ws.query("mesh.info", &json!({"mesh": mesh})).map_err(e)?["cells"].clone();
    ws.checkout(Rev::Ref("main".into())).map_err(e)?;
    Ok(Api {
        state: state.to_string(),
        layers,
        nodes,
        replay_root: root.hash().to_string(),
        replay_state_at: at.hash().to_string(),
        replay_layers: hexes(&root.layer_hashes()),
        fork_alt,
        fork_coarse,
        coarse_cells,
        back_on_main: ws.state().hash().to_string(),
    })
}

// ------------------------------------------------------------------ the test

#[test]
fn milestone() {
    let t = scratch();
    let ts = |p: &Path| p.to_string_lossy().into_owned();
    let script = data("milestone.smesh");
    let script_s = ts(&script);
    let fork_s = ts(&data("fork.smesh"));
    let mut rows = Rows::default();

    // ---------------- §4.3 CLI, project
    let cli = t.join("cli");
    let cli_s = ts(&cli);
    let new = ok_text(&["new", &cli_s], None, Threads::Inherit);
    rows.need(&[1, 3, 4, 5, 6], "smesh new $T/cli", &new);
    let mut run_args = vec!["run", &script_s, "-p", &cli_s];
    run_args.extend(PIN);
    run_args.push("--json");
    let cli_run = ok_json(&run_args, None, Threads::Inherit);
    let h_cli = cli_run
        .as_ref()
        .map_err(Clone::clone)
        .and_then(|v| hash_of(&v["state"], "state"));
    if let Ok(v) = &cli_run {
        let printed = v["output"].as_str().unwrap_or("").trim().to_owned();
        if let Ok(h) = &h_cli {
            rows.eq(
                1,
                "CLI script `print session.hash()` vs ScriptResult.state",
                &printed,
                h,
            );
        }
    }
    let cli_hash = ok_json(&["hash", "-p", &cli_s, "--json"], None, Threads::Inherit);
    let cli_log = ok_json(
        &["log", "-p", &cli_s, "-n", "100", "--json"],
        None,
        Threads::Inherit,
    );
    let re_smsh = cli.join("re.smsh");
    let re_vtu = cli.join("re.vtu");
    let re1 = ok_json(
        &["export", &ts(&re_smsh), "-p", &cli_s, "--json"],
        None,
        Threads::Inherit,
    );
    let re2 = ok_json(
        &["export", &ts(&re_vtu), "-p", &cli_s, "--json"],
        None,
        Threads::Inherit,
    );
    // Node list N_cli (oldest first) and the state at each node.
    let n_cli: R<Vec<(String, String)>> = cli_log.as_ref().map_err(Clone::clone).and_then(|log| {
        let mut nodes: Vec<String> = log["nodes"]
            .as_array()
            .ok_or("log has no nodes")?
            .iter()
            .map(|n| hash_of(&n["node"], "node"))
            .collect::<R<_>>()?;
        nodes.reverse();
        nodes
            .into_iter()
            .map(|n| {
                let h = ok_json(
                    &["hash", "--at", &n, "-p", &cli_s, "--json"],
                    None,
                    Threads::Inherit,
                )?;
                Ok((n, hash_of(&h["state"], "state")?))
            })
            .collect()
    });

    // ---------------- §4.3 CLI, memory runs (each in its own working directory)
    let mem_run = |dir: &str, pin: bool, threads: Threads| -> (PathBuf, R<String>) {
        let d = t.join(dir);
        let _ = std::fs::create_dir_all(&d);
        let mut a = vec!["run", script_s.as_str(), "--memory", "--print-hash"];
        if pin {
            a.extend(PIN);
        }
        let h = ok_text(&a, Some(&d), threads).and_then(|s| last_line(&s));
        (d, h)
    };
    let (m1, h_1t) = mem_run("m1", true, Threads::One);
    let (m2, h_mem) = mem_run("m2", true, Threads::Default);
    let (m3, h_m3) = mem_run("m3", false, Threads::Inherit);
    // Per-layer hashes of the memory path: the same script through `smesh serve --stdio --memory`.
    let serve_layers = |dir: &str, threads: Threads| -> R<(String, BTreeMap<String, String>)> {
        let d = t.join(dir);
        let _ = std::fs::create_dir_all(&d);
        let req = format!(
            "{}\n{}\n",
            json!({"jsonrpc":"2.0","id":1,"method":"script.run","params":{"text": read_text(&script), "path": script_s}}),
            json!({"jsonrpc":"2.0","id":2,"method":"session.hash"})
        );
        let mut a = vec!["serve", "--stdio", "--memory"];
        a.extend(PIN);
        let o = spawn(&a, Some(&d), threads, Some(&req))?;
        if o.status.code() != Some(0) {
            return Err(describe(&a, &o));
        }
        let out = String::from_utf8_lossy(&o.stdout).into_owned();
        let last = out.lines().last().ok_or("serve: no output")?;
        let v: Value = serde_json::from_str(last).map_err(|e| format!("serve reply: {e}"))?;
        let r = v
            .get("result")
            .ok_or_else(|| format!("serve session.hash error: {v}"))?;
        if let Some(first) = out.lines().next()
            && serde_json::from_str::<Value>(first)
                .ok()
                .and_then(|f| f.get("error").cloned())
                .is_some()
        {
            return Err(format!("serve script.run failed: {first}"));
        }
        Ok((hash_of(&r["state"], "state")?, layers_of(r)?))
    };
    let mem_layers_1t = serve_layers("s1", Threads::One);
    let mem_layers = serve_layers("s2", Threads::Default);

    // ---------------- §4.3 CLI, fork project
    let fork = t.join("fork");
    let fork_p = ts(&fork);
    let fork_run = ok_json(
        &["run", &fork_s, "-p", &fork_p, "--json"],
        None,
        Threads::Inherit,
    );
    let fork_log = ok_json(&["log", "-p", &fork_p, "--json"], None, Threads::Inherit);
    let fork_hash_at = |rev: &str| -> R<String> {
        let v = ok_json(
            &["hash", "--at", rev, "-p", &fork_p, "--json"],
            None,
            Threads::Inherit,
        )?;
        hash_of(&v["state"], "state")
    };
    let fork_main = fork_hash_at("main");
    let fork_alt = fork_hash_at("alt");
    let fork_coarse = fork_hash_at("coarse");
    let fork_checkout = ok_json(
        &["checkout", "alt", "-p", &fork_p, "--json"],
        None,
        Threads::Inherit,
    );
    let fork_after = ok_json(&["hash", "-p", &fork_p, "--json"], None, Threads::Inherit)
        .and_then(|v| hash_of(&v["state"], "state"));
    // The fork script on its own, in memory (its asserts run in the script).
    let fork_mem = ok_text(
        &["run", &fork_s, "--memory", "--print-hash"],
        Some(&m1),
        Threads::Inherit,
    )
    .and_then(|s| last_line(&s));

    // ---------------- §4.3 CLI, unpinned project (row 4's unpinned half on the CLI side)
    let cliu = t.join("cliu");
    let cliu_s = ts(&cliu);
    let cliu_nodes: R<Vec<(String, String)>> = ok_json(
        &["run", &script_s, "-p", &cliu_s, "--json"],
        None,
        Threads::Inherit,
    )
    .and_then(|_| {
        let log = ok_json(
            &["log", "-p", &cliu_s, "-n", "100", "--json"],
            None,
            Threads::Inherit,
        )?;
        let mut nodes: Vec<String> = log["nodes"]
            .as_array()
            .ok_or("log has no nodes")?
            .iter()
            .map(|n| hash_of(&n["node"], "node"))
            .collect::<R<_>>()?;
        nodes.reverse();
        nodes
            .into_iter()
            .map(|n| {
                let h = ok_json(
                    &["hash", "--at", &n, "-p", &cliu_s, "--json"],
                    None,
                    Threads::Inherit,
                )?;
                Ok((n, hash_of(&h["state"], "state")?))
            })
            .collect()
    });

    // ---------------- §4.4 MCP
    let mcp_pinned = mcp_session(&PIN, &mcp_milestone_lines(&t), Threads::Inherit);
    let mcpu_dir = t.join("mcpu");
    let _ = std::fs::create_dir_all(&mcpu_dir);
    let mcp_unpinned = mcp_session(&[], &mcp_milestone_lines(&mcpu_dir), Threads::Inherit);
    let mcp_fork = mcp_session(
        &PIN,
        &[
            json!({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"it","version":"0"}}}),
            json!({"jsonrpc":"2.0","method":"notifications/initialized"}),
            tool_call(12, "run_script", json!({"text": read_text(&data("fork.smesh"))})),
        ],
        Threads::Inherit,
    );
    let mut h_mcp: R<String> = Err("MCP session failed".into());
    let mut mcp_layers: R<BTreeMap<String, String>> = Err("MCP session failed".into());
    let mut n_mcp: R<Vec<(String, String)>> = Err("MCP session failed".into());
    match &mcp_pinned {
        Err(e) => {
            for r in [1, 4, 5] {
                rows.fail(r, format!("MCP pinned session: {e}"));
            }
        }
        Ok(rep) => {
            // initialize / tools/list
            let init_ok = rep
                .get(&1)
                .is_some_and(|r| r["result"]["protocolVersion"] == "2025-06-18");
            rows.check(1, init_ok, || format!("MCP initialize: {:?}", rep.get(&1)));
            let names: Vec<String> = rep
                .get(&2)
                .and_then(|r| r["result"]["tools"].as_array().cloned())
                .unwrap_or_default()
                .iter()
                .filter_map(|t| t["name"].as_str().map(str::to_owned))
                .collect();
            for want in [
                "model__create_brick",
                "mesh__size",
                "mesh__scheme",
                "mesh__run",
                "fem__block_add",
                "session_export",
                "session_hash",
                "run_script",
            ] {
                rows.check(1, names.iter().any(|n| n == want), || {
                    format!("MCP tools/list lacks {want}")
                });
            }
            match tool_ok(rep, 3) {
                Ok(sc) => rows.eq(
                    1,
                    "MCP create_brick outputs.volume",
                    &sc["delta"]["outputs"]["volume"],
                    &json!(["TopoVolume#1"]),
                ),
                Err(e) => rows.fail(1, e),
            }
            for id in [9, 10] {
                if let Err(e) = tool_ok(rep, id) {
                    rows.fail(5, format!("MCP session_export: {e}"));
                }
            }
            n_mcp = mcp_nodes(rep);
            let hr = tool_ok(rep, 11);
            h_mcp = hr.as_ref().map_err(Clone::clone).and_then(|v| hash_of(&v["state"], "state"));
            mcp_layers = hr.and_then(|v| layers_of(&v));
        }
    }
    match &mcp_fork {
        Ok(rep) => match tool_ok(rep, 12) {
            Ok(_) => rows.note(2, "MCP run_script(fork.smesh) in a fresh process: asserts pass"),
            Err(e) => rows.fail(2, format!("MCP run_script(fork.smesh): {e}")),
        },
        Err(e) => rows.fail(2, format!("MCP fork session: {e}")),
    }

    // ---------------- §4.5 in process
    let api = in_process();

    // ================= row 1: H_cli == H_mem == H_1t == H_mcp == H_api, and per layer
    let h_api = api.as_ref().map(|a| a.state.clone()).map_err(Clone::clone);
    let hashes = [
        ("H_cli", &h_cli),
        ("H_mem", &h_mem),
        ("H_1t", &h_1t),
        ("H_mcp", &h_mcp),
        ("H_api", &h_api),
    ];
    let mut known = BTreeMap::new();
    for (name, h) in hashes {
        if let Some(h) = rows.need(&[1], name, h) {
            known.insert(name, h.clone());
        }
    }
    let distinct: BTreeSet<&String> = known.values().collect();
    rows.check(1, distinct.len() <= 1, || {
        format!("state hashes differ: {known:?}")
    });
    let mut layer_maps: Vec<(&str, BTreeMap<String, String>)> = Vec::new();
    let cli_layers = cli_hash.as_ref().map_err(Clone::clone).and_then(layers_of);
    for (name, l) in [
        ("CLI (hash -p)", cli_layers.clone()),
        (
            "memory serve, 1 thread",
            mem_layers_1t
                .as_ref()
                .map(|x| x.1.clone())
                .map_err(Clone::clone),
        ),
        (
            "memory serve, default threads",
            mem_layers
                .as_ref()
                .map(|x| x.1.clone())
                .map_err(Clone::clone),
        ),
        ("MCP", mcp_layers.clone()),
        (
            "in process",
            api.as_ref().map(|a| a.layers.clone()).map_err(Clone::clone),
        ),
    ] {
        if let Some(l) = rows.need(&[1], &format!("layer hashes, {name}"), &l) {
            layer_maps.push((name, l.clone()));
        }
    }
    for (name, l) in &layer_maps {
        for want in LAYERS {
            rows.check(1, l.contains_key(want), || {
                format!(
                    "{name}: no `{want}` layer hash (present: {:?})",
                    l.keys().collect::<Vec<_>>()
                )
            });
        }
    }
    if let Some((n0, l0)) = layer_maps.first() {
        for (n, l) in &layer_maps[1..] {
            rows.check(1, l == l0, || {
                format!("layer hashes {n} {l:?} != {n0} {l0:?}")
            });
        }
    }
    if let (Ok((s, _)), Ok(h)) = (&mem_layers, &h_mem) {
        rows.eq(1, "memory serve state vs H_mem", s, h);
    }

    // ================= row 2: fork
    if let Ok(v) = rows
        .need(&[2], "CLI fork run -p $T/fork", &fork_run)
        .cloned()
        .ok_or(())
    {
        rows.check(2, v["state"].is_string(), || {
            format!("fork run result: {v}")
        });
    }
    if let Some(log) = rows.need(&[2], "CLI fork log", &fork_log) {
        for b in ["main", "alt", "coarse"] {
            let ok = log["branches"][b].is_string();
            rows.check(2, ok, || {
                format!("fork log lacks branch `{b}`: {}", log["branches"])
            });
        }
    }
    let (fm, fa, fc) = (
        rows.need(&[2], "hash --at main", &fork_main).cloned(),
        rows.need(&[2], "hash --at alt", &fork_alt).cloned(),
        rows.need(&[2], "hash --at coarse", &fork_coarse).cloned(),
    );
    if let (Some(m), Some(a)) = (&fm, &fa) {
        rows.eq(2, "CLI hash(alt) vs hash(main)", a, m);
    }
    if let (Some(m), Some(c)) = (&fm, &fc) {
        rows.check(2, c != m, || {
            format!("CLI hash(coarse) == hash(main) == {m}")
        });
    }
    rows.need(&[2], "smesh checkout alt -p $T/fork", &fork_checkout);
    if let (Some(after), Ok(h)) = (
        rows.need(&[2], "hash after checkout alt", &fork_after)
            .cloned(),
        &h_cli,
    ) {
        rows.eq(2, "hash after `checkout alt` vs H_cli", &after, h);
    }
    if let (Some(fmem), Ok(h)) = (
        rows.need(&[2], "smesh run fork.smesh --memory", &fork_mem)
            .cloned(),
        &h_api,
    ) {
        rows.eq(2, "fork.smesh final (main) hash vs H_api", &fmem, h);
    }
    if let Some(a) = rows.need(&[2], "in-process fork", &api) {
        rows.eq(
            2,
            "in process: hash(alt) vs hash(main)",
            &a.fork_alt,
            &a.state,
        );
        rows.check(2, a.fork_coarse != a.state, || {
            "in process: hash(coarse) == hash(main)".into()
        });
        rows.eq(
            2,
            "in process: coarse mesh.info.cells",
            &a.coarse_cells,
            &json!(125),
        );
        rows.eq(
            2,
            "in process: checkout main is bit-identical",
            &a.back_on_main,
            &a.state,
        );
    }

    // ================= row 3: cold replay reproduces H_cli under the strict policy
    if let (Some(v), Ok(h)) = (rows.need(&[3], "smesh hash -p $T/cli", &cli_hash), &h_cli) {
        let s = v["state"].as_str().unwrap_or("").to_owned();
        rows.eq(3, "hash -p (reopen, strict replay) vs H_cli", &s, h);
    }
    if let Some(a) = rows.need(&[3], "in process replay", &api) {
        rows.eq(
            3,
            "Workspace::state_from_root(head) vs H_api",
            &a.replay_root,
            &a.state,
        );
        rows.eq(
            3,
            "Workspace::state_at(head) vs H_api",
            &a.replay_state_at,
            &a.state,
        );
        rows.eq(3, "replayed layer hashes", &a.replay_layers, &a.layers);
    }

    // ================= row 4: node hashes
    let n_api = api.as_ref().map(|a| a.nodes.clone()).map_err(Clone::clone);
    let (nc, nm, na) = (
        rows.need(&[4], "CLI node list", &n_cli).cloned(),
        rows.need(&[4], "MCP node list", &n_mcp).cloned(),
        rows.need(&[4], "in-process node list", &n_api).cloned(),
    );
    if let (Some(c), Some(m)) = (&nc, &nm) {
        rows.eq(4, "pinned node lists CLI vs MCP", c, m);
    }
    if let (Some(m), Some(a)) = (&nm, &na) {
        rows.eq(4, "pinned node lists MCP vs in process", m, a);
    }
    let unpinned_vs = |rows: &mut Rows,
                       what: &str,
                       pinned: &Option<Vec<(String, String)>>,
                       un: &R<Vec<(String, String)>>| {
        if let (Some(p), Some(u)) = (pinned, rows.need(&[4], what, un).cloned()) {
            rows.eq(4, &format!("{what}: node count"), &p.len(), &u.len());
            for (i, (a, b)) in p.iter().zip(&u).enumerate() {
                rows.check(4, a.0 != b.0, || {
                    format!("{what}: node {i} equal pinned and unpinned ({})", a.0)
                });
                rows.eq(4, &format!("{what}: Check.state of node {i}"), &a.1, &b.1);
            }
        }
    };
    let n_mcpu = mcp_unpinned
        .as_ref()
        .map_err(Clone::clone)
        .and_then(mcp_nodes);
    unpinned_vs(&mut rows, "MCP unpinned", &nm, &n_mcpu);
    unpinned_vs(&mut rows, "CLI unpinned", &nc, &cliu_nodes);

    // ================= rows 5-7: exports
    let get = |rows: &mut Rows, row: &[u8], p: &Path| -> Option<Vec<u8>> {
        let r = read_bytes(p);
        rows.need(row, "export file", &r).cloned()
    };
    rows.need(&[5], "smesh export re.smsh -p", &re1);
    rows.need(&[5], "smesh export re.vtu -p", &re2);
    let vtu_cli = get(&mut rows, &[5, 6], &cli.join("out.vtu"));
    let vtu_mcp = get(&mut rows, &[5], &t.join("mcp.vtu"));
    let vtu_re = get(&mut rows, &[5], &re_vtu);
    let vtu_m1 = get(&mut rows, &[5, 6, 7], &m1.join("out.vtu"));
    let vtu_m2 = get(&mut rows, &[5, 7], &m2.join("out.vtu"));
    let vtu_m3 = get(&mut rows, &[5], &m3.join("out.vtu"));
    let smsh_cli = get(&mut rows, &[5, 6], &cli.join("out.smsh"));
    let smsh_mcp = get(&mut rows, &[5], &t.join("mcp.smsh"));
    let smsh_re = get(&mut rows, &[5], &re_smsh);
    let smsh_m1 = get(&mut rows, &[5, 6, 7], &m1.join("out.smsh"));
    let smsh_m2 = get(&mut rows, &[5, 7], &m2.join("out.smsh"));
    let smsh_m3 = get(&mut rows, &[5], &m3.join("out.smsh"));
    let same = |rows: &mut Rows, row: u8, what: &str, a: &Option<Vec<u8>>, b: &Option<Vec<u8>>| {
        if let (Some(a), Some(b)) = (a, b) {
            rows.check(row, a == b, || {
                format!("{what}: bytes differ ({} vs {} bytes)", a.len(), b.len())
            });
        }
    };
    let vref = vtu_m1.clone();
    same(&mut rows, 5, "out.vtu CLI(-p) vs memory", &vtu_cli, &vref);
    same(&mut rows, 5, "out.vtu MCP vs memory", &vtu_mcp, &vref);
    same(
        &mut rows,
        5,
        "re-export .vtu after replay vs memory",
        &vtu_re,
        &vref,
    );
    same(&mut rows, 5, "out.vtu unpinned vs pinned", &vtu_m3, &vref);
    let sref = smsh_m1.clone();
    same(
        &mut rows,
        5,
        "out.smsh CLI(-p) vs memory (pinned)",
        &smsh_cli,
        &sref,
    );
    same(
        &mut rows,
        5,
        "out.smsh MCP vs memory (pinned)",
        &smsh_mcp,
        &sref,
    );
    same(
        &mut rows,
        5,
        "re-export .smsh after replay vs memory (pinned)",
        &smsh_re,
        &sref,
    );
    if let (Some(u), Some(p)) = (&smsh_m3, &sref) {
        match (smsh_split(u), smsh_split(p)) {
            (Ok((mut hu, bu)), Ok((mut hp, bp))) => {
                // Unpinned: only the provenance may differ; header HashInfo and arrays must match.
                rows.eq(
                    5,
                    "unpinned .smsh header mesh.hashes",
                    &hu["mesh"]["hashes"],
                    &hp["mesh"]["hashes"],
                );
                if let (Some(a), Some(b)) = (hu.as_object_mut(), hp.as_object_mut()) {
                    a.remove("provenance");
                    b.remove("provenance");
                }
                rows.eq(5, "unpinned .smsh header without provenance", &hu, &hp);
                rows.check(5, bu == bp, || "unpinned .smsh arrays differ".into());
            }
            (a, b) => rows.fail(
                5,
                format!("unpinned .smsh decode: {:?} / {:?}", a.err(), b.err()),
            ),
        }
    }
    // row 7
    same(
        &mut rows,
        7,
        "out.vtu RAYON_NUM_THREADS=1 vs default",
        &vtu_m1,
        &vtu_m2,
    );
    same(
        &mut rows,
        7,
        "out.smsh RAYON_NUM_THREADS=1 vs default",
        &smsh_m1,
        &smsh_m2,
    );
    if let (Some(a), Some(b)) = (
        rows.need(&[7], "H_1t", &h_1t).cloned(),
        rows.need(&[7], "H_mem", &h_mem).cloned(),
    ) {
        rows.eq(7, "state hash 1 thread vs default", &a, &b);
    }
    if let (Ok(a), Ok(b)) = (&mem_layers_1t, &mem_layers) {
        rows.eq(7, "layer hashes 1 thread vs default", a, b);
    }
    if let (Some(u), Some(p)) = (
        rows.need(&[4], "H unpinned (m3)", &h_m3).cloned(),
        &h_mem.as_ref().ok(),
    ) {
        rows.eq(4, "unpinned run state hash vs pinned", &u, p);
    }

    // ================= row 6: content
    for (name, v) in [("memory", &vtu_m1), ("CLI -p", &vtu_cli)] {
        let Some(bytes) = v else { continue };
        match vtu_check::decode(bytes).and_then(|d| Ok((vtu_check::hex_facts(&d)?, d))) {
            Err(e) => rows.fail(6, format!("{name} out.vtu: decode: {e}")),
            Ok((f, d)) => {
                rows.eq(6, &format!("{name} vtu nodes"), &d.np, &1331);
                rows.eq(6, &format!("{name} vtu cells"), &d.nc, &1000);
                rows.check(6, f.all_hex8, || {
                    format!("{name}: not all cells are VTK_HEXAHEDRON")
                });
                rows.check(6, f.cells_valid, || {
                    format!("{name}: a cell has repeated or out-of-range corners")
                });
                rows.check(6, f.max_lattice_dev <= 1e-12, || {
                    format!("{name}: max lattice deviation {}", f.max_lattice_dev)
                });
                rows.check(6, f.lattice_bijective, || {
                    format!("{name}: points are not the lattice {{-5..5}}^3 one to one")
                });
                rows.check(6, (f.total_volume - 1000.0).abs() <= 1e-9, || {
                    format!("{name}: total volume {}", f.total_volume)
                });
                rows.check(6, (f.min_scaled_jacobian - 1.0).abs() <= 1e-12, || {
                    format!("{name}: min scaled Jacobian {}", f.min_scaled_jacobian)
                });
                let block = d
                    .arrays
                    .get("CellData/set:block_1")
                    .map(|a| a.values.iter().filter(|&&x| x == 1.0).count());
                rows.eq(
                    6,
                    &format!("{name} vtu set:block_1 members"),
                    &block,
                    &Some(1000),
                );
                // The stdlib Python decoder, and VTK when SM_PYTHON has it.
                vtu_python(&mut rows, name, &t, bytes, &d);
            }
        }
    }
    for (name, s) in [("memory", &smsh_m1), ("CLI -p", &smsh_cli)] {
        let Some(bytes) = s else { continue };
        match smsh_split(bytes) {
            Err(e) => rows.fail(6, format!("{name} out.smsh: {e}")),
            Ok((h, _)) => {
                rows.eq(
                    6,
                    &format!("{name} smsh n_vertices"),
                    &h["mesh"]["n_vertices"],
                    &json!(1331),
                );
                rows.eq(
                    6,
                    &format!("{name} smsh n_cells"),
                    &h["mesh"]["n_cells"],
                    &json!(1000),
                );
                let sets = h["sets"].as_array().cloned().unwrap_or_default();
                let block = sets.iter().find(|s| s["name"] == "block_1");
                match block {
                    None => rows.fail(6, format!("{name} smsh: no cell set block_1 in {sets:?}")),
                    Some(b) => {
                        rows.eq(
                            6,
                            &format!("{name} smsh block_1 label"),
                            &b["label"],
                            &json!(1),
                        );
                        rows.eq(
                            6,
                            &format!("{name} smsh block_1 role"),
                            &b["role"],
                            &json!("cell_set"),
                        );
                        let arr = h["arrays"]
                            .as_array()
                            .and_then(|a| a.iter().find(|x| x["name"] == b["array"]).cloned());
                        rows.eq(
                            6,
                            &format!("{name} smsh block_1 cells"),
                            &arr.map(|a| a["shape"].clone()),
                            &Some(json!([1000])),
                        );
                    }
                }
            }
        }
    }
    // Full decode with sm_meshio, `validate`, and the min scaled Jacobian via sm_quality.
    for (name, s) in [("memory", &smsh_m1), ("CLI -p", &smsh_cli)] {
        let Some(bytes) = s else { continue };
        match smsh_rebuild(bytes) {
            Err(e) => rows.fail(6, format!("{name} out.smsh decode/rebuild: {e}")),
            Ok(mesh) => {
                let v = sm_mesh_api::validate(&mesh);
                if !v.is_empty() {
                    rows.fail(
                        6,
                        format!(
                            "{name} out.smsh validate: {} violations, first {:?}",
                            v.len(),
                            v.first()
                        ),
                    );
                }
                match sm_quality::report(
                    &mesh,
                    &[sm_quality::Metric::ScaledJacobian],
                    &sm_quality::Options::default(),
                ) {
                    Err(e) => rows.fail(6, format!("{name} quality: {e}")),
                    Ok(r) => match r.first() {
                        Some((_, sum)) if sum.count == 1000 && (sum.min - 1.0).abs() < 1e-12 => {}
                        other => rows.fail(
                            6,
                            format!("{name} min scaled Jacobian != 1 over 1000 cells: {other:?}"),
                        ),
                    },
                }
            }
        }
    }

    // ================= row 8: reference vs production topology host
    rows.fail(
        8,
        "not evaluable: no public way to build the standard distribution over `CoreLayers::new(CellComplex)`; \
         `sm_server::distribution` hard-codes `TopoStore`, and assembling it by hand needs sm-layer-api, \
         sm-topo-api, sm-model, sm-mesher, sm-fem, sm-tess, sm-sizing, which sm-cli does not depend on",
    );

    // ---------------- report
    let mut report = String::new();
    for row in 1..=8u8 {
        let status = if rows.fails.contains_key(&row) {
            "FAIL"
        } else {
            "ok"
        };
        report.push_str(&format!("row {row}: {status}\n"));
        for f in rows.fails.get(&row).into_iter().flatten() {
            report.push_str(&format!("    - {f}\n"));
        }
        for n in rows.notes.get(&row).into_iter().flatten() {
            report.push_str(&format!("    . {n}\n"));
        }
    }
    eprintln!("{report}");
    assert!(rows.fails.is_empty(), "milestone rows failed:\n{report}");
}

/// Decode `bytes` with `vtu_check/vtu_check.py` (stdlib) and compare with the Rust decode; when
/// `SM_PYTHON` is set, VTK must load the file too.
fn vtu_python(rows: &mut Rows, name: &str, t: &Path, bytes: &[u8], d: &vtu_check::Vtu) {
    let explicit = std::env::var("SM_PYTHON").ok();
    let py = explicit.clone().unwrap_or_else(|| "python3".into());
    let runs = Command::new(&py)
        .arg("--version")
        .output()
        .is_ok_and(|o| o.status.success());
    if !runs {
        if explicit.is_some() {
            rows.fail(6, format!("SM_PYTHON={py} does not run"));
        } else {
            rows.note(6, "python3 not available: stdlib VTU decode skipped");
        }
        return;
    }
    let file = t.join(format!("py-{}.vtu", name.replace(' ', "_")));
    if std::fs::write(&file, bytes).is_err() {
        rows.fail(6, "cannot write the VTU copy for Python");
        return;
    }
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/vtu_check/vtu_check.py");
    let out = Command::new(&py).arg("-I").arg(&script).arg(&file).output();
    let j: Value = match out {
        Ok(o) if o.status.success() => match serde_json::from_slice(&o.stdout) {
            Ok(j) => j,
            Err(e) => return rows.fail(6, format!("vtu_check.py output: {e}")),
        },
        Ok(o) => {
            return rows.fail(
                6,
                format!(
                    "vtu_check.py failed: {}",
                    String::from_utf8_lossy(&o.stderr)
                ),
            );
        }
        Err(e) => return rows.fail(6, format!("vtu_check.py: {e}")),
    };
    rows.eq(6, &format!("{name} python np"), &j["np"], &json!(d.np));
    rows.eq(6, &format!("{name} python nc"), &j["nc"], &json!(d.nc));
    for (key, arr) in &d.arrays {
        let (sec, an) = key.split_once('/').unwrap_or(("", key));
        let got: Vec<f64> = j["sections"][sec][an]["values"]
            .as_array()
            .map(|v| v.iter().filter_map(Value::as_f64).collect())
            .unwrap_or_default();
        rows.check(6, got == arr.values, || {
            format!("{name}: python and Rust decodes of {key} differ")
        });
    }
    match j["vtk"].as_object() {
        Some(v) => {
            rows.eq(
                6,
                &format!("{name} VTK n_cells"),
                &v["n_cells"],
                &json!(1000),
            );
            let types_ok = v["types"]
                .as_array()
                .is_some_and(|a| a.iter().all(|x| x == 12));
            rows.check(6, types_ok, || format!("{name}: VTK sees non-hex cells"));
            let outward = v["outward"]
                .as_array()
                .is_some_and(|a| a.iter().all(|x| x == true));
            rows.check(6, outward, || format!("{name}: VTK finds an inward face"));
            let vol: f64 = v["volume"]
                .as_array()
                .map(|a| a.iter().filter_map(Value::as_f64).sum())
                .unwrap_or(0.0);
            rows.check(6, (vol - 1000.0).abs() <= 1e-9, || {
                format!("{name}: VTK total volume {vol}")
            });
        }
        None if explicit.is_some() => rows.fail(
            6,
            format!(
                "SM_PYTHON is set but VTK cannot be imported: {}",
                j["vtk_import_error"]
            ),
        ),
        None => rows.note(
            6,
            "VTK not importable from python3: VTK check skipped (set SM_PYTHON to require it)",
        ),
    }
}
