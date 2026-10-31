//! W3a integration milestone (plan `docs/design/w3/plan.md` §4 and rulings INT-1..6): the
//! three-volume part built two ways (path A: content-addressed STEP import; path B: webcut +
//! unite + imprint + merge), meshed hex / wedge / tet (hybrid) and all-tet + polyhedral dual
//! (tet branch), exported to Exodus, OpenFOAM, `.vtu` and `.smsh` through the CLI (§4.3), MCP
//! (§4.4) and in process (§4.5), then every assertion row 1-14 of §4.6.
//!
//! Collect-all-rows style (like `milestone.rs`): every row is evaluated, failures are recorded
//! per row and per path, and the test fails once at the end with the full report. Measured
//! values and wall times are printed as notes. Run it as
//! `cargo test -p sm-cli --test milestone_w3a -- --nocapture` (and with `RAYON_NUM_THREADS=1`).
//!
//! `emit_conformance_scripts` writes the `w3a_hybrid` and `w3a_tet` fixtures of
//! `tools/conformance.sh` (path B) to `$MILESTONE_W3A_EMIT` (default
//! `$CARGO_TARGET_TMPDIR/milestone_w3a`).

mod w3a_support;

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use serde_json::{Value, json};
use sm_core::EntityId;
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh_api::MeshRead;
use sm_op_api::{Hash256, Rev, State};
use sm_protocol::{Endpoint, ExportFormat, Request};
use sm_server::{ApplyOptions, FixedClock, InProcess, Server, Workspace};

use w3a_support::*;

// Row 8 scaled-Jacobian floors. The plan's initial values are hex 0.2, wedge 0.3, tet 0.3
// ("calibrated on the first run; tets against tri's 20 degree surface guarantee"). Calibration
// (first run, 2026-10-30, both paths, 1 and N threads identical): hex 1.0, wedge 0.670 (B) /
// 0.701 (A) after smoothing; tets of the unsmoothed hybrid 0.282 (B) / 0.307 (A), tets of the
// all-tet mesh `mt` 0.286 (both), tets of the hybrid after `mesh.smooth` (laplacian, 5 sweeps,
// fix_boundary false) 0.199 (B) / 0.253 (A). Hex and wedge keep the initial floors; the tet
// floor is lowered to 0.25 for unsmoothed tets and to 0.15 after smoothing (the guard allows
// any drop above `guard_floor`, checked separately).
// Re-run round 1 (2026-10-30, after sm-scheme-tet's `split_boundary_heavy` centroid split): tets
// of the unsmoothed hybrid 0.147 (A) / 0.146 (B), `mt` 0.147 (A) / 0.134 (B). The floors are NOT
// lowered: row 8 fails and the drop is reported to crates/sm-scheme-tet/.
// Re-run round 2 (2026-10-30, sm-poly/sm-quality/sm-smooth fixes merged; the sm-scheme-tet
// rework 5a69115 is not in this merge): unchanged, hybrid 0.147 (A) / 0.146 (B), `mt` 0.147 (A)
// / 0.134 (B), after smoothing 0.185 (A) / 0.154 (B). Floors unchanged; row 8 still fails.
/// Minimum scaled Jacobian of the hexes.
const SJ_HEX: f64 = 0.2;
/// Minimum scaled Jacobian of the wedges.
const SJ_WEDGE: f64 = 0.3;
/// Minimum scaled Jacobian of unsmoothed tets (initial 0.3, calibrated: measured 0.282).
const SJ_TET: f64 = 0.25;
/// Minimum scaled Jacobian of tets after the milestone's smoothing (measured 0.199).
const SJ_TET_SMOOTHED: f64 = 0.15;

/// sha256 of `sm_testkit::w3a::STEP` (pinned by the testkit).
fn step_sha() -> &'static str {
    sm_testkit::step::W3A_PART.sha256
}

fn scratch() -> PathBuf {
    let d = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("milestone_w3a-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("scratch dir");
    d
}

fn ts(p: &Path) -> String {
    p.to_string_lossy().into_owned()
}

// ------------------------------------------------------------------ conformance fixtures

#[test]
fn emit_conformance_scripts() {
    let dir = std::env::var_os("MILESTONE_W3A_EMIT")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_TARGET_TMPDIR")).join("milestone_w3a"));
    std::fs::create_dir_all(&dir).expect("emit dir");
    std::fs::write(dir.join("w3a_hybrid.smesh"), emit_hybrid()).expect("write w3a_hybrid");
    std::fs::write(dir.join("w3a_tet.smesh"), emit_tet()).expect("write w3a_tet");
    // The emitted scripts are self-contained (path B needs no input file).
    for s in [emit_hybrid(), emit_tet()] {
        assert!(!s.contains('@'), "unresolved placeholder in an emitted script");
    }
}

// ------------------------------------------------------------------ in process (§4.5)

struct Api {
    h0: Hash256,
    hybrid: State,
    presmooth: State,
    tet: State,
    hybrid_layers: BTreeMap<String, String>,
    tet_layers: BTreeMap<String, String>,
    /// (node, state) of the head + hybrid ops.
    nodes: Vec<(String, String)>,
    replay_root: String,
    replay_at: String,
    replay_tet: String,
    fork_alt: String,
    fork_coarse: String,
    back_main: String,
    tet2: String,
    m: EntityId,
    mt: EntityId,
    pm: EntityId,
    timings: Vec<(String, Duration)>,
    /// Queries on the final states.
    q: BTreeMap<String, R<Value>>,
    /// `mesh.flag_cells` on a branch: (set size, n_warn + n_fail, branch hash, hash after checkout).
    flag: R<(usize, usize, String, String)>,
    /// POLYHEDRON declared: export of `pm` (ok) and of `mt` (error kind / message).
    mismatch: R<(R<Value>, R<Value>)>,
}

fn id_of(s: &str) -> R<EntityId> {
    s.parse::<EntityId>().map_err(|x| format!("{s}: {x}"))
}

/// What an applied op left: its node, state hash and first `mesh` output.
struct Applied {
    node: Hash256,
    state: Hash256,
    mesh: Option<EntityId>,
}

fn apply_all(ws: &mut Workspace, ops: &[(String, Value)], timings: &mut Vec<(String, Duration)>, label: &str) -> R<Vec<Applied>> {
    let opts = ApplyOptions::default();
    let mut out = Vec::new();
    for (op, p) in ops {
        let t0 = Instant::now();
        let r = ws.apply_json(op, p.clone(), &opts).map_err(|x| format!("{label} {op}: {x}"))?;
        if op == "mesh.run" || op == "mesh.to_poly" || op == "mesh.smooth" {
            timings.push((format!("{label} {op}"), t0.elapsed()));
        }
        let mesh = r.delta.outputs.get("mesh").and_then(|v| v.first()).copied();
        out.push(Applied { node: r.node, state: r.state, mesh });
    }
    Ok(out)
}

fn hexes(m: &BTreeMap<String, Hash256>) -> BTreeMap<String, String> {
    m.iter().map(|(k, v)| (k.clone(), v.to_string())).collect()
}

fn out0(r: &Applied, role: &str) -> R<EntityId> {
    r.mesh.ok_or_else(|| format!("no `{role}` output"))
}

fn joined<T>(x: std::thread::Result<T>) -> R<T> {
    x.map_err(|_| "thread panicked".to_owned())
}

fn stage_into(ws: Workspace) -> R<Workspace> {
    let server = Server::from_workspace(ws);
    let got = server.stage_input(sm_testkit::w3a::STEP).map_err(e)?;
    if got.sha256.to_string().as_str() != step_sha() {
        return Err(format!("Server::stage_input sha256 {} != pinned {}", got.sha256, step_sha()));
    }
    Ok(server.into_workspace())
}

fn with_size(ops: &[(String, Value)], size: f64) -> Vec<(String, Value)> {
    ops.iter()
        .map(|(op, p)| {
            let mut p = p.clone();
            if op == "mesh.size" {
                p["size"] = json!(size);
            }
            (op.clone(), p)
        })
        .collect()
}

fn export_req(mesh: EntityId) -> Request {
    Request::Export { path: None, format: Some(ExportFormat::Exodus), mesh: Some(mesh), provenance: false, inline: true }
}

fn in_process(h: Head, ops: &Ops) -> R<Api> {
    let mut timings = Vec::new();
    let mut ws: Workspace = sm_server::standard().author(AUTHOR).clock(FixedClock(TIME.to_owned())).memory().map_err(e)?;
    if h == Head::A {
        ws = stage_into(ws)?;
    }
    let head = apply_all(&mut ws, &ops.head, &mut timings, "api")?;
    let h0 = head.last().ok_or("no head ops")?.node;
    let hyb = apply_all(&mut ws, &ops.hybrid, &mut timings, "api hybrid")?;
    let nodes: Vec<(String, String)> = head.iter().chain(&hyb).map(|r| (r.node.to_string(), r.state.to_string())).collect();
    let run_ix = ops.hybrid.iter().position(|(o, _)| o == "mesh.run").ok_or("no mesh.run")?;
    let m = out0(&hyb[run_ix], "mesh")?;
    let main_head = ws.session().position().ok_or("no head")?;
    let hybrid = ws.state().clone();
    let hybrid_layers = hexes(&hybrid.layer_hashes());
    let presmooth = ws.state_at(&Rev::Node(hyb[run_ix].node)).map_err(e)?;
    let replay_root = ws.state_from_root(main_head).map_err(e)?.hash().to_string();
    let replay_at = ws.state_at(&Rev::Node(main_head)).map_err(e)?.hash().to_string();
    // Queries on the hybrid state.
    let mut q = BTreeMap::new();
    let mut query = |ws: &Workspace, key: &str, name: &str, p: Value| {
        q.insert(key.to_owned(), ws.query(name, &p).map_err(e));
    };
    query(&ws, "validate_m", "mesh.validate", json!({"mesh": m.to_string()}));
    query(&ws, "quality_generic", "mesh.quality", json!({"mesh": m.to_string(), "profile": "generic"}));
    query(&ws, "quality_transport", "mesh.quality", json!({"mesh": m.to_string(), "profile": "transport"}));
    query(
        &ws,
        "quality_spread",
        "mesh.quality",
        json!({"mesh": m.to_string(), "metrics": ["scaled_jacobian"], "thresholds": {"scaled_jacobian": {"warn": 0.999}}}),
    );
    let flag_q = json!({"mesh": m.to_string(), "metrics": ["scaled_jacobian"], "thresholds": {"scaled_jacobian": {"warn": 0.6}}});
    query(&ws, "quality_flag", "mesh.quality", flag_q.clone());
    query(&ws, "faces_all", "topo.related", json!({"kind": "TopoFace", "within": []}));
    // Part facts (row 5): from the head's volumes.
    let main_hash = ws.state().hash().to_string();
    // Fork in process (row 2): alt == main, coarse != main, back on main.
    ws.fork("alt", Some(h0)).map_err(e)?;
    apply_all(&mut ws, &ops.hybrid, &mut timings, "api alt")?;
    let fork_alt = ws.state().hash().to_string();
    ws.fork("coarse", Some(h0)).map_err(e)?;
    apply_all(&mut ws, &with_size(&ops.hybrid, 2.0), &mut timings, "api coarse")?;
    let fork_coarse = ws.state().hash().to_string();
    ws.checkout(Rev::Ref("main".into())).map_err(e)?;
    let back_main = ws.state().hash().to_string();
    // flag_cells on a branch after smoothing (row 8).
    let flag = (|| -> R<(usize, usize, String, String)> {
        let qv = ws.query("mesh.quality", &flag_q).map_err(e)?;
        let nw: usize = qv["metrics"]
            .as_array()
            .ok_or("no metrics")?
            .iter()
            .map(|x| x["n_warn"].as_u64().unwrap_or(0) as usize + x["n_fail"].as_u64().unwrap_or(0) as usize)
            .sum();
        ws.fork("flag", Some(main_head)).map_err(e)?;
        let mut p = flag_q.clone();
        p["name"] = json!("q_warn");
        p["level"] = json!("warn");
        ws.apply_json("mesh.flag_cells", p, &ApplyOptions::default()).map_err(e)?;
        let layer = ws.state().layer::<MeshLayer>().ok_or("no meshes layer")?;
        let set = layer.get(m).ok_or("flagged mesh gone")?.mesh().set_by_name("q_warn").map(|s| s.members.len()).ok_or("no q_warn set")?;
        let flagged = ws.state().hash().to_string();
        ws.checkout(Rev::Ref("main".into())).map_err(e)?;
        Ok((set, nw, flagged, ws.state().hash().to_string()))
    })();
    // The tet branch.
    ws.fork("tet", Some(h0)).map_err(e)?;
    let tr = apply_all(&mut ws, &ops.tet, &mut timings, "api tet")?;
    let ri = ops.tet.iter().position(|(o, _)| o == "mesh.run").ok_or("no tet mesh.run")?;
    let pi = ops.tet.iter().position(|(o, _)| o == "mesh.to_poly").ok_or("no mesh.to_poly")?;
    let mt = out0(&tr[ri], "mesh")?;
    let pm = out0(&tr[pi], "mesh")?;
    let tet = ws.state().clone();
    let tet_layers = hexes(&tet.layer_hashes());
    let tet_head = ws.session().position().ok_or("no tet head")?;
    let replay_tet = ws.state_from_root(tet_head).map_err(e)?.hash().to_string();
    query(&ws, "validate_mt", "mesh.validate", json!({"mesh": mt.to_string()}));
    query(&ws, "validate_pm", "mesh.validate", json!({"mesh": pm.to_string()}));
    query(&ws, "poly_info", "mesh.poly_info", json!({"mesh": pm.to_string()}));
    ws.fork("tet2", Some(h0)).map_err(e)?;
    apply_all(&mut ws, &ops.tet, &mut timings, "api tet2")?;
    let tet2 = ws.state().hash().to_string();
    // POLYHEDRON declared on every block: pm exports, mt is an element mismatch (§4.2).
    ws.fork("tetpoly", Some(tet_head)).map_err(e)?;
    let poly_types: Vec<(String, Value)> =
        (1..=3).map(|b| ("fem.block_element_type".to_owned(), json!({"block": b, "element_type": "POLYHEDRON"}))).collect();
    let mismatch = apply_all(&mut ws, &poly_types, &mut timings, "api tetpoly").map(|_| ());
    let mut server = Server::from_workspace(ws);
    let mismatch = mismatch.map(|()| {
        let a = server.call(export_req(pm)).map_err(|x| format!("{}: {}", x.kind, x.message));
        let b = server.call(export_req(mt)).map_err(|x| format!("{}: {}", x.kind, x.message));
        (a, b)
    });
    drop(server);
    if hybrid.hash().to_string() != main_hash {
        return Err("State clone hash differs from the workspace hash".into());
    }
    Ok(Api {
        h0,
        hybrid,
        presmooth,
        tet,
        hybrid_layers,
        tet_layers,
        nodes,
        replay_root,
        replay_at,
        replay_tet,
        fork_alt,
        fork_coarse,
        back_main,
        tet2,
        m,
        mt,
        pm,
        timings,
        q,
        flag,
        mismatch,
    })
}

/// Row 14: the same ops over `reference_standard()` (CellComplex core), plus the main script
/// through a reference `Server`.
fn reference_core(h: Head, ops: &Ops, script: &str, script_path: &Path) -> R<(String, String, BTreeMap<String, String>)> {
    let pinned = || sm_server::reference_standard().author(AUTHOR).clock(FixedClock(TIME.to_owned()));
    let mut ws = pinned().memory().map_err(e)?;
    if h == Head::A {
        ws = stage_into(ws)?;
    }
    let mut t = Vec::new();
    apply_all(&mut ws, &ops.head, &mut t, "ref")?;
    let h0 = ws.session().position().ok_or("no head")?;
    apply_all(&mut ws, &ops.hybrid, &mut t, "ref hybrid")?;
    let hyb = ws.state().hash().to_string();
    ws.fork("tet", Some(h0)).map_err(e)?;
    apply_all(&mut ws, &ops.tet, &mut t, "ref tet")?;
    let tet = ws.state().hash().to_string();
    // The script through a reference server (memory, exports into the script's directory).
    let server = Server::new(pinned()).map_err(e)?;
    let mut ip = InProcess::new(server);
    let req = json!({"jsonrpc":"2.0","id":1,"method":"script.run","params":{"text": script, "path": ts(script_path)}});
    let reply: Value = serde_json::from_str(&ip.send_text(&req.to_string())).map_err(e)?;
    let res = reply.get("result").ok_or_else(|| format!("reference script.run: {}", reply["error"]))?;
    let out = printed(res["output"].as_str().unwrap_or(""));
    Ok((hyb, tet, out))
}

// ------------------------------------------------------------------ the CLI / MCP side

struct Cli {
    run: Value,
    printed: BTreeMap<String, String>,
    ops: Ops,
    hash: R<Value>,
    hash_tet: R<Value>,
    log_nodes: R<Vec<(String, String)>>,
    cold: R<(String, String)>,
    re_exo: R<Value>,
    re_case: R<Value>,
    validate: R<(i32, Value)>,
    quality: R<Value>,
    stage: Option<R<Value>>,
    wall: Duration,
}

/// (state, printed lines) of a script run.
type MemRun = (String, BTreeMap<String, String>);

/// (state, printed) of a `smesh run ... --json` in a directory.
fn mem_run(script: &Path, dir: &Path, pin: bool, threads: Threads) -> R<MemRun> {
    std::fs::create_dir_all(dir).map_err(e)?;
    let s = ts(script);
    let mut a = vec!["run", s.as_str(), "--memory", "--json"];
    if pin {
        a.extend(PIN);
    }
    let v = ok_json(&a, Some(dir), threads)?;
    Ok((hash_of(&v["state"], "state")?, printed(v["output"].as_str().unwrap_or(""))))
}

/// `script.run` + `session.hash` (main and at `tet`) through `smesh serve --stdio --memory`.
fn serve_layers(script: &Path, dir: &Path, threads: Threads) -> R<[(String, BTreeMap<String, String>); 2]> {
    std::fs::create_dir_all(dir).map_err(e)?;
    let req = format!(
        "{}\n{}\n{}\n",
        json!({"jsonrpc":"2.0","id":1,"method":"script.run","params":{"text": read_text(script), "path": ts(script)}}),
        json!({"jsonrpc":"2.0","id":2,"method":"session.hash"}),
        json!({"jsonrpc":"2.0","id":3,"method":"session.hash","params":{"at":{"ref":"tet"}}})
    );
    let mut a = vec!["serve", "--stdio", "--memory"];
    a.extend(PIN);
    let o = spawn(&a, Some(dir), threads, Some(&req))?;
    if o.status.code() != Some(0) {
        return Err(describe(&a, &o));
    }
    let mut by_id = BTreeMap::new();
    for l in String::from_utf8_lossy(&o.stdout).lines().filter(|l| !l.trim().is_empty()) {
        let v: Value = serde_json::from_str(l).map_err(|x| format!("serve reply: {x}"))?;
        by_id.insert(v["id"].as_i64().unwrap_or(-1), v);
    }
    let get = |id: i64| -> R<(String, BTreeMap<String, String>)> {
        let v = by_id.get(&id).ok_or_else(|| format!("serve: no reply {id}"))?;
        let r = v.get("result").ok_or_else(|| format!("serve {id}: {}", v["error"]))?;
        Ok((hash_of(&r["state"], "state")?, layers_of(r)?))
    };
    by_id.get(&1).and_then(|v| v.get("result")).ok_or_else(|| format!("serve script.run failed: {:?}", by_id.get(&1)))?;
    Ok([get(2)?, get(3)?])
}

fn cli_project(h: Head, dir: &Path, script: &Path, step: &Path) -> R<Cli> {
    let proj = dir.join("proj");
    let p = ts(&proj);
    ok_text(&["new", &p], None, Threads::Inherit)?;
    let s = ts(script);
    let mut a = vec!["run", s.as_str(), "-p", p.as_str()];
    a.extend(PIN);
    a.push("--json");
    let t0 = Instant::now();
    let run = ok_json(&a, None, Threads::Inherit)?;
    let wall = t0.elapsed();
    let printed = printed(run["output"].as_str().unwrap_or(""));
    let ops = split_ops(&run)?;
    let hash = ok_json(&["hash", "-p", &p, "--json"], None, Threads::Inherit);
    let hash_tet = ok_json(&["hash", "-p", &p, "--at", "tet", "--json"], None, Threads::Inherit);
    let log_nodes = ok_json(&["log", "-p", &p, "-n", "1000", "--json"], None, Threads::Inherit).and_then(|log| {
        let mut nodes: Vec<String> =
            log["nodes"].as_array().ok_or("log has no nodes")?.iter().map(|n| hash_of(&n["node"], "node")).collect::<R<_>>()?;
        nodes.reverse();
        nodes
            .into_iter()
            .map(|n| {
                let hv = ok_json(&["hash", "--at", &n, "-p", &p, "--json"], None, Threads::Inherit)?;
                Ok((n, hash_of(&hv["state"], "state")?))
            })
            .collect()
    });
    // Cold replay from a copy of the project directory only.
    let copy = dir.join("proj_copy");
    let cold = copy_dir(&proj, &copy).and_then(|()| {
        let c = ts(&copy);
        let a = ok_json(&["hash", "-p", &c, "--json"], None, Threads::Inherit)?;
        let b = ok_json(&["hash", "-p", &c, "--at", "tet", "--json"], None, Threads::Inherit)?;
        Ok((hash_of(&a["state"], "state")?, hash_of(&b["state"], "state")?))
    });
    let re_exo = ok_json(&["export", &ts(&proj.join("re.exo")), "-p", &p, "--json"], None, Threads::Inherit);
    let re_case = ok_json(&["export", &ts(&proj.join("recase")), "--format", "openfoam", "-p", &p, "--json"], None, Threads::Inherit);
    let validate = any_json(&["validate", "-p", &p, "--json"], None, Threads::Inherit);
    let quality = ok_json(&["quality", "-p", &p, "--profile", "transport", "--json"], None, Threads::Inherit);
    let stage = (h == Head::A).then(|| ok_json(&["stage", &ts(step), "--memory", "--json"], None, Threads::Inherit));
    Ok(Cli { run, printed, ops, hash, hash_tet, log_nodes, cold, re_exo, re_case, validate, quality, stage, wall })
}

struct Fork {
    printed: BTreeMap<String, String>,
    alt: R<String>,
    coarse: R<String>,
    main: R<String>,
    tet2: R<String>,
    polyblocks: bool,
}

fn cli_fork(dir: &Path, script: &Path) -> R<Fork> {
    let proj = dir.join("forkproj");
    let p = ts(&proj);
    ok_text(&["new", &p], None, Threads::Inherit)?;
    let s = ts(script);
    let mut a = vec!["run", s.as_str(), "-p", p.as_str()];
    a.extend(PIN);
    a.push("--json");
    let run = ok_json(&a, None, Threads::Inherit)?;
    let state = |extra: &[&str]| -> R<String> {
        let mut a = vec!["hash", "-p", p.as_str(), "--json"];
        a.extend_from_slice(extra);
        hash_of(&ok_json(&a, None, Threads::Inherit)?["state"], "state")
    };
    let alt = ok_text(&["checkout", "alt", "-p", &p], None, Threads::Inherit).and_then(|_| state(&[]));
    let coarse = state(&["--at", "coarse"]);
    let tet2 = state(&["--at", "tet2"]);
    let main = ok_text(&["checkout", "main", "-p", &p], None, Threads::Inherit).and_then(|_| state(&[]));
    Ok(Fork { printed: printed(run["output"].as_str().unwrap_or("")), alt, coarse, main, tet2, polyblocks: proj.join("polyblocks.exo").is_file() })
}

/// The MCP lines of one path: the recorded ops as tool calls, the queries, exports, the tet
/// branch and back. Returns the lines and the ids of interest.
struct McpPlan {
    lines: Vec<Value>,
    op_ids: Vec<i64>,
    hash_hybrid: i64,
    hash_tet: i64,
    hash_back: i64,
    exports: Vec<(i64, String)>,
    validate: i64,
    quality: i64,
}

fn mcp_plan(h: Head, ops: &Ops, out: &Path, step: &Path) -> McpPlan {
    let mut lines = mcp_init();
    let mut id = 10;
    let mut next = || {
        id += 1;
        id
    };
    let mut op_ids = Vec::new();
    if h == Head::A {
        lines.push(tool_call(next(), "session_stage", json!({"path": ts(step)})));
    }
    for (op, p) in &ops.head {
        let i = next();
        op_ids.push(i);
        lines.push(tool_call(i, &tool_name(op), p.clone()));
    }
    // The tet branch starts at the head (h0); created here so unpinned runs need no node hash.
    lines.push(tool_call(next(), "timeline_branch", json!({"name": "tet"})));
    lines.push(tool_call(next(), "timeline_checkout", json!({"rev": "main"})));
    for (op, p) in &ops.hybrid {
        let i = next();
        op_ids.push(i);
        lines.push(tool_call(i, &tool_name(op), p.clone()));
    }
    let hash_hybrid = next();
    lines.push(tool_call(hash_hybrid, "session_hash", json!({})));
    lines.push(tool_call(next(), "topo__related", json!({"kind": "TopoFace", "within": ["TopoVolume#1"]})));
    let validate = next();
    lines.push(tool_call(validate, "mesh__validate", json!({})));
    let quality = next();
    lines.push(tool_call(quality, "mesh__quality", json!({"profile": "transport"})));
    let mut exports = Vec::new();
    let mut export = |lines: &mut Vec<Value>, i: i64, name: &str, extra: Value| {
        let mut a = json!({"path": ts(&out.join(name))});
        if let (Some(o), Some(x)) = (a.as_object_mut(), extra.as_object()) {
            o.extend(x.clone());
        }
        lines.push(tool_call(i, "session_export", a));
        exports.push((i, name.to_owned()));
    };
    export(&mut lines, next(), "out.exo", json!({"format": "exodus"}));
    export(&mut lines, next(), "out.smsh", json!({"provenance": true}));
    export(&mut lines, next(), "out.vtu", json!({}));
    export(&mut lines, next(), "case", json!({"format": "openfoam"}));
    lines.push(tool_call(next(), "timeline_checkout", json!({"rev": "tet"})));
    let mut tet_ids = Vec::new();
    for (op, p) in &ops.tet {
        let i = next();
        tet_ids.push(i);
        lines.push(tool_call(i, &tool_name(op), p.clone()));
    }
    let hash_tet = next();
    lines.push(tool_call(hash_tet, "session_hash", json!({})));
    // Mesh ids of the tet branch: the first mesh.run output and the to_poly output; the MCP
    // side uses the ids the CLI recorded (the same ops give the same ids).
    let mt = ops.tet.iter().find(|(o, _)| o == "mesh.to_poly").and_then(|(_, p)| p["mesh"].as_str()).unwrap_or("Mesh#1").to_owned();
    let pm = next_mesh(&mt);
    export(&mut lines, next(), "tet.exo", json!({"format": "exodus", "mesh": mt}));
    export(&mut lines, next(), "tetcase", json!({"format": "openfoam", "mesh": mt}));
    export(&mut lines, next(), "poly.exo", json!({"format": "exodus", "mesh": pm}));
    export(&mut lines, next(), "poly.vtu", json!({"mesh": pm}));
    export(&mut lines, next(), "poly.smsh", json!({"mesh": pm}));
    export(&mut lines, next(), "polycase", json!({"format": "openfoam", "mesh": pm}));
    lines.push(tool_call(next(), "mesh__poly_info", json!({"mesh": pm})));
    lines.push(tool_call(next(), "timeline_checkout", json!({"rev": "main"})));
    let hash_back = next();
    lines.push(tool_call(hash_back, "session_hash", json!({})));
    let _ = tet_ids;
    McpPlan { lines, op_ids, hash_hybrid, hash_tet, hash_back, exports, validate, quality }
}

/// `Mesh#n` -> `Mesh#(n+1)` (the poly mesh is minted right after its source in the tet branch;
/// checked against the CLI's printed ids).
fn next_mesh(id: &str) -> String {
    id.strip_prefix("Mesh#").and_then(|n| n.parse::<u64>().ok()).map_or_else(|| id.to_owned(), |n| format!("Mesh#{}", n + 1))
}

// ------------------------------------------------------------------ export comparisons

const HYBRID_FILES: [&str; 3] = ["out.exo", "out.vtu", "out.smsh"];
const TET_FILES: [&str; 4] = ["tet.exo", "poly.exo", "poly.vtu", "poly.smsh"];
const CASES: [&str; 3] = ["case", "tetcase", "polycase"];

/// Every export of one run directory: name -> bytes (cases flattened as `case/constant/...`).
fn exports_in(dir: &Path) -> R<BTreeMap<String, Vec<u8>>> {
    let mut out = BTreeMap::new();
    for n in HYBRID_FILES.iter().chain(&TET_FILES) {
        out.insert((*n).to_owned(), read_bytes(&dir.join(n))?);
    }
    for c in CASES {
        for (k, v) in tree(&dir.join(c))? {
            out.insert(format!("{c}/{k}"), v);
        }
    }
    Ok(out)
}

/// `.smsh` bytes equal except the header's `provenance`.
fn smsh_equal_but_provenance(a: &[u8], b: &[u8]) -> bool {
    let split = |x: &[u8]| -> Option<(Value, Vec<u8>)> {
        let n = u64::from_le_bytes(x.get(8..16)?.try_into().ok()?) as usize;
        let mut v: Value = serde_json::from_slice(x.get(16..16 + n)?).ok()?;
        v.as_object_mut()?.remove("provenance");
        Some((v, x[16 + n..].to_vec()))
    };
    matches!((split(a), split(b)), (Some(x), Some(y)) if x == y)
}

/// Compare two export sets; `out.smsh` may differ only in provenance when `unpinned`.
fn compare_exports(rows: &mut Tagged<'_>, what: &str, a: &BTreeMap<String, Vec<u8>>, b: &BTreeMap<String, Vec<u8>>, unpinned: bool) {
    let ka: Vec<&String> = a.keys().collect();
    let kb: Vec<&String> = b.keys().collect();
    if ka != kb {
        rows.fail(4, format!("{what}: file lists differ: {ka:?} vs {kb:?}"));
        return;
    }
    for (k, x) in a {
        let y = &b[k];
        if x == y {
            continue;
        }
        if unpinned && k == "out.smsh" && smsh_equal_but_provenance(x, y) {
            continue;
        }
        rows.fail(4, format!("{what}: {k} differs ({} vs {} bytes)", x.len(), y.len()));
    }
}

/// The `ExportResult.files` manifest hash: SHA-256 of the sorted `name\tsha256\n` lines.
fn check_manifest(rows: &mut Tagged<'_>, what: &str, res: &Value, dir: Option<&Path>) {
    let Some(files) = res["files"].as_array() else {
        rows.fail(4, format!("{what}: no `files` in {res}"));
        return;
    };
    let mut lines: Vec<String> = files.iter().map(|f| format!("{}\t{}\n", f["name"].as_str().unwrap_or(""), f["sha256"].as_str().unwrap_or(""))).collect();
    lines.sort();
    let want = sha_hex(lines.concat().as_bytes());
    rows.eq(4, &format!("{what}: manifest sha256"), &res["sha256"].as_str().unwrap_or("").to_owned(), &want);
    if let Some(d) = dir {
        for f in files {
            let name = f["name"].as_str().unwrap_or("");
            match read_bytes(&d.join(name)) {
                Ok(b) => rows.eq(4, &format!("{what}: sha256 of {name}"), &f["sha256"].as_str().unwrap_or("").to_owned(), &sha_hex(&b)),
                Err(x) => rows.fail(4, format!("{what}: {x}")),
            }
        }
    }
}

// ------------------------------------------------------------------ one path

#[derive(Default)]
struct PathFacts {
    counts: BTreeMap<String, usize>,
}

fn layer_check(rows: &mut Tagged<'_>, what: &str, l: &BTreeMap<String, String>) {
    let keys: BTreeSet<&str> = l.keys().map(String::as_str).collect();
    let want: BTreeSet<&str> = LAYERS.into_iter().collect();
    if keys != want {
        let extra: Vec<&&str> = keys.difference(&want).collect();
        let missing: Vec<&&str> = want.difference(&keys).collect();
        rows.note(1, format!("{what}: layer set deviates from {LAYERS:?}: extra {extra:?}, missing {missing:?}"));
    }
}

fn mesh_of(state: &State, id: EntityId) -> R<&dyn MeshRead> {
    let layer = state.layer::<MeshLayer>().ok_or("no meshes layer")?;
    Ok(layer.get(id).ok_or_else(|| format!("{id} not in the meshes layer"))?.mesh())
}

#[allow(clippy::too_many_lines)]
fn run_path(h: Head, t: &Path, step: &Path, all: &mut Rows, facts: &mut PathFacts, timings: &mut Vec<String>) {
    let mut rows = Tagged { rows: all, tag: h.tag() };
    let dir = t.join(h.lower());
    let _ = std::fs::create_dir_all(&dir);
    let script = dir.join(format!("w3a_{}.smesh", h.lower()));
    let fork_path = dir.join(format!("w3a_fork_{}.smesh", h.lower()));
    let main_text = main_script(h, step);
    let fork_text = fork_script(h, step);
    if let Err(x) = std::fs::write(&script, &main_text).and_then(|()| std::fs::write(&fork_path, &fork_text)) {
        rows.fail(1, format!("write scripts: {x}"));
        return;
    }
    let all_rows: Vec<u8> = (1..=14).collect();

    // ---------------- §4.3 CLI project (gives the op records everything else replays)
    let cli = cli_project(h, &dir, &script, step);
    let Some(cli) = rows.need(&all_rows, "CLI project run", &cli) else { return };
    timings.push(format!("[{}] CLI `smesh run` of the main script (project, debug build): {:.2?}", h.tag(), cli.wall));
    let h_cli = hash_of(&cli.run["state"], "state").unwrap_or_default();
    let h0 = cli.printed.get("h0").cloned().unwrap_or_default();
    let h_cli_tet = cli.printed.get("tet").cloned().unwrap_or_default();
    rows.eq(1, "CLI printed hybrid hash vs ScriptResult.state (on main)", &cli.printed.get("hybrid").cloned().unwrap_or_default(), &h_cli);

    // ---------------- the independent runs, concurrently
    let (mems, serves, fork, mcp_p, mcp_u, mcp_fork, api, refc) = std::thread::scope(|s| {
        let mem = |name: &'static str, pin: bool, th: Threads| {
            let d = dir.join(name);
            let sc = script.clone();
            s.spawn(move || (d.clone(), mem_run(&sc, &d, pin, th)))
        };
        let m1 = mem("m1", true, Threads::One);
        let m2 = mem("m2", true, Threads::Default);
        let m3 = mem("m3", false, Threads::Inherit);
        let s1 = { let (d, sc) = (dir.join("s1"), script.clone()); s.spawn(move || serve_layers(&sc, &d, Threads::One)) };
        let s2 = { let (d, sc) = (dir.join("s2"), script.clone()); s.spawn(move || serve_layers(&sc, &d, Threads::Default)) };
        let fk = { let (d, sc) = (dir.clone(), fork_path.clone()); s.spawn(move || cli_fork(&d, &sc)) };
        let mcp = |name: &'static str, pin: bool| {
            let out = dir.join(name);
            let _ = std::fs::create_dir_all(&out);
            let plan = mcp_plan(h, &cli.ops, &out, step);
            s.spawn(move || {
                let extra: &[&str] = if pin { &PIN } else { &[] };
                (plan_ids(&plan), out.clone(), mcp_session(extra, &plan.lines, Threads::Inherit, Some(&out)))
            })
        };
        let mp = mcp("mcp", true);
        let mu = mcp("mcpu", false);
        let mf = {
            let text = fork_text.clone();
            let fp = ts(&fork_path);
            let out = dir.join("mcpfork");
            let _ = std::fs::create_dir_all(&out);
            s.spawn(move || {
                let mut lines = mcp_init();
                lines.push(tool_call(2, "run_script", json!({"text": text, "path": fp})));
                mcp_session(&PIN, &lines, Threads::Inherit, Some(&out)).and_then(|r| tool_ok(&r, 2))
            })
        };
        let rc = {
            // Without the exports: an in-memory server would write them into the test's cwd.
            let txt: String = main_text.lines().filter(|l| !l.starts_with("session.export")).map(|l| format!("{l}\n")).collect();
            let (ops, sc) = (cli.ops.clone(), dir.join("refcore.smesh"));
            s.spawn(move || reference_core(h, &ops, &txt, &sc))
        };
        let api = in_process(h, &cli.ops);
        (
            [joined(m1.join()), joined(m2.join()), joined(m3.join())],
            [joined(s1.join()).and_then(|x| x), joined(s2.join()).and_then(|x| x)],
            joined(fk.join()).and_then(|x| x),
            joined(mp.join()),
            joined(mu.join()),
            joined(mf.join()).and_then(|x| x),
            api,
            joined(rc.join()).and_then(|x| x),
        )
    });
    let api = rows.need(&[1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14], "in-process §4.5", &api);

    // ---------------- row 1: hashes across front ends
    let mem_res: Vec<R<(PathBuf, MemRun)>> = mems
        .into_iter()
        .map(|r| r.and_then(|(d, x)| x.map(|v| (d, v))))
        .collect();
    let names = ["H_1t (memory, RAYON_NUM_THREADS=1)", "H_mem (memory, default threads)", "H_unpinned (memory)"];
    for (k, r) in mem_res.iter().enumerate() {
        let row: &[u8] = if k == 2 { &[3] } else { &[1, 4] };
        if let Some((_, (st, pr))) = rows.need(row, names[k], r) {
            rows.eq(row[0], &format!("{} vs H_cli (hybrid)", names[k]), st, &h_cli);
            rows.eq(row[0], &format!("{} vs H_cli (tet)", names[k]), &pr.get("tet").cloned().unwrap_or_default(), &h_cli_tet);
        }
    }
    if let Ok(hv) = &cli.hash {
        rows.eq(1, "smesh hash -p vs run state", &hash_of(&hv["state"], "state").unwrap_or_default(), &h_cli);
    }
    let cli_layers = cli.hash.as_ref().map_err(Clone::clone).and_then(layers_of);
    let cli_layers_tet = cli.hash_tet.as_ref().map_err(Clone::clone).and_then(layers_of);
    if let Ok(hv) = &cli.hash_tet {
        rows.eq(1, "smesh hash --at tet vs printed tet", &hash_of(&hv["state"], "state").unwrap_or_default(), &h_cli_tet);
    }
    if let (Ok(l), Ok(lt)) = (&cli_layers, &cli_layers_tet) {
        layer_check(&mut rows, "CLI layers", l);
        for (k, r) in serves.iter().enumerate() {
            match r {
                Ok([(sh, sl), (tt, tl)]) => {
                    rows.eq(1, &format!("serve {k} hybrid state"), sh, &h_cli);
                    rows.eq(1, &format!("serve {k} tet state"), tt, &h_cli_tet);
                    rows.eq(1, &format!("serve {k} hybrid layers"), sl, l);
                    rows.eq(1, &format!("serve {k} tet layers"), tl, lt);
                }
                Err(x) => rows.fail(1, format!("serve {k}: {x}")),
            }
        }
        if let Some(api) = api {
            rows.eq(1, "H_api (hybrid)", &api.hybrid.hash().to_string(), &h_cli);
            rows.eq(1, "H_api (tet)", &api.tet.hash().to_string(), &h_cli_tet);
            rows.eq(1, "api hybrid layers", &api.hybrid_layers, l);
            rows.eq(1, "api tet layers", &api.tet_layers, lt);
            rows.eq(1, "api h0 vs CLI printed h0", &api.h0.to_string(), &h0);
        }
    }

    // ---------------- MCP
    let mut mcp_nodes: Option<Vec<(String, String)>> = None;
    for (label, pinned, r) in [("MCP pinned", true, &mcp_p), ("MCP unpinned", false, &mcp_u)] {
        let Some((ids, out, replies)) = rows.need(&[1, 3, 4], label, r) else { continue };
        let Some(replies) = rows.need(&[1, 3, 4], label, replies) else { continue };
        let st = |id: i64| tool_ok(replies, id).and_then(|v| hash_of(&v["state"], "state"));
        let row = if pinned { 1 } else { 3 };
        match st(ids.hash_hybrid) {
            Ok(x) => rows.eq(row, &format!("{label} H_mcp (hybrid)"), &x, &h_cli),
            Err(x) => rows.fail(row, format!("{label} session_hash: {x}")),
        }
        match st(ids.hash_tet) {
            Ok(x) => rows.eq(row, &format!("{label} H_mcp (tet)"), &x, &h_cli_tet),
            Err(x) => rows.fail(row, format!("{label} session_hash (tet): {x}")),
        }
        match st(ids.hash_back) {
            Ok(x) => rows.eq(2, &format!("{label} back on main"), &x, &h_cli),
            Err(x) => rows.fail(2, format!("{label} session_hash (back): {x}")),
        }
        if pinned {
            if let (Ok(v), Ok(l)) = (tool_ok(replies, ids.hash_hybrid), &cli_layers) {
                match layers_of(&v) {
                    Ok(ml) => rows.eq(1, "MCP layers (hybrid)", &ml, l),
                    Err(x) => rows.fail(1, format!("MCP layers: {x}")),
                }
            }
            let nodes: R<Vec<(String, String)>> = ids
                .op_ids
                .iter()
                .map(|&i| {
                    let v = tool_ok(replies, i)?;
                    Ok((hash_of(&v["node"], "node")?, hash_of(&v["state"], "state")?))
                })
                .collect();
            match nodes {
                Ok(n) => mcp_nodes = Some(n),
                Err(x) => rows.fail(3, format!("MCP op replies: {x}")),
            }
            match tool_ok(replies, ids.validate) {
                Ok(v) => rows.check(6, v["valid"] == json!(true), || format!("MCP mesh__validate: valid {} (by_code {})", v["valid"], v["by_code"])),
                Err(x) => rows.fail(6, format!("MCP mesh__validate: {x}")),
            }
            if let Err(x) = tool_ok(replies, ids.quality) {
                rows.fail(8, format!("MCP mesh__quality: {x}"));
            }
        }
        for (i, name) in &ids.exports {
            match tool_ok(replies, *i) {
                Ok(v) if name.ends_with("case") => check_manifest(&mut rows, &format!("{label} {name}"), &v, Some(&out.join(name))),
                Ok(_) => {}
                Err(x) => rows.fail(4, format!("{label} export {name}: {x}")),
            }
        }
    }
    match &mcp_fork {
        Ok(v) => {
            let pr = printed(v["output"].as_str().unwrap_or(""));
            rows.eq(2, "MCP run_script fork: printed hybrid", &pr.get("hybrid").cloned().unwrap_or_default(), &h_cli);
            rows.eq(2, "MCP run_script fork: printed tet", &pr.get("tet").cloned().unwrap_or_default(), &h_cli_tet);
        }
        Err(x) => rows.fail(2, format!("MCP run_script of the fork script (fresh process): {x}")),
    }

    // ---------------- row 2: fork
    match &fork {
        Ok(f) => {
            rows.eq(2, "fork script printed hybrid vs H_cli", &f.printed.get("hybrid").cloned().unwrap_or_default(), &h_cli);
            rows.eq(2, "fork script printed tet vs H_cli tet", &f.printed.get("tet").cloned().unwrap_or_default(), &h_cli_tet);
            rows.eq(2, "smesh checkout alt && smesh hash", f.alt.as_ref().unwrap_or(&String::new()), &h_cli);
            rows.check(2, f.coarse.as_ref().is_ok_and(|c| *c != h_cli), || format!("coarse {:?} must differ from main {h_cli}", f.coarse));
            rows.eq(2, "checkout main", f.main.as_ref().unwrap_or(&String::new()), &h_cli);
            rows.eq(2, "tet2 == tet", f.tet2.as_ref().unwrap_or(&String::new()), &h_cli_tet);
            rows.check(2, f.polyblocks, || "POLYHEDRON sub-branch did not export polyblocks.exo".into());
        }
        Err(x) => rows.fail(2, format!("fork script: {x}")),
    }
    if let Some(api) = api {
        rows.eq(2, "Rust fork alt == main", &api.fork_alt, &h_cli);
        rows.check(2, api.fork_coarse != h_cli, || "Rust fork coarse == main".into());
        rows.eq(2, "Rust checkout main", &api.back_main, &h_cli);
        rows.eq(2, "Rust tet2 == tet", &api.tet2, &h_cli_tet);
        match &api.mismatch {
            Ok((pm_ok, mt_err)) => {
                rows.check(2, pm_ok.is_ok(), || format!("POLYHEDRON blocks: export of pm failed: {pm_ok:?}"));
                rows.check(2, matches!(mt_err, Err(m) if m.contains("element-mismatch")), || {
                    format!("POLYHEDRON blocks: export of mt must fail with element-mismatch, got {:?}", mt_err.as_ref().map(|_| "ok"))
                });
            }
            Err(x) => rows.fail(2, format!("POLYHEDRON sub-branch: {x}")),
        }
    }

    // ---------------- row 3: replay, node lists, records
    match &cli.cold {
        Ok((a, b)) => {
            rows.eq(3, "cold replay (copied project) hybrid", a, &h_cli);
            rows.eq(3, "cold replay (copied project) tet", b, &h_cli_tet);
        }
        Err(x) => rows.fail(3, format!("cold replay: {x}")),
    }
    if let Some(api) = api {
        rows.eq(3, "Workspace::state_from_root", &api.replay_root, &h_cli);
        rows.eq(3, "Workspace::state_at", &api.replay_at, &h_cli);
        rows.eq(3, "state_from_root (tet)", &api.replay_tet, &h_cli_tet);
    }
    match (&cli.log_nodes, &mcp_nodes) {
        (Ok(c), Some(m)) => {
            rows.eq(3, "CLI vs MCP node lists (pinned)", c, m);
            if let Some(api) = api {
                rows.eq(3, "CLI vs in-process node lists (pinned)", c, &api.nodes);
            }
        }
        (Err(x), _) => rows.fail(3, format!("CLI log: {x}")),
        _ => {}
    }
    if h == Head::A {
        let rec = cli.ops.head.iter().find(|(o, _)| o == "model.import_step").map(|(_, p)| p.clone());
        rows.eq(3, "recorded model.import_step params", &rec, &Some(json!({"source": {"sha256": step_sha()}})));
        // No file of the project mentions the staged file's path.
        match tree(&dir.join("proj")) {
            Ok(files) => {
                let needle = ts(step);
                let hits: Vec<&String> = files.iter().filter(|(_, b)| String::from_utf8_lossy(b).contains(&needle)).map(|(k, _)| k).collect();
                rows.check(3, hits.is_empty(), || format!("project files mention the STEP path: {hits:?}"));
                rows.check(3, files.keys().any(|k| k.starts_with("inputs/") && k.contains(step_sha())), || "project has no inputs/<sha256> blob".into());
            }
            Err(x) => rows.fail(3, x),
        }
        match &cli.stage {
            Some(Ok(v)) => rows.eq(3, "smesh stage sha256", &v["sha256"].as_str().unwrap_or("").to_owned(), &step_sha().to_owned()),
            Some(Err(x)) => rows.fail(3, format!("smesh stage: {x}")),
            None => {}
        }
    }

    // ---------------- row 4: export bytes
    let proj = dir.join("proj");
    let base = exports_in(&proj);
    if let Some(base) = rows.need(&[4], "CLI project exports", &base) {
        for (k, r) in mem_res.iter().enumerate() {
            if let Ok((d, _)) = r {
                match exports_in(d) {
                    Ok(x) => compare_exports(&mut rows, &format!("CLI vs {}", names[k]), base, &x, k == 2),
                    Err(x) => rows.fail(4, x),
                }
            }
        }
        for (label, unp, r) in [("MCP pinned", false, &mcp_p), ("MCP unpinned", true, &mcp_u)] {
            if let Ok((_, out, Ok(_))) = r {
                match exports_in(out) {
                    Ok(x) => compare_exports(&mut rows, &format!("CLI vs {label}"), base, &x, unp),
                    Err(x) => rows.fail(4, x),
                }
            }
        }
        match read_bytes(&proj.join("re.exo")) {
            Ok(b) => rows.check(4, Some(&b) == base.get("out.exo"), || "re-export after replay: re.exo != out.exo".into()),
            Err(x) => rows.fail(4, x),
        }
        match tree(&proj.join("recase")) {
            Ok(re) => {
                let orig: BTreeMap<String, Vec<u8>> =
                    base.iter().filter_map(|(k, v)| k.strip_prefix("case/").map(|s| (s.to_owned(), v.clone()))).collect();
                rows.check(4, re == orig, || "re-export after replay: recase != case".into());
            }
            Err(x) => rows.fail(4, x),
        }
    }
    match &cli.re_case {
        Ok(v) => check_manifest(&mut rows, "CLI export recase", v, Some(&proj.join("recase"))),
        Err(x) => rows.fail(4, format!("smesh export --format openfoam: {x}")),
    }
    if let Err(x) = &cli.re_exo {
        rows.fail(4, format!("smesh export re.exo: {x}"));
    }
    for ev in cli.run["events"].as_array().into_iter().flatten() {
        if ev["event"] == "session" && ev["command"] == "export" && ev["result"]["files"].is_array() {
            let path = ev["result"]["path"].as_str().map(PathBuf::from);
            check_manifest(&mut rows, &format!("script export {}", ev["result"]["path"]), &ev["result"], path.as_deref());
        }
    }

    // ---------------- §4.3 validate / quality through the CLI
    match &cli.validate {
        Ok((code, v)) => rows.check(6, *code == 0 && v["valid"] == json!(true), || {
            format!("`smesh validate -p` exited {code}: valid {}, by_code {}, by_owner {}", v["valid"], v["by_code"], v["by_owner"])
        }),
        Err(x) => rows.fail(6, format!("smesh validate: {x}")),
    }
    match &cli.quality {
        Ok(v) => rows.check(8, v["verdict"].is_string(), || format!("smesh quality --profile transport: no verdict in {v}")),
        Err(x) => rows.fail(8, format!("smesh quality: {x}")),
    }

    // ---------------- row 14: reference core
    match &refc {
        Ok((hy, te, pr)) => {
            rows.eq(14, "reference_standard apply (hybrid)", hy, &h_cli);
            rows.eq(14, "reference_standard apply (tet)", te, &h_cli_tet);
            rows.eq(14, "reference server script (hybrid)", &pr.get("hybrid").cloned().unwrap_or_default(), &h_cli);
            rows.eq(14, "reference server script (tet)", &pr.get("tet").cloned().unwrap_or_default(), &h_cli_tet);
        }
        Err(x) => rows.fail(14, format!("reference core: {x}")),
    }

    let Some(api) = api else { return };
    for (k, d) in &api.timings {
        timings.push(format!("[{}] {k}: {d:.2?}", h.tag()));
    }
    mesh_rows(h, &mut rows, api, &cli.ops, &dir, &proj, facts);
}

/// Model faces of a volume, from `topo.related` on a state.
fn faces_of(state: &State, v: EntityId) -> R<BTreeSet<EntityId>> {
    let model = state.layer::<ModelLayer>().ok_or("no model layer")?;
    let out: BTreeSet<EntityId> = model.topo().volume_faces(v).iter().copied().collect();
    if out.is_empty() {
        return Err(format!("{v} has no faces"));
    }
    Ok(out)
}

fn param_id(ops: &[(String, Value)], pred: impl Fn(&str, &Value) -> bool, path: &[&str]) -> R<EntityId> {
    let (_, p) = ops.iter().find(|(o, p)| pred(o, p)).ok_or("op not found")?;
    let mut v = p;
    for k in path {
        v = if let Ok(i) = k.parse::<usize>() { &v[i] } else { &v[*k] };
    }
    id_of(v.as_str().ok_or_else(|| format!("{path:?} is not an id in {p}"))?)
}

#[allow(clippy::too_many_lines)]
fn mesh_rows(h: Head, rows: &mut Tagged<'_>, api: &Api, ops: &Ops, fork_dir: &Path, proj: &Path, facts: &mut PathFacts) {
    // Volumes and sweep faces from the recorded ops.
    // v1, v2, v3 are the volumes of blocks 1, 2, 3 (the recorded `mesh.size` targets are sorted
    // by id, which is not the V1, V2, V3 order on path B).
    let block = |b: i64| param_id(&ops.hybrid, |o, p| o == "fem.block_add" && p["block"] == json!(b), &["volumes", "0"]);
    let vols: R<[EntityId; 3]> = (|| Ok([block(1)?, block(2)?, block(3)?]))();
    let Some(vols) = rows.need(&[5, 7, 9, 10, 11, 12, 13], "volumes from the recorded fem.block_add", &vols) else { return };
    let vols = *vols;
    let sweep = |vol: EntityId, key: &str| {
        let t = vol.to_string();
        param_id(&ops.hybrid, |o, p| o == "mesh.scheme" && p["scheme"] == "sweep" && p["targets"][0] == json!(t), &["params", key, "0"])
    };
    let (s1, s2, t2) = (sweep(vols[0], "source"), sweep(vols[1], "source"), sweep(vols[1], "target"));

    // ---------------- row 5: part facts
    let faces: R<[BTreeSet<EntityId>; 3]> = (|| Ok([faces_of(&api.hybrid, vols[0])?, faces_of(&api.hybrid, vols[1])?, faces_of(&api.hybrid, vols[2])?]))();
    let faces = match faces {
        Ok(f) => f,
        Err(x) => {
            rows.fail(5, format!("faces of the volumes: {x}"));
            return;
        }
    };
    let union: BTreeSet<EntityId> = faces.iter().flatten().copied().collect();
    let shared: BTreeSet<EntityId> = union.iter().copied().filter(|f| faces.iter().filter(|s| s.contains(f)).count() == 2).collect();
    rows.eq(5, "faces", &union.len(), &sm_testkit::w3a::FACES);
    rows.eq(5, "shared faces", &shared.len(), &3);
    if let Some(model) = api.hybrid.layer::<ModelLayer>() {
        let topo = model.topo();
        let vol_count = model_volumes(&api.hybrid);
        rows.eq(5, "volumes", &vol_count, &3);
        for (k, v) in vols.iter().enumerate() {
            match topo.volume_moments(*v) {
                Ok(mo) => rows.near(5, &format!("volume of V{}", k + 1), mo.volume, sm_testkit::w3a::VOLUMES[k], 1e-9),
                Err(x) => rows.fail(5, format!("volume_moments {v}: {x}")),
            }
        }
        let mut areas: Vec<f64> = shared.iter().filter_map(|f| topo.face_moments(*f).ok().map(|m| m.area)).collect();
        areas.sort_by(f64::total_cmp);
        let want = sm_testkit::w3a::SHARED_FACE_AREAS;
        rows.check(5, areas.len() == 3 && areas.iter().zip(want).all(|(a, b)| (a - b).abs() <= 1e-9), || format!("shared areas {areas:?} vs {want:?}"));
    } else {
        rows.fail(5, "no model layer");
    }
    facts.counts.insert("faces".into(), union.len());

    // ---------------- meshes
    let m = mesh_of(&api.hybrid, api.m);
    let pre = mesh_of(&api.presmooth, api.m);
    let mt = mesh_of(&api.tet, api.mt);
    let pm = mesh_of(&api.tet, api.pm);
    let (Some(m), Some(pre), Some(mt), Some(pm)) = (rows.need(&[6, 7, 8, 9, 10, 13], "hybrid mesh", &m), rows.need(&[8], "pre-smoothing mesh", &pre), rows.need(&[6, 7, 11, 12], "tet mesh", &mt), rows.need(&[6, 11, 12, 13], "poly mesh", &pm)) else {
        return;
    };
    let (m, pre, mt, pm) = (*m, *pre, *mt, *pm);
    for (k, n) in meshchk::shapes(m) {
        facts.counts.insert(format!("m {k}"), n);
    }
    facts.counts.insert("mt cells".into(), mt.cell_count());
    facts.counts.insert("pm cells".into(), pm.cell_count());
    rows.note(5, format!("mesh counts: m {:?} ({} vertices), mt {} tets ({} vertices), pm {:?}", meshchk::shapes(m), m.vertex_count(), mt.cell_count(), mt.vertex_count(), meshchk::shapes(pm)));

    // ---------------- row 6: validity
    for (key, label) in [("validate_m", "m"), ("validate_mt", "mt"), ("validate_pm", "pm")] {
        match api.q.get(key) {
            Some(Ok(v)) => rows.check(6, v["valid"] == json!(true), || format!("mesh.validate {label}: valid {}, by_code {}, by_owner {}", v["valid"], v["by_code"], v["by_owner"])),
            Some(Err(x)) => rows.fail(6, format!("mesh.validate {label}: {x}")),
            None => rows.fail(6, format!("mesh.validate {label} not run")),
        }
    }
    for (label, mesh) in [("m", m), ("mt", mt), ("pm", pm), ("m before smoothing", pre)] {
        let v = meshchk::independent_validate(mesh);
        rows.check(6, v.is_empty(), || format!("sm_mesh_api::validate + validate_geometry({label}): {v:?}"));
    }

    // ---------------- row 7: conformity
    match (&s1, &s2, &t2) {
        (Ok(s1), Ok(s2), Ok(t2)) => {
            let c = meshchk::conformity(m, &shared, *s1, *s2, *t2);
            rows.eq(7, "hexes = 4 x source quads", &c.hexes, &(4 * c.source_quads));
            rows.eq(7, "wedges = 4 x source tris", &c.wedges, &(4 * c.source_tris));
            rows.check(7, c.other == 0, || format!("{} cells are not hex, wedge or tet", c.other));
            rows.check(7, c.interface_tris > 0 && c.bad_interface.is_empty(), || format!("V2|V3 interface ({} tris): {:?}", c.interface_tris, c.bad_interface));
            rows.check(7, c.bad_sides.is_empty(), || format!("sidedness: {:?}", c.bad_sides));
            rows.eq(7, "NonConformal faces in m", &c.nonconformal, &0);
            rows.near(7, "boundary area of m", c.boundary_area, sm_testkit::w3a::OUTER_AREA, 1e-9);
            rows.near(7, "cell volume sum of m", c.total_volume, sm_testkit::w3a::TOTAL_VOLUME, 1e-9);
            for (k, v) in vols.iter().enumerate() {
                rows.near(7, &format!("block {} volume", k + 1), c.per_volume.get(v).copied().unwrap_or(0.0), sm_testkit::w3a::VOLUMES[k], 1e-9);
            }
            rows.note(7, format!("m: {} hex, {} wedge, {} tet; source quads {}, source tris {}, interface tris {}", c.hexes, c.wedges, c.tets, c.source_quads, c.source_tris, c.interface_tris));
        }
        _ => rows.fail(7, format!("sweep faces from the recorded ops: {s1:?} {s2:?} {t2:?}")),
    }
    let nc = (0..mt.face_count()).filter(|&f| mt.face_kind(sm_mesh_api::FaceIx::new(f)) == sm_mesh_api::FaceKind::NonConformal).count();
    rows.eq(7, "NonConformal faces in mt", &nc, &0);
    let (per, tot) = meshchk::volumes(mt);
    rows.near(7, "cell volume sum of mt", tot, sm_testkit::w3a::TOTAL_VOLUME, 1e-9);
    for (k, v) in vols.iter().enumerate() {
        rows.near(7, &format!("mt volume on V{}", k + 1), per.get(v).copied().unwrap_or(0.0), sm_testkit::w3a::VOLUMES[k], 1e-9);
    }

    // ---------------- row 8: quality and smoothing (thresholds initial; measured values noted)
    match (meshchk::min_sj(m), meshchk::min_sj(pre), meshchk::min_sj(mt)) {
        (Ok(after), Ok(before), Ok(tet)) => {
            rows.note(8, format!("min scaled Jacobian: m after smoothing {after:?}, before {before:?}, mt {tet:?}"));
            let g = |mp: &BTreeMap<String, f64>, k: &str| mp.get(k).copied().unwrap_or(f64::NAN);
            for (what, v, lim) in [
                ("hex (m)", g(&after, "Hex"), SJ_HEX),
                ("wedge (m)", g(&after, "Wedge"), SJ_WEDGE),
                ("tet (m, after smoothing)", g(&after, "Tet"), SJ_TET_SMOOTHED),
                ("tet (m, before smoothing)", g(&before, "Tet"), SJ_TET),
                ("tet (mt)", g(&tet, "Tet"), SJ_TET),
            ] {
                rows.check(8, v >= lim, || format!("min scaled Jacobian {what} {v} < {lim}"));
            }
            let floor = 1e-3;
            for k in ["Hex", "Wedge", "Tet"] {
                let lim = g(&before, k).min(floor);
                rows.check(8, g(&after, k) >= lim, || format!("smoothing lowered min SJ of {k}: {} < min(initial {}, guard_floor {floor})", g(&after, k), g(&before, k)));
            }
        }
        (a, b, c) => rows.fail(8, format!("scaled Jacobian: {:?} {:?} {:?}", a.err(), b.err(), c.err())),
    }
    let sc = meshchk::smoothing(pre, m);
    rows.check(8, sc.topology_same, || "smoothing changed the topology".into());
    rows.eq(8, "vertex-classified nodes moved by smoothing", &sc.vertex_nodes_moved, &0);
    rows.check(8, sc.max_curve_dev <= 1e-12, || format!("curve nodes off their lines by {:e}", sc.max_curve_dev));
    rows.check(8, sc.max_surface_dev <= 1e-12, || format!("surface nodes off their planes by {:e}", sc.max_surface_dev));
    rows.check(8, sc.moved > 0, || "smoothing moved no node".into());
    rows.note(8, format!("smoothing moved {} nodes; max curve deviation {:e}, max surface deviation {:e}", sc.moved, sc.max_curve_dev, sc.max_surface_dev));
    for key in ["quality_generic", "quality_transport"] {
        match api.q.get(key) {
            Some(Ok(v)) => {
                rows.check(8, v["verdict"].is_string(), || format!("{key}: no verdict"));
                let counts = [m.cell_count(), m.face_count(), (0..m.face_count()).filter(|&f| m.face_right(sm_mesh_api::FaceIx::new(f)).is_some()).count()];
                for mm in v["metrics"].as_array().into_iter().flatten() {
                    let hg = &mm["histogram"];
                    let n: u64 = hg["counts"].as_array().into_iter().flatten().filter_map(Value::as_u64).sum::<u64>() + hg["above"].as_u64().unwrap_or(0) + hg["below"].as_u64().unwrap_or(0);
                    rows.check(8, counts.contains(&(n as usize)), || format!("{key} {}: histogram adds up to {n}, not one of {counts:?}", mm["metric"]));
                }
                rows.note(8, format!("{key}: verdict {}, suggestions {:?}", v["verdict"], v["suggestions"].as_array().into_iter().flatten().map(|s| (s["op"].clone(), s["available"].clone())).collect::<Vec<_>>()));
                for s in v["suggestions"].as_array().into_iter().flatten() {
                    rows.check(8, s["available"] == json!(true), || format!("{key}: suggestion {} not available", s["op"]));
                }
            }
            Some(Err(x)) => rows.fail(8, format!("{key}: {x}")),
            None => rows.fail(8, format!("{key} not run")),
        }
    }
    // A spread-out warning (every cell below SJ 0.999) must produce the `smooth` suggestion,
    // with `available: true` (OpNames wiring, INT-3).
    match api.q.get("quality_spread") {
        Some(Ok(v)) => {
            let sm: Vec<&Value> = v["suggestions"].as_array().into_iter().flatten().filter(|s| s["op"] == "mesh.smooth").collect();
            rows.check(8, !sm.is_empty() && sm.iter().all(|s| s["available"] == json!(true)), || {
                format!("no available `smooth` suggestion for spread scaled-Jacobian warnings: {}", v["suggestions"])
            });
        }
        Some(Err(x)) => rows.fail(8, format!("quality (spread): {x}")),
        None => {}
    }
    match &api.flag {
        Ok((set, nw, flagged, back)) => {
            rows.eq(8, "flag_cells set size vs n_warn + n_fail", set, nw);
            rows.check(8, *flagged != api.hybrid.hash().to_string(), || "flag_cells did not change the state hash".into());
            rows.eq(8, "checkout after flag_cells", back, &api.hybrid.hash().to_string());
            rows.note(8, format!("flag_cells (scaled_jacobian warn 0.6): {set} cells"));
        }
        Err(x) => rows.fail(8, format!("mesh.flag_cells branch: {x}")),
    }

    // ---------------- row 9: sidesets
    let areas = [meshchk::area_on(m, &faces[0]), meshchk::area_on(m, &faces[1]), meshchk::area_on(m, &faces[2])];
    for (k, (a, want)) in areas.iter().zip(sm_testkit::w3a::SIDESET_AREAS).enumerate() {
        rows.near(9, &format!("sideset {} area", k + 1), *a, want, 1e-9);
    }

    // ---------------- row 10: Exodus
    match read_bytes(&proj.join("out.exo")) {
        Ok(b) => {
            let r = exo::check_hybrid(&b, m, &vols, &faces, ["HEX8", "WEDGE6", "TET4"]);
            for p in &r.problems {
                let row = if p.starts_with("side set") { 9 } else { 10 };
                rows.fail(row, format!("out.exo: {p}"));
            }
            rows.note(10, format!("out.exo blocks {:?}", r.blocks));
        }
        Err(x) => rows.fail(10, x),
    }
    match read_bytes(&proj.join("poly.exo")) {
        Ok(b) => {
            let r = exo::check_poly(&b, pm, &vols);
            for p in &r.problems {
                rows.fail(10, format!("poly.exo: {p}"));
            }
            rows.note(10, format!("poly.exo blocks {:?}", r.blocks));
        }
        Err(x) => rows.fail(10, x),
    }

    // ---------------- row 11: OpenFOAM
    for (case, mesh) in [("case", m), ("tetcase", mt), ("polycase", pm)] {
        let r = foam::check_case(&proj.join(case));
        for p in &r.problems {
            rows.fail(11, format!("{case}: {p}"));
        }
        if r.ties > 0 {
            rows.note(11, format!("{case}: {} foam.upper_triangular_tie warnings (faces sharing an owner/neighbour pair; checkMesh judges)", r.ties));
        }
        if let Some(pmsh) = &r.mesh {
            rows.eq(11, &format!("{case}: points"), &pmsh.points.len(), &mesh.vertex_count());
            rows.eq(11, &format!("{case}: cells"), &(pmsh.n_cells as usize), &mesh.cell_count());
            rows.eq(11, &format!("{case}: faces"), &pmsh.n_faces(), &mesh.face_count());
        }
    }
    // INT-1: polycase cells = pairs + wedges - split_cells.
    let pairs = meshchk::vertex_volume_pairs(mt);
    let info = api.q.get("poly_info").and_then(|r| r.as_ref().ok());
    let from_info = info.and_then(|v| Some((v.get("wedges")?.as_u64()? as usize, v.get("split_cells")?.as_u64()? as usize)));
    let lib = (|| -> R<(usize, usize, usize)> {
        let model = api.tet.layer::<ModelLayer>().ok_or("no model layer")?;
        let mut alloc = sm_mesh_api::SerialAlloc::new();
        let (_, rep) = sm_poly::dualize(mt, &sm_poly::PolyOptions::new(), &sm_poly::TopologyResolver(model.topo()), sm_mesh::MeshBuilder::new(), &mut alloc).map_err(e)?;
        Ok((rep.cells, rep.wedges, rep.split_cells))
    })();
    match &lib {
        Ok((cells, w, sc)) => {
            rows.note(11, format!("dual report (library): cells {cells}, pairs {pairs}, wedges {w}, split_cells {sc}"));
            rows.eq(11, "poly cells = pairs + wedges - split_cells (library DualReport)", &pm.cell_count(), &(pairs + w - sc));
        }
        Err(x) => rows.fail(11, format!("sm_poly::dualize: {x}")),
    }
    match from_info {
        Some((w, sc)) => rows.eq(11, "poly cells = pairs + wedges - split_cells (mesh.poly_info)", &pm.cell_count(), &(pairs + w - sc)),
        None => rows.fail(11, format!("mesh.poly_info reports no `wedges` / `split_cells` (keys: {:?})", info.and_then(Value::as_object).map(|o| o.keys().cloned().collect::<Vec<_>>()))),
    }

    // ---------------- row 12: poly
    let (per, tot) = meshchk::volumes(pm);
    rows.near(12, "dual volume sum", tot, sm_testkit::w3a::TOTAL_VOLUME, 1e-9);
    for (k, v) in vols.iter().enumerate() {
        rows.near(12, &format!("dual volume on V{}", k + 1), per.get(v).copied().unwrap_or(0.0), sm_testkit::w3a::VOLUMES[k], 1e-9);
    }
    let sh = meshchk::shapes(pm);
    rows.check(12, sh.contains_key("Polyhedron"), || format!("pm has no polyhedra: {sh:?}"));
    // Starness of the dual cells (what `mesh.validate`'s validity reads for polyhedra).
    if let Ok(col) = sm_quality::compute(pm, sm_quality::Metric::Starness, &sm_quality::Options::default()) {
        let neg: Vec<(usize, f64)> = col.iter().copied().enumerate().filter(|(_, v)| *v < 0.0).collect();
        let min = col.iter().copied().filter(|v| !v.is_nan()).fold(f64::INFINITY, f64::min);
        rows.note(12, format!("pm starness: min {min}, {} cells below 0 (first: {:?})", neg.len(), neg.iter().take(5).collect::<Vec<_>>()));
    }
    match info {
        Some(v) => {
            let want = serde_json::to_value(sm_poly::poly_stats(pm)).unwrap_or(Value::Null);
            rows.eq(12, "mesh.poly_info vs poly_stats", v, &want);
        }
        None => rows.fail(12, format!("mesh.poly_info: {:?}", api.q.get("poly_info"))),
    }
    // `.smsh` header: (element_types, mesh.stats); the file must also decode.
    let smsh_head = |p: &Path| -> R<(Vec<String>, Value, usize)> {
        let b = read_bytes(p)?;
        let d = sm_meshio::SmshFile::parse(&b).map_err(e)?.decode().map_err(e)?;
        let n = u64::from_le_bytes(b.get(8..16).ok_or("short")?.try_into().map_err(e)?) as usize;
        let hv: Value = serde_json::from_slice(b.get(16..16 + n).ok_or("short header")?).map_err(e)?;
        let types = hv["element_types"].as_array().into_iter().flatten().map(|t| t["element_type"].as_str().unwrap_or("").to_owned()).collect();
        Ok((types, hv["mesh"]["stats"].clone(), d.cell_ids.len()))
    };
    let census = pm.cell_count() as u64;
    // poly.smsh comes from the tet branch, whose blocks declare no element type (§4.2), so its
    // header lists none; the polyhedra are checked through `mesh.stats`. The declared
    // `POLYHEDRON` header is checked on polyblocks.smsh, exported by the fork script's sub-branch.
    match smsh_head(&proj.join("poly.smsh")) {
        Ok((t, st, cells)) => {
            rows.check(12, t.is_empty() || t.iter().all(|x| x == "POLYHEDRON"), || format!("poly.smsh element_types {t:?}"));
            rows.eq(12, "poly.smsh decoded cells vs census", &(cells as u64), &census);
            rows.eq(12, "poly.smsh mesh.stats n_polyhedron vs census", &st["n_polyhedron"].as_u64(), &Some(census));
        }
        Err(x) => rows.fail(12, format!("poly.smsh: {x}")),
    }
    match smsh_head(&fork_dir.join("forkproj").join("polyblocks.smsh")) {
        Ok((t, st, _)) => {
            rows.eq(12, "polyblocks.smsh element_types (POLYHEDRON declared)", &t, &vec!["POLYHEDRON".to_owned(); 3]);
            rows.eq(12, "polyblocks.smsh mesh.stats n_polyhedron vs census", &st["n_polyhedron"].as_u64(), &Some(census));
        }
        Err(x) => rows.fail(12, format!("polyblocks.smsh (fork sub-branch): {x}")),
    }
    let smsh_types = |name: &str| smsh_head(&proj.join(name)).map(|x| x.0);
    match smsh_types("out.smsh") {
        Ok(t) => rows.eq(12, "out.smsh element_types", &t, &vec!["HEX8".to_owned(), "WEDGE6".to_owned(), "TET4".to_owned()]),
        Err(x) => rows.fail(12, format!("out.smsh: {x}")),
    }

    // ---------------- row 13: tracking
    for (label, mesh) in [("m", m), ("pm", pm)] {
        let r = track::check(mesh, &vols);
        for p in &r.problems {
            rows.fail(13, format!("{label}: {p}"));
        }
        rows.note(13, format!("{label}: {}", r.facts.join("; ")));
    }
    let _ = h;
}

fn model_volumes(state: &State) -> usize {
    state.layer::<ModelLayer>().map_or(0, |m| m.topo().cells(sm_topo_api::Dim::Volume).len())
}

/// The ids of an MCP plan (the lines are moved into the thread).
fn plan_ids(p: &McpPlan) -> McpPlan {
    McpPlan { lines: Vec::new(), op_ids: p.op_ids.clone(), hash_hybrid: p.hash_hybrid, hash_tet: p.hash_tet, hash_back: p.hash_back, exports: p.exports.clone(), validate: p.validate, quality: p.quality }
}

// ------------------------------------------------------------------ the test

#[test]
fn milestone_w3a() {
    let t0 = Instant::now();
    let t = scratch();
    let step = t.join("w3a_part.step");
    std::fs::write(&step, sm_testkit::w3a::STEP).expect("write the STEP file");
    let mut rows = Rows::default();
    let mut timings = Vec::new();
    let mut facts = [PathFacts::default(), PathFacts::default()];
    for (k, h) in [Head::A, Head::B].into_iter().enumerate() {
        let th = Instant::now();
        run_path(h, &t, &step, &mut rows, &mut facts[k], &mut timings);
        timings.push(format!("[{}] path total: {:.2?}", h.tag(), th.elapsed()));
    }
    // Row 5: paths A and B give the same model counts (mesh counts: advisory note only).
    let model_keys = ["faces"];
    for k in model_keys {
        rows.eq(5, &format!("A vs B {k}"), &facts[0].counts.get(k), &facts[1].counts.get(k));
    }
    let mesh_diff: Vec<String> = facts[0]
        .counts
        .iter()
        .filter(|(k, v)| !model_keys.contains(&k.as_str()) && facts[1].counts.get(*k) != Some(v))
        .map(|(k, v)| format!("{k}: A {v} / B {:?}", facts[1].counts.get(k)))
        .collect();
    if !mesh_diff.is_empty() {
        rows.note(5, format!("advisory: mesh counts differ between paths A and B: {mesh_diff:?}"));
    }
    timings.push(format!("milestone total (debug build, both paths): {:.2?}", t0.elapsed()));

    let mut report = String::from("\nW3a milestone rows (plan §4.6):\n");
    for row in 1..=14u8 {
        let status = if rows.fails.contains_key(&row) { "FAIL" } else { "pass" };
        report.push_str(&format!("  row {row:2}: {status}\n"));
        for f in rows.fails.get(&row).into_iter().flatten() {
            report.push_str(&format!("      fail: {f}\n"));
        }
        for n in rows.notes.get(&row).into_iter().flatten() {
            report.push_str(&format!("      note: {n}\n"));
        }
    }
    report.push_str("wall times (record only, no gate):\n");
    for l in &timings {
        report.push_str(&format!("  {l}\n"));
    }
    println!("{report}");
    if std::env::var_os("MILESTONE_W3A_REPORT").is_some() {
        let _ = std::fs::write(Path::new(env!("CARGO_TARGET_TMPDIR")).join("milestone_w3a_report.txt"), &report);
    }
    assert!(rows.fails.is_empty(), "{} row(s) failed:{report}", rows.fails.len());
}
