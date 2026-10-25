//! P0 capabilities through the production distribution `sm_server::standard()` (real IIA
//! solver, auto-size hook, registered schemes), driven by the real `smesh` binary (script runs
//! with `--json`, and the MCP stdio path). Expected values are derived by hand below, not read
//! back from the code under test.
//!
//! Derivations (brick(x,y,z) is centred on the origin; every edge of a brick is axis-aligned):
//!
//! * Row 1. `mesh.intervals(count: 3)` on the volume sets 3 on every owned edge; a map volume
//!   then has 3*3*3 = 27 hexes and (3+1)^3 = 64 nodes at `-L/2 + L k/3`, k = 0..3 per axis.
//! * Row 2/6. `auto_size(f) = clamp(0.01 d 2^((f-1)/2), l_min/4, d/2)`, `n = max(1, round(L/s))`
//!   (half away from zero), `d` the bounding-box diagonal, `l_min` the shortest edge.
//!   brick(10,10,10): d = 10 sqrt(3) = 17.3205, l_min = 10, bounds [2.5, 8.660].
//!   f=1: 0.1732 -> 2.5 (lower clamp) -> n = 4 -> 64 hexes. f=5: 0.1732*4 = 0.6928 -> 2.5 -> 64.
//!   f=9: 0.1732*16 = 2.7713 -> 10/2.7713 = 3.608 -> 4 -> 64.
//!   f=10: 0.1732*2^4.5 = 0.1732*22.627 = 3.9192 -> 10/3.9192 = 2.552 -> 3 -> 27 hexes.
//!   brick(4,2,1): d = sqrt(21) = 4.5826, l_min = 1, bounds [0.25, 2.2913].
//!   f=10: 0.045826*22.627 = 1.0369 -> 4/1.0369 = 3.857 -> 4, 2/1.0369 = 1.929 -> 2,
//!   1/1.0369 = 0.964 -> 1 (max(1, round)) -> 4*2*1 = 8 hexes.
//! * Row 3. `mesh.default_size(size: 2)`: every edge 10/2 = 5 -> 125 hexes.
//! * Row 4/5. Hard schemes equal (edges) and map (faces, volume) on a brick give the same lattice
//!   as the default plan: with 2.5 on every edge, 4 intervals -> 64 hexes. An unknown scheme
//!   name is a typed error (exit 1, message names the scheme).
//! * Row 6 (cubit-meshing-semantics §2 "Firmness"): any interval command raises a curve to at
//!   least soft, at the same level the latest command wins, and a firmer request is kept.
//!   (a) soft volume count 3 then `mesh.size(2)` on brick 10 -> size replaces it: 5 -> 125.
//!   (b) `mesh.size(2)` then a soft volume count 3 -> count replaces it: 3 -> 27.
//!   (c) a hard count 2 on one edge then `mesh.size(2.5)` on the volume: the hard edge and its
//!   three parallel edges keep 2 (opposite sides of a map face are equal), the other two
//!   directions get 10/2.5 = 4 -> 2*4*4 = 32 hexes.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Output, Stdio};

use serde_json::{Value, json};

fn scratch(tag: &str) -> PathBuf {
    let d = PathBuf::from(env!("CARGO_TARGET_TMPDIR")).join(format!("p0_rows_{tag}"));
    let _ = std::fs::remove_dir_all(&d);
    std::fs::create_dir_all(&d).expect("scratch dir");
    d
}

fn smesh(args: &[&str], stdin: &str) -> Output {
    let mut c = Command::new(env!("CARGO_BIN_EXE_smesh"));
    c.args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped());
    let mut child = c.spawn().expect("spawn smesh");
    child
        .stdin
        .take()
        .expect("stdin")
        .write_all(stdin.as_bytes())
        .expect("write stdin");
    child.wait_with_output().expect("wait smesh")
}

/// Result of a script run.
struct Ran {
    code: i32,
    /// stderr followed by stdout (with `--json` a failure's diagnostics arrive on stdout).
    stderr: String,
    /// Result of every `mesh.info` query, in order.
    infos: Vec<Value>,
}

/// Run `script` with `run --memory --json`; `tag` names its scratch directory.
fn run_script(tag: &str, script: &str) -> Ran {
    let dir = scratch(tag);
    let file = dir.join("t.smesh");
    std::fs::write(&file, script).expect("write script");
    let o = smesh(
        &["run", file.to_str().expect("utf8"), "--memory", "--json"],
        "",
    );
    let mut infos = Vec::new();
    if let Ok(v) = serde_json::from_slice::<Value>(&o.stdout) {
        for e in v["events"].as_array().into_iter().flatten() {
            if e["event"] == "query" && e["name"] == "mesh.info" {
                infos.push(e["result"].clone());
            }
        }
    }
    Ran {
        code: o.status.code().expect("exit code"),
        stderr: format!(
            "{}{}",
            String::from_utf8_lossy(&o.stderr),
            String::from_utf8_lossy(&o.stdout)
        ),
        infos,
    }
}

/// Run a script that ends in one `mesh.info` and return it; the run must succeed.
fn info_of(tag: &str, script: &str) -> Value {
    let r = run_script(tag, script);
    assert_eq!(r.code, 0, "{tag}: exit {} stderr: {}", r.code, r.stderr);
    assert_eq!(r.infos.len(), 1, "{tag}: one mesh.info expected");
    r.infos[0].clone()
}

fn cells(info: &Value) -> u64 {
    info["cells"].as_u64().expect("cells")
}

fn hex_count(info: &Value) -> u64 {
    info["cell_shapes"]["Hex"].as_u64().unwrap_or(0)
}

/// Assert `n` hexes per axis `(nx,ny,nz)` from the cell, vertex and volume counts.
fn assert_grid(info: &Value, n: (u64, u64, u64), vol: f64) {
    assert_eq!(cells(info), n.0 * n.1 * n.2, "{info}");
    assert_eq!(hex_count(info), cells(info), "all cells are hexes: {info}");
    assert_eq!(
        info["vertices"].as_u64().expect("vertices"),
        (n.0 + 1) * (n.1 + 1) * (n.2 + 1),
        "{info}"
    );
    let v = info["volume"].as_f64().expect("volume");
    assert!((v - vol).abs() < 1e-9 * vol, "volume {v} vs {vol}");
}

const TAIL: &str = "let m = mesh.run(targets: b.volume)\nlet i = mesh.info(mesh: m.mesh[0])\n";

/// Decode the exported `.smsh` and check the nodes are exactly the product lattice
/// `-L/2 + L k/n` per axis (all `(nx+1)(ny+1)(nz+1)` distinct nodes).
fn assert_lattice(path: &std::path::Path, len: [f64; 3], n: [usize; 3]) {
    let bytes = std::fs::read(path).expect("read smsh");
    let file = sm_meshio::SmshFile::parse(&bytes).expect("parse smsh");
    let d = file.decode().expect("decode smsh");
    let total = (n[0] + 1) * (n[1] + 1) * (n[2] + 1);
    assert_eq!(d.x.len(), total, "node count");
    let mut seen = std::collections::BTreeSet::new();
    for i in 0..d.x.len() {
        let mut key = [0usize; 3];
        for (a, c) in [d.x[i], d.y[i], d.z[i]].into_iter().enumerate() {
            let t = (c + len[a] / 2.0) / len[a] * n[a] as f64;
            let k = t.round();
            assert!(
                (t - k).abs() < 1e-9,
                "node {i} axis {a}: {c} is off the lattice"
            );
            assert!(
                k >= 0.0 && k <= n[a] as f64,
                "node {i} axis {a} outside the brick"
            );
            key[a] = k as usize;
        }
        seen.insert(key);
    }
    assert_eq!(seen.len(), total, "lattice nodes are distinct and complete");
}

#[test]
fn row1_interval_count_gives_27_hexes_on_the_lattice() {
    let dir = scratch("row1_out");
    let out = dir.join("a.smsh");
    let script = format!(
        "let b = model.create_brick(x: 4.0, y: 2.0, z: 1.0)\n\
         mesh.intervals(targets: b.volume, count: 3)\n\
         mesh.scheme(targets: b.volume, scheme: \"map\")\n\
         {TAIL}session.export(path: \"{}\")\n",
        out.display()
    );
    let info = info_of("row1", &script);
    assert_grid(&info, (3, 3, 3), 8.0);
    assert_eq!(info["stale"], false);
    assert_lattice(&out, [4.0, 2.0, 1.0], [3, 3, 3]);
}

/// MCP stdio: `tools/call` replies keyed by id.
fn mcp(calls: Vec<(&str, Value)>) -> BTreeMap<i64, Value> {
    let mut lines = vec![
        json!({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"it","version":"0"}}}),
        json!({"jsonrpc":"2.0","method":"notifications/initialized"}),
    ];
    for (i, (name, args)) in calls.into_iter().enumerate() {
        lines.push(json!({"jsonrpc":"2.0","id":i as i64 + 2,"method":"tools/call","params":{"name":name,"arguments":args}}));
    }
    let text: String = lines.iter().map(|l| format!("{l}\n")).collect();
    let o = smesh(&["mcp", "--memory"], &text);
    assert_eq!(
        o.status.code(),
        Some(0),
        "{}",
        String::from_utf8_lossy(&o.stderr)
    );
    String::from_utf8_lossy(&o.stdout)
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let v: Value = serde_json::from_str(l).expect("json reply");
            (v["id"].as_i64().expect("id"), v)
        })
        .collect()
}

/// `structuredContent` of a successful tool call.
fn ok(replies: &BTreeMap<i64, Value>, id: i64) -> Value {
    let r = &replies[&id]["result"];
    assert_eq!(
        r["isError"],
        Value::Bool(false),
        "id {id}: {}",
        replies[&id]
    );
    r["structuredContent"].clone()
}

#[test]
fn row1_and_2_over_mcp() {
    // Interval count 3 on brick(4,2,1), then auto 10 on brick(10,10,10) (27 hexes, see header).
    let r = mcp(vec![
        ("model__create_brick", json!({"x":4,"y":2,"z":1})),
        (
            "mesh__intervals",
            json!({"targets":["TopoVolume#1"],"count":3}),
        ),
        ("mesh__run", json!({"targets":["TopoVolume#1"]})),
        ("mesh__info", json!({"mesh":"Mesh#1"})),
        ("model__create_brick", json!({"x":10,"y":10,"z":10})),
        ("mesh__size", json!({"targets":["TopoVolume#2"],"auto":10})),
        ("mesh__run", json!({"targets":["TopoVolume#2"]})),
        ("mesh__info", json!({"mesh":"Mesh#2"})),
    ]);
    let a = ok(&r, 5);
    assert_eq!(a["cells"], 27, "{a}");
    assert_eq!(a["vertices"], 64, "{a}");
    let b = ok(&r, 9);
    assert_eq!(b["cells"], 27, "{b}");
}

/// Reference auto size per the documented formula (independent of `sm_sizing`).
fn auto_ref(dims: [f64; 3], f: u32) -> f64 {
    let d = dims.iter().map(|x| x * x).sum::<f64>().sqrt();
    let lmin = dims.iter().cloned().fold(f64::INFINITY, f64::min);
    (0.01 * d * 2f64.powf((f64::from(f) - 1.0) / 2.0)).clamp(lmin / 4.0, d / 2.0)
}

fn count_ref(l: f64, s: f64) -> u64 {
    ((l / s).round() as u64).max(1)
}

#[test]
fn row2_auto_size_counts() {
    // (dims, factor, expected per-edge counts) hand-derived in the header; the reference
    // function below must agree with them as a check on the derivation itself.
    type Case = ([f64; 3], u32, (u64, u64, u64));
    let cases: [Case; 6] = [
        ([10.0; 3], 1, (4, 4, 4)),
        ([10.0; 3], 5, (4, 4, 4)),
        ([10.0; 3], 9, (4, 4, 4)),
        ([10.0; 3], 10, (3, 3, 3)),
        ([4.0, 2.0, 1.0], 10, (4, 2, 1)),
        ([10.0; 3], 7, (4, 4, 4)),
    ];
    for (k, (dims, f, n)) in cases.into_iter().enumerate() {
        let s = auto_ref(dims, f);
        assert_eq!(
            (
                count_ref(dims[0], s),
                count_ref(dims[1], s),
                count_ref(dims[2], s)
            ),
            n,
            "derivation for {dims:?} auto {f}"
        );
        let script = format!(
            "let b = model.create_brick(x: {:?}, y: {:?}, z: {:?})\n\
             mesh.size(targets: b.volume, auto: {f})\n\
             mesh.scheme(targets: b.volume, scheme: \"map\")\n{TAIL}",
            dims[0], dims[1], dims[2]
        );
        let info = info_of(&format!("row2_{k}"), &script);
        assert_grid(&info, n, dims[0] * dims[1] * dims[2]);
    }
}

#[test]
fn row2_auto_out_of_range_is_an_error() {
    for f in [0, 11] {
        let r = run_script(
            &format!("row2_bad{f}"),
            &format!(
                "let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)\nmesh.size(targets: b.volume, auto: {f})\n"
            ),
        );
        assert_eq!(r.code, 1, "auto {f}: {}", r.stderr);
        assert!(r.stderr.contains("auto"), "{}", r.stderr);
    }
}

#[test]
fn row3_default_size_gives_125_hexes() {
    let script = format!(
        "let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)\nmesh.default_size(size: 2.0)\n{TAIL}"
    );
    assert_grid(&info_of("row3", &script), (5, 5, 5), 1000.0);
}

#[test]
fn row4_explicit_schemes_through_standard() {
    let dir = scratch("row4_out");
    let out = dir.join("s.smsh");
    let script = format!(
        "let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)\n\
         mesh.size(targets: b.volume, size: 2.5)\n\
         mesh.scheme(targets: b.edges, scheme: \"equal\")\n\
         mesh.scheme(targets: b.faces, scheme: \"map\")\n\
         mesh.scheme(targets: b.volume, scheme: \"map\")\n\
         {TAIL}session.export(path: \"{}\")\n",
        out.display()
    );
    let info = info_of("row4", &script);
    // Row 5: the same lattice as the unconstrained plan, and the first candidate was accepted.
    assert_grid(&info, (4, 4, 4), 1000.0);
    assert_eq!(info["attempts"], 0, "{info}");
    assert_eq!(info["rejections"], 0, "{info}");
    assert_lattice(&out, [10.0; 3], [4, 4, 4]);

    // Same hash of the mesh without any explicit scheme choice: the plan picks map itself.
    let plain = info_of(
        "row4_plain",
        &format!(
            "let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)\nmesh.size(targets: b.volume, size: 2.5)\n{TAIL}"
        ),
    );
    assert_eq!(plain["topology_hash"], info["topology_hash"]);
    assert_eq!(plain["geometry_hash"], info["geometry_hash"]);
}

#[test]
fn row4_unknown_scheme_is_a_typed_error() {
    for target in ["b.volume", "b.faces", "b.edges"] {
        let r = run_script(
            "row4_unknown",
            &format!(
                "let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)\nmesh.scheme(targets: {target}, scheme: \"nonesuch\")\n"
            ),
        );
        assert_eq!(r.code, 1, "{target}: {}", r.stderr);
        assert!(r.stderr.contains("nonesuch"), "{target}: {}", r.stderr);
    }
    // Over MCP the failure is a tool error naming the scheme, not a transport error.
    let r = mcp(vec![
        ("model__create_brick", json!({"x":10,"y":10,"z":10})),
        (
            "mesh__scheme",
            json!({"targets":["TopoVolume#1"],"scheme":"nonesuch"}),
        ),
    ]);
    assert_eq!(r[&3]["result"]["isError"], Value::Bool(true), "{}", r[&3]);
    assert!(r[&3].to_string().contains("nonesuch"), "{}", r[&3]);
}

#[test]
fn row5_info_reports_what_exists() {
    // There is no query reporting the per-entity schemes that ran (mesh.info carries only
    // counts, hashes and plan attempts/rejections), so this asserts those.
    let script = format!(
        "let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)\n\
         mesh.size(targets: b.volume, size: 2.5)\n\
         mesh.scheme(targets: b.edges, scheme: \"equal\")\n\
         mesh.scheme(targets: b.faces, scheme: \"map\")\n\
         mesh.scheme(targets: b.volume, scheme: \"map\")\n{TAIL}"
    );
    let info = info_of("row5", &script);
    assert_eq!(
        info["selection"].as_array().map(Vec::len),
        Some(1),
        "{info}"
    );
    assert_eq!(info["cell_shapes"], json!({"Hex": 64}), "{info}");
    assert_eq!(info["attempts"], 0);
    assert_eq!(info["rejections"], 0);
}

#[test]
fn row6_firmness_through_standard() {
    let head = "let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)\n";
    // (a) a later size replaces an earlier soft volume count.
    let a = info_of(
        "row6a",
        &format!(
            "{head}mesh.intervals(targets: b.volume, count: 3)\nmesh.size(targets: b.volume, size: 2.0)\n{TAIL}"
        ),
    );
    assert_grid(&a, (5, 5, 5), 1000.0);
    // (b) a later soft count replaces an earlier size.
    let b = info_of(
        "row6b",
        &format!(
            "{head}mesh.size(targets: b.volume, size: 2.0)\nmesh.intervals(targets: b.volume, count: 3)\n{TAIL}"
        ),
    );
    assert_grid(&b, (3, 3, 3), 1000.0);
    // (c) a hard curve count survives a later size on the volume.
    let c = info_of(
        "row6c",
        &format!(
            "{head}mesh.intervals(targets: b.edges[0], count: 2)\nmesh.size(targets: b.volume, size: 2.5)\n\
             mesh.scheme(targets: b.volume, scheme: \"map\")\n{TAIL}"
        ),
    );
    assert_eq!(cells(&c), 32, "{c}");
    assert_eq!(hex_count(&c), 32, "{c}");
}
