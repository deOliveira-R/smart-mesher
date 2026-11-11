//! W3b-SERVER-HOST: AUTO acceptance B, the real-scheme census. The `default` strategy of `mesh.auto`
//! runs against the eight real scheme crates through `standard()`.
//!
//! Fixtures are built by ops only. `mesh.auto` stores explicit assignments, so after it a plan row
//! reads `intent#N`; the `Auto { score }` rows the strategy saw are visible through `mesh.plan`
//! on the same targets before `mesh.auto`, or through the census (`sm_mesher::census`).
mod common;

use std::sync::Arc;

use common::TempDir;
use serde_json::{Value, json};
use sm_mesher::{CensusReport, MesherConfig};
use sm_protocol::{Endpoint, Request, RpcError};
use sm_server::{Server, standard_schemes, standard_with};
use sm_sizing::{AnglePicker, IiaSolver};

const TIME: &str = "2026-10-24T00:00:00Z";

fn builder() -> sm_engine::WorkspaceBuilder {
    standard_with(Some("tester".into()), Some(TIME.into()))
}

fn server() -> Server {
    Server::new(builder()).expect("server")
}

fn script(s: &mut Server, text: &str) -> Result<Value, RpcError> {
    s.call(Request::ScriptRun { text: text.into(), path: None, includes: Default::default() })
}

fn ok(s: &mut Server, text: &str) -> Value {
    script(s, text).unwrap_or_else(|e| panic!("script failed: {e}\n{}", e.data["rendered"].as_str().unwrap_or("")))
}

fn query(s: &mut Server, name: &str, input: Value) -> Value {
    s.call(Request::Query { name: name.into(), input, at: None }).unwrap_or_else(|e| panic!("query {name}: {e} {}", e.data))
}

fn state_hash(s: &mut Server) -> String {
    s.call(Request::Hash { at: None }).expect("hash")["state"].as_str().expect("hex").to_owned()
}

fn volumes(s: &mut Server) -> Vec<String> {
    query(s, "view.scene", json!({}))["volumes"].as_array().expect("volumes").iter().map(|v| v["id"].as_str().expect("id").to_owned()).collect()
}

/// The diagnostics of the first op `name` in a `script.run` result.
fn diagnostics(run: &Value, op: &str) -> Vec<Value> {
    run["events"]
        .as_array()
        .expect("events")
        .iter()
        .filter(|e| e["op"] == op)
        .flat_map(|e| e["delta"]["diagnostics"].as_array().cloned().unwrap_or_default())
        .collect()
}

fn auto_tet(run: &Value) -> Vec<Value> {
    diagnostics(run, "mesh.auto").into_iter().filter(|d| d["code"] == "mesh.auto_tet").collect()
}

/// One non-curve row of a mesh's plan.
#[derive(Debug, Clone, PartialEq)]
struct Row {
    target: String,
    kind: String,
    scheme: String,
    how: String,
    score: Option<u64>,
}

fn plan_rows(report: &Value) -> Vec<Row> {
    report["entities"]
        .as_array()
        .expect("entities")
        .iter()
        .filter(|e| e["kind"] != "curve")
        .map(|e| Row {
            target: e["target"].as_str().expect("target").to_owned(),
            kind: e["kind"].as_str().expect("kind").to_owned(),
            scheme: e["scheme"].as_str().expect("scheme").to_owned(),
            how: e["how"].as_str().expect("how").to_owned(),
            score: e["score"].as_u64(),
        })
        .collect()
}

fn mesh_plan(s: &mut Server) -> Vec<Row> {
    plan_rows(&query(s, "mesh.plan", json!({"mesh": "Mesh#1"})))
}

fn info(s: &mut Server) -> Value {
    query(s, "mesh.info", json!({"mesh": "Mesh#1"}))
}

fn valid(s: &mut Server) -> bool {
    let v = query(s, "mesh.validate", json!({"mesh": "Mesh#1"}));
    v["valid"].as_bool().unwrap_or_else(|| panic!("{v}"))
}

/// `TopoVolume#3` as a script writes it: `Volume#3`.
fn script_id(id: &str) -> String {
    id.strip_prefix("Topo").unwrap_or(id).to_owned()
}

fn scheme_of<'a>(rows: &'a [Row], target: &str) -> &'a Row {
    rows.iter().find(|r| r.target == target).unwrap_or_else(|| panic!("no row for {target}: {rows:?}"))
}

fn faces_by_scheme<'a>(rows: &'a [Row], scheme: &str) -> Vec<&'a Row> {
    rows.iter().filter(|r| r.kind == "surface" && r.scheme == scheme && r.how != "copy").collect()
}

// ---- fixtures ------------------------------------------------------------------------------------

const BRICK: &str = "let a = model.create_brick(x: 10.0, y: 10.0, z: 10.0)\nmesh.size(targets: a.volume, size: 2.5)\n";
const BRICK_EXPLICIT: &str = "let b = model.create_brick(x: 10.0, y: 10.0, z: 10.0)\n\
    mesh.size(targets: b.volume, size: 2.5)\n\
    mesh.scheme(targets: b.edges, scheme: \"equal\")\n\
    mesh.scheme(targets: b.faces, scheme: \"map\")\n\
    mesh.scheme(targets: b.volume, scheme: \"map\")\n\
    let m = mesh.run(targets: b.volume)\n";
const RUN: &str = "mesh.auto(strategy: \"default\")\nlet m = mesh.run(targets: r.volumes)\n";

/// A 2x2x1 slab minus a tool spanning [1,3]x[1,3]x[-1,2]: an L, volume 3, 8 faces.
const L_SLAB: &str = "let a = model.create_brick(x: 2.0, y: 2.0, z: 1.0, center: [1.0, 1.0, 0.5])\n\
    let t = model.create_brick(x: 2.0, y: 2.0, z: 3.0, center: [2.0, 2.0, 0.5])\n\
    let r = model.subtract(targets: a.body, tools: t.body)\n\
    mesh.size(targets: r.volumes, size: 0.5)\n";

/// 2x2x1 brick cut by the vertical plane x + y = 1, keeping the triangular prism.
const PRISM: &str = "let a = model.create_brick(x: 2.0, y: 2.0, z: 1.0, center: [1.0, 1.0, 0.5])\n\
    let r = model.webcut_plane(targets: a.body, plane: {origin: [1.0, 0.0, 0.0], normal: [1.0, 1.0, 0.0]}, keep: \"negative\")\n\
    mesh.size(targets: r.volumes, size: 0.5)\n";

/// The corner of a 2-cube cut by the plane through (1,0,0), (0,1,0), (0,0,1): a tetrahedron, volume 1/6.
const TETRA: &str = "let a = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [1.0, 1.0, 1.0])\n\
    let r = model.webcut_plane(targets: a.body, plane: {origin: [1.0, 0.0, 0.0], normal: [1.0, 1.0, 1.0]}, keep: \"negative\")\n\
    mesh.size(targets: r.volumes, size: 0.25)\n";

/// A 4x4x1 plate minus a z cylinder of radius 1 (volume 16 - pi).
const PLATE: &str = "let a = model.create_brick(x: 4.0, y: 4.0, z: 1.0)\n\
    let t = model.create_cylinder(height: 3.0, radius: 1.0)\n\
    let r = model.subtract(targets: a.body, tools: t.body)\n\
    mesh.size(targets: r.volumes, size: 0.5)\n";

/// Two merged 2-cubes; the left carries a hard `tetmesh`.
const TWO_BRICKS_HARD_TET: &str = "let a = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [1.0, 1.0, 1.0])\n\
    let b = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [3.0, 1.0, 1.0])\n\
    model.imprint(bodies: a.body | b.body)\n\
    model.merge(entities: a.body | b.body)\n\
    mesh.size(targets: a.volume | b.volume, size: 1.0)\n\
    mesh.scheme(targets: a.volume, scheme: \"tetmesh\")\n";

/// A round pin cell: a 1.26 x 1.26 x 1.0 moderator brick with a z cylinder (r = 0.4, height `h`, centered)
/// of fuel. The moderator is the brick minus a consumed copy of the cylinder (no `keep`), then imprint and
/// merge. `h` = 1.0 goes through, 0.8 is enclosed. `brick_first` picks the creation order (entity ids).
fn pin_fixture(h: f64, brick_first: bool) -> String {
    let brick = "let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)\n";
    let fuel = format!("let fu = model.create_cylinder(height: {h}, radius: 0.4)\n");
    let tool = format!("let tl = model.create_cylinder(height: {h}, radius: 0.4)\n");
    let head = if brick_first { format!("{brick}{fuel}{tool}") } else { format!("{fuel}{tool}{brick}") };
    format!(
        "{head}let mo = model.subtract(targets: br.body, tools: tl.body)\n\
         model.imprint(bodies: all bodies)\nmodel.merge()\n\
         assert count(all volumes) == 2, \"fuel and moderator\"\n\
         mesh.size(targets: all volumes, size: 0.3)\n"
    )
}

/// The pin cell and a 3x3 lattice of it by `model.copy` (as in the W3b milestone), 18 volumes.
fn lattice_fixture(h: f64) -> String {
    format!(
        "{}let vs = all volumes\n\
         let cx = model.copy(entities: vs, count: 2, xform: {{kind: \"move\", delta: [1.26, 0.0, 0.0]}})\n\
         let cy = model.copy(entities: vs | cx.volumes, count: 2, xform: {{kind: \"move\", delta: [0.0, 1.26, 0.0]}})\n\
         model.imprint(bodies: all bodies)\nmodel.merge()\n\
         assert count(all volumes) == 18, \"3x3 lattice of two-volume pins\"\n\
         mesh.size(targets: all volumes, size: 0.3)\n",
        pin_fixture(h, true)
    )
}

const RUN_ALL: &str = "mesh.auto(strategy: \"default\")\nlet m = mesh.run(targets: all volumes)\n";

fn run_fixture(fixture: &str, tail: &str) -> (Server, Value) {
    let mut s = server();
    let v = ok(&mut s, &format!("{fixture}{tail}"));
    (s, v)
}

// ---- B1: a block is a block ----------------------------------------------------------------------

#[test]
fn b1_brick_auto_equals_the_explicit_map_script() {
    let mut explicit = server();
    ok(&mut explicit, BRICK_EXPLICIT);
    let want = info(&mut explicit);

    let mut s = server();
    ok(&mut s, BRICK);
    // The planner scores map 100 for an unconstrained brick before any `mesh.auto`.
    let vol = &volumes(&mut s)[0];
    let plan = plan_rows(&query(&mut s, "mesh.plan", json!({"targets": [vol]})));
    let row = scheme_of(&plan, vol);
    assert_eq!((row.scheme.as_str(), row.how.as_str(), row.score), ("map", "auto", Some(100)), "{plan:?}");

    let run = ok(&mut s, &format!("mesh.auto(strategy: \"default\")\nlet m = mesh.run(targets: [{}])\n", script_id(vol)));
    assert!(auto_tet(&run).is_empty(), "{:?}", diagnostics(&run, "mesh.auto"));
    let got = info(&mut s);
    assert_eq!(got["topology_hash"], want["topology_hash"]);
    assert_eq!(got["geometry_hash"], want["geometry_hash"]);
    assert_eq!(got["cells"], 64);
    // The stored record is the explicit choice, and replay never calls the strategy.
    let rows = mesh_plan(&mut s);
    assert_eq!(scheme_of(&rows, vol).scheme, "map");
}

// ---- B2: L-slab, sweep with a submap source ------------------------------------------------------

#[test]
fn b2_l_slab_sweeps_with_a_submap_source() {
    // Negative control: the strict registry lists (no `mesh.auto`) cannot mesh the L at all.
    let mut strict = server();
    let e = script(&mut strict, &format!("{L_SLAB}let m = mesh.run(targets: r.volumes)\n")).expect_err("strict lists fail");
    assert!(e.message.contains("no applicable scheme") && e.message.contains("8 faces; map needs exactly 6"), "{e}");

    let (mut s, run) = run_fixture(L_SLAB, RUN);
    assert!(auto_tet(&run).is_empty());
    let vol = volumes(&mut s)[0].clone();

    let rows = mesh_plan(&mut s);
    assert_eq!(scheme_of(&rows, &vol).scheme, "sweep");
    assert_eq!(faces_by_scheme(&rows, "submap").len(), 1, "{rows:?}");
    let i = info(&mut s);
    assert_eq!(i["cell_shapes"], json!({"Hex": 24}));
    assert_eq!(i["cells"], 24);
    assert!((i["volume"].as_f64().expect("volume") - 3.0).abs() < 1e-12, "{i}");
    assert!(valid(&mut s));
}

// ---- B3: triangular prism, pave source -----------------------------------------------------------

#[test]
fn b3_triangular_prism_sweeps_with_a_pave_source() {
    let (mut s, run) = run_fixture(PRISM, RUN);
    assert!(auto_tet(&run).is_empty());
    let vol = volumes(&mut s)[0].clone();
    let rows = mesh_plan(&mut s);
    assert_eq!(scheme_of(&rows, &vol).scheme, "sweep");
    assert_eq!(faces_by_scheme(&rows, "pave").len(), 1, "{rows:?}");
    let i = info(&mut s);
    assert_eq!(i["cell_shapes"], json!({"Hex": 6}), "{i}");
    assert!((i["volume"].as_f64().expect("volume") - 0.5).abs() < 1e-12, "{i}");
    assert!(valid(&mut s));
}

// ---- B4: tetrahedron -----------------------------------------------------------------------------

#[test]
fn b4_tetrahedron_falls_to_tetmesh_with_a_warning() {
    let (mut s, run) = run_fixture(TETRA, RUN);
    let vol = volumes(&mut s)[0].clone();
    let rows = mesh_plan(&mut s);
    assert_eq!(scheme_of(&rows, &vol).scheme, "tetmesh");
    let faces = rows.iter().filter(|r| r.kind == "surface").collect::<Vec<_>>();
    assert_eq!(faces.len(), 4, "{rows:?}");
    assert!(faces.iter().all(|r| r.how == "imposed" && r.scheme == "tridelaunay"), "{faces:?}");
    let i = info(&mut s);
    assert!((i["volume"].as_f64().expect("volume") - 1.0 / 6.0).abs() < 1e-12, "{i}");
    assert_eq!(i["cell_shapes"].as_object().map(|m| m.keys().cloned().collect::<Vec<_>>()), Some(vec!["Tet".to_owned()]));
    assert!(valid(&mut s));

    // `mesh.auto_tet`: names the volume, quotes why map and sweep were passed over, ends with the fix.
    let w = auto_tet(&run);
    assert_eq!(w.len(), 1, "{w:?}");
    let msg = w[0]["message"].as_str().expect("message");
    assert!(msg.contains("map: 4 faces; map needs exactly 6"), "{msg}");
    assert!(msg.contains("sweep: no pair of faces gives columns"), "{msg}");
    assert!(msg.contains("mesh.scheme") && msg.contains("sweep"), "{msg}");
    assert_eq!(w[0]["entities"], json!([vol]));
}

// ---- B5: plate with a hole --------------------------------------------------------------------------

/// A plate with a through hole meshes all-hex through `mesh.auto` (W3c-CYL-SCHEMES: the sweep strip check
/// picks the annulus pair). Ruling D5 (`ogrid` in `default`, with layers and score following the size): the annulus is an
/// O-grid with 20 circle nodes and 3 radial layers, so the volume is 16 minus the 20-gon (10 sin 18 degrees) and
/// within 0.5 percent of 16 - pi (the 12-sided pave polygon was 13, 1.1 percent).
#[test]
fn b5_plate_with_a_hole_meshes_all_hex() {
    let (mut s, run) = run_fixture(PLATE, RUN);
    let i = info(&mut s);
    let got = i["volume"].as_f64().expect("volume");
    assert!((got - (16.0 - 10.0 * (std::f64::consts::PI / 10.0).sin())).abs() < 1e-9, "O-grid polygon volume {got}");
    assert!((got - (16.0 - std::f64::consts::PI)).abs() / (16.0 - std::f64::consts::PI) < 0.005, "{got}");
    assert_eq!(i["cell_shapes"], json!({"Hex": 120}), "observed on the merged code: {i}");
    assert!(valid(&mut s));
    assert!(auto_tet(&run).is_empty(), "no tet fallback warning");

    // The workaround a user has today: an explicit tet mesher meshes it, volume within 2 percent of 16 - pi.
    let mut t = server();
    ok(&mut t, &format!("{PLATE}mesh.scheme(targets: r.volumes, scheme: \"tetmesh\")\nlet m = mesh.run(targets: r.volumes)\n"));
    let i = info(&mut t);
    let exact = 16.0 - std::f64::consts::PI;
    let got = i["volume"].as_f64().expect("volume");
    assert!((got - exact).abs() / exact < 0.02, "volume {got} vs {exact}");
    assert!(valid(&mut t));
    assert_eq!(i["cell_shapes"].as_object().map(|m| m.keys().cloned().collect::<Vec<_>>()), Some(vec!["Tet".to_owned()]), "{i}");
    assert!(i["rejections"].as_u64().is_some_and(|r| r <= 2), "{i}");
}

// ---- B6: auto next to a hard neighbour (pinned) --------------------------------------------------

/// `mesh.auto` next to a hard `tetmesh` neighbour (W3c-MESHER-REPLAN, ruling C2). The right volume's `map`
/// imposes quad on the shared face and the neighbour imposes tri, so the plan-time replan rejects `map`; the
/// volume falls to the best remaining scheme (`sweep` from the tri face: wedges conformal to the tets).
#[test]
fn b6_auto_next_to_a_hard_tet_neighbour_falls_back_to_a_conformal_scheme() {
    let mut s = server();
    ok(&mut s, &format!("{TWO_BRICKS_HARD_TET}mesh.auto(strategy: \"default\")\nlet m = mesh.run(targets: a.volume | b.volume)\n"));
    let i = info(&mut s);
    assert!((i["volume"].as_f64().expect("volume") - 16.0).abs() < 1e-9, "{i}");
    assert!(valid(&mut s), "the shared face conforms");
    let rows = mesh_plan(&mut s);
    let vols: Vec<&Row> = rows.iter().filter(|r| r.kind == "volume").collect();
    assert_eq!(vols.len(), 2, "{rows:?}");
    assert!(vols.iter().any(|r| r.scheme == "tetmesh"), "left stays tetmesh: {rows:?}");
    assert!(vols.iter().any(|r| r.scheme != "tetmesh" && r.scheme != "map"), "right is not map: {rows:?}");
    assert_eq!(i["cell_shapes"], json!({"Tet": 160, "Wedge": 32}), "observed on the final code: {i}");
    // The right volume's rejected `map` attempt is not observable through the public API: `mesh.auto` stores
    // explicit assignments (`intent#0`, empty `alternatives` in `mesh.plan`), the replan runs in a scratch
    // plan, `mesh.auto` emits no diagnostic when it does not fall to tets, and a plan before `mesh.auto` has no
    // surface or volume rows for this model. What is observable is the outcome and its recorded reason.
    let right = rows.iter().find(|r| r.kind == "volume" && r.scheme == "sweep").expect("right volume sweeps");
    assert_eq!(right.how, "intent#0", "{rows:?}");
    let report = query(&mut s, "mesh.plan", json!({"mesh": "Mesh#1"}));
    let reason = report["entities"].as_array().expect("entities").iter().find(|e| e["target"] == right.target.as_str()).expect("row")["reason"].as_str().expect("reason").to_owned();
    assert!(reason.contains("1-to-1 sweepable from"), "{reason}");

    // With the right volume also explicit, the hard neighbours mesh and the interface conforms.
    let mut t = server();
    ok(&mut t, &format!("{TWO_BRICKS_HARD_TET}mesh.scheme(targets: b.volume, scheme: \"tetmesh\")\nlet m = mesh.run(targets: a.volume | b.volume)\n"));
    let i = info(&mut t);
    assert!((i["volume"].as_f64().expect("volume") - 16.0).abs() < 1e-9, "{i}");
    assert!(valid(&mut t));
}

const B6_TEXT: &str = "mesh.auto(strategy: \"default\")\nlet m = mesh.run(targets: a.volume | b.volume)\n";

/// B6 determinism (acceptance 11): two runs, 1 thread vs default, and in-process vs CLI vs MCP.
#[test]
fn b6_hash_is_deterministic_and_equal_in_process_cli_and_mcp() {
    let text = format!("{TWO_BRICKS_HARD_TET}{B6_TEXT}");
    let hash = || {
        let mut s = server();
        ok(&mut s, &text);
        state_hash(&mut s)
    };
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    let a = one.install(hash);
    let want = hash();
    assert_eq!(a, want, "1 thread vs default");
    assert_eq!(want, hash(), "two runs");

    let dir = TempDir::new("b6-cli");
    let file = dir.path().join("b6.smesh");
    std::fs::write(&file, &text).expect("write script");
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let mut input = std::io::Cursor::new(Vec::new());
    let mut io = sm_cli::Io { stdin: &mut input, stdout: &mut out, stderr: &mut err };
    let mut connect = |s: &sm_cli::Settings| sm_cli::connect_standard(s);
    let args = ["smesh", "--memory", "--author", "tester", "--fixed-time", TIME, "run", file.to_str().expect("utf8"), "--print-hash"];
    let code = sm_cli::run(args, &mut connect, &mut io);
    let stdout = String::from_utf8_lossy(&out).into_owned();
    assert_eq!(code, 0, "{} {}", stdout, String::from_utf8_lossy(&err));
    assert_eq!(stdout.lines().last(), Some(want.as_str()), "CLI hash");

    let mut m = server();
    let msgs = [
        json!({"jsonrpc":"2.0","id":0,"method":"initialize","params":{"protocolVersion":"2025-06-18"}}),
        json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"run_script","arguments":{"text": text}}}),
    ];
    let input: String = msgs.iter().map(|m| format!("{m}\n")).collect();
    let mut out = Vec::new();
    sm_mcp::run_stdio(&mut m, std::io::Cursor::new(input.into_bytes()), &mut out).expect("mcp");
    let replies: Vec<Value> = String::from_utf8(out).expect("utf8").lines().map(|l| serde_json::from_str(l).expect("json")).collect();
    assert_ne!(replies[1]["result"]["isError"], json!(true), "{}", replies[1]);
    assert_eq!(state_hash(&mut m), want, "MCP hash");
}

// ---- B8: determinism, and the three front ends ---------------------------------------------------

fn scenario_hashes() -> Vec<String> {
    [L_SLAB, PRISM, TETRA]
        .iter()
        .map(|f| {
            let (mut s, _) = run_fixture(f, RUN);
            state_hash(&mut s)
        })
        .chain({
            let (mut s, _) = run_fixture(BRICK, "mesh.auto(strategy: \"default\")\nlet m = mesh.run(targets: a.volume)\n");
            [state_hash(&mut s)]
        })
        .collect()
}

#[test]
fn b8_auto_runs_are_deterministic_across_runs_and_thread_counts() {
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    let a = one.install(scenario_hashes);
    let b = scenario_hashes();
    let c = scenario_hashes();
    assert_eq!(a, b, "1 thread vs default");
    assert_eq!(b, c, "two runs");
    let mut distinct = a.clone();
    distinct.sort();
    distinct.dedup();
    assert_eq!(distinct.len(), a.len(), "the scenarios are different states");
}

#[test]
fn b8_l_slab_hash_is_equal_in_process_cli_and_mcp() {
    let text = format!("{L_SLAB}{RUN}");
    let mut direct = server();
    ok(&mut direct, &text);
    let want = state_hash(&mut direct);

    // CLI: `smesh run`.
    let dir = TempDir::new("auto-cli");
    let file = dir.path().join("l.smesh");
    std::fs::write(&file, &text).expect("write script");
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let mut input = std::io::Cursor::new(Vec::new());
    let mut io = sm_cli::Io { stdin: &mut input, stdout: &mut out, stderr: &mut err };
    let mut connect = |s: &sm_cli::Settings| sm_cli::connect_standard(s);
    let args = ["smesh", "--memory", "--author", "tester", "--fixed-time", TIME, "run", file.to_str().expect("utf8"), "--print-hash"];
    let code = sm_cli::run(args, &mut connect, &mut io);
    let stdout = String::from_utf8_lossy(&out).into_owned();
    assert_eq!(code, 0, "{} {}", stdout, String::from_utf8_lossy(&err));
    assert_eq!(stdout.lines().last(), Some(want.as_str()), "CLI hash");

    // MCP: `run_script`.
    let mut m = server();
    let msgs = [
        json!({"jsonrpc":"2.0","id":0,"method":"initialize","params":{"protocolVersion":"2025-06-18"}}),
        json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"run_script","arguments":{"text": text}}}),
    ];
    let input: String = msgs.iter().map(|m| format!("{m}\n")).collect();
    let mut out = Vec::new();
    sm_mcp::run_stdio(&mut m, std::io::Cursor::new(input.into_bytes()), &mut out).expect("mcp");
    let replies: Vec<Value> = String::from_utf8(out).expect("utf8").lines().map(|l| serde_json::from_str(l).expect("json")).collect();
    assert_ne!(replies[1]["result"]["isError"], json!(true), "{}", replies[1]);
    assert_eq!(state_hash(&mut m), want, "MCP hash");
}

// ---- B9: the trimesh / tridelaunay tie ------------------------------------------------------------

#[test]
fn b9_trimesh_and_tridelaunay_give_identical_meshes_under_a_tet_parent() {
    let hashes: Vec<(Value, Value)> = ["trimesh", "tridelaunay"]
        .iter()
        .map(|face_scheme| {
            let mut s = server();
            ok(
                &mut s,
                &format!(
                    "{TETRA}mesh.scheme(targets: topo.related(kind: \"TopoFace\", within: r.volumes), scheme: \"{face_scheme}\")\n\
                     mesh.scheme(targets: r.volumes, scheme: \"tetmesh\")\nlet m = mesh.run(targets: r.volumes)\n"
                ),
            );
            let i = info(&mut s);
            (i["topology_hash"].clone(), i["geometry_hash"].clone())
        })
        .collect();
    assert_eq!(hashes[0], hashes[1]);
}

// ---- B1/B7 census --------------------------------------------------------------------------------

fn census_config() -> MesherConfig {
    MesherConfig::new(standard_schemes().expect("schemes"))
        .with_solver(Box::new(IiaSolver::default()))
        .with_picker(Box::new(AnglePicker))
        .with_size_fields(Arc::new(sm_sizing::size_field))
        .with_auto_size(Arc::new(|view, targets, factor| sm_sizing::auto_size(view, targets, factor).map_err(|e| e.to_string())))
}

fn census_of(s: &mut Server) -> CensusReport {
    let cfg = census_config();
    let all: Vec<sm_core::EntityId> = volumes(s).iter().map(|v| v.parse().expect("entity id")).collect();
    sm_mesher::census(s.workspace().state(), &cfg, None, Some(&all)).expect("census")
}

/// Every model the repository's scripts build, one census row per volume: what `mesh.auto` would change
/// against what `mesh.run` does today. The test pins the rule that makes the old scripts safe (a volume with
/// an intent scheme never changes) and prints the changed cases for the report.
#[test]
fn census_of_the_fixture_models_and_the_repository_scripts() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut corpus: Vec<(String, String)> = vec![
        ("brick".into(), BRICK.into()),
        ("l-slab".into(), L_SLAB.into()),
        ("prism".into(), PRISM.into()),
        ("tetra".into(), TETRA.into()),
        ("plate".into(), PLATE.into()),
        ("two-bricks-hard-tet".into(), TWO_BRICKS_HARD_TET.into()),
    ];
    for rel in [
        "crates/sm-cli/tests/data/milestone.smesh",
        "crates/sm-cli/tests/data/fork.smesh",
        "crates/sm-cli/tests/data/w3a/head_a.smesh",
        "crates/sm-cli/tests/data/w3a/head_b.smesh",
        "crates/sm-cli/tests/data/w3a/tail_tet.smesh",
        "crates/sm-cli/tests/data/w3a/tail_hybrid.smesh",
        "crates/sm-cli/tests/data/w3a/checks.smesh",
        "crates/sm-py/tests/slice.smesh",
        "tools/conformance/fixtures/hexbox.smesh",
        "tools/conformance/fixtures/tetbox.smesh",
        "tools/conformance/fixtures/wedgeprism.smesh",
        "tools/conformance/fixtures/polydual.smesh",
    ] {
        if let Ok(text) = std::fs::read_to_string(root.join(rel)) {
            corpus.push((rel.to_owned(), text));
        }
    }
    // Scripts export to relative paths; send those to a temporary directory so the tree stays clean.
    let tmp = TempDir::new("census");
    let out = format!("\"{}/", tmp.path().display());
    let (mut volumes_seen, mut changed, mut report) = (0usize, 0usize, String::new());
    for (name, text) in &corpus {
        let text = text.replace("\"out.", &out);
        let mut s = server();
        // Includes and failures do not matter: the census reads whatever model the script got to.
        let _ = script(&mut s, &text);
        let census = census_of(&mut s);
        volumes_seen += census.rows.len();
        for row in census.changed() {
            changed += 1;
            report.push_str(&format!("| {name} | {} | {:?} | {:?} |\n", row.volume, row.before, row.after));
        }
        // A volume that carries an intent scheme is untouched by the strategy.
        for row in &census.rows {
            if row.before.selection.as_deref() == Some("Intent") {
                assert!(!row.changed, "{name}: {row:?}");
            }
        }
    }
    drop(tmp);
    println!("census: {} models, {volumes_seen} volumes, {changed} changed\n{report}", corpus.len());
    assert!(volumes_seen >= corpus.len() - 1);
    // The cases the acceptance fixtures are built for must be among the changed rows.
    assert!(changed >= 4, "L-slab, prism, tetra and plate change: {report}");
}

// ---- catalog ---------------------------------------------------------------------------------------

#[test]
fn standard_catalog_has_the_view_auto_and_record_entries() {
    let mut s = server();
    let cat = s.call(Request::Catalog).expect("catalog");
    let names = |key: &str| -> Vec<String> { cat[key].as_array().expect(key).iter().map(|o| o["name"].as_str().expect("name").to_owned()).collect() };
    let (ops, queries) = (names("ops"), names("queries"));
    assert!(ops.contains(&"mesh.auto".to_owned()), "{ops:?}");
    for q in ["view.scene", "view.mesh_skin", "view.mesh_scalar", "mesh.auto_strategies"] {
        assert!(queries.contains(&q.to_owned()), "{q} missing: {queries:?}");
    }
    // Queries and ops are catalogued once each, in sorted order.
    let mut sorted = queries.clone();
    sorted.sort();
    sorted.dedup();
    assert_eq!(sorted, queries);
}

// ---- W3c-CYL-PIN: round pin and lattice ------------------------------------------------------------

const PIN_VOLUME: f64 = 1.26 * 1.26;

/// Runs `text` and returns the server, the run result and `mesh.info`.
fn run_info(text: &str) -> (Server, Value, Value) {
    let mut s = server();
    let run = ok(&mut s, text);
    let i = info(&mut s);
    (s, run, i)
}

fn assert_pin(text: &str, volumes: f64, shapes: Value, tet_warnings: usize) -> String {
    let (mut s, run, i) = run_info(text);
    let got = i["volume"].as_f64().expect("volume");
    assert!((got - volumes).abs() < 1e-12, "volume {got} vs {volumes}: {i}");
    assert_eq!(i["cell_shapes"], shapes, "observed on the merged code: {i}");
    assert!(valid(&mut s), "valid and conformal: {i}");
    assert_eq!(auto_tet(&run).len(), tet_warnings, "mesh.auto_tet warnings");
    state_hash(&mut s)
}

/// Through pin (H = 1.0): all-hex, both creation orders. The cylinder sweeps a pave disk, the moderator an
/// annulus; the shared cylinder face is quads on both sides, so `validate` is empty.
#[test]
fn pin_through_is_all_hex_in_both_creation_orders() {
    let a = assert_pin(&format!("{}{RUN_ALL}", pin_fixture(1.0, true)), PIN_VOLUME, json!({"Hex": 48}), 0);
    let b = assert_pin(&format!("{}{RUN_ALL}", pin_fixture(1.0, false)), PIN_VOLUME, json!({"Hex": 48}), 0);
    assert_ne!(a, b, "different entity ids are different states");

    // Per-volume check against the polygon formula. The size-0.3 pave disk of the r = 0.4 cylinder is an
    // n = 8 polygon, so the curved interface is the inscribed octagon: fuel = H (n/2) r^2 sin(2 pi/n).
    // The draft's "2.2 percent of pi r^2" bound was written for n = 12 and does not hold at n = 8.
    let (n, r, h) = (8.0_f64, 0.4_f64, 1.0_f64);
    let fuel = h * (n / 2.0) * r * r * (2.0 * std::f64::consts::PI / n).sin();
    for (target, want, hex) in [("fu.volume", fuel, 12), ("mo.volumes", PIN_VOLUME - fuel, 36)] {
        let tail = format!("mesh.auto(strategy: \"default\")\nlet m = mesh.run(targets: {target})\n");
        let (mut s, _, i) = run_info(&format!("{}{tail}", pin_fixture(1.0, true)));
        let got = i["volume"].as_f64().expect("volume");
        assert!((got - want).abs() < 1e-12, "{target}: volume {got} vs polygon {want}: {i}");
        assert_eq!(i["cell_shapes"], json!({"Hex": hex}), "{target}: {i}");
        assert!(valid(&mut s), "{target}: {i}");
    }
}

/// Enclosed pin (H = 0.8, ruling C1): all-tet, with a `mesh.auto_tet` warning for each of the two volumes.
/// The planner replan that resolves the quad/tri conflict on the cylinder face runs inside `mesh.auto`
/// (scratch plan), so `mesh.run` plans nothing twice and `mesh.info.rejections` is 0: the warnings are the
/// evidence (acceptance 7 is asserted through them).
#[test]
fn pin_enclosed_is_all_tet_in_both_creation_orders() {
    for brick_first in [true, false] {
        let text = format!("{}{RUN_ALL}", pin_fixture(0.8, brick_first));
        let (mut s, run, i) = run_info(&text);
        assert_eq!(i["cell_shapes"], json!({"Tet": 733}), "brick_first={brick_first}: {i}");
        assert!((i["volume"].as_f64().expect("volume") - PIN_VOLUME).abs() < 1e-12, "{i}");
        assert!(valid(&mut s));
        assert_eq!(i["rejections"], 0, "the replan runs in a scratch plan inside mesh.auto: {i}");
        let w = auto_tet(&run);
        assert_eq!(w.len(), 2, "one warning per volume: {w:?}");
        assert!(w.iter().all(|d| d["entities"].as_array().map(Vec::len) == Some(1)), "{w:?}");
        // Each warning quotes why map and sweep were passed over, and ends with the fix.
        for d in &w {
            let msg = d["message"].as_str().expect("message");
            assert!(msg.contains("passed over: map: ") && msg.contains("map needs exactly 6; sweep: "), "{msg}");
            assert!(msg.contains("To keep hexes: `mesh.scheme"), "{msg}");
        }
        if brick_first {
            // Entity ids are fixed by the creation order: fuel is Volume#2, moderator Volume#4.
            let text_of = |v: &str| {
                let d = w.iter().find(|d| d["entities"] == json!([v])).unwrap_or_else(|| panic!("no warning for {v}: {w:?}"));
                d["message"].as_str().expect("message").to_owned()
            };
            let fuel = text_of("TopoVolume#2");
            assert!(fuel.contains("map: 3 faces; map needs exactly 6"), "{fuel}");
            assert!(fuel.contains("sweep: rejected after a NotApplicable failure in Resolve"), "{fuel}");
            assert!(fuel.contains("overridden by imposition"), "{fuel}");
            let moderator = text_of("TopoVolume#4");
            assert!(moderator.contains("map: 9 faces; map needs exactly 6"), "{moderator}");
            assert!(moderator.contains("sweep: no pair of faces gives columns"), "{moderator}");
        }
    }
}

/// The through pin's stored plan: the cylinder and the moderator are each a sweep from a pave source with a
/// copy target; the side faces and the shared cylinder face are map (imposed by the sweeps).
#[test]
fn pin_through_plan_rows_are_sweeps_of_pave_sources_with_copy_targets() {
    let (mut s, _, i) = run_info(&format!("{}{RUN_ALL}", pin_fixture(1.0, true)));
    assert_eq!(i["rejections"], 0, "{i}");
    let report = query(&mut s, "mesh.plan", json!({"mesh": "Mesh#1"}));
    let rows = plan_rows(&report);
    // Fuel is Volume#2, moderator Volume#4 (brick first).
    for v in ["TopoVolume#2", "TopoVolume#4"] {
        let r = scheme_of(&rows, v);
        assert_eq!((r.kind.as_str(), r.scheme.as_str(), r.how.as_str()), ("volume", "sweep", "intent#0"), "{v}: {rows:?}");
    }
    assert_eq!(rows.iter().filter(|r| r.kind == "volume").count(), 2, "{rows:?}");
    let pave = faces_by_scheme(&rows, "pave");
    assert_eq!(pave.len(), 2, "one pave source per volume: {rows:?}");
    assert!(pave.iter().all(|r| r.how == "intent#0"), "{pave:?}");
    let copies: Vec<&Row> = rows.iter().filter(|r| r.kind == "surface" && r.scheme == "copy").collect();
    assert_eq!(copies.len(), 2, "one copy target per volume: {rows:?}");
    // Every copy target copies one of the pave sources, and no two copy the same one.
    let mut sources: Vec<&str> = pave.iter().map(|r| r.target.as_str()).collect();
    sources.sort_unstable();
    let mut copied: Vec<&str> = Vec::new();
    for c in &copies {
        let e = report["entities"].as_array().expect("entities").iter().find(|e| e["target"] == c.target.as_str()).expect("copy entity");
        let of = e["copy_of"].as_str().unwrap_or_else(|| panic!("copy_of: {e}"));
        assert!(sources.contains(&of), "{} copies {of}, sources {sources:?}", c.target);
        copied.push(of);
    }
    copied.sort_unstable();
    assert_eq!(copied, sources, "each pave source is copied exactly once (two rows share a `copy_of` otherwise)");
    // The four brick sides and the cylinder face map; nothing else is a face scheme.
    let mut map: Vec<&str> = faces_by_scheme(&rows, "map").iter().map(|r| r.target.as_str()).collect();
    map.sort();
    assert_eq!(map, ["TopoFace#1", "TopoFace#2", "TopoFace#3", "TopoFace#4", "TopoFace#9"], "{rows:?}");
    assert_eq!(rows.iter().filter(|r| r.kind == "surface").count(), 9, "{rows:?}");
}

/// 3x3 lattices by `model.copy`: 18 volumes each, through all-hex, enclosed all-tet.
#[test]
fn pin_lattices_through_and_enclosed() {
    assert_pin(&format!("{}{RUN_ALL}", lattice_fixture(1.0)), 9.0 * PIN_VOLUME, json!({"Hex": 432}), 0);
    let (mut s, _, i) = run_info(&format!("{}{RUN_ALL}", lattice_fixture(0.8)));
    assert_eq!(i["selection"].as_array().map(Vec::len), Some(18));
    assert_eq!(i["cell_shapes"], json!({"Tet": 6647}), "{i}");
    assert!((i["volume"].as_f64().expect("volume") - 9.0 * PIN_VOLUME).abs() < 1e-12, "{i}");
    assert!(valid(&mut s));
}

fn pin_scenario_hashes() -> Vec<String> {
    let texts = [
        format!("{}{RUN_ALL}", pin_fixture(1.0, true)),
        format!("{}{RUN_ALL}", pin_fixture(0.8, true)),
        format!("{}{RUN_ALL}", lattice_fixture(1.0)),
        format!("{}{RUN_ALL}", lattice_fixture(0.8)),
        format!("{PLATE}{RUN}"),
    ];
    texts
        .iter()
        .map(|t| {
            let mut s = server();
            ok(&mut s, t);
            state_hash(&mut s)
        })
        .collect()
}

/// Two runs and 1 thread vs default give the same state hashes for pins, lattices and B5.
#[test]
fn pin_runs_are_deterministic_across_runs_and_thread_counts() {
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    let a = one.install(pin_scenario_hashes);
    let b = pin_scenario_hashes();
    let c = pin_scenario_hashes();
    assert_eq!(a, b, "1 thread vs default");
    assert_eq!(b, c, "two runs");
    let mut distinct = a.clone();
    distinct.sort();
    distinct.dedup();
    assert_eq!(distinct.len(), a.len(), "the scenarios are different states");
}

/// In-process, CLI (`smesh run --print-hash`) and MCP (`run_script`) reach the same state hash for `text`.
fn assert_three_way_hash(name: &str, text: &str) {
    let mut direct = server();
    ok(&mut direct, text);
    let want = state_hash(&mut direct);

    let dir = TempDir::new("pin-cli");
    let file = dir.path().join("pin.smesh");
    std::fs::write(&file, text).expect("write script");
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let mut input = std::io::Cursor::new(Vec::new());
    let mut io = sm_cli::Io { stdin: &mut input, stdout: &mut out, stderr: &mut err };
    let mut connect = |s: &sm_cli::Settings| sm_cli::connect_standard(s);
    let args = ["smesh", "--memory", "--author", "tester", "--fixed-time", TIME, "run", file.to_str().expect("utf8"), "--print-hash"];
    let code = sm_cli::run(args, &mut connect, &mut io);
    let stdout = String::from_utf8_lossy(&out).into_owned();
    assert_eq!(code, 0, "{name}: {} {}", stdout, String::from_utf8_lossy(&err));
    assert_eq!(stdout.lines().last(), Some(want.as_str()), "{name}: CLI hash");

    let mut m = server();
    let msgs = [
        json!({"jsonrpc":"2.0","id":0,"method":"initialize","params":{"protocolVersion":"2025-06-18"}}),
        json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"run_script","arguments":{"text": text}}}),
    ];
    let input: String = msgs.iter().map(|m| format!("{m}\n")).collect();
    let mut out = Vec::new();
    sm_mcp::run_stdio(&mut m, std::io::Cursor::new(input.into_bytes()), &mut out).expect("mcp");
    let replies: Vec<Value> = String::from_utf8(out).expect("utf8").lines().map(|l| serde_json::from_str(l).expect("json")).collect();
    assert_ne!(replies[1]["result"]["isError"], json!(true), "{name}: {}", replies[1]);
    assert_eq!(state_hash(&mut m), want, "{name}: MCP hash");
}

/// In-process, CLI and MCP agree on the single through pin, the through lattice, the enclosed pin and B5.
#[test]
fn pin_hashes_are_equal_in_process_cli_and_mcp() {
    for (name, text) in [
        ("through pin", format!("{}{RUN_ALL}", pin_fixture(1.0, true))),
        ("lattice", format!("{}{RUN_ALL}", lattice_fixture(1.0))),
        ("enclosed", format!("{}{RUN_ALL}", pin_fixture(0.8, true))),
        ("b5 plate", format!("{PLATE}{RUN}")),
    ] {
        assert_three_way_hash(name, &text);
    }
}
