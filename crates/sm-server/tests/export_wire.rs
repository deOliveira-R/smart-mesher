//! W3b-EXPORT-WIRE: DAGMC `.h5m`, Gmsh `.msh` and the `.smrt` handover through `session.export`
//! (in-process, CLI and MCP), and `model.export` with a `path`.
mod common;

use std::cell::RefCell;
use std::io::Cursor;
use std::path::Path;
use std::rc::Rc;

use base64::Engine as _;
use common::TempDir;
use serde_json::{Value, json};
use sm_geom_api::Ray;
use sm_layer_api::MeshLayer;
use sm_op_api::Hash256;
use sm_protocol::{Endpoint, ExportFormat, ExportResult, NoteSeverity, Request, RpcError};
use sm_rt_api::{BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, MeshInput, PartitionOptions, RecoveryPolicy, RegionRule, TraceOptions, Tracker};
use sm_server::{Server, standard_with};

const TIME: &str = "2026-10-24T00:00:00Z";

fn builder() -> sm_engine::WorkspaceBuilder {
    standard_with(Some("tester".into()), Some(TIME.into()))
}

const TWO_BRICKS: &str = r#"
let a = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [1.0, 1.0, 1.0])
let b = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [3.0, 1.0, 1.0])
model.imprint(bodies: a.body | b.body)
model.merge(entities: a.body | b.body)
fem.block_add(block: 1, volumes: a.volume, name: "fuel")
fem.block_add(block: 2, volumes: b.volume, name: "water")
mesh.size(targets: a.volume | b.volume, size: 1.0)
"#;

const HEX: &str = r#"
mesh.scheme(targets: a.volume | b.volume, scheme: "map")
let m = mesh.run(targets: a.volume | b.volume)
"#;

/// A brick cut by three oblique planes into a general hexahedron, map-meshed: its interior
/// faces are warped. (The dual `POLY` below has no face above the warp tolerance, so it cannot
/// exercise the pass.)
const WARPED: &str = r#"
let a = model.create_brick(x: 4.0, y: 4.0, z: 4.0, center: [2.0, 2.0, 2.0])
let c1 = model.webcut_plane(targets: a.body, plane: {origin: [3.0, 0.0, 0.0], normal: [1.0, 0.4, 0.0]})
let c2 = model.webcut_plane(targets: c1.negative, plane: {origin: [0.0, 0.0, 3.0], normal: [0.3, 0.1, 1.0]})
let c3 = model.webcut_plane(targets: c2.negative, plane: {origin: [0.0, 3.0, 0.0], normal: [0.2, 1.0, 0.3]})
let v = topo.related(kind: "TopoVolume", within: c3.negative)
mesh.size(targets: v, size: 0.5)
mesh.scheme(targets: v, scheme: "map")
let m = mesh.run(targets: v)
"#;

const POLY: &str = r#"
mesh.scheme(targets: topo.related(kind: "TopoFace", within: a.volume | b.volume), scheme: "trimesh")
mesh.scheme(targets: a.volume | b.volume, scheme: "tetmesh")
let mt = mesh.run(targets: a.volume | b.volume)
let pm = mesh.to_poly(mesh: mt.mesh[0], keep_source: false)
"#;

fn server(parts: &[&str]) -> Server {
    let mut s = Server::new(builder()).expect("server");
    let text = parts.concat();
    s.call(Request::ScriptRun { text, path: None, includes: Default::default() }).unwrap_or_else(|e| panic!("fixture script: {e} {}", e.data));
    s
}

fn hex_server() -> Server {
    server(&[TWO_BRICKS, HEX])
}

fn export_req(path: &Path, format: Option<ExportFormat>, options: Option<Value>) -> Request {
    Request::Export {
        path: Some(path.to_string_lossy().into_owned()),
        format,
        mesh: None,
        provenance: false,
        inline: false,
        options,
        split_small_determinant: None,
        empty_patches: vec![],
        wedge_patches: vec![],
        split_warped_faces: false,
    }
}

fn export(s: &mut Server, path: &Path, format: Option<ExportFormat>, options: Option<Value>) -> Result<ExportResult, RpcError> {
    let v = s.call(export_req(path, format, options))?;
    Ok(serde_json::from_value(v).expect("ExportResult"))
}

fn read(p: &Path) -> Vec<u8> {
    std::fs::read(p).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
}

/// A server shared between the in-process calls and the CLI.
struct Shared(Rc<RefCell<Server>>);

impl Endpoint for Shared {
    fn call(&mut self, req: Request) -> Result<Value, RpcError> {
        self.0.borrow_mut().call(req)
    }
}

/// `smesh <args>` against `server`; returns (exit code, stdout, stderr).
fn cli(server: &Rc<RefCell<Server>>, args: &[&str]) -> (i32, String, String) {
    let (mut out, mut err) = (Vec::new(), Vec::new());
    let mut input = Cursor::new(Vec::new());
    let mut io = sm_cli::Io { stdin: &mut input, stdout: &mut out, stderr: &mut err };
    let mut connect = |_: &sm_cli::Settings| -> Result<Box<dyn Endpoint>, String> { Ok(Box::new(Shared(Rc::clone(server)))) };
    let argv: Vec<&str> = std::iter::once("smesh").chain(args.iter().copied()).collect();
    let code = sm_cli::run(argv, &mut connect, &mut io);
    (code, String::from_utf8_lossy(&out).into_owned(), String::from_utf8_lossy(&err).into_owned())
}

/// One MCP `tools/call`; returns the `result` object.
fn mcp(server: &mut Server, tool: &str, args: Value) -> Value {
    let msgs = [
        json!({"jsonrpc":"2.0","id":0,"method":"initialize","params":{"protocolVersion":"2025-06-18"}}),
        json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":tool,"arguments":args}}),
    ];
    let input: String = msgs.iter().map(|m| format!("{m}\n")).collect();
    let mut out = Vec::new();
    sm_mcp::run_stdio(server, Cursor::new(input.into_bytes()), &mut out).expect("run");
    let replies: Vec<Value> = String::from_utf8(out).expect("utf8").lines().map(|l| serde_json::from_str(l).expect("json")).collect();
    replies[1].clone()
}

#[test]
fn each_format_is_byte_identical_in_process_cli_and_mcp() {
    let dir = TempDir::new("wire-same");
    let s = Rc::new(RefCell::new(hex_server()));
    for ext in ["h5m", "msh", "smrt"] {
        let p0 = dir.path().join(format!("inproc.{ext}"));
        let r = export(&mut s.borrow_mut(), &p0, None, None).unwrap_or_else(|e| panic!("{ext}: {e} {}", e.data));
        let want = read(&p0);
        assert!(!want.is_empty() && r.bytes as usize == want.len(), "{ext}");
        assert_eq!(r.sha256, Hash256::of(&want), "{ext}");

        let p1 = dir.path().join(format!("cli.{ext}"));
        let (code, _, err) = cli(&s, &["export", p1.to_str().expect("utf8")]);
        assert_eq!(code, 0, "{ext}: {err}");
        assert_eq!(read(&p1), want, "{ext}: cli differs");

        let p2 = dir.path().join(format!("mcp.{ext}"));
        let reply = mcp(&mut s.borrow_mut(), "session_export", json!({"path": p2.to_string_lossy()}));
        assert!(reply["result"]["isError"] != json!(true), "{ext}: {reply}");
        assert_eq!(read(&p2), want, "{ext}: mcp differs");
    }
}

#[test]
fn options_reach_the_writers_through_every_front_end() {
    let dir = TempDir::new("wire-opts");
    let s = Rc::new(RefCell::new(hex_server()));
    let ascii = dir.path().join("ascii.msh");
    export(&mut s.borrow_mut(), &ascii, None, None).expect("ascii");
    let bin = dir.path().join("bin.msh");
    export(&mut s.borrow_mut(), &bin, None, Some(json!({"binary": true}))).expect("binary");
    let (a, b) = (read(&ascii), read(&bin));
    assert_ne!(a, b, "binary differs from ascii");
    assert!(b.starts_with(b"$MeshFormat\n4.1 1 8\n"), "binary header: {:?}", String::from_utf8_lossy(&b[..24.min(b.len())]));
    assert!(a.starts_with(b"$MeshFormat\n4.1 0 8\n"));
    let parsed = sm_meshio::gmsh::parse_msh4(&b).expect("binary parses");
    assert_eq!(parsed.node_count(), sm_meshio::gmsh::parse_msh4(&a).expect("ascii parses").node_count());

    let cli_bin = dir.path().join("cli_bin.msh");
    let (code, _, err) = cli(&s, &["export", cli_bin.to_str().expect("utf8"), "--option", "binary=true"]);
    assert_eq!(code, 0, "{err}");
    assert_eq!(read(&cli_bin), b);
    let mcp_bin = dir.path().join("mcp_bin.msh");
    let reply = mcp(&mut s.borrow_mut(), "session_export", json!({"path": mcp_bin.to_string_lossy(), "options": {"binary": true}}));
    assert!(reply["result"]["isError"] != json!(true), "{reply}");
    assert_eq!(read(&mcp_bin), b);

    // The DAGMC graveyard is opt-in and named `mat:Graveyard`.
    let plain = dir.path().join("plain.h5m");
    export(&mut s.borrow_mut(), &plain, None, None).expect("plain");
    let grave = dir.path().join("grave.h5m");
    export(&mut s.borrow_mut(), &grave, None, Some(json!({"graveyard": true}))).expect("graveyard");
    let (p, g) = (read(&plain), read(&grave));
    let has = |bytes: &[u8], needle: &[u8]| bytes.windows(needle.len()).any(|w| w == needle);
    assert!(!has(&p, b"mat:Graveyard") && has(&g, b"mat:Graveyard"));
    assert!(has(&p, b"mat:fuel") && has(&p, b"mat:water"), "material groups from the block names");
    let cli_grave = dir.path().join("cli_grave.h5m");
    let (code, _, err) = cli(&s, &["export", cli_grave.to_str().expect("utf8"), "--option", "graveyard=true"]);
    assert_eq!(code, 0, "{err}");
    assert_eq!(read(&cli_grave), g);
}

#[test]
fn polyhedra_are_a_typed_error_with_the_hint_unless_a_policy_is_given() {
    let dir = TempDir::new("wire-poly");
    let mut s = server(&[TWO_BRICKS, POLY]);
    let p = dir.path().join("poly.msh");
    let e = export(&mut s, &p, None, None).expect_err("polyhedra refused by default");
    assert_eq!(e.kind, "polyhedra-unsupported", "{e}");
    assert!(e.data["hint"].is_string() && e.data["count"].as_u64().unwrap_or(0) > 0, "{}", e.data);
    assert!(e.message.contains("polyhedra"), "{}", e.message);
    assert!(!p.exists(), "nothing is written on failure");
    for policy in ["decompose", "drop"] {
        export(&mut s, &p, None, Some(json!({"polyhedra": policy}))).unwrap_or_else(|e| panic!("{policy}: {e}"));
        assert!(!read(&p).is_empty());
    }
    let r = export(&mut s, &p, None, Some(json!({"polyhedra": "drop"}))).expect("drop");
    assert!(r.notes.iter().any(|n| n.code == "msh.poly_dropped"), "{:?}", r.notes);
}

#[test]
fn loss_notes_carry_the_format_codes() {
    let dir = TempDir::new("wire-notes");
    // No material groups: the volumes are in no set.
    let mut s = server(&[
        "let a = model.create_brick(x: 1.0, y: 1.0, z: 1.0)\nmesh.size(targets: a.volume, size: 0.5)\nmesh.scheme(targets: a.volume, scheme: \"map\")\nmesh.run(targets: a.volume)\n",
    ]);
    let r = export(&mut s, &dir.path().join("a.h5m"), None, None).expect("h5m");
    assert!(r.notes.iter().any(|n| n.code == "h5m.volume_without_material"), "{:?}", r.notes);
    assert!(r.notes.iter().all(|n| n.code.starts_with("h5m.")), "{:?}", r.notes);
    assert!(r.notes.iter().all(|n| n.severity == NoteSeverity::Info), "{:?}", r.notes);
    // Inline carries the same bytes as the file.
    let v = s
        .call(Request::Export {
            path: None,
            format: Some(ExportFormat::Dagmc),
            mesh: None,
            provenance: false,
            inline: true,
            options: None,
            split_small_determinant: None,
            empty_patches: vec![],
            wedge_patches: vec![],
            split_warped_faces: false,
        })
        .expect("inline");
    let inline: ExportResult = serde_json::from_value(v).expect("result");
    let bytes = base64::engine::general_purpose::STANDARD.decode(inline.base64.expect("base64")).expect("decode");
    assert_eq!(bytes, read(&dir.path().join("a.h5m")));
    assert_eq!(inline.notes, r.notes);
}

#[test]
fn unknown_option_keys_are_rejected_per_format() {
    let dir = TempDir::new("wire-badopt");
    let mut s = hex_server();
    let cases = [
        ("a.msh", json!({"graveyard": true})),
        ("a.msh", json!({"binary": "yes"})),
        ("a.msh", json!({"polyhedra": "repair"})),
        ("a.h5m", json!({"binary": true})),
        ("a.h5m", json!({"graveyard": 3})),
        ("a.smrt", json!({"binary": true})),
        ("a.smsh", json!({"binary": true})),
        ("a.vtu", json!({"graveyard": true})),
    ];
    for (name, options) in cases {
        let p = dir.path().join(name);
        let e = export(&mut s, &p, None, Some(options.clone())).expect_err(&format!("{name} {options}"));
        assert_eq!(e.kind, "bad-request", "{name} {options}: {e}");
        assert!(!p.exists(), "{name} {options}: nothing written");
    }
    let e = export(&mut s, &dir.path().join("a.msh"), None, Some(json!({"graveyard": true}))).expect_err("bad key");
    assert!(e.message.contains("graveyard") && e.message.contains("binary"), "{}", e.message);
    // A format that disagrees with the extension is still refused.
    let e = export(&mut s, &dir.path().join("a.msh"), Some(ExportFormat::Dagmc), None).expect_err("mismatch");
    assert_eq!(e.kind, "bad-request");
}

#[test]
fn exports_are_deterministic_across_runs_and_thread_counts() {
    let dir = TempDir::new("wire-det");
    let mut s = hex_server();
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    for ext in ["h5m", "msh", "smrt"] {
        let p = dir.path().join(format!("x.{ext}"));
        let first = export(&mut s, &p, None, None).expect("first").sha256;
        let again = export(&mut s, &p, None, None).expect("again").sha256;
        let single = one.install(|| export(&mut s, &p, None, None).expect("1 thread").sha256);
        assert_eq!(first, again, "{ext}");
        assert_eq!(first, single, "{ext}: 1 thread");
        assert_eq!(Hash256::of(&read(&p)), first);
    }
    // A freshly built session gives the same bytes.
    let mut t = hex_server();
    let (p, q) = (dir.path().join("s.smrt"), dir.path().join("t.smrt"));
    export(&mut s, &p, None, None).expect("s");
    export(&mut t, &q, None, None).expect("t");
    assert_eq!(read(&p), read(&q));
}

#[test]
fn smrt_loads_to_the_in_process_tracker() {
    let dir = TempDir::new("wire-smrt");
    let mut s = hex_server();
    let p = dir.path().join("a.smrt");
    let r = export(&mut s, &p, None, None).expect("smrt");
    assert!(r.notes.is_empty(), "a fully blocked model exports with no note: {:?}", r.notes);
    let image = sm_rt::read_smrt(&read(&p)).expect("read_smrt");
    let loaded = sm_rt::load_image(&image).expect("load_image");

    // Q2: the regions carry the block (material) names, not volume ids. The image also carries the
    // majorant partition (W3b-INT D1), which `rt.info`'s default build does not have.
    let state = s.workspace().state();
    let layer = state.layer::<MeshLayer>().expect("meshes");
    let (_, entry) = layer.meshes().next().expect("one mesh");
    assert!(loaded.caps().has(sm_rt_api::TrackerCaps::MAJORANT), "the handover image carries the partition");
    assert!(loaded.majorant_view().is_some_and(|v| v.hierarchy().node_count() > 0));
    let loaded_names: Vec<String> = loaded.regions().0.iter().map(|r| r.name.clone()).collect();
    assert_eq!(loaded_names, ["fuel", "water"], "regions are named by block");

    // Built in process from the same mesh, with the block sets lowered onto it.
    let model = state.layer::<sm_layer_api::ModelLayer>().expect("model");
    let none = sm_fem::FemLayer::default();
    let lowered = sm_fem::lower(model, entry, state.layer::<sm_fem::FemLayer>().unwrap_or(&none), layer.alloc()).expect("lower");
    let (rules, complement, spec) = (RegionRule::ByCellSets(vec!["fuel".into(), "water".into()]), ComplementPolicy::Outside, BoundarySpec::default());
    let input = BuildInput::Mesh(MeshInput::new((*lowered.mesh).as_read(), &rules, &spec, &complement));
    let mut part = PartitionOptions::default();
    part.enabled = true;
    let mut opts = BuildOptions::default().with_partition(part);
    opts.recovery = RecoveryPolicy::Fail;
    let (built, _) = sm_rt::MeshTrackerBuilder.build_concrete(&input, &opts).expect("build");
    assert_eq!(built.structure_hash(), loaded.structure_hash());

    // 1e4 seeded rays through both: bit-identical bundles.
    let mut rng = sm_testkit::surf3::SplitMix64(0x5eed);
    let rays: Vec<Ray> = (0..10_000)
        .map(|_| {
            let o = |lo: f64, hi: f64, r: &mut sm_testkit::surf3::SplitMix64| lo + (hi - lo) * (r.unit() + 1.0) / 2.0;
            let origin = sm_core::Point3::new(o(-1.0, 5.0, &mut rng), o(-1.0, 3.0, &mut rng), o(-1.0, 3.0, &mut rng));
            let dir = sm_core::Vector3::new(rng.unit(), rng.unit(), rng.unit());
            Ray { origin, dir }
        })
        .collect();
    let (a, b) = (built.trace_bundle(&rays, &TraceOptions::default()).expect("built"), loaded.trace_bundle(&rays, &TraceOptions::default()).expect("loaded"));
    assert_eq!(a.content_hash(), b.content_hash());
}

fn staged_part(s: &mut Server) {
    let r = s.stage_input(sm_testkit::w3a::STEP).expect("stage");
    s.call(Request::Apply { op: "model.import_step".into(), params: json!({"source": {"sha256": r.sha256}}), version: None }).expect("import");
}

#[test]
fn model_export_with_a_path_writes_the_bytes_outside_history() {
    let dir = TempDir::new("wire-step");
    let mut s = Server::new(builder()).expect("server");
    staged_part(&mut s);
    let (hash, nodes) = (s.workspace().state_hash(), s.workspace().timeline().len());

    // Without a path the result is returned as is.
    let plain = s.call(Request::Query { name: "model.export".into(), input: json!({}), at: None }).expect("inline");
    let inline = base64::engine::general_purpose::STANDARD.decode(plain["step"].as_str().expect("step")).expect("b64");
    assert!(!dir.path().join("part.step").exists());

    let path = dir.path().join("part.step");
    let with = s
        .call(Request::Query { name: "model.export".into(), input: json!({"path": path.to_string_lossy()}), at: None })
        .expect("with path");
    assert_eq!(with, plain, "the reply is the same with or without a path");
    assert_eq!(read(&path), inline, "file is byte-equal to the inline bytes");
    assert_eq!((s.workspace().state_hash(), s.workspace().timeline().len()), (hash, nodes), "no timeline node, no hash change");
    assert!(std::fs::read_dir(dir.path()).expect("dir").all(|e| !e.expect("e").file_name().to_string_lossy().ends_with(".part")), "no temp file left");

    // Re-import gives the three volumes.
    let mut t = Server::new(builder()).expect("server");
    let r = t.stage_input(&read(&path)).expect("stage");
    let out = t.call(Request::Apply { op: "model.import_step".into(), params: json!({"source": {"sha256": r.sha256}}), version: None }).expect("import");
    let vols = out["delta"]["outputs"]["volumes"].as_array().expect("volumes").clone();
    assert_eq!(vols.len(), 3);
    for (v, want) in vols.iter().zip(sm_testkit::w3a::VOLUMES) {
        let m = t.call(Request::Query { name: "model.measure".into(), input: json!({"entity": v}), at: None }).expect("measure");
        let got = m["volume"].as_f64().expect("volume");
        assert!(((got - want) / want).abs() <= 1e-9, "{got} vs {want}");
    }

    // A script call writes too, and a bad path is a typed io error.
    let script = dir.path().join("script.step");
    let text = format!("model.export(path: {:?})\n", script.to_string_lossy());
    s.call(Request::ScriptRun { text, path: None, includes: Default::default() }).expect("script");
    assert_eq!(read(&script), inline);
    let bad = s
        .call(Request::Query { name: "model.export".into(), input: json!({"path": dir.path().join("no/such/dir/x.step").to_string_lossy()}), at: None })
        .expect_err("io");
    assert_eq!(bad.kind, "io");
}

/// Plugin installation in `Server::open` stays on the calling thread: the scoped `OPENING`
/// binding that points a builder's input store at the project is thread-local.
#[test]
fn plugins_install_on_the_calling_thread_when_a_project_opens() {
    use sm_op_api::{Plugin, Registry, RegistryError};
    use std::sync::{Arc, Mutex};
    use std::thread::ThreadId;

    struct Probe(Arc<Mutex<Vec<ThreadId>>>);
    impl Plugin for Probe {
        fn id(&self) -> &str {
            "test.thread-probe"
        }
        fn register(&self, _: &mut Registry) -> Result<(), RegistryError> {
            self.0.lock().expect("lock").push(std::thread::current().id());
            Ok(())
        }
    }

    let dir = TempDir::new("wire-open-thread");
    let project = dir.path().join("p");
    drop(Server::create(builder(), &project, "p").expect("create"));
    let seen = Arc::new(Mutex::new(Vec::new()));
    let probe = Arc::clone(&seen);
    let opener = std::thread::spawn(move || {
        let me = std::thread::current().id();
        let s = Server::open(builder().plugin(Probe(probe)), &project).expect("open");
        // The open bound the distribution's input store to this project, on this thread.
        assert!(s.stage_input(b"x").is_ok());
        me
    });
    let me = opener.join().expect("thread");
    let ids = seen.lock().expect("lock").clone();
    assert!(!ids.is_empty(), "the probe plugin was installed");
    assert!(ids.iter().all(|t| *t == me), "every install ran on the opening thread: {ids:?} vs {me:?}");
    assert_ne!(me, std::thread::current().id());
}

#[test]
fn script_help_lists_the_new_formats() {
    let mut s = Server::new(builder()).expect("server");
    let r = s.call(Request::ScriptRun { text: "session.export(\"a.xyz\", format: \"nope\")\n".into(), path: None, includes: Default::default() }).expect_err("unknown format");
    let rendered = r.data["rendered"].as_str().unwrap_or_default().to_owned() + &r.message;
    for f in ["dagmc", "gmsh", "smrt"] {
        assert!(rendered.contains(f), "{f} missing from: {rendered}");
    }
}

#[test]
fn smrt_regions_fall_back_to_volume_ids_without_blocks() {
    let dir = TempDir::new("wire-smrt-nob");
    let script: String = TWO_BRICKS.lines().filter(|l| !l.contains("block_add")).collect::<Vec<_>>().join("\n");
    assert!(!script.contains("block_add"));
    let mut s = server(&[&script, HEX]);
    let p = dir.path().join("a.smrt");
    let r = export(&mut s, &p, None, None).expect("smrt");
    let image = sm_rt::read_smrt(&read(&p)).expect("read_smrt");
    let loaded = sm_rt::load_image(&image).expect("load_image");
    let names: Vec<String> = loaded.regions().0.iter().map(|r| r.name.clone()).collect();
    assert!(names.len() >= 2 && names.iter().all(|n| n.starts_with("TopoVolume#")), "{names:?}");
    assert_eq!(r.notes.len(), 1, "{:?}", r.notes);
    let n = &r.notes[0];
    assert_eq!(n.code, "smrt-region-names");
    assert_eq!(n.severity, NoteSeverity::Warning);
    assert!(n.detail.contains("no blocks") && n.detail.contains("fem.block_add"), "{}", n.detail);
}

#[test]
fn smrt_warns_when_a_volume_is_in_no_block() {
    let dir = TempDir::new("wire-smrt-partial");
    let script: String = TWO_BRICKS.lines().filter(|l| !l.contains("block: 2")).collect::<Vec<_>>().join("\n");
    let mut s = server(&[&script, HEX]);
    let p = dir.path().join("a.smrt");
    let r = export(&mut s, &p, None, None).expect("smrt");
    assert_eq!(r.notes.len(), 1, "{:?}", r.notes);
    assert!(r.notes[0].is_warning() && r.notes[0].code == "smrt-region-names", "{:?}", r.notes);
    assert!(r.notes[0].detail.contains("cells are in no block"), "{}", r.notes[0].detail);
    let loaded = sm_rt::load_image(&sm_rt::read_smrt(&read(&p)).expect("read_smrt")).expect("load_image");
    let names: Vec<String> = loaded.regions().0.iter().map(|r| r.name.clone()).collect();
    assert!(names.iter().all(|n| n.starts_with("TopoVolume#")), "{names:?}");
}

/// A block and a sideset cannot share a name: the op refuses the second one in either order, so
/// `smrt_regions` never sees two sets of one name and the first-match lookup of `ByCellSets`
/// cannot mis-pick. (`sm_fem::lower` also rejects collisions with mesh sets.)
#[test]
fn smrt_block_and_sideset_names_cannot_collide() {
    let dir = TempDir::new("wire-smrt-dup");
    let faces = "topo.related(kind: \"TopoFace\", within: a.volume)";
    let run = |text: String| Server::new(builder()).expect("server").call(Request::ScriptRun { text, path: None, includes: Default::default() });
    let side = format!("fem.sideset_add(sideset: 1, faces: {faces}, name: \"fuel\")\n");
    let e = run(format!("{TWO_BRICKS}{side}")).expect_err("sideset named like a block");
    assert!(e.to_string().contains("already used"), "{e}");
    // Sideset first, block second: refused too.
    let no_blocks: String = TWO_BRICKS.lines().filter(|l| !l.contains("fem.block_add")).collect::<Vec<_>>().join("\n");
    let e = run(format!("{no_blocks}\n{side}fem.block_add(block: 1, volumes: a.volume, name: \"fuel\")\n")).expect_err("block named like a sideset");
    assert!(e.to_string().contains("already used"), "{e}");
    // The surviving state exports with block names and no warning.
    let mut s = server(&[TWO_BRICKS, &format!("fem.sideset_add(sideset: 1, faces: {faces}, name: \"outer\")\n"), HEX]);
    let p = dir.path().join("a.smrt");
    let r = export(&mut s, &p, None, None).expect("smrt");
    assert!(r.notes.is_empty(), "{:?}", r.notes);
    let loaded = sm_rt::load_image(&sm_rt::read_smrt(&read(&p)).expect("read_smrt")).expect("load_image");
    let names: Vec<String> = loaded.regions().0.iter().map(|r| r.name.clone()).collect();
    assert_eq!(names, ["fuel", "water"]);
}

fn foam_req(path: &Path, format: ExportFormat, warp: bool) -> Request {
    match export_req(path, Some(format), None) {
        Request::Export { path, format, mesh, provenance, inline, options, split_small_determinant, empty_patches, wedge_patches, .. } => {
            Request::Export { path, format, mesh, provenance, inline, options, split_small_determinant, empty_patches, wedge_patches, split_warped_faces: warp }
        }
        other => other,
    }
}

fn foam_run(s: &mut Server, dir: &Path, name: &str, warp: bool) -> ExportResult {
    let v = s.call(foam_req(&dir.join(name), ExportFormat::Openfoam, warp)).unwrap_or_else(|e| panic!("{e} {}", e.data));
    serde_json::from_value(v).expect("ExportResult")
}

fn case_files(dir: &Path, name: &str) -> Vec<(String, Vec<u8>)> {
    let root = dir.join(name);
    let mut out = Vec::new();
    let mut stack = vec![root.clone()];
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(&d).expect("read_dir") {
            let p = e.expect("entry").path();
            if p.is_dir() {
                stack.push(p);
            } else {
                out.push((p.strip_prefix(&root).expect("prefix").to_string_lossy().into_owned(), read(&p)));
            }
        }
    }
    out.sort();
    out
}

/// The point count of a case: the first bare integer line of `constant/polyMesh/points`.
fn n_points(files: &[(String, Vec<u8>)]) -> u64 {
    let (_, b) = files.iter().find(|(n, _)| n.ends_with("polyMesh/points")).expect("points");
    String::from_utf8_lossy(b).lines().find_map(|l| l.trim().parse::<u64>().ok()).expect("count line")
}

fn warp_count(r: &ExportResult) -> Option<u64> {
    r.notes.iter().find(|n| n.code == "foam.warped_faces_split").map(|n| n.count)
}

#[test]
fn split_warped_faces_reaches_the_openfoam_exporter() {
    let dir = TempDir::new("wire-warp");
    let mut s = server(&[WARPED]);
    let on = foam_run(&mut s, dir.path(), "on", true);
    let n = warp_count(&on).unwrap_or_else(|| panic!("precondition: the oblique hexahedron has warped faces: {:?}", on.notes));
    assert!(n > 0);
    let off = foam_run(&mut s, dir.path(), "off", false);
    assert_eq!(warp_count(&off), None, "{:?}", off.notes);
    let (a, b) = (case_files(dir.path(), "on"), case_files(dir.path(), "off"));
    let faces = |v: &[(String, Vec<u8>)]| v.iter().find(|(n, _)| n.ends_with("polyMesh/faces")).map(|(_, b)| b.clone()).expect("faces");
    assert_ne!(faces(&a), faces(&b), "flag on changes the faces file");
    let owner = |v: &[(String, Vec<u8>)]| v.iter().find(|(n, _)| n.ends_with("polyMesh/owner")).map(|(_, b)| b.clone()).expect("owner");
    assert_ne!(owner(&a), owner(&b));
    assert_eq!(n_points(&a), n_points(&b) + n, "one apex point per split face");
    // Default (flag absent) equals an explicit false: byte-identical.
    let again = foam_run(&mut s, dir.path(), "off2", false);
    assert_eq!(case_files(dir.path(), "off2"), b);
    assert_eq!(again.sha256, off.sha256);
    // Determinism: two runs give the same bytes.
    let first = foam_run(&mut s, dir.path(), "on2", true);
    assert_eq!(case_files(dir.path(), "on2"), a);
    assert_eq!(first.sha256, on.sha256);
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    let single = one.install(|| foam_run(&mut s, dir.path(), "on3", true));
    assert_eq!(single.sha256, on.sha256, "1 thread");
    assert_eq!(case_files(dir.path(), "on3"), a);
}

#[test]
fn split_warped_faces_is_openfoam_only_and_planar_meshes_are_unchanged() {
    let dir = TempDir::new("wire-warp-bad");
    let mut s = hex_server();
    for (fmt, name) in [(ExportFormat::Gmsh, "a.msh"), (ExportFormat::Dagmc, "a.h5m"), (ExportFormat::Smrt, "a.smrt"), (ExportFormat::Vtu, "a.vtu")] {
        let e = s.call(foam_req(&dir.path().join(name), fmt, true)).expect_err("openfoam only");
        assert_eq!(e.kind, "bad-request", "{e}");
        assert!(e.message.contains("split_warped_faces"), "{e}");
    }
    let on = foam_run(&mut s, dir.path(), "on", true);
    let off = foam_run(&mut s, dir.path(), "off", false);
    assert_eq!(warp_count(&on), None, "{:?}", on.notes);
    assert_eq!(case_files(dir.path(), "on"), case_files(dir.path(), "off"));
    assert_eq!(on.sha256, off.sha256);
}
