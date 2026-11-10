//! W4-DAGMC-MODEL: `session.export` with `source: model` writes a DAGMC `.h5m` straight from
//! the B-rep (no volume mesh), through the protocol (in-process) and MCP.
//!
//! The geometry-level checks (closed volumes, analytic volumes, the chord-tolerance bound) are
//! unit tests of `model_export`. The structural comparison with MOAB's `facets_to_h5m` is the
//! gated `moab_structural_equality` below.
mod common;

use std::io::Cursor;
use std::path::Path;
use std::process::Command;

use base64::Engine as _;
use common::TempDir;
use serde_json::{Value, json};
use sm_protocol::{Endpoint, ExportFormat, ExportResult, ExportSource, Request, RpcError};
use sm_server::{Server, standard_with};

const TIME: &str = "2026-10-24T00:00:00Z";

/// A pin cell (cylinder in a square moderator) with blocks, and no mesh at all.
const PIN: &str = r#"
let br = model.create_brick(x: 1.26, y: 1.26, z: 1.0)
let fu = model.create_cylinder(height: 1.0, radius: 0.4)
let tl = model.create_cylinder(height: 1.0, radius: 0.4)
let mo = model.subtract(targets: br.body, tools: tl.body)
model.imprint(bodies: all bodies)
model.merge()
fem.block_add(block: 1, volumes: fu.volume, name: "fuel")
fem.block_add(block: 2, volumes: mo.volumes, name: "moderator")
"#;

fn server(script: &str) -> Server {
    let mut s = Server::new(standard_with(Some("tester".into()), Some(TIME.into()))).expect("server");
    s.call(Request::ScriptRun { text: script.into(), path: None, includes: Default::default() }).unwrap_or_else(|e| panic!("fixture script: {e} {}", e.data));
    s
}

fn req(path: Option<&Path>, format: Option<ExportFormat>, options: Option<Value>, source: ExportSource, inline: bool) -> Request {
    Request::Export {
        path: path.map(|p| p.to_string_lossy().into_owned()),
        format,
        mesh: None,
        provenance: false,
        inline,
        options,
        split_small_determinant: None,
        empty_patches: vec![],
        wedge_patches: vec![],
        split_warped_faces: false,
        source,
    }
}

fn export(s: &mut Server, r: Request) -> Result<ExportResult, RpcError> {
    Ok(serde_json::from_value(s.call(r)?).expect("ExportResult"))
}

fn inline_bytes(r: &ExportResult) -> Vec<u8> {
    base64::engine::general_purpose::STANDARD.decode(r.base64.as_deref().expect("inline data")).expect("b64")
}

fn mcp(server: &mut Server, args: Value) -> Value {
    let msgs = [
        json!({"jsonrpc":"2.0","id":0,"method":"initialize","params":{"protocolVersion":"2025-06-18"}}),
        json!({"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"session_export","arguments":args}}),
    ];
    let input: String = msgs.iter().map(|m| format!("{m}\n")).collect();
    let mut out = Vec::new();
    sm_mcp::run_stdio(server, Cursor::new(input.into_bytes()), &mut out).expect("run");
    let replies: Vec<Value> = String::from_utf8(out).expect("utf8").lines().map(|l| serde_json::from_str(l).expect("json")).collect();
    replies[1]["result"].clone()
}

#[test]
fn model_source_needs_no_mesh_and_is_the_same_through_every_door() {
    let dir = TempDir::new("dagmc-model");
    let mut s = server(PIN);
    let opts = json!({"chord_tol": 1e-3, "graveyard": true});
    let file = dir.path().join("pin.h5m");
    let (hash, nodes) = (s.workspace().state_hash(), s.workspace().timeline().len());

    let r = export(&mut s, req(Some(&file), None, Some(opts.clone()), ExportSource::Model, true)).expect("export");
    let bytes = inline_bytes(&r);
    assert_eq!(&bytes[..8], b"\x89HDF\r\n\x1a\n");
    // Per os-arch; INT pins it (this is the value for the pin cell at chord_tol 1e-3 + graveyard).
    eprintln!("pin h5m sha256 {}", r.sha256);
    assert_eq!(std::fs::read(&file).expect("file"), bytes, "the file is the inline bytes");
    assert!(r.notes.iter().all(|n| !n.is_warning()), "{:?}", r.notes);
    assert_eq!((s.workspace().state_hash(), s.workspace().timeline().len()), (hash, nodes), "an export is outside history");

    // MCP: same bytes.
    let m = mcp(&mut s, json!({"format": "dagmc", "source": "model", "inline": true, "options": opts}));
    assert_eq!(m["isError"], false, "{m}");
    let data = m["structuredContent"]["base64"].as_str().expect("base64");
    assert_eq!(base64::engine::general_purpose::STANDARD.decode(data).expect("b64"), bytes);

    // The source defaults to the mesh: a state with no mesh says so.
    let e = export(&mut s, req(None, Some(ExportFormat::Dagmc), None, ExportSource::Mesh, true)).expect_err("no mesh");
    assert_eq!(e.kind, "no-mesh");
}

#[test]
fn model_source_is_deterministic_across_runs_and_thread_counts() {
    let run = |threads: Option<usize>| {
        let mut s = server(PIN);
        let go = |s: &mut Server| inline_bytes(&export(s, req(None, Some(ExportFormat::Dagmc), Some(json!({"chord_tol": 1e-3})), ExportSource::Model, true)).expect("export"));
        match threads {
            Some(n) => rayon::ThreadPoolBuilder::new().num_threads(n).build().expect("pool").install(|| go(&mut s)),
            None => go(&mut s),
        }
    };
    let a = run(None);
    assert_eq!(a, run(None), "two runs");
    assert_eq!(a, run(Some(1)), "1 thread vs default");
}

#[test]
fn model_source_errors_are_typed() {
    let mut s = server(PIN);
    let mut call = |format, options, source, mesh: Option<&str>| {
        let mut r = req(None, Some(format), options, source, true);
        if let Request::Export { mesh: m, .. } = &mut r {
            *m = mesh.map(|x| x.parse().expect("id"));
        }
        export(&mut s, r).expect_err("error")
    };
    // Other formats keep the unsupported-source error.
    for f in [ExportFormat::Smsh, ExportFormat::Gmsh, ExportFormat::Smrt, ExportFormat::Vtu, ExportFormat::Exodus] {
        assert_eq!(call(f, None, ExportSource::Model, None).kind, "unsupported-source", "{f:?}");
    }
    // `mesh` does not apply; `chord_tol` is not a mesh option; bad values and keys are refused.
    assert_eq!(call(ExportFormat::Dagmc, None, ExportSource::Model, Some("Mesh#1")).kind, "bad-request");
    assert_eq!(call(ExportFormat::Dagmc, Some(json!({"chord_tol": 0.01})), ExportSource::Mesh, None).kind, "bad-request");
    assert_eq!(call(ExportFormat::Dagmc, Some(json!({"chord_tol": -1.0})), ExportSource::Model, None).kind, "bad-request");
    assert_eq!(call(ExportFormat::Dagmc, Some(json!({"binary": true})), ExportSource::Model, None).kind, "bad-request");
}

/// Gated (`SM_CONFORMANCE=1`): our `.h5m` and MOAB's `facets_to_h5m` of the same facets agree
/// structurally (triangle and volume counts, sense pairs, surfaces under MOAB's grouping rule,
/// signed volumes). Needs docker with the `smrt-bench-dagmc-moab` image and a Python with h5py
/// (`SM_H5PY_PYTHON`, default `python3`).
#[test]
fn moab_structural_equality() {
    if std::env::var_os("SM_CONFORMANCE").is_none_or(|v| v != "1") {
        eprintln!("moab_structural_equality: skipped (set SM_CONFORMANCE=1)");
        return;
    }
    let dir = TempDir::new("dagmc-moab");
    let mut s = server(PIN);
    let ours = dir.path().join("ours.h5m");
    export(&mut s, req(Some(&ours), None, Some(json!({"chord_tol": 1e-3})), ExportSource::Model, false)).expect("export");
    let py = std::env::var("SM_H5PY_PYTHON").unwrap_or_else(|_| "python3".into());
    let script = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/dagmc_vs_moab.py");
    let out = Command::new(py).arg("-I").arg("-B").arg(&script).arg(&ours).arg(dir.path()).output().expect("python");
    assert!(out.status.success(), "stdout: {}\nstderr: {}", String::from_utf8_lossy(&out.stdout), String::from_utf8_lossy(&out.stderr));
    eprintln!("{}", String::from_utf8_lossy(&out.stdout));
}
