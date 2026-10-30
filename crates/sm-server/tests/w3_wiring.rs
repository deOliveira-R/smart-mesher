//! W3 integration (D3, D6): the standard distribution carries every plugin and service, exports
//! Exodus and OpenFOAM, and stages STEP inputs content-addressed so a project replays from its
//! directory alone.
mod common;

use common::TempDir;
use serde_json::{Value, json};
use sm_op_api::Hash256;
use sm_protocol::{Endpoint, ExportFormat, ExportResult, Request, RpcError};
use sm_server::{Server, reference_standard, standard, standard_with};

const TIME: &str = "2026-10-24T00:00:00Z";

fn std_builder() -> sm_engine::WorkspaceBuilder {
    standard_with(Some("tester".into()), Some(TIME.into()))
}

fn call(s: &mut Server, r: Request) -> Value {
    s.call(r).unwrap_or_else(|e| panic!("request failed: {e}"))
}

fn err(s: &mut Server, r: Request) -> RpcError {
    s.call(r).expect_err("must fail")
}

fn apply(s: &mut Server, op: &str, params: Value) -> Value {
    call(s, Request::Apply { op: op.into(), params, version: None })
}

fn hash(s: &mut Server) -> Hash256 {
    s.workspace().state_hash()
}

/// A 3x3x3 map-meshed brick in the server.
fn meshed_brick(s: &mut Server) {
    let b = apply(s, "model.create_brick", json!({"x": 3.0, "y": 3.0, "z": 3.0}));
    let vol = b["delta"]["outputs"]["volume"][0].as_str().expect("volume").to_owned();
    apply(s, "mesh.size", json!({"targets": [vol], "size": 1.0}));
    apply(s, "mesh.scheme", json!({"targets": [vol], "scheme": "map"}));
    apply(s, "mesh.run", json!({"targets": [vol]}));
}

fn export(s: &mut Server, path: Option<&str>, format: Option<ExportFormat>, inline: bool) -> Result<ExportResult, RpcError> {
    let r = s.call(Request::Export { path: path.map(Into::into), format, mesh: None, provenance: false, inline })?;
    Ok(serde_json::from_value(r).expect("ExportResult"))
}

fn encode_b64(bytes: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    for c in bytes.chunks(3) {
        let n = (u32::from(c[0]) << 16) | (u32::from(*c.get(1).unwrap_or(&0)) << 8) | u32::from(*c.get(2).unwrap_or(&0));
        for i in 0..4 {
            if i <= c.len() {
                out.push(T[((n >> (18 - 6 * i)) & 63) as usize] as char);
            } else {
                out.push('=');
            }
        }
    }
    out
}

fn files_below(root: &std::path::Path) -> Vec<String> {
    fn walk(dir: &std::path::Path, root: &std::path::Path, out: &mut Vec<String>) {
        for e in std::fs::read_dir(dir).expect("read_dir") {
            let p = e.expect("entry").path();
            if p.is_dir() {
                walk(&p, root, out);
            } else {
                out.push(p.strip_prefix(root).expect("below").to_string_lossy().into_owned());
            }
        }
    }
    let mut v = Vec::new();
    walk(root, root, &mut v);
    v.sort();
    v
}

#[test]
fn op_names_service_lists_every_registered_op() {
    for b in [standard(), reference_standard()] {
        let ws = b.memory().expect("workspace");
        let names = ws.session().engine().services().get::<sm_diag::OpNames>().expect("OpNames service");
        let reg: Vec<String> = ws.registry().op_names().into_iter().map(str::to_owned).collect();
        assert_eq!(names.0.iter().cloned().collect::<Vec<_>>(), reg);
        for op in ["model.import_step", "model.imprint", "mesh.smooth", "mesh.to_poly", "mesh.flag_cells", "fem.block_add"] {
            assert!(reg.iter().any(|n| n == op), "{op} is not registered; have {reg:?}");
        }
        for q in ["rt.info", "rt.check", "rt.locate", "mesh.validate", "mesh.quality", "cad.step_info"] {
            assert!(ws.registry().query_names().contains(&q), "query {q} is not registered");
        }
    }
}

#[test]
fn exodus_exports_by_extension_and_inline_and_openfoam_writes_a_case_directory() {
    let dir = TempDir::new("w3exp");
    let mut s = Server::from_factory(std_builder).expect("server");
    meshed_brick(&mut s);

    for ext in ["exo", "e", "g", "exodus"] {
        let p = dir.path().join(format!("m.{ext}"));
        let r = export(&mut s, Some(p.to_str().expect("utf8")), None, false).expect(ext);
        assert_eq!(r.format, ExportFormat::Exodus, "{ext}");
        assert!(r.files.is_empty());
        assert_eq!(std::fs::read(&p).expect("file").len() as u64, r.bytes);
    }
    let inline = export(&mut s, None, Some(ExportFormat::Exodus), true).expect("inline exodus");
    assert!(inline.base64.is_some());
    // OpenFOAM is never inferred from a path.
    let e = err(&mut s, Request::Export { path: Some(dir.path().join("case").to_string_lossy().into_owned()), format: None, mesh: None, provenance: false, inline: false });
    assert_eq!(e.kind, "bad-request");
    // Multi-file formats cannot be inline.
    let e = err(&mut s, Request::Export { path: None, format: Some(ExportFormat::Openfoam), mesh: None, provenance: false, inline: true });
    assert_eq!(e.kind, "bad-request");
    assert!(e.message.contains("several files"), "{}", e.message);
    let e = err(&mut s, Request::Export { path: Some("x".into()), format: Some(ExportFormat::Openfoam), mesh: None, provenance: false, inline: true });
    assert_eq!(e.kind, "bad-request");

    // An explicit openfoam on a single-file extension is a mistake, not a case directory.
    for name in ["x.vtu", "case.exo"] {
        let p = dir.path().join(name);
        let e = err(&mut s, Request::Export { path: Some(p.to_string_lossy().into_owned()), format: Some(ExportFormat::Openfoam), mesh: None, provenance: false, inline: false });
        assert_eq!(e.kind, "bad-request", "{name}");
        assert!(!p.exists(), "{name} must not be created");
    }

    let case = dir.path().join("case");
    let case_s = case.to_str().expect("utf8");
    let r = export(&mut s, Some(case_s), Some(ExportFormat::Openfoam), false).expect("openfoam");
    assert_eq!(r.format, ExportFormat::Openfoam);
    assert_eq!(r.path.as_deref(), Some(case_s));
    assert!(r.base64.is_none());
    let names: Vec<&str> = r.files.iter().map(|f| f.name.as_str()).collect();
    for n in ["constant/polyMesh/points", "constant/polyMesh/faces", "constant/polyMesh/owner", "constant/polyMesh/neighbour", "constant/polyMesh/boundary"] {
        assert!(names.contains(&n), "{n} missing from {names:?}");
    }
    let mut sorted = names.clone();
    sorted.sort_unstable();
    assert_eq!(names, sorted, "files are sorted by name");
    // The files on disk are exactly the manifest (no temp leftovers) and match their hashes.
    assert_eq!(files_below(&case), sorted.iter().map(|s| (*s).to_owned()).collect::<Vec<_>>());
    let mut manifest = String::new();
    let mut total = 0;
    for f in &r.files {
        let bytes = std::fs::read(case.join(&f.name)).expect("file");
        assert_eq!(Hash256::of(&bytes), f.sha256, "{}", f.name);
        assert_eq!(bytes.len() as u64, f.bytes);
        total += f.bytes;
        manifest.push_str(&format!("{}\t{}\n", f.name, f.sha256.to_hex()));
    }
    assert_eq!(r.sha256, Hash256::of(manifest.as_bytes()));
    assert_eq!(r.bytes, total);
    // Re-exporting is bit-identical and leaves no temp files.
    let again = export(&mut s, Some(case_s), Some(ExportFormat::Openfoam), false).expect("again");
    assert_eq!(again, r);
    assert_eq!(files_below(&case).len(), names.len());
}

#[test]
fn a_missing_blob_is_a_typed_error_and_leaves_the_state_unchanged() {
    let mut s = Server::new(standard()).expect("server");
    let before = hash(&mut s);
    let e = err(&mut s, Request::Apply { op: "model.import_step".into(), params: json!({"source": {"sha256": "0".repeat(64)}}), version: None });
    assert_eq!(e.kind, "invalid-params", "{e}");
    assert!(e.message.contains("step.input_missing"), "{e}");
    assert_eq!(hash(&mut s), before, "a failed import changes nothing");
    assert!(s.workspace().timeline().is_empty());
}

#[test]
fn inline_and_sha256_sources_give_the_same_state_hash() {
    let mut a = Server::new(standard_with(Some("t".into()), Some(TIME.into()))).expect("a");
    let staged = a.stage_input(sm_testkit::w3a::STEP).expect("stage");
    assert_eq!(staged.bytes, sm_testkit::w3a::STEP.len() as u64);
    assert_eq!(staged.sha256, Hash256::of(sm_testkit::w3a::STEP).to_hex());
    apply(&mut a, "model.import_step", json!({"source": {"sha256": staged.sha256}}));

    let mut b = Server::new(standard_with(Some("t".into()), Some(TIME.into()))).expect("b");
    apply(&mut b, "model.import_step", json!({"source": {"inline": encode_b64(sm_testkit::w3a::STEP)}}));
    assert_eq!(hash(&mut a), hash(&mut b));
    // Staging is idempotent.
    assert_eq!(a.stage_input(sm_testkit::w3a::STEP).expect("again"), staged);
}

#[test]
fn session_stage_reads_the_file_and_the_project_directory_alone_replays_the_import() {
    let dir = TempDir::new("w3stage");
    let step = dir.path().join("part.step");
    std::fs::write(&step, sm_testkit::w3a::STEP).expect("write step");
    let proj = dir.path().join("proj");

    let mut s = Server::create(std_builder(), &proj, "proj").expect("create");
    let script = format!(
        "let s = session.stage(\"{}\")\nlet p = model.import_step(source: {{ sha256: s.sha256 }})\n",
        step.to_string_lossy().replace('\\', "\\\\")
    );
    call(&mut s, Request::ScriptRun { text: script, path: None, includes: Default::default() });
    let h = hash(&mut s);
    assert_ne!(h, Hash256::of(b""), "something was imported");
    call(&mut s, Request::Save);
    let blob = proj.join("inputs").join(format!("{}.stp", Hash256::of(sm_testkit::w3a::STEP).to_hex()));
    assert!(blob.is_file(), "the blob lives in <project>/inputs/");
    drop(s);

    // Copy the project elsewhere (the STEP file is deleted too) and replay it from the copy.
    let copy = dir.path().join("elsewhere").join("copy");
    copy_dir(&proj, &copy);
    std::fs::remove_file(&step).expect("rm step");
    let mut reopened = Server::from_factory(std_builder).expect("server");
    call(&mut reopened, Request::Open { path: copy.to_string_lossy().into_owned(), create: false });
    assert_eq!(hash(&mut reopened), h, "the copied project replays to the same state");

    // Direct open with a builder from `standard()` binds the store the same way.
    drop(reopened);
    let direct = Server::open(std_builder(), &copy).expect("open");
    assert_eq!(direct.workspace().state_hash(), h);
}

#[test]
fn stage_without_a_project_lands_in_memory() {
    let mut s = Server::new(reference_standard()).expect("server");
    let r = s.stage_input(b"ISO-10303-21; not a real file").expect("stage");
    assert_eq!(r.bytes, 29);
    // A missing file is a typed host error.
    let e = err(&mut s, Request::ScriptRun { text: "session.stage(\"/definitely/not/here.step\")\n".into(), path: None, includes: Default::default() });
    assert_eq!(e.kind, "script-error");
}

fn copy_dir(from: &std::path::Path, to: &std::path::Path) {
    std::fs::create_dir_all(to).expect("mkdir");
    for e in std::fs::read_dir(from).expect("read_dir") {
        let e = e.expect("entry");
        let dst = to.join(e.file_name());
        if e.path().is_dir() {
            copy_dir(&e.path(), &dst);
        } else {
            std::fs::copy(e.path(), dst).expect("copy");
        }
    }
}

/// Stage the STEP file, import it, save, then replay the project from a copy.
fn stage_import_save_replay(mut s: Server, proj: &std::path::Path, copy: &std::path::Path) {
    let staged = s.stage_input(sm_testkit::w3a::STEP).expect("stage");
    apply(&mut s, "model.import_step", json!({"source": {"sha256": staged.sha256}}));
    let h = hash(&mut s);
    call(&mut s, Request::Save);
    let blob = proj.join("inputs").join(format!("{}.stp", staged.sha256));
    assert!(blob.is_file(), "the blob lives in <project>/inputs/");
    drop(s);
    copy_dir(proj, copy);
    let mut reopened = Server::from_factory(std_builder).expect("server");
    call(&mut reopened, Request::Open { path: copy.to_string_lossy().into_owned(), create: false });
    assert_eq!(hash(&mut reopened), h, "the project directory alone replays");
}

#[test]
fn two_builders_before_create_still_stage_into_the_project() {
    let dir = TempDir::new("w3two");
    let proj = dir.path().join("proj");
    let b = std_builder();
    let _other = standard();
    let s = Server::create(b, &proj, "proj").expect("create");
    stage_import_save_replay(s, &proj, &dir.path().join("copy"));
}

#[test]
fn a_stale_pending_store_from_an_in_memory_server_is_not_bound_to_a_project() {
    let dir = TempDir::new("w3stale");
    let proj = dir.path().join("proj");
    let mem = Server::new(standard()).expect("mem");
    // A builder from elsewhere (no distribution store) must not drag the memory server's store along.
    let s = Server::create(sm_engine::WorkspaceBuilder::new(), &proj, "proj").expect("create");
    drop(s);
    let staged = mem.stage_input(b"hello").expect("stage");
    assert_eq!(staged.bytes, 5);
    let inputs = proj.join("inputs");
    let leaked = inputs.is_dir() && std::fs::read_dir(&inputs).expect("dir").next().is_some();
    assert!(!leaked, "the in-memory server wrote into a project it is not attached to");
}
