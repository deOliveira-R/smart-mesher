//! Rust-side acceptance of the bindings core: `Host` equals `InProcess`, tables equal the mesh and
//! the `.smsh` writer's arrays, typed errors, determinism, and the unsafe-source scan.

use proptest::prelude::*;
use serde_json::{Value, json};
use sm_core::EntityId;
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block_with_edges;
use sm_mesh_api::{MeshEntityKind, MeshRead};
use sm_meshio::SmshWriter;
use sm_protocol::{METHODS, Hash256};
use _native::core::{Column, Host, HostError, MeshTables, tables_of};
use sm_server::{InProcess, Server};
use sm_testkit::strategies::{config, hex_block_spec};

const BRICK: &str = "let b = model.create_brick(x: 4.0, y: 3.0, z: 2.0)\nmesh.size(targets: b.volume, size: 1.0)\nmesh.scheme(targets: b.volume, scheme: \"map\")\nmesh.run(targets: b.volume)\n";

fn host() -> Host {
    Host::memory().expect("host")
}

fn run(h: &Host, text: &str) -> Value {
    h.call("script.run", &json!({ "text": text })).expect("script")
}

// ---------------------------------------------------------------- acceptance 9: Host::call == InProcess

/// One valid-ish and one failing params object per method, so replies and errors both compare.
fn params_for(method: &str) -> Vec<Value> {
    match method {
        "apply" => vec![json!({"op": "model.create_brick", "params": {"x": 2.0}}), json!({"op": "no.such_op", "params": {}}), json!({"op": "model.create_brick", "params": {"x": "a"}})],
        "query" => vec![json!({"name": "model.list", "input": {}}), json!({"name": "no.such_query"})],
        "script.run" => vec![json!({"text": "let b = model.create_brick(x: 2.0)\n"}), json!({"text": "model.nothing()\n"})],
        "script.record" => vec![json!({}), json!({"verify": true})],
        "timeline.log" => vec![json!({"limit": 5}), json!({})],
        "timeline.checkout" => vec![json!({"rev": "empty"}), json!({"rev": {"ref": "nope"}})],
        "timeline.branch" => vec![json!({"name": "alt"}), json!({"name": ""})],
        "session.hash" => vec![json!({}), json!({"at": "empty"})],
        "session.export" => vec![json!({"inline": true, "format": "smsh"}), json!({"format": "vtu"})],
        "session.open" => vec![json!({"path": "/nonexistent-smartmesh-dir/x"})],
        _ => vec![json!({})],
    }
}

fn parts(text: &str) -> (Option<Value>, Option<Value>) {
    let v: Value = serde_json::from_str(text).expect("reply json");
    (v.get("result").cloned(), v.get("error").cloned())
}

#[test]
fn host_call_equals_in_process_for_every_method() {
    let h = host();
    let mut ip = InProcess::new(Server::from_factory(sm_server::standard).expect("server"));
    for &method in METHODS {
        for params in params_for(method) {
            let text = json!({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).to_string();
            let want = parts(&ip.send_text(&text));
            let got = match h.call(method, &params) {
                Ok(v) => (Some(v), None),
                Err(e) => (None, Some(serde_json::to_value(&e).expect("err json"))),
            };
            // Node hashes and timestamps depend on the wall clock; compare what does not.
            assert_eq!(strip(&got), strip(&want), "{method} {params}");
        }
    }
}

/// Drop wall-clock fields (`time`) and node hashes derived from them.
fn strip(p: &(Option<Value>, Option<Value>)) -> Value {
    fn walk(v: &mut Value) {
        match v {
            Value::Object(m) => {
                for k in ["time", "node", "position", "parent", "branches", "message_time"] {
                    m.remove(k);
                }
                m.values_mut().for_each(walk);
            }
            Value::Array(a) => a.iter_mut().for_each(walk),
            _ => {}
        }
    }
    let mut v = json!({"result": p.0, "error": p.1});
    walk(&mut v);
    v
}

#[test]
fn every_protocol_method_is_exercised() {
    for &m in METHODS {
        assert!(!params_for(m).is_empty());
    }
}

// ---------------------------------------------------------------- mesh tables

fn smsh_arrays(bytes: &[u8]) -> Vec<(String, String, Vec<u8>)> {
    let hl = u64::from_le_bytes(bytes[8..16].try_into().expect("len")) as usize;
    let header: Value = serde_json::from_slice(&bytes[16..16 + hl]).expect("header");
    let start = 16 + hl;
    header["arrays"]
        .as_array()
        .expect("arrays")
        .iter()
        .map(|a| {
            let off = start + a["offset"].as_u64().expect("offset") as usize;
            let n = a["nbytes"].as_u64().expect("nbytes") as usize;
            (a["name"].as_str().expect("name").to_owned(), a["dtype"].as_str().expect("dtype").to_owned(), bytes[off..off + n].to_vec())
        })
        .collect()
}

fn le_bytes(c: &Column) -> Vec<u8> {
    match c {
        Column::U8(v) => v.clone(),
        Column::U32(v) => v.iter().flat_map(|x| x.to_le_bytes()).collect(),
        Column::U64(v) => v.iter().flat_map(|x| x.to_le_bytes()).collect(),
        Column::F64(v) => v.iter().flat_map(|x| x.to_le_bytes()).collect(),
        _ => unreachable!("column kind"),
    }
}

/// Every table equals the same-named `.smsh` array byte for byte and dtype for dtype.
fn assert_matches_file(m: &dyn MeshRead) {
    let t = tables_of(m);
    let file = smsh_arrays(&SmshWriter::new(m).to_bytes().expect("smsh"));
    for (name, col) in &t.columns {
        let (_, dtype, bytes) = file.iter().find(|(n, _, _)| n == name).unwrap_or_else(|| panic!("file lacks {name}"));
        assert_eq!(col.dtype(), dtype, "{name} dtype");
        assert_eq!(&le_bytes(col), bytes, "{name} bytes");
    }
}

fn assert_matches_mesh(m: &dyn MeshRead) {
    let t = tables_of(m);
    let (nv, nf, nc) = (m.count(MeshEntityKind::Vertex), m.count(MeshEntityKind::Face), m.count(MeshEntityKind::Cell));
    let len = |n: &str| t.get(n).unwrap_or_else(|| panic!("table {n}")).len();
    assert_eq!((len("x"), len("vertex_ids"), len("vertex_class")), (m.coords().len(), nv, nv));
    assert_eq!((len("face_ids"), len("face_left"), len("face_right"), len("face_kind"), len("face_class")), (nf, nf, nf, nf, nf));
    assert_eq!((len("cell_ids"), len("cell_shape"), len("cell_parent"), len("cell_active"), len("cell_class")), (nc, nc, nc, nc, nc));
    assert_eq!((len("face_offsets"), len("cell_face_offsets"), len("cell_corner_offsets")), (nf + 1, nc + 1, nc + 1));
    let cols = m.columns();
    assert_eq!(t.get("cell_faces").map(Column::len), Some(cols.cell_faces.len()));
    let Some(Column::F64(x)) = t.get("x") else { panic!("x column") };
    assert_eq!(x.as_slice(), m.coords().x);
    let Some(Column::U32(fr)) = t.get("face_right") else { panic!("face_right") };
    assert_eq!(fr.as_slice(), cols.face_right);
    let Some(Column::U64(ids)) = t.get("cell_ids") else { panic!("cell_ids") };
    assert!(ids.windows(2).all(|w| w[0] < w[1]), "ids ascend");
    assert_matches_file(m);
}

proptest! {
    #![proptest_config(config(24))]
    #[test]
    fn tables_equal_the_mesh_over_lattice_sizes(spec in hex_block_spec()) {
        let m = spec.build().expect("mesh");
        assert_matches_mesh(&m);
    }
}

#[test]
fn tables_equal_the_mesh_for_every_fixture_and_with_an_edge_table() {
    for f in sm_testkit::meshes::all_mesh_fixtures().expect("fixtures") {
        assert_matches_mesh(&f.mesh);
    }
    let m = hex_block_with_edges([3, 2, 2], [1.0, 1.0, 1.0], &mut SerialAlloc::new()).expect("edges mesh");
    assert!(m.edges().is_some());
    assert_matches_mesh(&m);
}

// ---------------------------------------------------------------- errors

#[test]
fn no_mesh_ambiguous_mesh_and_unknown_mesh() {
    let h = host();
    let e = h.mesh_tables(None).expect_err("no mesh");
    assert!(matches!(e, HostError::NoMesh), "{e}");
    run(&h, BRICK);
    let id = h.mesh_ids().expect("ids")[0];
    assert!(h.mesh_tables(None).is_ok());
    assert!(h.mesh_tables(Some(id)).is_ok());
    let other: EntityId = "Mesh#99".parse().expect("id");
    assert!(matches!(h.mesh_tables(Some(other)), Err(HostError::UnknownMesh(_))));
    run(&h, "let c = model.create_brick(x: 1.0, center: [9.0, 9.0, 9.0])\nmesh.size(targets: c.volume, size: 1.0)\nmesh.scheme(targets: c.volume, scheme: \"map\")\nmesh.run(targets: c.volume)\n");
    let e = h.mesh_tables(None).expect_err("ambiguous");
    let HostError::AmbiguousMesh { count, meshes } = &e else { panic!("{e}") };
    assert_eq!((*count, meshes.len()), (2, 2));
    // Same words and code as the export error.
    let exp = h.call("session.export", &json!({"inline": true, "format": "smsh"})).expect_err("export");
    assert_eq!(exp.kind, e.to_rpc().kind);
    assert_eq!(exp.message, e.to_string());
}

#[test]
fn a_poisoned_lock_is_an_internal_error_not_a_panic() {
    let h = host();
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| h.with_server(|_| -> () { panic!("boom") })));
    assert!(r.is_err());
    let e = h.call("timeline.status", &json!({})).expect_err("poisoned");
    assert_eq!(e.code, sm_protocol::INTERNAL_ERROR);
    assert!(matches!(h.stage(b"x"), Err(HostError::Poisoned)));
}

// ---------------------------------------------------------------- determinism

fn tables_hash(t: &MeshTables) -> Hash256 {
    let mut all = Vec::new();
    for (n, c) in &t.columns {
        all.extend_from_slice(n.as_bytes());
        all.extend_from_slice(c.bytes());
    }
    Hash256::of(&all)
}

fn slice_hash(_threads: usize) -> (String, Hash256) {
    let pool = std::thread::Builder::new().spawn(move || {
        let h = host();
        run(&h, BRICK);
        let state = h.call("session.hash", &json!({})).expect("hash")["state"].as_str().expect("state").to_owned();
        (state, tables_hash(&h.mesh_tables(None).expect("tables")))
    });
    pool.expect("spawn").join().expect("join")
}

#[test]
fn determinism_state_hash_and_table_bytes_run_twice_and_across_threads() {
    // `check` runs f(1) and f(N) repeatedly and concurrently; every result must be identical.
    let r = sm_testkit::determinism::check(2, 4, slice_hash).expect("deterministic");
    assert_eq!(r, slice_hash(1));
    let fixture = sm_testkit::meshes::hex_block_2x2x2().expect("fixture");
    sm_testkit::determinism::check(2, 4, |_| tables_hash(&tables_of(&fixture.mesh))).expect("tables deterministic");
}

// ---------------------------------------------------------------- source scan

#[test]
fn no_unsafe_block_fn_or_impl_in_the_crate_source() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let (mut deny, mut allow) = (0, 0);
    for entry in std::fs::read_dir(dir).expect("src") {
        let path = entry.expect("entry").path();
        let text = std::fs::read_to_string(&path).expect("read");
        // Tokens, not substrings: comments may say "unsafe".
        let code: String = text.lines().filter(|l| !l.trim_start().starts_with("//")).collect::<Vec<_>>().join("\n");
        deny += code.matches("#![deny(unsafe_code)]").count();
        allow += code.matches("#[allow(unsafe_code)]").count();
        let toks: Vec<&str> = code.split(|c: char| !(c.is_alphanumeric() || c == '_')).filter(|t| !t.is_empty()).collect();
        for (i, t) in toks.iter().enumerate() {
            if *t == "unsafe" {
                let next = toks.get(i + 1).copied().unwrap_or("");
                // `#[allow(unsafe_code)]` tokenizes as `unsafe_code`, never as `unsafe`.
                panic!("`unsafe` token in {} before `{next}`", path.display());
            }
        }
    }
    assert_eq!(deny, 1, "one #![deny(unsafe_code)]");
    assert!(allow <= 1, "at most one #[allow(unsafe_code)]");
}
