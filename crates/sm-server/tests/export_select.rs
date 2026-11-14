//! W4-EXPORT-SELECT: the `select` option of `session.export` for smsh, VTU, Exodus, Gmsh and
//! DAGMC (one validation for both dispatch paths), its composition with the `regions` split
//! (ruling D9), its refusals and the absent-option byte identity. Everything is exported inline.
mod common;

use std::collections::BTreeSet;

use base64::Engine as _;
use serde_json::{Value, json};
use sm_meshio::gmsh::parse_msh4;
use sm_meshio::{SmshFile, SmshMesh};
use sm_protocol::{Endpoint, ExportFormat, ExportResult, ExportSource, NoteSeverity, Request, RpcError};
use sm_server::{Server, standard_with};

const TIME: &str = "2026-10-24T00:00:00Z";

/// Three 2 x 2 x 2 bricks in a row, one block each, a sideset `head` on the faces of the first and
/// `tail` on those of the last (the sides of that volume), meshed with 1 x 1 x 1 hexes: 8 cells
/// per brick, a 6 x 2 x 2 lattice of cells.
const THREE: &str = r#"
let a = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [1.0, 1.0, 1.0])
let b = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [3.0, 1.0, 1.0])
let c = model.create_brick(x: 2.0, y: 2.0, z: 2.0, center: [5.0, 1.0, 1.0])
model.imprint(bodies: a.body | b.body | c.body)
model.merge(entities: a.body | b.body | c.body)
fem.block_add(block: 1, volumes: a.volume, name: "fuel")
fem.block_add(block: 2, volumes: b.volume, name: "mid")
fem.block_add(block: 3, volumes: c.volume, name: "water")
fem.sideset_add(sideset: 1, faces: topo.related(kind: "TopoFace", within: a.volume), wrt: a.volume, name: "head")
fem.sideset_add(sideset: 2, faces: topo.related(kind: "TopoFace", within: c.volume), wrt: c.volume, name: "tail")
mesh.size(targets: a.volume | b.volume | c.volume, size: 1.0)
mesh.scheme(targets: a.volume | b.volume | c.volume, scheme: "map")
let m = mesh.run(targets: a.volume | b.volume | c.volume)
"#;

fn server(script: &str) -> Server {
    let mut s = Server::new(standard_with(Some("tester".into()), Some(TIME.into()))).expect("server");
    s.call(Request::ScriptRun { text: script.into(), path: None, includes: Default::default() })
        .unwrap_or_else(|e| panic!("fixture script: {e} {}", e.data));
    s
}

fn call(s: &mut Server, format: ExportFormat, options: Option<Value>, source: ExportSource) -> Result<ExportResult, RpcError> {
    let r = Request::Export {
        path: None,
        format: Some(format),
        mesh: None,
        provenance: false,
        inline: true,
        options,
        split_small_determinant: None,
        empty_patches: vec![],
        wedge_patches: vec![],
        split_warped_faces: false,
        source,
    };
    Ok(serde_json::from_value(s.call(r)?).expect("ExportResult"))
}

fn export_full(s: &mut Server, format: ExportFormat, options: Option<Value>) -> Result<ExportResult, RpcError> {
    call(s, format, options, ExportSource::Mesh)
}

fn export(s: &mut Server, format: ExportFormat, options: Option<Value>) -> Result<Vec<u8>, RpcError> {
    export_full(s, format, options).map(|r| bytes_of(&r))
}

fn bytes_of(r: &ExportResult) -> Vec<u8> {
    base64::engine::general_purpose::STANDARD.decode(r.base64.as_deref().expect("inline data")).expect("b64")
}

fn smsh_of(b: &[u8]) -> SmshMesh {
    SmshFile::parse(b).expect("parse").decode().expect("decode")
}

fn two_blocks() -> Option<Value> {
    Some(json!({"select": {"blocks": ["fuel", "mid"]}}))
}

#[test]
fn two_blocks_of_three_in_every_format() {
    let mut s = server(THREE);
    let all = smsh_of(&export(&mut s, ExportFormat::Smsh, None).expect("smsh"));
    assert_eq!((all.cell_ids.len(), all.vertex_ids.len()), (24, 7 * 3 * 3));

    // smsh: the cells of the two blocks and exactly their nodes; the sideset of the dropped brick
    // is gone, `head` stays (the 2 x 2 faces of the end brick's six faces: 6 x 4 = 24 sides).
    let d = smsh_of(&export(&mut s, ExportFormat::Smsh, two_blocks()).expect("smsh"));
    assert_eq!((d.cell_ids.len(), d.vertex_ids.len()), (16, 5 * 3 * 3));
    let names: Vec<&str> = d.sets.iter().map(|s| s.name.as_str()).collect();
    assert_eq!(names, ["fuel", "mid", "head"]);
    assert_eq!(d.sets[2].members.len(), 24);

    // VTU.
    let vtu = export(&mut s, ExportFormat::Vtu, two_blocks()).expect("vtu");
    let text = String::from_utf8_lossy(&vtu);
    assert!(text.contains("NumberOfPoints=\"45\" NumberOfCells=\"16\""), "{}", &text[..300.min(text.len())]);

    // Gmsh: 45 nodes, 16 hexahedra (type 5).
    let g = parse_msh4(&export(&mut s, ExportFormat::Gmsh, two_blocks()).expect("gmsh")).expect("parse msh4");
    assert_eq!(g.node_count(), 45);
    assert_eq!(g.elements_by_type().get(&5), Some(&16));

    // Exodus: 16 elements, 45 nodes, two element blocks.
    let x = sm_exodus::read_exodus(&export(&mut s, ExportFormat::Exodus, two_blocks()).expect("exodus")).expect("read");
    assert_eq!((x.num_elem, x.num_nodes, x.blocks.len()), (16, 45, 2));
    let xmax = x.coords[0].iter().cloned().fold(f64::MIN, f64::max);
    assert!((xmax - 4.0).abs() < 1e-12, "the kept nodes span x in [0, 4]: {xmax}");

    // DAGMC: the selection is a smaller mesh, so a smaller file; the dropped brick's volume is not in it.
    let full = export(&mut s, ExportFormat::Dagmc, None).expect("dagmc");
    let cut = export(&mut s, ExportFormat::Dagmc, two_blocks()).expect("dagmc select");
    assert_eq!(&cut[..8], b"\x89HDF\r\n\x1a\n");
    assert!(cut.len() < full.len(), "{} vs {}", cut.len(), full.len());
}

#[test]
fn absent_options_and_a_keep_everything_select_are_the_plain_bytes() {
    let mut s = server(THREE);
    for format in [ExportFormat::Smsh, ExportFormat::Vtu, ExportFormat::Exodus, ExportFormat::Gmsh, ExportFormat::Dagmc] {
        let plain = export(&mut s, format, None).expect("plain");
        for o in [
            json!({"select": null}),
            json!({"select": {}}),
            json!({"select": {"blocks": "all", "sidesets": "all", "nodesets": "all", "dimension": "auto"}}),
            json!({"select": {"dimension": 3}}),
        ] {
            assert_eq!(export(&mut s, format, Some(o.clone())).expect("select"), plain, "{format:?} {o}");
        }
    }
}

#[test]
fn a_named_empty_sideset_is_a_warning_and_an_unknown_name_lists_the_choices() {
    let mut s = server(THREE);
    let o = Some(json!({"select": {"blocks": ["fuel", "mid"], "sidesets": ["head", "tail"]}}));
    for format in [ExportFormat::Smsh, ExportFormat::Vtu, ExportFormat::Exodus, ExportFormat::Gmsh, ExportFormat::Dagmc] {
        let r = export_full(&mut s, format, o.clone()).expect("select");
        let n = r.notes.iter().find(|n| n.code == "export.empty_selection").unwrap_or_else(|| panic!("{format:?}: {:?}", r.notes));
        assert_eq!(n.severity, NoteSeverity::Warning);
        assert!(n.detail.contains("tail"), "{}", n.detail);
    }
    for format in [ExportFormat::Smsh, ExportFormat::Vtu, ExportFormat::Exodus, ExportFormat::Gmsh, ExportFormat::Dagmc] {
        let e = export(&mut s, format, Some(json!({"select": {"blocks": ["gone"]}}))).expect_err("unknown block");
        assert_eq!(e.kind, "bad-request", "{format:?}");
        let msg = e.to_string();
        assert!(msg.contains("gone") && msg.contains("fuel, mid, water"), "{msg}");
        let e = export(&mut s, format, Some(json!({"select": {"sidesets": ["fuel"]}}))).expect_err("a block is not a sideset");
        assert!(e.to_string().contains("head, tail"), "{e}");
    }
}

#[test]
fn the_shape_of_select_is_checked_before_anything_else() {
    let mut s = server(THREE);
    let kind = |r: Result<Vec<u8>, RpcError>| r.expect_err("an error").kind;
    for o in [
        json!({"select": 3}),
        json!({"select": {"blocks": 3}}),
        json!({"select": {"blocks": [1]}}),
        json!({"select": {"bogus": []}}),
        json!({"select": {"dimension": 4}}),
        json!({"select": {"overwrite": "yes"}}),
        json!({"select": {"dimension": 2}}),
    ] {
        for format in [ExportFormat::Smsh, ExportFormat::Vtu, ExportFormat::Exodus, ExportFormat::Gmsh, ExportFormat::Dagmc] {
            assert_eq!(kind(export(&mut s, format, Some(o.clone()))), "bad-request", "{format:?} {o}");
        }
    }
    // OpenFOAM keeps its patches and zones; the tracking image has no sets to select.
    assert_eq!(kind(export(&mut s, ExportFormat::Openfoam, two_blocks())), "bad-request");
    assert_eq!(kind(export(&mut s, ExportFormat::Smrt, two_blocks())), "bad-request");
    // `source: model` has no volume mesh to select from.
    let e = call(&mut s, ExportFormat::Dagmc, two_blocks(), ExportSource::Model).expect_err("model");
    assert_eq!(e.kind, "bad-request");
    // An empty selection is not an export.
    assert_eq!(kind(export(&mut s, ExportFormat::Smsh, Some(json!({"select": {"blocks": []}})))), "bad-request");
}

#[test]
fn overwrite_false_refuses_an_existing_file_and_true_replaces_it() {
    let dir = common::TempDir::new("select-overwrite");
    let path = dir.path().join("a.vtu");
    let mut s = server(THREE);
    let req = |options: Option<Value>| Request::Export {
        path: Some(path.to_string_lossy().into_owned()),
        format: Some(ExportFormat::Vtu),
        mesh: None,
        provenance: false,
        inline: false,
        options,
        split_small_determinant: None,
        empty_patches: vec![],
        wedge_patches: vec![],
        split_warped_faces: false,
        source: ExportSource::Mesh,
    };
    s.call(req(Some(json!({"select": {"overwrite": false}})))).expect("first write: nothing is there");
    let first = std::fs::read(&path).expect("file");
    let e = s.call(req(Some(json!({"select": {"overwrite": false, "blocks": ["fuel"]}})))).expect_err("exists");
    assert_eq!(e.kind, "file-exists");
    assert_eq!(std::fs::read(&path).expect("file"), first, "the file is untouched");
    s.call(req(Some(json!({"select": {"overwrite": true, "blocks": ["fuel"]}})))).expect("replace");
    assert_ne!(std::fs::read(&path).expect("file"), first);
}

#[test]
fn the_selection_composes_with_the_region_split_and_keeps_the_region_names() {
    let mut s = server(THREE);
    let split = json!({"split": ["lattice", "ring", "sector"]});
    let mut regions = |o: Value| -> Vec<(String, usize)> {
        let d = smsh_of(&export(&mut s, ExportFormat::Smsh, Some(o)).expect("smsh"));
        d.sets.iter().filter(|x| x.name.starts_with("region/")).map(|x| (x.name.clone(), x.members.len())).collect()
    };
    let full = regions(json!({"regions": split}));
    assert_eq!(full.iter().map(|r| r.0.as_str()).collect::<Vec<_>>(), ["region/fuel", "region/mid", "region/water"]);
    let picked = regions(json!({"regions": split, "select": {"blocks": ["fuel", "water"]}}));
    // Only the regions of the selected blocks, whole and under the same names.
    assert_eq!(picked.iter().map(|r| r.0.as_str()).collect::<Vec<_>>(), ["region/fuel", "region/water"]);
    for (n, len) in &picked {
        assert_eq!(full.iter().find(|f| &f.0 == n).map(|f| f.1), Some(*len), "{n}");
        assert_eq!(*len, 8);
    }
    let names: BTreeSet<String> = picked.into_iter().map(|r| r.0).collect();
    assert!(!names.contains("region/mid"));
}

#[test]
fn selection_is_deterministic_across_runs_and_thread_counts() {
    let go = || {
        let mut s = server(THREE);
        [ExportFormat::Smsh, ExportFormat::Vtu, ExportFormat::Exodus, ExportFormat::Gmsh, ExportFormat::Dagmc]
            .map(|f| export(&mut s, f, two_blocks()).expect("export"))
    };
    let a = go();
    assert_eq!(a, go());
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool").install(go);
    assert_eq!(a, one);
}
