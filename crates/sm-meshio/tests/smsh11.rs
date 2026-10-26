//! `.smsh` 1.1: v1.0 compatibility, typed element sets, `mesh.stats`, loss report.

mod common;

use common::py::{python, run_json, tmp};
use common::*;
use sm_meshio::{MeshioError, SmshFile, SmshWriter, element_type_shape};
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::*;
use sm_testkit::meshes::{dual_patch, mixed_transition};

const V10: &[u8] = include_bytes!("data/v10_hex_block.smsh");

fn with_cells_set(mut m: ReferenceMesh, name: &str, cells: std::ops::Range<usize>) -> ReferenceMesh {
    MeshEdit::add_set(&mut m, name, SetMembers::Cells(cells.map(CellIx::new).collect()), &mut SerialAlloc::new()).unwrap();
    m
}

#[test]
fn v10_golden_still_decodes_and_matches_its_mesh() {
    let f = SmshFile::parse(V10).unwrap();
    assert_eq!((f.header().version.major, f.header().version.minor), (1, 0));
    f.verify_checksums().unwrap();
    let d = f.decode().unwrap();
    assert!(d.element_types.is_empty());
    assert!(d.header.mesh.stats.is_none());
    // The golden was written from this mesh by the unmodified 1.0 writer.
    let m = hex_block([3, 2, 2], [0.5, 0.25, 1.0], &mut SerialAlloc::new()).unwrap();
    assert_matches(&m, &d);
    // The 1.1 writer gives the same arrays; only the header differs.
    let now = SmshFile::parse(&SmshWriter::new(&m).provenance("v10-golden").to_bytes().unwrap()).unwrap().decode().unwrap();
    assert_eq!(now.header.version.minor, 1);
    assert_eq!(now.x, d.x);
    assert_eq!(now.face_vertices, d.face_vertices);
    assert_eq!(now.cell_faces, d.cell_faces);
}

#[test]
fn element_type_names_and_shapes() {
    assert_eq!(element_type_shape("HEX8"), Some(CellShape::Hex));
    assert_eq!(element_type_shape("HEX27"), Some(CellShape::Hex));
    assert_eq!(element_type_shape("TET10"), Some(CellShape::Tet));
    assert_eq!(element_type_shape("WEDGE15"), Some(CellShape::Wedge));
    assert_eq!(element_type_shape("PYRAMID13"), Some(CellShape::Pyramid));
    assert_eq!(element_type_shape("QUAD8"), Some(CellShape::Quad));
    assert_eq!(element_type_shape("TRI6"), Some(CellShape::Tri));
    assert_eq!(element_type_shape("POLYHEDRON"), Some(CellShape::Polyhedron));
    assert_eq!(element_type_shape("POLYGON"), Some(CellShape::Polygon));
    assert_eq!(element_type_shape("NFACED"), None);
    assert_eq!(element_type_shape("NSIDED"), None);
    assert_eq!(element_type_shape("hex8"), None);
}

#[test]
fn hex8_on_a_1000_cell_block_round_trips() {
    let m = with_cells_set(hex_block([10, 10, 10], [0.1; 3], &mut SerialAlloc::new()).unwrap(), "block_1", 0..1000);
    let bytes = SmshWriter::new(&m).element_type("block_1", "HEX8").to_bytes().unwrap();
    let f = SmshFile::parse(&bytes).unwrap();
    f.verify_checksums().unwrap();
    let d = f.decode().unwrap();
    assert_eq!(d.element_types, vec![("block_1".to_string(), "HEX8".to_string())]);
    assert_eq!(d.header.element_types.len(), 1);
    assert_eq!((d.header.element_types[0].shape, d.header.version.minor), (CellShape::Hex.code(), 1));
    assert_eq!(d.header.mesh.stats.as_ref().unwrap().n_hex, 1000);
    assert_matches(&m, &d);
    // Without a record the header still carries the (empty) list.
    let plain = SmshFile::parse(&SmshWriter::new(&m).to_bytes().unwrap()).unwrap().decode().unwrap();
    assert!(plain.element_types.is_empty());
    let b = SmshWriter::new(&m).to_bytes().unwrap();
    let hl = u64::from_le_bytes(b[8..16].try_into().unwrap()) as usize;
    assert!(String::from_utf8_lossy(&b[16..16 + hl]).contains("\"element_types\":[]"));
}

#[test]
fn polyhedron_on_dual_patch_round_trips() {
    let m = with_cells_set(dual_patch().unwrap().mesh, "cells", 0..7);
    let d = SmshFile::parse(&SmshWriter::new(&m).element_type("cells", "POLYHEDRON").to_bytes().unwrap()).unwrap().decode().unwrap();
    assert_eq!(d.element_types, vec![("cells".to_string(), "POLYHEDRON".to_string())]);
    assert_eq!(d.header.element_types[0].shape, CellShape::Polyhedron.code());
    assert_matches(&m, &d);
}

fn err_of(w: SmshWriter) -> MeshioError {
    w.to_bytes().expect_err("the writer must refuse")
}

#[test]
fn element_type_violations_are_typed_errors() {
    let m = with_cells_set(hex_block([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap(), "b", 0..8);
    // Hexes declared as tets.
    assert!(matches!(err_of(SmshWriter::new(&m).element_type("b", "TET4")), MeshioError::Inconsistent(_)));
    // Unknown set.
    assert!(matches!(err_of(SmshWriter::new(&m).element_type("nope", "HEX8")), MeshioError::Inconsistent(_)));
    // Duplicate record for one set, same or different type.
    assert!(matches!(err_of(SmshWriter::new(&m).element_type("b", "HEX8").element_type("b", "HEX8")), MeshioError::Inconsistent(_)));
    // HO name without an HO record.
    assert!(matches!(err_of(SmshWriter::new(&m).element_type("b", "HEX27")), MeshioError::Inconsistent(_)));
    // Exodus spelling is not a .smsh name.
    assert!(matches!(err_of(SmshWriter::new(&m).element_type("b", "NFACED")), MeshioError::Unsupported(_)));
    assert!(matches!(err_of(SmshWriter::new(&m).element_type("b", "hex8")), MeshioError::Unsupported(_)));
    // POLYHEDRON on hexes: the shape must match the cell, not just "3D".
    assert!(matches!(err_of(SmshWriter::new(&m).element_type("b", "POLYHEDRON")), MeshioError::Inconsistent(_)));
}

#[test]
fn element_type_set_kind_and_overlap_checks() {
    let mut m = hex_block([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let mut a = SerialAlloc::new();
    MeshEdit::add_set(&mut m, "lo", SetMembers::Cells((0..5).map(CellIx::new).collect()), &mut a).unwrap();
    MeshEdit::add_set(&mut m, "hi", SetMembers::Cells((4..8).map(CellIx::new).collect()), &mut a).unwrap();
    MeshEdit::add_set(&mut m, "v", SetMembers::Vertices(vec![VertexIx(0)]), &mut a).unwrap();
    // Same type on an overlap is fine; different types on a shared cell are not.
    let ok = SmshWriter::new(&m).element_type("lo", "HEX8").element_type("hi", "HEX8").to_bytes().unwrap();
    SmshFile::parse(&ok).unwrap().decode().unwrap();
    assert!(matches!(err_of(SmshWriter::new(&m).element_type("lo", "HEX8").element_type("hi", "HEX20")), MeshioError::Inconsistent(_)));
    // Not a cell set.
    assert!(matches!(err_of(SmshWriter::new(&m).element_type("v", "HEX8")), MeshioError::Inconsistent(_)));
    // Records come out ascending by set id, whatever the call order.
    let d = SmshFile::parse(&SmshWriter::new(&m).element_type("hi", "HEX8").element_type("lo", "HEX8").to_bytes().unwrap()).unwrap().decode().unwrap();
    assert_eq!(d.element_types[0].0, "lo");
}

#[test]
fn high_order_names_need_the_right_node_count() {
    let m = with_cells_set(ho_hex(), "b", 0..1);
    let d = SmshFile::parse(&SmshWriter::new(&m).element_type("b", "HEX27").to_bytes().unwrap()).unwrap().decode().unwrap();
    assert_eq!(d.element_types[0].1, "HEX27");
    assert!(matches!(err_of(SmshWriter::new(&m).element_type("b", "HEX20")), MeshioError::Inconsistent(_)));
}

#[test]
fn stats_match_the_mesh() {
    let m = mixed_transition().unwrap().mesh;
    let d = SmshFile::parse(&SmshWriter::new(&m).to_bytes().unwrap()).unwrap().decode().unwrap();
    let s = d.header.mesh.stats.clone().unwrap();
    assert_eq!((s.n_hex, s.n_wedge, s.n_pyramid, s.n_tet, s.n_polyhedron), (1, 1, 1, 1, 0));
    assert_eq!((s.n_interior_faces, s.n_boundary_faces, s.n_nonconformal_faces, s.n_partition_cut_faces), (3, 14, 0, 0));
    assert_eq!((s.max_cell_faces, s.max_face_vertices), (6, 4));
    for (name, m) in corpus() {
        let s = SmshFile::parse(&SmshWriter::new(&m).to_bytes().unwrap()).unwrap().header().mesh.stats.clone().unwrap();
        let cells = |sh: CellShape| (0..m.cell_count()).filter(|c| m.cell_shape(CellIx::new(*c)) == sh).count() as u64;
        let faces = |k: FaceKind| (0..m.face_count()).filter(|f| m.face_kind(FaceIx::new(*f)) == k).count() as u64;
        assert_eq!(
            [s.n_tet, s.n_hex, s.n_wedge, s.n_pyramid, s.n_polyhedron, s.n_tri, s.n_quad, s.n_polygon],
            [CellShape::Tet, CellShape::Hex, CellShape::Wedge, CellShape::Pyramid, CellShape::Polyhedron, CellShape::Tri, CellShape::Quad, CellShape::Polygon].map(cells),
            "{name}"
        );
        assert_eq!(
            [s.n_interior_faces, s.n_boundary_faces, s.n_nonconformal_faces, s.n_partition_cut_faces],
            [FaceKind::Interior, FaceKind::Boundary, FaceKind::NonConformal, FaceKind::PartitionCut].map(faces),
            "{name}"
        );
        assert_eq!(s.max_cell_faces, (0..m.cell_count()).map(|c| m.cell_faces(CellIx::new(c)).len()).max().unwrap_or(0) as u64, "{name}");
        assert_eq!(s.max_face_vertices, (0..m.face_count()).map(|f| m.face_vertices(FaceIx::new(f)).len()).max().unwrap_or(0) as u64, "{name}");
    }
}

#[test]
fn python_reader_returns_the_same_element_types_and_stats() {
    let Some(py) = python() else {
        eprintln!("python3 not available: skipping");
        return;
    };
    let script = concat!(env!("CARGO_MANIFEST_DIR"), "/readers/smsh.py");
    let m = with_cells_set(dual_patch().unwrap().mesh, "cells", 0..7);
    let path = tmp("typed.smsh");
    SmshWriter::new(&m).element_type("cells", "POLYHEDRON").write_file(&path).unwrap();
    let j = run_json(&py, script, &path);
    assert_eq!(j["element_types"], serde_json::json!([["cells", "POLYHEDRON"]]));
    assert_eq!(j["version"], serde_json::json!({"major": 1, "minor": 1}));
    let s = &j["stats"];
    assert_eq!(s["n_polyhedron"], 7);
    assert_eq!(s["n_interior_faces"], 12);
    assert_eq!(s["n_boundary_faces"], 32);
    assert_eq!(s["max_cell_faces"], 8);
    assert_eq!(s["max_face_vertices"], 6);
    // A v1.0 file reads with no types and no stats.
    let old = tmp("v10.smsh");
    std::fs::write(&old, V10).unwrap();
    let j = run_json(&py, script, &old);
    assert_eq!(j["element_types"], serde_json::json!([]));
    assert!(j["stats"].is_null());
}

#[test]
fn smsh_loss_report_is_empty() {
    for (name, m) in corpus() {
        assert!(SmshWriter::new(&m).loss_report().is_lossless(), "{name}");
    }
}
