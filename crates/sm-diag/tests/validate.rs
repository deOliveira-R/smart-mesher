//! `mesh.validate`: clean meshes, broken meshes, ordering, JSON shape.
#![allow(missing_docs)]

mod common;
use common::*;
use serde_json::json;
use sm_core::{EntityKind, Point3};
use sm_diag::{Finding, GeometryStatus, Severity, ValidateOptions, build_report, validate_mesh};
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, FaceIx, MeshEntityKind, MeshRead, SerialAlloc, validate};
use sm_testkit::broken::Tampered;
use sm_testkit::meshes::all_mesh_fixtures;

fn opts() -> ValidateOptions {
    ValidateOptions::default()
}

#[test]
fn every_fixture_validates_clean() {
    for fx in all_mesh_fixtures().unwrap() {
        let r = validate_mesh(&fx.mesh, &opts(), None);
        assert!(r.valid && r.total == 0 && r.violations.is_empty(), "{}: {:?}", fx.name, r.violations);
        assert!(r.geometry_checked && r.geometry_skipped_reason.is_none(), "{}", fx.name);
        assert!(!r.truncated && r.by_code.is_empty() && r.by_owner.is_empty());
    }
}

#[test]
fn stub_brick_mesh_run_is_valid_through_the_query() {
    let mut r = Rig::new();
    r.apply(CreateBrick { size: [1.0, 2.0, 3.0] }).unwrap();
    r.json("mesh.size", json!({"targets": ["TopoVolume#1"], "size": 0.5})).unwrap();
    r.json("mesh.scheme", json!({"targets": ["TopoVolume#1"], "scheme": "sweep", "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}})).unwrap();
    r.json("mesh.run", json!({"targets": ["TopoVolume#1"]})).unwrap();
    let v = r.query("mesh.validate", json!({})).unwrap();
    assert_eq!(v["valid"], true, "{v}");
    assert_eq!(v["total"], 0);
    assert_eq!(v["geometry_checked"], true);
    assert_eq!(v["schema_version"], 1);
    assert_finite(&v, "validate");
}

fn bbox_of_cell(m: &dyn MeshRead, c: CellIx) -> ([f64; 3], [f64; 3]) {
    let co = m.coords();
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for sf in m.cell_faces(c) {
        for v in m.face_vertices(sf.face()) {
            let p = [co.x[v.get()], co.y[v.get()], co.z[v.get()]];
            for a in 0..3 {
                lo[a] = lo[a].min(p[a]);
                hi[a] = hi[a].max(p[a]);
            }
        }
    }
    (lo, hi)
}

fn inside(p: [f64; 3], b: ([f64; 3], [f64; 3])) -> bool {
    (0..3).all(|a| p[a] >= b.0[a] - 1e-12 && p[a] <= b.1[a] + 1e-12)
}

#[test]
fn swapped_left_right_is_reported_at_the_face() {
    let m = hex_block([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let f = (0..m.face_count()).find(|&f| m.face_right(FaceIx::new(f)).is_some()).unwrap();
    let t = Tampered::of(&m).swap_left_right(f);
    let r = validate_mesh(&t, &opts(), None);
    assert!(!r.valid);
    let v = r.violations.iter().find(|v| v.code == "face.left_right").unwrap_or_else(|| panic!("{:?}", r.violations));
    assert_eq!(v.entity.unwrap().index, f as u32);
    assert_eq!(v.entity.unwrap().id, Some(m.id(MeshEntityKind::Face, f as u32)));
    assert_eq!(v.owner, Some(m.classification(MeshEntityKind::Face, f as u32).entity));
    assert!(!r.geometry_checked);
    assert!(r.geometry_skipped_reason.as_deref().unwrap().contains("face.left_right"));
    // The location of a face violation is the face centroid, inside its cells' bounding boxes.
    let loc = v.location.expect("location");
    assert!(inside(loc, bbox_of_cell(&t, CellIx::new(0))) && inside(loc, bbox_of_cell(&t, CellIx::new(1))));
}

#[test]
fn reversed_loop_is_structural_and_skips_geometry() {
    let m = hex_block([2, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let t = Tampered::of(&m).reverse_loop(0);
    let r = validate_mesh(&t, &opts(), None);
    assert!(!r.valid && !r.geometry_checked);
    assert!(r.violations.iter().any(|v| v.entity.is_some_and(|e| e.index == 0) || v.code == "cell.closed" || v.code == "cell.side"), "{:?}", r.violations);
    assert!(r.by_code.keys().any(|c| c.starts_with("loop.") || c.starts_with("cell.") || c.starts_with("face.")));
    assert_eq!(r.by_code.values().sum::<usize>(), r.total);
    assert!(r.geometry_skipped_reason.is_some());
}

#[test]
fn vertex_moved_through_the_opposite_face_gives_an_inverted_cell() {
    let m = hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    // The single hex: move vertex 0 (the origin) through the opposite corner.
    let t = Tampered::of(&m).move_vertex(0, Point3::new(2.0, 2.0, 2.0));
    let r = validate_mesh(&t, &opts(), None);
    assert!(!r.valid && r.geometry_checked, "{:?}", r.violations);
    let v = r.violations.iter().find(|v| v.code == "quality.inverted").unwrap_or_else(|| panic!("{:?}", r.violations));
    assert_eq!(v.entity.unwrap().index, 0);
    assert_eq!(v.owner, Some(m.classification(MeshEntityKind::Cell, 0).entity));
    assert!(inside(v.location.unwrap(), bbox_of_cell(&t, CellIx::new(0))));
    assert_eq!(v.owner.unwrap().kind(), EntityKind::TopoVolume);
    assert!(v.near.iter().all(|n| n.kind() != EntityKind::TopoVolume), "near is of lower dimension: {:?}", v.near);
    assert!(!v.near.is_empty() && v.near.len() <= 8);
    assert!(v.near.windows(2).all(|w| w[0] < w[1]), "sorted, distinct");
    assert_eq!(r.by_code.values().sum::<usize>(), r.total);
    for v in &r.violations {
        if let Some(e) = v.entity {
            assert!(v.location.is_some() && e.id.is_some(), "{v:?}");
        }
    }
}

#[test]
fn declassified_vertex_is_reported_with_its_bogus_owner() {
    let m = hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let t = Tampered::of(&m).declassify(MeshEntityKind::Vertex, 3);
    let r = validate_mesh(&t, &opts(), None);
    assert!(!r.valid);
    let v = r.violations.iter().find(|v| v.code == "class.entity" && v.entity.is_some_and(|e| e.index == 3 && e.id == Some(m.id(MeshEntityKind::Vertex, 3)))).unwrap_or_else(|| panic!("{:?}", r.violations));
    assert_eq!(v.owner, Some(t.classification(MeshEntityKind::Vertex, 3).entity));
    assert_eq!(v.owner.unwrap().kind(), EntityKind::MeshVertex);
    assert!(v.near.is_empty(), "a non-model owner has no dimension to be below");
    assert_eq!(v.location, Some([m.point(3).x, m.point(3).y, m.point(3).z]));
    // Classification findings are not structural: geometry still ran.
    assert!(r.geometry_checked);
}

#[test]
fn order_is_stable_under_reversal_of_the_input_findings() {
    let m = hex_block([2, 2, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let t = Tampered::of(&m)
        .move_vertex(0, Point3::new(1.5, 1.5, 1.5))
        .declassify(MeshEntityKind::Vertex, 4)
        .declassify(MeshEntityKind::Face, 2)
        .swap_left_right(0);
    let mut raw: Vec<Finding> = validate(&t).into_iter().map(|v| Finding::new(v.code.as_str(), Severity::Error, v.entity, v.detail)).collect();
    raw.push(Finding::new("mesh.stale", Severity::Warning, None, "stale"));
    assert!(raw.len() >= 3, "{raw:?}");
    let forward = build_report(&t, raw.clone(), &opts(), GeometryStatus::Checked);
    raw.reverse();
    let backward = build_report(&t, raw.clone(), &opts(), GeometryStatus::Checked);
    assert_eq!(forward, backward);
    let sev: Vec<Severity> = forward.violations.iter().map(|v| v.severity).collect();
    assert!(sev.windows(2).all(|w| w[0] <= w[1]), "errors before warnings");
    // The limit cuts after sorting.
    let cut = build_report(&t, raw, &ValidateOptions { limit: 2, ..opts() }, GeometryStatus::Checked);
    assert_eq!(cut.violations, forward.violations[..2]);
    assert!(cut.truncated && cut.total == forward.total);
    assert!(forward.by_owner.windows(2).all(|w| w[0].count > w[1].count || (w[0].count == w[1].count && w[0].owner < w[1].owner)));
}

#[test]
fn class_retired_and_stale_are_reported() {
    let m = hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let vol = id(EntityKind::TopoVolume, 1);
    let live = |e: sm_core::EntityId| e != vol;
    let r = validate_mesh(&m, &opts(), Some(&live));
    let v = r.violations.iter().find(|v| v.code == "diag.class_retired").expect("class_retired");
    assert_eq!(v.owner, Some(vol));
    assert!(matches!(v.entity, Some(e) if e.id == Some(m.id(MeshEntityKind::Cell, 0))));
    assert!(!r.valid);

    // Through the query: mesh a brick, then delete its volume: the entry goes stale and its
    // classification entities are retired.
    let mut rig = Rig::new();
    rig.apply(CreateBrick { size: [1.0, 2.0, 3.0] }).unwrap();
    rig.json("mesh.size", json!({"targets": ["TopoVolume#1"], "size": 0.5})).unwrap();
    rig.json("mesh.scheme", json!({"targets": ["TopoVolume#1"], "scheme": "sweep", "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}})).unwrap();
    rig.json("mesh.run", json!({"targets": ["TopoVolume#1"]})).unwrap();
    let ok = rig.query("mesh.validate", json!({})).unwrap();
    assert_eq!(ok["valid"], true, "{ok}");
    rig.apply(KillVolume { volume: vol }).unwrap();
    let r = rig.query("mesh.validate", json!({})).unwrap();
    assert_eq!(r["valid"], false, "{r}");
    assert_eq!(r["by_code"]["mesh.stale"], 1, "{r}");
    assert!(r["by_code"]["diag.class_retired"].as_u64().unwrap() >= 1, "{r}");
    let stale = r["violations"].as_array().unwrap().iter().find(|v| v["code"] == "mesh.stale").unwrap();
    assert_eq!(stale["severity"], "warning");
    assert!(stale["entity"].is_null() && stale["owner"].is_null() && stale["location"].is_null());
    assert_finite(&r, "validate");
}

#[test]
fn query_picks_the_only_mesh_and_lists_candidates_otherwise() {
    let mut rig = Rig::new();
    let e = rig.query("mesh.validate", json!({})).unwrap_err();
    assert!(e.contains("no mesh"), "{e}");
    rig.apply(CreateBrick { size: [1.0; 3] }).unwrap();
    rig.apply(InsertPair { kind: "sheared".into() }).unwrap();
    assert!(rig.query("mesh.validate", json!({})).is_ok());
    rig.apply(InsertPair { kind: "stretched".into() }).unwrap();
    let e = rig.query("mesh.validate", json!({})).unwrap_err();
    assert!(e.contains("Mesh#1") && e.contains("Mesh#2"), "{e}");
    assert!(rig.query("mesh.validate", json!({"mesh": "Mesh#9"})).is_err());
    let e = rig.query("mesh.validate", json!({"mesh": "Mesh#1", "limit": 10001})).unwrap_err();
    assert!(e.contains("limit"), "{e}");
}
