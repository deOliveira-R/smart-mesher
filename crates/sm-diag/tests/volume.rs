//! `mesh.volume_check`: per-region mesh volume against the model, analytic checks.
#![allow(missing_docs)]

mod common;
use std::f64::consts::PI;

use common::*;
use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind};
use sm_diag::volume_report;
use sm_mesh_api::SerialAlloc;
use sm_testkit::determinism::check;
use sm_testkit::meshes;

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-300)
}

fn f(v: &Value, k: &str) -> f64 {
    v[k].as_f64().unwrap_or_else(|| panic!("{k} missing in {v}"))
}

fn cylinder_rig(n: usize, r: f64, h: f64, kind: &str) -> (Rig, EntityId) {
    let mut rig = Rig::new();
    rig.apply(CreateCylinder { r, h }).expect("cylinder");
    let (_, d) = rig.apply(InsertCylinderMesh { n, r, h, kind: kind.into() }).expect("mesh");
    let mesh = d.outputs["mesh"][0];
    (rig, mesh)
}

/// The analytic mesh volume of an n-gon prism inscribed in the cylinder.
fn chord_volume(n: usize, r: f64, h: f64) -> f64 {
    0.5 * n as f64 * r * r * (2.0 * PI / n as f64).sin() * h
}

#[test]
fn cylinder_matches_the_inscribed_polygon_prism_and_the_deficit_is_the_chord_error() {
    for kind in ["wedge", "tet"] {
        for n in [4usize, 6, 7, 16, 31] {
            let (r, h) = (1.3, 2.7);
            let (rig, mesh) = cylinder_rig(n, r, h, kind);
            let v = rig.query("mesh.volume_check", json!({"mesh": mesh})).unwrap();
            let row = &v["regions"][0];
            assert_eq!(v["regions"].as_array().unwrap().len(), 1);
            assert_eq!(row["region"], "TopoVolume#1");
            let (want_mesh, want_model) = (chord_volume(n, r, h), PI * r * r * h);
            assert!(rel(f(row, "mesh_volume"), want_mesh) < 1e-12, "{kind} n={n}: {row}");
            assert!(rel(f(row, "model_volume"), want_model) < 1e-12, "{kind} n={n}: {row}");
            let d = f(row, "deficit");
            assert!((d - (want_model - want_mesh)).abs() < 1e-12 * want_model, "{kind} n={n}: deficit {d}");
            assert!(d > 0.0, "an inscribed mesh is smaller than the cylinder");
            assert!((f(row, "relative_deficit") - d / want_model).abs() < 1e-14);
            assert_eq!(f(&v, "total_deficit"), d);
            assert_eq!(v["cells"], if kind == "wedge" { n } else { 3 * n });
        }
    }
}

#[test]
fn chord_error_shrinks_as_one_over_n_squared() {
    let ds: Vec<f64> = [8usize, 16, 32]
        .iter()
        .map(|&n| {
            let (rig, m) = cylinder_rig(n, 1.0, 1.0, "wedge");
            f(&rig.query("mesh.volume_check", json!({"mesh": m})).unwrap()["regions"][0], "relative_deficit")
        })
        .collect();
    // 1 - sin(x)/x with x = 2 pi / n is about x^2 / 6: halving x quarters the deficit.
    for (a, b) in ds.iter().zip(&ds[1..]) {
        assert!((a / b - 4.0).abs() < 0.2, "{a} / {b}");
    }
    assert!((ds[0] - (1.0 - (2.0 * PI / 8.0).sin() / (2.0 * PI / 8.0))).abs() < 1e-12);
}

#[test]
fn brick_has_no_deficit_for_hex_and_tet() {
    let (rig, mesh) = Rig::with_block([3, 4, 5], &[]);
    let v = rig.query("mesh.volume_check", json!({"mesh": mesh})).unwrap();
    assert_eq!(v["regions"].as_array().unwrap().len(), 1);
    assert!(rel(f(&v["regions"][0], "mesh_volume"), 60.0) < 1e-12);
    assert!(rel(f(&v["regions"][0], "model_volume"), 60.0) < 1e-12);
    assert!(f(&v["regions"][0], "deficit").abs() < 1e-12 * 60.0);
    // Omitting `mesh` picks the only one.
    assert_eq!(rig.query("mesh.volume_check", json!({})).unwrap(), v);
    // A tet block classified on the same brick.
    let mut rig = Rig::new();
    rig.apply(CreateBrick { size: [2.0, 3.0, 4.0] }).unwrap();
    let t = meshes::tet_block([2, 3, 4], [1.0; 3]).unwrap();
    let r = volume_report(&t.mesh, &|_| Ok(24.0));
    assert!(r.regions[0].deficit.unwrap().abs() < 1e-12 * 24.0);
    assert_eq!(r.regions[0].cells, 2 * 3 * 4 * 6);
}

#[test]
fn tet_and_both_dual_modes_report_the_same_volume() {
    use sm_mesh_api::Classification;
    use sm_poly::{EdgeResolver, FaceMode, PolyOptions, dualize};
    // Engine path on a brick: mesh.to_poly, then the query on both meshes.
    let mut rig = Rig::new();
    rig.apply(CreateBrick { size: [3.0, 2.0, 2.0] }).unwrap();
    let (_, d) = rig.apply(InsertTetBlock { n: [3, 2, 2] }).unwrap();
    let tet = d.outputs["mesh"][0];
    let (_, d) = rig.json("mesh.to_poly", json!({"mesh": tet.to_string()})).expect("to_poly");
    let poly = d.outputs["mesh"][0];
    let a = rig.query("mesh.volume_check", json!({"mesh": tet})).unwrap();
    let b = rig.query("mesh.volume_check", json!({"mesh": poly})).unwrap();
    assert!(a["cells"].as_u64() != b["cells"].as_u64(), "the dual has other cells");
    for v in [&a, &b] {
        assert!(rel(f(&v["regions"][0], "mesh_volume"), 12.0) < 1e-12);
        assert!(f(&v["regions"][0], "deficit").abs() < 1e-12 * 12.0);
    }
    // Library path on the cylinder, both face modes: the dual encloses the same volume, so the
    // chord deficit is the same.
    struct Pair;
    impl EdgeResolver for Pair {
        fn resolve(&self, a: EntityId, b: EntityId, _e: [Classification; 2]) -> Option<EntityId> {
            EntityId::new(EntityKind::TopoEdge, a.serial() * 1000 + b.serial())
        }
    }
    let (r, h, n) = (1.0, 2.0, 12);
    let (p, c) = cylinder_cells(n, r, h, "tet");
    let src = meshes::build_mesh(&p, &c, &mut SerialAlloc::new()).unwrap();
    let model = |_: EntityId| Ok(PI * r * r * h);
    let base = volume_report(&src, &model);
    assert!(rel(base.regions[0].mesh_volume.unwrap(), chord_volume(n, r, h)) < 1e-12);
    for faces in [FaceMode::Planar, FaceMode::Keep] {
        let mut alloc = SerialAlloc::new();
        let (fin, _) = dualize(&src, &PolyOptions::new().with_faces(faces), &Pair, sm_mesh::MeshBuilder::new(), &mut alloc).expect("dualize");
        let rep = volume_report(&fin.mesh, &model);
        assert_eq!(rep.regions.len(), 1);
        assert!(rel(rep.regions[0].mesh_volume.unwrap(), base.regions[0].mesh_volume.unwrap()) < 1e-12, "{faces:?}");
        assert!((rep.total_deficit.unwrap() - base.total_deficit.unwrap()).abs() < 1e-12 * PI * r * r * h, "{faces:?}");
    }
}

#[test]
fn every_volume_gets_its_own_row() {
    let fx = meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 1).unwrap();
    // Volumes 1 and 2 of an 4x2x2 block split after the first x layer: 1*2*2 and 3*2*2 unit cubes.
    let model = |v: EntityId| Ok(if v.serial() == 1 { 4.0 } else { 13.0 });
    let r = volume_report(&fx.mesh, &model);
    assert_eq!(r.regions.len(), 2);
    assert_eq!((r.regions[0].region.serial(), r.regions[1].region.serial()), (1, 2));
    assert!(rel(r.regions[0].mesh_volume.unwrap(), 4.0) < 1e-12);
    assert!(rel(r.regions[1].mesh_volume.unwrap(), 12.0) < 1e-12);
    assert!((r.regions[0].deficit.unwrap()).abs() < 1e-12);
    assert!(rel(r.regions[1].deficit.unwrap(), 1.0) < 1e-12);
    assert!(rel(r.total_mesh_volume.unwrap(), 16.0) < 1e-12);
    assert!(rel(r.total_model_volume.unwrap(), 17.0) < 1e-12);
    assert_eq!(r.cells, sm_mesh_api::MeshRead::cell_count(&fx.mesh));
}

#[test]
fn a_missing_model_volume_is_null_with_a_note() {
    let (mut rig, mesh) = Rig::with_block([2, 2, 2], &[]);
    rig.apply(KillVolume { volume: id(EntityKind::TopoVolume, 1) }).unwrap();
    let v = rig.query("mesh.volume_check", json!({"mesh": mesh})).unwrap();
    let row = &v["regions"][0];
    assert!(row.get("model_volume").is_none_or(Value::is_null));
    assert!(row["model_note"].as_str().is_some_and(|s| !s.is_empty()), "{row}");
    assert!(row.get("deficit").is_none_or(Value::is_null));
    assert!(rel(f(row, "mesh_volume"), 8.0) < 1e-12);
    assert!(v.get("total_deficit").is_none_or(Value::is_null));
    assert_finite(&v, "volume");
}

#[test]
fn bad_requests_are_refused() {
    let (rig, _) = Rig::with_block([1, 1, 1], &[]);
    assert!(rig.query("mesh.volume_check", json!({"extra": 1})).is_err());
    assert!(rig.query("mesh.volume_check", json!({"mesh": "Mesh#99"})).is_err());
}

#[test]
fn reply_is_bit_identical_across_threads_and_runs() {
    let (rig, mesh) = cylinder_rig(37, 1.1, 0.9, "tet");
    let run = |t: usize| {
        rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap().install(|| serde_json::to_string(&rig.query("mesh.volume_check", json!({"mesh": mesh})).unwrap()).unwrap())
    };
    let a = check(1, 4, run).unwrap();
    assert_eq!(a, run(1));
}

#[test]
fn reply_is_golden() {
    let (rig, mesh) = cylinder_rig(8, 1.0, 2.0, "wedge");
    let v = rig.query("mesh.volume_check", json!({"mesh": mesh})).unwrap();
    assert_finite(&v, "volume");
    golden("mesh.volume_check.reply.json", &v, 1e-9);
}
