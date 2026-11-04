//! Scheme notes end to end (ruling INT-7): a tet mesh whose reporting floor is tightened in
//! the scheme parameters yields located findings in `mesh.quality`; the default floor yields none.
#![allow(missing_docs)]

mod common;

use std::sync::Arc;

use common::{CreateBrick, Rig, registry, rig_op_names};
use serde_json::{Value, json};
use sm_mesher::MesherConfig;
use sm_op_api::{DerivedCache, Engine, Services, Session};
use sm_scheme_api::SchemeRegistry;

fn real_registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    sm_scheme_curve::register(&mut r).expect("curve");
    sm_scheme_tri::register(&mut r).expect("tri");
    sm_scheme_tet::register(&mut r).expect("tet");
    r
}

fn brick_rig() -> Rig {
    let services = Services::new().with_versioned("mesher", 1, MesherConfig::new(real_registry())).with(rig_op_names());
    let engine = Engine::new(Arc::new(registry()), services, Arc::new(DerivedCache::default())).expect("engine");
    let mut r = Rig { s: Session::new(engine) };
    r.apply(CreateBrick { size: [1.0, 1.0, 1.0] }).expect("brick");
    r
}

/// Brick meshed with real schemes; `params` go to `tetmesh`. Returns the rig and the mesh id.
fn tet_rig(params: Value) -> (Rig, String) {
    let mut r = brick_rig();
    let edges: Vec<String> = (1..=12).map(|i| format!("TopoEdge#{i}")).collect();
    let faces: Vec<String> = (1..=6).map(|i| format!("TopoFace#{i}")).collect();
    r.json("mesh.size", json!({"targets": ["TopoVolume#1"], "size": 0.34})).unwrap();
    r.json("mesh.scheme", json!({"targets": edges, "scheme": "equal"})).unwrap();
    r.json("mesh.scheme", json!({"targets": faces, "scheme": "trimesh"})).unwrap();
    r.json("mesh.scheme", json!({"targets": ["TopoVolume#1"], "scheme": "tetmesh", "params": params})).unwrap();
    let (_, d) = r.json("mesh.run", json!({"targets": ["TopoVolume#1"]})).unwrap();
    let mesh = d.outputs["mesh"][0].to_string();
    (r, mesh)
}

#[test]
fn default_floor_reports_nothing_extra() {
    let (r, mesh) = tet_rig(json!({}));
    let q = r.query("mesh.quality", json!({"mesh": mesh})).unwrap();
    assert!(q.get("scheme_notes").is_none(), "{q}");
    assert!(q.get("n_scheme_notes").is_none(), "{q}");
}

#[test]
fn tightened_floor_gives_located_findings() {
    let (r, mesh) = tet_rig(json!({"min_dihedral_deg": 60.0}));
    let q = r.query("mesh.quality", json!({"mesh": mesh})).unwrap();
    let n = q["n_scheme_notes"].as_u64().expect("count") as usize;
    assert!(n > 0, "{q}");
    let notes = q["scheme_notes"].as_array().expect("notes");
    assert_eq!(notes.len(), n.min(sm_diag::MAX_SCHEME_NOTES));
    for f in notes {
        assert_eq!(f["metric"], "min_dihedral");
        assert!(f["value"].as_f64().unwrap() < 60.0);
        assert_eq!(f["threshold"].as_f64().unwrap(), 60.0);
        assert!(!f["suggestion"].as_str().unwrap().is_empty());
        assert_eq!(f["owner"], "TopoVolume#1", "classification-based location");
        assert!(f["cell"]["id"].is_string());
        assert!(f["position"].as_array().unwrap().iter().all(|x| x.as_f64().is_some_and(f64::is_finite)));
        assert!(!f["entities"].as_array().unwrap().is_empty());
    }
    // Worst first by value / threshold.
    let vals: Vec<f64> = notes.iter().map(|f| f["value"].as_f64().unwrap()).collect();
    assert!(vals.windows(2).all(|w| w[0] <= w[1]), "worst first");
}

#[test]
fn findings_are_deterministic() {
    let a = tet_rig(json!({"min_mean_ratio": 0.9}));
    let b = tet_rig(json!({"min_mean_ratio": 0.9}));
    let qa = a.0.query("mesh.quality", json!({"mesh": a.1})).unwrap();
    let qb = b.0.query("mesh.quality", json!({"mesh": b.1})).unwrap();
    assert_eq!(qa, qb);
    assert!(qa["scheme_notes"].as_array().unwrap().iter().all(|f| f["metric"] == "mean_ratio"));
}

#[test]
fn out_of_range_floor_is_rejected() {
    let mut r = brick_rig();
    let r1 = r.json("mesh.scheme", json!({"targets": ["TopoVolume#1"], "scheme": "tetmesh", "params": {"min_dihedral_deg": 80.0}}));
    let r2 = r.json("mesh.run", json!({"targets": ["TopoVolume#1"]}));
    assert!(r1.is_err() || r2.is_err());
}
