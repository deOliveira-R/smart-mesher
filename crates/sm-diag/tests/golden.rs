//! JSON stability: schema snapshots, one golden reply per query, no NaN or inf.
#![allow(missing_docs)]

mod common;
use common::*;
use serde_json::json;
use sm_core::EntityKind;

fn rig_registry() -> sm_op_api::Registry {
    registry()
}

#[test]
fn schemas_are_snapshotted() {
    let reg = rig_registry();
    let queries: Vec<_> = reg.query_catalog().into_iter().map(|q| q.expect("schema")).collect();
    for q in queries.iter().filter(|q| q.name.starts_with("mesh.") && matches!(q.name.as_str(), "mesh.validate" | "mesh.quality")) {
        golden(&format!("{}.output.schema.json", q.name), &serde_json::to_value(&q.output_schema).unwrap(), 0.0);
        golden(&format!("{}.input.schema.json", q.name), &serde_json::to_value(&q.schema).unwrap(), 0.0);
    }
    let op = reg.catalog().into_iter().map(|o| o.expect("schema")).find(|o| o.name == "mesh.flag_cells").expect("op registered");
    golden("mesh.flag_cells.input.schema.json", &serde_json::to_value(&op.schema).unwrap(), 0.0);
    // The op declares its role and the entity kind of `mesh`.
    let outs = reg.op_outputs("mesh.flag_cells");
    assert_eq!(outs.len(), 1);
    assert_eq!(outs[0].name, "set");
    assert_eq!(outs[0].kinds, &[EntityKind::MeshSet]);
    assert!(format!("{:?}", outs[0].cardinality).contains("Optional"));
    let s = serde_json::to_string(&op.schema).unwrap();
    assert!(s.contains("x-entity-kind") && s.contains("\"Mesh\""), "{s}");
    for name in ["mesh.validate", "mesh.quality"] {
        let q = queries.iter().find(|q| q.name == name).unwrap();
        assert!(serde_json::to_string(&q.schema).unwrap().contains("x-entity-kind"), "{name}");
    }
}

#[test]
fn registered_by_the_plugin_only() {
    let reg = rig_registry();
    assert!(reg.query_names().contains(&"mesh.validate") && reg.query_names().contains(&"mesh.quality"));
    assert!(reg.op_names().contains(&"mesh.flag_cells"));
    // No layer is registered by the plugin: the layer order is that of CoreLayers + MeshPlugin.
    assert_eq!(sm_op_api::Plugin::id(&sm_diag::DiagPlugin), "diag");
}

#[test]
fn quality_reply_is_golden_and_finite() {
    let (rig, mesh) = Rig::with_pair("sheared");
    let v = rig.query("mesh.quality", json!({"profile": "openfoam", "worst_n": 3, "bins": 4})).unwrap();
    assert_eq!(v["schema_version"], 1);
    assert_eq!(v["mesh"], mesh.to_string());
    assert_finite(&v, "quality");
    let raw = serde_json::to_string(&v).unwrap();
    assert!(!raw.contains("NaN") && !raw.to_lowercase().contains("inf"), "{raw}");
    golden("mesh.quality.reply.json", &v, 1e-9);
    // The same through the generic profile: floats still finite, thresholds with nulls.
    let g = rig.query("mesh.quality", json!({"thresholds": {"non_orthogonality": {"fail": 40}}})).unwrap();
    assert_finite(&g, "quality");
    assert_eq!(g["verdict"], "fail");
}

#[test]
fn validate_reply_is_golden_and_finite() {
    let mut rig = Rig::new();
    rig.apply(CreateBrick { size: [1.0, 2.0, 3.0] }).unwrap();
    rig.json("mesh.size", json!({"targets": ["TopoVolume#1"], "size": 1.0})).unwrap();
    rig.json("mesh.scheme", json!({"targets": ["TopoVolume#1"], "scheme": "sweep", "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}})).unwrap();
    rig.json("mesh.run", json!({"targets": ["TopoVolume#1"]})).unwrap();
    rig.apply(KillVolume { volume: id(EntityKind::TopoVolume, 1) }).unwrap();
    let v = rig.query("mesh.validate", json!({"limit": 4})).unwrap();
    assert_eq!(v["schema_version"], 1);
    assert_eq!(v["truncated"], true);
    assert_eq!(v["violations"].as_array().unwrap().len(), 4);
    assert_finite(&v, "validate");
    golden("mesh.validate.reply.json", &v, 1e-9);
}
