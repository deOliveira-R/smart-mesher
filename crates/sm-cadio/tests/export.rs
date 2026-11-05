//! `model.export` (W3b-MODEL-EXPORT): the read-only STEP export query, on the real `standard()`
//! distribution. Library-level writer tests stay in `roundtrip.rs`.

use std::collections::BTreeMap;

use base64::Engine as _;
use base64::engine::general_purpose::STANDARD;
use serde_json::{Value, json};
use sm_cadio::ExportStepReply;
use sm_core::EntityId;
use sm_layer_api::ModelLayer;
use sm_server::{ApplyOptions, Workspace, standard};
use sm_topo_api::{TopologyView, validate, validate_geometry};

type Outputs = BTreeMap<String, Vec<EntityId>>;

fn ws() -> Workspace {
    standard().memory().expect("workspace")
}

fn apply(ws: &mut Workspace, op: &str, params: Value) -> Outputs {
    ws.apply_json(op, params, &ApplyOptions::default()).unwrap_or_else(|e| panic!("{op}: {e}")).delta.outputs.into_iter().collect()
}

fn import(ws: &mut Workspace, bytes: &[u8]) -> Outputs {
    apply(ws, "model.import_step", json!({"source": {"inline": STANDARD.encode(bytes)}}))
}

fn export(ws: &Workspace, input: Value) -> (Vec<u8>, ExportStepReply) {
    let v = ws.query("model.export", &input).unwrap_or_else(|e| panic!("model.export {input}: {e}"));
    let r: ExportStepReply = serde_json::from_value(v).expect("reply");
    (STANDARD.decode(&r.step).expect("base64"), r)
}

fn volume_of(ws: &Workspace, id: EntityId) -> f64 {
    let v = ws.query("model.measure", &json!({"entity": id.to_string()})).expect("measure");
    v["volume"].as_f64().expect("volume")
}

fn topo(ws: &Workspace) -> &dyn TopologyView {
    ws.state().layer::<ModelLayer>().expect("model").topo()
}

fn valid(ws: &Workspace) -> bool {
    validate(topo(ws)).is_empty() && validate_geometry(topo(ws)).is_empty()
}

fn rel(a: f64, b: f64) -> f64 {
    ((a - b) / b).abs()
}

fn face_counts(ws: &Workspace, vols: &[EntityId]) -> Vec<usize> {
    vols.iter().map(|v| topo(ws).boundary(*v).len()).collect()
}

#[test]
fn w3a_part_exports_and_reimports() {
    let mut a = ws();
    let out = import(&mut a, sm_testkit::w3a::STEP);
    let (hash, nodes) = (a.state_hash(), a.timeline().len());
    let (bytes, rep) = export(&a, json!({}));
    assert_eq!((a.state_hash(), a.timeline().len()), (hash, nodes), "a read-only query changes neither hash nor timeline");
    assert_eq!(rep.report.solids, 3);
    assert_eq!(rep.report.bytes as usize, bytes.len());
    let mut b = ws();
    let out2 = import(&mut b, &bytes);
    assert_eq!(out2["bodies"].len(), 3);
    for (i, want) in sm_testkit::w3a::VOLUMES.iter().enumerate() {
        let got = volume_of(&b, out2["volumes"][i]);
        assert!(rel(got, *want) <= 1e-9, "volume {i}: {got} vs {want}");
    }
    assert_eq!(face_counts(&a, &out["volumes"]), face_counts(&b, &out2["volumes"]));
    assert!(valid(&b));
}

#[test]
fn bodies_subset_writes_only_those_bodies() {
    let mut a = ws();
    let out = import(&mut a, sm_testkit::w3a::STEP);
    for pick in [1usize, 2] {
        let (bytes, rep) = export(&a, json!({"bodies": [out["bodies"][pick].to_string()]}));
        assert_eq!(rep.report.solids, 1);
        let mut b = ws();
        let o = import(&mut b, &bytes);
        assert_eq!(o["bodies"].len(), 1);
        assert!(rel(volume_of(&b, o["volumes"][0]), sm_testkit::w3a::VOLUMES[pick]) <= 1e-9);
    }
    // Volumes are accepted too, and the order of the list does not matter.
    let sel = |ix: [usize; 2]| json!({"bodies": ix.map(|i| out["volumes"][i].to_string())});
    let (x, _) = export(&a, sel([2, 0]));
    let (y, _) = export(&a, sel([0, 2]));
    assert_eq!(x, y);
}

#[test]
fn primitives_round_trip() {
    use core::f64::consts::PI;
    let cases: Vec<(&str, Value, f64)> = vec![
        ("model.create_frustum", json!({"height": 3.0, "radius": 2.0, "top": 1.0}), 7.0 * PI),
        ("model.create_torus", json!({"major": 5.0, "minor": 1.0}), 10.0 * PI * PI),
        ("model.create_sphere_part", json!({"radius": 2.0, "octants": {"x": "positive", "y": "positive", "z": "positive"}}), PI * 8.0 / 6.0),
        ("model.create_sphere_part", json!({"radius": 2.0, "octants": {"z": "positive"}}), 2.0 * PI * 8.0 / 3.0),
        ("model.create_sphere_part", json!({"radius": 2.0, "inner": 1.0}), 28.0 * PI / 3.0),
    ];
    for (op, params, analytic) in cases {
        let mut a = ws();
        let out = apply(&mut a, op, params.clone());
        let vol = out["volume"][0];
        let (bytes, _) = export(&a, json!({}));
        let mut b = ws();
        let o = import(&mut b, &bytes);
        let (v0, v1) = (volume_of(&a, vol), volume_of(&b, o["volumes"][0]));
        assert!(rel(v1, v0) <= 1e-9, "{op} {params}: {v1} vs {v0}");
        assert!(rel(v1, analytic) <= 1e-6, "{op} {params}: {v1} vs analytic {analytic}");
        assert!(valid(&b), "{op} {params}");
    }
}

#[test]
fn output_is_byte_identical_across_runs_and_thread_counts() {
    let mut a = ws();
    import(&mut a, sm_testkit::w3a::STEP);
    apply(&mut a, "model.create_frustum", json!({"height": 3.0, "radius": 2.0, "top": 1.0, "center": [20.0, 0.0, 0.0]}));
    let run = |threads: Option<usize>| {
        let f = || export(&a, json!({"schema": "ap242"})).0;
        match threads {
            None => f(),
            Some(n) => rayon::ThreadPoolBuilder::new().num_threads(n).build().expect("pool").install(f),
        }
    };
    let first = run(None);
    assert_eq!(first, run(None));
    assert_eq!(first, run(Some(1)));
    assert_eq!(first, run(Some(4)));
    let text = String::from_utf8(first).expect("ascii");
    assert!(text.contains("AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF") && text.contains("1970-01-01T00:00:00"));
}

#[test]
fn refusals_name_the_entity() {
    let mut a = ws();
    let out = apply(&mut a, "model.create_brick", json!({"x": 1.0, "y": 1.0, "z": 1.0}));
    let (hash, nodes) = (a.state_hash(), a.timeline().len());
    let err = |input: Value| a.query("model.export", &input).expect_err("refused").to_string();
    let e = err(json!({"bodies": ["TopoBody#9999"]}));
    assert!(e.contains("TopoBody#9999"), "{e}");
    let e = err(json!({"bodies": []}));
    assert!(e.contains("step.empty_selection"), "{e}");
    let face = out["faces"][0];
    let e = err(json!({"bodies": [face.to_string()]}));
    assert!(e.contains(&face.to_string()), "{e}");
    let e = err(json!({"schema": "ap999"}));
    assert!(!e.is_empty());
    // An unknown field is still rejected; `path` is part of the schema and accepted.
    assert!(a.query("model.export", &json!({"nope": 1})).is_err());
    assert!(a.query("model.export", &json!({"path": "out.step"})).is_ok());
    assert_eq!((a.state_hash(), a.timeline().len()), (hash, nodes));
    // A model with nothing to write.
    let empty = ws();
    let e = empty.query("model.export", &json!({})).expect_err("nothing to write").to_string();
    assert!(e.contains("no volume"), "{e}");
}

#[test]
fn unsupported_carrier_is_a_typed_error_naming_the_cell() {
    use sm_topo_api::{Carrier, CellComplex, OpId, Sense, SurfaceCarrier, TopologyEdit};
    let mut cc = CellComplex::new();
    cc.begin_op(OpId(1)).expect("begin");
    let ids = sm_topo_api::fixtures::brick(&mut cc, sm_core::Point3::new(0.0, 0.0, 0.0), [1.0; 3]).expect("brick");
    let q = sm_geom_api::SurfaceDef::Implicit(sm_geom_api::ImplicitDef::Quadric(sm_geom_api::Quadric::sphere(&sm_geom_api::Frame::at(sm_core::Point3::new(0.0, 0.0, 0.0)), 1.0)));
    let h = cc.shapes_mut().expect("shapes").add_surface(q).expect("add");
    cc.set_carrier(ids.faces[0], Carrier::Surface(SurfaceCarrier { surface: h, sense: Sense::Forward })).expect("carrier");
    cc.end_op().expect("end");
    // No registered op builds an implicit-surface face (only the library path reaches this
    // carrier), so the `model.export` query is covered here through the error mapping below.
    let e = sm_cadio::write_step(&cc, None, &sm_cadio::WriteOptions::default(), &mut Vec::new()).expect_err("implicit");
    assert!(matches!(&e, sm_cadio::StepError::Unsupported { entity, .. } if entity.contains(&ids.faces[0].to_string())), "{e:?}");
    assert_eq!(e.code(), "step.unsupported");
    // Through the op error mapping the code and the face survive.
    let oe: sm_op_api::OpError = e.into();
    assert!(oe.to_string().contains("step.unsupported") && oe.to_string().contains(&ids.faces[0].to_string()), "{oe}");
}

#[test]
fn model_export_is_in_the_catalog_as_a_query() {
    let w = ws();
    assert!(w.registry().query_names().contains(&"model.export"));
    assert!(!w.registry().op_names().contains(&"model.export"), "not a timeline op");
    let cat = w.registry().query_catalog();
    let info = cat.into_iter().filter_map(Result::ok).find(|q| q.name == "model.export").expect("catalog row");
    let schema = serde_json::to_value(&info.schema).expect("schema");
    assert_eq!(schema["title"], "model.export");
    assert_eq!(schema["additionalProperties"], false);
    for p in ["bodies", "schema", "path"] {
        assert!(schema["properties"][p].is_object(), "{p}: {schema}");
    }
    assert!(schema["properties"]["bodies"].to_string().contains("TopoBody"), "{schema}");
    let out = serde_json::to_value(&info.output_schema).expect("output schema");
    assert!(out.to_string().contains("losses"), "{out}");
}
