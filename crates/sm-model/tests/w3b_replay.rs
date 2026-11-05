//! W3b-PRIM acceptance 9: determinism (runs, 1 vs N threads, both hosts), undo/redo and cold
//! replay over a scenario that touches every new op; plus the catalog shape of the new ops.

mod common;

use common::*;
use serde_json::json;
use sm_core::EntityKind;
use sm_model::{
    AxisName, AxisSpec, CopyXform, CreateFrustum, CreatePrism, CreatePyramid, CreateSpherePart, CreateTorus, HalfSpace, ModelCopy, MoveTarget, MoveTo, Octants, PointSpec, RotateAbout,
};
use sm_op_api::{DerivedCache, Hash256, Op, QuietControl, Session};

fn vol(n: u64) -> sm_core::EntityId {
    id(EntityKind::TopoVolume, n)
}

fn scenario() -> Vec<Box<dyn Op>> {
    use HalfSpace::Positive as P;
    vec![
        Box::new(CreateFrustum { height: 4.0, radius: 3.0, minor: Some(2.0), top: Some(1.5), center: Some([-30.0, 0.0, 0.0]) }),
        Box::new(CreatePrism { height: 3.0, sides: 6, radius: 2.0, minor: None, center: Some([-20.0, 0.0, 0.0]) }),
        Box::new(CreatePyramid { height: 3.0, sides: 5, radius: 2.0, top: Some(0.5), minor: None, center: Some([-10.0, 0.0, 0.0]) }),
        Box::new(CreateTorus { major: 5.0, minor: 1.0, center: Some([10.0, 0.0, 0.0]) }),
        Box::new(CreateSpherePart { radius: 2.0, inner: Some(1.0), octants: Some(Octants { x: Some(P), y: None, z: Some(P) }), center: Some([30.0, 0.0, 0.0]) }),
        Box::new(MoveTo { entities: vec![vol(1)], to: MoveTarget::Point(PointSpec { x: Some(-40.0), y: None, z: None }), anchor: None }),
        Box::new(RotateAbout { entities: vec![vol(2)], angle_deg: 33.0, axis: Some(AxisSpec::Name(AxisName::Z)), origin: Some([-20.0, 0.0, 0.0]), vertices: None, face: None }),
        Box::new(ModelCopy { entities: vec![vol(3)], count: Some(3), xform: Some(CopyXform::Rotate { angle_deg: 20.0, axis: Some(AxisSpec::Vector([1.0, 1.0, 1.0])), origin: None, vertices: None, face: None }) }),
        Box::new(ModelCopy { entities: vec![vol(4), vol(5)], count: Some(2), xform: Some(CopyXform::Reflect { normal: [1.0, 2.0, 3.0], origin: Some([0.0, 1.0, 0.0]) }) }),
        Box::new(ModelCopy { entities: vec![id(EntityKind::TopoBody, 1)], count: None, xform: Some(CopyXform::Scale { factor: 1.25, origin: None }) }),
    ]
}

fn run_scenario(mut s: Session) -> (Session, Hash256) {
    let mut head = Hash256::default();
    for op in scenario() {
        head = run(&mut s, op.as_ref()).expect("scenario op").0;
    }
    (s, head)
}

fn final_hash(factory: Factory) -> (Hash256, Hash256) {
    let (s, _) = run_scenario(session(factory));
    (s.state().hash(), s.state().layer_hash("model").expect("model layer"))
}

#[test]
fn scenario_is_deterministic_across_runs_threads_and_hosts() {
    let baseline = sm_testkit::determinism::check(2, 4, |_threads| final_hash(cell_complex)).expect("deterministic");
    let prod = sm_testkit::determinism::check(2, 4, |_threads| final_hash(topo_store)).expect("deterministic");
    assert_eq!(baseline, prod, "the reference and the production store give one state hash");
}

#[test]
fn every_new_op_alone_is_deterministic() {
    for (i, op) in scenario().into_iter().enumerate() {
        // Ops 5.. need earlier volumes; the first five are self-contained creators.
        if i >= 5 {
            break;
        }
        let h = sm_testkit::determinism::check(2, 4, |_t| {
            let mut s = session(topo_store);
            run(&mut s, op.as_ref()).expect("op");
            s.state().hash()
        });
        assert!(h.is_ok(), "{}: {h:?}", op.name());
    }
}

#[test]
fn undo_redo_and_cold_replay_do_not_diverge() {
    for factory in [cell_complex as Factory, topo_store] {
        let (mut s, head) = run_scenario(Session::new(engine(factory, DerivedCache::disabled())));
        let full = s.state().hash();
        let n = scenario().len();
        let q = QuietControl::new();
        for _ in 0..n {
            s.undo().expect("undo");
        }
        assert_eq!(s.state().entity_count(), 0, "undone to the root");
        for _ in 0..n {
            s.redo().expect("redo");
        }
        assert_eq!(s.state().hash(), full);
        let cold = s.state_at_from_root(head, &q.control()).expect("cold replay");
        assert_eq!(cold.hash(), full);
        assert_eq!(cold.layer_hashes(), s.state().layer_hashes());
    }
}

#[test]
fn json_records_decode_and_round_trip() {
    let s = session(cell_complex);
    let reg = s.engine().registry();
    for (name, params) in [
        ("model.create_frustum", json!({"height": 4, "radius": 3, "top": 1})),
        ("model.create_prism", json!({"height": 3, "sides": 6, "radius": 2})),
        ("model.create_pyramid", json!({"height": 3, "sides": 4, "radius": 2, "top": 1})),
        ("model.create_torus", json!({"major": 5, "minor": 1})),
        ("model.create_sphere_part", json!({"radius": 2, "inner": 1, "octants": {"x": "positive", "z": "negative"}})),
        ("model.rotate_about", json!({"entities": ["TopoVolume#1"], "angle_deg": 90, "axis": "z"})),
        ("model.rotate_about", json!({"entities": ["TopoVolume#1"], "angle_deg": 90, "vertices": ["TopoVertex#1", "TopoVertex#2"]})),
        ("model.copy", json!({"entities": ["TopoBody#1"], "count": 3, "xform": {"kind": "move", "delta": [1, 2, 3]}})),
        ("model.copy", json!({"entities": ["TopoBody#1"], "xform": {"kind": "rotate", "angle_deg": 90, "face": "TopoFace#2"}})),
    ] {
        let rec = sm_op_api::OpRecord::new(name, 1, params).expect("record");
        let op = reg.decode(&rec).unwrap_or_else(|e| panic!("{name}: {e:?}"));
        let again = reg.decode(&sm_op_api::OpRecord::new(name, 1, op.encode().expect("encode")).expect("record")).expect("decode again");
        assert_eq!(op.encode().expect("a"), again.encode().expect("b"), "{name}");
    }
    // Unknown fields are refused.
    let rec = sm_op_api::OpRecord::new("model.create_prism", 1, json!({"height": 3, "sides": 6, "radius": 2, "bogus": 1})).expect("record");
    assert!(reg.decode(&rec).is_err());
}

#[test]
fn schemas_declare_roles_positional_and_entity_kinds() {
    let reg = registry(cell_complex);
    for name in ["model.create_frustum", "model.create_prism", "model.create_pyramid", "model.create_torus", "model.create_sphere_part"] {
        let roles: Vec<String> = reg.op_outputs(name).iter().map(|r| r.name.to_owned()).collect();
        assert_eq!(roles, ["body", "volume", "faces", "edges", "vertices"], "{name}");
    }
    for name in ["model.move_to", "model.rotate_about"] {
        assert_eq!(reg.op_outputs(name).iter().map(|r| r.name).collect::<Vec<_>>(), ["volumes"]);
    }
    assert_eq!(reg.op_outputs("model.copy").iter().map(|r| r.name).collect::<Vec<_>>(), ["volumes", "bodies"]);
    let pos = |name: &str, field: &str| reg.schema(name).expect("schema").as_value()["properties"][field]["x-positional"] == json!(true);
    assert!(pos("model.create_frustum", "height"));
    assert!(pos("model.create_prism", "sides"));
    assert!(pos("model.create_pyramid", "sides"));
    assert!(pos("model.create_sphere_part", "radius"));
    // Every entity input carries `x-entity-kind`.
    let text = |name: &str| reg.schema(name).expect("schema").as_value().to_string();
    assert!(text("model.move_to").contains("x-entity-kind"));
    let rot = text("model.rotate_about");
    assert!(rot.contains("TopoVertex") && rot.contains("TopoFace") && rot.contains("TopoVolume"), "{rot}");
    let cp = text("model.copy");
    assert!(cp.contains("TopoBody") && cp.contains("TopoVertex") && cp.contains("TopoFace"), "{cp}");
    let mv = reg.schema("model.move_to").expect("schema").as_value().clone();
    let target = mv["properties"]["to"].to_string() + &mv["$defs"].to_string();
    assert!(target.contains("TopoVertex") && target.contains("TopoEdge"), "{target}");
}
