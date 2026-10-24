//! T1 determinism: repeated runs, 1 vs N threads (these ops use no threads; the suite is also
//! run with `RAYON_NUM_THREADS=1`), undo/redo and cold replay from the root.

mod common;

use common::*;
use serde_json::json;
use sm_core::EntityKind;
use sm_model::{CreateBrick, CreateCylinder, CreateSphere, Delete, Move, Reflect, Rotate, Scale};
use sm_op_api::{DerivedCache, Hash256, Op, OpRecord, QuietControl, Session};

fn vol(n: u64) -> sm_core::EntityId {
    id(EntityKind::TopoVolume, n)
}

/// A scenario that touches every op of the plugin.
fn scenario() -> Vec<Box<dyn Op>> {
    vec![
        Box::new(CreateBrick { x: 10.0, y: None, z: None, center: None }),
        Box::new(CreateCylinder { height: 3.0, radius: 1.25, center: Some([20.0, 0.0, 0.0]) }),
        Box::new(CreateSphere { radius: 2.0, center: Some([-20.0, 0.0, 0.0]) }),
        Box::new(Move { entities: vec![vol(1)], delta: [1.0, 2.0, 3.0] }),
        Box::new(Rotate { entities: vec![vol(2)], axis: [1.0, 1.0, 0.0], angle_deg: 33.0, origin: Some([20.0, 0.0, 0.0]) }),
        Box::new(Scale { entities: vec![vol(3)], factor: 1.5, origin: None }),
        Box::new(Reflect { entities: vec![vol(1), vol(2)], normal: [1.0, 2.0, 3.0], origin: Some([0.0, 1.0, 0.0]) }),
        Box::new(Reflect { entities: vec![vol(3)], normal: [0.0, 0.0, 1.0], origin: None }),
        Box::new(Delete { bodies: vec![id(EntityKind::TopoBody, 2)] }),
    ]
}

fn run_scenario(mut s: Session) -> (Session, Vec<Hash256>) {
    let mut heads = Vec::new();
    for op in scenario() {
        let (h, _) = run(&mut s, op.as_ref()).expect("scenario op");
        heads.push(h);
    }
    (s, heads)
}

fn final_hash(factory: Factory) -> (Hash256, Hash256) {
    let (s, _) = run_scenario(session(factory));
    (s.state().hash(), s.state().layer_hash("model").expect("model layer"))
}

#[test]
fn scenario_is_deterministic_across_runs_threads_and_hosts() {
    // The ops are single-threaded; `check` still catches hidden global state (concurrent runs).
    let baseline = sm_testkit::determinism::check(2, 4, |_threads| final_hash(cell_complex)).expect("deterministic");
    let prod = sm_testkit::determinism::check(2, 4, |_threads| final_hash(topo_store)).expect("deterministic");
    assert_eq!(baseline, prod, "the reference and the production store give one state hash");
}

#[test]
fn undo_redo_and_cold_replay_do_not_diverge() {
    for factory in [cell_complex as Factory, topo_store] {
        let (mut s, heads) = run_scenario(Session::new(engine(factory, DerivedCache::disabled())));
        let full = s.state().hash();
        let head = *heads.last().expect("head");
        let q = QuietControl::new();
        for k in 0..scenario().len() {
            s.undo().expect("undo");
            if k == 3 {
                assert_ne!(s.state().hash(), full);
            }
        }
        assert_eq!(s.state().entity_count(), 0, "undone to the root");
        for _ in 0..scenario().len() {
            s.redo().expect("redo");
        }
        assert_eq!(s.state().hash(), full);
        // Replaying every node from the root re-checks each recorded state hash (no `Diverged`).
        let cold = s.state_at_from_root(head, &q.control()).expect("cold replay");
        assert_eq!(cold.hash(), full);
        assert_eq!(cold.layer_hashes(), s.state().layer_hashes());
    }
}

#[test]
fn json_params_hash_like_typed_params() {
    // `10` (an integer) and `10.0` must record the same node body and give the same state.
    let mut a = session(cell_complex);
    run(&mut a, &CreateBrick { x: 10.0, y: None, z: None, center: None }).expect("typed");
    let mut b = session(cell_complex);
    let rec = OpRecord::new("model.create_brick", 1, json!({"x": 10})).expect("record");
    let op = b.engine().registry().decode(&rec).expect("decode");
    run(&mut b, op.as_ref()).expect("json");
    assert_eq!(a.state().hash(), b.state().hash());
    // Omitted optional fields and explicit nulls decode alike.
    let rec = OpRecord::new("model.create_brick", 1, json!({"x": 10, "y": null, "z": null, "center": null})).expect("record");
    let op = b.engine().registry().decode(&rec).expect("decode nulls");
    assert_eq!(op.encode().expect("encode"), json!({"x": 10.0}));
}
