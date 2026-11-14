//! `mesh.interval_relation`: relative intervals as IIA rows (W4-SIZE-IIA-OP).

mod common;

use common::*;
use serde_json::json;
use sm_layer_api::MeshLayer;
use sm_op_api::Session;

fn counts(s: &Session) -> Vec<(sm_core::EntityId, u32)> {
    let layer = s.state().layer::<MeshLayer>().expect("layer");
    let (_, e) = layer.meshes().next().expect("mesh");
    e.plan.as_ref().expect("plan").intervals.clone()
}

fn count_of(s: &Session, n: u64) -> u32 {
    counts(s)
        .iter()
        .find(|(e, _)| *e == edge(n))
        .expect("edge planned")
        .1
}

/// Brick 10^3 with size 1 (every edge would be 10), `b[0]` hard at `nb`, then `a rel ratio * b`.
fn related(a: &[&str], b: &[&str], ratio: u32, rel: &str, nb: u32) -> Result<Session, String> {
    let mut s = brick_session([10.0; 3], 1.0);
    run_json(
        &mut s,
        "mesh.intervals",
        json!({"targets": [b[0]], "count": nb}),
    )
    .map_err(|e| e.to_string())?;
    run_json(
        &mut s,
        "mesh.interval_relation",
        json!({"a": a, "b": b, "ratio": ratio, "relation": rel}),
    )
    .map_err(|e| e.to_string())?;
    mesh_run(&mut s).map_err(|e| e.to_string())?;
    Ok(s)
}

#[test]
fn ratio_two_gives_na_equal_two_nb_exactly() {
    for nb in [1u32, 3, 4, 7] {
        let s = related(&["TopoEdge#5"], &["TopoEdge#1"], 2, "eq", nb).expect("run");
        assert_eq!(count_of(&s, 1), nb);
        assert_eq!(count_of(&s, 5), 2 * nb, "n_a = 2 n_b for n_b = {nb}");
    }
}

#[test]
fn default_ratio_is_one_and_le_ge_bound_the_count() {
    let s = related(&["TopoEdge#5"], &["TopoEdge#1"], 1, "eq", 5).expect("run");
    assert_eq!(count_of(&s, 5), 5);
    let s = related(&["TopoEdge#5"], &["TopoEdge#1"], 2, "le", 3).expect("run");
    assert!(count_of(&s, 5) <= 6);
    let s = related(&["TopoEdge#5"], &["TopoEdge#1"], 2, "ge", 8).expect("run");
    assert!(count_of(&s, 5) >= 16);
}

#[test]
fn sums_over_chains_balance() {
    // n(e5) + n(e9) = 2 n(e1)
    let s = related(&["TopoEdge#5", "TopoEdge#9"], &["TopoEdge#1"], 2, "eq", 4).expect("run");
    assert_eq!(count_of(&s, 5) + count_of(&s, 9), 8);
}

#[test]
fn recorded_and_deterministic() {
    let h = |with: bool| {
        let mut s = brick_session([10.0; 3], 1.0);
        if with {
            run_json(
                &mut s,
                "mesh.interval_relation",
                json!({"a": ["TopoEdge#5"], "b": ["TopoEdge#1"], "ratio": 2}),
            )
            .expect("relation");
        }
        mesh_run(&mut s).expect("run");
        (s.state().hash(), counts(&s))
    };
    let (h1, c1) = h(true);
    let (h2, c2) = h(true);
    assert_eq!((h1, &c1), (h2, &c2), "two runs agree");
    let (h0, _) = h(false);
    assert_ne!(h1, h0, "the row is part of the state hash");
    // A repeated command is idempotent for the stored intent.
    let mut s = brick_session([10.0; 3], 1.0);
    for _ in 0..2 {
        run_json(
            &mut s,
            "mesh.interval_relation",
            json!({"a": ["TopoEdge#5"], "b": ["TopoEdge#1"], "ratio": 2}),
        )
        .expect("relation");
    }
    let l = s.state().layer::<sm_mesher::IntentLayer>().expect("intent");
    assert_eq!(l.intent().intervals.len(), 1);
}

#[test]
fn bad_parameters_are_typed_errors() {
    let mut s = brick_session([10.0; 3], 1.0);
    let mut bad = |p: serde_json::Value| {
        let r = run_json(&mut s, "mesh.interval_relation", p);
        assert!(r.is_err(), "expected an error");
    };
    bad(json!({"a": ["TopoEdge#5"], "b": ["TopoEdge#1"], "ratio": 0}));
    bad(json!({"a": [], "b": ["TopoEdge#1"]}));
    bad(json!({"a": ["TopoFace#1"], "b": ["TopoEdge#1"]}));
    bad(json!({"a": ["TopoEdge#1"], "b": ["TopoEdge#1"]}));
    bad(json!({"a": ["TopoEdge#999"], "b": ["TopoEdge#1"]}));
}

/// Undo removes the row (hash back to the one before), redo restores it, and a replay from the
/// root verifies the recorded hash of the relation's node.
#[test]
fn undo_redo_and_replay_reproduce_the_hash() {
    let mut s = brick_session([10.0; 3], 1.0);
    let before = s.state().hash();
    run_json(
        &mut s,
        "mesh.interval_relation",
        json!({"a": ["TopoEdge#5"], "b": ["TopoEdge#1"], "ratio": 2, "relation": "le"}),
    )
    .expect("relation");
    let with_row = s.state().hash();
    assert_ne!(with_row, before);
    let node = s.position().expect("position");
    let rows = |s: &Session| {
        s.state()
            .layer::<sm_mesher::IntentLayer>()
            .expect("intent")
            .intent()
            .intervals
            .len()
    };
    assert_eq!(rows(&s), 1);
    s.undo().expect("undo");
    assert_eq!(s.state().hash(), before, "undo removes the row");
    assert_eq!(rows(&s), 0);
    s.redo().expect("redo");
    assert_eq!(s.state().hash(), with_row, "redo restores the same state");
    assert_eq!(rows(&s), 1);
    let q = sm_op_api::QuietControl::new();
    let replayed = s
        .state_at_from_root(node, &q.control())
        .expect("replay verifies the recorded hash");
    assert_eq!(replayed.hash(), with_row);
}
