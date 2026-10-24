//! Timeline behavior through the facade: checkout, fork and replay give bit-identical states.

mod common;

use std::sync::Arc;

use common::*;
use serde_json::json;
use sm_engine::EngineError;
use sm_op_api::{DerivedCache, Rev, TimelineError};

/// Ten ops on main: returns the node and state hash after each, plus the counter id.
fn ten(
    ws: &mut sm_engine::Workspace,
) -> (
    Vec<(sm_op_api::Hash256, sm_op_api::Hash256)>,
    sm_core::EntityId,
) {
    let mut v = Vec::new();
    let r = ws
        .apply(
            &Create {
                name: "a".into(),
                initial: 0,
                gain: Some(0.1),
            },
            &opts(),
        )
        .unwrap();
    let id = r.delta.outputs["counter"][0];
    v.push((r.node, r.state));
    for k in 1..10i64 {
        let r = if k % 3 == 0 {
            ws.apply(&ScaleGain { id, factor: 3.0 }, &opts()).unwrap()
        } else {
            ws.apply(&Add { id, delta: k }, &opts()).unwrap()
        };
        v.push((r.node, r.state));
    }
    (v, id)
}

#[test]
fn checkout_and_fork_replay_bit_identical_states() {
    let (b, _) = builder();
    let mut ws = b.memory().unwrap();
    let (nodes, id) = ten(&mut ws);
    let head10 = nodes[9];

    // Fork from node 3, diverge, switch back.
    ws.fork("try", Some(nodes[2].0)).unwrap();
    assert_eq!(ws.state_hash(), nodes[2].1);
    let side = add(&mut ws, id, 1000);
    assert_ne!(ws.state_hash(), nodes[2].1);
    ws.checkout(Rev::Ref("main".into())).unwrap();
    assert_eq!(ws.state_hash(), head10.1);
    // Every node reproduces its recorded state, by checkout and by cold replay from the root.
    for (n, s) in &nodes {
        ws.checkout(Rev::Node(*n)).unwrap();
        assert_eq!(ws.state_hash(), *s);
        assert_eq!(ws.state_from_root(*n).unwrap().hash(), *s);
    }
    ws.checkout(Rev::Node(side)).unwrap();
    assert_eq!(ws.timeline().depth(side), Some(4));
    ws.checkout(Rev::Empty).unwrap();
    assert_eq!(total(&ws), 0);
}

#[test]
fn cold_derived_cache_reproduces_hashes() {
    let (b, _) = builder();
    let mut warm = b.memory().unwrap();
    let (nodes, _) = ten(&mut warm);
    let (b2, _) = builder();
    let mut cold = b2
        .derived_cache(Arc::new(DerivedCache::disabled()))
        .memory()
        .unwrap();
    let (nodes2, _) = ten(&mut cold);
    assert_eq!(nodes, nodes2, "node and state hashes are reproducible");
}

#[test]
fn json_and_typed_paths_record_the_same_node() {
    let (b, _) = builder();
    let mut a = b.memory().unwrap();
    let (b, _) = builder();
    let mut j = b.memory().unwrap();
    let ra = a
        .apply(
            &Create {
                name: "x".into(),
                initial: 5,
                gain: None,
            },
            &opts(),
        )
        .unwrap();
    // Extra unknown field, null and int-for-float are canonicalized away.
    let rj = j
        .apply_json(
            "counter.create",
            json!({"name": "x", "initial": 5, "gain": null, "junk": 1}),
            &opts(),
        )
        .unwrap();
    assert_eq!(ra.node, rj.node);
    let id = ra.delta.outputs["counter"][0];
    let r = j
        .apply_json(
            "counter.scale_gain",
            json!({"id": id, "factor": 2}),
            &opts(),
        )
        .unwrap();
    let r2 = a.apply(&ScaleGain { id, factor: 2.0 }, &opts()).unwrap();
    assert_eq!(r.node, r2.node);
    // Analytic: gain 1.0 * 2 = 2, read back through a query.
    let g = j.query("counter.gain", &json!({"id": id})).unwrap();
    assert!((g.as_f64().unwrap() - 2.0).abs() < 1e-12);
}

#[test]
fn plugin_registration_order_does_not_matter() {
    let (b, _) = builder();
    let mut x = b.memory().unwrap();
    let count = ApplyCount::default();
    let mut y = sm_engine::Workspace::builder()
        .plugin(ExtrasPlugin)
        .plugin(CounterPlugin)
        .services(
            sm_op_api::Services::new()
                .with(count)
                .with(Flaky::default()),
        )
        .clock(sm_engine::FixedClock(TIME.into()))
        .author("tester")
        .memory()
        .unwrap();
    assert_eq!(x.catalog_json(), y.catalog_json());
    let (nx, _) = ten(&mut x);
    let (ny, _) = ten(&mut y);
    assert_eq!(nx, ny);
}

#[test]
fn failing_op_leaves_state_and_timeline_untouched() {
    let (b, _) = builder();
    let mut ws = b.memory().unwrap();
    create(&mut ws, "a", 1);
    let before = ws.status();
    let e = ws
        .apply_json("counter.fail", json!({}), &opts())
        .unwrap_err();
    assert_eq!(e.code(), "failed");
    assert_eq!(ws.status(), before);
    let e = ws
        .apply_json("counter.nope", json!({}), &opts())
        .unwrap_err();
    assert_eq!(e.code(), "unknown-op");
}

#[test]
fn query_at_reads_other_revisions_without_moving_head() {
    let (b, _) = builder();
    let mut ws = b.memory().unwrap();
    let id = create(&mut ws, "a", 1);
    let n1 = ws.status().position.unwrap();
    add(&mut ws, id, 10);
    let head = ws.status();
    let t = ws
        .query_at(&Rev::Node(n1), "counter.total", &json!({}))
        .unwrap();
    assert_eq!(t["total"], 1);
    assert_eq!(total(&ws), 11);
    assert_eq!(ws.status(), head);
    assert!(matches!(
        ws.query_at(&Rev::Ref("nope".into()), "counter.total", &json!({})),
        Err(EngineError::Timeline(TimelineError::UnknownRef(_)))
    ));
}

#[test]
fn log_walks_to_the_root_newest_first() {
    let (b, _) = builder();
    let mut ws = b.memory().unwrap();
    let id = create(&mut ws, "a", 1);
    add(&mut ws, id, 1);
    add(&mut ws, id, 1);
    let log = ws.log(None, 10).unwrap();
    assert_eq!(
        log.iter().map(|e| e.op.as_str()).collect::<Vec<_>>(),
        ["counter.add", "counter.add", "counter.create"]
    );
    assert_eq!(log[0].depth, 3);
    assert_eq!(log[2].parent, None);
    assert_eq!(log[0].author, "tester");
    assert_eq!(ws.log(None, 2).unwrap().len(), 2);
}

#[test]
fn branch_management_rules() {
    let (b, _) = builder();
    let mut ws = b.memory().unwrap();
    assert!(matches!(
        ws.fork("x", None),
        Err(EngineError::Timeline(TimelineError::EmptyPosition))
    ));
    let id = create(&mut ws, "a", 1);
    add(&mut ws, id, 1);
    ws.create_branch("keep", None).unwrap();
    ws.fork("feat/one", None).unwrap();
    assert!(!ws.branch_name_ok("feat"), "file/directory clash");
    assert!(!ws.branch_name_ok("bad name"));
    assert!(matches!(
        ws.delete_branch("feat/one"),
        Err(EngineError::Timeline(TimelineError::HeadOnBranch(_)))
    ));
    ws.rename_branch("feat/one", "feat/two").unwrap();
    assert_eq!(ws.status().head, sm_op_api::Head::Ref("feat/two".into()));
    ws.checkout(Rev::Ref("main".into())).unwrap();
    ws.delete_branch("feat/two").unwrap();
    assert_eq!(
        ws.status().branches.keys().cloned().collect::<Vec<_>>(),
        ["keep", "main"]
    );
}

#[test]
fn naming_a_branch_after_the_unborn_head_branch_is_refused() {
    // Undo to the root leaves HEAD on an unborn branch; creating that name would leave HEAD
    // on a born branch while the position stays empty.
    let (b, _) = builder();
    let mut ws = b.memory().unwrap();
    create(&mut ws, "a", 1);
    let n1 = ws.status().position.unwrap();
    ws.fork("b0", None).unwrap();
    ws.undo().unwrap();
    assert_eq!(ws.status().position, None);
    for r in [
        ws.create_branch("b0", Some(n1)),
        ws.fork("b0", Some(n1)),
        ws.rename_branch("main", "b0"),
    ] {
        assert_eq!(r.unwrap_err().code(), "head-on-branch");
    }
    ws.redo().unwrap();
    assert_eq!(total(&ws), 1);
}

#[test]
fn prepare_off_thread_then_commit_through_the_facade() {
    let dir = TempDir::new("prep");
    let (b, _) = builder();
    let mut ws = b
        .create(dir.project("p"), sm_engine::ProjectConfig::new("p"))
        .unwrap();
    let id = create(&mut ws, "a", 0);
    let base = ws.prepare_base();
    let q = sm_op_api::QuietControl::new();
    let prepared = std::thread::scope(|s| {
        s.spawn(|| base.prepare(&Add { id, delta: 5 }, &q.control()).unwrap())
            .join()
            .unwrap()
    });
    let r = ws.commit_prepared(prepared, &opts()).unwrap();
    assert_eq!(total(&ws), 5);
    assert!(!ws.dirty(), "committed and autosaved");
    // A base taken before another op is stale and records nothing.
    let stale = ws
        .prepare_base()
        .prepare(&Add { id, delta: 1 }, &q.control())
        .unwrap();
    add(&mut ws, id, 100);
    let e = ws.commit_prepared(stale, &opts()).unwrap_err();
    assert_eq!(e.code(), "stale-base");
    assert_eq!(total(&ws), 105);
    assert_ne!(r.node, ws.status().position.unwrap());
}
