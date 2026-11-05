//! W3b-AUTO acceptance A: the strategy registry, `mesh.auto` and its stored record, on the stub
//! schemes. (Acceptance B runs through `standard()` in SERVER-HOST.)

mod common;

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex};

use common::*;
use serde_json::json;
use sm_layer_api::MeshLayer;
use sm_mesher::{
    AutoInput, AutoLists, AutoStrategy, DefaultStrategy, MesherConfig, SchemeAssignment,
    replan_budget,
};
use sm_op_api::{DerivedCache, Hash256, OpRecord, QuietControl, Session};
use sm_scheme_api::stubs;
use sm_scheme_api::{
    Applicability, Caps, GenCtx, GenerateFailure, IntervalCtx, IntervalSink, Params, Patch,
    PlanCtx, Requirements, Scheme, SchemeDim, SchemeError, Target,
};

// ------------------------------------------------------------------ test schemes

type Calls = Arc<Mutex<BTreeMap<(Target, String), usize>>>;

/// A surface scheme that behaves as `map` with a fixed score, under another name, and counts
/// `applicability` calls per (target, scheme).
struct Twin {
    name: &'static str,
    calls: Calls,
}

impl Scheme for Twin {
    fn name(&self) -> &str {
        self.name
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, p: &Params) -> Caps {
        stubs::Map.caps(p)
    }
    fn applicability(&self, cx: &PlanCtx<'_>, t: Target, p: &Params) -> Applicability {
        *self
            .calls
            .lock()
            .expect("lock")
            .entry((t, self.name.to_owned()))
            .or_default() += 1;
        match stubs::Map.applicability(cx, t, p) {
            Applicability::Applicable { .. } => Applicability::yes(150, "twin"),
            other => other,
        }
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        t: Target,
        p: &Params,
        out: &mut Requirements<'_>,
    ) -> Result<(), SchemeError> {
        stubs::Map.requirements(cx, t, p, out)
    }
    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        t: Target,
        p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> Result<(), SchemeError> {
        stubs::Map.interval_constraints(cx, t, p, out)
    }
    fn generate(&self, cx: &GenCtx<'_>, t: Target, p: &Params) -> Result<Patch, GenerateFailure> {
        stubs::Map.generate(cx, t, p)
    }
}

/// `sweep` under another name and score.
struct SweepAs {
    name: &'static str,
    score: u32,
}

impl Scheme for SweepAs {
    fn name(&self) -> &str {
        self.name
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, p: &Params) -> Caps {
        stubs::Sweep.caps(p)
    }
    fn applicability(&self, cx: &PlanCtx<'_>, t: Target, p: &Params) -> Applicability {
        match stubs::Sweep.applicability(cx, t, p) {
            Applicability::Applicable { .. } => Applicability::yes(self.score, "sweep alias"),
            other => other,
        }
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        t: Target,
        p: &Params,
        out: &mut Requirements<'_>,
    ) -> Result<(), SchemeError> {
        stubs::Sweep.requirements(cx, t, p, out)
    }
    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        t: Target,
        p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> Result<(), SchemeError> {
        stubs::Sweep.interval_constraints(cx, t, p, out)
    }
    fn generate(&self, cx: &GenCtx<'_>, t: Target, p: &Params) -> Result<Patch, GenerateFailure> {
        stubs::Sweep.generate(cx, t, p)
    }
}

/// A volume scheme that always scores highest and always fails to generate (fallback).
struct Flaky;

impl Scheme for Flaky {
    fn name(&self) -> &str {
        "flaky"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::HEX
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(200, "eager")
    }
    fn generate(&self, _cx: &GenCtx<'_>, t: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        Err(GenerateFailure::fallback(t, "flaky", "refuses to mesh"))
    }
}

fn lists(surface: &[&str], volume: &[&str]) -> AutoLists {
    let v = |a: &[&str]| a.iter().map(|s| (*s).to_owned()).collect();
    AutoLists {
        curve: v(&["equal"]),
        surface: v(surface),
        volume: v(volume),
    }
}

fn session_with(
    reg: sm_scheme_api::SchemeRegistry,
    cfg: impl FnOnce(MesherConfig) -> MesherConfig,
) -> Session {
    Session::new(engine_cfg(
        cfg(MesherConfig::new(reg)),
        DerivedCache::default(),
    ))
}

fn brick_n(s: &mut Session, n: usize) {
    for i in 0..n {
        let op = CreateBrick {
            origin: [3.0 * i as f64, 0.0, 0.0],
            size: [1.0; 3],
        };
        run(s, &op).expect("brick");
    }
}

/// The stored record of the head node.
fn head_record(s: &Session) -> OpRecord {
    let h = s.position().expect("head");
    s.timeline().get(h).expect("node").body.op.clone()
}

fn assignments_of(rec: &OpRecord) -> Vec<(String, String, Option<serde_json::Value>)> {
    rec.params["assignments"]
        .as_array()
        .expect("assignments")
        .iter()
        .map(|a| {
            (
                a["target"].as_str().expect("target").to_owned(),
                a["scheme"].as_str().expect("scheme").to_owned(),
                a.get("params").cloned(),
            )
        })
        .collect()
}

// ------------------------------------------------------------------ A1

#[test]
fn a1_stored_record_is_explicit_and_equals_the_scheme_script() {
    let mut a = brick_session_plain();
    let (_, d) = run_json(&mut a, "mesh.auto", json!({})).expect("auto");
    assert!(d.diagnostics.is_empty(), "{:?}", d.diagnostics);
    let rec = head_record(&a);
    assert_eq!(rec.name, "mesh.auto");
    assert_eq!(rec.params["strategy"], "default");
    assert!(rec.params.get("targets").is_none());
    let asg = assignments_of(&rec);
    assert!(
        asg.iter().any(|x| x.0 == "TopoVolume#1" && x.1 == "sweep"),
        "{asg:?}"
    );
    mesh_run(&mut a).expect("run a");

    // The same intent through `mesh.scheme` per assignment.
    let mut b = brick_session_plain();
    for (t, scheme, params) in &asg {
        let mut p = json!({"targets": [t], "scheme": scheme});
        if let Some(pp) = params {
            p["params"] = pp.clone();
        }
        run_json(&mut b, "mesh.scheme", p).expect("scheme");
    }
    mesh_run(&mut b).expect("run b");
    assert_eq!(a.state().hash(), b.state().hash());
}

fn brick_session_plain() -> Session {
    let mut s = session();
    run(&mut s, &brick_op([2.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("size");
    s
}

// ------------------------------------------------------------------ A2

#[test]
fn a2_unknown_strategy_and_wrong_kind_targets_are_invalid_and_change_nothing() {
    let mut s = brick_session_plain();
    let h = s.state().hash();
    let bad = [
        json!({"strategy": "nope"}),
        json!({"targets": ["TopoEdge#1"]}),
        json!({"targets": ["TopoVolume#1", "TopoFace#1"]}),
        json!({"targets": ["TopoVolume#99"]}),
        json!({"targets": ["TopoVolume#1"], "assignments": []}),
        json!({"assignments": [{"target": "TopoVolume#1", "scheme": "nonesuch"}]}),
        json!({"assignments": [{"target": "TopoEdge#1", "scheme": "equal"}]}),
        json!({"assignments": [
            {"target": "TopoVolume#1", "scheme": "sweep"},
            {"target": "TopoVolume#1", "scheme": "sweep"}]}),
    ];
    for p in bad {
        let r = run_json(&mut s, "mesh.auto", p.clone());
        let e = format!("{:?}", r.expect_err("must fail"));
        assert!(
            e.contains("Invalid") || e.contains("UnknownEntity"),
            "{p}: {e}"
        );
        assert_eq!(s.state().hash(), h, "{p}");
    }
    let e = format!(
        "{:?}",
        run_json(&mut s, "mesh.auto", json!({"strategy": "nope"})).expect_err("fail")
    );
    assert!(e.contains("default"), "names the known strategies: {e}");
}

// ------------------------------------------------------------------ A3 / A4

fn twin_session(order: &[&'static str], calls: &Calls) -> Session {
    let mut reg = stubs::registry();
    for n in ["twin_a", "twin_b"] {
        reg.register(Box::new(Twin {
            name: n,
            calls: calls.clone(),
        }))
        .expect("twin");
    }
    let order: Vec<&str> = order.to_vec();
    let mut s = session_with(reg, |c| {
        c.with_strategy(Arc::new(DefaultStrategy::with_lists(lists(
            &order,
            &["sweep"],
        ))))
    });
    run(&mut s, &brick_op([2.0; 3])).expect("brick");
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoFace#1"], "size": 1.0}),
    )
    .expect("size");
    s
}

#[test]
fn a3_list_order_is_the_tie_break() {
    let winner = |order: &[&'static str]| {
        let calls = Calls::default();
        let mut s = twin_session(order, &calls);
        run_json(&mut s, "mesh.auto", json!({"targets": ["TopoFace#1"]})).expect("auto");
        let asg = assignments_of(&head_record(&s));
        assert_eq!(asg.len(), 1, "{asg:?}");
        assert_eq!(asg[0].0, "TopoFace#1");
        asg[0].1.clone()
    };
    assert_eq!(winner(&["twin_a", "twin_b"]), "twin_a");
    assert_eq!(winner(&["twin_b", "twin_a"]), "twin_b");
}

#[test]
fn a4_applicability_is_called_at_most_once_per_target_and_candidate() {
    for targets in [json!(["TopoFace#1"]), json!(["TopoVolume#1"])] {
        let calls = Calls::default();
        let mut s = twin_session(&["twin_a", "twin_b"], &calls);
        run_json(&mut s, "mesh.auto", json!({"targets": targets})).expect("auto");
        let c = calls.lock().expect("lock");
        assert!(!c.is_empty(), "the counting scheme was consulted");
        for (k, n) in c.iter() {
            assert_eq!(*n, 1, "{k:?} scored {n} times in one plan");
        }
    }
}

// ------------------------------------------------------------------ A5

#[test]
fn a5_replan_budget_values() {
    assert_eq!(replan_budget(0), 16);
    assert_eq!(replan_budget(40), 96);
}

fn flaky_session(strategy: DefaultStrategy, n: usize) -> Session {
    let mut reg = stubs::registry();
    reg.register(Box::new(Flaky)).expect("flaky");
    let mut s = session_with(reg, |c| c.with_strategy(Arc::new(strategy)));
    brick_n(&mut s, n);
    s
}

#[test]
fn a5_twenty_volume_fallback_model_succeeds_within_the_budget_and_fails_at_a_fixed_16() {
    let l = lists(&["map"], &["flaky", "sweep"]);
    let mut s = flaky_session(DefaultStrategy::with_lists(l.clone()), 20);
    run_json(&mut s, "mesh.auto", json!({})).expect("budget 16 + 2n suffices");
    let asg = assignments_of(&head_record(&s));
    let vols: Vec<_> = asg
        .iter()
        .filter(|a| a.0.starts_with("TopoVolume"))
        .collect();
    assert_eq!(vols.len(), 20);
    assert!(vols.iter().all(|a| a.1 == "sweep"), "{vols:?}");

    let mut s = flaky_session(DefaultStrategy::with_lists(l).with_fixed_budget(16), 20);
    let h = s.state().hash();
    let e = format!(
        "{:?}",
        run_json(&mut s, "mesh.auto", json!({})).expect_err("fixed 16 fails")
    );
    assert!(e.contains("gave up") || e.contains("Failed"), "{e}");
    assert_eq!(s.state().hash(), h);
}

// ------------------------------------------------------------------ A6

#[test]
fn a6_auto_tet_only_for_a_tet_fallback_volume() {
    let mut reg = stubs::registry();
    reg.register(Box::new(Flaky)).expect("flaky");
    reg.register(Box::new(SweepAs {
        name: "tetfall",
        score: 10,
    }))
    .expect("tetfall");
    let strat = |first: &str| {
        DefaultStrategy::with_lists(lists(&["map"], &[first, "tetfall"])).with_tet_scheme("tetfall")
    };
    // First choice works: no warning.
    let mut s = session_with(reg, |c| c.with_strategy(Arc::new(strat("sweep"))));
    brick_n(&mut s, 1);
    let (_, d) = run_json(&mut s, "mesh.auto", json!({})).expect("auto");
    assert!(
        d.diagnostics.iter().all(|x| x.code != "mesh.auto_tet"),
        "{:?}",
        d.diagnostics
    );

    // First choice fails to generate: the tet fallback is used and explained.
    let mut reg = stubs::registry();
    reg.register(Box::new(Flaky)).expect("flaky");
    reg.register(Box::new(SweepAs {
        name: "tetfall",
        score: 10,
    }))
    .expect("tetfall");
    let mut s = session_with(reg, |c| c.with_strategy(Arc::new(strat("flaky"))));
    brick_n(&mut s, 2);
    let (_, d) = run_json(&mut s, "mesh.auto", json!({})).expect("auto");
    let tet: Vec<_> = d
        .diagnostics
        .iter()
        .filter(|x| x.code == "mesh.auto_tet")
        .collect();
    assert_eq!(tet.len(), 2, "{:?}", d.diagnostics);
    for t in tet {
        assert!(t.message.contains("flaky"), "{}", t.message);
        assert!(t.message.contains("refuses to mesh"), "{}", t.message);
        assert!(t.message.contains("mesh.scheme"), "{}", t.message);
        assert_eq!(t.entities.len(), 1);
    }
}

// ------------------------------------------------------------------ A7 / A8

/// Assigns every target the scheme `pick` names; counts its calls.
struct Counting {
    pick: &'static str,
    calls: Arc<AtomicUsize>,
}

impl AutoStrategy for Counting {
    fn name(&self) -> &'static str {
        "counting"
    }
    fn choose(&self, input: &AutoInput<'_>) -> Result<Vec<SchemeAssignment>, SchemeError> {
        self.calls.fetch_add(1, Ordering::SeqCst);
        Ok(input
            .targets()
            .iter()
            .map(|&t| SchemeAssignment::new(t, self.pick))
            .collect())
    }
}

fn counting_engine(
    pick: &'static str,
    calls: &Arc<AtomicUsize>,
    cache: DerivedCache,
) -> sm_op_api::Engine {
    let cfg = MesherConfig::new(stubs::registry()).with_strategy(Arc::new(Counting {
        pick,
        calls: calls.clone(),
    }));
    engine_cfg(cfg, cache)
}

fn record_timeline(pick: &'static str, calls: &Arc<AtomicUsize>) -> (Session, Vec<Hash256>) {
    let mut s = Session::new(counting_engine(pick, calls, DerivedCache::disabled()));
    let mut nodes = Vec::new();
    run(&mut s, &brick_op([2.0; 3])).expect("brick");
    nodes.push(s.position().expect("pos"));
    run_json(&mut s, "mesh.auto", json!({"strategy": "counting"})).expect("auto");
    nodes.push(s.position().expect("pos"));
    run_json(
        &mut s,
        "mesh.size",
        json!({"targets": ["TopoVolume#1"], "size": 1.0}),
    )
    .expect("size");
    nodes.push(s.position().expect("pos"));
    (s, nodes)
}

#[test]
fn a7_replay_from_root_calls_no_strategy_and_reproduces_every_hash() {
    let calls = Arc::new(AtomicUsize::new(0));
    let (s, nodes) = record_timeline("sweep", &calls);
    assert_eq!(calls.load(Ordering::SeqCst), 1, "one call at record time");
    let q = QuietControl::new();
    for &n in &nodes {
        let st = s
            .state_at_from_root(n, &q.control())
            .expect("replay verifies the recorded hash");
        assert_eq!(st.hash(), s.timeline().get(n).expect("node").check.state);
    }
    assert_eq!(
        calls.load(Ordering::SeqCst),
        1,
        "replay called the strategy 0 times"
    );
    let rec = head_record_of(&s, nodes[1]);
    assert_eq!(assignments_of(&rec)[0].1, "sweep");
}

fn head_record_of(s: &Session, h: Hash256) -> OpRecord {
    s.timeline().get(h).expect("node").body.op.clone()
}

#[test]
fn a8_swapping_the_strategy_does_not_move_recorded_hashes() {
    let c1 = Arc::new(AtomicUsize::new(0));
    let (s1, nodes) = record_timeline("sweep", &c1);
    let recorded: Vec<Hash256> = nodes
        .iter()
        .map(|&n| s1.timeline().get(n).expect("node").check.state)
        .collect();
    let recs: Vec<OpRecord> = nodes.iter().map(|&n| head_record_of(&s1, n)).collect();

    // Engine with the v2 implementation under the same name; replay the stored records.
    let c2 = Arc::new(AtomicUsize::new(0));
    let eng2 = counting_engine("mapvol", &c2, DerivedCache::disabled());
    let q = QuietControl::new();
    let mut st = eng2.registry().new_state();
    for (rec, want) in recs.iter().zip(&recorded) {
        eng2.apply_record(&mut st, rec, &q.control())
            .expect("replay");
        assert_eq!(st.hash(), *want);
    }
    assert_eq!(c2.load(Ordering::SeqCst), 0);

    // A fresh mesh.auto under v2 chooses differently.
    let mut s2 = Session::new(counting_engine("mapvol", &c2, DerivedCache::disabled()));
    run(&mut s2, &brick_op([2.0; 3])).expect("brick");
    run_json(&mut s2, "mesh.auto", json!({"strategy": "counting"})).expect("auto");
    assert_eq!(c2.load(Ordering::SeqCst), 1);
    assert_eq!(assignments_of(&head_record(&s2))[0].1, "mapvol");
    assert_ne!(
        s2.state().hash(),
        s1.timeline().get(nodes[1]).expect("node").check.state
    );
}

// ------------------------------------------------------------------ A9

fn auto_flow(threads: usize) -> (Hash256, String) {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .expect("pool");
    pool.install(|| {
        let mut s = brick_session_plain();
        run_json(&mut s, "mesh.auto", json!({})).expect("auto");
        let rec = serde_json::to_string(&head_record(&s).params).expect("json");
        mesh_run(&mut s).expect("run");
        let exact = sm_op_api::Layer::exact_hash(s.state().layer::<MeshLayer>().expect("layer"));
        assert_ne!(exact, Hash256::of(b""));
        (s.state().hash(), rec)
    })
}

#[test]
fn a9_mesh_auto_is_deterministic_across_runs_and_threads() {
    sm_testkit::determinism::check(2, 4, auto_flow).expect("deterministic");
}

#[test]
fn a9_explicit_runs_do_not_see_the_strategy_registry() {
    // With and without extra strategies registered, an explicit-scheme run is bit-identical.
    let run_it = |extra: bool| {
        let cfg = MesherConfig::new(stubs::registry());
        let cfg = if extra {
            cfg.with_strategy(Arc::new(DefaultStrategy::with_lists(lists(
                &["pave"],
                &["sweep"],
            ))))
        } else {
            cfg
        };
        let mut s = Session::new(engine_cfg(cfg, DerivedCache::default()));
        run(&mut s, &brick_op([4.0; 3])).expect("brick");
        run_json(
            &mut s,
            "mesh.size",
            json!({"targets": ["TopoVolume#1"], "size": 1.0}),
        )
        .expect("size");
        run_json(
            &mut s,
            "mesh.scheme",
            json!({"targets": ["TopoVolume#1"], "scheme": "sweep",
                   "params": {"source": ["TopoFace#5"], "target": ["TopoFace#6"]}}),
        )
        .expect("scheme");
        mesh_run(&mut s).expect("run");
        s.state().hash()
    };
    assert_eq!(run_it(false), run_it(true));
}

#[test]
fn strategies_query_lists_names_and_descriptions() {
    let s = session();
    let q = QuietControl::new();
    let out = s
        .engine()
        .run_query(s.state(), "mesh.auto_strategies", &json!({}), &q.control())
        .expect("query");
    let rows = out["strategies"].as_array().expect("rows");
    assert_eq!(rows.len(), 1);
    assert_eq!(rows[0]["name"], "default");
    assert!(!rows[0]["description"].as_str().expect("str").is_empty());
}

#[test]
fn census_lists_changed_cases_and_reads_only() {
    // Registry lists make volumes `mapvol` (no sweep candidate before); the strategy adds sweep.
    let mut reg = stubs::registry();
    reg.set_auto(SchemeDim::Volume, &["mapvol"])
        .expect("auto list");
    let cfg = MesherConfig::new(reg);
    let mut s = Session::new(engine_cfg(cfg.clone(), DerivedCache::default()));
    brick_n(&mut s, 2);
    let h = s.state().hash();
    let rep = sm_mesher::census(s.state(), &cfg, None, None).expect("census");
    assert_eq!(s.state().hash(), h);
    assert_eq!(rep.rows.len(), 2);
    assert_eq!(rep.strategy, "default");
    for r in &rep.rows {
        assert_eq!(r.after.scheme.as_deref(), Some("sweep"), "{r:?}");
        assert_ne!(r.before.scheme.as_deref(), Some("sweep"), "{r:?}");
        assert!(r.changed);
    }
    let md = rep.to_markdown();
    assert!(md.contains("2 changed"), "{md}");
    assert_eq!(
        rep,
        sm_mesher::census(s.state(), &cfg, None, None).expect("again")
    );
    assert!(sm_mesher::census(s.state(), &cfg, Some("nope"), None).is_err());
}
