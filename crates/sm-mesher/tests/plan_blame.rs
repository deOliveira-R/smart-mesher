//! W3c-MESHER-REPLAN: `mesh.auto` replans at plan time. Two cut bricks share a cap face; one
//! neighbour imposes tri on it, the other quad. The planner reports `NoApplicableScheme` with the
//! blameable imposers, and `scratch_plan` rejects them in turn (sharing the replan budget with
//! generation-time rejections).

mod common;

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use common::*;
use serde_json::json;
use sm_mesher::{
    AutoInput, AutoLists, AutoStrategy, DefaultStrategy, MesherConfig, SchemeAssignment,
};
use sm_op_api::{DerivedCache, Hash256, OpRecord, QuietControl, Session};
use sm_scheme_api::stubs;
use sm_scheme_api::{
    Applicability, Caps, Firmness, GenCtx, GenerateFailure, IntervalCtx, IntervalSink, Params,
    Patch, PlanCtx, Requirements, Scheme, SchemeDim, SchemeError, Target,
};

/// A stub `sweep` along the axis through the shared cap (the face created last, highest id), with
/// its own score, an optional hard imposition on the cap, an optional volume it applies to, and an
/// optional generation failure.
struct CapSweep {
    name: &'static str,
    score: u32,
    only: Option<u64>,
    need: Option<Caps>,
    fail_gen: bool,
}

impl CapSweep {
    /// The cap and the sweep parameters (cap as source, its opposite face as target).
    fn pair(&self, cx: &PlanCtx<'_>, t: Target) -> Option<(sm_core::EntityId, Params)> {
        let faces = cx.input.piece_faces(t).ok()?;
        let cap = *faces.iter().max()?;
        for g in faces.iter().copied().filter(|f| *f != cap) {
            let p = Params(json!({"source": cap, "target": g}));
            if matches!(
                stubs::Sweep.applicability(cx, t, &p),
                Applicability::Applicable { .. }
            ) {
                return Some((cap, p));
            }
        }
        None
    }
}

impl Scheme for CapSweep {
    fn name(&self) -> &str {
        self.name
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, p: &Params) -> Caps {
        stubs::Sweep.caps(p)
    }
    fn applicability(&self, cx: &PlanCtx<'_>, t: Target, _p: &Params) -> Applicability {
        let v = t.entity().map(|e| e.serial());
        if self.only.is_some() && self.only != v {
            return Applicability::no("not for this volume");
        }
        match self.pair(cx, t) {
            Some(_) => Applicability::yes(self.score, "cap sweep"),
            None => Applicability::no("no cap pair"),
        }
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        t: Target,
        _p: &Params,
        out: &mut Requirements<'_>,
    ) -> Result<(), SchemeError> {
        let (cap, p) = self.pair(cx, t).ok_or_else(|| SchemeError::NotApplicable {
            target: t,
            scheme: sm_scheme_api::SchemeName::new(self.name),
            reason: "no cap pair".into(),
        })?;
        stubs::Sweep.requirements(cx, t, &p, out)?;
        if let Some(c) = self.need {
            out.require_caps(Target::Surface(cap), c, Firmness::Hard);
        }
        Ok(())
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
        if self.fail_gen {
            return Err(GenerateFailure::fallback(t, self.name, "refuses to mesh"));
        }
        stubs::Sweep.generate(cx, t, p)
    }
}

fn sweeps(triswp: Option<u64>, quadswp: Option<u64>, others: Option<u64>) -> Vec<CapSweep> {
    let c = |name, score, only, need, fail_gen| CapSweep {
        name,
        score,
        only,
        need,
        fail_gen,
    };
    vec![
        c("triswp", 150, triswp, Some(Caps::TRI), false),
        c("quadswp", 150, quadswp, Some(Caps::QUAD), false),
        c("flakyswp", 100, others, None, true),
        c("okswp", 40, others, None, false),
    ]
}

fn lists(volume: &[&str]) -> AutoLists {
    let v = |a: &[&str]| a.iter().map(|s| (*s).to_owned()).collect();
    AutoLists {
        curve: v(&["equal"]),
        surface: v(&["map", "tri"]),
        volume: v(volume),
    }
}

/// Delegates to the default strategy and counts `choose` calls.
struct CountingDefault {
    inner: DefaultStrategy,
    calls: Arc<AtomicUsize>,
}

impl AutoStrategy for CountingDefault {
    fn name(&self) -> &'static str {
        self.inner.name()
    }
    fn choose(&self, input: &AutoInput<'_>) -> Result<Vec<SchemeAssignment>, SchemeError> {
        self.calls.fetch_add(1, Ordering::SeqCst);
        self.inner.choose(input)
    }
}

/// A different strategy (other name, other choices) that must never run during replay.
struct Other {
    calls: Arc<AtomicUsize>,
}

impl AutoStrategy for Other {
    fn name(&self) -> &'static str {
        "other"
    }
    fn choose(&self, input: &AutoInput<'_>) -> Result<Vec<SchemeAssignment>, SchemeError> {
        self.calls.fetch_add(1, Ordering::SeqCst);
        Ok(input
            .targets()
            .iter()
            .map(|&t| SchemeAssignment::new(t, "flakyswp"))
            .collect())
    }
}

fn cut_engine(
    schemes: Vec<CapSweep>,
    strat: Arc<dyn AutoStrategy>,
    cache: DerivedCache,
) -> sm_op_api::Engine {
    let mut reg = stubs::registry();
    for s in schemes {
        reg.register(Box::new(s)).expect("register");
    }
    engine_cfg(MesherConfig::new(reg).with_strategy(strat), cache)
}

fn cut_session(schemes: Vec<CapSweep>, hard_v1: Option<&str>, strat: DefaultStrategy) -> Session {
    cut_session_with(schemes, hard_v1, Arc::new(strat))
}

fn cut_session_with(
    schemes: Vec<CapSweep>,
    hard_v1: Option<&str>,
    strat: Arc<dyn AutoStrategy>,
) -> Session {
    let mut s = Session::new(cut_engine(schemes, strat, DerivedCache::default()));
    run(
        &mut s,
        &CreateBrick {
            origin: [0.0; 3],
            size: [1.0, 2.0, 3.0],
        },
    )
    .expect("brick");
    let (_, d) = run(&mut s, &Webcut { frac: 0.5 }).expect("webcut");
    let vols: Vec<String> = d.outputs["volumes"].iter().map(|v| v.to_string()).collect();
    assert_eq!(vols, ["TopoVolume#2", "TopoVolume#3"]);
    run_json(&mut s, "mesh.size", json!({"targets": vols, "size": 1.0})).expect("size");
    if let Some(h) = hard_v1 {
        run_json(
            &mut s,
            "mesh.scheme",
            json!({"targets": [vols[0]], "scheme": h}),
        )
        .expect("hard");
    }
    s
}

fn run_both(s: &mut Session) -> Result<(Hash256, sm_op_api::Delta), sm_op_api::TimelineError> {
    run_json(
        s,
        "mesh.run",
        json!({"targets": ["TopoVolume#2", "TopoVolume#3"]}),
    )
}

fn head_record(s: &Session) -> OpRecord {
    let h = s.position().expect("head");
    s.timeline().get(h).expect("node").body.op.clone()
}

fn scheme_of(rec: &OpRecord, target: &str) -> String {
    rec.params["assignments"]
        .as_array()
        .expect("assignments")
        .iter()
        .find(|a| a["target"] == target)
        .unwrap_or_else(|| panic!("no assignment for {target}: {}", rec.params))["scheme"]
        .as_str()
        .expect("scheme")
        .to_owned()
}

fn err_text<T>(r: Result<T, sm_op_api::TimelineError>) -> String {
    format!("{:?}", r.err().expect("expected an error"))
}

#[test]
fn plan_time_blame_rejects_the_auto_imposer_and_replays() {
    // V1 is hard `triswp` (tri on the cap); V2's best candidate `quadswp` imposes quad on it.
    let strat = DefaultStrategy::with_lists(lists(&["quadswp", "okswp"]));
    let calls = Arc::new(AtomicUsize::new(0));
    let counting = CountingDefault {
        inner: strat.clone(),
        calls: calls.clone(),
    };
    let mut s = cut_session_with(sweeps(None, None, None), Some("triswp"), Arc::new(counting));
    run_json(&mut s, "mesh.auto", json!({})).expect("auto replans at plan time");
    assert_eq!(scheme_of(&head_record(&s), "TopoVolume#3"), "okswp");
    let auto_rec = head_record(&s);
    run_both(&mut s).expect("run");
    let at_record = calls.load(Ordering::SeqCst);
    assert_eq!(at_record, 1, "one strategy call at record time");

    // The stored record is explicit: a second build gives the same hashes.
    let h = s.state().hash();
    let mut t = cut_session(sweeps(None, None, None), Some("triswp"), strat.clone());
    run_json(&mut t, "mesh.auto", json!({})).expect("auto");
    run_both(&mut t).expect("run");
    assert_eq!(t.state().hash(), h);

    // (a) Replay from the root reproduces every node's hash without calling the strategy.
    let q = QuietControl::new();
    let head = s.position().expect("head");
    let nodes = s.timeline().path_to_root(head).expect("path");
    assert!(nodes.len() >= 5, "{}", nodes.len());
    for &n in &nodes {
        let st = s
            .state_at_from_root(n, &q.control())
            .expect("replay verifies the recorded hash");
        assert_eq!(st.hash(), s.timeline().get(n).expect("node").check.state);
    }
    assert_eq!(calls.load(Ordering::SeqCst), at_record, "replay: 0 calls");

    // (b) apply_record of the stored records under an engine with another strategy.
    let c2 = Arc::new(AtomicUsize::new(0));
    let eng2 = cut_engine(
        sweeps(None, None, None),
        Arc::new(Other { calls: c2.clone() }),
        DerivedCache::disabled(),
    );
    let mut st = eng2.registry().new_state();
    let mut saw_auto = false;
    for &n in &nodes {
        let node = s.timeline().get(n).expect("node");
        eng2.apply_record(&mut st, &node.body.op, &q.control())
            .expect("replay");
        assert_eq!(st.hash(), node.check.state);
        saw_auto |= node.body.op == auto_rec;
    }
    assert!(saw_auto, "the mesh.auto record was replayed");
    assert_eq!(c2.load(Ordering::SeqCst), 0);

    // Negative control: no replan allowed gives the planner's own error and changes nothing.
    let mut c = cut_session(
        sweeps(None, None, None),
        Some("triswp"),
        strat.with_fixed_budget(0),
    );
    let before = c.state().hash();
    let e = err_text(run_json(&mut c, "mesh.auto", json!({})));
    assert!(e.contains("no applicable scheme"), "{e}");
    assert_eq!(c.state().hash(), before);
}

#[test]
fn the_second_imposed_by_entry_can_be_the_one_blamed() {
    // Both volumes are auto. `triswp` applies to V1 only (no fallback), `quadswp` and the others to
    // V2 only, and they conflict on the cap. imposed_by sorts V1 first; rejecting it leaves V1 with
    // no scheme, so the loop must go on to V2's `quadswp`.
    let l = lists(&["triswp", "quadswp", "okswp"]);
    let build =
        |strat: DefaultStrategy| cut_session(sweeps(Some(2), Some(3), Some(3)), None, strat);
    let mut s = build(DefaultStrategy::with_lists(l.clone()));
    run_json(&mut s, "mesh.auto", json!({})).expect("auto");
    let rec = head_record(&s);
    assert_eq!(scheme_of(&rec, "TopoVolume#2"), "triswp");
    assert_eq!(scheme_of(&rec, "TopoVolume#3"), "okswp");
    run_both(&mut s).expect("run");

    // Two tries were needed, so a budget of 1 stops (budget 2 passes).
    let mut c = build(DefaultStrategy::with_lists(l.clone()).with_fixed_budget(1));
    let e = err_text(run_json(&mut c, "mesh.auto", json!({})));
    assert!(e.contains("ReplanLimit") || e.contains("gave up"), "{e}");
    let mut d = build(DefaultStrategy::with_lists(l).with_fixed_budget(2));
    run_json(&mut d, "mesh.auto", json!({})).expect("budget 2");
}

#[test]
fn plan_time_and_generation_time_rejections_share_one_budget() {
    // n = 1 plan-time rejection (`quadswp`), m = 1 generation-time rejection (`flakyswp`).
    let l = lists(&["quadswp", "flakyswp", "okswp"]);
    let go = |budget: usize| {
        let strat = DefaultStrategy::with_lists(l.clone()).with_fixed_budget(budget);
        let mut s = cut_session(sweeps(None, None, None), Some("triswp"), strat);
        run_json(&mut s, "mesh.auto", json!({})).map(|_| head_record(&s))
    };
    let rec = go(2).expect("n + m = 2 fits a budget of 2");
    assert_eq!(scheme_of(&rec, "TopoVolume#3"), "okswp");
    let e = err_text(go(1));
    assert!(e.contains("ReplanLimit") || e.contains("gave up"), "{e}");
}
