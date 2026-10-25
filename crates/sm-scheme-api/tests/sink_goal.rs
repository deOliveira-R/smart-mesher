//! `IntervalSink::goal`: a real-valued goal from a scheme, through the planner with the
//! reference solver. Goals average with the other goals of the class; soft and hard counts win.

use sm_scheme_api::stubs::{brick_setup, registry};
use sm_scheme_api::{
    Applicability, Caps, Driver, Firm, Firmness, GenCtx, GenerateFailure, IntervalCtx,
    IntervalRequest, IntervalSink, MeshInput, MeshIntent, Params, Patch, PlanCtx, ReferenceCornerPicker, ReferenceIntervalSolver,
    Scheme, SchemeChoice, SchemeDim, Target, WholeFaces,
};

/// A curve scheme that only states a goal for its own edge.
struct Goaler;

impl Scheme for Goaler {
    fn name(&self) -> &str {
        "goaler"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Curve
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::NONE
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(10, "goal")
    }
    fn interval_constraints(
        &self,
        _cx: &IntervalCtx<'_>,
        target: Target,
        p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let goal: f64 = p.get("goaler", "goal").ok().flatten().unwrap_or(1.0);
        if let Target::Curve(e) = target {
            out.goal(e, goal);
        }
        Ok(())
    }
    fn generate(&self, _cx: &GenCtx<'_>, t: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        Err(GenerateFailure::fatal(t, "goaler", "plan-only scheme"))
    }
}

/// Plan the brick with `goaler(goal)` on its first x-edge. All four x-edges get `req` (when
/// given) as their interval request; returns the solved count of the first x-edge.
fn solved(goal: f64, req: Option<Firm<IntervalRequest>>) -> u32 {
    let (topo, ids, model) = brick_setup([4.0, 4.0, 4.0]);
    let mut reg = registry();
    reg.register(Box::new(Goaler)).expect("register");
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    let e = ids.edges[0];
    intent.set_scheme(
        e,
        SchemeChoice::with_params("goaler", serde_json::json!({ "goal": goal }), Firmness::Hard),
    );
    if let Some(r) = req {
        for k in 0..4 {
            intent.set_intervals(ids.edges[k], r);
        }
    }
    let plan = driver.plan(&input, &intent).expect("plan");
    plan.intervals.iter().find(|x| x.0 == e).expect("edge").1
}

#[test]
fn goal_averages_with_the_other_goals_and_is_rounded_once() {
    let auto4 = Some(Firm::auto(IntervalRequest::Count(4)));
    // Four default counts of 4 and a goal of 14: mean (16 + 14) / 5 = 6.
    assert_eq!(solved(14.0, auto4), 6);
    // A real goal moves the mean below the rounding step: (16 + 9.5) / 5 = 5.1 -> 5.
    assert_eq!(solved(9.5, auto4), 5);
    // A goal equal to the mean changes nothing.
    assert_eq!(solved(4.0, auto4), 4);
}

#[test]
fn soft_and_hard_counts_override_a_goal() {
    assert_eq!(solved(30.0, Some(Firm::soft(IntervalRequest::Count(7)))), 7);
    assert_eq!(solved(30.0, Some(Firm::hard(IntervalRequest::Count(2)))), 2);
}
