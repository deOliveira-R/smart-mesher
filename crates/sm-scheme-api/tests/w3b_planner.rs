//! W3b planner items: scheme count preferences, the firmness cascade, plan-once meshing.

use sm_core::EntityId;
use sm_mesh_api::{MeshRead, SerialAlloc};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_scheme_api::stubs::{brick_setup, registry};
use sm_scheme_api::{
    Applicability, Caps, Driver, DriverOptions, Firm, Firmness, GenCtx, GenerateFailure,
    IntervalConstraint, IntervalCtx, IntervalRequest, IntervalSink, MeshInput, MeshIntent, Params,
    Patch, PlanCtx, ReferenceCornerPicker, ReferenceIntervalSolver, RequestCascade, Scheme,
    SchemeChoice, SchemeDim, SchemeError, Sequential, Target, WholeFaces,
};
use sm_topo_api::TopologyViewExt;

/// A curve scheme that derives a count from sizes and states it as a goal or a preference.
struct Deriver;

impl Scheme for Deriver {
    fn name(&self) -> &str {
        "deriver"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Curve
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::NONE
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(10, "derive")
    }
    fn interval_constraints(
        &self,
        _cx: &IntervalCtx<'_>,
        target: Target,
        p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let n: f64 = p.get("deriver", "n").ok().flatten().unwrap_or(1.0);
        let prefer: bool = p.get("deriver", "prefer").ok().flatten().unwrap_or(false);
        if let Target::Curve(e) = target {
            if prefer {
                out.prefer(e, n);
            } else {
                out.goal(e, n);
            }
        }
        Ok(())
    }
    fn generate(&self, _cx: &GenCtx<'_>, t: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        Err(GenerateFailure::fatal(t, "deriver", "plan-only scheme"))
    }
}

/// Plan a 4x4x4 brick whose first edge runs `deriver(n)`; every edge of the brick gets `req`
/// when given. Returns the solved count of that edge.
fn solved(n: f64, prefer: bool, req: Option<Firm<IntervalRequest>>) -> u32 {
    let (topo, ids, model) = brick_setup([4.0, 4.0, 4.0]);
    let mut reg = registry();
    reg.register(Box::new(Deriver)).expect("register");
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    let e = ids.edges[0];
    intent.set_scheme(
        e,
        SchemeChoice::with_params(
            "deriver",
            serde_json::json!({ "n": n, "prefer": prefer }),
            Firmness::Hard,
        ),
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
fn a_preference_replaces_the_sizing_goal() {
    // The documented case: n = 6 derived from sizes, no request on the edge. The sizing goal
    // is one interval on each of the class's four edges, so a plain goal averages to
    // (6 + 1 + 1 + 1) / 4 = 2.5 -> 2; a preference gives 6.
    assert_eq!(solved(6.0, false, None), 2);
    assert_eq!(solved(6.0, true, None), 6);
}

#[test]
fn a_preference_replaces_default_counts_and_is_not_a_soft_count() {
    let auto4 = Some(Firm::auto(IntervalRequest::Count(4)));
    assert_eq!(solved(6.0, false, auto4), 4); // (16 + 6) / 5 = 4.4
    assert_eq!(solved(6.0, true, auto4), 6);
    // A user's soft or hard count still wins over a preference.
    assert_eq!(solved(6.0, true, Some(Firm::soft(IntervalRequest::Count(3)))), 3);
    assert_eq!(solved(6.0, true, Some(Firm::hard(IntervalRequest::Count(2)))), 2);
}

/// The interval request the planner recorded for `edge` as `(count, firmness)`.
fn recorded(plan: &sm_scheme_api::Plan, edge: EntityId) -> Option<(u32, Firmness)> {
    plan.problem.intervals.iter().find_map(|r| match r.item {
        IntervalConstraint::Fixed {
            edge: e,
            count,
            firmness,
        } if e == edge && r.origin == sm_scheme_api::problem::Origin::Intent => {
            Some((count, firmness))
        }
        _ => None,
    })
}

/// Plan the brick with a request on its first face and one on an edge of that face (the edge
/// request is issued first when `edge_first`); returns what the planner recorded on the edge.
fn cascaded(
    cascade: RequestCascade,
    face_req: Firm<IntervalRequest>,
    edge_req: Firm<IntervalRequest>,
    edge_first: bool,
) -> (u32, Firmness) {
    let (topo, ids, model) = brick_setup([4.0, 4.0, 4.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let mut driver = Driver::new(&reg, &solver, &picker);
    driver.options = DriverOptions::new().with_cascade(cascade);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let face = ids.faces[0];
    let edge = topo.coedges(face).next().expect("coedge").key.edge;
    let mut intent = MeshIntent::default();
    if edge_first {
        intent.set_intervals(edge, edge_req);
        intent.set_intervals(face, face_req);
    } else {
        intent.set_intervals(face, face_req);
        intent.set_intervals(edge, edge_req);
    }
    let plan = driver.plan(&input, &intent).expect("plan");
    recorded(&plan, edge).expect("recorded")
}

#[test]
fn level_cascade_lets_an_edge_beat_a_firmer_face() {
    let (face, edge) = (
        Firm::hard(IntervalRequest::Count(5)),
        Firm::soft(IntervalRequest::Count(3)),
    );
    assert_eq!(
        cascaded(RequestCascade::Level, face, edge, false),
        (3, Firmness::Soft)
    );
}

#[test]
fn firmness_cascade_lets_the_firmer_request_win() {
    let (face, edge) = (
        Firm::hard(IntervalRequest::Count(5)),
        Firm::soft(IntervalRequest::Count(3)),
    );
    for edge_first in [false, true] {
        assert_eq!(
            cascaded(RequestCascade::Firmness, face, edge, edge_first),
            (5, Firmness::Hard)
        );
    }
}

#[test]
fn firmness_cascade_lets_the_latest_equally_firm_command_win() {
    let (face, edge) = (
        Firm::soft(IntervalRequest::Count(5)),
        Firm::soft(IntervalRequest::Count(3)),
    );
    // Edge first, then face: the face is the latest command.
    assert_eq!(
        cascaded(RequestCascade::Firmness, face, edge, true),
        (5, Firmness::Soft)
    );
    // Face first, then edge: the edge is the latest command.
    assert_eq!(
        cascaded(RequestCascade::Firmness, face, edge, false),
        (3, Firmness::Soft)
    );
    // The level rule ignores the order.
    assert_eq!(
        cascaded(RequestCascade::Level, face, edge, true),
        (3, Firmness::Soft)
    );
}

#[test]
fn command_order_is_not_serialized_and_does_not_affect_equality() {
    let (_, ids, _) = brick_setup([1.0; 3]);
    let mut a = MeshIntent::default();
    a.set_intervals(ids.edges[0], Firm::soft(IntervalRequest::Count(2)));
    a.set_intervals(ids.edges[1], Firm::soft(IntervalRequest::Count(2)));
    let json = serde_json::to_string(&a).expect("ser");
    assert!(!json.contains("seq"), "{json}");
    let b: MeshIntent = serde_json::from_str(&json).expect("de");
    assert_eq!(a, b);
}

#[test]
fn mesh_from_plan_equals_mesh_and_enforces_the_budget() {
    let (topo, ids, model) = brick_setup([2.0; 3]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    for &e in &ids.edges {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(3)));
    }
    let mut driver = Driver::new(&reg, &solver, &picker);
    let plan = driver.plan(&input, &intent).expect("plan");
    let est = Driver::estimate_cells(&plan, &topo, &sel);
    assert!((est - 27.0).abs() < 1e-9, "{est}");

    let mut a0 = SerialAlloc::new();
    let direct = driver
        .mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut a0)
        .expect("mesh");
    let mut a1 = SerialAlloc::new();
    let once = driver
        .mesh_from_plan(
            &input,
            &intent,
            plan.clone(),
            &Sequential,
            ReferenceBuilder::new(),
            &mut a1,
        )
        .expect("mesh_from_plan");
    assert_eq!(direct.plan, once.plan);
    assert_eq!(direct.attempts, once.attempts);
    assert_eq!(direct.finished.mesh.cell_count(), 27);
    assert_eq!(once.finished.mesh.cell_count(), 27);

    // The budget holds on the plan handed in, and passes at exactly the estimate.
    driver.options = DriverOptions::new().with_max_cells(26);
    let mut a2 = SerialAlloc::new();
    let e = driver.mesh_from_plan(
        &input,
        &intent,
        plan.clone(),
        &Sequential,
        ReferenceBuilder::new(),
        &mut a2,
    );
    assert!(matches!(e, Err(SchemeError::OverBudget { max: 26, .. })));
    driver.options = DriverOptions::new().with_max_cells(27);
    let mut a3 = SerialAlloc::new();
    driver
        .mesh_from_plan(&input, &intent, plan, &Sequential, ReferenceBuilder::new(), &mut a3)
        .expect("within budget");
}
