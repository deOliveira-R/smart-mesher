//! W0f gaps4: `Requirements::reads` (a curve scheme reading a sibling curve's patch).

use sm_scheme_api::stubs as stub_schemes;

use sm_core::{EntityId, EntityKind};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{SerialAlloc, validate};
use sm_scheme_api::{
    Applicability, Caps, Driver, Firm, Firmness, GenCtx, GenerateFailure, IntervalRequest, Job, MeshConstraint, MeshInput, MeshIntent, Params, Patch, PlanCtx,
    ReferenceCornerPicker, ReferenceIntervalSolver, Requirements, Scheme, SchemeChoice, SchemeDim, SchemeError, SchemeRegistry, Sequential, Target, WholeFaces,
};
use stub_schemes::{Equal, brick_setup, registry};

/// Curve copy by count: reads the patch of `source` and repeats its node count. Param
/// `bad` selects a refused read (`self`, `parent`, `outside`).
struct ReadCopy;

impl Scheme for ReadCopy {
    fn name(&self) -> &str {
        "read_copy"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Curve
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::NONE
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(1, "test")
    }
    fn requirements(&self, cx: &PlanCtx<'_>, target: Target, p: &Params, out: &mut Requirements<'_>) -> sm_scheme_api::Result<()> {
        let bad = p.get::<String>("read_copy", "bad")?;
        let other = match bad.as_deref() {
            Some("self") => target,
            Some("parent") => {
                let Target::Curve(e) = target else { unreachable!() };
                let f = cx.input.topo.coboundary(e).iter().map(|i| i.cell).find(|c| c.kind() == EntityKind::TopoFace).unwrap();
                Target::Surface(f)
            }
            Some("outside") => Target::Curve(EntityId::new(EntityKind::TopoEdge, 999).unwrap()),
            _ => Target::Curve(p.get::<EntityId>("read_copy", "source")?.expect("source")),
        };
        out.reads(other);
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, p: &Params) -> Result<Patch, GenerateFailure> {
        let Target::Curve(e) = target else { unreachable!() };
        let source = p.get::<EntityId>("read_copy", "source").ok().flatten().expect("source");
        // Guaranteed by `reads`: the source patch exists already.
        let src = cx.patches.get(Target::Curve(source)).ok_or_else(|| GenerateFailure::fatal(target, "read_copy", "source patch missing"))?;
        let n = src.nodes.len() as u32 + 1;
        let fr: Vec<f64> = (1..n).map(|i| f64::from(i) / f64::from(n)).collect();
        cx.curve_patch(e, &fr)
    }
}

fn reg() -> SchemeRegistry {
    let mut r = registry();
    r.register(Box::new(ReadCopy)).unwrap();
    let _ = Equal;
    r
}

fn choice(params: serde_json::Value, f: Firmness) -> SchemeChoice {
    SchemeChoice::with_params("read_copy", params, f)
}

#[test]
fn a_curve_scheme_reads_a_sibling_curve_patch() {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let reg = reg();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    for (e, n) in [(0, 2u32), (4, 3), (8, 4)] {
        intent.set_intervals(ids.edges[e], Firm::hard(IntervalRequest::Count(n)));
    }
    intent.set_scheme(ids.volume, SchemeChoice::with_params("sweep", serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }), Firmness::Hard));
    // x-edge 0 copies x-edge 1 (opposite sides of the -z face).
    let (dst, src) = (ids.edges[0], ids.edges[1]);
    intent.set_scheme(dst, choice(serde_json::json!({ "source": src }), Firmness::Hard));
    let plan = driver.plan(&input, &intent).expect("plan");
    let read = plan.problem.constraints.iter().find(|r| matches!(r.item, MeshConstraint::Precedes { before, after } if before == Target::Curve(src) && after == Target::Curve(dst)));
    assert!(read.is_some(), "a scheme-origin Precedes is recorded");
    let wave = |t: Target| plan.waves.iter().position(|w| w.contains(&Job::Generate(t))).unwrap();
    assert!(wave(Target::Curve(src)) < wave(Target::Curve(dst)));
    let mut alloc = SerialAlloc::new();
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut alloc).expect("mesh");
    assert_eq!(out.plan.target(Target::Curve(dst)).unwrap().scheme.as_str(), "read_copy");
    assert!(validate(&out.finished.mesh).is_empty());
}

#[test]
fn reads_are_refused_for_self_ancestors_and_targets_outside_the_region() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = reg();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    for bad in ["self", "parent", "outside"] {
        let mut intent = MeshIntent::default();
        intent.set_scheme(ids.edges[0], choice(serde_json::json!({ "bad": bad, "source": ids.edges[1] }), Firmness::Hard));
        let e = driver.plan(&input, &intent).err().unwrap_or_else(|| panic!("{bad} accepted"));
        assert!(matches!(e, SchemeError::BadRequirement { .. }), "{bad}: {e:?}");
    }
}

#[test]
fn a_read_cycle_blames_the_reading_scheme_and_falls_back() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = reg();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let (a, b) = (ids.edges[0], ids.edges[1]);
    let mut intent = MeshIntent::default();
    intent.set_scheme(a, choice(serde_json::json!({ "source": b }), Firmness::Soft));
    intent.set_scheme(b, choice(serde_json::json!({ "source": a }), Firmness::Soft));
    // Planning alone reports the cycle as Infeasible with a blame.
    let Err(e) = driver.plan(&input, &intent) else { panic!("cycle accepted") };
    let blame = e.blame().expect("blamed");
    assert_eq!(blame.1.as_str(), "read_copy");
    assert!(matches!(e, SchemeError::Infeasible(_)), "{e:?}");
    // The driver rejects it and re-plans: one of the two falls back to another curve scheme.
    let mut alloc = SerialAlloc::new();
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut alloc).expect("mesh after fallback");
    assert!(out.attempts.iter().any(|t| t.scheme.as_str() == "read_copy"));
    let schemes: Vec<&str> = [a, b].iter().map(|e| out.plan.target(Target::Curve(*e)).unwrap().scheme.as_str()).collect();
    assert!(schemes.contains(&"read_copy") && schemes.iter().any(|s| *s != "read_copy"), "{schemes:?}");
    // Hard choices are not blameable: the cycle stays a schedule error.
    let mut hard = MeshIntent::default();
    hard.set_scheme(a, choice(serde_json::json!({ "source": b }), Firmness::Hard));
    hard.set_scheme(b, choice(serde_json::json!({ "source": a }), Firmness::Hard));
    assert!(matches!(driver.plan(&input, &hard), Err(SchemeError::Schedule { .. })));
}
