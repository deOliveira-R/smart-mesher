//! Pave (even loop sum) with a fallback to triangles, driven by interval infeasibility (D7),
//! by the intent chain and by the registry fallback chain (G3).

mod stub_schemes;

use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{CellIx, MeshEntityKind, MeshRead, SerialAlloc, cell_volume, validate};
use sm_scheme_api::{
    Attempt, Driver, FailureKind, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, Phase,
    ReferenceCornerPicker, ReferenceIntervalSolver, Rejections, SchemeChoice, SchemeDim,
    SchemeError, Selection, Sequential, Target, WholeFaces,
};
use sm_topo_api::fixtures::BrickIds;
use stub_schemes::{brick_setup, registry};

fn sweep_z(ids: &BrickIds) -> SchemeChoice {
    SchemeChoice::with_params(
        "sweep",
        serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }),
        Firmness::Hard,
    )
}

/// Loop of the -z face: x-edges 0 and 1 and y-edges 4 and 5 (the sweep leaves them
/// independent because the source is not mapped). Counts 3 + 2 + 2 + 2 = 9: odd.
fn odd_loop(ids: &BrickIds, firm: fn(IntervalRequest) -> Firm<IntervalRequest>) -> MeshIntent {
    let mut intent = MeshIntent::default();
    for (e, n) in [(0, 3), (1, 2), (4, 2), (5, 2)] {
        intent.set_intervals(ids.edges[e], firm(IntervalRequest::Count(n)));
    }
    intent.set_scheme(ids.volume, sweep_z(ids));
    intent.set_scheme(ids.faces[4], SchemeChoice::new("pave", Firmness::Soft));
    intent
}

fn loop_sum(plan: &sm_scheme_api::Plan, ids: &BrickIds) -> u32 {
    [0, 1, 4, 5]
        .iter()
        .map(|e| plan.intervals(ids.edges[*e]).expect("count"))
        .sum()
}

#[test]
fn soft_counts_are_repaired_to_an_even_sum_and_pave_meshes_the_source() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let intent = odd_loop(&ids, Firm::soft);
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    let tp = out
        .plan
        .target(Target::Surface(ids.faces[4]))
        .expect("source");
    assert_eq!(tp.scheme.as_str(), "pave");
    assert_eq!(
        loop_sum(&out.plan, &ids) % 2,
        0,
        "pave's EvenSum was honoured"
    );
    // N/2 quads in the fan, one layer of hexes, unit volume.
    let n = loop_sum(&out.plan, &ids) as usize;
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), n / 2);
    let vol: f64 = (0..n / 2).map(|c| cell_volume(m, CellIx(c as u32))).sum();
    assert!((vol - 1.0).abs() < 1e-12, "volume {vol}");
}

#[test]
fn hard_odd_counts_blame_pave_and_the_driver_falls_back_to_tri() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = odd_loop(&ids, Firm::hard);
    intent.entity_mut(ids.faces[4]).fallbacks = vec![SchemeChoice::new("tri", Firmness::Default)];
    // Planning alone reports the blame: the even-sum constraint is pave's.
    let face = Target::Surface(ids.faces[4]);
    match driver.plan(&input, &intent) {
        Err(SchemeError::Infeasible(inf)) => assert_eq!(
            inf.blame.as_ref().map(|b| (b.0, b.1.as_str())),
            Some((face, "pave")),
            "{inf:?}"
        ),
        other => panic!("expected Infeasible, got {:?}", other.map(|_| ())),
    }
    // `mesh` rejects it and re-plans to the next choice of the chain.
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    let tp = out.plan.target(face).expect("source");
    assert_eq!(
        (tp.scheme.as_str(), &tp.selection),
        ("tri", &Selection::Intent { rank: 1 })
    );
    assert_eq!(
        loop_sum(&out.plan, &ids),
        9,
        "no parity constraint any more"
    );
    let a: &Attempt = out.attempts.first().expect("history");
    assert_eq!(
        (a.target, a.scheme.as_str(), a.phase, a.kind),
        (face, "pave", Phase::Intervals, FailureKind::Constraint)
    );
    assert!(out.plan.rejections.contains(face, &a.scheme));
    // Nine triangles in the fan, nine wedges, unit volume.
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 9);
    let vol: f64 = (0..9).map(|c| cell_volume(m, CellIx(c))).sum();
    assert!((vol - 1.0).abs() < 1e-12, "volume {vol}");
}

#[test]
fn registry_fallback_chains_follow_the_last_intent_choice() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let mut reg = registry();
    assert!(
        reg.set_fallback(SchemeDim::Surface, "pave", &["nope"])
            .is_err()
    );
    reg.set_fallback(SchemeDim::Surface, "pave", &["tri"])
        .expect("chain");
    assert_eq!(
        reg.fallbacks(SchemeDim::Surface, "pave")
            .iter()
            .map(|n| n.as_str())
            .collect::<Vec<_>>(),
        vec!["tri"]
    );
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let intent = odd_loop(&ids, Firm::hard);
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    let tp = out
        .plan
        .target(Target::Surface(ids.faces[4]))
        .expect("source");
    assert_eq!(
        (tp.scheme.as_str(), &tp.selection),
        ("tri", &Selection::Registry { rank: 0 })
    );
    // Replaying (intent, stored rejections) gives the same plan without a failed attempt.
    let replay = driver
        .plan_with(&input, &intent, &out.plan.rejections)
        .expect("replay");
    assert_eq!(replay, out.plan);
    assert!(
        driver
            .plan_with(&input, &intent, &Rejections::default())
            .is_err()
    );
}

#[test]
fn registry_auto_lists_and_intent_auto_lists_override_the_default() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let mut reg = registry();
    reg.set_auto(SchemeDim::Surface, &["tri", "pave"])
        .expect("auto");
    assert!(reg.set_auto(SchemeDim::Surface, &["nope"]).is_err());
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    intent.set_scheme(ids.volume, sweep_z(&ids));
    // The registry list restricts auto selection of the (unimposed) source face to tri/pave: pave scores higher.
    let plan = driver.plan(&input, &intent).expect("plan");
    assert_eq!(
        plan.target(Target::Surface(ids.faces[4]))
            .map(|t| t.scheme.as_str()),
        Some("pave")
    );
    // The intent's list wins over the registry's.
    intent.defaults.auto.surface = vec![sm_scheme_api::SchemeName::new("tri")];
    let plan = driver.plan(&input, &intent).expect("plan");
    let tp = plan.target(Target::Surface(ids.faces[4])).expect("source");
    assert_eq!(
        (tp.scheme.as_str(), &tp.selection),
        ("tri", &Selection::Auto { score: 20 })
    );
}
