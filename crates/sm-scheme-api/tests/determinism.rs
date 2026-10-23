//! Determinism: the full pipeline twice, with the sequential and the scoped-thread executor,
//! and on four `std` threads at once, must give identical plans and meshes (API crates do not
//! use rayon; decisions.md W0c 8). Covers the deck copy classes, the pave-to-tri re-plan and
//! the claiming group sweep.

mod stub_schemes;

use sm_core::{EntityKind, Vector3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{SerialAlloc, geometry_hash, topology_hash};
use sm_model_api::{BoundaryKind, Deck};
use sm_scheme_api::{Driver, Executor, Firm, Firmness, GroupId, IntervalRequest, MeshGroup, MeshInput, MeshIntent, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice, ScopedThreads, Sequential, SizingSpec, WholeFaces};
use stub_schemes::{brick_setup, registry, row_setup};

type Hashes = (u64, u64, String);

fn run(exec: &dyn Executor) -> Hashes {
    let (topo, ids, mut model) = brick_setup([1.0, 1.5, 2.0]);
    let b = model.add_boundary("periodic-x", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).expect("deck");
    model.assign_pairs(b, &[(ids.faces[0], ids.faces[1])]).expect("pair");
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Constant { size: 0.25 });
    intent.set_intervals(ids.edges[8], Firm::soft(IntervalRequest::Count(5)));
    intent.set_scheme(ids.volume, SchemeChoice::with_params("sweep", serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }), Firmness::Hard));
    let out = driver.mesh(&input, &intent, exec, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh");
    let m = &out.finished.mesh;
    (topology_hash(m), geometry_hash(m), serde_json::to_string(&out.plan).expect("plan json"))
}

/// Pave fails on an odd hard loop and the driver re-plans to the triangle fan.
fn run_replan(exec: &dyn Executor) -> Hashes {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = MeshIntent::default();
    for (e, n) in [(0, 3), (1, 2), (4, 2), (5, 2)] {
        intent.set_intervals(ids.edges[e], Firm::hard(IntervalRequest::Count(n)));
    }
    intent.set_scheme(ids.volume, SchemeChoice::with_params("sweep", serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }), Firmness::Hard));
    intent.set_scheme(ids.faces[4], SchemeChoice::new("pave", Firmness::Soft));
    intent.entity_mut(ids.faces[4]).fallbacks = vec![SchemeChoice::new("tri", Firmness::Default)];
    let out = driver.mesh(&input, &intent, exec, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh");
    let m = &out.finished.mesh;
    (topology_hash(m), geometry_hash(m), serde_json::to_string(&out.plan).expect("plan json"))
}

/// A group of three stacked bricks swept as one; the group claims the interfaces and the target.
fn run_group(exec: &dyn Executor) -> Hashes {
    let (topo, ids, model) = row_setup(3);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel: Vec<_> = ids.iter().map(|b| b.volume).collect();
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Constant { size: 0.5 });
    let params = serde_json::json!({ "source": [ids[0].faces[0]], "target": [ids[2].faces[1]] });
    intent.groups.push(MeshGroup { id: GroupId(1), members: sel.clone(), scheme: Some(SchemeChoice::with_params("msweep", params, Firmness::Hard)), fallbacks: Vec::new(), conversion: None });
    let out = driver.mesh(&input, &intent, exec, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh");
    let m = &out.finished.mesh;
    (topology_hash(m), geometry_hash(m), serde_json::to_string(&out.plan).expect("plan json"))
}

fn check(f: fn(&dyn Executor) -> Hashes) {
    let a = f(&Sequential);
    let b = f(&Sequential);
    let c = f(&ScopedThreads(4));
    assert_eq!(a, b);
    assert_eq!(a, c);
    let threads: Vec<_> = (0..4).map(|i| std::thread::spawn(move || if i % 2 == 0 { f(&Sequential) } else { f(&ScopedThreads(3)) })).collect();
    for t in threads {
        assert_eq!(t.join().expect("thread"), a);
    }
}

#[test]
fn identical_across_runs_executors_and_threads() {
    check(run);
}

#[test]
fn replanned_mesh_is_identical_across_runs_executors_and_threads() {
    check(run_replan);
}

#[test]
fn group_sweep_is_identical_across_runs_executors_and_threads() {
    check(run_group);
}

#[test]
fn sizing_goals_and_soft_counts_resolve_as_expected() {
    let (_, _, plan_json) = run(&Sequential);
    let plan: sm_scheme_api::Plan = serde_json::from_str(&plan_json).expect("plan");
    // Size 0.25: x edges 4, y edges 6; z edges soft 5 (overrides the 8 the size would give).
    let mut counts: Vec<u32> = plan.intervals.iter().map(|x| x.1).collect();
    counts.sort_unstable();
    assert_eq!(counts, vec![4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6]);
    let _ = EntityKind::TopoEdge;
}
