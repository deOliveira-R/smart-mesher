//! Determinism: the full pipeline twice, with the sequential and the scoped-thread executor,
//! and on four `std` threads at once, must give identical plans and meshes (API crates do not
//! use rayon; decisions.md W0c 8).

mod stub_schemes;

use sm_core::Vector3;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{SerialAlloc, geometry_hash, topology_hash};
use sm_model_api::{BoundaryKind, Deck};
use sm_scheme_api::{Driver, Executor, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeChoice, ScopedThreads, Sequential, SizingSpec, WholeFaces};
use stub_schemes::{brick_setup, registry};

fn run<E: Executor>(exec: &E) -> (u64, u64, String) {
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

#[test]
fn identical_across_runs_executors_and_threads() {
    let a = run(&Sequential);
    let b = run(&Sequential);
    let c = run(&ScopedThreads(4));
    assert_eq!(a, b);
    assert_eq!(a, c);
    let threads: Vec<_> = (0..4).map(|i| std::thread::spawn(move || if i % 2 == 0 { run(&Sequential) } else { run(&ScopedThreads(3)) })).collect();
    for t in threads {
        assert_eq!(t.join().expect("thread"), a);
    }
}

#[test]
fn sizing_goals_and_soft_counts_resolve_as_expected() {
    let (_, _, plan_json) = run(&Sequential);
    let plan: sm_scheme_api::Plan = serde_json::from_str(&plan_json).expect("plan");
    // Size 0.25: x edges 4, y edges 6; z edges soft 5 (overrides the 8 the size would give).
    let mut counts: Vec<u32> = plan.intervals.iter().map(|x| x.1).collect();
    counts.sort_unstable();
    assert_eq!(counts, vec![4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6]);
}
