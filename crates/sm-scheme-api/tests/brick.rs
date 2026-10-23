//! End-to-end: the topology brick fixture meshed by `equal` + `map` + 1-to-1 `sweep` through the
//! reference driver, written into the reference `MeshBuild`, checked against analytic values.

mod common;

use common::{Broken, registry, sweep_params};
use sm_core::{EntityId, Point3, Vector3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{MeshEntityKind, MeshRead, SerialAlloc, cell_volume, geometry_hash, topology_hash, validate};
use sm_model_api::{BoundaryKind, Deck, Model};
use sm_scheme_api::{
    DriverError, Executor, Firmness, IntervalError, IntervalSpec, JobKind, MeshIntent, MeshPatch, ProblemInput, ReferenceIntervalSolver, Requirement, RunOutput, SchemeChoice, SchemeFailure, SchemeRegistry, SchemeSpec, SequentialExecutor,
    SweepRole, Target, VertexType, run,
};
use sm_topo_api::fixtures::{BrickIds, brick, with_op};
use sm_topo_api::{CellComplex, OpId};
use std::sync::Arc;
use std::sync::Mutex;

const SIZE: [f64; 3] = [2.0, 1.0, 1.0];

fn setup() -> (CellComplex, BrickIds) {
    let mut cc = CellComplex::new();
    let ids = with_op(&mut cc, OpId(1), |t| brick(t, Point3::new(0.0, 0.0, 0.0), SIZE)).expect("brick");
    (cc, ids)
}

fn sweep_intent(ids: &BrickIds) -> MeshIntent {
    let mut intent = MeshIntent { mesh: vec![Target::Topo(ids.volume)], default_size: Some(0.5), ..MeshIntent::default() };
    // -z is the source, +z the target. The source face scheme is left to auto selection.
    intent.set_scheme(Target::Topo(ids.volume), SchemeChoice::Fixed(sweep_params(ids.faces[4], ids.faces[5])), Firmness::Hard);
    intent
}

fn mesh_it(cc: &CellComplex, model: &Model, intent: &MeshIntent, reg: &SchemeRegistry, exec: &dyn Executor) -> Result<RunOutput, DriverError> {
    let inp = ProblemInput { topo: cc, model, intent, registry: reg, size: None };
    run(&inp, &ReferenceIntervalSolver, exec, 4)
}

fn write(out: &RunOutput, model: &Model) -> ReferenceMesh {
    let mut b = ReferenceBuilder::new();
    out.ledger.write_mesh(&mut b, model).expect("write");
    let mut alloc = SerialAlloc::new();
    sm_mesh_api::MeshBuild::finish(b, &mut alloc).expect("finish").mesh
}

/// Runs each job on its own scoped thread (results returned in index order).
struct Threaded;

impl Executor for Threaded {
    fn run_all(&self, n: usize, task: &(dyn Fn(usize) -> Result<MeshPatch, SchemeFailure> + Sync)) -> Vec<Result<MeshPatch, SchemeFailure>> {
        let slots: Vec<Mutex<Option<Result<MeshPatch, SchemeFailure>>>> = (0..n).map(|_| Mutex::new(None)).collect();
        std::thread::scope(|s| {
            for (i, slot) in slots.iter().enumerate() {
                s.spawn(move || {
                    let r = task(i);
                    *slot.lock().expect("slot") = Some(r);
                });
            }
        });
        slots.into_iter().map(|m| m.into_inner().expect("slot").expect("ran")).collect()
    }
}

#[test]
fn sweep_brick_end_to_end() {
    let (cc, ids) = setup();
    let model = Model::new();
    let reg = registry();
    let out = mesh_it(&cc, &model, &sweep_intent(&ids), &reg, &SequentialExecutor).expect("mesh");
    let p = &out.problem;

    // Jobs: 12 equal curves, 1 map (source, auto-selected), 1 sweep writing volume + target + 4 linking.
    let schemes: Vec<&str> = p.jobs.iter().map(|j| j.scheme().map_or("copy", |s| s.name.as_str())).collect();
    assert_eq!(schemes.iter().filter(|s| **s == "equal").count(), 12);
    assert_eq!(schemes.iter().filter(|s| **s == "map").count(), 1);
    assert_eq!(schemes.iter().filter(|s| **s == "sweep").count(), 1);
    let sweep = p.jobs.iter().find(|j| j.scheme().is_some_and(|s| s.name.as_str() == "sweep")).expect("sweep job");
    assert_eq!(sweep.footprint.writes.len(), 6, "volume + target + 4 linking faces");
    assert_eq!(sweep.roles[&Target::Topo(ids.faces[4])], SweepRole::Source);
    assert_eq!(sweep.roles[&Target::Topo(ids.faces[5])], SweepRole::Target);
    assert_eq!(sweep.roles.values().filter(|r| **r == SweepRole::Linking).count(), 4);
    // The sweep imposed the source role on -z.
    assert!(p.impositions[&Target::Topo(ids.faces[4])].iter().any(|i| i.requirement == Requirement::Role(SweepRole::Source)));

    // Schedule: curves, then the source map, then the sweep.
    assert_eq!(out.schedule.waves.len(), 3);
    assert_eq!(out.schedule.waves[0].len(), 12);
    assert_eq!(out.schedule.waves[1].len(), 1);
    assert_eq!(out.schedule.waves[2].len(), 1);

    // Intervals: size 0.5 gives 4 x 2 x 2, consistent with the map and sweep constraints.
    for (k, e) in ids.edges.iter().enumerate() {
        let want = if k < 4 { 4 } else { 2 };
        assert_eq!(out.intervals.get(*e), Some(want), "edge {k}");
    }

    // Vertex types: the map picked End (soft) at the four corners of -z; sweep typed linking faces.
    let ends_on = |f: EntityId| p.vertex_types.iter().filter(|(k, s)| k.face == f && s.ty == VertexType::End).count();
    assert_eq!(ends_on(ids.faces[4]), 4);
    for f in [ids.faces[0], ids.faces[1], ids.faces[2], ids.faces[3]] {
        assert_eq!(ends_on(f), 4);
    }

    // Ledger: 8 source quads, 8 target quads, 16 hexes.
    assert_eq!(out.ledger.face(&Target::Topo(ids.faces[4])).map(|f| f.len()), Some(8));
    assert_eq!(out.ledger.face(&Target::Topo(ids.faces[5])).map(|f| f.len()), Some(8));
    assert_eq!(out.ledger.cells(&Target::Topo(ids.volume)).map(|c| c.len()), Some(16));

    // Mesh database: analytic counts and volume, valid, classified.
    let mesh = write(&out, &model);
    assert!(validate(&mesh).is_empty(), "{:?}", validate(&mesh));
    assert_eq!(mesh.count(MeshEntityKind::Cell), 16);
    assert_eq!(mesh.count(MeshEntityKind::Vertex), 5 * 3 * 3);
    let mut total = 0.0;
    for c in 0..16u32 {
        let v = cell_volume(&mesh, sm_mesh_api::CellIx(c));
        assert!(v > 0.0, "cell {c} inverted");
        assert!((v - 0.125).abs() < 1e-12);
        total += v;
    }
    assert!((total - SIZE[0] * SIZE[1] * SIZE[2]).abs() < 1e-12);
    // Boundary faces on each model face: -x/+x 2x2, -y/+y 4x2, -z/+z 4x2.
    let on = |f: EntityId| mesh.classified_on(f).map_or(0, |s| s.faces.len());
    assert_eq!([on(ids.faces[0]), on(ids.faces[1]), on(ids.faces[2]), on(ids.faces[3]), on(ids.faces[4]), on(ids.faces[5])], [4, 4, 8, 8, 8, 8]);
    assert_eq!(mesh.classified_on(ids.volume).map(|s| s.cells.len()), Some(16));
    // Nodes on model edges keep their curve parameter.
    let edge_nodes = mesh.classified_on(ids.edges[0]).map_or(0, |s| s.vertices.len());
    assert_eq!(edge_nodes, 3);
}

#[test]
fn determinism_runs_and_threads() {
    let (cc, ids) = setup();
    let model = Model::new();
    let reg = registry();
    let intent = sweep_intent(&ids);
    let a = mesh_it(&cc, &model, &intent, &reg, &SequentialExecutor).expect("a");
    let b = mesh_it(&cc, &model, &intent, &reg, &SequentialExecutor).expect("b");
    let c = mesh_it(&cc, &model, &intent, &reg, &Threaded).expect("c");
    assert_eq!(a.ledger.hash(), b.ledger.hash());
    assert_eq!(a.ledger.hash(), c.ledger.hash());
    assert_eq!(a.schedule, c.schedule);
    assert_eq!(a.problem, c.problem);
    let (ma, mc) = (write(&a, &model), write(&c, &model));
    assert_eq!(topology_hash(&ma), topology_hash(&mc));
    assert_eq!(geometry_hash(&ma), geometry_hash(&mc));
    // Also from a fresh topology build on another thread.
    let h = std::thread::spawn(|| {
        let (cc, ids) = setup();
        let model = Model::new();
        let out = mesh_it(&cc, &model, &sweep_intent(&ids), &registry(), &Threaded).expect("t");
        out.ledger.hash()
    })
    .join()
    .expect("thread");
    assert_eq!(h, a.ledger.hash());
}

#[test]
fn auto_selection_falls_back_after_execution_failure() {
    let (cc, ids) = setup();
    let model = Model::new();
    let mut reg = registry();
    reg.register(Arc::new(Broken));
    let base = mesh_it(&cc, &model, &sweep_intent(&ids), &registry(), &SequentialExecutor).expect("base");
    let out = mesh_it(&cc, &model, &sweep_intent(&ids), &reg, &SequentialExecutor).expect("fallback");
    // Auto picked `broken` (score 1000), it failed with NextInChain, the driver excluded it and re-planned.
    assert_eq!(out.intent.excluded[&Target::Topo(ids.faces[4])], vec![sm_scheme_api::SchemeName::new("broken")]);
    assert_eq!(out.ledger.hash(), base.ledger.hash());
}

#[test]
fn chain_falls_back_in_order() {
    let (cc, ids) = setup();
    let model = Model::new();
    let mut reg = registry();
    reg.register(Arc::new(Broken));
    let mut intent = sweep_intent(&ids);
    intent.set_scheme(Target::Topo(ids.faces[4]), SchemeChoice::Chain(vec![SchemeSpec::named("broken"), SchemeSpec::named("map")]), Firmness::Hard);
    let out = mesh_it(&cc, &model, &intent, &reg, &SequentialExecutor).expect("chain");
    let src = out.problem.jobs.iter().find(|j| j.targets == [Target::Topo(ids.faces[4])]).expect("source job");
    assert_eq!(src.scheme().map(|s| s.name.as_str()), Some("map"));
}

#[test]
fn hard_interval_conflict_is_reported() {
    let (cc, ids) = setup();
    let model = Model::new();
    let mut intent = sweep_intent(&ids);
    // The two x-edges of the -z face are opposite map sides: 3 vs 5 cannot both hold.
    intent.set_intervals(Target::Topo(ids.edges[0]), IntervalSpec::Count(3), Firmness::Hard);
    intent.set_intervals(Target::Topo(ids.edges[1]), IntervalSpec::Count(5), Firmness::Hard);
    let err = mesh_it(&cc, &model, &intent, &registry(), &SequentialExecutor).expect_err("conflict");
    assert!(matches!(err, DriverError::Intervals(IntervalError::HardConflict(_))), "{err:?}");
}

#[test]
fn unknown_scheme_and_not_applicable() {
    let (cc, ids) = setup();
    let model = Model::new();
    let mut intent = sweep_intent(&ids);
    intent.set_scheme(Target::Topo(ids.faces[4]), SchemeChoice::fixed("nope"), Firmness::Hard);
    let err = mesh_it(&cc, &model, &intent, &registry(), &SequentialExecutor).expect_err("unknown");
    assert!(matches!(err, DriverError::UnknownScheme(_)));
    // Sweep without source/target parameters is not applicable, and nothing else meshes volumes.
    let mut intent = sweep_intent(&ids);
    intent.set_scheme(Target::Topo(ids.volume), SchemeChoice::fixed("sweep"), Firmness::Hard);
    let err = mesh_it(&cc, &model, &intent, &registry(), &SequentialExecutor).expect_err("n/a");
    match err {
        DriverError::NoApplicableScheme { attempts, .. } => assert!(attempts[0].reason.contains("source")),
        e => panic!("{e:?}"),
    }
}

fn periodic_x(ids: &BrickIds) -> Model {
    let mut model = Model::new();
    let b = model.add_boundary("periodic-x", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(SIZE[0], 0.0, 0.0) })).expect("boundary");
    model.assign_pairs(b, &[(ids.faces[0], ids.faces[1])]).expect("pair");
    model
}

#[test]
fn deck_pair_becomes_a_copy_job() {
    let (cc, ids) = setup();
    let model = periodic_x(&ids);
    let intent = MeshIntent { mesh: vec![Target::Topo(ids.faces[0]), Target::Topo(ids.faces[1])], default_size: Some(0.25), ..MeshIntent::default() };
    let out = mesh_it(&cc, &model, &intent, &registry(), &SequentialExecutor).expect("mesh");
    let copy = out.problem.jobs.iter().find(|j| matches!(j.kind, JobKind::Copy(_))).expect("copy job");
    assert!(copy.footprint.writes.contains(&Target::Topo(ids.faces[1])));
    assert_eq!(copy.footprint.writes.len(), 5, "+x face and its four edges");
    // Both deck faces carry the no-boundary-layer rule; B carries the copy requirement.
    for f in [ids.faces[0], ids.faces[1]] {
        assert!(out.problem.impositions[&Target::Topo(f)].iter().any(|i| i.requirement == Requirement::NoBoundaryLayer));
    }
    assert!(out.problem.impositions[&Target::Topo(ids.faces[1])].iter().any(|i| matches!(i.requirement, Requirement::CopyOf { .. })));
    let t = sm_core::Transform::translation(Vector3::new(SIZE[0], 0.0, 0.0));
    assert!(sm_scheme_api::copy::verify_copy(&out.ledger, &Target::Topo(ids.faces[0]), &Target::Topo(ids.faces[1]), &t, 1e-6));
    assert_eq!(out.ledger.face(&Target::Topo(ids.faces[1])).map(|f| f.len()), Some(16));
}

#[test]
fn deck_pair_written_by_sweep_is_verified() {
    let (cc, ids) = setup();
    let model = periodic_x(&ids);
    let out = mesh_it(&cc, &model, &sweep_intent(&ids), &registry(), &SequentialExecutor).expect("mesh");
    assert_eq!(out.problem.deck_checks.len(), 1);
    assert!(out.problem.jobs.iter().all(|j| !matches!(j.kind, JobKind::Copy(_))));
}

#[cfg(feature = "serde")]
#[test]
fn intent_is_serde() {
    fn is_serde<T: serde::Serialize + serde::de::DeserializeOwned>() {}
    is_serde::<MeshIntent>();
}

#[test]
fn transform_runs_after_writers_and_replaces_atomically() {
    let (cc, ids) = setup();
    let model = Model::new();
    let mut reg = registry();
    reg.register(Arc::new(common::NoopTransform));
    let base = mesh_it(&cc, &model, &sweep_intent(&ids), &registry(), &SequentialExecutor).expect("base");
    let mut intent = sweep_intent(&ids);
    intent.transforms.push(sm_scheme_api::TransformIntent { scheme: SchemeSpec::named("noop-transform"), targets: vec![Target::Topo(ids.volume)] });
    let out = mesh_it(&cc, &model, &intent, &reg, &SequentialExecutor).expect("transform");
    assert_eq!(out.schedule.waves.len(), 4);
    let last = out.schedule.waves[3][0];
    assert!(matches!(out.problem.jobs[last.0 as usize].kind, JobKind::Transform(_)));
    assert_eq!(out.ledger.writer(&Target::Topo(ids.volume)), Some(last));
    assert_eq!(topology_hash(&write(&out, &model)), topology_hash(&write(&base, &model)));
}
