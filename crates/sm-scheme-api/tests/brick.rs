//! End-to-end: equal + map + 1-to-1 sweep on the reference brick, through the driver, into the
//! reference `MeshBuild`. Analytic checks (counts, volumes, coordinates, classification).

mod stub_schemes;

use sm_core::{EntityId, Vector3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{MeshRead, SerialAlloc, cell_volume, validate};
use sm_model_api::{BoundaryKind, Deck};
use sm_scheme_api::{
    Applicability, CopyReason, Driver, ElemShape, Firm, Firmness, GenCtx, GenerateFailure, IntervalConstraint, IntervalRequest, IntervalSolver, MeshConstraint, MeshInput, MeshIntent, Origin, Params, Patch, PlanCtx, Record, ReferenceCornerPicker,
    ReferenceIntervalSolver, Role, Scheme, SchemeChoice, SchemeDim, SchemeError, Selection, Sequential, Target, VertexType, WholeFaces,
};
use stub_schemes::{brick_setup, registry};

const X: usize = 0; // an x-edge
const Y: usize = 4; // a y-edge
const Z: usize = 8; // a z-edge

fn counts_intent(ids: &sm_topo_api::fixtures::BrickIds, n: [u32; 3]) -> MeshIntent {
    let mut intent = MeshIntent::default();
    intent.set_intervals(ids.edges[X], Firm::hard(IntervalRequest::Count(n[0])));
    intent.set_intervals(ids.edges[Y], Firm::hard(IntervalRequest::Count(n[1])));
    intent.set_intervals(ids.edges[Z], Firm::hard(IntervalRequest::Count(n[2])));
    intent
}

fn sweep_z(ids: &sm_topo_api::fixtures::BrickIds) -> SchemeChoice {
    SchemeChoice::with_params("sweep", serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }), Firmness::Hard)
}

#[test]
fn sweep_brick_is_analytic() {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = counts_intent(&ids, [2, 3, 4]);
    intent.set_scheme(ids.volume, sweep_z(&ids));

    let mut alloc = SerialAlloc::new();
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut alloc).expect("mesh");
    let plan = &out.plan;

    // Interval propagation through map sides and sweep layers.
    for (k, want) in [(0..4, 2u32), (4..8, 3), (8..12, 4)] {
        for e in k {
            assert_eq!(plan.intervals(ids.edges[e]), Some(want), "edge {e}");
        }
    }
    // Roles, impositions, copies.
    let src = Target::Surface(ids.faces[4]);
    let tgt = Target::Surface(ids.faces[5]);
    let vol = Target::Volume(ids.volume);
    assert_eq!(plan.roles_of(src).collect::<Vec<_>>(), vec![Role::SweepSource { volume: vol }]);
    assert_eq!(plan.roles_of(tgt).collect::<Vec<_>>(), vec![Role::SweepTarget { volume: vol }]);
    for f in &ids.faces[..4] {
        let tp = plan.target(Target::Surface(*f)).expect("linking plan");
        assert_eq!(tp.scheme.as_str(), "map");
        assert_eq!(tp.selection, Selection::Imposed);
        assert_eq!(plan.roles_of(Target::Surface(*f)).collect::<Vec<_>>(), vec![Role::SweepLinking { volume: vol }]);
    }
    let copy = plan.copy_into(tgt).expect("target is a copy");
    assert!(matches!(copy.item.reason, CopyReason::SweepTarget { .. }));
    assert!(matches!(plan.target(tgt).map(|t| &t.selection), Some(Selection::Copy { .. })));
    // Every map face has four End corners.
    for l in &plan.layouts {
        assert_eq!(l.corners.iter().filter(|c| c.vertex_type == VertexType::End).count(), 4);
        assert_eq!(l.sides.len(), 4);
    }

    // The mesh.
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(sm_mesh_api::MeshEntityKind::Cell), 24);
    assert_eq!(m.count(sm_mesh_api::MeshEntityKind::Vertex), 3 * 4 * 5);
    let total: f64 = (0..24).map(|c| cell_volume(m, sm_mesh_api::CellIx(c))).sum();
    assert!((total - 3.0).abs() < 1e-12, "volume {total}");
    for c in 0..24 {
        assert!((cell_volume(m, sm_mesh_api::CellIx(c)) - 0.125).abs() < 1e-12);
    }
    // Faces per model face: -x/+x 3x4, -y/+y 2x4, -z/+z 2x3.
    for (f, want) in ids.faces.iter().zip([12, 12, 8, 8, 6, 6]) {
        assert_eq!(m.classified_on(*f).map(|s| s.faces.len()), Some(want), "face {f}");
    }
    // Every vertex sits on the 0.5 grid.
    let xyz = m.coords();
    for i in 0..xyz.x.len() {
        for c in [xyz.x[i], xyz.y[i], xyz.z[i]] {
            assert_eq!((c * 2.0).round(), c * 2.0);
        }
    }
    assert!(out.rejections.0.is_empty());
}

#[test]
fn auto_selection_finds_a_sweep() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let intent = counts_intent(&ids, [2, 2, 2]);
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh");
    let vp = out.plan.target(Target::Volume(ids.volume)).expect("volume");
    assert_eq!(vp.scheme.as_str(), "sweep");
    assert!(matches!(vp.selection, Selection::Auto { score: 60 }));
    // Lowest-id opposite pair: -x -> +x.
    assert!(out.plan.copy_into(Target::Surface(ids.faces[1])).is_some());
    assert_eq!(out.finished.mesh.count(sm_mesh_api::MeshEntityKind::Cell), 8);
}

#[test]
fn hard_pave_on_a_linking_face_conflicts_with_the_sweep() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = counts_intent(&ids, [1, 1, 1]);
    intent.set_scheme(ids.volume, sweep_z(&ids));
    intent.set_scheme(ids.faces[1], SchemeChoice::new("pave", Firmness::Hard));
    match driver.plan(&input, &intent) {
        Err(SchemeError::ImpositionConflict { child, imposed_by, required, .. }) => {
            assert_eq!(child, Target::Surface(ids.faces[1]));
            assert_eq!(imposed_by, Target::Volume(ids.volume));
            assert_eq!(required.iter().map(|s| s.as_str()).collect::<Vec<_>>(), vec!["map", "submap"]);
        }
        other => panic!("expected ImpositionConflict, got {:?}", other.map(|_| ())),
    }
    // A soft pave is overridden instead, and recorded.
    intent.set_scheme(ids.faces[1], SchemeChoice::new("pave", Firmness::Soft));
    let plan = driver.plan(&input, &intent).expect("plan");
    let tp = plan.target(Target::Surface(ids.faces[1])).expect("face");
    assert_eq!(tp.scheme.as_str(), "map");
    assert_eq!(tp.rejected[0].0.as_str(), "pave");
    // The sweep target is a copy: a hard pave there conflicts too, a soft one is overridden.
    intent.set_scheme(ids.faces[5], SchemeChoice::new("pave", Firmness::Hard));
    match driver.plan(&input, &intent) {
        Err(SchemeError::ImpositionConflict { child, imposed_by, .. }) => {
            assert_eq!((child, imposed_by), (Target::Surface(ids.faces[5]), Target::Surface(ids.faces[4])));
        }
        other => panic!("expected ImpositionConflict, got {:?}", other.map(|_| ())),
    }
    intent.set_scheme(ids.faces[5], SchemeChoice::new("pave", Firmness::Soft));
    let plan = driver.plan(&input, &intent).expect("plan");
    let tp = plan.target(Target::Surface(ids.faces[5])).expect("target face");
    assert_eq!((tp.scheme.as_str(), tp.rejected[0].0.as_str()), ("copy", "pave"));
}

#[test]
fn hard_counts_that_break_map_sides_are_infeasible() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = counts_intent(&ids, [2, 2, 2]);
    intent.set_intervals(ids.edges[1], Firm::hard(IntervalRequest::Count(3))); // another x-edge
    match driver.plan(&input, &intent) {
        Err(SchemeError::Infeasible(inf)) => {
            assert!(inf.entities.contains(&ids.edges[0]) && inf.entities.contains(&ids.edges[1]), "{inf:?}");
            assert!(!inf.constraints.is_empty());
        }
        other => panic!("expected Infeasible, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn parity_and_sums_in_the_reference_solver() {
    let e = |n: u64| EntityId::new(sm_core::EntityKind::TopoEdge, n).expect("id");
    let edges = [e(1), e(2), e(3)];
    let rec = |id: u32, item| Record { id: sm_scheme_api::ConstraintId(id), origin: Origin::Intent, item };
    // a + b == c with goals 2, 3 and c hard 7, plus parity of a + b + c.
    let cs = vec![
        rec(0, IntervalConstraint::Goal { edge: e(1), goal: 2.0 }),
        rec(1, IntervalConstraint::Goal { edge: e(2), goal: 3.0 }),
        rec(2, IntervalConstraint::Fixed { edge: e(3), count: 7, firmness: Firmness::Hard }),
        rec(3, IntervalConstraint::SumEqual { lhs: vec![e(1), e(2)], rhs: vec![e(3)] }),
        rec(4, IntervalConstraint::EvenSum { edges: vec![e(1), e(2), e(3)] }),
    ];
    let s = ReferenceIntervalSolver::default().solve(&edges, &cs).expect("solve");
    let v: Vec<u32> = s.iter().map(|x| x.1).collect();
    assert_eq!(v[0] + v[1], 7);
    assert_eq!(v[2], 7);
    assert_eq!((v[0] + v[1] + v[2]) % 2, 0);
    // Odd hard loop that must be even: infeasible, naming the parity constraint.
    let bad = vec![
        rec(0, IntervalConstraint::Fixed { edge: e(1), count: 1, firmness: Firmness::Hard }),
        rec(1, IntervalConstraint::Fixed { edge: e(2), count: 2, firmness: Firmness::Hard }),
        rec(2, IntervalConstraint::EvenSum { edges: vec![e(1), e(2)] }),
    ];
    let err = ReferenceIntervalSolver::default().solve(&edges[..2], &bad).expect_err("odd");
    assert!(err.constraints.contains(&sm_scheme_api::ConstraintId(2)), "{err:?}");
}

/// A surface scheme that always asks for a fallback, with a high auto score.
struct AlwaysFail;

impl Scheme for AlwaysFail {
    fn name(&self) -> &str {
        "always_fail"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(200, "eager")
    }
    fn generate(&self, _cx: &GenCtx<'_>, target: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        Err(GenerateFailure::fallback(target, "always_fail", "refuses"))
    }
}

#[test]
fn generation_failure_falls_back_and_replans() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let mut reg = registry();
    reg.register(Box::new(AlwaysFail)).expect("register");
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = counts_intent(&ids, [2, 2, 2]);
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh");
    // Only the source face is free to pick always_fail (linking faces are imposed, the target is a copy).
    assert_eq!(out.rejections.0.len(), 1);
    let ((t, s), _) = out.rejections.0.iter().next().expect("one");
    assert_eq!((*t, s.as_str()), (Target::Surface(ids.faces[4]), "always_fail"));
    let tp = out.plan.target(Target::Surface(ids.faces[4])).expect("source");
    assert_eq!(tp.scheme.as_str(), "map");
    assert!(validate(&out.finished.mesh).is_empty());
}

#[test]
fn translation_deck_pair_is_meshed_as_an_exact_copy() {
    let (topo, ids, mut model) = brick_setup([1.0, 1.0, 1.0]);
    let b = model.add_boundary("periodic-x", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).expect("deck");
    model.assign_pairs(b, &[(ids.faces[0], ids.faces[1])]).expect("pair");
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = counts_intent(&ids, [2, 3, 4]);
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh");
    let plan = &out.plan;
    let c = plan.copy_into(Target::Surface(ids.faces[1])).expect("deck copy");
    assert_eq!(c.item.reason, CopyReason::Deck { boundary: b });
    for f in [ids.faces[0], ids.faces[1]] {
        assert!(plan.problem.constraints.iter().any(|r| r.item == MeshConstraint::NoBoundaryLayer { face: f }));
    }
    // +x patch points are the -x points shifted by (1, 0, 0), element by element.
    let (pa, pb) = (out.patches.get(Target::Surface(ids.faces[0])).expect("a"), out.patches.get(Target::Surface(ids.faces[1])).expect("b"));
    assert_eq!(pa.elements.len(), pb.elements.len());
    for ((sa, na), (sb, nb)) in pa.elements.iter().zip(pb.elements.iter()) {
        assert_eq!((sa, sb), (ElemShape::Quad, ElemShape::Quad));
        let mut qa: Vec<[i64; 3]> = na.iter().map(|k| q(out.patches.point(*k).expect("p"), 1.0)).collect();
        let mut qb: Vec<[i64; 3]> = nb.iter().map(|k| q(out.patches.point(*k).expect("p"), 0.0)).collect();
        qa.sort_unstable();
        qb.sort_unstable();
        assert_eq!(qa, qb);
    }
    assert!(validate(&out.finished.mesh).is_empty());
    assert_eq!(out.finished.mesh.count(sm_mesh_api::MeshEntityKind::Cell), 24);

    // A boundary layer on a deck face is refused.
    intent.entity_mut(ids.faces[0]).boundary_layer = Some(sm_scheme_api::BoundaryLayerIntent { first: 0.01, growth: 1.2, layers: 3 });
    assert!(matches!(driver.plan(&input, &intent), Err(SchemeError::BoundaryLayerOnDeck { .. })));
}

fn q(p: sm_core::Point3, dx: f64) -> [i64; 3] {
    let s = 1u64 << 20;
    [((p.x + dx) * s as f64).round() as i64, (p.y * s as f64).round() as i64, (p.z * s as f64).round() as i64]
}
