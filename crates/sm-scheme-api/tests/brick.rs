//! End-to-end: equal + map + 1-to-1 sweep on the reference brick, through the driver, into the
//! reference `MeshBuild`. Analytic checks (counts, volumes, coordinates, classification), plus
//! selection, impositions, infeasibility, fallback and deck copies.

mod stub_schemes;

use sm_core::{EntityId, Vector3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{MeshRead, SerialAlloc, cell_volume, validate};
use sm_model_api::{BoundaryKind, Deck};
use sm_scheme_api::{
    Caps, CopyMode, CopyReason, Driver, FailureKind, Firm, Firmness, IntervalConstraint,
    IntervalRequest, IntervalSolver, MeshConstraint, MeshInput, MeshIntent, Need, Origin, Phase,
    Record, ReferenceCornerPicker, ReferenceIntervalSolver, Rejections, Role, SchemeChoice,
    SchemeError, Selection, Sequential, Target, VertexType, WholeFaces,
};
use stub_schemes::{AlwaysFail, brick_setup, registry};

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
    SchemeChoice::with_params(
        "sweep",
        serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }),
        Firmness::Hard,
    )
}

#[test]
fn sweep_brick_is_analytic() {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = counts_intent(&ids, [2, 3, 4]);
    intent.set_scheme(ids.volume, sweep_z(&ids));

    let mut alloc = SerialAlloc::new();
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut alloc,
        )
        .expect("mesh");
    let plan = &out.plan;

    // Interval propagation through map sides and sweep layers.
    for (k, want) in [(0..4, 2u32), (4..8, 3), (8..12, 4)] {
        for e in k {
            assert_eq!(plan.intervals(ids.edges[e]), Some(want), "edge {e}");
        }
    }
    // Roles, capability impositions, copy class.
    let src = Target::Surface(ids.faces[4]);
    let tgt = Target::Surface(ids.faces[5]);
    let vol = Target::Volume(ids.volume);
    assert_eq!(
        plan.roles_of(src).collect::<Vec<_>>(),
        vec![Role::SweepSource { volume: vol }]
    );
    assert_eq!(
        plan.roles_of(tgt).collect::<Vec<_>>(),
        vec![Role::SweepTarget { volume: vol }]
    );
    for f in &ids.faces[..4] {
        let tp = plan.target(Target::Surface(*f)).expect("linking plan");
        assert_eq!(tp.scheme.as_str(), "map");
        assert!(
            matches!(tp.selection, Selection::Imposed { score: 100 }),
            "{:?}",
            tp.selection
        );
        assert!(tp.caps.contains(Caps::QUAD | Caps::STRUCTURED));
        assert_eq!(
            plan.roles_of(Target::Surface(*f)).collect::<Vec<_>>(),
            vec![Role::SweepLinking { volume: vol }]
        );
    }
    let (class, member) = plan.copy_of(tgt).expect("target is a copy");
    assert_eq!(class.root, src);
    assert_eq!(member.mode, CopyMode::Generate);
    let tp = plan.target(tgt).expect("target plan");
    assert!(
        matches!(tp.selection, Selection::Copy { class: 0 })
            || matches!(tp.selection, Selection::Copy { .. })
    );
    assert_eq!(tp.scheme.as_str(), "copy");
    assert!(
        tp.caps.contains(Caps::QUAD),
        "copy inherits the root's capabilities"
    );
    let sweep_copy = plan
        .problem
        .copies
        .iter()
        .find(|c| c.item.target == tgt)
        .expect("constraint");
    assert!(matches!(
        sweep_copy.item.reason,
        CopyReason::SweepTarget { .. }
    ));
    // Every map face has four End corners; the logical sides turn left.
    for l in &plan.layouts {
        assert_eq!(
            l.corners
                .iter()
                .filter(|c| c.vertex_type == VertexType::End)
                .count(),
            4
        );
        assert_eq!(l.sides.len(), 4);
        let dirs: Vec<_> = l.sides.iter().map(|s| s.dir).collect();
        assert_eq!(
            dirs,
            vec![
                sm_scheme_api::SideDir::PlusI,
                sm_scheme_api::SideDir::PlusJ,
                sm_scheme_api::SideDir::MinusI,
                sm_scheme_api::SideDir::MinusJ
            ]
        );
    }

    // The mesh.
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(sm_mesh_api::MeshEntityKind::Cell), 24);
    assert_eq!(m.count(sm_mesh_api::MeshEntityKind::Vertex), 3 * 4 * 5);
    let total: f64 = (0..24)
        .map(|c| cell_volume(m, sm_mesh_api::CellIx(c)))
        .sum();
    assert!((total - 3.0).abs() < 1e-12, "volume {total}");
    for c in 0..24 {
        assert!((cell_volume(m, sm_mesh_api::CellIx(c)) - 0.125).abs() < 1e-12);
    }
    // Faces per model face: -x/+x 3x4, -y/+y 2x4, -z/+z 2x3.
    for (f, want) in ids.faces.iter().zip([12, 12, 8, 8, 6, 6]) {
        assert_eq!(
            m.classified_on(*f).map(|s| s.faces.len()),
            Some(want),
            "face {f}"
        );
    }
    // Every vertex sits on the 0.5 grid.
    let xyz = m.coords();
    for i in 0..xyz.x.len() {
        for c in [xyz.x[i], xyz.y[i], xyz.z[i]] {
            assert_eq!((c * 2.0).round(), c * 2.0);
        }
    }
    assert!(out.plan.rejections.is_empty());
}

#[test]
fn equal_curve_nodes_are_analytic() {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = counts_intent(&ids, [2, 3, 4]);
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    // The z-edge 8 has length 2 and 4 intervals: interior nodes at 0.5, 1.0, 1.5 above its start.
    let e = ids.edges[Z];
    let patch = out.patches.get(Target::Curve(e)).expect("curve patch");
    assert_eq!(patch.nodes.len(), 3);
    let (a, _) = topo_ends(&topo, e);
    let z0 = sm_scheme_api::geom::vertex_point(&topo, a)
        .expect("start")
        .z;
    for (i, n) in patch.nodes.iter().enumerate() {
        assert_eq!(n.key.index, i as u32 + 1);
        assert!(
            (n.point.z - z0 - 0.5 * (i as f64 + 1.0)).abs() < 1e-12,
            "node {i} at {}",
            n.point.z
        );
        assert!(matches!(n.param, sm_mesh_api::Param::Curve(_)));
    }
    // One segment per interval, referencing both end vertices.
    assert_eq!(patch.elements.len(), 4);
    assert_eq!(patch.refs.len(), 2);
}

fn topo_ends(t: &dyn sm_topo_api::TopologyView, e: EntityId) -> (EntityId, EntityId) {
    use sm_topo_api::TopologyViewExt;
    t.edge_ends(e).expect("ends")
}

#[test]
fn auto_selection_records_score_reason_and_losers() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let intent = counts_intent(&ids, [2, 2, 2]);
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    let vp = out.plan.target(Target::Volume(ids.volume)).expect("volume");
    assert_eq!(vp.scheme.as_str(), "sweep");
    assert!(
        matches!(vp.selection, Selection::Auto { score: 60 }),
        "{:?}",
        vp.selection
    );
    assert_eq!(vp.reason, "1-to-1 sweepable");
    // Applicable losers are recorded with their scores; inapplicable ones with their reasons.
    let lost = |scheme: &str| {
        out.plan
            .attempts
            .iter()
            .find(|a| a.target == Target::Volume(ids.volume) && a.scheme.as_str() == scheme)
    };
    let mapvol = lost("mapvol").expect("mapvol was evaluated");
    assert_eq!(
        (mapvol.kind, mapvol.phase),
        (FailureKind::Rejected, Phase::Resolve)
    );
    assert!(
        mapvol.message.contains("outscored by `sweep` (30 < 60)"),
        "{}",
        mapvol.message
    );
    let ms = lost("msweep").expect("msweep was evaluated");
    assert_eq!(ms.kind, FailureKind::NotApplicable);
    // Lowest-id opposite pair: -x -> +x.
    assert!(out.plan.copy_of(Target::Surface(ids.faces[1])).is_some());
    assert_eq!(
        out.finished.mesh.count(sm_mesh_api::MeshEntityKind::Cell),
        8
    );
    // The same attempts are in the Meshed summary.
    assert!(out.attempts.iter().any(|a| a.scheme.as_str() == "mapvol"));
}

#[test]
fn hard_pave_on_a_linking_face_conflicts_with_the_sweep() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = counts_intent(&ids, [1, 1, 1]);
    intent.set_scheme(ids.volume, sweep_z(&ids));
    intent.set_scheme(ids.faces[1], SchemeChoice::new("pave", Firmness::Hard));
    match driver.plan(&input, &intent) {
        Err(SchemeError::ImpositionConflict {
            child,
            imposed_by,
            required,
            ..
        }) => {
            assert_eq!(child, Target::Surface(ids.faces[1]));
            assert_eq!(imposed_by, Target::Volume(ids.volume));
            assert_eq!(required, Need::Caps(Caps::QUAD | Caps::STRUCTURED));
        }
        other => panic!("expected ImpositionConflict, got {:?}", other.map(|_| ())),
    }
    // A soft pave is overridden instead, and recorded.
    intent.set_scheme(ids.faces[1], SchemeChoice::new("pave", Firmness::Soft));
    let plan = driver.plan(&input, &intent).expect("plan");
    let tp = plan.target(Target::Surface(ids.faces[1])).expect("face");
    assert_eq!(tp.scheme.as_str(), "map");
    let a = plan
        .attempts
        .iter()
        .find(|a| a.target == Target::Surface(ids.faces[1]) && a.scheme.as_str() == "pave")
        .expect("pave attempt");
    assert_eq!(a.kind, FailureKind::Rejected);
    // The sweep target is a copy: a hard pave there conflicts too, a soft one is overridden.
    intent.set_scheme(ids.faces[5], SchemeChoice::new("pave", Firmness::Hard));
    match driver.plan(&input, &intent) {
        Err(SchemeError::ImpositionConflict {
            child, imposed_by, ..
        }) => {
            assert_eq!(
                (child, imposed_by),
                (Target::Surface(ids.faces[5]), Target::Surface(ids.faces[4]))
            );
        }
        other => panic!("expected ImpositionConflict, got {:?}", other.map(|_| ())),
    }
    intent.set_scheme(ids.faces[5], SchemeChoice::new("pave", Firmness::Soft));
    let plan = driver.plan(&input, &intent).expect("plan");
    let tp = plan
        .target(Target::Surface(ids.faces[5]))
        .expect("target face");
    assert_eq!(tp.scheme.as_str(), "copy");
    assert!(
        plan.attempts
            .iter()
            .any(|a| a.target == Target::Surface(ids.faces[5]) && a.scheme.as_str() == "pave")
    );
}

#[test]
fn hard_counts_that_break_map_sides_are_infeasible() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = counts_intent(&ids, [2, 2, 2]);
    intent.set_intervals(ids.edges[1], Firm::hard(IntervalRequest::Count(3))); // another x-edge
    intent.set_scheme(ids.volume, sweep_z(&ids));
    match driver.plan(&input, &intent) {
        Err(SchemeError::Infeasible(inf)) => {
            assert!(
                inf.entities.contains(&ids.edges[0]) && inf.entities.contains(&ids.edges[1]),
                "{inf:?}"
            );
            assert!(!inf.constraints.is_empty());
            // The blame is a scheme-origin constraint of a non-hard selection (a face scheme).
            let (t, s) = inf.blame.clone().expect("blame");
            assert!(
                matches!(t, Target::Surface(_)) && s.as_str() == "map",
                "{t:?} {s}"
            );
        }
        other => panic!("expected Infeasible, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn parity_and_sums_in_the_reference_solver() {
    let e = |n: u64| EntityId::new(sm_core::EntityKind::TopoEdge, n).expect("id");
    let edges = [e(1), e(2), e(3)];
    let rec = |id: u32, item| Record {
        id: sm_scheme_api::ConstraintId(id),
        origin: Origin::Intent,
        item,
    };
    // a + b == c with goals 2, 3 and c hard 7, plus parity of a + b + c.
    let cs = vec![
        rec(
            0,
            IntervalConstraint::Goal {
                edge: e(1),
                goal: 2.0,
            },
        ),
        rec(
            1,
            IntervalConstraint::Goal {
                edge: e(2),
                goal: 3.0,
            },
        ),
        rec(
            2,
            IntervalConstraint::Fixed {
                edge: e(3),
                count: 7,
                firmness: Firmness::Hard,
            },
        ),
        rec(3, IntervalConstraint::sum_equal(&[e(1), e(2)], &[e(3)])),
        rec(
            4,
            IntervalConstraint::EvenSum {
                edges: vec![e(1), e(2), e(3)],
            },
        ),
    ];
    let s = ReferenceIntervalSolver::default()
        .solve(&edges, &cs)
        .expect("solve");
    let v: Vec<u32> = s.iter().map(|x| x.1).collect();
    assert_eq!(v[0] + v[1], 7);
    assert_eq!(v[2], 7);
    assert_eq!((v[0] + v[1] + v[2]) % 2, 0);
    // Odd hard loop that must be even: infeasible, naming the parity constraint.
    let bad = vec![
        rec(
            0,
            IntervalConstraint::Fixed {
                edge: e(1),
                count: 1,
                firmness: Firmness::Hard,
            },
        ),
        rec(
            1,
            IntervalConstraint::Fixed {
                edge: e(2),
                count: 2,
                firmness: Firmness::Hard,
            },
        ),
        rec(
            2,
            IntervalConstraint::EvenSum {
                edges: vec![e(1), e(2)],
            },
        ),
    ];
    let err = ReferenceIntervalSolver::default()
        .solve(&edges[..2], &bad)
        .expect_err("odd");
    assert!(
        err.constraints.contains(&sm_scheme_api::ConstraintId(2)),
        "{err:?}"
    );
}

#[test]
fn generation_failure_falls_back_and_replans() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let mut reg = registry();
    reg.register(Box::new(AlwaysFail)).expect("register");
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = counts_intent(&ids, [2, 2, 2]);
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    // Only the source face is free to pick always_fail (linking faces are imposed, the target is a copy).
    assert_eq!(out.plan.rejections.len(), 1);
    let a = out.plan.rejections.attempts().next().expect("one");
    assert_eq!(
        (a.target, a.scheme.as_str(), a.phase, a.kind),
        (
            Target::Surface(ids.faces[4]),
            "always_fail",
            Phase::Generate,
            FailureKind::Quality
        )
    );
    let tp = out
        .plan
        .target(Target::Surface(ids.faces[4]))
        .expect("source");
    assert_eq!(tp.scheme.as_str(), "map");
    assert!(validate(&out.finished.mesh).is_empty());
    // The first attempt of the Meshed summary is the failure that caused the re-plan.
    assert_eq!(out.attempts.first(), Some(a));
    // Replaying (intent, rejections) reproduces the plan exactly.
    let again = driver
        .plan_with(&input, &intent, &out.plan.rejections)
        .expect("replay");
    assert_eq!(again, out.plan);
    // The stored rejections survive a JSON round trip.
    let text = serde_json::to_string(&out.plan.rejections).expect("json");
    let back: Rejections = serde_json::from_str(&text).expect("parse");
    assert_eq!(back, out.plan.rejections);
}

#[test]
fn translation_deck_pair_is_meshed_as_an_exact_copy() {
    let (topo, ids, mut model) = brick_setup([1.0, 1.0, 1.0]);
    let b = model
        .add_boundary(
            "periodic-x",
            BoundaryKind::Deck(Deck::Translation {
                vector: Vector3::new(1.0, 0.0, 0.0),
            }),
        )
        .expect("deck");
    model
        .assign_pairs(b, &[(ids.faces[0], ids.faces[1])])
        .expect("pair");
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = counts_intent(&ids, [2, 3, 4]);
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    let plan = &out.plan;
    let c = plan
        .problem
        .copy_into(Target::Surface(ids.faces[1]))
        .expect("deck copy");
    assert_eq!(c.item.reason, CopyReason::Deck { boundary: b });
    // Deck faces forbid boundary layers as a hard capability imposition with a deck origin.
    for f in [ids.faces[0], ids.faces[1]] {
        let imp = plan
            .problem
            .impositions
            .iter()
            .find(|r| {
                r.item.child == Target::Surface(f)
                    && r.item.need == Need::Forbid(Caps::BOUNDARY_LAYER)
            })
            .expect("forbid");
        assert_eq!(
            (imp.item.firmness, &imp.origin),
            (Firmness::Hard, &Origin::Deck { boundary: b })
        );
    }
    // The +x face is a copy of the -x face, and the copied mesh equals the image element by element.
    let (class, member) = plan.copy_of(Target::Surface(ids.faces[1])).expect("class");
    assert_eq!(
        (class.root, member.mode),
        (Target::Surface(ids.faces[0]), CopyMode::Generate)
    );
    let (pa, pb) = (
        out.patches.get(Target::Surface(ids.faces[0])).expect("a"),
        out.patches.get(Target::Surface(ids.faces[1])).expect("b"),
    );
    assert_eq!(pa.elements.len(), pb.elements.len());
    let key = |p: &sm_scheme_api::Patch, l: u32| p.key(l).expect("key");
    for ((sa, na), (sb, nb)) in pa.elements.iter().zip(pb.elements.iter()) {
        assert_eq!(
            (sa, sb),
            (
                sm_scheme_api::ElemShape::Quad,
                sm_scheme_api::ElemShape::Quad
            )
        );
        let mut qa: Vec<[i64; 3]> = na
            .iter()
            .map(|l| q(out.patches.point(key(pa, *l)).expect("p"), 1.0))
            .collect();
        let mut qb: Vec<[i64; 3]> = nb
            .iter()
            .map(|l| q(out.patches.point(key(pb, *l)).expect("p"), 0.0))
            .collect();
        qa.sort_unstable();
        qb.sort_unstable();
        assert_eq!(qa, qb);
    }
    assert!(validate(&out.finished.mesh).is_empty());
    assert_eq!(
        out.finished.mesh.count(sm_mesh_api::MeshEntityKind::Cell),
        24
    );

    // A boundary layer on a deck face is refused.
    intent.entity_mut(ids.faces[0]).boundary_layer = Some(sm_scheme_api::BoundaryLayerIntent {
        first: 0.01,
        growth: 1.2,
        layers: 3,
    });
    assert!(matches!(
        driver.plan(&input, &intent),
        Err(SchemeError::BoundaryLayerOnDeck { .. })
    ));
}

#[test]
fn boundary_layer_needs_a_capable_volume_scheme() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = counts_intent(&ids, [1, 1, 1]);
    intent.entity_mut(ids.faces[2]).boundary_layer = Some(sm_scheme_api::BoundaryLayerIntent {
        first: 0.01,
        growth: 1.2,
        layers: 3,
    });
    // No registered volume scheme has the capability: the intent is refused, with attempts.
    match driver.plan(&input, &intent) {
        Err(SchemeError::NoApplicableScheme { target, tried, .. }) => {
            assert_eq!(target, Target::Volume(ids.volume));
            assert!(
                tried
                    .iter()
                    .any(|a| a.message.contains("BOUNDARY_LAYER") || a.message.contains("Caps")),
                "{tried:?}"
            );
        }
        other => panic!("expected NoApplicableScheme, got {:?}", other.map(|_| ())),
    }
    // A hard sweep is told that it cannot honour the request.
    intent.set_scheme(ids.volume, sweep_z(&ids));
    assert!(matches!(
        driver.plan(&input, &intent),
        Err(SchemeError::NotApplicable { .. })
    ));
    // The constraint is recorded when a capable scheme exists.
    let mut reg2 = registry();
    struct Bl;
    impl sm_scheme_api::Scheme for Bl {
        fn name(&self) -> &str {
            "bl"
        }
        fn dim(&self) -> sm_scheme_api::SchemeDim {
            sm_scheme_api::SchemeDim::Volume
        }
        fn caps(&self, _p: &sm_scheme_api::Params) -> Caps {
            Caps::BOUNDARY_LAYER | Caps::TET
        }
        fn applicability(
            &self,
            _cx: &sm_scheme_api::PlanCtx<'_>,
            _t: Target,
            _p: &sm_scheme_api::Params,
        ) -> sm_scheme_api::Applicability {
            sm_scheme_api::Applicability::yes(1, "bl")
        }
        fn generate(
            &self,
            _cx: &sm_scheme_api::GenCtx<'_>,
            t: Target,
            _p: &sm_scheme_api::Params,
        ) -> Result<sm_scheme_api::Patch, sm_scheme_api::GenerateFailure> {
            Err(sm_scheme_api::GenerateFailure::new(
                t,
                "bl",
                FailureKind::Unsupported,
                "plan only",
            ))
        }
    }
    reg2.register(Box::new(Bl)).expect("bl");
    let driver2 = Driver::new(&reg2, &solver, &picker);
    intent
        .entities
        .get_mut(&ids.volume)
        .expect("volume intent")
        .scheme = None;
    let plan = driver2.plan(&input, &intent).expect("plan");
    assert_eq!(
        plan.target(Target::Volume(ids.volume))
            .map(|t| t.scheme.as_str()),
        Some("bl")
    );
    assert!(plan.problem.constraints.iter().any(
        |c| matches!(c.item, MeshConstraint::BoundaryLayer { face, .. } if face == ids.faces[2])
    ));
}

fn q(p: sm_core::Point3, dx: f64) -> [i64; 3] {
    let s = 1u64 << 20;
    [
        ((p.x + dx) * s as f64).round() as i64,
        (p.y * s as f64).round() as i64,
        (p.z * s as f64).round() as i64,
    ]
}
