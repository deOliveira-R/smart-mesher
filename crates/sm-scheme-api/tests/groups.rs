//! Volumes that are meshed together: the many-to-one style multisweep of three stacked
//! volumes (interval matching, claimed faces written by the group), atomic failure,
//! standalone transforms, and meshing a decomposition piece.

mod stub_schemes;

use sm_core::{EntityId, Vector3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{
    CellIx, MeshEntityKind, MeshRead, SerialAlloc, cell_volume, geometry_hash, topology_hash,
    validate,
};
use sm_model_api::{BoundaryKind, Deck, Domain, PieceFace, PieceRef, PieceShape};
use sm_scheme_api::{
    CopyMode, Driver, FailureKind, Firm, Firmness, GroupId, IntervalRequest, Job, MeshGroup,
    MeshInput, MeshIntent, Phase, Plan, ReferenceCornerPicker, ReferenceIntervalSolver,
    SchemeChoice, SchemeError, Selection, Sequential, Target, TransformId, TransformIntent,
    WholeFaces,
};
use stub_schemes::{registry, row_setup};

fn solver_parts() -> (ReferenceIntervalSolver, ReferenceCornerPicker) {
    (ReferenceIntervalSolver::default(), ReferenceCornerPicker)
}

fn group_intent(ids: &[sm_topo_api::fixtures::BrickIds], extra: serde_json::Value) -> MeshIntent {
    let sel: Vec<EntityId> = ids.iter().map(|b| b.volume).collect();
    let mut params =
        serde_json::json!({ "source": [ids[0].faces[0]], "target": [ids[ids.len() - 1].faces[1]] });
    if let (Some(p), Some(e)) = (params.as_object_mut(), extra.as_object()) {
        p.extend(e.clone());
    }
    let mut intent = MeshIntent::default();
    intent.groups.push(MeshGroup {
        id: GroupId(1),
        members: sel,
        scheme: Some(SchemeChoice::with_params("msweep", params, Firmness::Hard)),
        fallbacks: Vec::new(),
        conversion: None,
    });
    intent
}

/// Hard y/z counts, hard x counts on the first and last volume: the middle one follows from the
/// path sum.
fn counted(ids: &[sm_topo_api::fixtures::BrickIds], intent: &mut MeshIntent, nx: [u32; 2]) {
    intent.set_intervals(ids[0].edges[0], Firm::hard(IntervalRequest::Count(nx[0])));
    intent.set_intervals(
        ids[ids.len() - 1].edges[0],
        Firm::hard(IntervalRequest::Count(nx[1])),
    );
    intent.set_intervals(ids[0].edges[4], Firm::hard(IntervalRequest::Count(2)));
    intent.set_intervals(ids[0].edges[8], Firm::hard(IntervalRequest::Count(3)));
}

#[test]
fn many_to_one_sweep_matches_intervals_across_three_volumes() {
    let (topo, ids, model) = row_setup(3);
    let reg = registry();
    let (solver, picker) = solver_parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel: Vec<EntityId> = ids.iter().map(|b| b.volume).collect();
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = group_intent(&ids, serde_json::json!({ "layers": 8 }));
    counted(&ids, &mut intent, [2, 3]);
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

    // Interval matching: A has 2 layers, C has 3, the path sum 8 leaves 3 for B.
    for (b, want) in ids.iter().zip([2u32, 3, 3]) {
        for e in &b.edges[..4] {
            assert_eq!(plan.intervals(*e), Some(want), "x-edges of {}", b.volume);
        }
    }
    // The group claims both interfaces and the target; they have no scheme of their own.
    let g = Target::Group(GroupId(1));
    let claimed = [ids[0].faces[1], ids[1].faces[1], ids[2].faces[1]];
    for f in claimed {
        assert_eq!(plan.claimed_by(Target::Surface(f)), Some(g));
        let tp = plan.target(Target::Surface(f)).expect("claimed plan");
        assert!(
            matches!(tp.selection, Selection::Claimed { by } if by == g),
            "{:?}",
            tp.selection
        );
        assert_eq!(tp.scheme.as_str(), "msweep");
    }
    assert_eq!(plan.problem.claims.len(), 3);
    // The footprint is derived: the group writes itself and what it claims, and does not read
    // what it claims.
    let fp = plan
        .footprints
        .iter()
        .find(|f| f.job == Job::Generate(g))
        .expect("group footprint");
    for f in claimed {
        assert!(fp.writes.contains(&Target::Surface(f)) && !fp.reads.contains(&Target::Surface(f)));
    }
    assert!(fp.writes.contains(&g));
    let wave_of = |j: Job| {
        plan.waves
            .iter()
            .position(|w| w.contains(&j))
            .expect("scheduled")
    };
    assert!(
        wave_of(Job::Generate(Target::Surface(ids[0].faces[0]))) < wave_of(Job::Generate(g)),
        "the source is meshed before the group"
    );

    // The mesh: 2 x 3 source quads, 8 layers.
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 6 * 8);
    let total: f64 = (0..6 * 8).map(|c| cell_volume(m, CellIx(c as u32))).sum();
    assert!((total - 3.0).abs() < 1e-12, "volume {total}");
    for (b, layers) in ids.iter().zip([2usize, 3, 3]) {
        assert_eq!(
            m.classified_on(b.volume).map(|s| s.cells.len()),
            Some(6 * layers),
            "cells of {}",
            b.volume
        );
    }
    // Claimed faces are classified like any other: one quad per source element.
    for f in claimed {
        assert_eq!(
            m.classified_on(f).map(|s| s.faces.len()),
            Some(6),
            "face {f}"
        );
    }
    // Their patches exist (split from the group's) and are owned by the faces.
    let t = out
        .patches
        .get(Target::Surface(ids[2].faces[1]))
        .expect("claimed target patch");
    assert!(t.nodes.iter().all(|n| n.key.owner == ids[2].faces[1]));
    assert_eq!(t.elements.len(), 6);
}

#[test]
fn infeasible_path_sum_names_the_group_scheme() {
    let (topo, ids, model) = row_setup(3);
    let reg = registry();
    let (solver, picker) = solver_parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel: Vec<EntityId> = ids.iter().map(|b| b.volume).collect();
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = group_intent(&ids, serde_json::json!({ "layers": 4 }));
    counted(&ids, &mut intent, [2, 3]); // 2 + 3 > 4: impossible for any positive middle count
    match driver.plan(&input, &intent) {
        Err(SchemeError::Infeasible(inf)) => {
            assert!(inf.constraints.len() >= 3, "{inf:?}");
            // The group's choice is a hard intent, so it is not blamed; the cheapest re-plan is a
            // face scheme (lowest dimension among the soft selections).
            let (t, s) = inf.blame.expect("blame");
            assert!(
                matches!(t, Target::Surface(_)) && s.as_str() == "map",
                "{t:?} {s}"
            );
        }
        other => panic!("expected Infeasible, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn failing_group_leaves_the_allocator_and_later_meshes_untouched() {
    let (topo, ids, model) = row_setup(3);
    let reg = registry();
    let (solver, picker) = solver_parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel: Vec<EntityId> = ids.iter().map(|b| b.volume).collect();
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let state = |a: &SerialAlloc| a.state().collect::<Vec<_>>();
    let good = {
        let mut intent = group_intent(&ids, serde_json::json!({}));
        counted(&ids, &mut intent, [1, 1]);
        intent
    };
    let reference = {
        let mut alloc = SerialAlloc::new();
        let out = driver
            .mesh(
                &input,
                &good,
                &Sequential,
                ReferenceBuilder::new(),
                &mut alloc,
            )
            .expect("mesh");
        (
            topology_hash(&out.finished.mesh),
            geometry_hash(&out.finished.mesh),
            state(&alloc),
        )
    };
    let mut alloc = SerialAlloc::new();
    let before = state(&alloc);
    // An internal failure of the group scheme is fatal; a quality failure is re-planned and
    // then has nothing left to try. Neither writes anything.
    for hook in ["boom", "quality"] {
        let mut intent = group_intent(&ids, serde_json::json!({ "fail": hook }));
        counted(&ids, &mut intent, [1, 1]);
        match driver.mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut alloc,
        ) {
            Err(SchemeError::Generate(g)) => {
                assert_eq!((hook, g.kind), ("boom", FailureKind::Internal))
            }
            Err(SchemeError::NoApplicableScheme { target, tried, .. }) => {
                assert_eq!((hook, target), ("quality", Target::Group(GroupId(1))));
                assert!(
                    tried
                        .iter()
                        .any(|a| a.scheme.as_str() == "msweep" && a.kind == FailureKind::Rejected),
                    "{tried:?}"
                );
            }
            other => panic!("{hook}: unexpected {:?}", other.map(|_| ())),
        }
        assert_eq!(state(&alloc), before, "{hook}: ids must not shift");
    }
    // A failing conversion of the group rolls back the whole mesh too.
    let mut intent = good.clone();
    intent.groups[0].conversion = Some(SchemeChoice::new("boom", Firmness::Hard));
    assert!(
        driver
            .mesh(
                &input,
                &intent,
                &Sequential,
                ReferenceBuilder::new(),
                &mut alloc
            )
            .is_err()
    );
    assert_eq!(state(&alloc), before);
    // The next success is bit-identical to a run on a fresh allocator.
    let out = driver
        .mesh(
            &input,
            &good,
            &Sequential,
            ReferenceBuilder::new(),
            &mut alloc,
        )
        .expect("mesh");
    assert_eq!(
        (
            topology_hash(&out.finished.mesh),
            geometry_hash(&out.finished.mesh),
            state(&alloc)
        ),
        reference
    );
}

#[test]
fn a_mesh_that_fails_verification_is_rejected_and_replanned() {
    let (topo, ids, mut model) = row_setup(3);
    // The deck says the +x end of the stack is the -x end translated by 3: the group writes the
    // +x face itself, so the driver verifies it against the image of the -x face.
    let b = model
        .add_boundary(
            "periodic-x",
            BoundaryKind::Deck(Deck::Translation {
                vector: Vector3::new(3.0, 0.0, 0.0),
            }),
        )
        .expect("deck");
    model
        .assign_pairs(b, &[(ids[0].faces[0], ids[2].faces[1])])
        .expect("pair");
    let reg = registry();
    let (solver, picker) = solver_parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel: Vec<EntityId> = ids.iter().map(|b| b.volume).collect();
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = group_intent(&ids, serde_json::json!({}));
    counted(&ids, &mut intent, [1, 1]);
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("a consistent group verifies");
    let (class, member) = out
        .plan
        .copy_of(Target::Surface(ids[2].faces[1]))
        .expect("target is a class member");
    assert_eq!(
        (class.root, member.mode),
        (Target::Surface(ids[0].faces[0]), CopyMode::Verify)
    );
    assert!(validate(&out.finished.mesh).is_empty());
    // A scheme that writes a slightly different target is caught at generation time.
    let mut bad = group_intent(&ids, serde_json::json!({ "perturb": true }));
    counted(&ids, &mut bad, [1, 1]);
    let plan = driver.plan(&input, &bad).expect("the plan is fine");
    match driver.generate(&input, &plan, &Sequential) {
        Err(SchemeError::CopyMismatch { reason, .. }) => {
            assert!(reason.contains("away from the image"), "{reason}")
        }
        other => panic!("expected CopyMismatch, got {:?}", other.map(|_| ())),
    }
    // `mesh` rejects the group scheme and re-plans; with nothing else to try (even for a soft
    // choice) it fails with the history, and nothing is committed.
    let mut alloc = SerialAlloc::new();
    let before: Vec<_> = alloc.state().collect();
    for firmness in [Firmness::Hard, Firmness::Soft] {
        bad.groups[0].scheme = bad.groups[0].scheme.take().map(|mut s| {
            s.firmness = firmness;
            s
        });
        match driver.mesh(
            &input,
            &bad,
            &Sequential,
            ReferenceBuilder::new(),
            &mut alloc,
        ) {
            Err(SchemeError::NoApplicableScheme { tried, .. }) => {
                let a = tried
                    .iter()
                    .find(|a| a.scheme.as_str() == "msweep")
                    .expect("msweep attempt");
                assert_eq!(a.kind, FailureKind::Rejected);
                assert!(
                    a.message.contains("Verify") && a.message.contains("away from the image"),
                    "{}",
                    a.message
                );
            }
            other => panic!("expected NoApplicableScheme, got {:?}", other.map(|_| ())),
        }
        assert_eq!(alloc.state().collect::<Vec<_>>(), before);
    }
    let _ = Phase::Verify;
}

fn brick_sweep(ids: &sm_topo_api::fixtures::BrickIds) -> SchemeChoice {
    SchemeChoice::with_params(
        "sweep",
        serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }),
        Firmness::Hard,
    )
}

#[test]
fn standalone_transforms_run_after_their_writers_and_commit_atomically() {
    let (topo, ids, model) = stub_schemes::brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = solver_parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = MeshIntent::default();
    intent.set_scheme(ids.volume, brick_sweep(&ids));
    intent.set_intervals(ids.edges[8], Firm::hard(IntervalRequest::Count(2)));
    intent.set_intervals(ids.edges[0], Firm::hard(IntervalRequest::Count(2)));
    intent.set_intervals(ids.edges[4], Firm::hard(IntervalRequest::Count(2)));
    let plain = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    let h = |o: &sm_scheme_api::Meshed<_>| {
        (
            topology_hash(&o.finished.mesh),
            geometry_hash(&o.finished.mesh),
        )
    };
    // A transform job is scheduled after the volume and a no-op leaves the mesh bit-identical.
    intent.transforms.push(TransformIntent {
        id: TransformId(1),
        targets: vec![ids.volume],
        transform: SchemeChoice::new("noop", Firmness::Hard),
    });
    let noop = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    assert_eq!(h(&plain), h(&noop));
    let wave_of = |p: &Plan, j: Job| {
        p.waves
            .iter()
            .position(|w| w.contains(&j))
            .expect("scheduled")
    };
    assert!(
        wave_of(&noop.plan, Job::Generate(Target::Volume(ids.volume)))
            < wave_of(&noop.plan, Job::Transform(TransformId(1)))
    );
    let fp = noop
        .plan
        .footprints
        .iter()
        .find(|f| f.job == Job::Transform(TransformId(1)))
        .expect("footprint");
    assert_eq!(
        (fp.reads.clone(), fp.writes.clone()),
        (
            vec![Target::Volume(ids.volume)],
            vec![Target::Volume(ids.volume)]
        )
    );
    // A lifting transform moves the volume-owned (interior layer) nodes only.
    intent.transforms[0].transform =
        SchemeChoice::with_params("lift", serde_json::json!({ "dz": 0.1 }), Firmness::Hard);
    let lifted = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    assert_ne!(h(&plain), h(&lifted));
    assert_eq!(
        lifted.finished.mesh.count(MeshEntityKind::Vertex),
        plain.finished.mesh.count(MeshEntityKind::Vertex)
    );
    // A failing transform fails the whole mesh and leaves the allocator alone.
    intent.transforms[0].transform = SchemeChoice::new("boom", Firmness::Hard);
    let mut alloc = SerialAlloc::new();
    let before: Vec<_> = alloc.state().collect();
    assert!(
        matches!(driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut alloc), Err(SchemeError::Generate(g)) if g.kind == FailureKind::Internal)
    );
    assert_eq!(alloc.state().collect::<Vec<_>>(), before);
    // Transforms must name selected pieces and registered transforms.
    intent.transforms[0].transform = SchemeChoice::new("nope", Firmness::Hard);
    assert!(matches!(
        driver.plan(&input, &intent),
        Err(SchemeError::UnknownScheme { .. })
    ));
}

#[test]
fn a_decomposition_piece_is_meshed_and_classified_on_its_cell() {
    let (topo, ids, mut model) = row_setup(2);
    // Two merged volumes of one color form one composite piece: the interface is internal, all
    // ten boundary faces are whole model faces, so the default resolver handles it.
    let mat = model.add_color(1, "mat", Domain::Volume).expect("color");
    model
        .color_volumes(mat, &[ids[0].volume, ids[1].volume])
        .expect("color volumes");
    let cell = model
        .composite(
            &[
                PieceRef::Whole(ids[0].volume),
                PieceRef::Whole(ids[1].volume),
            ],
            &topo,
        )
        .expect("composite");
    model
        .set_shape(
            cell,
            PieceShape::Sweep {
                source: vec![PieceFace::Whole(ids[0].faces[0])],
                target: vec![PieceFace::Whole(ids[1].faces[1])],
            },
        )
        .expect("shape");
    let reg = registry();
    let (solver, picker) = solver_parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [cell];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    assert_eq!(
        input
            .piece_faces(Target::Volume(cell))
            .expect("faces")
            .len(),
        10
    );
    let mut intent = MeshIntent::default();
    intent.set_intervals(ids[0].edges[0], Firm::hard(IntervalRequest::Count(2)));
    intent.set_intervals(ids[1].edges[0], Firm::hard(IntervalRequest::Count(3)));
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    // The shape hint picked the sweep; columns pass through the interior vertices.
    let tp = out.plan.target(Target::Volume(cell)).expect("piece plan");
    assert_eq!(
        (tp.scheme.as_str(), &tp.selection),
        ("sweep", &Selection::PieceShape)
    );
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 5);
    let total: f64 = (0..5).map(|c| cell_volume(m, CellIx(c))).sum();
    assert!((total - 2.0).abs() < 1e-12, "volume {total}");
    assert_eq!(
        m.classified_on(cell).map(|s| s.cells.len()),
        Some(5),
        "cells classified on the PartitionCell"
    );
    assert!(m.classified_on(ids[0].volume).is_none());
    // The path edges follow the hard counts on the two bricks.
    assert_eq!(out.plan.intervals(ids[0].edges[0]), Some(2));
    assert_eq!(out.plan.intervals(ids[1].edges[0]), Some(3));
}
