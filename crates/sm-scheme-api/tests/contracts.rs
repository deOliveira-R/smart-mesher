//! Smaller contract points: requirement validation (D5), blame of the imposing parent (D7),
//! scheduling errors (D6), patch API (D11, D12), polyhedral cells (D13), linear interval rows,
//! geometry helpers, executors.

mod stub_schemes;

use core::f64::consts::FRAC_PI_2;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{CellIx, MeshEntityKind, MeshRead, Param, SerialAlloc, cell_volume, validate};
use sm_scheme_api::geom;
use sm_scheme_api::{
    Applicability, Attempt, Caps, ConstraintId, Driver, ElemShape, Elements, FailureKind, Firmness,
    GenCtx, GenerateFailure, IntervalConstraint, IntervalSolver, MeshConstraint, MeshInput,
    MeshIntent, NodeGrid, NodeKey, Origin, Params, Patch, PatchBuilder, Phase, PlanCtx, Record,
    ReferenceCornerPicker, ReferenceIntervalSolver, Rejections, Rel, Requirements, Scheme,
    SchemeChoice, SchemeDim, SchemeError, Selection, Sequential, Target, WholeFaces,
};
use sm_topo_api::{Side, TopologyView};
use stub_schemes::{brick_setup, registry, row_setup};

fn parts() -> (ReferenceIntervalSolver, ReferenceCornerPicker) {
    (ReferenceIntervalSolver::default(), ReferenceCornerPicker)
}

// ---------------------------------------------------------------- D5: requirements are checked

#[test]
fn requirements_about_non_children_are_refused_with_a_typed_error() {
    let (topo, ids, model) = row_setup(2);
    let reg = registry();
    let (solver, picker) = parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids[0].volume, ids[1].volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = MeshIntent::default();
    // A volume imposing on its sibling volume: not a strict sub-entity.
    intent.set_scheme(
        ids[0].volume,
        SchemeChoice::with_params(
            "badreq",
            serde_json::json!({ "other": ids[1].volume }),
            Firmness::Hard,
        ),
    );
    intent.set_scheme(ids[1].volume, SchemeChoice::new("tetvol", Firmness::Hard));
    match driver.plan(&input, &intent) {
        Err(SchemeError::BadRequirement { by, child, reason }) => {
            assert_eq!(
                (by, child),
                (Target::Volume(ids[0].volume), Target::Volume(ids[1].volume))
            );
            assert!(reason.contains("sub-entity"), "{reason}");
        }
        other => panic!("expected BadRequirement, got {:?}", other.map(|_| ())),
    }
    // Imposing on itself is no better.
    intent.set_scheme(
        ids[0].volume,
        SchemeChoice::with_params(
            "badreq",
            serde_json::json!({ "other": ids[0].volume }),
            Firmness::Hard,
        ),
    );
    assert!(matches!(
        driver.plan(&input, &intent),
        Err(SchemeError::BadRequirement { .. })
    ));
}

/// A volume scheme that declares a precedence cycle between two of its faces, or claims a
/// face another volume claims too.
struct Odd;

impl Scheme for Odd {
    fn name(&self) -> &str {
        "odd"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::NONE
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(1, "test")
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        let faces = cx.input.piece_faces(target)?;
        match p.get::<String>("odd", "mode")?.as_deref() {
            Some("cycle") => {
                let (a, b) = (Target::Surface(faces[0]), Target::Surface(faces[1]));
                out.constraint(MeshConstraint::Precedes {
                    before: a,
                    after: b,
                });
                out.constraint(MeshConstraint::Precedes {
                    before: b,
                    after: a,
                });
            }
            Some("claim") => {
                let shared: EntityId = p.get("odd", "face")?.expect("face");
                out.claim(Target::Surface(shared));
            }
            _ => {}
        }
        Ok(())
    }
    fn generate(&self, _cx: &GenCtx<'_>, t: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        Err(GenerateFailure::fatal(t, "odd", "plan only"))
    }
}

#[test]
fn scheduling_errors_are_typed_and_name_the_constraints() {
    let (topo, ids, model) = row_setup(2);
    let mut reg = registry();
    reg.register(Box::new(Odd)).expect("register");
    let (solver, picker) = parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids[0].volume, ids[1].volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = MeshIntent::default();
    intent.set_scheme(
        ids[0].volume,
        SchemeChoice::with_params(
            "odd",
            serde_json::json!({ "mode": "cycle" }),
            Firmness::Hard,
        ),
    );
    intent.set_scheme(ids[1].volume, SchemeChoice::new("tetvol", Firmness::Hard));
    match driver.plan(&input, &intent) {
        Err(SchemeError::Schedule {
            jobs,
            constraints,
            reason,
        }) => {
            assert!(reason.contains("cycle"), "{reason}");
            assert!(
                jobs.len() >= 2 && constraints.len() == 2,
                "{jobs:?} {constraints:?}"
            );
        }
        other => panic!("expected Schedule, got {:?}", other.map(|_| ())),
    }
    // Two volumes claim the interface face: two writers.
    let shared = ids[0].faces[1];
    intent.set_scheme(
        ids[0].volume,
        SchemeChoice::with_params(
            "odd",
            serde_json::json!({ "mode": "claim", "face": shared }),
            Firmness::Hard,
        ),
    );
    intent.set_scheme(
        ids[1].volume,
        SchemeChoice::with_params(
            "odd",
            serde_json::json!({ "mode": "claim", "face": shared }),
            Firmness::Hard,
        ),
    );
    match driver.plan(&input, &intent) {
        Err(SchemeError::Schedule {
            reason,
            constraints,
            ..
        }) => {
            assert!(reason.contains("written by two jobs"), "{reason}");
            assert_eq!(constraints.len(), 2);
        }
        other => panic!("expected Schedule, got {:?}", other.map(|_| ())),
    }
}

// ---------------------------------------------------------------- D7: blame the imposing parent

#[test]
fn a_child_without_a_scheme_blames_the_soft_parent_and_replanning_succeeds() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let (solver, picker) = parts();
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    // A registry without structured surface schemes: nothing can serve the linking faces of a
    // sweep (or the faces of a mapped volume), so their parents are blamed one after another.
    let mut reg = sm_scheme_api::SchemeRegistry::new();
    for s in [
        Box::new(stub_schemes::Equal) as Box<dyn Scheme>,
        Box::new(stub_schemes::Pave),
        Box::new(stub_schemes::Tri),
        Box::new(stub_schemes::Sweep),
        Box::new(stub_schemes::MapVol),
        Box::new(stub_schemes::TetVol),
    ] {
        reg.register(s).expect("register");
    }
    let driver = Driver::new(&reg, &solver, &picker);
    let mut intent = MeshIntent::default();
    let mut rej = Rejections::default();
    let mut blamed = Vec::new();
    let plan = loop {
        match driver.plan_with(&input, &intent, &rej) {
            Ok(p) => break p,
            Err(e) => {
                let (t, s) = e.blame().unwrap_or_else(|| panic!("no blame for {e}"));
                blamed.push((t, s.as_str().to_owned()));
                rej.insert(Attempt {
                    target: t,
                    scheme: s,
                    phase: Phase::Resolve,
                    kind: FailureKind::NotApplicable,
                    message: e.to_string(),
                });
            }
        }
    };
    let vol = Target::Volume(ids.volume);
    assert_eq!(
        blamed,
        vec![(vol, "sweep".to_owned()), (vol, "mapvol".to_owned())]
    );
    let tp = plan.target(vol).expect("volume");
    assert_eq!(
        (tp.scheme.as_str(), &tp.selection),
        ("tetvol", &Selection::Auto { score: 10 })
    );
    assert_eq!(plan.rejections.len(), 2);
    // A hard parent is never blamed: the error stands.
    intent.set_scheme(ids.volume, SchemeChoice::new("mapvol", Firmness::Hard));
    match driver.plan(&input, &intent) {
        Err(e @ SchemeError::NoApplicableScheme { .. }) => assert_eq!(e.blame(), None),
        other => panic!("expected NoApplicableScheme, got {:?}", other.map(|_| ())),
    }
}

// ---------------------------------------------------------------- D13: polyhedral cells

#[test]
fn a_polyhedral_cell_is_committed_through_add_polyhedron() {
    let (topo, ids, model) = brick_setup([1.0, 2.0, 3.0]);
    let reg = registry();
    let (solver, picker) = parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput {
        topo: &topo,
        model: &model,
        pieces: &WholeFaces,
        selection: &sel,
    };
    let mut intent = MeshIntent::default();
    intent.set_scheme(ids.volume, SchemeChoice::new("polyvol", Firmness::Hard));
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .expect("mesh");
    let patch = out.patches.get(Target::Volume(ids.volume)).expect("patch");
    assert_eq!(
        patch.elements.get(0).map(|e| e.0),
        Some(ElemShape::Polyhedron)
    );
    let faces: Vec<&[u32]> = patch.elements.polyhedron_faces(0).expect("faces").collect();
    assert_eq!(faces.len(), 6);
    assert!(faces.iter().all(|f| f.len() == 4));
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 1);
    assert!((cell_volume(m, CellIx(0)) - 6.0).abs() < 1e-12);
    assert_eq!(m.classified_on(ids.volume).map(|s| s.cells.len()), Some(1));
    for f in &ids.faces {
        assert_eq!(m.classified_on(*f).map(|s| s.faces.len()), Some(1));
    }
}

// ---------------------------------------------------------------- patch API (D11, D12)

fn e(k: EntityKind, n: u64) -> EntityId {
    EntityId::new(k, n).expect("id")
}

#[test]
fn elements_and_grids_return_options_instead_of_panicking() {
    let mut el = Elements::new();
    assert!(el.is_empty());
    el.push(ElemShape::Tri, &[0, 1, 2]);
    el.push_polyhedron(&[&[0, 1, 2], &[0, 2, 3], &[0, 3, 1], &[1, 3, 2]]);
    assert_eq!(el.len(), 2);
    assert_eq!(el.get(0), Some((ElemShape::Tri, &[0u32, 1, 2][..])));
    assert_eq!(el.get(2), None);
    assert!(el.polyhedron_faces(0).is_none(), "a triangle has no faces");
    assert!(el.polyhedron_faces(5).is_none());
    assert_eq!(el.polyhedron_faces(1).expect("faces").count(), 4);
    assert_eq!(el.iter().count(), 2);

    let k = |i: u32| NodeKey::new(e(EntityKind::TopoFace, 1), i);
    let g = NodeGrid::new(3, 2, (0..6).map(k).collect());
    assert_eq!(g.at(2, 1), Some(k(5)));
    assert_eq!(g.at(3, 0), None);
    assert_eq!(g.at(0, 2), None);
    assert_eq!(g.position(k(4)), Some((1, 1)));
    assert_eq!(
        g.position(NodeKey::new(e(EntityKind::TopoFace, 2), 0)),
        None
    );
    // Boundary node in the middle of a side: the line crosses the grid; corners have none.
    assert_eq!(g.line_from(k(1)), Some(vec![k(1), k(4)]));
    assert_eq!(g.line_from(k(4)), Some(vec![k(4), k(1)]));
    assert_eq!(g.line_from(k(0)), None);
    // A malformed grid is not an error path: it just has no positions.
    assert_eq!(NodeGrid::new(3, 3, vec![k(0)]).position(k(0)), None);
}

#[test]
fn patch_builder_dedups_references_and_renumbers() {
    let t = Target::Surface(e(EntityKind::TopoFace, 1));
    let (v0, v1) = (
        NodeKey::vertex(e(EntityKind::TopoVertex, 0)),
        NodeKey::vertex(e(EntityKind::TopoVertex, 1)),
    );
    let mut b = PatchBuilder::new(t);
    let r0 = b.refer(v0);
    let n0 = b.node(
        NodeKey::new(e(EntityKind::TopoFace, 1), 0),
        Point3::new(0.5, 0.5, 0.0),
        Param::None,
    );
    let r1 = b.refer(v1);
    assert_eq!(b.refer(v0), r0, "the same key gives the same index");
    assert_ne!(r0, r1);
    let n1 = b.node(
        NodeKey::new(e(EntityKind::TopoFace, 1), 1),
        Point3::new(0.6, 0.5, 0.0),
        Param::None,
    );
    b.owner(e(EntityKind::TopoFace, 1));
    b.push(ElemShape::Tri, &[n0, r0, r1]);
    b.push(ElemShape::Tri, &[n1, r1, r0]);
    let p: Patch = b.finish();
    assert_eq!((p.nodes.len(), p.refs.len(), p.local_len()), (2, 2, 4));
    // References are numbered after the new nodes.
    assert_eq!(p.elements.get(0), Some((ElemShape::Tri, &[0u32, 2, 3][..])));
    assert_eq!(p.elements.get(1), Some((ElemShape::Tri, &[1u32, 3, 2][..])));
    assert_eq!(
        (p.key(0), p.key(2), p.key(3), p.key(4)),
        (Some(p.nodes[0].key), Some(v0), Some(v1), None)
    );
    assert_eq!(p.owners.len(), 2);
}

// ---------------------------------------------------------------- linear interval rows

#[test]
fn linear_rows_with_coefficients_and_inequalities() {
    let ed = |n: u64| e(EntityKind::TopoEdge, n);
    let edges = [ed(1), ed(2)];
    let rec = |id: u32, item| Record {
        id: ConstraintId(id),
        origin: Origin::Intent,
        item,
    };
    let solve =
        |cs: Vec<Record<IntervalConstraint>>| ReferenceIntervalSolver::default().solve(&edges, &cs);
    // 2a + b == 9 with a hard 3: b == 3.
    let r = solve(vec![
        rec(
            0,
            IntervalConstraint::Fixed {
                edge: ed(1),
                count: 3,
                firmness: Firmness::Hard,
            },
        ),
        rec(
            1,
            IntervalConstraint::Linear {
                terms: vec![(ed(1), 2), (ed(2), 1)],
                rel: Rel::Eq,
                rhs: 9,
            },
        ),
    ])
    .expect("solve");
    assert_eq!(r.iter().map(|x| x.1).collect::<Vec<_>>(), vec![3, 3]);
    // a + b <= 5 with goals 4 and 4: pulled down to the bound.
    let goals = |extra| {
        vec![
            rec(
                0,
                IntervalConstraint::Goal {
                    edge: ed(1),
                    goal: 4.0,
                },
            ),
            rec(
                1,
                IntervalConstraint::Goal {
                    edge: ed(2),
                    goal: 4.0,
                },
            ),
            rec(2, extra),
        ]
    };
    let r = solve(goals(IntervalConstraint::Linear {
        terms: vec![(ed(1), 1), (ed(2), 1)],
        rel: Rel::Le,
        rhs: 5,
    }))
    .expect("le");
    assert!(
        r.iter().map(|x| x.1).sum::<u32>() <= 5 && r.iter().all(|x| x.1 >= 1),
        "{r:?}"
    );
    // a + b >= 12 with goals 4 and 4: pushed up.
    let r = solve(goals(IntervalConstraint::Linear {
        terms: vec![(ed(1), 1), (ed(2), 1)],
        rel: Rel::Ge,
        rhs: 12,
    }))
    .expect("ge");
    assert!(r.iter().map(|x| x.1).sum::<u32>() >= 12, "{r:?}");
    // Already satisfied inequalities change nothing.
    let r = solve(goals(IntervalConstraint::Linear {
        terms: vec![(ed(1), 1), (ed(2), 1)],
        rel: Rel::Le,
        rhs: 100,
    }))
    .expect("slack");
    assert_eq!(r.iter().map(|x| x.1).collect::<Vec<_>>(), vec![4, 4]);
    // Two hard counts against a bound they break: infeasible, naming the row.
    let err = solve(vec![
        rec(
            0,
            IntervalConstraint::Fixed {
                edge: ed(1),
                count: 3,
                firmness: Firmness::Hard,
            },
        ),
        rec(
            1,
            IntervalConstraint::Fixed {
                edge: ed(2),
                count: 3,
                firmness: Firmness::Hard,
            },
        ),
        rec(
            2,
            IntervalConstraint::Linear {
                terms: vec![(ed(1), 1), (ed(2), 1)],
                rel: Rel::Le,
                rhs: 5,
            },
        ),
    ])
    .expect_err("infeasible");
    assert!(
        err.constraints.contains(&ConstraintId(2)) && err.blame.is_none(),
        "{err:?}"
    );
    // `sum_equal` cancels an edge on both sides and sorts its terms.
    assert_eq!(
        IntervalConstraint::sum_equal(&[ed(2), ed(1)], &[ed(2)]),
        IntervalConstraint::Linear {
            terms: vec![(ed(1), 1)],
            rel: Rel::Eq,
            rhs: 0
        }
    );
    assert_eq!(
        IntervalConstraint::sum_equal(&[ed(1)], &[ed(2)]).edges(),
        vec![ed(1), ed(2)]
    );
}

// ---------------------------------------------------------------- geometry helpers

#[test]
fn geometry_helpers_on_the_brick() {
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    let loops = geom::face_loops(&topo, ids.faces[4]);
    assert_eq!((loops.len(), loops[0].len()), (1, 4));
    for c in geom::corners(&topo, ids.faces[4]).expect("corners") {
        assert!((geom::corner_angle(&topo, c.key).expect("angle") - FRAC_PI_2).abs() < 1e-9);
    }
    // A point off the plane snaps onto it, with a surface parameter.
    let (p, param) =
        geom::snap_to_face(&topo, ids.faces[4], Point3::new(0.25, 0.75, 0.3)).expect("snap");
    assert!(p.z.abs() < 1e-12 && (p.x - 0.25).abs() < 1e-12 && matches!(param, Param::Surface(_)));
    let uses = geom::volume_face_uses(&topo, ids.volume);
    assert_eq!(uses.len(), 6);
    assert!(
        uses.iter()
            .all(|(f, s)| ids.faces.contains(f) && *s == Side::Left)
    );
    assert_eq!(topo.volume_faces(ids.volume).len(), 6);
}

// ---------------------------------------------------------------- executors

#[test]
fn executors_run_every_job_exactly_once() {
    use sm_scheme_api::{Executor, ScopedThreads};
    use std::sync::atomic::{AtomicUsize, Ordering};
    for exec in [
        &Sequential as &dyn Executor,
        &ScopedThreads(1),
        &ScopedThreads(3),
        &ScopedThreads(64),
    ] {
        for n in [0usize, 1, 5, 17] {
            let hits: Vec<AtomicUsize> = (0..n).map(|_| AtomicUsize::new(0)).collect();
            exec.run(n, &|i| {
                hits[i].fetch_add(1, Ordering::SeqCst);
            });
            assert!(
                hits.iter().all(|h| h.load(Ordering::SeqCst) == 1),
                "n = {n}"
            );
        }
    }
}
