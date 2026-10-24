//! Copy classes (defects D2 to D4, D6): the deck x sweep matrix, redundant constraints that
//! disagree, capability impositions through `copy`, copies contributed late, and the
//! "face after volume" schedule.

use sm_scheme_api::stubs as stub_schemes;

use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{MeshEntityKind, MeshRead, SerialAlloc, validate};
use sm_model_api::{BoundaryKind, Deck, Model};
use sm_scheme_api::{
    Applicability, Caps, CopyConstraint, CopyMode, CopyReason, Driver, Firmness, GenCtx,
    GenerateFailure, GroupId, Job, MeshGroup, MeshInput, MeshIntent, Need, Origin, Params, Patch,
    PlanCtx, ReferenceCornerPicker, ReferenceIntervalSolver, Requirements, Scheme, SchemeChoice,
    SchemeDim, SchemeError, Selection, Sequential, Target, WholeFaces,
};
use sm_topo_api::fixtures::{BrickIds, brick, merge_touching_bricks, with_op};
use sm_topo_api::{CellComplex, Coincidence, OpId, TopologyEdit, TopologyViewExt};
use stub_schemes::{brick_setup, registry};

fn sweep_z(ids: &BrickIds) -> SchemeChoice {
    SchemeChoice::with_params(
        "sweep",
        serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }),
        Firmness::Hard,
    )
}

fn deck_model(a: EntityId, b: EntityId, v: Vector3) -> (Model, EntityId) {
    let mut model = Model::new();
    let bd = model
        .add_boundary(
            "periodic",
            BoundaryKind::Deck(Deck::Translation { vector: v }),
        )
        .expect("deck");
    model.assign_pairs(bd, &[(a, b)]).expect("pair");
    (model, bd)
}

/// Row of the matrix: a translation deck pair whose faces are the sweep's source and target,
/// in either order.
fn deck_sweep_matrix_row(a_is_source: bool) {
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    let (a, b, v) = if a_is_source {
        (ids.faces[4], ids.faces[5], Vector3::new(0.0, 0.0, 1.0))
    } else {
        (ids.faces[5], ids.faces[4], Vector3::new(0.0, 0.0, -1.0))
    };
    let (model, bd) = deck_model(a, b, v);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    intent.set_scheme(ids.volume, sweep_z(&ids));
    let out = driver
        .mesh(
            &input,
            &intent,
            &Sequential,
            ReferenceBuilder::new(),
            &mut SerialAlloc::new(),
        )
        .unwrap_or_else(|e| panic!("row a_is_source={a_is_source}: {e}"));
    let plan = &out.plan;
    // One class: the sweep source is the root, whatever the deck direction.
    let (class, member) = plan
        .copy_of(Target::Surface(ids.faces[5]))
        .expect("target is a member");
    assert_eq!(
        (class.root, member.mode),
        (Target::Surface(ids.faces[4]), CopyMode::Generate)
    );
    assert_eq!(
        plan.target(Target::Surface(ids.faces[4]))
            .map(|t| t.scheme.as_str()),
        Some("map")
    );
    // Both constraints are in the problem; one is the tree edge and the other only a check.
    let on_pair: Vec<_> = plan
        .problem
        .copies
        .iter()
        .filter(|c| matches!(c.item.target, Target::Surface(_)))
        .collect();
    assert_eq!(on_pair.len(), 2, "the sweep copy and the deck copy");
    assert!(
        on_pair
            .iter()
            .any(|c| c.item.reason == CopyReason::Deck { boundary: bd })
    );
    assert!(
        on_pair
            .iter()
            .any(|c| matches!(c.item.reason, CopyReason::SweepTarget { .. }))
    );
    assert_eq!(member.constraints.len(), 1);
    // The member's placement is the pure translation by one.
    let p = member.from_root.apply_point(Point3::new(0.25, 0.5, 0.0));
    assert!((p - Point3::new(0.25, 0.5, 1.0)).norm() < 1e-12);
    // The copied face has identical element connectivity and the mesh is valid.
    let (pa, pb) = (
        out.patches
            .get(Target::Surface(ids.faces[4]))
            .expect("source"),
        out.patches
            .get(Target::Surface(ids.faces[5]))
            .expect("target"),
    );
    assert_eq!(pa.elements.len(), pb.elements.len());
    assert!(validate(&out.finished.mesh).is_empty());
    assert_eq!(out.finished.mesh.count(MeshEntityKind::Cell), 1);
}

#[test]
fn deck_pair_equal_to_the_sweep_source_and_target_is_one_class() {
    // Middle row of the matrix: A = sweep source, B = sweep target (was CopyConflict).
    deck_sweep_matrix_row(true);
}

#[test]
fn deck_pair_opposite_to_the_sweep_is_one_class_without_a_cycle() {
    // Bottom row: A = sweep target, B = sweep source (was a dependency cycle).
    deck_sweep_matrix_row(false);
}

#[test]
fn redundant_copy_constraints_that_disagree_are_a_conflict() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    intent.set_scheme(ids.volume, sweep_z(&ids));
    // A user copy of the target from the source by a half turn about the x axis: it maps the
    // square onto the target too, but with another vertex correspondence than the translation.
    let half_turn = Transform::rotation(
        Point3::new(0.0, 0.5, 0.5),
        Vector3::new(1.0, 0.0, 0.0),
        core::f64::consts::PI,
    )
    .expect("axis");
    intent.set_scheme(
        ids.faces[5],
        SchemeChoice::with_params(
            "copy",
            serde_json::json!({ "source": ids.faces[4], "transform": half_turn }),
            Firmness::Soft,
        ),
    );
    match driver.plan(&input, &intent) {
        Err(SchemeError::CopyConflict { target, reason, .. }) => {
            assert_eq!(target, Target::Surface(ids.faces[5]));
            assert!(
                reason.contains("maps to") || reason.contains("apart"),
                "{reason}"
            );
        }
        other => panic!("expected CopyConflict, got {:?}", other.map(|_| ())),
    }
}

/// A volume scheme that needs structured quads on one face only.
struct OneSided;

impl Scheme for OneSided {
    fn name(&self) -> &str {
        "onesided"
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
        _cx: &PlanCtx<'_>,
        _target: Target,
        p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        let face: EntityId = p.get("onesided", "face")?.expect("face param");
        out.require_caps(
            Target::Surface(face),
            Caps::QUAD | Caps::STRUCTURED,
            Firmness::Hard,
        );
        Ok(())
    }
    fn generate(&self, _cx: &GenCtx<'_>, t: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        Err(GenerateFailure::fatal(t, "onesided", "plan only"))
    }
}

#[test]
fn an_imposition_on_a_copy_member_constrains_the_class_root() {
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    // Deck pair -x -> +x. The volume only demands quad/structured on +x, the copy.
    let (model, _) = deck_model(ids.faces[0], ids.faces[1], Vector3::new(1.0, 0.0, 0.0));
    let mut reg = registry();
    reg.register(Box::new(OneSided)).expect("register");
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    intent.set_scheme(
        ids.volume,
        SchemeChoice::with_params(
            "onesided",
            serde_json::json!({ "face": ids.faces[1] }),
            Firmness::Hard,
        ),
    );
    // A soft pave on the root would give the copy the wrong capabilities: it is overridden.
    intent.set_scheme(ids.faces[0], SchemeChoice::new("pave", Firmness::Soft));
    let plan = driver.plan(&input, &intent).expect("plan");
    let root = plan.target(Target::Surface(ids.faces[0])).expect("root");
    assert_eq!(root.scheme.as_str(), "map", "{:?}", root);
    assert!(matches!(root.selection, Selection::Imposed { .. }));
    let copy = plan.target(Target::Surface(ids.faces[1])).expect("copy");
    assert!(
        copy.caps.contains(Caps::QUAD | Caps::STRUCTURED),
        "INHERIT resolves through the class root"
    );
    assert!(
        plan.attempts
            .iter()
            .any(|a| a.target == Target::Surface(ids.faces[0]) && a.scheme.as_str() == "pave")
    );
    // A hard pave on the root contradicts the imposition on the copy.
    intent.set_scheme(ids.faces[0], SchemeChoice::new("pave", Firmness::Hard));
    match driver.plan(&input, &intent) {
        Err(SchemeError::ImpositionConflict {
            child,
            imposed_by,
            required,
            ..
        }) => {
            assert_eq!(
                (child, imposed_by),
                (Target::Surface(ids.faces[1]), Target::Volume(ids.volume))
            );
            assert_eq!(required, Need::Caps(Caps::QUAD | Caps::STRUCTURED));
        }
        other => panic!("expected ImpositionConflict, got {:?}", other.map(|_| ())),
    }
}

/// A surface scheme that, while planning, declares its two x-parallel edges copies of each
/// other. Contributed after the closure ran for the volume: defect D4.
struct EdgeCopy;

impl Scheme for EdgeCopy {
    fn name(&self) -> &str {
        "edgecopy"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::QUAD | Caps::STRUCTURED
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(500, "test")
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else {
            return Ok(());
        };
        let t = cx.input.topo;
        // The two edges of the face that run along x.
        let xs: Vec<EntityId> = t
            .face_edges(f)
            .iter()
            .copied()
            .filter(|e| {
                let (a, b) = t.edge_ends(*e).expect("ends");
                let (pa, pb) = (
                    sm_scheme_api::geom::vertex_point(t, a).expect("a"),
                    sm_scheme_api::geom::vertex_point(t, b).expect("b"),
                );
                (pb - pa).x.abs() > 0.5
            })
            .collect();
        let [e1, e2] = xs.as_slice() else {
            return Ok(());
        };
        let shift = sm_scheme_api::geom::vertex_point(t, t.edge_ends(*e2).expect("ends").0)
            .expect("p")
            - sm_scheme_api::geom::vertex_point(t, t.edge_ends(*e1).expect("ends").0).expect("p");
        out.copy(CopyConstraint::new(
            Target::Curve(*e1),
            Target::Curve(*e2),
            Transform::translation(shift),
            CopyReason::User,
        ));
        out.vertex_rule(f, sm_scheme_api::VertexRule::Map);
        Ok(())
    }
    fn interval_constraints(
        &self,
        cx: &sm_scheme_api::IntervalCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut sm_scheme_api::IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else {
            return Ok(());
        };
        let layout = cx.layout(f).expect("layout");
        let edges = |k: usize| -> Vec<EntityId> {
            layout.sides[k].coedges.iter().map(|c| c.edge).collect()
        };
        out.equal(&edges(0), &edges(2));
        out.equal(&edges(1), &edges(3));
        Ok(())
    }
    fn generate(&self, _cx: &GenCtx<'_>, t: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        Err(GenerateFailure::fatal(t, "edgecopy", "plan only"))
    }
}

#[test]
fn copies_contributed_by_a_face_scheme_are_closed_and_classed() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let mut reg = registry();
    reg.register(Box::new(EdgeCopy)).expect("register");
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    let face = ids.faces[4]; // -z: edges 0 and 1 run along x
    intent.set_scheme(ids.volume, SchemeChoice::new("mapvol", Firmness::Hard));
    intent.set_scheme(face, SchemeChoice::new("edgecopy", Firmness::Hard));
    for f in ids.faces.iter().filter(|f| **f != face) {
        intent.set_scheme(*f, SchemeChoice::new("map", Firmness::Hard));
    }
    intent.set_intervals(
        ids.edges[0],
        sm_scheme_api::Firm::soft(sm_scheme_api::IntervalRequest::Count(3)),
    );
    let plan = driver.plan(&input, &intent).expect("plan");
    let (class, member) = plan
        .copy_of(Target::Curve(ids.edges[1]))
        .expect("the second edge became a member");
    assert_eq!(
        (class.root, member.mode),
        (Target::Curve(ids.edges[0]), CopyMode::Generate)
    );
    assert_eq!(
        plan.target(Target::Curve(ids.edges[1]))
            .map(|t| t.scheme.as_str()),
        Some("copy")
    );
    // The closure added the vertex correspondence and the interval equality (origin: driver).
    let rec = plan
        .problem
        .copy_into(Target::Curve(ids.edges[1]))
        .expect("constraint");
    assert_eq!(rec.item.vertex_map.len(), 2);
    assert_eq!(plan.intervals(ids.edges[0]), plan.intervals(ids.edges[1]));
    assert_eq!(plan.intervals(ids.edges[0]), Some(3));
    assert!(
        plan.problem
            .intervals
            .iter()
            .any(|r| r.origin == Origin::Driver)
    );
}

/// Four volumes: a row of three merged bricks swept as a group (which claims the +x end), and
/// a separate brick whose -x face is a deck copy of that claimed end.
#[test]
fn a_deck_copy_of_a_claimed_face_is_scheduled_after_the_group() {
    let mut topo = CellComplex::new();
    let (row, d) = with_op(&mut topo, OpId(1), |t| {
        let mut v: Vec<BrickIds> = Vec::new();
        for i in 0..3 {
            v.push(brick(t, Point3::new(i as f64, 0.0, 0.0), [1.0, 1.0, 1.0])?);
        }
        for w in v.windows(2) {
            t.record_coincidence(
                w[0].faces[1],
                w[1].faces[0],
                Coincidence::CoincidentUnimprinted,
            )?;
        }
        for w in v.windows(2) {
            merge_touching_bricks(t, &w[0], &w[1])?;
        }
        let d = brick(t, Point3::new(5.0, 0.0, 0.0), [1.0, 1.0, 1.0])?;
        Ok((v, d))
    })
    .expect("fixture");
    // C.+x (x = 3) -> D.-x (x = 5).
    let (model, _) = deck_model(row[2].faces[1], d.faces[0], Vector3::new(2.0, 0.0, 0.0));
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let mut sel: Vec<EntityId> = row.iter().map(|b| b.volume).collect();
    sel.push(d.volume);
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    let params = serde_json::json!({ "source": [row[0].faces[0]], "target": [row[2].faces[1]] });
    intent.groups.push(MeshGroup::new(GroupId(1), row.iter().map(|b| b.volume).collect()).with_scheme(SchemeChoice::with_params("msweep", params, Firmness::Hard)));
    intent.set_scheme(
        d.volume,
        SchemeChoice::with_params(
            "sweep",
            serde_json::json!({ "source": [d.faces[2]], "target": [d.faces[3]] }),
            Firmness::Hard,
        ),
    );
    let plan = driver.plan(&input, &intent).expect("plan");

    let (g, claimed, copy, dvol) = (
        Target::Group(GroupId(1)),
        Target::Surface(row[2].faces[1]),
        Target::Surface(d.faces[0]),
        Target::Volume(d.volume),
    );
    assert_eq!(plan.claimed_by(claimed), Some(g));
    // The claimed face is the class root (nothing else in the class is a sweep source), and
    // D's -x face, a linking face of D's sweep, is generated as its image.
    let (class, member) = plan.copy_of(copy).expect("member");
    assert_eq!((class.root, member.mode), (claimed, CopyMode::Generate));
    assert!(
        plan.target(copy)
            .expect("copy plan")
            .caps
            .contains(Caps::QUAD | Caps::STRUCTURED),
        "the claimer's caps satisfy the sweep's imposition on its linking face"
    );
    // "Face after volume": group, then the copy, then D's own volume.
    let wave_of = |j: Job| {
        plan.waves
            .iter()
            .position(|w| w.contains(&j))
            .expect("scheduled")
    };
    assert!(wave_of(Job::Generate(g)) < wave_of(Job::Generate(copy)));
    assert!(wave_of(Job::Generate(copy)) < wave_of(Job::Generate(dvol)));
    let fp = plan
        .footprints
        .iter()
        .find(|f| f.job == Job::Generate(copy))
        .expect("footprint");
    assert!(
        fp.reads.contains(&claimed),
        "the copy reads its (claimed) root: {fp:?}"
    );
}
