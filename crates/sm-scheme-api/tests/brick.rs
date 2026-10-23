//! End-to-end: equal curves, mapped faces and a 1-to-1 sweep on the sm-topo-api brick fixture,
//! written into the sm-mesh-api reference builder.

mod common;
use common::*;
use sm_core::{EntityKind, Point3, Vector3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{FaceKind, MeshBuild, MeshRead, Param, SerialAlloc, cell_volume, validate};
use sm_model_api::{BoundaryKind, Deck, Model, PieceRef, Plane};
use sm_scheme_api::*;
use sm_topo_api::{Dim, TopologyView};
use std::collections::BTreeSet;

fn finish(out: &MeshOutcome, shape: &dyn ShapeView) -> sm_mesh_api::reference::ReferenceMesh {
    let mut b = ReferenceBuilder::new();
    emit(&out.mesh, shape, &mut b).expect("emit");
    let mut alloc = SerialAlloc::new();
    let m = b.finish(&mut alloc).expect("finish").mesh;
    let v = validate(&m);
    assert!(v.is_empty(), "validation: {v:?}");
    m
}

#[test]
fn sweep_brick_end_to_end() {
    let (t, ids) = brick_complex();
    let shape = TopoShape::new(&t);
    let reg = registry();
    let intent = sweep_intent(&ids);
    let out = driver(&reg, &shape, &intent).run(&[key(ids.volume)], &Sequential).expect("mesh");

    // Intervals: 4 along x and y (size 0.25), 3 along z (hard count propagated by the sweep).
    for (i, e) in ids.edges.iter().enumerate() {
        assert_eq!(out.plan.intervals.get(key(*e)), Some(if i >= 8 { 3 } else { 4 }), "edge {i}");
    }
    // Roles: -z source (map), +z target (claimed by the sweep), sides linking (map).
    let plan = &out.plan;
    assert_eq!(plan.job_of(key(ids.faces[5])).map(|j| j.plan.scheme.as_str()), Some("volume.sweep"));
    for f in &ids.faces[..5] {
        assert_eq!(plan.job_of(key(*f)).map(|j| j.plan.scheme.as_str()), Some("surface.map"));
    }
    assert!(plan.imposed_on(key(ids.faces[0])).iter().any(|r| r.req == Requirement::Role(Role::Linking)));
    assert!(plan.imposed_on(key(ids.faces[4])).iter().any(|r| r.req == Requirement::Role(Role::Source)));
    assert_eq!(out.mesh.cell_count(), 48);
    assert_eq!(out.mesh.node_count(), 5 * 5 * 4);

    let m = finish(&out, &shape);
    assert_eq!(m.count(sm_mesh_api::MeshEntityKind::Cell), 48);
    let vol: f64 = (0..48).map(|c| cell_volume(&m, sm_mesh_api::CellIx::new(c))).sum();
    assert!((vol - 1.0).abs() < 1e-12, "volume {vol}");
    // Boundary faces are classified on the six topology faces: 2 x 16 + 4 x 12.
    let mut boundary = 0;
    for f in 0..m.count(sm_mesh_api::MeshEntityKind::Face) {
        let fi = sm_mesh_api::FaceIx::new(f);
        if m.face_kind(fi) == FaceKind::Boundary {
            boundary += 1;
            let c = m.classification(sm_mesh_api::MeshEntityKind::Face, f as u32);
            assert_eq!(c.entity.kind(), EntityKind::TopoFace);
        }
    }
    assert_eq!(boundary, 80);
    // Curve nodes carry their curve parameter.
    let on_edge = (0..m.count(sm_mesh_api::MeshEntityKind::Vertex))
        .map(|v| m.classification(sm_mesh_api::MeshEntityKind::Vertex, v as u32))
        .filter(|c| c.entity.kind() == EntityKind::TopoEdge)
        .inspect(|c| assert!(matches!(c.param, Param::Curve(_))))
        .count();
    assert_eq!(on_edge, 8 * 3 + 4 * 2);
}

#[test]
fn auto_selection_picks_sweep_and_map() {
    let (t, ids) = brick_complex();
    let shape = TopoShape::new(&t);
    let reg = registry();
    let mut intent = MeshIntent::new();
    intent.default_size = Some(0.5);
    let out = driver(&reg, &shape, &intent).run(&[key(ids.volume)], &Sequential).expect("mesh");
    assert_eq!(out.mesh.cell_count(), 8);
    assert_eq!(out.plan.job_of(key(ids.volume)).map(|j| j.plan.scheme.as_str()), Some("volume.sweep"));
    // First valid pair in ascending order: -x -> +x.
    assert_eq!(out.plan.job_of(key(ids.faces[1])).map(|j| j.plan.scheme.as_str()), Some("volume.sweep"));
    finish(&out, &shape);
}

#[test]
fn fallback_chain_replans_after_apply_failure() {
    let (t, ids) = brick_complex();
    let shape = TopoShape::new(&t);
    let reg = registry();
    let mut intent = sweep_intent(&ids);
    intent.set_scheme(key(ids.volume), SchemeChoice::Chain { schemes: vec![SchemeSpec::new("test.fail"), SchemeSpec::new("volume.sweep")], firmness: Firmness::Soft });
    let out = driver(&reg, &shape, &intent).run(&[key(ids.volume)], &Sequential).expect("mesh");
    assert!(out.attempts.iter().any(|a| a.scheme.as_str() == "test.fail" && a.phase == Phase::Apply && a.kind == FailureKind::Quality));
    assert_eq!(out.plan.job_of(key(ids.volume)).map(|j| j.plan.scheme.as_str()), Some("volume.sweep"));
    finish(&out, &shape);

    // A hard chain with only the failing scheme reports every attempt.
    intent.set_scheme(key(ids.volume), SchemeChoice::hard(SchemeSpec::new("test.fail")));
    match driver(&reg, &shape, &intent).run(&[key(ids.volume)], &Sequential) {
        Err(DriveError::NoScheme { target, attempts }) => {
            assert_eq!(target, key(ids.volume));
            assert!(attempts.iter().any(|a| a.phase == Phase::Apply));
        }
        other => panic!("expected NoScheme, got {other:?}"),
    }
}

#[test]
fn map_reports_vertex_types_and_side_constraints() {
    let (t, ids) = brick_complex();
    let shape = TopoShape::new(&t);
    let reg = registry();
    let intent = sweep_intent(&ids);
    let d = driver(&reg, &shape, &intent);
    let plan = d.plan(&[key(ids.volume)], &BTreeSet::new()).expect("plan");
    let job = plan.job_of(key(ids.faces[0])).expect("job");
    assert_eq!(job.plan.vertex_types.len(), 4);
    assert!(job.plan.vertex_types.iter().all(|a| a.vertex_type == VertexType::End && a.firmness == Firmness::Soft));
    assert_eq!(job.plan.intervals.len(), 2);
    assert!(job.caps.contains(Caps::QUAD | Caps::STRUCTURED));

    // A hard Side vertex type on one corner leaves only 3 End vertices: map is not applicable,
    // and nothing else is registered for faces.
    let corner = job.plan.vertex_types[0].corner;
    let mut intent2 = intent.clone();
    intent2.entry(key(ids.faces[0])).vertex_types.push(CornerAssignment { corner, vertex_type: VertexType::Side, firmness: Firmness::Hard });
    match driver(&reg, &shape, &intent2).plan(&[key(ids.volume)], &BTreeSet::new()) {
        Err(DriveError::NoScheme { target, attempts }) => {
            assert_eq!(target, key(ids.faces[0]));
            assert!(attempts.iter().any(|a| a.phase == Phase::Applicability && a.message.contains("4 End")));
        }
        other => panic!("expected NoScheme, got {other:?}"),
    }
}

#[test]
fn hard_interval_conflict_is_reported() {
    let (t, ids) = brick_complex();
    let shape = TopoShape::new(&t);
    let reg = registry();
    let mut intent = sweep_intent(&ids);
    intent.set_size(key(ids.edges[11]), SizeSpec::Intervals { n: 5, firmness: Firmness::Hard });
    match driver(&reg, &shape, &intent).run(&[key(ids.volume)], &Sequential) {
        Err(DriveError::Intervals(IntervalError::HardConflict { a, b, .. })) => assert_eq!((a.min(b), a.max(b)), (3, 5)),
        other => panic!("expected a hard conflict, got {other:?}"),
    }
}

#[test]
fn deck_pair_forces_copy_and_mirror_pins_plane() {
    let (t, ids) = brick_complex();
    let mut model = Model::new();
    let periodic = model.add_boundary("px", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).expect("deck");
    model.assign_pairs(periodic, &[(ids.faces[0], ids.faces[1])]).expect("pair");
    let mirror = model.add_boundary("ym", BoundaryKind::Deck(Deck::Mirror { plane: Plane::new(Vector3::new(0.0, -1.0, 0.0), 0.0).expect("plane") })).expect("mirror");
    model.assign_self_paired(mirror, &[ids.faces[2]]).expect("self");
    let shape = TopoShape::with_model(&t, &model);
    let reg = registry();
    let intent = sweep_intent(&ids);
    let out = driver(&reg, &shape, &intent).run(&[key(ids.volume)], &Sequential).expect("mesh");
    let plan = &out.plan;

    // +x is a copy of -x (a mapped face), which still satisfies the sweep's STRUCTURED need.
    let bx = plan.job_of(key(ids.faces[1])).expect("job");
    assert_eq!(bx.plan.scheme.as_str(), SURFACE_COPY);
    assert!(bx.caps.contains(Caps::QUAD | Caps::STRUCTURED));
    assert!(plan.imposed_on(key(ids.faces[0])).iter().any(|r| r.req == Requirement::Forbid { caps: Caps::BOUNDARY_LAYER }));
    assert!(plan.imposed_on(key(ids.faces[2])).iter().any(|r| matches!(r.req, Requirement::OnPlane { .. })));
    // The four curves of +x are copies; z-edges at x = 1 inherit the hard 3.
    for e in [ids.edges[9], ids.edges[11], ids.edges[5], ids.edges[7]] {
        assert_eq!(plan.job_of(key(e)).map(|j| j.plan.scheme.as_str()), Some(CURVE_COPY), "{e}");
    }
    assert_eq!(plan.intervals.get(key(ids.edges[11])), Some(3));
    // Every node of +x is the translate of a node of -x.
    let a = out.mesh.surface(key(ids.faces[0])).expect("-x");
    let b = out.mesh.surface(key(ids.faces[1])).expect("+x");
    for n in &b.faces.nodes {
        let p = out.mesh.point(*n);
        assert!(a.faces.nodes.iter().any(|m| (out.mesh.point(*m) + Vector3::new(1.0, 0.0, 0.0) - p).norm() < 1e-12));
    }
    finish(&out, &shape);
}

#[test]
fn whole_piece_target_is_the_volume() {
    let (t, ids) = brick_complex();
    let shape = TopoShape::new(&t);
    let reg = registry();
    let intent = sweep_intent(&ids);
    let a = driver(&reg, &shape, &intent).run(&[ShapeKey::Piece(PieceRef::Whole(ids.volume))], &Sequential).expect("piece");
    let b = driver(&reg, &shape, &intent).run(&[key(ids.volume)], &Sequential).expect("volume");
    assert_eq!(a.mesh, b.mesh);
    assert_eq!(t.dim_of(ids.volume), Some(Dim::Volume));
}

#[test]
fn intent_and_plan_round_trip_through_json() {
    let (t, ids) = brick_complex();
    let shape = TopoShape::new(&t);
    let reg = registry();
    let intent = sweep_intent(&ids);
    let text = serde_json::to_string(&intent).expect("ser");
    let back: MeshIntent = serde_json::from_str(&text).expect("de");
    assert_eq!(back, intent);
    back.check().expect("check");
    let plan = driver(&reg, &shape, &intent).plan(&[key(ids.volume)], &BTreeSet::new()).expect("plan");
    for j in &plan.jobs {
        let s = serde_json::to_string(&j.plan).expect("ser plan");
        let p: Plan = serde_json::from_str(&s).expect("de plan");
        assert_eq!(p, j.plan);
    }
    let s = serde_json::to_string(&plan.problem).expect("ser problem");
    assert_eq!(serde_json::from_str::<IntervalProblem>(&s).expect("de problem"), plan.problem);
}

/// Splits nothing: replaces each hex by itself. Used to check transform atomicity.
struct Identity {
    break_it: bool,
}

impl MeshTransform for Identity {
    fn id(&self) -> &str {
        if self.break_it { "test.broken" } else { "test.identity" }
    }
    fn caps(&self) -> Caps {
        Caps::HEX
    }
    fn applicability(&self, _cx: &PlanCtx<'_>) -> Applicability {
        Applicability::yes(1.0, "always")
    }
    fn apply(&self, _shape: &dyn ShapeView, mesh: &CommittedMesh, targets: &[ShapeKey]) -> Result<TransformPatch, SchemeError> {
        let mut p = TransformPatch::default();
        for c in mesh.volumes().filter(|c| c.targets.iter().any(|t| targets.contains(t))) {
            for i in 0..c.len() {
                let nodes: Vec<PatchRef> = c.cell(i).iter().map(|n| PatchRef::Old(*n)).collect();
                p.volume.cells.push(c.shape[i], &nodes, 0);
            }
        }
        if self.break_it {
            p.volume.cells.push(sm_mesh_api::CellShape::Hex, &[PatchRef::New(7); 8], 0);
        }
        Ok(p)
    }
}

#[test]
fn transforms_commit_all_or_nothing() {
    let (t, ids) = brick_complex();
    let shape = TopoShape::new(&t);
    let mut reg = registry();
    reg.register_transform(std::sync::Arc::new(Identity { break_it: false })).expect("reg");
    reg.register_transform(std::sync::Arc::new(Identity { break_it: true })).expect("reg");
    let intent = sweep_intent(&ids);
    let d = driver(&reg, &shape, &intent);
    let mut mesh = d.run(&[key(ids.volume)], &Sequential).expect("mesh").mesh;
    let before = mesh.clone();
    assert!(matches!(d.transform(&mut mesh, &SchemeId::new("test.broken"), &[key(ids.volume)]), Err(DriveError::BadPatch { .. })));
    assert_eq!(mesh, before, "a failed transform changes nothing");
    d.transform(&mut mesh, &SchemeId::new("test.identity"), &[key(ids.volume)]).expect("identity");
    assert_eq!(mesh.cell_count(), before.cell_count());
    assert_eq!(mesh.volumes().next().map(|c| c.scheme.as_str()), Some("test.identity"));
}

#[test]
fn reference_solver_handles_sums_and_parity() {
    let k = |s: u64| ShapeKey::Cell(sm_core::EntityId::new(EntityKind::TopoEdge, s).expect("id"));
    let problem = IntervalProblem {
        curves: vec![k(1), k(2), k(3)],
        constraints: vec![
            SourcedConstraint { constraint: IntervalConstraint::Count { curve: k(1), n: 2, firmness: Firmness::Hard }, by: k(1), scheme: None, why: "hard".into() },
            SourcedConstraint { constraint: IntervalConstraint::Count { curve: k(2), n: 3, firmness: Firmness::Hard }, by: k(2), scheme: None, why: "hard".into() },
            SourcedConstraint { constraint: IntervalConstraint::sides_equal(&[k(1), k(2)], &[k(3)]), by: k(3), scheme: None, why: "submap side".into() },
            SourcedConstraint { constraint: IntervalConstraint::Even { curves: vec![k(1), k(2), k(3)] }, by: k(3), scheme: None, why: "pave".into() },
        ],
    };
    let s = SimpleSolver.solve(&problem).expect("solve");
    assert_eq!((s.get(k(1)), s.get(k(2)), s.get(k(3))), (Some(2), Some(3), Some(5)));
    assert!(problem.constraints.iter().all(|c| c.constraint.is_satisfied(&s)));
    let _ = Point3::new(0.0, 0.0, 0.0);
}
