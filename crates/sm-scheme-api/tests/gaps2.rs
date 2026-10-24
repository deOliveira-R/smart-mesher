//! W0f gaps 2: embedded conformity, transforms over faces and curves, user interval rows and
//! the commit node correspondence.

mod stub_schemes;

use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, Interval, Line};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{CellShape, MeshBuild, MeshRead, SerialAlloc, VertexIx, validate};
use sm_model_api::{BoundaryKind, Deck, Model};
use sm_scheme_api::{
    Applicability, Caps, Driver, ElemShape, Firm, Firmness, GenCtx, GenerateFailure,
    IntervalConstraint, IntervalRequest, Job, MeshInput, MeshIntent, NodeKey, Params, Patch,
    PatchBuilder, PlanCtx, ReferenceCornerPicker, ReferenceIntervalSolver, Rel, Requirements,
    Scheme, SchemeChoice, SchemeDim, SchemeError, Sequential, Target, TransformId,
    TransformIntent, WholeFaces,
};
use sm_topo_api::fixtures::{BrickIds, with_op};
use sm_topo_api::{CellComplex, CurveCarrier, OpId, Sense, TopologyEdit, TopologyView, TopologyViewExt};
use stub_schemes::{brick_setup, registry};

fn parts() -> (ReferenceIntervalSolver, ReferenceCornerPicker) {
    (ReferenceIntervalSolver::default(), ReferenceCornerPicker)
}

fn counts(ids: &BrickIds, n: [u32; 3]) -> MeshIntent {
    let mut intent = MeshIntent::default();
    for (e, c) in [(ids.edges[0], n[0]), (ids.edges[4], n[1]), (ids.edges[8], n[2])] {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(c)));
    }
    intent
}

// ------------------------------------------------------------------ embedded members

/// A unit brick whose `-z` face carries an embedded diagonal (between two opposite corners of
/// its loop) and, optionally, an embedded interior point. Returns the diagonal and the point.
fn brick_with_wire(point: bool) -> (CellComplex, BrickIds, EntityId, Option<EntityId>) {
    let (mut topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    let face = ids.faces[4];
    let lv: Vec<EntityId> = topo.loop_vertices(face, 0).collect();
    let (a, c) = (lv[0], lv[2]);
    let (pa, pc) = (sm_scheme_api::geom::vertex_point(&topo, a).unwrap(), sm_scheme_api::geom::vertex_point(&topo, c).unwrap());
    let (diag, pt) = with_op(&mut topo, OpId(2), |t| {
        let line = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(pa, pc - pa).unwrap())).unwrap();
        let e = t.make_edge(a, c, Some(CurveCarrier { curve: line, range: Interval::new(0.0, (pc - pa).norm()), sense: Sense::Forward }))?;
        t.embed(face, e)?;
        let p = if point {
            let v = t.make_vertex(Point3::new(0.6, 0.3, 0.0))?;
            t.embed(face, v)?;
            Some(v)
        } else {
            None
        };
        Ok((e, p))
    })
    .unwrap();
    (topo, ids, diag, pt)
}

/// Meshes a face with one embedded edge between two loop vertices as the two polygons the
/// edge splits it into, through `GenCtx::embedded` (it ignores embedded points).
struct SplitPoly;

impl Scheme for SplitPoly {
    fn name(&self) -> &str {
        "splitpoly"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::POLY
    }
    fn applicability(&self, cx: &PlanCtx<'_>, t: Target, _p: &Params) -> Applicability {
        match t {
            Target::Surface(f) if cx.input.embedded_edges(f).len() == 1 => Applicability::yes(1, "one embedded edge"),
            _ => Applicability::no("needs exactly one embedded edge"),
        }
    }
    fn requirements(&self, cx: &PlanCtx<'_>, t: Target, _p: &Params, out: &mut Requirements<'_>) -> sm_scheme_api::Result<()> {
        // Embedded curves are children a requirement may name.
        if let Target::Surface(f) = t {
            for e in cx.input.embedded_edges(f) {
                out.require_caps(Target::Curve(e), Caps::NONE, Firmness::Soft);
            }
        }
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        let fatal = |r: String| GenerateFailure::fatal(target, "splitpoly", r);
        let Target::Surface(f) = target else { return Err(fatal("not a face".into())) };
        let lp = cx.loops(f).map_err(|e| fatal(e.to_string()))?;
        let ring = &lp[0].nodes;
        let chains = cx.embedded(f).map_err(|e| fatal(e.to_string()))?;
        let chain = chains.iter().find(|c| c.len() > 1).ok_or_else(|| fatal("no chain".into()))?;
        let (s, e) = (chain[0], chain[chain.len() - 1]);
        let i = ring.iter().position(|k| *k == s).ok_or_else(|| fatal("start off the loop".into()))?;
        let j = ring.iter().position(|k| *k == e).ok_or_else(|| fatal("end off the loop".into()))?;
        let n = ring.len();
        let walk = |from: usize, to: usize| -> Vec<NodeKey> {
            let mut v = Vec::new();
            let mut k = from;
            loop {
                v.push(ring[k]);
                if k == to {
                    break;
                }
                k = (k + 1) % n;
            }
            v
        };
        let inner: Vec<NodeKey> = chain[1..chain.len() - 1].to_vec();
        // Loop i -> j, then back along the chain; loop j -> i, then forward along the chain.
        let mut p1 = walk(i, j);
        p1.extend(inner.iter().rev().copied());
        let mut p2 = walk(j, i);
        p2.extend(inner.iter().copied());
        let mut pb = PatchBuilder::new(target);
        for poly in [p1, p2] {
            let l: Vec<u32> = poly.iter().map(|k| pb.refer(*k)).collect();
            pb.push(ElemShape::Polygon, &l);
        }
        Ok(pb.finish())
    }
}

#[test]
fn embedded_edges_get_curve_targets_and_faces_must_conform() {
    let (topo, ids, diag, _) = brick_with_wire(false);
    let model = Model::new();
    let mut reg = registry();
    reg.register(Box::new(SplitPoly)).unwrap();
    let (solver, picker) = parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let face = ids.faces[4];
    let sel = [face];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    assert_eq!(input.embedded_edges(face), vec![diag]);
    let mut intent = counts(&ids, [2, 2, 2]);
    intent.set_intervals(diag, Firm::hard(IntervalRequest::Count(3)));
    intent.set_scheme(face, SchemeChoice::new("splitpoly", Firmness::Hard));
    let plan = driver.plan(&input, &intent).expect("plan");
    assert_eq!(plan.intervals(diag), Some(3), "the embedded edge is an interval variable");
    assert!(plan.target(Target::Curve(diag)).is_some(), "and a curve target");
    // The face job reads the embedded curve, so it runs in a later wave.
    let wave_of = |j: Job| plan.waves.iter().position(|w| w.contains(&j)).unwrap();
    assert!(wave_of(Job::Generate(Target::Curve(diag))) < wave_of(Job::Generate(Target::Surface(face))));
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new()).expect("conforming mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.cell_count(), 2);
    assert!((0..2).all(|c| m.cell_shape(sm_mesh_api::CellIx::new(c)) == CellShape::Polygon));
    assert_eq!(m.classified_on(diag).map(|c| c.faces.len()), Some(3), "the facets along the wire lie on it");
    // GenCtx::embedded: one chain in the edge's own direction.
    let cx = GenCtx { input: &input, plan: &out.plan, patches: &out.patches, size: None };
    let (a, c) = topo.edge_ends(diag).unwrap();
    assert_eq!(cx.embedded(face).unwrap(), vec![vec![NodeKey::vertex(a), NodeKey::new(diag, 1), NodeKey::new(diag, 2), NodeKey::vertex(c)]]);
    // A scheme that ignores the wire is refused instead of silently not conforming.
    intent.set_scheme(face, SchemeChoice::new("tri", Firmness::Hard));
    match driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new()) {
        Err(SchemeError::InvalidPatch { reason, .. }) => assert!(reason.contains("embedded"), "{reason}"),
        other => panic!("expected InvalidPatch, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn embedded_points_get_vertex_nodes_the_face_must_reference() {
    let (topo, ids, _diag, pt) = brick_with_wire(true);
    let pt = pt.unwrap();
    let model = Model::new();
    let mut reg = registry();
    reg.register(Box::new(SplitPoly)).unwrap();
    let (solver, picker) = parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let face = ids.faces[4];
    let sel = [face];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    assert_eq!(input.embedded_vertices(face), vec![pt]);
    let mut intent = counts(&ids, [2, 2, 2]);
    intent.set_scheme(face, SchemeChoice::new("splitpoly", Firmness::Hard));
    let plan = driver.plan(&input, &intent).unwrap();
    assert!(plan.problem.vertices.contains(&pt), "the point gets a vertex node");
    let ps = driver.generate(&input, &plan, &Sequential);
    match ps {
        Err(SchemeError::InvalidPatch { reason, .. }) => assert!(reason.contains(&format!("{:?}", NodeKey::vertex(pt))), "{reason}"),
        other => panic!("expected InvalidPatch, got {:?}", other.map(|_| ())),
    }
}

// ------------------------------------------------------------------ transforms on faces and curves

#[test]
fn a_surface_transform_runs_before_the_sweep_that_reads_it() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = counts(&ids, [2, 2, 2]);
    intent.set_scheme(ids.volume, SchemeChoice::with_params("sweep", serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }), Firmness::Hard));
    intent.transforms.push(TransformIntent {
        id: TransformId(1),
        targets: vec![ids.faces[4]],
        transform: SchemeChoice::with_params("lift", serde_json::json!({ "dz": 0.01 }), Firmness::Hard),
    });
    let plan = driver.plan(&input, &intent).expect("plan");
    assert_eq!(plan.transforms[0].targets, vec![Target::Surface(ids.faces[4])]);
    let wave_of = |j: Job| plan.waves.iter().position(|w| w.contains(&j)).unwrap();
    let (tr, vol) = (Job::Transform(TransformId(1)), Job::Generate(Target::Volume(ids.volume)));
    assert!(wave_of(Job::Generate(Target::Surface(ids.faces[4]))) < wave_of(tr));
    assert!(wave_of(tr) < wave_of(vol), "the volume reads the transformed face");
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    // The source face's interior node was lifted, and the volume mesh uses it.
    let key = out.patches.get(Target::Surface(ids.faces[4])).unwrap().nodes[0].key;
    assert_eq!(key.owner, ids.faces[4]);
    let v = out.vertex_of(key).unwrap();
    assert!((m.point(v.get()).z - 0.01).abs() < 1e-15);
    // A target outside the region is refused.
    intent.transforms[0].targets = vec![EntityId::new(sm_core::EntityKind::TopoFace, 999).unwrap()];
    assert!(matches!(driver.plan(&input, &intent), Err(SchemeError::Input(_))));
}

#[test]
fn a_copy_of_a_transformed_root_is_generated_from_the_transformed_patch() {
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    // Translation deck -x -> +x: the +x face is a generated copy of the -x face.
    let mut model = Model::new();
    let bd = model.add_boundary("px", BoundaryKind::Deck(Deck::Translation { vector: Vector3::new(1.0, 0.0, 0.0) })).unwrap();
    model.assign_pairs(bd, &[(ids.faces[0], ids.faces[1])]).unwrap();
    let reg = registry();
    let (solver, picker) = parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.faces[0], ids.faces[1]];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = counts(&ids, [2, 2, 2]);
    intent.set_scheme(ids.faces[0], SchemeChoice::new("map", Firmness::Hard));
    intent.transforms.push(TransformIntent {
        id: TransformId(3),
        targets: vec![ids.faces[0]],
        transform: SchemeChoice::with_params("lift", serde_json::json!({ "dz": 0.05 }), Firmness::Hard),
    });
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new()).expect("mesh");
    let root = out.patches.get(Target::Surface(ids.faces[0])).unwrap();
    let copy = out.patches.get(Target::Surface(ids.faces[1])).unwrap();
    assert!(!root.nodes.is_empty());
    for n in &root.nodes {
        let img = copy.nodes.iter().find(|c| c.key.index == n.key.index).unwrap();
        assert!((img.point.z - n.point.z).abs() < 1e-15 && (img.point.x - n.point.x - 1.0).abs() < 1e-15);
        assert!((n.point.z - 0.5 - 0.05).abs() < 1e-15, "the root's interior node is lifted");
    }
}

#[test]
fn a_curve_transform_is_read_by_its_faces() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let face = ids.faces[4];
    let sel = [face];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let e = topo.face_edges(face)[0];
    let mut intent = counts(&ids, [2, 2, 2]);
    intent.transforms.push(TransformIntent { id: TransformId(1), targets: vec![e], transform: SchemeChoice::with_params("lift", serde_json::json!({ "dz": 0.0 }), Firmness::Hard) });
    intent.transforms.push(TransformIntent { id: TransformId(2), targets: vec![e], transform: SchemeChoice::new("noop", Firmness::Hard) });
    let plan = driver.plan(&input, &intent).unwrap();
    let wave_of = |j: Job| plan.waves.iter().position(|w| w.contains(&j)).unwrap();
    let (t1, t2, f) = (Job::Transform(TransformId(1)), Job::Transform(TransformId(2)), Job::Generate(Target::Surface(face)));
    assert!(wave_of(t1) < wave_of(t2), "transforms over one target chain in id order");
    assert!(wave_of(t2) < wave_of(f), "the face waits for the last transform over its edge");
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new()).unwrap();
    assert!(validate(&out.finished.mesh).is_empty());
}

// ------------------------------------------------------------------ interval rows

#[test]
fn user_interval_rows_enter_the_solve() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let face = ids.faces[2]; // -y: x edges and z edges
    let sel = [face];
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = MeshIntent::default();
    intent.set_intervals(ids.edges[8], Firm::hard(IntervalRequest::Count(2)));
    // count(x edge) = 2 * count(z edge), written with a repeated term that canonicalizes.
    intent.intervals.push(IntervalConstraint::Linear { terms: vec![(ids.edges[8], -1), (ids.edges[0], 1), (ids.edges[8], -1)], rel: Rel::Eq, rhs: 0 });
    let plan = driver.plan(&input, &intent).expect("plan");
    assert_eq!(plan.intervals(ids.edges[0]), Some(4));
    let row = plan.problem.intervals.iter().find(|r| r.origin == sm_scheme_api::Origin::Intent && matches!(r.item, IntervalConstraint::Linear { .. })).unwrap();
    assert_eq!(row.item, IntervalConstraint::Linear { terms: vec![(ids.edges[0], 1), (ids.edges[8], -2)], rel: Rel::Eq, rhs: 0 });
    let mut c = intent.clone();
    c.canonicalize();
    assert_eq!(c.intervals[0], row.item);
    // check(): bad rows are refused.
    for bad in [
        IntervalConstraint::Linear { terms: vec![(ids.edges[0], 0)], rel: Rel::Eq, rhs: 0 },
        IntervalConstraint::Linear { terms: vec![], rel: Rel::Eq, rhs: 0 },
        IntervalConstraint::Linear { terms: vec![(ids.faces[0], 1)], rel: Rel::Eq, rhs: 1 },
        IntervalConstraint::Linear { terms: vec![(ids.edges[0], 1)], rel: Rel::Le, rhs: i64::MAX },
        IntervalConstraint::Fixed { edge: ids.edges[0], count: 0, firmness: Firmness::Hard },
        IntervalConstraint::Range { edge: ids.edges[0], min: 3, max: Some(2) },
        IntervalConstraint::Goal { edge: ids.edges[0], goal: f64::NAN },
        IntervalConstraint::EvenSum { edges: vec![] },
    ] {
        let mut i = MeshIntent::default();
        i.intervals.push(bad.clone());
        assert!(matches!(i.check(), Err(SchemeError::Input(_))), "{bad:?}");
    }
    // An edge outside the region is refused by the planner.
    let mut far = MeshIntent::default();
    far.intervals.push(IntervalConstraint::sum_equal(&[ids.edges[0]], &[ids.edges[4]]));
    assert!(matches!(driver.plan(&input, &far), Err(SchemeError::Input(_))), "edges[4] is not on the -y face");
    // Serde: the field defaults, and struct literals keep compiling through Default.
    let parsed: MeshIntent = serde_json::from_value(serde_json::json!({})).unwrap();
    assert!(parsed.intervals.is_empty());
    let lit = MeshIntent { intervals: vec![IntervalConstraint::EvenSum { edges: vec![ids.edges[0]] }], ..Default::default() };
    let back: MeshIntent = serde_json::from_str(&serde_json::to_string(&lit).unwrap()).unwrap();
    assert_eq!(back, lit);
}

// ------------------------------------------------------------------ commit correspondence

#[test]
fn node_i_becomes_vertex_i() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = parts();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = ids.faces;
    let input = MeshInput { topo: &topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let intent = counts(&ids, [2, 3, 2]);
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new()).unwrap();
    let m = &out.finished.mesh;
    for (i, (k, p, _)) in out.patches.nodes().enumerate() {
        assert_eq!(out.patches.index_of(k), Some(i as u32));
        assert_eq!(out.vertex_of(k), Some(VertexIx(i as u32)));
        assert_eq!(m.point(i), p);
    }
    assert_eq!(out.vertex_of(NodeKey::new(ids.volume, 7)), None);
    // A builder that already holds vertices breaks the correspondence: refused.
    let mut b = ReferenceBuilder::surface();
    b.add_vertex(Point3::new(9.0, 9.0, 9.0)).unwrap();
    assert!(matches!(driver.commit(&input, &out.patches, b, &mut SerialAlloc::new()), Err(SchemeError::Input(_))));
}
