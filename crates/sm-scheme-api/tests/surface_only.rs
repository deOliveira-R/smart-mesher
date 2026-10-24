//! Surface-only meshes (W0e decision 5): a selection of faces is meshed into a dimension-2
//! builder; plus the curve rule on claimed edges and the sizing-function error.

mod stub_schemes;

use sm_core::EntityId;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{
    CellIx, CellShape, FaceIx, FaceKind, MeshDim, MeshRead, SerialAlloc, cell_volume, validate,
};
use sm_scheme_api::geom;
use sm_scheme_api::{
    Applicability, Caps, Driver, ElemShape, Firm, Firmness, GenCtx, GenerateFailure,
    IntervalRequest, MeshInput, MeshIntent, NodeKey, Params, Patch, PatchBuilder, PlanCtx,
    ReferenceCornerPicker, ReferenceIntervalSolver, Requirements, Scheme, SchemeChoice, SchemeDim,
    SchemeError, Sequential, SizingSpec, Target, WholeFaces,
};
use sm_topo_api::{TopologyView, TopologyViewExt};
use stub_schemes::{brick_setup, registry};

fn counts(ids: &sm_topo_api::fixtures::BrickIds, n: [u32; 3]) -> MeshIntent {
    let mut intent = MeshIntent::default();
    for (e, c) in [(ids.edges[0], n[0]), (ids.edges[4], n[1]), (ids.edges[8], n[2])] {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(c)));
    }
    intent
}

#[test]
fn the_faces_of_a_brick_mesh_as_one_closed_surface() {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = ids.faces;
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let out = driver
        .mesh(&input, &counts(&ids, [2, 3, 4]), &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new())
        .expect("surface mesh");
    let m = &out.finished.mesh;
    assert_eq!(m.dim(), MeshDim::Two);
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    // Quads: -x/+x 3 x 4, -y/+y 2 x 4, -z/+z 2 x 3.
    assert_eq!(m.cell_count(), 2 * (12 + 8 + 6));
    assert!((0..m.cell_count()).all(|c| m.cell_shape(CellIx::new(c)) == CellShape::Quad));
    // A closed surface: every facet has two cells, and V - E + F = 2.
    assert!((0..m.face_count()).all(|f| m.face_kind(FaceIx::new(f)) == FaceKind::Interior));
    assert_eq!(m.vertex_count() as i64 - m.face_count() as i64 + m.cell_count() as i64, 2);
    // Total area 2 (1.5 * 2 + 1 * 2 + 1 * 1.5) = 13.
    let area: f64 = (0..m.cell_count()).map(|c| cell_volume(m, CellIx::new(c))).sum();
    assert!((area - 13.0).abs() < 1e-12, "{area}");
    // Cells lie on their faces; facets on the brick edges lie on those edges.
    for f in ids.faces {
        assert!(m.classified_on(f).is_some_and(|c| !c.cells.is_empty()));
    }
    for e in ids.edges {
        let n = out.plan.intervals(e).unwrap() as usize;
        assert_eq!(m.classified_on(e).map(|c| c.faces.len()), Some(n), "edge {e}");
    }
    // Every cell follows its face's outward normal (the brick's faces all have the volume on
    // their left), so the polygon normals point out of the brick.
    for c in 0..m.cell_count() {
        let ci = CellIx::new(c);
        let cyc = m.cell_corners(ci).unwrap();
        let p: Vec<_> = cyc.iter().map(|v| m.point(v.get())).collect();
        let n = (p[1] - p[0]).cross(p[2] - p[0]);
        let face = m.cell_class(ci).entity;
        let want = geom::face_normal(&topo, face, p[0]).unwrap();
        assert!(n.dot(want) > 0.0, "cell {c} on {face}");
    }
    // Deterministic across executors.
    let again = driver
        .mesh(&input, &counts(&ids, [2, 3, 4]), &sm_scheme_api::ScopedThreads(4), ReferenceBuilder::surface(), &mut SerialAlloc::new())
        .expect("surface mesh");
    assert_eq!(sm_mesh_api::topology_hash(&again.finished.mesh), sm_mesh_api::topology_hash(m));
    assert_eq!(sm_mesh_api::geometry_hash(&again.finished.mesh), sm_mesh_api::geometry_hash(m));
}

#[test]
fn one_face_is_an_open_surface_and_selections_cannot_mix() {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.faces[4]];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let out = driver
        .mesh(&input, &counts(&ids, [2, 3, 4]), &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new())
        .expect("one face");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.cell_count(), 6);
    let boundary = (0..m.face_count()).filter(|f| m.face_kind(FaceIx::new(*f)) == FaceKind::Boundary).count();
    assert_eq!(boundary, 2 * (2 + 3));
    let area: f64 = (0..6).map(|c| cell_volume(m, CellIx::new(c))).sum();
    assert!((area - 1.5).abs() < 1e-12);
    // Faces and pieces cannot be mixed.
    let mixed = [ids.faces[4], ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &mixed);
    assert!(matches!(driver.plan(&input, &counts(&ids, [2, 3, 4])), Err(SchemeError::Input(_))));
    // A volume selection cannot be committed into a dimension-2 builder.
    let vsel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &vsel);
    let mut it = counts(&ids, [2, 3, 4]);
    it.set_scheme(ids.volume, SchemeChoice::with_params("sweep", serde_json::json!({ "source": [ids.faces[4]], "target": [ids.faces[5]] }), Firmness::Hard));
    match driver.mesh(&input, &it, &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new()) {
        Err(SchemeError::InvalidPatch { reason, .. }) => assert!(reason.contains("dimension-2"), "{reason}"),
        other => panic!("expected InvalidPatch, got {:?}", other.map(|_| ())),
    }
}

#[test]
fn a_sizing_function_without_a_size_field_is_an_error() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.faces[4]];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Function { name: "curvature".into(), params: Params::default() });
    match driver.plan(&input, &intent) {
        Err(SchemeError::Input(r)) => assert!(r.contains("needs a SizeField"), "{r}"),
        other => panic!("expected a sizing error, got {:?}", other.map(|_| ())),
    }
    intent.defaults.sizing = Some(SizingSpec::Constant { size: -1.0 });
    assert!(matches!(driver.plan(&input, &intent), Err(SchemeError::Input(_))));
    // With an explicit size field the function is fine.
    let field = sm_scheme_api::ConstantSize(0.5);
    let mut d2 = Driver::new(&reg, &solver, &picker);
    d2.size = Some(&field);
    intent.defaults.sizing = Some(SizingSpec::Function { name: "curvature".into(), params: Params::default() });
    let plan = d2.plan(&input, &intent).expect("plan with a size field");
    assert_eq!(plan.intervals(ids.edges[0]), Some(2));
}

/// A surface scheme that claims the first edge of its face and writes it itself (nodes and
/// segments), meshing the face as one polygon. `offset` misnumbers the edge nodes.
struct ClaimEdge;

fn claimed_edge(t: &dyn TopologyView, f: EntityId) -> EntityId {
    t.face_edges(f)[0]
}

impl Scheme for ClaimEdge {
    fn name(&self) -> &str {
        "claimedge"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::POLY
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(1, "test claimer")
    }
    fn requirements(&self, cx: &PlanCtx<'_>, target: Target, _p: &Params, out: &mut Requirements<'_>) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.claim(Target::Curve(claimed_edge(cx.input.topo, f)));
        }
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, p: &Params) -> Result<Patch, GenerateFailure> {
        let fatal = |r: String| GenerateFailure::fatal(target, "claimedge", r);
        let Target::Surface(f) = target else { return Err(fatal("not a face".into())) };
        let t = cx.input.topo;
        let e = claimed_edge(t, f);
        let off: u32 = p.get::<u32>("claimedge", "offset").ok().flatten().unwrap_or(0);
        let n = cx.intervals(e).ok_or_else(|| fatal("no intervals".into()))?;
        let (a, b) = t.edge_ends(e).ok_or_else(|| fatal("ends".into()))?;
        let mut pb = PatchBuilder::new(target);
        let mut chain = vec![pb.refer(NodeKey::vertex(a))];
        let mut own = std::collections::BTreeMap::new();
        for i in 1..n {
            let k = NodeKey::new(e, i + off);
            let q = geom::edge_point(t, e, f64::from(i) / f64::from(n)).map_err(|x| fatal(x.to_string()))?;
            let l = pb.node(k, q, sm_mesh_api::Param::None);
            own.insert(NodeKey::new(e, i), l);
            chain.push(l);
        }
        chain.push(pb.refer(NodeKey::vertex(b)));
        let lp = cx.loops(f).map_err(|x| fatal(x.to_string()))?;
        let poly: Vec<u32> = lp[0].nodes.iter().map(|k| own.get(k).copied().unwrap_or_else(|| pb.refer(*k))).collect();
        pb.push(ElemShape::Polygon, &poly);
        for w in chain.windows(2) {
            pb.owner(e);
            pb.push(ElemShape::Segment, w);
        }
        // Owners: one per element (the polygon belongs to the face itself).
        let mut patch = pb.finish();
        let mut owners = vec![f];
        owners.extend(std::iter::repeat_n(e, chain.len() - 1));
        patch.owners = owners;
        Ok(patch)
    }
}

#[test]
fn a_claimed_edge_is_split_out_and_checked_against_the_curve_rule() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let mut reg = registry();
    reg.register(Box::new(ClaimEdge)).unwrap();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let face = ids.faces[4];
    let e = claimed_edge(&topo, face);
    let sel = [face];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    intent.set_intervals(e, Firm::hard(IntervalRequest::Count(3)));
    intent.set_scheme(face, SchemeChoice::new("claimedge", Firmness::Hard));
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new()).expect("claimed edge");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.cell_count(), 1);
    assert_eq!(m.cell_shape(CellIx(0)), CellShape::Polygon);
    assert_eq!(m.classified_on(e).map(|c| c.faces.len()), Some(3), "the split curve patch classifies its facets");
    assert!(out.patches.get(Target::Curve(e)).is_some(), "the claimed edge became a patch of its own");
    // Misnumbered edge nodes (2..=3 instead of 1..=2) are refused.
    intent.set_scheme(face, SchemeChoice::with_params("claimedge", serde_json::json!({ "offset": 1 }), Firmness::Hard));
    match driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::surface(), &mut SerialAlloc::new()) {
        Err(SchemeError::InvalidPatch { reason, .. }) => assert!(reason.contains("needs nodes"), "{reason}"),
        other => panic!("expected InvalidPatch, got {:?}", other.map(|_| ())),
    }
}
