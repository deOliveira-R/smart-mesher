//! Acceptance 10 and 11 (adapter part): the scheme through `sm_scheme_api::Driver`.
//!
//! The registry is built by hand (decision 12: never combine `stubs::registry()` with a real
//! crate that reuses a stub name): the stub `equal` curve scheme, a test-local triangle surface
//! scheme (`tritest`, `Caps::TRI`, splits the structured grid of a four-sided face) and the real
//! `tetmesh`.

use sm_core::{EntityId, Point3};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{CellIx, CellShape, FaceIx, MeshEntityKind, MeshRead, SerialAlloc, VertexIx, cell_volume, validate};
use sm_scheme_api::geom::{snap_to_face, vertex_point};
use sm_scheme_api::stubs::{Equal, brick_setup};
use sm_scheme_api::{
    Applicability, Caps, Driver, ElemShape, Executor, FailureKind, Firmness, GenCtx, GenerateFailure, IntervalCtx, IntervalSink, MeshInput, MeshIntent, NodeKey, Params,
    Patch, PatchBuilder, PlanCtx, ReferenceCornerPicker, ReferenceIntervalSolver, Requirements, Scheme, SchemeChoice, SchemeDim, SchemeError, SchemeRegistry,
    ScopedThreads, Sequential, SizingSpec, Target, VertexRule, WholeFaces,
};
use sm_scheme_tet::{NAME, TetVolume};
use sm_topo_api::fixtures::{brick_with_baffle, brick_with_void, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyEdit, TopologyView};
use std::collections::BTreeSet;

type GResult<T> = Result<T, GenerateFailure>;

/// Test-local triangle scheme: the structured grid of a four-sided face, each cell split along
/// alternating diagonals. With `broken_face` set, the interior node of that face is pushed far
/// into the volume so that its triangles cut through the solid.
struct TriTest {
    broken_face: Option<EntityId>,
    /// Non-zero: emit the same triangles in a scrambled element order, with rotated corners and
    /// reversed node creation order (the patch is the same mesh, listed differently).
    shuffle: u64,
}

impl Scheme for TriTest {
    fn name(&self) -> &str {
        "tritest"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::TRI
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, target: Target, _p: &Params) -> Applicability {
        match target {
            Target::Surface(_) => Applicability::yes(30, "four-sided grid split into triangles"),
            _ => Applicability::no("not a surface"),
        }
    }
    fn requirements(&self, _cx: &PlanCtx<'_>, target: Target, _p: &Params, out: &mut Requirements<'_>) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.vertex_rule(f, VertexRule::Map);
        }
        Ok(())
    }
    fn interval_constraints(&self, cx: &IntervalCtx<'_>, target: Target, _p: &Params, out: &mut IntervalSink<'_>) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else { return Ok(()) };
        let layout = cx.layout(f).ok_or_else(|| SchemeError::Input(format!("{f} has no layout")))?;
        if layout.sides.len() != 4 {
            return Err(SchemeError::Input(format!("{f}: {} sides", layout.sides.len())));
        }
        let edges = |k: usize| -> Vec<EntityId> { layout.sides[k].coedges.iter().map(|c| c.edge).collect() };
        out.equal(&edges(0), &edges(2));
        out.equal(&edges(1), &edges(3));
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        let fatal = |r: String| GenerateFailure::fatal(target, "tritest", r);
        let Target::Surface(f) = target else { return Err(fatal("not a surface".into())) };
        let side = |k: usize| cx.side_nodes(f, k).ok_or_else(|| fatal(format!("side {k} unavailable")));
        let (s0, s1, s2, s3) = (side(0)?, side(1)?, side(2)?, side(3)?);
        let (ni, nj) = (s0.len(), s1.len());
        let mut keys = vec![NodeKey::new(f, u32::MAX); ni * nj];
        for i in 0..ni {
            keys[i] = s0[i];
            keys[(nj - 1) * ni + i] = s2[ni - 1 - i];
        }
        for j in 0..nj {
            keys[j * ni + ni - 1] = s1[j];
            keys[j * ni] = s3[nj - 1 - j];
        }
        let pt = |k: NodeKey| cx.point(k).ok_or_else(|| fatal(format!("node {k:?} missing")));
        let mut pb = PatchBuilder::new(target);
        let mut idx = vec![0u32; ni * nj];
        let order: Vec<(usize, usize)> = {
            let mut o: Vec<(usize, usize)> = (0..nj).flat_map(|j| (0..ni).map(move |i| (i, j))).collect();
            if self.shuffle != 0 {
                o.reverse();
            }
            o
        };
        let (p00, p10, p01, p11) = (pt(keys[0])?, pt(keys[ni - 1])?, pt(keys[(nj - 1) * ni])?, pt(keys[nj * ni - 1])?);
        {
            for &(i, j) in &order {
                let at = j * ni + i;
                if i == 0 || j == 0 || i == ni - 1 || j == nj - 1 {
                    idx[at] = pb.refer(keys[at]);
                    continue;
                }
                let (u, v) = (i as f64 / (ni - 1) as f64, j as f64 / (nj - 1) as f64);
                let lerp = |a: Point3, b: Point3, t: f64| a + (b - a) * t;
                let mut p = lerp(lerp(p00, p10, u), lerp(p01, p11, u), v);
                if self.broken_face == Some(f) && i == ni / 2 && j == nj / 2 {
                    // Push one node far into the volume: its triangles cut through the solid.
                    let c = lerp(lerp(p00, p10, 0.5), lerp(p01, p11, 0.5), 0.5);
                    p = c + (Point3::new(5.0, 5.0, 5.0) - c) * 3.0;
                }
                let (p, param) = if self.broken_face == Some(f) { (p, sm_mesh_api::Param::None) } else { snap_to_face(cx.input.topo, f, p).map_err(|e| GenerateFailure::new(target, "tritest", FailureKind::Geometry, e.to_string()))? };
                let key = NodeKey::new(f, ((j - 1) * (ni - 2) + (i - 1)) as u32);
                keys[at] = key;
                idx[at] = pb.node(key, p, param);
            }
        }
        let mut tris: Vec<[u32; 3]> = Vec::new();
        for j in 0..nj - 1 {
            for i in 0..ni - 1 {
                let (a, b, c, d) = (idx[j * ni + i], idx[j * ni + i + 1], idx[(j + 1) * ni + i + 1], idx[(j + 1) * ni + i]);
                if (i + j) % 2 == 0 {
                    tris.push([a, b, c]);
                    tris.push([a, c, d]);
                } else {
                    tris.push([a, b, d]);
                    tris.push([b, c, d]);
                }
            }
        }
        if self.shuffle != 0 {
            let mix = |x: u64| {
                let mut z = x.wrapping_add(0x9E37_79B9_7F4A_7C15).wrapping_mul(self.shuffle | 1);
                z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
                z ^ (z >> 27)
            };
            let mut keyed: Vec<(u64, [u32; 3])> = tris.iter().enumerate().map(|(k, t)| (mix(k as u64), *t)).collect();
            keyed.sort_unstable_by_key(|x| x.0);
            tris = keyed.into_iter().map(|(h, t)| { let r = (h % 3) as usize; [t[r], t[(r + 1) % 3], t[(r + 2) % 3]] }).collect();
        }
        for t in &tris {
            pb.push(ElemShape::Tri, t);
        }
        Ok(pb.finish())
    }
}

fn registry(broken: Option<EntityId>) -> SchemeRegistry {
    registry_shuffled(broken, 0)
}

fn registry_shuffled(broken: Option<EntityId>, shuffle: u64) -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    r.register(Box::new(Equal)).unwrap();
    r.register(Box::new(TriTest { broken_face: broken, shuffle })).unwrap();
    sm_scheme_tet::register(&mut r).unwrap();
    r
}

fn size_intent(h: f64) -> MeshIntent {
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Constant { size: h });
    intent
}

fn mesh_with(
    topo: &CellComplex,
    vol: EntityId,
    intent: &MeshIntent,
    reg: &SchemeRegistry,
    exec: &dyn Executor,
) -> Result<sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh>, SchemeError> {
    let model = sm_model_api::Model::new();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(reg, &solver, &picker);
    let sel = [vol];
    let input = MeshInput::new(topo, &model, &WholeFaces, &sel);
    driver.mesh(&input, intent, exec, ReferenceBuilder::new(), &mut SerialAlloc::new())
}

/// Boundary faces (one cell) of the mesh equal the surface patches, as vertex triples.
fn boundary_equals_patches(out: &sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh>, faces: &[EntityId]) {
    let m = &out.finished.mesh;
    let canon = |mut t: [u32; 3]| {
        t.sort_unstable();
        t
    };
    let mut want: BTreeSet<[u32; 3]> = BTreeSet::new();
    let mut n_want = 0;
    for f in faces {
        let patch = out.patches.get(Target::Surface(*f)).expect("surface patch");
        for (shape, local) in patch.elements.iter() {
            assert_eq!(shape, ElemShape::Tri);
            let v = |i: usize| out.vertex_of(patch.key(local[i]).unwrap()).unwrap().get() as u32;
            want.insert(canon([v(0), v(1), v(2)]));
            n_want += 1;
        }
    }
    let mut got: BTreeSet<[u32; 3]> = BTreeSet::new();
    let mut n_got = 0;
    for f in 0..m.count(MeshEntityKind::Face) {
        let f = FaceIx::new(f);
        if m.face_right(f).is_none() {
            let vs = m.face_vertices(f);
            assert_eq!(vs.len(), 3);
            got.insert(canon([vs[0].get() as u32, vs[1].get() as u32, vs[2].get() as u32]));
            n_got += 1;
        }
    }
    assert_eq!(n_want, n_got, "boundary face count");
    assert_eq!(want, got, "boundary faces equal the surface patch triangles");
}

fn state_signature(out: &sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh>) -> (u64, [usize; 3], usize) {
    let m = &out.finished.mesh;
    (
        sm_testkit::check::mesh_hash(m),
        [m.count(MeshEntityKind::Vertex), m.count(MeshEntityKind::Face), m.count(MeshEntityKind::Cell)],
        out.patches.node_count(),
    )
}

#[test]
fn brick10_size2_through_the_driver() {
    let (topo, ids, _) = brick_setup([10.0, 10.0, 10.0]);
    let reg = registry(None);
    let intent = size_intent(2.0);
    let out = mesh_with(&topo, ids.volume, &intent, &reg, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    sm_testkit::check::mesh_valid(m).unwrap();
    sm_testkit::check::classification_complete(m).unwrap();
    let n_cells = m.count(MeshEntityKind::Cell);
    assert!(n_cells > 100, "cells {n_cells}");
    let mut total = 0.0;
    for c in 0..n_cells {
        let c = CellIx::new(c);
        assert_eq!(m.cell_shape(c), CellShape::Tet);
        total += cell_volume(m, c);
    }
    assert!((total - 1000.0).abs() < 1e-9 * 1000.0, "volume {total}");
    boundary_equals_patches(&out, &ids.faces);
    // The tetmesh plan: scheme name, TRI imposed on the faces.
    let tp = out.plan.target(Target::Volume(ids.volume)).unwrap();
    assert_eq!(tp.scheme.as_str(), NAME);
    for f in &ids.faces {
        assert!(out.plan.target(Target::Surface(*f)).unwrap().caps.contains(Caps::TRI));
    }
    // Steiner nodes are owned by the piece with Param::None.
    let vp = out.patches.get(Target::Volume(ids.volume)).unwrap();
    assert!(!vp.nodes.is_empty(), "refinement adds interior nodes");
    for n in &vp.nodes {
        assert_eq!(n.key.owner, ids.volume);
        assert_eq!(n.param, sm_mesh_api::Param::None);
    }
    // Counts stable across runs; Sequential and ScopedThreads(4) give identical state.
    let again = mesh_with(&topo, ids.volume, &intent, &reg, &Sequential).unwrap();
    let threaded = mesh_with(&topo, ids.volume, &intent, &reg, &ScopedThreads(4)).unwrap();
    assert_eq!(state_signature(&out), state_signature(&again));
    assert_eq!(state_signature(&out), state_signature(&threaded));
    assert_eq!(format!("{:?}", out.finished.mesh.coords()), format!("{:?}", threaded.finished.mesh.coords()));
    let _ = (VertexIx::new(0), vertex_point as fn(&dyn TopologyView, EntityId) -> _);
}

#[test]
fn brick_with_void_has_both_shells() {
    let mut topo = CellComplex::new();
    let vb = with_op(&mut topo, OpId(1), |t| brick_with_void(t, Point3::new(0.0, 0.0, 0.0), [10.0, 10.0, 10.0], Point3::new(3.0, 3.0, 3.0), [4.0, 4.0, 4.0])).unwrap();
    let reg = registry(None);
    let out = mesh_with(&topo, vb.outer.volume, &size_intent(2.0), &reg, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    sm_testkit::check::classification_complete(m).unwrap();
    let total: f64 = (0..m.count(MeshEntityKind::Cell)).map(|c| cell_volume(m, CellIx::new(c))).sum();
    assert!((total - (1000.0 - 64.0)).abs() < 1e-9 * 1000.0, "volume {total}");
    let mut faces = vb.outer.faces.to_vec();
    faces.extend_from_slice(&vb.void.faces);
    boundary_equals_patches(&out, &faces);
}

#[test]
fn baffle_and_embedded_members_are_not_applicable() {
    let reg = registry(None);
    let model = sm_model_api::Model::new();
    let intent = MeshIntent::default();
    let params = Params::empty();
    // Baffle.
    let mut topo = CellComplex::new();
    let (b, _) = with_op(&mut topo, OpId(1), brick_with_baffle).unwrap();
    let sel = [b.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let cx = PlanCtx::new(&input, &intent);
    assert!(matches!(TetVolume.applicability(&cx, Target::Volume(b.volume), &params), Applicability::NotApplicable { .. }));
    // Embedded vertex in the volume.
    let (mut topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    with_op(&mut topo, OpId(2), |t| {
        let v = t.make_vertex(Point3::new(0.5, 0.5, 0.5))?;
        t.embed(ids.volume, v)?;
        Ok(v)
    })
    .unwrap();
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let cx = PlanCtx::new(&input, &intent);
    match TetVolume.applicability(&cx, Target::Volume(ids.volume), &params) {
        Applicability::NotApplicable { reason, .. } => assert!(reason.contains("embedded"), "{reason}"),
        other => panic!("{other:?}"),
    }
    // Through the driver, a hard request is an error that names the scheme.
    let mut intent = size_intent(0.5);
    intent.set_scheme(ids.volume, SchemeChoice::new(NAME, Firmness::Hard));
    let r = mesh_with(&topo, ids.volume, &intent, &reg, &Sequential);
    assert!(matches!(r, Err(SchemeError::NotApplicable { .. }) | Err(SchemeError::NoApplicableScheme { .. })), "{:?}", r.err());
}

#[test]
fn unknown_keys_and_bad_values_are_not_applicable() {
    let reg = registry(None);
    let model = sm_model_api::Model::new();
    let intent = MeshIntent::default();
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let cx = PlanCtx::new(&input, &intent);
    for bad in [
        serde_json::json!({"colour": 1}),
        serde_json::json!({"size": -1.0}),
        serde_json::json!({"optimize": 3}),
        serde_json::json!({"radius_edge": 0.2}),
        serde_json::json!({"gradation": 0.5}),
        serde_json::json!({"size": "big"}),
    ] {
        match TetVolume.applicability(&cx, Target::Volume(ids.volume), &Params(bad.clone())) {
            Applicability::NotApplicable { reason, .. } => assert!(reason.contains("tetmesh params"), "{bad}: {reason}"),
            other => panic!("{bad}: {other:?}"),
        }
    }
    let good = Params(serde_json::json!({"size": 0.5, "radius_edge": 2.0, "optimize": 1, "gradation": 1.2}));
    assert!(matches!(TetVolume.applicability(&cx, Target::Volume(ids.volume), &good), Applicability::Applicable { score: 10, .. }));
    let _ = reg;
}

#[test]
fn params_change_the_mesh() {
    let (topo, ids, _) = brick_setup([10.0, 10.0, 10.0]);
    let reg = registry(None);
    let mut coarse = size_intent(2.0);
    coarse.set_scheme(ids.volume, SchemeChoice::with_params(NAME, serde_json::json!({"optimize": 0}), Firmness::Hard));
    let mut fine = size_intent(2.0);
    fine.set_scheme(ids.volume, SchemeChoice::with_params(NAME, serde_json::json!({"size": 1.0}), Firmness::Hard));
    let a = mesh_with(&topo, ids.volume, &coarse, &reg, &Sequential).unwrap();
    let b = mesh_with(&topo, ids.volume, &fine, &reg, &Sequential).unwrap();
    eprintln!("cells: default {} / size 1.0 {}", a.finished.mesh.count(MeshEntityKind::Cell), b.finished.mesh.count(MeshEntityKind::Cell));
    assert!(b.finished.mesh.count(MeshEntityKind::Cell) > a.finished.mesh.count(MeshEntityKind::Cell));
}

#[test]
fn failure_names_the_face_and_carries_node_keys() {
    let (topo, ids, _) = brick_setup([10.0, 10.0, 10.0]);
    let broken = ids.faces[4];
    let reg = registry(Some(broken));
    let r = mesh_with(&topo, ids.volume, &size_intent(2.0), &reg, &Sequential);
    let Err(SchemeError::Generate(g)) = r else { panic!("expected a generation failure, got {:?}", r.err()) };
    assert_eq!(g.scheme.as_str(), NAME);
    assert_eq!(g.kind, FailureKind::Internal);
    assert!(g.reason.contains("TopoFace#"), "{}", g.reason);
    assert!(g.reason.contains("suggested action"), "{}", g.reason);
    assert!(g.nodes.len() >= 2, "nodes {:?}", g.nodes);
    // The nodes belong to the mesh (curve, vertex or face nodes).
    eprintln!("failure: {}", g.reason);
}

/// Acceptance 12 (record only): a million-tet brick through the driver, including the
/// serial commit into the reference builder (the bulk commit is W3b).
#[test]
#[ignore = "performance record"]
fn record_adapter_perf() {
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    let reg = registry(None);
    let mut intent = size_intent(1.0 / 56.0);
    intent.set_scheme(ids.volume, SchemeChoice::with_params(NAME, serde_json::json!({"size": 0.018}), Firmness::Hard));
    let t = std::time::Instant::now();
    let out = mesh_with(&topo, ids.volume, &intent, &reg, &Sequential).expect("mesh");
    let s = t.elapsed().as_secs_f64();
    let cells = out.finished.mesh.count(MeshEntityKind::Cell);
    eprintln!("adapter: {cells} tets through the driver in {s:.1} s = {:.0} tets/s (generation + serial commit)", cells as f64 / s);
}


// ---------------------------------------------------------------- PartitionCell pieces

/// A 2 x 1 x 1 brick cut at x = 0.8 by a realized plane cut: the two pieces are `PartitionCell`
/// targets of the overlay (ruling S4: their Steiner nodes are owned by the cell).
struct Split {
    overlay: sm_topo_api::Overlay,
    model: sm_model_api::Model,
    pieces: [EntityId; 2],
    cut_face: EntityId,
}

fn split() -> Split {
    use sm_core::Vector3;
    use sm_model_api::{Domain, PieceRef, Plane};
    use sm_topo_api::fixtures::brick;
    let mut t = CellComplex::new();
    let ids = with_op(&mut t, OpId(1), |t| brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 1.0, 1.0])).unwrap();
    let host: std::sync::Arc<dyn TopologyView> = std::sync::Arc::new(t);
    let mut overlay = sm_topo_api::Overlay::new(host);
    let mut model = sm_model_api::Model::new();
    let mat = model.add_color(1, "fuel", Domain::Volume).unwrap();
    model.color_volumes(mat, &[ids.volume]).unwrap();
    let pieces = model.split_realized(PieceRef::Whole(ids.volume), Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.8).unwrap(), &mut overlay).unwrap();
    let cut = model.decomposition().split_of(PieceRef::Whole(ids.volume)).unwrap().cut;
    let cut_face = model.decomposition().cut_cells(cut).unwrap().faces[0];
    Split { overlay, model, pieces, cut_face }
}

#[test]
fn partition_cell_pieces_mesh_conformingly_and_own_their_steiner_nodes() {
    use sm_scheme_api::DecompositionFaces;
    let s = split();
    let reg = registry(None);
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let input = MeshInput::new(&s.overlay, &s.model, &DecompositionFaces, &s.pieces);
    let intent = size_intent(0.25);
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("mesh both pieces");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    let mut vol = [0.0; 2];
    for c in 0..m.count(MeshEntityKind::Cell) {
        let ci = CellIx::new(c);
        assert_eq!(m.cell_shape(ci), CellShape::Tet);
        let on = m.cell_class(ci).entity;
        let k = s.pieces.iter().position(|p| *p == on).expect("cells are classified on the pieces");
        vol[k] += cell_volume(m, ci);
    }
    assert!((vol[0] - 0.8).abs() < 1e-9 && (vol[1] - 1.2).abs() < 1e-9, "{vol:?}");
    // The cut face is meshed once and shared by both pieces.
    let cut = m.classified_on(s.cut_face).expect("faces on the cut");
    assert!(!cut.faces.is_empty());
    assert!(cut.faces.iter().all(|f| m.face_kind(*f) == sm_mesh_api::FaceKind::Interior));
    // Steiner nodes are owned by the cell they were generated for, with no parametrisation.
    for p in s.pieces {
        let vp = out.patches.get(Target::Volume(p)).expect("piece patch");
        assert!(vp.nodes.iter().all(|n| n.key.owner == p && n.param == sm_mesh_api::Param::None));
    }
    assert!(s.pieces.iter().any(|p| !out.patches.get(Target::Volume(*p)).unwrap().nodes.is_empty()), "refinement adds interior nodes");
}

/// The tets of a volume patch as sorted node-key quadruples, and its Steiner nodes by key.
type PatchContent = (Vec<Vec<NodeKey>>, Vec<(NodeKey, [u64; 3])>);

fn patch_content(out: &sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh>, vol: EntityId) -> PatchContent {
    let p = out.patches.get(Target::Volume(vol)).expect("volume patch");
    let mut tets: Vec<Vec<NodeKey>> = p
        .elements
        .iter()
        .map(|(_, local)| {
            let mut k: Vec<NodeKey> = local.iter().map(|&i| p.key(i).unwrap()).collect();
            k.sort_unstable();
            k
        })
        .collect();
    tets.sort_unstable();
    let mut nodes: Vec<(NodeKey, [u64; 3])> = p.nodes.iter().map(|n| (n.key, [n.point.x.to_bits(), n.point.y.to_bits(), n.point.z.to_bits()])).collect();
    nodes.sort_unstable();
    (tets, nodes)
}

/// Acceptance 11 through the adapter: the same surface patches listed in a different node and
/// element order (rotated corners, reversed node creation) give the same tets and Steiner nodes.
#[test]
fn adapter_result_does_not_depend_on_patch_listing_order() {
    let (topo, ids, _) = brick_setup([10.0, 10.0, 10.0]);
    let intent = size_intent(2.0);
    let base = mesh_with(&topo, ids.volume, &intent, &registry(None), &Sequential).expect("mesh");
    let want = patch_content(&base, ids.volume);
    assert!(!want.0.is_empty() && !want.1.is_empty());
    for seed in [1u64, 2, 7] {
        let out = mesh_with(&topo, ids.volume, &intent, &registry_shuffled(None, seed), &Sequential).expect("mesh");
        assert_eq!(patch_content(&out, ids.volume), want, "shuffle {seed}");
    }
}
