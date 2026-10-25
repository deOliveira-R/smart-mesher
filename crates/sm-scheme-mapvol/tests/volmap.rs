#![allow(clippy::err_expect)] // `Meshed` is not `Debug`, so `expect_err` is unavailable
//! End to end: stub `equal` + stub surface `map` + this crate's volume `map`, through the
//! driver into the reference mesh builder. Analytic checks.

use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, Interval, Line, Plane, SurfaceDef};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{CellIx, MeshEntityKind, MeshRead, SerialAlloc, cell_volume, validate};
use sm_model_api::Model;
use sm_quality::{Metric, Options, compute};
use sm_scheme_api::stubs::{brick_setup, registry};
use sm_scheme_api::{
    Applicability, Caps, Driver, Executor, Firm, Firmness, IntervalRequest, MeshInput, MeshIntent, Meshed, PlanCtx, ReferenceCornerPicker,
    ReferenceIntervalSolver, ScopedThreads, Scheme, SchemeChoice, SchemeDim, SchemeError, SchemeRegistry, Selection, Sequential, SizingSpec,
    Target, WholeFaces,
};
use sm_topo_api::fixtures::{BrickIds, cylinder, with_op};
use sm_topo_api::{CellComplex, CoedgeSpec, CurveCarrier, FaceUseSpec, OpId, Sense, ShellKind, ShellSpec, Side, SurfaceCarrier, TopologyEdit};

type Out = Meshed<ReferenceMesh>;

fn reg() -> SchemeRegistry {
    let mut r = registry();
    sm_scheme_mapvol::register(&mut r).expect("register");
    r
}

fn mesh_with(topo: &CellComplex, model: &Model, intent: &MeshIntent, vol: EntityId, exec: &dyn Executor) -> Result<Out, SchemeError> {
    let r = reg();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&r, &solver, &picker);
    let sel = [vol];
    let input = MeshInput::new(topo, model, &WholeFaces, &sel);
    driver.mesh(&input, intent, exec, ReferenceBuilder::new(), &mut SerialAlloc::new())
}

fn map_intent(vol: EntityId) -> MeshIntent {
    let mut intent = MeshIntent::default();
    intent.set_scheme(vol, SchemeChoice::new("map", Firmness::Hard));
    intent
}

fn counts(intent: &mut MeshIntent, ids: &BrickIds, n: [u32; 3]) {
    for (axis, first) in [0usize, 4, 8].into_iter().enumerate() {
        intent.set_intervals(ids.edges[first], Firm::hard(IntervalRequest::Count(n[axis])));
    }
}

fn coords(m: &ReferenceMesh) -> Vec<[f64; 3]> {
    let c = m.coords();
    (0..c.x.len()).map(|i| [c.x[i], c.y[i], c.z[i]]).collect()
}

fn volume(m: &ReferenceMesh) -> f64 {
    (0..m.count(MeshEntityKind::Cell)).map(|c| cell_volume(m, CellIx(c as u32))).sum()
}

fn min_scaled_jacobian(m: &ReferenceMesh) -> f64 {
    min_scaled_jacobian_of(m)
}

fn min_scaled_jacobian_of(m: &dyn MeshRead) -> f64 {
    compute(m, Metric::ScaledJacobian, &Options::default()).expect("quality").into_iter().fold(f64::INFINITY, f64::min)
}

#[test]
fn brick_through_the_production_mesh_builder_matches_the_reference() {
    let (topo, ids, model) = brick_setup([10.0, 10.0, 10.0]);
    let mut intent = map_intent(ids.volume);
    intent.defaults.sizing = Some(SizingSpec::Constant { size: 1.0 });
    let r = reg();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&r, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let out = driver.mesh(&input, &intent, &Sequential, sm_mesh::MeshBuilder::new(), &mut SerialAlloc::new()).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 1000);
    assert_eq!(m.count(MeshEntityKind::Vertex), 1331);
    assert!((min_scaled_jacobian_of(m) - 1.0).abs() < 1e-12);
    let reference = mesh_with(&topo, &model, &intent, ids.volume, &Sequential).expect("reference mesh");
    assert_eq!(sm_testkit::check::mesh_hash(m), sm_testkit::check::mesh_hash(&reference.finished.mesh));
}

#[test]
fn single_hex() {
    let (topo, ids, model) = brick_setup([2.0, 3.0, 4.0]);
    let mut intent = map_intent(ids.volume);
    counts(&mut intent, &ids, [1, 1, 1]);
    let out = mesh_with(&topo, &model, &intent, ids.volume, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 1);
    assert_eq!(m.count(MeshEntityKind::Vertex), 8);
    assert!((volume(m) - 24.0).abs() < 1e-12);
    assert!((min_scaled_jacobian(m) - 1.0).abs() < 1e-12);
}

#[test]
fn brick_10_cubed_size_1_is_the_integer_lattice() {
    let (topo, ids, model) = brick_setup([10.0, 10.0, 10.0]);
    let mut intent = map_intent(ids.volume);
    intent.defaults.sizing = Some(SizingSpec::Constant { size: 1.0 });
    let out = mesh_with(&topo, &model, &intent, ids.volume, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 1000);
    assert_eq!(m.count(MeshEntityKind::Vertex), 1331);
    for p in coords(m) {
        for c in p {
            assert!((c - c.round()).abs() < 1e-12 && (0.0..=10.0).contains(&c.round()), "node {p:?}");
        }
    }
    assert!((volume(m) - 1000.0).abs() < 1e-9, "volume {}", volume(m));
    assert!((min_scaled_jacobian(m) - 1.0).abs() < 1e-12);
    // The map volume writes no face mesh itself: the faces are the stub map's.
    let vp = out.plan.target(Target::Volume(ids.volume)).expect("plan");
    assert_eq!(vp.scheme.as_str(), "map");
    assert!(vp.caps.contains(Caps::HEX | Caps::STRUCTURED | Caps::BLOCK_STRUCTURED));
    for f in ids.faces {
        let fp = out.plan.target(Target::Surface(f)).expect("face plan");
        assert_eq!(fp.scheme.as_str(), "map");
        assert!(matches!(fp.selection, Selection::Imposed { .. }), "{:?}", fp.selection);
    }
}

#[test]
fn brick_2x3x4_with_hard_counts_has_24_hexes() {
    let (topo, ids, model) = brick_setup([1.0, 1.5, 2.0]);
    let mut intent = map_intent(ids.volume);
    counts(&mut intent, &ids, [2, 3, 4]);
    let out = mesh_with(&topo, &model, &intent, ids.volume, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 24);
    assert_eq!(m.count(MeshEntityKind::Vertex), 3 * 4 * 5);
    assert!((volume(m) - 3.0).abs() < 1e-12);
    for c in 0..24 {
        assert!((cell_volume(m, CellIx(c)) - 0.125).abs() < 1e-12);
    }
    // Every node is on the 0.5 lattice of the brick [0,1] x [0,1.5] x [0,2] scaled 1/2, 1/2, 1/2.
    for p in coords(m) {
        assert!(((p[0] * 2.0).round() / 2.0 - p[0]).abs() < 1e-12);
    }
}

/// Skewed hex through the topology-edit `make_*` calls (planar faces, straight edges).
struct Hex {
    edges: [EntityId; 12],
    volume: EntityId,
}

const EDGES: [(usize, usize); 12] = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)];
const CYCLES: [[usize; 4]; 6] = [[0, 4, 6, 2], [1, 3, 7, 5], [0, 1, 5, 4], [2, 6, 7, 3], [0, 2, 3, 1], [4, 5, 7, 6]];

fn sub3(a: Point3, b: Point3) -> Vector3 {
    a - b
}

fn build_hex(p: [Point3; 8], vorder: [usize; 8], eorder: [usize; 12], forder: [usize; 6], rot: [usize; 6]) -> (CellComplex, Hex) {
    let mut topo = CellComplex::new();
    let hex = with_op(&mut topo, OpId(1), |t| {
        let mut corners = [None; 8];
        for v in vorder {
            corners[v] = Some(t.make_vertex(p[v])?);
        }
        let corners: [EntityId; 8] = corners.map(|c| c.expect("corner"));
        let mut edges = [None; 12];
        for e in eorder {
            let (a, b) = EDGES[e];
            let d = sub3(p[b], p[a]);
            let u = d * (1.0 / d.norm());
            let curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(p[a], u)?))?;
            edges[e] = Some(t.make_edge(corners[a], corners[b], Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }))?);
        }
        let edges: [EntityId; 12] = edges.map(|e| e.expect("edge"));
        let coedge = |a: usize, b: usize| -> CoedgeSpec {
            match EDGES.iter().position(|e| *e == (a, b)) {
                Some(n) => CoedgeSpec { edge: edges[n], sense: Sense::Forward },
                None => CoedgeSpec { edge: edges[EDGES.iter().position(|e| *e == (b, a)).expect("edge")], sense: Sense::Reversed },
            }
        };
        let mut faces = [None; 6];
        for f in forder {
            let mut cyc = CYCLES[f];
            cyc.rotate_left(rot[f] % 4);
            let l: Vec<CoedgeSpec> = (0..4).map(|i| coedge(cyc[i], cyc[(i + 1) % 4])).collect();
            let n = sub3(p[cyc[1]], p[cyc[0]]).cross(sub3(p[cyc[2]], p[cyc[1]]));
            let frame = Frame::from_z(p[cyc[0]], n * (1.0 / n.norm()))?;
            let surface = t.shapes_mut()?.add_surface(SurfaceDef::Plane(Plane { frame }))?;
            faces[f] = Some(t.make_face(&[&l], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?);
        }
        let uses: Vec<FaceUseSpec> = faces.iter().map(|f| FaceUseSpec { face: f.expect("face"), side: Side::Left }).collect();
        let volume = t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
        t.make_body(&[volume])?;
        Ok(Hex { edges, volume })
    })
    .expect("hex");
    (topo, hex)
}

fn parallelepiped(o: [f64; 3], a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> [Point3; 8] {
    std::array::from_fn(|i| {
        let (x, y, z) = ((i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64);
        Point3::new(o[0] + x * a[0] + y * b[0] + z * c[0], o[1] + x * a[1] + y * b[1] + z * c[1], o[2] + x * a[2] + y * b[2] + z * c[2])
    })
}

fn hex_counts(intent: &mut MeshIntent, h: &Hex, n: [u32; 3]) {
    for (axis, firsts) in [[0usize, 1, 2, 3], [4, 5, 6, 7], [8, 9, 10, 11]].into_iter().enumerate() {
        intent.set_intervals(h.edges[firsts[0]], Firm::hard(IntervalRequest::Count(n[axis])));
    }
}

const ID6: [usize; 6] = [0, 1, 2, 3, 4, 5];
const ID8: [usize; 8] = [0, 1, 2, 3, 4, 5, 6, 7];
const ID12: [usize; 12] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11];

#[test]
fn parallelepiped_is_affine_exact() {
    let (o, a, b, c) = ([1.0, -2.0, 0.5], [3.0, 0.5, 0.0], [0.4, 2.0, 0.3], [0.1, -0.2, 1.5]);
    let (topo, h) = build_hex(parallelepiped(o, a, b, c), ID8, ID12, ID6, [0; 6]);
    let mut intent = map_intent(h.volume);
    hex_counts(&mut intent, &h, [3, 4, 5]);
    let out = mesh_with(&topo, &Model::new(), &intent, h.volume, &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert_eq!(m.count(MeshEntityKind::Cell), 60);
    // Equal spacing along straight edges: every node is a lattice point of the affine map, and
    // every lattice point is a node (nearest match within 1e-12 per component).
    let mut want: Vec<[f64; 3]> = Vec::new();
    for k in 0..=5 {
        for j in 0..=4 {
            for i in 0..=3 {
                let (x, y, z) = (i as f64 / 3.0, j as f64 / 4.0, k as f64 / 5.0);
                want.push(std::array::from_fn(|d| o[d] + x * a[d] + y * b[d] + z * c[d]));
            }
        }
    }
    let got = coords(m);
    assert_eq!(got.len(), want.len());
    let near = |p: &[f64; 3], set: &[[f64; 3]]| set.iter().map(|q| (0..3).map(|d| (p[d] - q[d]).abs()).fold(0.0, f64::max)).fold(f64::INFINITY, f64::min);
    for p in &got {
        assert!(near(p, &want) < 1e-12, "node {p:?} is off the affine lattice by {}", near(p, &want));
    }
    for q in &want {
        assert!(near(q, &got) < 1e-12, "lattice point {q:?} has no node");
    }
    let det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]);
    assert!((volume(m) - det).abs() < 1e-12 * det.abs().max(1.0), "{} vs {det}", volume(m));
}

fn geometric_cells(m: &ReferenceMesh) -> Vec<Vec<[i64; 3]>> {
    let c = coords(m);
    let mut out: Vec<Vec<[i64; 3]>> = (0..m.count(MeshEntityKind::Cell))
        .map(|ci| {
            let mut v: Vec<[i64; 3]> = m
                .cell_corners(CellIx(ci as u32))
                .expect("template cell")
                .iter()
                .map(|vi| {
                    let p = c[vi.0 as usize];
                    [(p[0] * 1e9).round() as i64, (p[1] * 1e9).round() as i64, (p[2] * 1e9).round() as i64]
                })
                .collect();
            v.sort_unstable();
            v
        })
        .collect();
    out.sort_unstable();
    out
}

/// (vertex order, edge order, face order, loop start per face).
type Variant = ([usize; 8], [usize; 12], [usize; 6], [usize; 6]);

#[test]
fn any_face_order_loop_start_and_vertex_order_gives_the_same_mesh() {
    let pts = parallelepiped([0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.5, 1.5, 0.0], [0.0, 0.25, 1.0]);
    let variants: [Variant; 4] = [
        (ID8, ID12, ID6, [0; 6]),
        ([7, 6, 5, 4, 3, 2, 1, 0], [11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0], [5, 4, 3, 2, 1, 0], [1, 2, 3, 0, 1, 2]),
        ([3, 5, 0, 7, 1, 6, 2, 4], [4, 9, 0, 11, 2, 7, 5, 1, 10, 3, 8, 6], [2, 0, 4, 1, 5, 3], [3, 3, 1, 1, 2, 2]),
        ([6, 0, 4, 2, 5, 7, 1, 3], [8, 1, 6, 3, 10, 5, 0, 7, 2, 9, 4, 11], [3, 5, 1, 4, 0, 2], [2, 0, 3, 1, 3, 1]),
    ];
    let mut base: Option<Vec<Vec<[i64; 3]>>> = None;
    for (v, e, f, r) in variants {
        let (topo, h) = build_hex(pts, v, e, f, r);
        let mut intent = map_intent(h.volume);
        hex_counts(&mut intent, &h, [2, 3, 2]);
        let out = mesh_with(&topo, &Model::new(), &intent, h.volume, &Sequential).expect("mesh");
        let m = &out.finished.mesh;
        assert!(validate(m).is_empty(), "{:?}", validate(m));
        assert_eq!(m.count(MeshEntityKind::Cell), 12);
        let cells = geometric_cells(m);
        match &base {
            None => base = Some(cells),
            Some(b) => assert_eq!(&cells, b),
        }
    }
}

#[test]
fn a_piece_without_six_faces_is_not_applicable() {
    let mut topo = CellComplex::new();
    let ids = with_op(&mut topo, OpId(1), |t| cylinder(t, 1.0, 2.0)).expect("cylinder");
    let model = Model::new();
    let intent = MeshIntent::default();
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let cx = PlanCtx::new(&input, &intent);
    let r = reg();
    let s = r.get(SchemeDim::Volume, "map").expect("registered");
    match s.applicability(&cx, Target::Volume(ids.volume), &Default::default()) {
        Applicability::NotApplicable { reason, .. } => assert!(reason.contains("faces"), "{reason}"),
        other => panic!("{other:?}"),
    }
    // A hard request for it through the driver reports the same.
    let err = mesh_with(&topo, &model, &map_intent(ids.volume), ids.volume, &Sequential).err().expect("refused");
    assert!(matches!(err, SchemeError::NotApplicable { .. }), "{err:?}");
}

#[test]
fn sequential_and_scoped_threads_are_bit_identical() {
    let (topo, ids, model) = brick_setup([3.0, 2.0, 5.0]);
    let mut intent = map_intent(ids.volume);
    counts(&mut intent, &ids, [6, 5, 7]);
    let a = mesh_with(&topo, &model, &intent, ids.volume, &Sequential).expect("seq");
    let b = mesh_with(&topo, &model, &intent, ids.volume, &ScopedThreads(4)).expect("par");
    assert_eq!(a.finished.mesh, b.finished.mesh);
    assert_eq!(a.patches.nodes().map(|(k, p, _)| (k, p.x.to_bits(), p.y.to_bits(), p.z.to_bits())).collect::<Vec<_>>(), b.patches.nodes().map(|(k, p, _)| (k, p.x.to_bits(), p.y.to_bits(), p.z.to_bits())).collect::<Vec<_>>());
}

#[test]
fn determinism_over_threads_and_repeats() {
    let bits = sm_testkit::determinism::check(2, 4, |threads| {
        let (topo, ids, model) = brick_setup([4.0, 3.0, 2.0]);
        let mut intent = map_intent(ids.volume);
        counts(&mut intent, &ids, [8, 6, 4]);
        let exec: Box<dyn Executor> = if threads > 1 { Box::new(ScopedThreads(threads)) } else { Box::new(Sequential) };
        let out = mesh_with(&topo, &model, &intent, ids.volume, exec.as_ref()).expect("mesh");
        let c = coords(&out.finished.mesh);
        let flat: Vec<f64> = c.iter().flatten().copied().collect();
        (sm_testkit::determinism::hash_f64_bits(&flat), sm_testkit::check::mesh_hash(&out.finished.mesh))
    })
    .expect("deterministic");
    assert_ne!(bits.0, 0);
}

#[test]
fn big_block_uses_the_parallel_paths_deterministically() {
    // 30^3 interior > the parallel threshold: same bits as a small-thread run.
    let (topo, ids, model) = brick_setup([3.0, 3.0, 3.0]);
    let mut intent = map_intent(ids.volume);
    counts(&mut intent, &ids, [30, 30, 30]);
    let a = mesh_with(&topo, &model, &intent, ids.volume, &Sequential).expect("mesh");
    let b = mesh_with(&topo, &model, &intent, ids.volume, &ScopedThreads(3)).expect("mesh");
    assert_eq!(sm_testkit::check::mesh_hash(&a.finished.mesh), sm_testkit::check::mesh_hash(&b.finished.mesh));
    // The interior fill and the orientation check run on rayon above the threshold; a one-thread
    // pool and the default pool give the same bits.
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().expect("pool");
    let c = one.install(|| mesh_with(&topo, &model, &intent, ids.volume, &Sequential).expect("mesh"));
    assert_eq!(sm_testkit::check::mesh_hash(&a.finished.mesh), sm_testkit::check::mesh_hash(&c.finished.mesh));
    assert_eq!(a.finished.mesh, c.finished.mesh);
    assert_eq!(a.finished.mesh.count(MeshEntityKind::Cell), 27000);
    assert!((volume(&a.finished.mesh) - 27.0).abs() < 1e-9);
}

#[test]
fn conflicting_hard_counts_on_parallel_edges_are_infeasible() {
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let mut intent = map_intent(ids.volume);
    // Edges 0 and 3 are both x-edges, on different faces of the block.
    intent.set_intervals(ids.edges[0], Firm::hard(IntervalRequest::Count(2)));
    intent.set_intervals(ids.edges[3], Firm::hard(IntervalRequest::Count(3)));
    let err = mesh_with(&topo, &model, &intent, ids.volume, &Sequential).err().expect("infeasible");
    // The driver rejects `map` for a face over the infeasible row and finds nothing else (the
    // volume imposes quad + structured faces); the failure names both hard counts.
    assert!(format!("{err:?}").contains("hard interval counts 2 and 3"), "{err:?}");
}

#[test]
fn the_volume_contributes_one_equality_per_parallel_edge_of_each_direction() {
    use sm_scheme_api::{IntervalConstraint, Origin, Rel};
    let (topo, ids, model) = brick_setup([1.0, 1.0, 1.0]);
    let mut intent = map_intent(ids.volume);
    counts(&mut intent, &ids, [2, 3, 4]);
    let out = mesh_with(&topo, &model, &intent, ids.volume, &Sequential).expect("mesh");
    let vol = Target::Volume(ids.volume);
    let rows: Vec<&IntervalConstraint> = out
        .plan
        .problem
        .intervals
        .iter()
        .filter(|r| matches!(&r.origin, Origin::Scheme { target, scheme } if *target == vol && scheme.as_str() == "map"))
        .map(|r| &r.item)
        .collect();
    assert_eq!(rows.len(), 9, "3 classes x 3 equalities: {rows:?}");
    let mut classes: Vec<Vec<EntityId>> = Vec::new();
    for r in rows {
        let IntervalConstraint::Linear { terms, rel: Rel::Eq, rhs: 0 } = r else { panic!("{r:?}") };
        assert_eq!(terms.len(), 2);
        assert_eq!(terms[0].1 + terms[1].1, 0);
        let (a, b) = (terms[0].0, terms[1].0);
        match classes.iter_mut().find(|c| c.contains(&a) || c.contains(&b)) {
            Some(c) => {
                for e in [a, b] {
                    if !c.contains(&e) {
                        c.push(e);
                    }
                }
            }
            None => classes.push(vec![a, b]),
        }
    }
    classes.iter_mut().for_each(|c| c.sort_unstable());
    classes.sort();
    let mut want: Vec<Vec<EntityId>> = [0..4, 4..8, 8..12].into_iter().map(|r| r.map(|e| ids.edges[e]).collect()).collect();
    want.iter_mut().for_each(|c| c.sort_unstable());
    want.sort();
    assert_eq!(classes, want);
}

#[test]
fn auto_selection_prefers_the_volume_map_on_a_brick() {
    let (topo, ids, model) = brick_setup([2.0, 2.0, 2.0]);
    let mut intent = MeshIntent::default();
    counts(&mut intent, &ids, [2, 2, 2]);
    let out = mesh_with(&topo, &model, &intent, ids.volume, &Sequential).expect("mesh");
    let vp = out.plan.target(Target::Volume(ids.volume)).expect("plan");
    assert_eq!(vp.scheme.as_str(), "map");
    assert!(matches!(vp.selection, Selection::Auto { score: 100 }), "{:?}", vp.selection);
}

#[test]
fn the_registry_name_is_map_at_volume_only() {
    let r = reg();
    assert!(r.get(SchemeDim::Volume, "map").is_some());
    assert!(r.get(SchemeDim::Surface, "map").is_some());
    let mut twice = reg();
    assert!(sm_scheme_mapvol::register(&mut twice).is_err(), "double registration is rejected");
    let _: &dyn Scheme = &sm_scheme_mapvol::VolumeMap;
}

/// A hex whose top face is rotated by `deg` degrees about the vertical axis of the unit cube.
fn twisted_hex(deg: f64) -> [Point3; 8] {
    let (s, c) = deg.to_radians().sin_cos();
    std::array::from_fn(|i| {
        let (x, y, z) = ((i & 1) as f64 - 0.5, ((i >> 1) & 1) as f64 - 0.5, ((i >> 2) & 1) as f64);
        if z == 0.0 { Point3::new(x, y, 0.0) } else { Point3::new(c * x - s * y, s * x + c * y, 1.0) }
    })
}

#[test]
fn a_folded_block_fails_with_quality_and_the_driver_falls_back() {
    let (topo, h) = build_hex(twisted_hex(170.0), ID8, ID12, ID6, [0; 6]);
    let mut intent = map_intent(h.volume);
    hex_counts(&mut intent, &h, [4, 4, 4]);
    // Without a fallback the driver rejects map after its Quality failure in generate and has
    // nothing left to try.
    let err = mesh_with(&topo, &Model::new(), &intent, h.volume, &Sequential).err().expect("refused");
    match &err {
        SchemeError::NoApplicableScheme { tried, .. } => {
            assert_eq!(tried.len(), 1, "{tried:?}");
            assert_eq!(tried[0].scheme.as_str(), "map");
            assert!(tried[0].message.contains("(Quality)"), "{}", tried[0].message);
        }
        other => panic!("{other:?}"),
    }
    // With a fallback in the chain the Quality failure moves the driver on to it (the stub
    // `tetvol` then fails itself, so the run still ends in NoApplicableScheme, but with both
    // attempts recorded in order).
    let mut with_fb = MeshIntent::default();
    with_fb.set_scheme(h.volume, SchemeChoice::new("map", Firmness::Hard));
    with_fb.entity_mut(h.volume).fallbacks = vec![SchemeChoice::new("tetvol", Firmness::Hard)];
    hex_counts(&mut with_fb, &h, [4, 4, 4]);
    let err = mesh_with(&topo, &Model::new(), &with_fb, h.volume, &Sequential).err().expect("both fail");
    match &err {
        SchemeError::NoApplicableScheme { tried, .. } => {
            let names: Vec<&str> = tried.iter().map(|a| a.scheme.as_str()).collect();
            assert_eq!(names, ["map", "tetvol"], "{tried:?}");
            assert!(tried[0].message.contains("(Quality)"), "{}", tried[0].message);
        }
        other => panic!("{other:?}"),
    }
}

/// A triangular bipyramid: six faces, one loop each, every edge shared twice, V - E + F = 2, so
/// it passes the topological applicability test, yet it is not hex-like.
fn bipyramid() -> (CellComplex, EntityId) {
    let mut topo = CellComplex::new();
    let vol = with_op(&mut topo, OpId(1), |t| {
        let mut p = vec![Point3::new(0.0, 0.0, 1.0), Point3::new(0.0, 0.0, -1.0)];
        for i in 0..3 {
            let a = i as f64 * std::f64::consts::TAU / 3.0;
            p.push(Point3::new(a.cos(), a.sin(), 0.0));
        }
        let v: Vec<EntityId> = p.iter().map(|q| t.make_vertex(*q)).collect::<Result<_, _>>()?;
        // Edges: apex-equator 0..3 (top), 3..6 (bottom), equator ring 6..9.
        let mut pairs = Vec::new();
        for (apex, _) in [(0usize, ()), (1, ())] {
            for i in 0..3 {
                pairs.push((apex, 2 + i));
            }
        }
        for i in 0..3 {
            pairs.push((2 + i, 2 + (i + 1) % 3));
        }
        let mut edges = Vec::new();
        for (a, b) in &pairs {
            let d = p[*b] - p[*a];
            let u = d * (1.0 / d.norm());
            let curve = t.shapes_mut()?.add_curve(CurveDef::Line(Line::new(p[*a], u)?))?;
            edges.push(t.make_edge(v[*a], v[*b], Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward }))?);
        }
        let coedge = |a: usize, b: usize| -> CoedgeSpec {
            match pairs.iter().position(|e| *e == (a, b)) {
                Some(n) => CoedgeSpec { edge: edges[n], sense: Sense::Forward },
                None => CoedgeSpec { edge: edges[pairs.iter().position(|e| *e == (b, a)).expect("edge")], sense: Sense::Reversed },
            }
        };
        let mut cycles: Vec<[usize; 3]> = Vec::new();
        for i in 0..3 {
            cycles.push([0, 2 + i, 2 + (i + 1) % 3]);
            cycles.push([1, 2 + (i + 1) % 3, 2 + i]);
        }
        let mut faces = Vec::new();
        for c in &cycles {
            let l: Vec<CoedgeSpec> = (0..3).map(|i| coedge(c[i], c[(i + 1) % 3])).collect();
            let n = (p[c[1]] - p[c[0]]).cross(p[c[2]] - p[c[1]]);
            let frame = Frame::from_z(p[c[0]], n * (1.0 / n.norm()))?;
            let surface = t.shapes_mut()?.add_surface(SurfaceDef::Plane(Plane { frame }))?;
            faces.push(t.make_face(&[&l], Some(SurfaceCarrier { surface, sense: Sense::Forward }))?);
        }
        let uses: Vec<FaceUseSpec> = faces.iter().map(|f| FaceUseSpec { face: *f, side: Side::Left }).collect();
        let volume = t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }])?;
        t.make_body(&[volume])?;
        Ok(volume)
    })
    .expect("bipyramid");
    (topo, vol)
}

#[test]
fn a_six_faced_piece_that_is_not_hex_like_is_refused_by_the_driver() {
    let (topo, vol) = bipyramid();
    let model = Model::new();
    let sel = [vol];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let intent = MeshIntent::default();
    let cx = PlanCtx::new(&input, &intent);
    let r = reg();
    let s = r.get(SchemeDim::Volume, "map").expect("registered");
    // The topological test alone accepts it...
    assert!(matches!(s.applicability(&cx, Target::Volume(vol), &Default::default()), Applicability::Applicable { .. }));
    // ...and the driver refuses the hard request before generate (triangles cannot be quads).
    let err = mesh_with(&topo, &model, &map_intent(vol), vol, &Sequential).err().expect("refused");
    match &err {
        SchemeError::NoApplicableScheme { target, tried, .. } => {
            assert!(matches!(target, Target::Surface(_)), "{target:?}");
            assert!(tried.iter().any(|a| a.message.contains("imposition from Volume")), "{tried:?}");
        }
        other => panic!("{other:?}"),
    }
}
