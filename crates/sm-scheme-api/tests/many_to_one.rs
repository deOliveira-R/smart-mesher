//! A real many-to-one sweep (W0e decision 6): two source faces swept into one target face.
//!
//! The volume is the brick `[0,2] x [0,1] x [0,1]` whose bottom is two faces (split along
//! `x = 1`, the sources) and whose top is one six-sided face (the target, with two `Side`
//! vertices at `x = 1`). The `m2one` scheme below claims the target and writes it as the
//! projection of both sources: columns of boundary nodes come from the vertical edges and the
//! structured linking faces, columns of nodes inside the sources (including the interior edge
//! where the two sources meet) are generated through the volume and end on target-face nodes.

mod stub_schemes;

use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, Interval, Line, Plane, SurfaceDef};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{CellShape, MeshRead, Param, SerialAlloc, cell_volume, topology_hash, validate};
use sm_model_api::Model;
use sm_scheme_api::geom;
use sm_scheme_api::{
    Applicability, Caps, Driver, ElemShape, FailureKind, Firm, Firmness, GenCtx, GenerateFailure,
    IntervalCtx, IntervalRequest, IntervalSink, MeshInput, MeshIntent, NodeKey, Params, Patch,
    PatchBuilder, PlanCtx, ReferenceCornerPicker, ReferenceIntervalSolver, Requirements, Role,
    Scheme, SchemeChoice, SchemeDim, Selection, Sequential, Target, WholeFaces,
};
use sm_topo_api::fixtures::with_op;
use sm_topo_api::{
    CellComplex, CoedgeSpec, CurveCarrier, FaceUseSpec, OpId, Sense, ShellKind, ShellSpec, Side,
    SurfaceCarrier, TopologyEdit, TopologyView, TopologyViewExt,
};
use std::collections::{BTreeMap, BTreeSet};

type GResult<T> = Result<T, GenerateFailure>;

// ------------------------------------------------------------------ fixture

struct Fixture {
    volume: EntityId,
    sources: [EntityId; 2],
    target: EntityId,
    /// One edge per direction: x (left half), y, z.
    edges: [EntityId; 3],
}

/// Build a polyhedron from points and outward vertex cycles (plane faces, line edges).
fn polyhedron(t: &mut CellComplex, pts: &[Point3], cycles: &[&[usize]]) -> (EntityId, Vec<EntityId>, BTreeMap<(usize, usize), EntityId>) {
    let vs: Vec<EntityId> = pts.iter().map(|p| t.make_vertex(*p).unwrap()).collect();
    let mut edges: BTreeMap<(usize, usize), EntityId> = BTreeMap::new();
    let mut faces = Vec::new();
    for cyc in cycles {
        let n = cyc.len();
        let mut loop_: Vec<CoedgeSpec> = Vec::new();
        for i in 0..n {
            let (a, b) = (cyc[i], cyc[(i + 1) % n]);
            let key = (a.min(b), a.max(b));
            let e = match edges.get(&key) {
                Some(e) => *e,
                None => {
                    let (p, q) = (pts[key.0], pts[key.1]);
                    let d = q - p;
                    let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(p, d).unwrap())).unwrap();
                    let e = t.make_edge(vs[key.0], vs[key.1], Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward })).unwrap();
                    edges.insert(key, e);
                    e
                }
            };
            loop_.push(CoedgeSpec { edge: e, sense: if a < b { Sense::Forward } else { Sense::Reversed } });
        }
        // Newell normal of the outward cycle.
        let mut nrm = Vector3::default();
        for i in 0..n {
            let (a, b) = (pts[cyc[i]], pts[cyc[(i + 1) % n]]);
            nrm = nrm + Vector3::new((a.y - b.y) * (a.z + b.z), (a.z - b.z) * (a.x + b.x), (a.x - b.x) * (a.y + b.y));
        }
        let frame = Frame::from_z(pts[cyc[0]], nrm).unwrap();
        let surface = t.shapes_mut().unwrap().add_surface(SurfaceDef::Plane(Plane { frame })).unwrap();
        faces.push(t.make_face(&[&loop_], Some(SurfaceCarrier { surface, sense: Sense::Forward })).unwrap());
    }
    let uses: Vec<FaceUseSpec> = faces.iter().map(|f| FaceUseSpec { face: *f, side: Side::Left }).collect();
    let v = t.make_volume(&[ShellSpec { kind: ShellKind::Outer, uses: &uses }]).unwrap();
    t.make_body(&[v]).unwrap();
    (v, faces, edges)
}

fn fixture() -> (CellComplex, Fixture) {
    let mut t = CellComplex::new();
    let fx = with_op(&mut t, OpId(1), |t| {
        // Bottom 0..6 (z = 0), top 6..12 (z = 1): (0,0) (1,0) (2,0) (0,1) (1,1) (2,1).
        let xy = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (0.0, 1.0), (1.0, 1.0), (2.0, 1.0)];
        let pts: Vec<Point3> = [0.0, 1.0].iter().flat_map(|z| xy.iter().map(move |(x, y)| Point3::new(*x, *y, *z))).collect();
        let cycles: [&[usize]; 9] = [
            &[0, 3, 4, 1],         // source: bottom left (-z)
            &[1, 4, 5, 2],         // source: bottom right (-z)
            &[6, 7, 8, 11, 10, 9], // target: top (+z), six vertices
            &[0, 6, 9, 3],         // -x
            &[2, 5, 11, 8],        // +x
            &[0, 1, 7, 6],         // -y left
            &[1, 2, 8, 7],         // -y right
            &[3, 9, 10, 4],        // +y left
            &[4, 10, 11, 5],       // +y right
        ];
        let (volume, faces, edges) = polyhedron(t, &pts, &cycles);
        Ok(Fixture { volume, sources: [faces[0], faces[1]], target: faces[2], edges: [edges[&(0, 1)], edges[&(0, 3)], edges[&(0, 6)]] })
    })
    .unwrap();
    (t, fx)
}

// ------------------------------------------------------------------ the m2one scheme

/// Many-to-one translational sweep of one piece: `sources` (several faces) into `target` (one
/// face). Claims the target and writes it; linking faces must be structured.
struct M2One;

#[derive(Clone, Debug)]
struct Setup {
    sources: Vec<EntityId>,
    target: EntityId,
    linking: Vec<EntityId>,
    /// Source vertex -> its vertical edge (oriented source -> target by the flag).
    vertical: BTreeMap<EntityId, (EntityId, bool)>,
}

fn setup(input: &MeshInput<'_>, target: Target, p: &Params) -> Result<Setup, String> {
    let t = input.topo;
    let sources: Vec<EntityId> = p.get::<Vec<EntityId>>("m2one", "sources").map_err(|e| e.to_string())?.ok_or("no sources")?;
    let tgt: EntityId = p.get::<EntityId>("m2one", "target").map_err(|e| e.to_string())?.ok_or("no target")?;
    if sources.len() < 2 {
        return Err("a many-to-one sweep needs several sources".into());
    }
    let faces = input.piece_faces(target).map_err(|e| e.to_string())?;
    if sources.iter().chain(std::iter::once(&tgt)).any(|f| !faces.contains(f)) {
        return Err("sources and target must bound the piece".into());
    }
    let linking: Vec<EntityId> = faces.iter().copied().filter(|f| *f != tgt && !sources.contains(f)).collect();
    let end_edges: BTreeSet<EntityId> = sources.iter().chain(std::iter::once(&tgt)).flat_map(|f| t.face_edges(*f).iter().copied()).collect();
    let tverts: BTreeSet<EntityId> = t.face_edges(tgt).iter().filter_map(|e| t.edge_ends(*e)).flat_map(|(a, b)| [a, b]).collect();
    let sverts: BTreeSet<EntityId> = sources.iter().flat_map(|f| t.face_edges(*f).iter().filter_map(|e| t.edge_ends(*e)).flat_map(|(a, b)| [a, b])).collect();
    let mut vertical = BTreeMap::new();
    for v in &sverts {
        let cand: Vec<(EntityId, bool)> = linking
            .iter()
            .flat_map(|l| t.face_edges(*l).iter().copied())
            .filter(|e| !end_edges.contains(e))
            .filter_map(|e| {
                let (a, b) = t.edge_ends(e)?;
                if a == *v && tverts.contains(&b) {
                    Some((e, true))
                } else if b == *v && tverts.contains(&a) {
                    Some((e, false))
                } else {
                    None
                }
            })
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect();
        match cand.as_slice() {
            [one] => {
                vertical.insert(*v, *one);
            }
            _ => return Err(format!("source vertex {v} needs exactly one linking edge to the target")),
        }
    }
    Ok(Setup { sources, target: tgt, linking, vertical })
}

fn mean(t: &dyn TopologyView, vs: impl Iterator<Item = EntityId>) -> Vector3 {
    let (mut acc, mut n) = (Vector3::default(), 0.0);
    for v in vs {
        if let Ok(p) = geom::vertex_point(t, v) {
            acc = acc + (p - Point3::default());
            n += 1.0;
        }
    }
    acc * (1.0 / f64::max(n, 1.0))
}

impl Scheme for M2One {
    fn name(&self) -> &str {
        "m2one"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::HEX | Caps::WEDGE
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, p: &Params) -> Applicability {
        match setup(cx.input, target, p) {
            Ok(_) => Applicability::yes(70, "many-to-one sweepable"),
            Err(e) => Applicability::no(e),
        }
    }
    fn requirements(&self, cx: &PlanCtx<'_>, target: Target, p: &Params, out: &mut Requirements<'_>) -> sm_scheme_api::Result<()> {
        let s = setup(cx.input, target, p).map_err(sm_scheme_api::SchemeError::Input)?;
        for f in &s.sources {
            out.role(Target::Surface(*f), Role::SweepSource { volume: target });
        }
        out.role(Target::Surface(s.target), Role::SweepTarget { volume: target });
        // The target is the projection of all sources: this scheme writes it.
        out.claim(Target::Surface(s.target));
        for l in &s.linking {
            out.role(Target::Surface(*l), Role::SweepLinking { volume: target });
            out.require_caps(Target::Surface(*l), Caps::QUAD | Caps::STRUCTURED, Firmness::Hard);
        }
        Ok(())
    }
    fn interval_constraints(&self, cx: &IntervalCtx<'_>, target: Target, p: &Params, out: &mut IntervalSink<'_>) -> sm_scheme_api::Result<()> {
        let s = setup(cx.input, target, p).map_err(sm_scheme_api::SchemeError::Input)?;
        let v: Vec<EntityId> = s.vertical.values().map(|x| x.0).collect();
        for w in v.windows(2) {
            out.equal(&[w[0]], &[w[1]]);
        }
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, p: &Params) -> GResult<Patch> {
        let fatal = |r: String| GenerateFailure::fatal(target, "m2one", r);
        let t = cx.input.topo;
        let vol = target.entity().ok_or_else(|| fatal("not a piece".into()))?;
        let s = setup(cx.input, target, p).map_err(fatal)?;
        let (&first_v, &(e0, _)) = s.vertical.iter().next().ok_or_else(|| fatal("no columns".into()))?;
        let _ = first_v;
        let layers = cx.intervals(e0).ok_or_else(|| fatal("no layer count".into()))? as usize;
        let tverts = t.face_edges(s.target).iter().filter_map(|e| t.edge_ends(*e)).flat_map(|(a, b)| [a, b]);
        let shift = mean(t, tverts.collect::<BTreeSet<_>>().into_iter()) - mean(t, s.vertical.keys().copied());
        // How many sources hold each source edge: 1 = on the union boundary, 2 = interior.
        let mut held: BTreeMap<EntityId, u32> = BTreeMap::new();
        for f in &s.sources {
            for e in t.face_edges(*f) {
                *held.entry(*e).or_default() += 1;
            }
        }
        let mut pb = PatchBuilder::new(target);
        let mut own: BTreeMap<NodeKey, u32> = BTreeMap::new();
        let mut pos: BTreeMap<NodeKey, Point3> = BTreeMap::new();
        let mut cols: BTreeMap<NodeKey, Vec<NodeKey>> = BTreeMap::new();
        let (mut vol_ix, mut tgt_ix) = (0u32, 0u32);
        let mut keys: Vec<NodeKey> = Vec::new();
        for f in &s.sources {
            let sp = cx.patches.get(Target::Surface(*f)).ok_or_else(|| fatal(format!("source {f} not meshed")))?;
            keys.extend(sp.nodes.iter().map(|n| n.key).chain(sp.refs.iter().copied()));
        }
        keys.sort_unstable();
        keys.dedup();
        for k in &keys {
            let col: Vec<NodeKey> = if let Some(&(e, up)) = s.vertical.get(&k.owner).filter(|_| k.index == 0) {
                let mut c = cx.edge_nodes(e).ok_or_else(|| fatal("vertical edge".into()))?;
                if !up {
                    c.reverse();
                }
                c
            } else if held.get(&k.owner) == Some(&1) {
                // A boundary edge of the sources: across its structured linking face.
                let l = s.linking.iter().copied().find(|l| t.face_edges(*l).contains(&k.owner)).ok_or_else(|| fatal(format!("edge {} has no linking face", k.owner)))?;
                let g = cx.grid(l).ok_or_else(|| fatal(format!("linking face {l} has no grid")))?;
                g.line_from(*k).ok_or_else(|| fatal(format!("{k:?} is not on the grid boundary")))?
            } else {
                // Inside the sources (face interior or the edge where two sources meet): a
                // generated column through the volume, ending on a target-face node.
                let a = cx.point(*k).ok_or_else(|| fatal(format!("node {k:?}")))?;
                let mut c = vec![*k];
                for q in 1..layers {
                    let key = NodeKey::new(vol, vol_ix);
                    vol_ix += 1;
                    let pt = a + shift * (q as f64 / layers as f64);
                    own.insert(key, pb.node(key, pt, Param::None));
                    pos.insert(key, pt);
                    c.push(key);
                }
                let top = NodeKey::new(s.target, tgt_ix);
                tgt_ix += 1;
                let pt = a + shift;
                own.insert(top, pb.node(top, pt, Param::None));
                pos.insert(top, pt);
                c.push(top);
                c
            };
            if col.len() != layers + 1 {
                return Err(fatal(format!("column at {k:?} has {} nodes, expected {}", col.len(), layers + 1)));
            }
            cols.insert(*k, col);
        }
        let point = |k: NodeKey| pos.get(&k).copied().or_else(|| cx.point(k)).ok_or_else(|| fatal(format!("node {k:?}")));
        let p0 = t.face_outline(s.sources[0]).first().copied().ok_or_else(|| fatal("outline".into()))?;
        let ns = geom::face_normal(t, s.sources[0], p0).map_err(|e| fatal(e.to_string()))?;
        let nt = geom::face_normal(t, s.target, p0 + shift).map_err(|e| fatal(e.to_string()))?;
        let flip_target = ns.dot(nt) < 0.0;
        let mut buf: Vec<u32> = Vec::with_capacity(8);
        for f in &s.sources {
            let sp = cx.patches.get(Target::Surface(*f)).ok_or_else(|| fatal("source".into()))?;
            for (shape, nodes) in sp.elements.iter() {
                let sk: Vec<NodeKey> = nodes.iter().map(|l| sp.key(*l)).collect::<Option<_>>().ok_or_else(|| fatal("element".into()))?;
                let cell = match shape {
                    ElemShape::Quad => CellShape::Hex,
                    ElemShape::Tri => CellShape::Wedge,
                    other => return Err(GenerateFailure::new(target, "m2one", FailureKind::Unsupported, format!("cannot sweep {other:?}"))),
                };
                let c: Vec<&Vec<NodeKey>> = sk.iter().map(|k| &cols[k]).collect();
                let (a, b, z) = (point(c[0][0])?, point(c[1][0])?, point(c[sk.len() - 1][0])?);
                let up = point(c[0][1])? - a;
                let forward = (b - a).cross(z - a).dot(up) > 0.0;
                let order: Vec<usize> = if forward { (0..sk.len()).collect() } else { std::iter::once(0).chain((1..sk.len()).rev()).collect() };
                for l in 0..layers {
                    buf.clear();
                    for layer in [l, l + 1] {
                        for &i in &order {
                            let k = c[i][layer];
                            buf.push(own.get(&k).copied().unwrap_or_else(|| pb.refer(k)));
                        }
                    }
                    pb.push(ElemShape::Cell(cell), &buf);
                }
                // The claimed target element above, along the target normal.
                buf.clear();
                for col in &c {
                    let k = col[layers];
                    buf.push(own.get(&k).copied().unwrap_or_else(|| pb.refer(k)));
                }
                if flip_target {
                    buf[1..].reverse();
                }
                pb.owner(s.target);
                pb.push(shape, &buf);
            }
        }
        // Cells carry no owner; give them the volume so owners stay one per element.
        let mut patch = pb.finish();
        if !patch.owners.is_empty() {
            let mut owners = Vec::with_capacity(patch.elements.len());
            let mut it = patch.owners.iter();
            for (shape, _) in patch.elements.iter() {
                owners.push(match shape {
                    ElemShape::Cell(_) => vol,
                    _ => *it.next().unwrap_or(&s.target),
                });
            }
            patch.owners = owners;
        }
        Ok(patch)
    }
}

// ------------------------------------------------------------------ the test

fn run(fx: &Fixture, topo: &CellComplex) -> sm_scheme_api::Meshed<sm_mesh_api::reference::ReferenceMesh> {
    let mut reg = stub_schemes::registry();
    reg.register(Box::new(M2One)).unwrap();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let model = Model::new();
    let sel = [fx.volume];
    let input = MeshInput { topo, model: &model, pieces: &WholeFaces, selection: &sel };
    let mut intent = MeshIntent::default();
    for (e, n) in fx.edges.iter().zip([2u32, 2, 3]) {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(n)));
    }
    intent.set_scheme(fx.volume, SchemeChoice::with_params("m2one", serde_json::json!({ "sources": fx.sources, "target": fx.target }), Firmness::Hard));
    driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut SerialAlloc::new()).expect("many-to-one mesh")
}

#[test]
fn two_sources_sweep_into_one_target() {
    let (topo, fx) = fixture();
    let out = run(&fx, &topo);
    let plan = &out.plan;
    let vol = Target::Volume(fx.volume);
    for s in fx.sources {
        assert_eq!(plan.roles_of(Target::Surface(s)).collect::<Vec<_>>(), vec![Role::SweepSource { volume: vol }]);
        assert_eq!(plan.target(Target::Surface(s)).map(|p| p.scheme.as_str()), Some("map"));
    }
    assert_eq!(plan.roles_of(Target::Surface(fx.target)).collect::<Vec<_>>(), vec![Role::SweepTarget { volume: vol }]);
    assert!(matches!(plan.target(Target::Surface(fx.target)).map(|p| &p.selection), Some(Selection::Claimed { by }) if *by == vol));
    let m = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    // Sources of different densities: left 2 x 2 quads, right 1 x 2 (its x edges are free and
    // get one interval), times 3 layers.
    assert_eq!(m.cell_count(), 18);
    assert!((0..18).all(|c| m.cell_shape(sm_mesh_api::CellIx(c)) == CellShape::Hex));
    let total: f64 = (0..18).map(|c| cell_volume(m, sm_mesh_api::CellIx(c))).sum();
    assert!((total - 2.0).abs() < 1e-12, "{total}");
    // The target is one face meshed by the projection of both sources: 4 + 2 boundary quads at
    // z = 1.
    let on_target = m.classified_on(fx.target).expect("target faces");
    assert_eq!(on_target.faces.len(), 6);
    for f in &on_target.faces {
        assert!(m.is_boundary_face(*f));
        assert!(m.face_vertices(*f).iter().all(|v| (m.point(v.get()).z - 1.0).abs() < 1e-12));
    }
    // Interior target-face nodes include the image of the edge where the two sources meet.
    assert_eq!(on_target.vertices.len(), 2, "the image of the chord's interior node and of the left face's interior node");
    for (s, n) in fx.sources.iter().zip([4, 2]) {
        assert_eq!(m.classified_on(*s).expect("source faces").faces.len(), n);
    }
    // Deterministic.
    let again = run(&fx, &topo);
    assert_eq!(topology_hash(&again.finished.mesh), topology_hash(m));
}
