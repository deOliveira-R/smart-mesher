//! Stub scheme implementors used by the integration tests: `equal` (curve), `map` (surface, with
//! vertex types and side-sum interval constraints), `sweep` (1-to-1 translational volume sweep
//! with source/target/linking roles) and `broken` (a surface scheme that always fails, to test
//! fallback). They are deliberately small: planar faces, straight edges, quad sources.

#![allow(dead_code)]

use sm_core::{EntityId, Point3, Vector3};
use sm_mesh_api::{CellShape, Param};
use sm_model_api::{PieceRef, PieceShape};
use sm_scheme_api::vertex::{chain_nodes, logical_sides, loop_corners, turn_sum, LoopCorner};
use sm_scheme_api::{
    Applicability, ExecCtx, FailureKind, Firmness, Footprint, Imposition, IntervalConstraint, JobPlan, JobRequest, MeshJob, MeshPatch, NodeKey, Origin, ParamValue, PatchBuilder, PlanCtx, Requirement, Scheme, SchemeDim, SchemeFailure,
    SchemeName, SchemeRegistry, SweepRole, Target, VertexType, VertexTypeSetting,
};
use sm_topo_api::{Carrier, CornerKey, Sense, TopologyView, TopologyViewExt};
use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

pub fn registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    r.register(Arc::new(EqualCurve));
    r.register(Arc::new(MapSurface));
    r.register(Arc::new(Sweep11));
    r
}

fn point(topo: &dyn TopologyView, v: EntityId) -> Option<Point3> {
    match topo.carrier(v) {
        Carrier::Point(p) => Some(*p),
        _ => None,
    }
}

fn face_normal(topo: &dyn TopologyView, f: EntityId) -> Option<Vector3> {
    topo.face_moments(f).ok()?.unit_normal(topo.tolerance().linear)
}

fn sub(a: Point3, b: Point3) -> Vector3 {
    a - b
}

// ---------------------------------------------------------------- equal

/// Equal intervals in the curve parameter (equal arc length on lines).
pub struct EqualCurve;

impl Scheme for EqualCurve {
    fn name(&self) -> &str {
        "equal"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Curve
    }
    fn applicability(&self, cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Applicability {
        match req.targets {
            [Target::Topo(e)] if matches!(cx.topo.carrier(*e), Carrier::Curve(_)) => Applicability::Applicable { score: 10, reason: "edge with a curve".into() },
            _ => Applicability::NotApplicable { reason: "needs one edge with a curve carrier".into() },
        }
    }
    fn plan(&self, cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Result<JobPlan, SchemeFailure> {
        let e = req.targets[0].topo().ok_or_else(|| SchemeFailure::fatal("equal", req.targets, FailureKind::Topology, "not an edge"))?;
        let (s, t) = cx.topo.edge_ends(e).ok_or_else(|| SchemeFailure::fatal("equal", req.targets, FailureKind::Topology, "edge without ends"))?;
        Ok(JobPlan { footprint: Footprint::default().read([Target::Topo(s), Target::Topo(t)]).write([req.targets[0]]), ..JobPlan::default() })
    }
    fn execute(&self, cx: &ExecCtx<'_>, job: &MeshJob) -> Result<MeshPatch, SchemeFailure> {
        let fail = |k, m: &str| SchemeFailure::fatal("equal", &job.targets, k, m);
        let target = job.targets[0];
        let e = target.topo().ok_or_else(|| fail(FailureKind::Topology, "not an edge"))?;
        let (s, t) = cx.topo.edge_ends(e).ok_or_else(|| fail(FailureKind::Topology, "no ends"))?;
        let n = cx.intervals.get(e).ok_or_else(|| fail(FailureKind::Intervals, "no interval count"))?;
        let Carrier::Curve(c) = cx.topo.carrier(e) else { return Err(fail(FailureKind::Geometry, "no curve")) };
        let curve = cx.topo.shapes().curve(c.curve).ok_or_else(|| fail(FailureKind::Geometry, "unknown curve"))?;
        let mut b = PatchBuilder::new();
        let mut list = Vec::with_capacity(n as usize + 1);
        list.push(b.reference(NodeKey::vertex(s)));
        for j in 1..n {
            let f = f64::from(j) / f64::from(n);
            let u = if c.sense == Sense::Forward { c.range.lo + f * (c.range.hi - c.range.lo) } else { c.range.hi - f * (c.range.hi - c.range.lo) };
            list.push(b.new_node(NodeKey::edge(e, j), curve.eval(u), Param::Curve(u)));
        }
        list.push(b.reference(NodeKey::vertex(t)));
        b.curve(target, &list);
        Ok(b.finish())
    }
}

// ---------------------------------------------------------------- map

/// Interior angle at each corner of the outer loop (straight edges, planar face).
fn corner_angles(topo: &dyn TopologyView, f: EntityId, corners: &[LoopCorner]) -> Option<Vec<f64>> {
    let n = face_normal(topo, f)?;
    let k = corners.len();
    let mut out = Vec::with_capacity(k);
    for i in 0..k {
        let prev = point(topo, corners[(i + k - 1) % k].vertex)?;
        let cur = point(topo, corners[i].vertex)?;
        let next = point(topo, corners[(i + 1) % k].vertex)?;
        let (din, dout) = (sub(cur, prev), sub(next, cur));
        let turn = n.dot(din.cross(dout)).atan2(din.dot(dout));
        out.push(core::f64::consts::PI - turn);
    }
    Some(out)
}

/// Vertex types of a face's outer loop: intent (hard) first, else from the corner angle (soft).
fn face_types(topo: &dyn TopologyView, types: &BTreeMap<CornerKey, VertexTypeSetting>, f: EntityId) -> Option<(Vec<LoopCorner>, Vec<VertexTypeSetting>)> {
    let corners = loop_corners(topo, f, 0);
    let angles = corner_angles(topo, f, &corners)?;
    let set = corners
        .iter()
        .zip(&angles)
        .map(|(c, &a)| types.get(&c.key).copied().unwrap_or(VertexTypeSetting { ty: VertexType::from_angle(a), firmness: Firmness::Soft }))
        .collect();
    Some((corners, set))
}

/// Corners, their types and the four logical sides of a mapped face.
type MapSides = (Vec<LoopCorner>, Vec<VertexTypeSetting>, Vec<Vec<sm_topo_api::CoedgeKey>>);

/// Transfinite map of a face with four logical sides.
pub struct MapSurface;

impl MapSurface {
    fn sides(cx_topo: &dyn TopologyView, types: &BTreeMap<CornerKey, VertexTypeSetting>, f: EntityId) -> Result<MapSides, String> {
        if cx_topo.loops(f).count() != 1 {
            return Err("map needs exactly one loop".into());
        }
        let (corners, set) = face_types(cx_topo, types, f).ok_or("no corner geometry")?;
        let ty = |k: &CornerKey| corners.iter().position(|c| c.key == *k).map_or(VertexType::Side, |i| set[i].ty);
        if turn_sum(&corners, ty) != 4 {
            return Err("vertex types do not sum to 4 quarter turns".into());
        }
        let sides = logical_sides(&corners, ty);
        if sides.len() != 4 {
            return Err(format!("{} logical sides, need 4", sides.len()));
        }
        Ok((corners, set, sides))
    }
}

impl Scheme for MapSurface {
    fn name(&self) -> &str {
        "map"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn applicability(&self, cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Applicability {
        let [Target::Topo(f)] = req.targets else { return Applicability::NotApplicable { reason: "one topology face".into() } };
        match Self::sides(cx.topo, &cx.intent.vertex_types, *f) {
            Ok(_) => Applicability::Applicable { score: 100, reason: "4 logical sides".into() },
            Err(reason) => Applicability::NotApplicable { reason },
        }
    }
    fn plan(&self, cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Result<JobPlan, SchemeFailure> {
        let fail = |m: String| SchemeFailure::fallback("map", req.targets, FailureKind::NotApplicable, m);
        let f = req.targets[0].topo().ok_or_else(|| fail("not a face".into()))?;
        let (corners, set, sides) = Self::sides(cx.topo, &cx.intent.vertex_types, f).map_err(fail)?;
        let edges = |s: &[sm_topo_api::CoedgeKey]| s.iter().map(|k| k.edge).collect::<Vec<_>>();
        let origin = Origin::Scheme { scheme: SchemeName::new("map"), target: req.targets[0] };
        let constraints = vec![
            IntervalConstraint::sums_equal(&edges(&sides[0]), &edges(&sides[2]), Firmness::Hard, origin.clone()),
            IntervalConstraint::sums_equal(&edges(&sides[1]), &edges(&sides[3]), Firmness::Hard, origin),
        ];
        let vertex_types = corners.iter().zip(&set).map(|(c, s)| (c.key, *s)).collect();
        let reads: BTreeSet<Target> = cx.topo.face_edges(f).iter().map(|&e| Target::Topo(e)).chain(corners.iter().map(|c| Target::Topo(c.vertex))).collect();
        Ok(JobPlan { footprint: Footprint::default().read(reads).write([req.targets[0]]), constraints, vertex_types, ..JobPlan::default() })
    }
    fn execute(&self, cx: &ExecCtx<'_>, job: &MeshJob) -> Result<MeshPatch, SchemeFailure> {
        let target = job.targets[0];
        let fail = |k, m: String| SchemeFailure::fallback("map", &job.targets, k, m);
        let f = target.topo().ok_or_else(|| fail(FailureKind::Topology, "not a face".into()))?;
        let (_, _, sides) = Self::sides(cx.topo, cx.vertex_types, f).map_err(|m| fail(FailureKind::NotApplicable, m))?;
        let s: Vec<Vec<NodeKey>> = sides.iter().map(|sd| chain_nodes(cx.ledger, sd)).collect::<Option<_>>().ok_or_else(|| fail(FailureKind::Topology, "side not meshed".into()))?;
        let (n, m) = (s[0].len() - 1, s[1].len() - 1);
        if s[2].len() != n + 1 || s[3].len() != m + 1 {
            return Err(fail(FailureKind::Intervals, "opposite sides differ".into()));
        }
        let mut b = PatchBuilder::new();
        let pt = |k: &NodeKey| cx.ledger.point(k).ok_or_else(|| fail(FailureKind::Topology, format!("{k:?} not committed")));
        // grid[j * (n + 1) + i] = local node index
        let mut grid = vec![0u32; (n + 1) * (m + 1)];
        let bnd = |i: usize, j: usize| -> Option<NodeKey> {
            if j == 0 {
                Some(s[0][i])
            } else if i == n {
                Some(s[1][j])
            } else if j == m {
                Some(s[2][n - i])
            } else if i == 0 {
                Some(s[3][m - j])
            } else {
                None
            }
        };
        let (p00, pn0, pnm, p0m) = (pt(&s[0][0])?, pt(&s[1][0])?, pt(&s[2][0])?, pt(&s[3][0])?);
        for j in 0..=m {
            for i in 0..=n {
                grid[j * (n + 1) + i] = match bnd(i, j) {
                    Some(k) => b.reference(k),
                    None => {
                        let (u, v) = (i as f64 / n as f64, j as f64 / m as f64);
                        let (bt, tp, lf, rt) = (pt(&s[0][i])?, pt(&s[2][n - i])?, pt(&s[3][m - j])?, pt(&s[1][j])?);
                        let c = |a: fn(&Point3) -> f64| {
                            (1.0 - v) * a(&bt) + v * a(&tp) + (1.0 - u) * a(&lf) + u * a(&rt) - ((1.0 - u) * (1.0 - v) * a(&p00) + u * (1.0 - v) * a(&pn0) + u * v * a(&pnm) + (1.0 - u) * v * a(&p0m))
                        };
                        let p = Point3::new(c(|p| p.x), c(|p| p.y), c(|p| p.z));
                        b.new_node(NodeKey::interior(target, ((j - 1) * (n - 1) + (i - 1)) as u32), p, Param::None)
                    }
                };
            }
        }
        b.begin_face(target);
        for j in 0..m {
            for i in 0..n {
                let g = |i: usize, j: usize| grid[j * (n + 1) + i];
                b.face(&[g(i, j), g(i + 1, j), g(i + 1, j + 1), g(i, j + 1)]);
            }
        }
        Ok(b.finish())
    }
}

// ---------------------------------------------------------------- sweep

/// The topology of a 1-to-1 sweep, derived from the volume and its source/target faces.
struct SweepTopo {
    volume: Target,
    source: EntityId,
    target: EntityId,
    linking: Vec<EntityId>,
    /// source vertex -> (linking edge, target vertex, edge runs source->target)
    zedge: BTreeMap<EntityId, (EntityId, EntityId, bool)>,
    /// source edge -> (target edge, same direction, linking face)
    across: BTreeMap<EntityId, (EntityId, bool, EntityId)>,
}

fn sweep_faces(cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Result<(Target, EntityId, EntityId), String> {
    let [vol] = req.targets else { return Err("1-to-1 sweep meshes one volume".into()) };
    if let (Some([s]), Some([t])) = (req.params.ids("source"), req.params.ids("target")) {
        return Ok((*vol, *s, *t));
    }
    // A decomposition piece declared sweepable carries its source/target.
    if let Target::Piece(PieceRef::Cell(c)) = vol
        && let Some(p) = cx.model.decomposition().piece(*c)
        && let PieceShape::Sweep { source, target } = &p.shape
        && let ([s], [t]) = (source.as_slice(), target.as_slice())
        && let (Some(s), Some(t)) = (s.base(), t.base())
    {
        return Ok((*vol, s, t));
    }
    Err("needs one source and one target face (params or PieceShape::Sweep)".into())
}

fn sweep_topo(topo: &dyn TopologyView, vol: Target, source: EntityId, target: EntityId) -> Result<SweepTopo, String> {
    let v = match vol {
        Target::Topo(v) | Target::Piece(PieceRef::Whole(v)) => v,
        _ => return Err("stub sweep handles whole volumes only".into()),
    };
    let faces = topo.volume_faces(v);
    if !faces.contains(&source) || !faces.contains(&target) || source == target {
        return Err("source/target must be distinct faces of the volume".into());
    }
    let se: BTreeSet<EntityId> = topo.face_edges(source).iter().copied().collect();
    let te: BTreeSet<EntityId> = topo.face_edges(target).iter().copied().collect();
    if !se.is_disjoint(&te) {
        return Err("source and target share an edge".into());
    }
    let linking: Vec<EntityId> = faces.iter().copied().filter(|&f| f != source && f != target).collect();
    let sv: BTreeSet<EntityId> = se.iter().filter_map(|&e| topo.edge_ends(e)).flat_map(|(a, b)| [a, b]).collect();
    let mut zedge = BTreeMap::new();
    for &f in &linking {
        for &e in topo.face_edges(f) {
            if se.contains(&e) || te.contains(&e) {
                continue;
            }
            let (a, b) = topo.edge_ends(e).ok_or("edge without ends")?;
            if sv.contains(&a) {
                zedge.insert(a, (e, b, true));
            } else if sv.contains(&b) {
                zedge.insert(b, (e, a, false));
            }
        }
    }
    if zedge.len() != sv.len() {
        return Err("not every source vertex has one linking edge".into());
    }
    let mut across = BTreeMap::new();
    for &e in &se {
        let (a, b) = topo.edge_ends(e).ok_or("edge without ends")?;
        let (ta, tb) = (zedge[&a].1, zedge[&b].1);
        let (et, same) = te
            .iter()
            .find_map(|&x| match topo.edge_ends(x) {
                Some((p, q)) if p == ta && q == tb => Some((x, true)),
                Some((p, q)) if p == tb && q == ta => Some((x, false)),
                _ => None,
            })
            .ok_or("no target edge for a source edge")?;
        let lf = linking.iter().copied().find(|&f| topo.face_edges(f).contains(&e)).ok_or("no linking face")?;
        if topo.face_edges(lf).len() != 4 {
            return Err("stub sweep needs four-sided linking faces".into());
        }
        across.insert(e, (et, same, lf));
    }
    Ok(SweepTopo { volume: vol, source, target, linking, zedge, across })
}

/// 1-to-1 translational sweep: the source face (any scheme, read) is extruded layer by layer;
/// the sweep writes the target face, the linking faces and the cells.
pub struct Sweep11;

impl Scheme for Sweep11 {
    fn name(&self) -> &str {
        "sweep"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn applicability(&self, cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Applicability {
        match sweep_faces(cx, req).and_then(|(v, s, t)| sweep_topo(cx.topo, v, s, t)) {
            Ok(st) => Applicability::Applicable { score: 50, reason: format!("{} linking faces", st.linking.len()) },
            Err(reason) => Applicability::NotApplicable { reason },
        }
    }
    fn plan(&self, cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Result<JobPlan, SchemeFailure> {
        let fail = |m: String| SchemeFailure::fallback("sweep", req.targets, FailureKind::NotApplicable, m);
        let (v, s, t) = sweep_faces(cx, req).map_err(fail)?;
        let st = sweep_topo(cx.topo, v, s, t).map_err(fail)?;
        let origin = Origin::Scheme { scheme: SchemeName::new("sweep"), target: v };
        let mut constraints = Vec::new();
        // Equal layer counts along every source -> target path.
        let z: Vec<EntityId> = st.zedge.values().map(|x| x.0).collect();
        for w in z.windows(2) {
            constraints.push(IntervalConstraint::sums_equal(&[w[0]], &[w[1]], Firmness::Hard, origin.clone()));
        }
        // Target edges match their source edges.
        for (&e, &(et, _, _)) in &st.across {
            constraints.push(IntervalConstraint::sums_equal(&[e], &[et], Firmness::Hard, origin.clone()));
        }
        // Linking faces must be mappable: their four corners are End (soft).
        let mut vertex_types = Vec::new();
        for &lf in &st.linking {
            for c in loop_corners(cx.topo, lf, 0) {
                vertex_types.push((c.key, VertexTypeSetting { ty: VertexType::End, firmness: Firmness::Soft }));
            }
        }
        let vol = match v {
            Target::Topo(x) | Target::Piece(PieceRef::Whole(x)) => x,
            _ => return Err(fail("whole volumes only".into())),
        };
        let edges: Vec<Target> = cx.topo.volume_faces(vol).iter().flat_map(|&f| cx.topo.face_edges(f).iter().map(|&e| Target::Topo(e))).collect();
        let mut roles = BTreeMap::new();
        roles.insert(Target::Topo(s), SweepRole::Source);
        roles.insert(Target::Topo(t), SweepRole::Target);
        for &lf in &st.linking {
            roles.insert(Target::Topo(lf), SweepRole::Linking);
        }
        let impositions = vec![Imposition { target: Target::Topo(s), requirement: Requirement::Role(SweepRole::Source), firmness: Firmness::Soft }];
        let footprint = Footprint::default().read([Target::Topo(s)]).read(edges).write([v, Target::Topo(t)]).write(st.linking.iter().map(|&f| Target::Topo(f)));
        Ok(JobPlan { footprint, constraints, vertex_types, impositions, roles })
    }
    fn execute(&self, cx: &ExecCtx<'_>, job: &MeshJob) -> Result<MeshPatch, SchemeFailure> {
        let fail = |k, m: String| SchemeFailure::fallback("sweep", &job.targets, k, m);
        let v = job.targets[0];
        let s = job.roles.iter().find(|(_, r)| **r == SweepRole::Source).and_then(|(t, _)| t.topo()).ok_or_else(|| fail(FailureKind::Internal, "no source role".into()))?;
        let t = job.roles.iter().find(|(_, r)| **r == SweepRole::Target).and_then(|(t, _)| t.topo()).ok_or_else(|| fail(FailureKind::Internal, "no target role".into()))?;
        let st = sweep_topo(cx.topo, v, s, t).map_err(|m| fail(FailureKind::Topology, m))?;
        let layers: BTreeSet<u32> = st.zedge.values().filter_map(|z| cx.intervals.get(z.0)).collect();
        let l_max = match layers.iter().collect::<Vec<_>>().as_slice() {
            [l] => **l,
            _ => return Err(fail(FailureKind::Intervals, "linking edges have different counts".into())),
        };
        let src = cx.ledger.face(&Target::Topo(s)).ok_or_else(|| fail(FailureKind::Topology, "source not meshed".into()))?;
        let (v0, (_, tv0, _)) = st.zedge.iter().next().ok_or_else(|| fail(FailureKind::Topology, "no vertex".into()))?;
        let pt = |k: &NodeKey| cx.ledger.point(k);
        let d = match (pt(&NodeKey::vertex(*v0)), pt(&NodeKey::vertex(*tv0))) {
            (Some(a), Some(b)) => b - a,
            _ => return Err(fail(FailureKind::Topology, "vertex nodes missing".into())),
        };
        let lm = l_max;
        // Node of source node `k` at layer `l`.
        let key_at = |k: &NodeKey, l: u32| -> Option<NodeKey> {
            if l == 0 {
                return Some(*k);
            }
            if k.owner == Target::Topo(s) {
                return Some(if l == lm { NodeKey::interior(Target::Topo(t), k.local) } else { NodeKey::interior(v, k.local * (lm - 1) + (l - 1)) });
            }
            let id = k.owner.topo()?;
            if let Some(&(ze, tv, fwd)) = st.zedge.get(&id) {
                return Some(if l == lm { NodeKey::vertex(tv) } else { NodeKey::edge(ze, if fwd { l } else { lm - l }) });
            }
            let &(et, same, lf) = st.across.get(&id)?;
            if l == lm {
                let n = cx.ledger.curve(&Target::Topo(id))?.len() as u32 - 1;
                return Some(NodeKey::edge(et, if same { k.local } else { n - k.local }));
            }
            Some(NodeKey::interior(Target::Topo(lf), (k.local - 1) * (lm - 1) + (l - 1)))
        };
        let mut b = PatchBuilder::new();
        let local = |b: &mut PatchBuilder, k: &NodeKey, l: u32| -> Result<u32, SchemeFailure> {
            let key = key_at(k, l).ok_or_else(|| fail(FailureKind::Topology, format!("no image of {k:?}")))?;
            if let Some(i) = b.local(&key) {
                return Ok(i);
            }
            if cx.ledger.node(&key).is_some() {
                return Ok(b.reference(key));
            }
            let p0 = pt(k).ok_or_else(|| fail(FailureKind::Topology, format!("{k:?} not committed")))?;
            let f = f64::from(l) / f64::from(lm);
            Ok(b.new_node(key, p0 + d * f, Param::None))
        };
        // Target face: images of the source elements, reversed (both faces point out of the volume).
        let mut el: Vec<u32> = Vec::with_capacity(4);
        b.begin_face(Target::Topo(t));
        for k in 0..src.len() {
            el.clear();
            for n in src.element(k).iter().rev() {
                el.push(local(&mut b, n, lm)?);
            }
            b.face(&el);
        }
        // Linking faces, one part each, quads oriented by the face normal.
        for &lf in &st.linking {
            b.begin_face(Target::Topo(lf));
            let normal = face_normal(cx.topo, lf);
            for (&e, &(_, _, f)) in &st.across {
                if f != lf {
                    continue;
                }
                let nodes = cx.ledger.curve(&Target::Topo(e)).ok_or_else(|| fail(FailureKind::Topology, "source edge not meshed".into()))?.to_vec();
                for w in nodes.windows(2) {
                    // Quad (w0,l) (w1,l) (w1,l+1) (w0,l+1) has normal along (p1 - p0) x d.
                    let flip = match (normal, pt(&w[0]), pt(&w[1])) {
                        (Some(n), Some(p0), Some(p1)) => n.dot((p1 - p0).cross(d)) < 0.0,
                        _ => false,
                    };
                    for l in 0..lm {
                        let mut q = [local(&mut b, &w[0], l)?, local(&mut b, &w[1], l)?, local(&mut b, &w[1], l + 1)?, local(&mut b, &w[0], l + 1)?];
                        if flip {
                            q.reverse();
                        }
                        b.face(&q);
                    }
                }
            }
        }
        // Cells: source quads reversed (normal along the sweep) give the hex bottom.
        b.begin_cells(v);
        for k in 0..src.len() {
            let q = src.element(k);
            if q.len() != 4 {
                return Err(fail(FailureKind::Unsupported, "stub sweep needs quad sources".into()));
            }
            let order = [q[0], q[3], q[2], q[1]];
            for l in 0..lm {
                let mut c = [0u32; 8];
                for (i, n) in order.iter().enumerate() {
                    c[i] = local(&mut b, n, l)?;
                    c[i + 4] = local(&mut b, n, l + 1)?;
                }
                b.cell(CellShape::Hex, &c);
            }
        }
        Ok(b.finish())
    }
}

// ---------------------------------------------------------------- broken

/// A surface scheme that is always applicable (highest score) and always fails at execution.
pub struct Broken;

impl Scheme for Broken {
    fn name(&self) -> &str {
        "broken"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _req: &JobRequest<'_>) -> Applicability {
        Applicability::Applicable { score: 1000, reason: "always".into() }
    }
    fn plan(&self, cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Result<JobPlan, SchemeFailure> {
        let reads: Vec<Target> = req.targets.iter().filter_map(|t| t.topo()).flat_map(|f| cx.topo.face_edges(f).iter().map(|&e| Target::Topo(e))).collect();
        Ok(JobPlan { footprint: Footprint::default().read(reads).write(req.targets.iter().copied()), ..JobPlan::default() })
    }
    fn execute(&self, _cx: &ExecCtx<'_>, job: &MeshJob) -> Result<MeshPatch, SchemeFailure> {
        Err(SchemeFailure::fallback("broken", &job.targets, FailureKind::Quality, "inverted elements"))
    }
}

/// Sweep parameters for a source and a target face.
pub fn sweep_params(source: EntityId, target: EntityId) -> sm_scheme_api::SchemeSpec {
    sm_scheme_api::SchemeSpec::named("sweep").with("source", ParamValue::Ids(vec![source])).with("target", ParamValue::Ids(vec![target]))
}

// ---------------------------------------------------------------- transform

/// A transform that rewrites the cells of all its targets in one call (stand-in for THex):
/// it re-emits each target's cells unchanged.
pub struct NoopTransform;

impl Scheme for NoopTransform {
    fn name(&self) -> &str {
        "noop-transform"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Transform
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Applicability {
        if req.targets.iter().all(|t| t.dim() == Some(3)) {
            Applicability::Applicable { score: 1, reason: "volumes".into() }
        } else {
            Applicability::NotApplicable { reason: "volumes only".into() }
        }
    }
    fn plan(&self, _cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Result<JobPlan, SchemeFailure> {
        Ok(JobPlan { footprint: Footprint::default().write(req.targets.iter().copied()), ..JobPlan::default() })
    }
    fn execute(&self, cx: &ExecCtx<'_>, job: &MeshJob) -> Result<MeshPatch, SchemeFailure> {
        let mut b = PatchBuilder::new();
        for &t in &job.targets {
            let cm = cx.ledger.cells(&t).ok_or_else(|| SchemeFailure::fatal("noop-transform", &job.targets, FailureKind::Topology, "not meshed"))?;
            b.begin_cells(t);
            for k in 0..cm.len() {
                let c: Vec<u32> = cm.corners(k).iter().map(|n| b.reference(*n)).collect();
                b.cell(cm.shapes[k], &c);
            }
        }
        Ok(b.finish())
    }
}
