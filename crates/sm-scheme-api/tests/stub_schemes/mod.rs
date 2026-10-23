//! Stub scheme implementors used by the integration tests: `equal` (curve), `map` (surface,
//! vertex rule + opposite-side interval equations, transfinite interior) and `sweep`
//! (1-to-1 volume sweep with source/target/linking roles). Plus a brick fixture and a
//! registry helper. Shared by several test binaries.
#![allow(dead_code)]

use core::f64::consts::FRAC_PI_2;
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_mesh_api::{CellShape, Param};
use sm_model_api::Model;
use sm_scheme_api::geom;
use sm_scheme_api::{
    Applicability, CopyConstraint, CopyReason, ElemShape, GenCtx, GenerateFailure, IntervalCtx, IntervalSink, MeshInput, NodeGrid, NodeKey, Params, Patch, PatchNode, PlanCtx, Requirements, Role, Scheme, SchemeDim, SchemeRegistry, Target, VertexRule,
};
use sm_topo_api::fixtures::{BrickIds, brick, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

// ---------------------------------------------------------------- equal (curve)

/// Uniform in curve parameter (= uniform in arc length for lines).
pub struct Equal;

impl Scheme for Equal {
    fn name(&self) -> &str {
        "equal"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Curve
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, _p: &Params) -> Applicability {
        let Target::Curve(e) = target else { return Applicability::no("not a curve") };
        match geom::edge_curve(cx.input.topo, e) {
            Ok(_) => Applicability::yes(50, "uniform in curve parameter"),
            Err(err) => Applicability::no(err.to_string()),
        }
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        let fatal = |r: String| GenerateFailure::fatal(target, "equal", r);
        let Target::Curve(e) = target else { return Err(fatal("not a curve".into())) };
        let n = cx.intervals(e).ok_or_else(|| fatal("no intervals".into()))?;
        let (c, cc) = geom::edge_curve(cx.input.topo, e).map_err(|x| fatal(x.to_string()))?;
        let mut patch = Patch::new(target);
        for i in 1..n {
            let t = geom::edge_param(&cc, f64::from(i) / f64::from(n));
            patch.nodes.push(PatchNode { key: NodeKey::new(e, i), point: c.eval(t), param: Param::Curve(t) });
        }
        let chain = cx.edge_nodes(e).ok_or_else(|| fatal("edge has no ends".into()))?;
        for w in chain.windows(2) {
            patch.elements.push(ElemShape::Segment, w);
        }
        Ok(patch)
    }
}

// ---------------------------------------------------------------- map (surface)

/// Mapping: four logical sides, opposite sides equal, transfinite (Coons) interior.
pub struct Map;

impl Scheme for Map {
    fn name(&self) -> &str {
        "map"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, _p: &Params) -> Applicability {
        let Target::Surface(f) = target else { return Applicability::no("not a surface") };
        let corners = match geom::corners(cx.input.topo, f) {
            Ok(c) => c,
            Err(e) => return Applicability::no(e.to_string()),
        };
        if corners.iter().any(|c| c.ring != 0) {
            return Applicability::no("map needs a single loop");
        }
        if corners.len() < 4 {
            return Applicability::no("fewer than four corners");
        }
        let fuzzy = 0.35;
        let ends = corners.iter().filter(|c| (c.angle - FRAC_PI_2).abs() < fuzzy).count();
        let sides = corners.iter().filter(|c| (c.angle - 2.0 * FRAC_PI_2).abs() < fuzzy).count();
        if ends == 4 && ends + sides == corners.len() { Applicability::yes(100, "four End corners") } else { Applicability::yes(30, "corners need picking") }
    }
    fn requirements(&self, _cx: &PlanCtx<'_>, target: Target, _p: &Params, out: &mut Requirements<'_>) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.vertex_rule(f, VertexRule::Map);
        }
        Ok(())
    }
    fn interval_constraints(&self, cx: &IntervalCtx<'_>, target: Target, _p: &Params, out: &mut IntervalSink<'_>) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else { return Ok(()) };
        let layout = cx.layout(f).ok_or_else(|| sm_scheme_api::SchemeError::Input(format!("map face {f} has no layout")))?;
        let edges = |k: usize| -> Vec<EntityId> { layout.sides[k].coedges.iter().map(|c| c.edge).collect() };
        if layout.sides.len() != 4 {
            return Err(sm_scheme_api::SchemeError::Input(format!("map face {f}: {} sides", layout.sides.len())));
        }
        out.equal(&edges(0), &edges(2));
        out.equal(&edges(1), &edges(3));
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> Result<Patch, GenerateFailure> {
        let fatal = |r: String| GenerateFailure::fatal(target, "map", r);
        let Target::Surface(f) = target else { return Err(fatal("not a surface".into())) };
        let side = |k: usize| cx.side_nodes(f, k).ok_or_else(|| fatal(format!("side {k} unavailable")));
        let (s0, s1, s2, s3) = (side(0)?, side(1)?, side(2)?, side(3)?);
        if s0.len() != s2.len() || s1.len() != s3.len() {
            return Err(fatal("opposite sides differ (solver bug)".into()));
        }
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
        // Normalized arc-length fractions along a boundary line.
        let frac = |line: &[Point3]| -> Vec<f64> {
            let mut acc = vec![0.0; line.len()];
            for q in 1..line.len() {
                acc[q] = acc[q - 1] + (line[q] - line[q - 1]).norm();
            }
            let total = acc[line.len() - 1].max(1e-300);
            acc.iter().map(|a| a / total).collect()
        };
        let row = |j: usize| -> Result<Vec<Point3>, GenerateFailure> { (0..ni).map(|i| pt(keys[j * ni + i])).collect() };
        let col = |i: usize| -> Result<Vec<Point3>, GenerateFailure> { (0..nj).map(|j| pt(keys[j * ni + i])).collect() };
        let (bot, top, lef, rig) = (row(0)?, row(nj - 1)?, col(0)?, col(ni - 1)?);
        let (ub, ut, vl, vr) = (frac(&bot), frac(&top), frac(&lef), frac(&rig));
        let surf = match cx.input.topo.carrier(f) {
            sm_topo_api::Carrier::Surface(sc) => cx.input.topo.shapes().surface(sc.surface),
            _ => None,
        };
        let mut patch = Patch::new(target);
        let lerp = |a: Point3, b: Point3, t: f64| a + (b - a) * t;
        for j in 1..nj - 1 {
            for i in 1..ni - 1 {
                let (u, v) = (0.5 * (ub[i] + ut[i]), 0.5 * (vl[j] + vr[j]));
                let edges = lerp(bot[i], top[i], v) - Point3::default();
                let sides = lerp(lef[j], rig[j], u) - Point3::default();
                let corners = lerp(lerp(bot[0], bot[ni - 1], u), lerp(top[0], top[ni - 1], u), v) - Point3::default();
                let mut p = Point3::default() + (edges + sides - corners);
                let mut param = Param::None;
                if let Some(s) = surf
                    && let Ok(pr) = s.project(p, None)
                {
                    p = pr.point;
                    param = Param::Surface([pr.uv.u, pr.uv.v]);
                }
                let key = NodeKey::new(f, ((j - 1) * (ni - 2) + (i - 1)) as u32);
                keys[j * ni + i] = key;
                patch.nodes.push(PatchNode { key, point: p, param });
            }
        }
        for j in 0..nj - 1 {
            for i in 0..ni - 1 {
                let q = [keys[j * ni + i], keys[j * ni + i + 1], keys[(j + 1) * ni + i + 1], keys[(j + 1) * ni + i]];
                patch.elements.push(ElemShape::Quad, &q);
            }
        }
        patch.grid = Some(NodeGrid { ni: ni as u32, nj: nj as u32, nodes: keys });
        Ok(patch)
    }
}

// ---------------------------------------------------------------- sweep (volume)

/// 1-to-1 sweep: source face meshed by any scheme, target = translated copy, linking faces
/// map/submap, layers along the linking edges.
pub struct Sweep;

/// Resolved sweep topology, computed identically in every phase.
struct SweepTopo {
    source: EntityId,
    target: EntityId,
    linking: Vec<EntityId>,
    /// Source vertex -> (linking edge, the vertex is the edge's start).
    columns: BTreeMap<EntityId, (EntityId, bool)>,
}

fn ids_param(p: &Params, key: &str) -> Result<Option<Vec<EntityId>>, String> {
    match p.get::<Vec<EntityId>>("sweep", key) {
        Ok(v) => Ok(v),
        Err(_) => p.get::<EntityId>("sweep", key).map(|o| o.map(|x| vec![x])).map_err(|e| e.to_string()),
    }
}

fn face_vertices(t: &dyn TopologyView, f: EntityId) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = t.face_edges(f).iter().filter_map(|e| t.edge_ends(*e)).flat_map(|(a, b)| [a, b]).collect();
    v.sort_unstable();
    v.dedup();
    v
}

fn sweep_topo(input: &MeshInput<'_>, target: Target, p: &Params) -> Result<SweepTopo, String> {
    let t = input.topo;
    let faces = input.piece_faces(target).map_err(|e| e.to_string())?;
    let try_pair = |s: EntityId, g: EntityId| -> Result<SweepTopo, String> {
        let (vs, vg) = (face_vertices(t, s), face_vertices(t, g));
        if vs.iter().any(|v| vg.contains(v)) {
            return Err("source and target touch".into());
        }
        let mut columns = BTreeMap::new();
        let piece_edges: Vec<EntityId> = {
            let mut e: Vec<EntityId> = faces.iter().flat_map(|f| input.face_edges(*f)).collect();
            e.sort_unstable();
            e.dedup();
            e
        };
        for v in &vs {
            let hits: Vec<(EntityId, bool)> = piece_edges
                .iter()
                .filter_map(|e| {
                    let (a, b) = t.edge_ends(*e)?;
                    if a == *v && vg.contains(&b) {
                        Some((*e, true))
                    } else if b == *v && vg.contains(&a) {
                        Some((*e, false))
                    } else {
                        None
                    }
                })
                .collect();
            match hits.as_slice() {
                [h] => {
                    columns.insert(*v, *h);
                }
                _ => return Err(format!("vertex {v} needs exactly one linking edge (stub: single-edge linking)")),
            }
        }
        if vs.len() != vg.len() {
            return Err("many-to-one/many-to-many sweeps are not in the stub".into());
        }
        let linking: Vec<EntityId> = faces.iter().copied().filter(|f| *f != s && *f != g).collect();
        for l in &linking {
            if input.face_edges(*l).len() != 4 {
                return Err(format!("linking face {l} is not four-sided (stub)"));
            }
        }
        Ok(SweepTopo { source: s, target: g, linking, columns })
    };
    match (ids_param(p, "source")?, ids_param(p, "target")?) {
        (Some(s), Some(g)) => match (s.as_slice(), g.as_slice()) {
            ([s], [g]) => try_pair(*s, *g),
            _ => Err("only 1-to-1 sweeps are in the stub".into()),
        },
        (None, None) => {
            let mut last = String::from("no source/target pair");
            for (i, &s) in faces.iter().enumerate() {
                for &g in &faces[i + 1..] {
                    match try_pair(s, g) {
                        Ok(st) => return Ok(st),
                        Err(e) => last = e,
                    }
                }
            }
            Err(last)
        }
        _ => Err("give both source and target, or neither".into()),
    }
}

fn mean(t: &dyn TopologyView, vs: &[EntityId]) -> Vector3 {
    let mut acc = Vector3::default();
    for v in vs {
        if let Ok(p) = geom::vertex_point(t, *v) {
            acc = acc + (p - Point3::default());
        }
    }
    acc * (1.0 / vs.len().max(1) as f64)
}

impl Scheme for Sweep {
    fn name(&self) -> &str {
        "sweep"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, p: &Params) -> Applicability {
        match sweep_topo(cx.input, target, p) {
            Ok(_) => Applicability::yes(60, "1-to-1 sweepable"),
            Err(e) => Applicability::no(e),
        }
    }
    fn requirements(&self, cx: &PlanCtx<'_>, target: Target, p: &Params, out: &mut Requirements<'_>) -> sm_scheme_api::Result<()> {
        let st = sweep_topo(cx.input, target, p).map_err(|e| sm_scheme_api::SchemeError::NotApplicable { target, scheme: sm_scheme_api::SchemeName::new("sweep"), reason: e })?;
        out.role(Target::Surface(st.source), Role::SweepSource { volume: target });
        out.role(Target::Surface(st.target), Role::SweepTarget { volume: target });
        for l in &st.linking {
            out.role(Target::Surface(*l), Role::SweepLinking { volume: target });
            out.impose(Target::Surface(*l), &["map", "submap"]);
        }
        let t = cx.input.topo;
        let vm: Vec<(EntityId, EntityId)> = st
            .columns
            .iter()
            .filter_map(|(v, (e, start))| {
                let (a, b) = t.edge_ends(*e)?;
                Some((*v, if *start { b } else { a }))
            })
            .collect();
        let shift = mean(t, &vm.iter().map(|x| x.1).collect::<Vec<_>>()) - mean(t, &vm.iter().map(|x| x.0).collect::<Vec<_>>());
        out.copy(CopyConstraint { source: Target::Surface(st.source), target: Target::Surface(st.target), transform: Transform::translation(shift), vertex_map: vm, reason: CopyReason::SweepTarget { volume: target } });
        Ok(())
    }
    fn interval_constraints(&self, cx: &IntervalCtx<'_>, target: Target, p: &Params, out: &mut IntervalSink<'_>) -> sm_scheme_api::Result<()> {
        let st = sweep_topo(cx.input, target, p).map_err(sm_scheme_api::SchemeError::Input)?;
        let links: Vec<EntityId> = st.columns.values().map(|x| x.0).collect();
        for w in links.windows(2) {
            out.equal(&[w[0]], &[w[1]]);
        }
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, p: &Params) -> Result<Patch, GenerateFailure> {
        let fatal = |r: String| GenerateFailure::fatal(target, "sweep", r);
        let st = sweep_topo(cx.input, target, p).map_err(fatal)?;
        let vol = target.entity().ok_or_else(|| fatal("group".into()))?;
        let t = cx.input.topo;
        let src = cx.patches.get(Target::Surface(st.source)).ok_or_else(|| fatal("source patch missing".into()))?;
        let (&_, &(e0, _)) = st.columns.iter().next().ok_or_else(|| fatal("no columns".into()))?;
        let layers = cx.intervals(e0).ok_or_else(|| fatal("no layer count".into()))? as usize;
        // Layer fractions from the first linking edge.
        let pts0: Vec<Point3> = cx.edge_nodes(e0).ok_or_else(|| fatal("edge".into()))?.iter().map(|k| cx.point(*k)).collect::<Option<_>>().ok_or_else(|| fatal("edge nodes".into()))?;
        let mut acc = vec![0.0; pts0.len()];
        for q in 1..pts0.len() {
            acc[q] = acc[q - 1] + (pts0[q] - pts0[q - 1]).norm();
        }
        let frac: Vec<f64> = acc.iter().map(|a| a / acc[layers]).collect();
        // Source nodes and their columns.
        let mut src_nodes: Vec<NodeKey> = src.elements.node_refs().to_vec();
        src_nodes.sort_unstable();
        src_nodes.dedup();
        let interior: Vec<NodeKey> = src_nodes.iter().copied().filter(|k| k.owner == st.source).collect();
        let mut patch = Patch::new(target);
        let mut pos: BTreeMap<NodeKey, Point3> = BTreeMap::new();
        let mut cols: BTreeMap<NodeKey, Vec<NodeKey>> = BTreeMap::new();
        for k in &src_nodes {
            let col: Vec<NodeKey> = match t.dim_of(k.owner) {
                Some(sm_topo_api::Dim::Vertex) => {
                    let (e, start) = st.columns.get(&k.owner).ok_or_else(|| fatal(format!("no column at {}", k.owner)))?;
                    let mut c = cx.edge_nodes(*e).ok_or_else(|| fatal("edge".into()))?;
                    if !start {
                        c.reverse();
                    }
                    c
                }
                Some(sm_topo_api::Dim::Edge) => {
                    let l = st.linking.iter().find(|l| t.face_edges(**l).contains(&k.owner)).ok_or_else(|| fatal(format!("edge {} has no linking face", k.owner)))?;
                    let g = cx.patches.get(Target::Surface(*l)).and_then(|p| p.grid.as_ref()).ok_or_else(|| fatal(format!("linking face {l} has no grid")))?;
                    g.line_from(*k).ok_or_else(|| fatal(format!("node {k:?} not on the grid boundary of {l}")))?
                }
                _ => {
                    let slot = interior.binary_search(k).map_err(|_| fatal("interior node".into()))?;
                    let top = NodeKey::new(st.target, k.index); // copy contract: same index
                    let (a, b) = (cx.point(*k).ok_or_else(|| fatal("node".into()))?, cx.point(top).ok_or_else(|| fatal(format!("target node {top:?} missing")))?);
                    let mut c = vec![*k];
                    for (l, fr) in frac.iter().enumerate().take(layers).skip(1) {
                        let key = NodeKey::new(vol, (slot * (layers - 1) + l - 1) as u32);
                        let p = a + (b - a) * *fr;
                        patch.nodes.push(PatchNode { key, point: p, param: Param::None });
                        pos.insert(key, p);
                        c.push(key);
                    }
                    c.push(top);
                    c
                }
            };
            if col.len() != layers + 1 {
                return Err(fatal(format!("column at {k:?} has {} nodes, expected {}", col.len(), layers + 1)));
            }
            cols.insert(*k, col);
        }
        let point = |k: NodeKey| pos.get(&k).copied().or_else(|| cx.point(k)).ok_or_else(|| fatal(format!("node {k:?}")));
        let mut buf: Vec<NodeKey> = Vec::with_capacity(8);
        for (shape, nodes) in src.elements.iter() {
            let cell = match shape {
                ElemShape::Quad => CellShape::Hex,
                ElemShape::Tri => CellShape::Wedge,
                other => return Err(GenerateFailure::fallback(target, "sweep", format!("cannot sweep {other:?}"))),
            };
            let c: Vec<&Vec<NodeKey>> = nodes.iter().map(|k| &cols[k]).collect();
            // Orientation: the bottom polygon must turn towards the next layer.
            let (p0, p1, pl) = (point(c[0][0])?, point(c[1][0])?, point(c[nodes.len() - 1][0])?);
            let up = point(c[0][1])? - p0;
            let forward = (p1 - p0).cross(pl - p0).dot(up) > 0.0;
            for l in 0..layers {
                buf.clear();
                for layer in [l, l + 1] {
                    if forward {
                        buf.extend(c.iter().map(|col| col[layer]));
                    } else {
                        buf.push(c[0][layer]);
                        buf.extend(c[1..].iter().rev().map(|col| col[layer]));
                    }
                }
                patch.elements.push(ElemShape::Cell(cell), &buf);
            }
        }
        Ok(patch)
    }
}

// ---------------------------------------------------------------- fixtures

/// The reference registry: equal, map, sweep.
pub fn registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    r.register(Box::new(Equal)).expect("equal");
    r.register(Box::new(Map)).expect("map");
    r.register(Box::new(Sweep)).expect("sweep");
    r
}

/// A brick `[0,size]` in a fresh complex, plus an empty model.
pub fn brick_setup(size: [f64; 3]) -> (CellComplex, BrickIds, Model) {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| brick(t, Point3::new(0.0, 0.0, 0.0), size)).expect("brick");
    (c, ids, Model::new())
}
