//! Stub scheme implementors used by the integration tests.
//!
//! Curve: `equal`. Surface: `map` (single grid), `submap` (direction sums from `SideDir`;
//! `BLOCK_STRUCTURED`, attaches no grid), `pave` (even loop sum, quad fan), `tri` (triangle
//! fan). Volume: `sweep` (walks linking-face strips with `GenCtx::strip_from`; columns may pass
//! through interior vertices, so it also meshes a composite piece), `msweep` (a group of stacked
//! volumes swept as one; it claims the interface faces and the target), plus plan-level
//! `mapvol`, `tetvol`, `polyvol`. Transforms: `noop`, `boom`. Fixtures: bricks and a row of
//! merged bricks. Shared by several test binaries.
#![allow(dead_code, clippy::needless_range_loop)]

use core::f64::consts::FRAC_PI_2;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3, Transform, Vector3};
use sm_mesh_api::{CellShape, Param};
use sm_model_api::Model;
use sm_scheme_api::geom;
use sm_scheme_api::{
    Applicability, Caps, CopyConstraint, CopyReason, ElemShape, FailureKind, Firmness, GenCtx,
    GenerateFailure, IntervalCtx, IntervalSink, MeshInput, MeshTransform, NodeGrid, NodeKey,
    Params, Patch, PatchBuilder, PlanCtx, Requirements, Role, Scheme, SchemeDim, SchemeRegistry,
    SideDir, Target, VertexRule,
};
use sm_topo_api::fixtures::{BrickIds, brick, merge_touching_bricks, with_op};
use sm_topo_api::{CellComplex, Coincidence, OpId, TopologyEdit, TopologyView, TopologyViewExt};
use std::collections::{BTreeMap, BTreeSet};

type GResult<T> = Result<T, GenerateFailure>;

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
    fn caps(&self, _p: &Params) -> Caps {
        Caps::NONE
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, _p: &Params) -> Applicability {
        let Target::Curve(e) = target else {
            return Applicability::no("not a curve");
        };
        match geom::edge_curve(cx.input.topo, e) {
            Ok(_) => Applicability::yes(50, "uniform in curve parameter"),
            Err(err) => Applicability::no(err.to_string()),
        }
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        let Target::Curve(e) = target else {
            return Err(GenerateFailure::fatal(target, "equal", "not a curve"));
        };
        let n = cx
            .intervals(e)
            .ok_or_else(|| GenerateFailure::fatal(target, "equal", "no intervals"))?;
        let fr: Vec<f64> = (1..n).map(|i| f64::from(i) / f64::from(n)).collect();
        cx.curve_patch(e, &fr)
    }
}

// ---------------------------------------------------------------- map / submap (surface)

/// Builds the transfinite patch of a face with four logical sides.
fn structured_patch(cx: &GenCtx<'_>, target: Target, name: &str) -> GResult<Patch> {
    let fatal = |r: String| GenerateFailure::fatal(target, name, r);
    let Target::Surface(f) = target else {
        return Err(fatal("not a surface".into()));
    };
    let layout = cx.plan.layout(f).ok_or_else(|| fatal("no layout".into()))?;
    if layout.sides.len() != 4 {
        return Err(GenerateFailure::new(
            target,
            name,
            FailureKind::Unsupported,
            format!("{} logical sides (stub meshes four)", layout.sides.len()),
        ));
    }
    let side = |k: usize| {
        cx.side_nodes(f, k)
            .ok_or_else(|| fatal(format!("side {k} unavailable")))
    };
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
    let pt = |k: NodeKey| {
        cx.point(k)
            .ok_or_else(|| fatal(format!("node {k:?} missing")))
    };
    // Normalized arc-length fractions along a boundary line.
    let frac = |line: &[Point3]| -> Vec<f64> {
        let mut acc = vec![0.0; line.len()];
        for q in 1..line.len() {
            acc[q] = acc[q - 1] + (line[q] - line[q - 1]).norm();
        }
        let total = acc[line.len() - 1].max(1e-300);
        acc.iter().map(|a| a / total).collect()
    };
    let row =
        |j: usize| -> GResult<Vec<Point3>> { (0..ni).map(|i| pt(keys[j * ni + i])).collect() };
    let col =
        |i: usize| -> GResult<Vec<Point3>> { (0..nj).map(|j| pt(keys[j * ni + i])).collect() };
    let (bot, top, lef, rig) = (row(0)?, row(nj - 1)?, col(0)?, col(ni - 1)?);
    let (ub, ut, vl, vr) = (frac(&bot), frac(&top), frac(&lef), frac(&rig));
    let mut pb = PatchBuilder::new(target);
    let mut idx = vec![0u32; ni * nj];
    for j in 0..nj {
        for i in 0..ni {
            let boundary = i == 0 || j == 0 || i == ni - 1 || j == nj - 1;
            let at = j * ni + i;
            if boundary {
                idx[at] = pb.refer(keys[at]);
                continue;
            }
            let lerp = |a: Point3, b: Point3, t: f64| a + (b - a) * t;
            let (u, v) = (0.5 * (ub[i] + ut[i]), 0.5 * (vl[j] + vr[j]));
            let edges = lerp(bot[i], top[i], v) - Point3::default();
            let sides = lerp(lef[j], rig[j], u) - Point3::default();
            let corners = lerp(
                lerp(bot[0], bot[ni - 1], u),
                lerp(top[0], top[ni - 1], u),
                v,
            ) - Point3::default();
            let p = Point3::default() + (edges + sides - corners);
            let (p, param) = geom::snap_to_face(cx.input.topo, f, p).map_err(|e| {
                GenerateFailure::new(target, name, FailureKind::Geometry, e.to_string())
            })?;
            let key = NodeKey::new(f, ((j - 1) * (ni - 2) + (i - 1)) as u32);
            keys[at] = key;
            idx[at] = pb.node(key, p, param);
        }
    }
    for j in 0..nj - 1 {
        for i in 0..ni - 1 {
            pb.push(
                ElemShape::Quad,
                &[
                    idx[j * ni + i],
                    idx[j * ni + i + 1],
                    idx[(j + 1) * ni + i + 1],
                    idx[(j + 1) * ni + i],
                ],
            );
        }
    }
    pb.grid(NodeGrid::new(ni as u32, nj as u32, keys));
    Ok(pb.finish())
}

/// Mapping: four logical sides, opposite sides equal, transfinite (Coons) interior.
pub struct Map;

impl Scheme for Map {
    fn name(&self) -> &str {
        "map"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::QUAD | Caps::STRUCTURED | Caps::BLOCK_STRUCTURED
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, _p: &Params) -> Applicability {
        let Target::Surface(f) = target else {
            return Applicability::no("not a surface");
        };
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
        let ends = corners
            .iter()
            .filter(|c| (c.angle - FRAC_PI_2).abs() < fuzzy)
            .count();
        let sides = corners
            .iter()
            .filter(|c| (c.angle - 2.0 * FRAC_PI_2).abs() < fuzzy)
            .count();
        if ends == 4 && ends + sides == corners.len() {
            Applicability::yes(100, "four End corners")
        } else {
            Applicability::yes(30, "corners need picking")
        }
    }
    fn requirements(
        &self,
        _cx: &PlanCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.vertex_rule(f, VertexRule::Map);
        }
        Ok(())
    }
    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else {
            return Ok(());
        };
        let layout = cx.layout(f).ok_or_else(|| {
            sm_scheme_api::SchemeError::Input(format!("map face {f} has no layout"))
        })?;
        if layout.sides.len() != 4 {
            return Err(sm_scheme_api::SchemeError::Input(format!(
                "map face {f}: {} sides",
                layout.sides.len()
            )));
        }
        let edges = |k: usize| -> Vec<EntityId> {
            layout.sides[k].coedges.iter().map(|c| c.edge).collect()
        };
        out.equal(&edges(0), &edges(2));
        out.equal(&edges(1), &edges(3));
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        structured_patch(cx, target, "map")
    }
}

/// Submapping: corner sum 4 and interval equations from the side directions
/// (`sum(+i) = sum(-i)`, `sum(+j) = sum(-j)`); meshes layouts with four logical sides.
pub struct Submap;

impl Scheme for Submap {
    fn name(&self) -> &str {
        "submap"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::QUAD | Caps::BLOCK_STRUCTURED
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, _p: &Params) -> Applicability {
        let Target::Surface(f) = target else {
            return Applicability::no("not a surface");
        };
        match geom::corners(cx.input.topo, f) {
            Ok(c) if c.iter().all(|c| c.ring == 0) && c.len() >= 4 => {
                Applicability::yes(90, "single loop")
            }
            Ok(_) => Applicability::no("submap needs a single loop of >= 4 corners"),
            Err(e) => Applicability::no(e.to_string()),
        }
    }
    fn requirements(
        &self,
        _cx: &PlanCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.vertex_rule(f, VertexRule::Submap);
        }
        Ok(())
    }
    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else {
            return Ok(());
        };
        let layout = cx.layout(f).ok_or_else(|| {
            sm_scheme_api::SchemeError::Input(format!("submap face {f} has no layout"))
        })?;
        let edges = |d: SideDir| -> Vec<EntityId> {
            layout
                .sides
                .iter()
                .filter(|s| s.dir == d)
                .flat_map(|s| s.coedges.iter().map(|c| c.edge))
                .collect()
        };
        out.equal(&edges(SideDir::PlusI), &edges(SideDir::MinusI));
        out.equal(&edges(SideDir::PlusJ), &edges(SideDir::MinusJ));
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        // Submap promises blocks, not a single grid: consumers walk strips (`strip_from`).
        let mut p = structured_patch(cx, target, "submap")?;
        p.grid = None;
        Ok(p)
    }
}

// ---------------------------------------------------------------- pave / tri (surface)

fn fan(cx: &GenCtx<'_>, target: Target, name: &str, quads: bool) -> GResult<Patch> {
    let Target::Surface(f) = target else {
        return Err(GenerateFailure::fatal(target, name, "not a surface"));
    };
    let loops = cx
        .loops(f)
        .map_err(|e| GenerateFailure::fatal(target, name, e.to_string()))?;
    let outer = loops
        .first()
        .ok_or_else(|| GenerateFailure::fatal(target, name, "no loop"))?;
    let n = outer.nodes.len();
    if quads && n % 2 != 0 {
        return Err(GenerateFailure::new(
            target,
            name,
            FailureKind::Constraint,
            format!("odd loop of {n} nodes"),
        ));
    }
    let mut pb = PatchBuilder::new(target);
    let refs: Vec<u32> = outer.nodes.iter().map(|k| pb.refer(*k)).collect();
    let mut acc = Vector3::default();
    for k in &outer.nodes {
        let p = cx
            .point(*k)
            .ok_or_else(|| GenerateFailure::fatal(target, name, format!("node {k:?} missing")))?;
        acc = acc + (p - Point3::default());
    }
    let c0 = Point3::default() + acc * (1.0 / n as f64);
    let (c, param) = geom::snap_to_face(cx.input.topo, f, c0)
        .map_err(|e| GenerateFailure::new(target, name, FailureKind::Geometry, e.to_string()))?;
    let ci = pb.node(NodeKey::new(f, 0), c, param);
    if quads {
        for i in 0..n / 2 {
            pb.push(
                ElemShape::Quad,
                &[ci, refs[2 * i], refs[2 * i + 1], refs[(2 * i + 2) % n]],
            );
        }
    } else {
        for i in 0..n {
            pb.push(ElemShape::Tri, &[ci, refs[i], refs[(i + 1) % n]]);
        }
    }
    Ok(pb.finish())
}

/// Paving stand-in: needs an even loop sum, meshes with a quad fan.
pub struct Pave;

impl Scheme for Pave {
    fn name(&self) -> &str {
        "pave"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::QUAD
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, _p: &Params) -> Applicability {
        let Target::Surface(f) = target else {
            return Applicability::no("not a surface");
        };
        if cx.input.topo.loops(f).count() == 1 {
            Applicability::yes(40, "single-loop face")
        } else {
            Applicability::no("stub paves single loops")
        }
    }
    fn requirements(
        &self,
        _cx: &PlanCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.vertex_rule(f, VertexRule::Free);
        }
        Ok(())
    }
    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else {
            return Ok(());
        };
        let edges: Vec<EntityId> = geom::face_loops(cx.input.topo, f)
            .iter()
            .flatten()
            .map(|k| k.edge)
            .collect();
        out.even(&edges);
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        fan(cx, target, "pave", true)
    }
}

/// Triangulation stand-in: a triangle fan, any loop.
pub struct Tri;

impl Scheme for Tri {
    fn name(&self) -> &str {
        "tri"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::TRI
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, _p: &Params) -> Applicability {
        let Target::Surface(f) = target else {
            return Applicability::no("not a surface");
        };
        if cx.input.topo.loops(f).count() == 1 {
            Applicability::yes(20, "single-loop face")
        } else {
            Applicability::no("stub triangulates single loops")
        }
    }
    fn requirements(
        &self,
        _cx: &PlanCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.vertex_rule(f, VertexRule::Free);
        }
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        fan(cx, target, "tri", false)
    }
}

// ---------------------------------------------------------------- stacks (sweep, msweep)

/// The analysis of a stacked sweep, computed in the requirements phase and stored as plan data.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Stack {
    /// Members in stacking order (one for a plain sweep or a composite piece).
    pub members: Vec<EntityId>,
    pub source: EntityId,
    pub target: EntityId,
    /// Faces between consecutive members (empty for a single piece).
    pub interfaces: Vec<EntityId>,
    pub linking: Vec<EntityId>,
    /// Source vertex -> path of `(edge, runs along the path)`.
    pub columns: Vec<(EntityId, Vec<(EntityId, bool)>)>,
    /// Member of every step of a path.
    pub step_member: Vec<usize>,
}

fn face_vertices(t: &dyn TopologyView, f: EntityId) -> Vec<EntityId> {
    let mut v: Vec<EntityId> = t
        .face_edges(f)
        .iter()
        .filter_map(|e| t.edge_ends(*e))
        .flat_map(|(a, b)| [a, b])
        .collect();
    v.sort_unstable();
    v.dedup();
    v
}

/// Discover the columns of a stack of `members` (volumes with their faces) between `source`
/// and `target`.
fn discover(
    t: &dyn TopologyView,
    members: &[(EntityId, Vec<EntityId>)],
    source: EntityId,
    target: EntityId,
) -> Result<Stack, String> {
    let all: BTreeSet<EntityId> = members.iter().flat_map(|m| m.1.iter().copied()).collect();
    let interfaces_set: BTreeSet<EntityId> = all
        .iter()
        .copied()
        .filter(|f| members.iter().filter(|m| m.1.contains(f)).count() >= 2)
        .collect();
    let (vs, vg) = (face_vertices(t, source), face_vertices(t, target));
    if vs.iter().any(|v| vg.contains(v)) {
        return Err("source and target touch".into());
    }
    if vs.len() != vg.len() {
        return Err("many-to-one/many-to-many sweeps are not in the stub".into());
    }
    let mut excluded: BTreeSet<EntityId> = BTreeSet::new();
    for f in std::iter::once(&source)
        .chain(std::iter::once(&target))
        .chain(interfaces_set.iter())
    {
        excluded.extend(t.face_edges(*f).iter().copied());
    }
    let piece_edges: BTreeSet<EntityId> = all
        .iter()
        .flat_map(|f| t.face_edges(*f).iter().copied())
        .collect();
    let pos = |v: EntityId| geom::vertex_point(t, v).map_err(|e| e.to_string());
    let mut columns = Vec::new();
    for &v in &vs {
        let mut path: Vec<(EntityId, bool)> = Vec::new();
        let (mut cur, mut last, mut dir): (EntityId, Option<EntityId>, Option<Vector3>) =
            (v, None, None);
        for _ in 0..64 {
            if vg.contains(&cur) {
                break;
            }
            let cands: Vec<(EntityId, EntityId)> = piece_edges
                .iter()
                .filter(|e| !excluded.contains(e) && Some(**e) != last)
                .filter_map(|e| {
                    let (a, b) = t.edge_ends(*e)?;
                    if a == cur {
                        Some((*e, b))
                    } else if b == cur {
                        Some((*e, a))
                    } else {
                        None
                    }
                })
                .collect();
            let pick = match (cands.as_slice(), dir) {
                ([one], _) => *one,
                (many, Some(d)) if !many.is_empty() => {
                    let p0 = pos(cur)?;
                    let mut best = None;
                    for c in many {
                        let q = pos(c.1)? - p0;
                        let u = q * (1.0 / q.norm().max(1e-300));
                        let dot = u.dot(d);
                        if best.is_none_or(|b: (f64, (EntityId, EntityId))| dot > b.0) {
                            best = Some((dot, *c));
                        }
                    }
                    match best {
                        Some((dot, c)) if dot > 0.9 => c,
                        _ => {
                            return Err(format!(
                                "column at {v}: no straight continuation at {cur}"
                            ));
                        }
                    }
                }
                _ => {
                    return Err(format!(
                        "vertex {cur} needs exactly one linking edge (stub)"
                    ));
                }
            };
            let (e, next) = pick;
            let (a, _) = t.edge_ends(e).ok_or("edge without ends")?;
            let q = pos(next)? - pos(cur)?;
            dir = Some(q * (1.0 / q.norm().max(1e-300)));
            path.push((e, a == cur));
            last = Some(e);
            cur = next;
        }
        if !vg.contains(&cur) {
            return Err(format!("column at {v} does not reach the target"));
        }
        columns.push((v, path));
    }
    let ends: BTreeSet<EntityId> = columns
        .iter()
        .filter_map(|(_, p)| p.last())
        .filter_map(|(e, fwd)| t.edge_ends(*e).map(|(a, b)| if *fwd { b } else { a }))
        .collect();
    if ends.len() != columns.len() {
        return Err("two columns end at one target vertex".into());
    }
    let steps = columns.first().map_or(0, |c| c.1.len());
    if columns.iter().any(|c| c.1.len() != steps) {
        return Err("columns have different lengths".into());
    }
    // Which member owns each step: the one whose non-interface, non-end faces hold the edge.
    let linking_of = |m: &(EntityId, Vec<EntityId>)| -> Vec<EntityId> {
        m.1.iter()
            .copied()
            .filter(|f| *f != source && *f != target && !interfaces_set.contains(f))
            .collect()
    };
    let mut step_member = Vec::new();
    for j in 0..steps {
        let e = columns[0].1[j].0;
        let ms: Vec<usize> = (0..members.len())
            .filter(|&i| {
                linking_of(&members[i])
                    .iter()
                    .any(|f| t.face_edges(*f).contains(&e))
            })
            .collect();
        match ms.as_slice() {
            [m] => step_member.push(*m),
            _ => {
                return Err(format!(
                    "step {j}: edge {e} does not belong to exactly one member"
                ));
            }
        }
    }
    // Stacking order = order of first appearance along the column.
    let mut order: Vec<usize> = Vec::new();
    for m in &step_member {
        if !order.contains(m) {
            order.push(*m);
        }
    }
    if order.len() != members.len() {
        return Err("a member is not on the column".into());
    }
    let step_member: Vec<usize> = step_member
        .iter()
        .map(|m| order.iter().position(|o| o == m).unwrap_or(0))
        .collect();
    let mut interfaces = Vec::new();
    for w in order.windows(2) {
        let shared: Vec<EntityId> = members[w[0]]
            .1
            .iter()
            .copied()
            .filter(|f| members[w[1]].1.contains(f))
            .collect();
        match shared.as_slice() {
            [f] => interfaces.push(*f),
            _ => return Err("consecutive members must share exactly one face".into()),
        }
    }
    let linking: Vec<EntityId> = all
        .iter()
        .copied()
        .filter(|f| *f != source && *f != target && !interfaces_set.contains(f))
        .collect();
    for l in &linking {
        if t.face_edges(*l).len() < 4 {
            return Err(format!("linking face {l} has fewer than four edges (stub)"));
        }
    }
    Ok(Stack {
        members: order.iter().map(|&i| members[i].0).collect(),
        source,
        target,
        interfaces,
        linking,
        columns,
        step_member,
    })
}

fn ids_param(p: &Params, key: &str) -> Result<Option<Vec<EntityId>>, String> {
    match p.get::<Vec<EntityId>>("sweep", key) {
        Ok(v) => Ok(v),
        Err(_) => p
            .get::<EntityId>("sweep", key)
            .map(|o| o.map(|x| vec![x]))
            .map_err(|e| e.to_string()),
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

fn end_vertex(t: &dyn TopologyView, path: &[(EntityId, bool)]) -> Option<EntityId> {
    let (e, fwd) = path.last()?;
    let (a, b) = t.edge_ends(*e)?;
    Some(if *fwd { b } else { a })
}

fn stack_of(cx: &GenCtx<'_>, target: Target, name: &str) -> GResult<Stack> {
    let d = cx
        .data(target)
        .ok_or_else(|| GenerateFailure::fatal(target, name, "no stack data"))?;
    serde_json::from_value(d.0.clone())
        .map_err(|e| GenerateFailure::fatal(target, name, e.to_string()))
}

fn stack_of_ic(cx: &IntervalCtx<'_>, target: Target) -> sm_scheme_api::Result<Stack> {
    let d = cx
        .data(target)
        .ok_or_else(|| sm_scheme_api::SchemeError::Input("no stack data".into()))?;
    serde_json::from_value(d.0.clone())
        .map_err(|e| sm_scheme_api::SchemeError::Input(e.to_string()))
}

/// Interval rows shared by both sweeps: every column has the same layer counts.
fn column_equalities(st: &Stack, out: &mut IntervalSink<'_>) {
    let Some(first) = st.columns.first() else {
        return;
    };
    for (_, path) in st.columns.iter().skip(1) {
        for (a, b) in first.1.iter().zip(path) {
            out.equal(&[a.0], &[b.0]);
        }
    }
}

/// Generate the cells (and, for a group, the claimed interface and target faces) of a stack.
/// `group` selects the group conventions: interface planes belong to the interface faces,
/// the target plane is written here, and cells carry their member as owner.
fn generate_stack(
    cx: &GenCtx<'_>,
    target: Target,
    st: &Stack,
    group: bool,
    name: &str,
    perturb: bool,
    misorient: bool,
) -> GResult<Patch> {
    let fatal = |r: String| GenerateFailure::fatal(target, name, r);
    let t = cx.input.topo;
    let src = cx
        .patches
        .get(Target::Surface(st.source))
        .ok_or_else(|| fatal("source patch missing".into()))?;
    let steps: Vec<usize> = st.columns[0]
        .1
        .iter()
        .map(|(e, _)| cx.intervals(*e).unwrap_or(1) as usize)
        .collect();
    let layers: usize = steps.iter().sum();
    // Plane -> member below it, and interface plane -> face.
    let mut layer_member = Vec::with_capacity(layers);
    let mut plane_owner: Vec<Option<EntityId>> = vec![None; layers + 1];
    let mut at = 0usize;
    for (j, n) in steps.iter().enumerate() {
        for _ in 0..*n {
            layer_member.push(st.step_member[j]);
        }
        at += n;
        if j + 1 < steps.len() && st.step_member[j + 1] != st.step_member[j] {
            plane_owner[at] = st.interfaces.get(st.step_member[j]).copied();
        }
    }
    let shared_vol = |l: usize| st.members[layer_member[l.saturating_sub(1).min(layers - 1)]];
    let owner_of_plane = |p: usize| -> EntityId {
        if group {
            plane_owner[p].unwrap_or_else(|| shared_vol(p))
        } else {
            target.entity().unwrap_or(st.members[0])
        }
    };
    // Fractions along the first column.
    let first = &st.columns[0].1;
    let mut chain: Vec<NodeKey> = Vec::new();
    for (e, fwd) in first {
        let mut c = cx.edge_nodes(*e).ok_or_else(|| fatal("edge".into()))?;
        if !*fwd {
            c.reverse();
        }
        let skip = usize::from(!chain.is_empty());
        chain.extend_from_slice(&c[skip..]);
    }
    let pts0: Vec<Point3> = chain
        .iter()
        .map(|k| cx.point(*k))
        .collect::<Option<_>>()
        .ok_or_else(|| fatal("column nodes".into()))?;
    let mut acc = vec![0.0; pts0.len()];
    for q in 1..pts0.len() {
        acc[q] = acc[q - 1] + (pts0[q] - pts0[q - 1]).norm();
    }
    let frac: Vec<f64> = acc.iter().map(|a| a / acc[layers]).collect();
    let shift = mean(
        t,
        &st.columns
            .iter()
            .filter_map(|(_, p)| end_vertex(t, p))
            .collect::<Vec<_>>(),
    ) - mean(t, &st.columns.iter().map(|c| c.0).collect::<Vec<_>>());

    let mut keys: Vec<NodeKey> = src
        .nodes
        .iter()
        .map(|n| n.key)
        .chain(src.refs.iter().copied())
        .collect();
    keys.sort_unstable();
    keys.dedup();
    let interior: Vec<NodeKey> = keys
        .iter()
        .copied()
        .filter(|k| k.owner == st.source)
        .collect();
    let target_nodes = !group;
    let mut pb = PatchBuilder::new(target);
    let mut own: BTreeMap<NodeKey, u32> = BTreeMap::new();
    let mut pos: BTreeMap<NodeKey, Point3> = BTreeMap::new();
    let mut cols: BTreeMap<NodeKey, Vec<NodeKey>> = BTreeMap::new();
    for k in &keys {
        let col: Vec<NodeKey> = match t.dim_of(k.owner) {
            Some(sm_topo_api::Dim::Vertex) => {
                let (_, path) = st
                    .columns
                    .iter()
                    .find(|c| c.0 == k.owner)
                    .ok_or_else(|| fatal(format!("no column at {}", k.owner)))?;
                let mut c: Vec<NodeKey> = Vec::new();
                for (e, fwd) in path {
                    let mut en = cx.edge_nodes(*e).ok_or_else(|| fatal("edge".into()))?;
                    if !*fwd {
                        en.reverse();
                    }
                    let skip = usize::from(!c.is_empty());
                    c.extend_from_slice(&en[skip..]);
                }
                c
            }
            Some(sm_topo_api::Dim::Edge) => {
                // Cross the linking faces one after another until the target boundary.
                let end_edges: BTreeSet<EntityId> =
                    t.face_edges(st.target).iter().copied().collect();
                let mut col = vec![*k];
                let mut used: Vec<EntityId> = Vec::new();
                let mut cur = *k;
                for _ in 0..16 {
                    let l = st
                        .linking
                        .iter()
                        .copied()
                        .find(|l| !used.contains(l) && t.face_edges(*l).contains(&cur.owner))
                        .ok_or_else(|| fatal(format!("edge {} has no linking face", cur.owner)))?;
                    used.push(l);
                    let line = cx.strip_from(l, cur).ok_or_else(|| {
                        fatal(format!("node {cur:?} starts no quad strip of linking face {l}"))
                    })?;
                    col.extend_from_slice(&line[1..]);
                    cur = *line.last().ok_or_else(|| fatal("empty line".into()))?;
                    if end_edges.contains(&cur.owner) {
                        break;
                    }
                }
                col
            }
            _ => {
                let slot = interior
                    .binary_search(k)
                    .map_err(|_| fatal("interior node".into()))?;
                let a = cx.point(*k).ok_or_else(|| fatal("node".into()))?;
                let mut c = vec![*k];
                for p in 1..layers {
                    let key = NodeKey::new(owner_of_plane(p), (slot * (layers - 1) + p - 1) as u32);
                    let q = a + shift * frac[p];
                    own.insert(key, pb.node(key, q, Param::None));
                    pos.insert(key, q);
                    c.push(key);
                }
                let top = NodeKey::new(st.target, k.index);
                if group {
                    let mut q = a + shift;
                    if perturb && slot == 0 {
                        q = q + Vector3::new(0.013, 0.0, 0.0);
                    }
                    own.insert(top, pb.node(top, q, Param::None));
                    pos.insert(top, q);
                }
                c.push(top);
                c
            }
        };
        if col.len() != layers + 1 {
            return Err(fatal(format!(
                "column at {k:?} has {} nodes, expected {}",
                col.len(),
                layers + 1
            )));
        }
        cols.insert(*k, col);
    }
    let _ = target_nodes;
    let idx = |pb: &mut PatchBuilder, k: NodeKey| -> u32 {
        own.get(&k).copied().unwrap_or_else(|| pb.refer(k))
    };
    let point = |k: NodeKey| {
        pos.get(&k)
            .copied()
            .or_else(|| cx.point(k))
            .ok_or_else(|| fatal(format!("node {k:?}")))
    };
    // Claimed planes follow their face's normal (the patch contract): flip the source
    // orientation where the plane face's normal opposes the source normal.
    let mut plane_flip = vec![false; layers + 1];
    if group {
        let p0 = t
            .face_outline(st.source)
            .first()
            .copied()
            .ok_or_else(|| fatal("source outline".into()))?;
        let ns = geom::face_normal(t, st.source, p0).map_err(|e| fatal(e.to_string()))?;
        for (p, flip) in plane_flip.iter_mut().enumerate().skip(1) {
            let face = if p == layers { Some(st.target) } else { plane_owner[p] };
            if let Some(f) = face {
                let q = p0 + shift * frac[p];
                let nf = geom::face_normal(t, f, q).map_err(|e| fatal(e.to_string()))?;
                *flip = ns.dot(nf) < 0.0;
            }
        }
    }
    let mut buf: Vec<u32> = Vec::with_capacity(8);
    for (shape, nodes) in src.elements.iter() {
        let sk: Vec<NodeKey> = nodes
            .iter()
            .map(|l| src.key(*l))
            .collect::<Option<_>>()
            .ok_or_else(|| fatal("source element".into()))?;
        let cell = match shape {
            ElemShape::Quad => CellShape::Hex,
            ElemShape::Tri => CellShape::Wedge,
            other => {
                return Err(GenerateFailure::new(
                    target,
                    name,
                    FailureKind::Unsupported,
                    format!("cannot sweep {other:?}"),
                ));
            }
        };
        let c: Vec<&Vec<NodeKey>> = sk.iter().map(|k| &cols[k]).collect();
        // Orientation: the bottom polygon must turn towards the next layer.
        let (p0, p1, pl) = (point(c[0][0])?, point(c[1][0])?, point(c[sk.len() - 1][0])?);
        let up = point(c[0][1])? - p0;
        let forward = (p1 - p0).cross(pl - p0).dot(up) > 0.0;
        for l in 0..layers {
            buf.clear();
            for layer in [l, l + 1] {
                let order: Vec<usize> = if forward {
                    (0..sk.len()).collect()
                } else {
                    std::iter::once(0).chain((1..sk.len()).rev()).collect()
                };
                for i in order {
                    buf.push(idx(&mut pb, c[i][layer]));
                }
            }
            if group {
                pb.owner(st.members[layer_member[l]]);
            }
            pb.push(ElemShape::Cell(cell), &buf);
        }
        if group {
            // Surface elements of the claimed planes, along each plane face's normal.
            for p in (1..=layers).filter(|p| *p == layers || plane_owner[*p].is_some()) {
                buf.clear();
                for col in &c {
                    buf.push(idx(&mut pb, col[p]));
                }
                // `misorient` (test hook) writes the target against its normal.
                if plane_flip[p] != (misorient && p == layers) {
                    buf[1..].reverse();
                }
                pb.owner(if p == layers {
                    st.target
                } else {
                    plane_owner[p].unwrap_or(st.target)
                });
                pb.push(shape, &buf);
            }
        }
    }
    Ok(pb.finish())
}

/// Sweep of one piece (a brick, or a composite of merged bricks): source meshed by any
/// scheme, target by a copy, linking faces structured.
pub struct Sweep;

fn single_stack(input: &MeshInput<'_>, target: Target, p: &Params) -> Result<Stack, String> {
    let t = input.topo;
    let faces = input.piece_faces(target).map_err(|e| e.to_string())?;
    let v = target.entity().ok_or("not a single piece")?;
    let one = [(v, faces.clone())];
    match (ids_param(p, "source")?, ids_param(p, "target")?) {
        (Some(s), Some(g)) => match (s.as_slice(), g.as_slice()) {
            ([s], [g]) => discover(t, &one, *s, *g),
            _ => Err("only 1-to-1 sweeps are in the stub".into()),
        },
        (None, None) => {
            let mut last = String::from("no source/target pair");
            for (i, &s) in faces.iter().enumerate() {
                for &g in &faces[i + 1..] {
                    match discover(t, &one, s, g) {
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

impl Scheme for Sweep {
    fn name(&self) -> &str {
        "sweep"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::HEX | Caps::WEDGE
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, p: &Params) -> Applicability {
        if matches!(target, Target::Group(_)) {
            return Applicability::no("use msweep for groups");
        }
        match single_stack(cx.input, target, p) {
            Ok(_) => Applicability::yes(60, "1-to-1 sweepable"),
            Err(e) => Applicability::no(e),
        }
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        let st = single_stack(cx.input, target, p).map_err(|e| {
            sm_scheme_api::SchemeError::NotApplicable {
                target,
                scheme: sm_scheme_api::SchemeName::new("sweep"),
                reason: e,
            }
        })?;
        out.role(
            Target::Surface(st.source),
            Role::SweepSource { volume: target },
        );
        out.role(
            Target::Surface(st.target),
            Role::SweepTarget { volume: target },
        );
        for l in &st.linking {
            out.role(Target::Surface(*l), Role::SweepLinking { volume: target });
            out.require_caps(
                Target::Surface(*l),
                Caps::QUAD | Caps::BLOCK_STRUCTURED,
                Firmness::Hard,
            );
        }
        let t = cx.input.topo;
        let vm: Vec<(EntityId, EntityId)> = st
            .columns
            .iter()
            .filter_map(|(v, path)| Some((*v, end_vertex(t, path)?)))
            .collect();
        let shift = mean(t, &vm.iter().map(|x| x.1).collect::<Vec<_>>())
            - mean(t, &vm.iter().map(|x| x.0).collect::<Vec<_>>());
        out.copy(CopyConstraint {
            source: Target::Surface(st.source),
            target: Target::Surface(st.target),
            transform: Transform::translation(shift),
            vertex_map: vm,
            reason: CopyReason::SweepTarget { volume: target },
        });
        out.set_data(Params(
            serde_json::to_value(&st)
                .map_err(|e| sm_scheme_api::SchemeError::Input(e.to_string()))?,
        ));
        Ok(())
    }
    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        column_equalities(&stack_of_ic(cx, target)?, out);
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        let st = stack_of(cx, target, "sweep")?;
        generate_stack(cx, target, &st, false, "sweep", false, false)
    }
}

/// Multisweep of a group of stacked volumes (a many-to-one style path sum): one source, one
/// target, volumes in between. The group writes the interface faces and the target face
/// itself (claims). `layers` (optional) is the total layer count along a column;
/// `perturb`/`fail` are test hooks.
pub struct MSweep;

fn group_stack(
    input: &MeshInput<'_>,
    cx: &PlanCtx<'_>,
    target: Target,
    p: &Params,
) -> Result<Stack, String> {
    let Target::Group(g) = target else {
        return Err("not a group".into());
    };
    let grp = cx
        .intent
        .groups
        .iter()
        .find(|x| x.id == g)
        .ok_or("unknown group")?;
    let mut members = Vec::new();
    for m in &grp.members {
        members.push((
            *m,
            input
                .piece_faces(Target::Volume(*m))
                .map_err(|e| e.to_string())?,
        ));
    }
    let s = ids_param(p, "source")?
        .and_then(|v| v.first().copied())
        .ok_or("msweep needs `source`")?;
    let g2 = ids_param(p, "target")?
        .and_then(|v| v.first().copied())
        .ok_or("msweep needs `target`")?;
    discover(input.topo, &members, s, g2)
}

impl Scheme for MSweep {
    fn name(&self) -> &str {
        "msweep"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::HEX | Caps::WEDGE | Caps::MESHES_CHILDREN | Caps::QUAD | Caps::STRUCTURED | Caps::BLOCK_STRUCTURED
    }
    fn supports_groups(&self) -> bool {
        true
    }
    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, p: &Params) -> Applicability {
        match group_stack(cx.input, cx, target, p) {
            Ok(_) => Applicability::yes(70, "stacked multisweep"),
            Err(e) => Applicability::no(e),
        }
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        let st = group_stack(cx.input, cx, target, p).map_err(|e| {
            sm_scheme_api::SchemeError::NotApplicable {
                target,
                scheme: sm_scheme_api::SchemeName::new("msweep"),
                reason: e,
            }
        })?;
        out.role(
            Target::Surface(st.source),
            Role::SweepSource { volume: target },
        );
        out.role(
            Target::Surface(st.target),
            Role::SweepTarget { volume: target },
        );
        out.claim(Target::Surface(st.target));
        for i in &st.interfaces {
            out.claim(Target::Surface(*i));
        }
        for l in &st.linking {
            out.role(Target::Surface(*l), Role::SweepLinking { volume: target });
            out.require_caps(
                Target::Surface(*l),
                Caps::QUAD | Caps::BLOCK_STRUCTURED,
                Firmness::Hard,
            );
        }
        out.set_data(Params(
            serde_json::to_value(&st)
                .map_err(|e| sm_scheme_api::SchemeError::Input(e.to_string()))?,
        ));
        Ok(())
    }
    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        p: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let st = stack_of_ic(cx, target)?;
        column_equalities(&st, out);
        if let Some(total) = p.get::<i64>("msweep", "layers")?
            && let Some((_, path)) = st.columns.first()
        {
            let terms: Vec<(EntityId, i32)> = path.iter().map(|(e, _)| (*e, 1)).collect();
            out.linear(&terms, sm_scheme_api::Rel::Eq, total);
        }
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, p: &Params) -> GResult<Patch> {
        match p.get::<String>("msweep", "fail").ok().flatten().as_deref() {
            Some("quality") => {
                return Err(GenerateFailure::fallback(
                    target,
                    "msweep",
                    "refused (test hook)",
                ));
            }
            Some("boom") => {
                return Err(GenerateFailure::fatal(target, "msweep", "boom (test hook)"));
            }
            _ => {}
        }
        let st = stack_of(cx, target, "msweep")?;
        let flag = |k: &str| p.get::<bool>("msweep", k).ok().flatten().unwrap_or(false);
        generate_stack(cx, target, &st, true, "msweep", flag("perturb"), flag("misorient"))
    }
}

// ---------------------------------------------------------------- plan-level volume stubs

/// Requires structured quad faces everywhere; generation is not implemented.
pub struct MapVol;

impl Scheme for MapVol {
    fn name(&self) -> &str {
        "mapvol"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::HEX | Caps::STRUCTURED
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(30, "plan-level stub")
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        for f in cx.input.piece_faces(target)? {
            out.require_caps(
                Target::Surface(f),
                Caps::QUAD | Caps::STRUCTURED,
                Firmness::Hard,
            );
        }
        Ok(())
    }
    fn generate(&self, _cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        Err(GenerateFailure::new(
            target,
            "mapvol",
            FailureKind::Unsupported,
            "plan-level stub",
        ))
    }
}

/// No requirements, last-resort score; generation is not implemented.
pub struct TetVol;

impl Scheme for TetVol {
    fn name(&self) -> &str {
        "tetvol"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::TET | Caps::TRI
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(10, "last resort")
    }
    fn generate(&self, _cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        Err(GenerateFailure::new(
            target,
            "tetvol",
            FailureKind::Unsupported,
            "plan-level stub",
        ))
    }
}

/// One polyhedral cell from the face loops of a brick whose faces each have one quad.
pub struct PolyVol;

impl Scheme for PolyVol {
    fn name(&self) -> &str {
        "polyvol"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Volume
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::POLY
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(5, "polyhedral stub")
    }
    fn requirements(
        &self,
        cx: &PlanCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        for f in cx.input.piece_faces(target)? {
            out.require_caps(
                Target::Surface(f),
                Caps::QUAD | Caps::STRUCTURED,
                Firmness::Hard,
            );
        }
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        let fatal = |r: String| GenerateFailure::fatal(target, "polyvol", r);
        let faces = cx
            .input
            .piece_faces(target)
            .map_err(|e| fatal(e.to_string()))?;
        let mut pb = PatchBuilder::new(target);
        let mut locals: Vec<Vec<u32>> = Vec::new();
        for f in faces {
            let loops = cx.loops(f).map_err(|e| fatal(e.to_string()))?;
            let outer = loops.first().ok_or_else(|| fatal("no loop".into()))?;
            locals.push(outer.nodes.iter().map(|k| pb.refer(*k)).collect());
        }
        let refs: Vec<&[u32]> = locals.iter().map(Vec::as_slice).collect();
        pb.push_polyhedron(&refs);
        Ok(pb.finish())
    }
}

/// A surface scheme that always asks for a fallback, with a high auto score.
pub struct AlwaysFail;

impl Scheme for AlwaysFail {
    fn name(&self) -> &str {
        "always_fail"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::QUAD
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(200, "eager")
    }
    fn generate(&self, _cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        Err(GenerateFailure::fallback(target, "always_fail", "refuses"))
    }
}

/// A volume scheme whose requirements are about something that is not its sub-entity.
pub struct BadReq;

impl Scheme for BadReq {
    fn name(&self) -> &str {
        "badreq"
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
        _target: Target,
        p: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        // A sibling volume, or (param `face`) a face of another model: neither is a sub-entity.
        let other = p
            .get::<EntityId>("badreq", "other")?
            .unwrap_or(cx.input.selection[0]);
        out.impose(Target::Volume(other), &["map"]);
        Ok(())
    }
    fn generate(&self, _cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        Err(GenerateFailure::new(
            target,
            "badreq",
            FailureKind::Unsupported,
            "plan-level stub",
        ))
    }
}

// ---------------------------------------------------------------- transforms

/// Leaves the patches as they are.
pub struct Noop;

impl MeshTransform for Noop {
    fn name(&self) -> &str {
        "noop"
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::NONE
    }
    fn apply(
        &self,
        _cx: &GenCtx<'_>,
        _targets: &[Target],
        _params: &Params,
        _patches: &mut [Patch],
    ) -> GResult<()> {
        Ok(())
    }
}

/// Moves every node owned by the patch's target up by `dz` (param), keeping topology.
pub struct Lift;

impl MeshTransform for Lift {
    fn name(&self) -> &str {
        "lift"
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::NONE
    }
    fn apply(
        &self,
        _cx: &GenCtx<'_>,
        targets: &[Target],
        params: &Params,
        patches: &mut [Patch],
    ) -> GResult<()> {
        let dz = params
            .get::<f64>("lift", "dz")
            .ok()
            .flatten()
            .unwrap_or(0.0);
        for (t, p) in targets.iter().zip(patches.iter_mut()) {
            let owned: Vec<EntityId> = t.entity().into_iter().collect();
            for n in &mut p.nodes {
                if owned.contains(&n.key.owner) {
                    n.point = n.point + Vector3::new(0.0, 0.0, dz);
                }
            }
        }
        Ok(())
    }
}

/// Always fails with an internal error.
pub struct Boom;

impl MeshTransform for Boom {
    fn name(&self) -> &str {
        "boom"
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::NONE
    }
    fn apply(
        &self,
        _cx: &GenCtx<'_>,
        targets: &[Target],
        _params: &Params,
        _patches: &mut [Patch],
    ) -> GResult<()> {
        Err(GenerateFailure::fatal(
            targets[0],
            "boom",
            "transform failed",
        ))
    }
}

// ---------------------------------------------------------------- fixtures

/// The reference registry: every stub.
pub fn registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    r.register(Box::new(Equal)).expect("equal");
    r.register(Box::new(Map)).expect("map");
    r.register(Box::new(Submap)).expect("submap");
    r.register(Box::new(Pave)).expect("pave");
    r.register(Box::new(Tri)).expect("tri");
    r.register(Box::new(Sweep)).expect("sweep");
    r.register(Box::new(MSweep)).expect("msweep");
    r.register(Box::new(MapVol)).expect("mapvol");
    r.register(Box::new(TetVol)).expect("tetvol");
    r.register(Box::new(PolyVol)).expect("polyvol");
    r.register(Box::new(BadReq)).expect("badreq");
    r.register_transform(Box::new(Noop)).expect("noop");
    r.register_transform(Box::new(Lift)).expect("lift");
    r.register_transform(Box::new(Boom)).expect("boom");
    r
}

/// A brick `[0,size]` in a fresh complex, plus an empty model.
pub fn brick_setup(size: [f64; 3]) -> (CellComplex, BrickIds, Model) {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| {
        brick(t, Point3::new(0.0, 0.0, 0.0), size)
    })
    .expect("brick");
    (c, ids, Model::new())
}

/// `n` unit bricks in a row along x, merged face to face.
pub fn row_setup(n: usize) -> (CellComplex, Vec<BrickIds>, Model) {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| {
        let mut v: Vec<BrickIds> = Vec::new();
        for i in 0..n {
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
        Ok(v)
    })
    .expect("row");
    (c, ids, Model::new())
}
