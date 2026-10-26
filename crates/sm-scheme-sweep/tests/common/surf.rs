//! Test-local surface schemes that stand in for pave/tri/polygon until integration:
//! a lattice quad mesher (any axis-aligned polygon on a uniform grid), a triangle fan, a ring of
//! quads and one polygon element. Node numbering is deterministic (raster / loop order).
#![allow(dead_code)]

use sm_core::Point3;
use sm_mesh_api::Param;
use sm_scheme_api::{
    Applicability, Caps, ElemShape, FailureKind, GenCtx, GenerateFailure, NodeKey, Params, Patch,
    PatchBuilder, PlanCtx, Requirements, Scheme, SchemeDim, Target, VertexRule, geom,
};
use sm_topo_api::{TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

type GResult<T> = Result<T, GenerateFailure>;

fn face_of(target: Target, name: &str) -> GResult<sm_core::EntityId> {
    match target {
        Target::Surface(f) => Ok(f),
        _ => Err(GenerateFailure::fatal(target, name, "not a surface")),
    }
}

/// Orient quad/tri index lists so they follow the face normal (right-hand rule): `flip`
/// reverses all but the first node.
fn normal_z(cx: &GenCtx<'_>, f: sm_core::EntityId) -> f64 {
    let t: &dyn TopologyView = cx.input.topo;
    let p = t.face_outline(f).first().copied().unwrap_or_default();
    geom::face_normal(t, f, p).map_or(1.0, |n| n.z)
}

/// Quad mesh of an axis-aligned planar polygon (in a plane `z = const`) on the lattice of
/// spacing `h` (parameter). Boundary nodes must lie on the lattice.
pub struct GridFace;

impl Scheme for GridFace {
    fn name(&self) -> &str {
        "gridface"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::QUAD | Caps::BLOCK_STRUCTURED
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(95, "lattice quads")
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
    fn generate(&self, cx: &GenCtx<'_>, target: Target, p: &Params) -> GResult<Patch> {
        let fatal = |r: String| GenerateFailure::fatal(target, "gridface", r);
        let f = face_of(target, "gridface")?;
        let h: f64 = p
            .get("gridface", "h")
            .map_err(|e| fatal(e.to_string()))?
            .ok_or_else(|| fatal("needs h".into()))?;
        let t: &dyn TopologyView = cx.input.topo;
        let outline = t.face_outline(f);
        let z = outline.first().map_or(0.0, |p| p.z);
        let lattice = |p: Point3| -> (i64, i64) { ((p.x / h).round() as i64, (p.y / h).round() as i64) };
        let loops = cx.loops(f).map_err(|e| fatal(e.to_string()))?;
        let mut boundary: BTreeMap<(i64, i64), NodeKey> = BTreeMap::new();
        for lp in &loops {
            for k in &lp.nodes {
                let pt = cx
                    .point(*k)
                    .ok_or_else(|| fatal(format!("boundary node {k:?} missing")))?;
                boundary.insert(lattice(pt), *k);
            }
        }
        // Cell (i, j) spans lattice points (i..i+1, j..j+1); it is inside if its centre is.
        let poly: Vec<(f64, f64)> = outline.iter().map(|p| (p.x, p.y)).collect();
        let inside = |x: f64, y: f64| {
            let mut c = false;
            let n = poly.len();
            for i in 0..n {
                let ((x1, y1), (x2, y2)) = (poly[i], poly[(i + 1) % n]);
                if (y1 > y) != (y2 > y) && x < x1 + (y - y1) / (y2 - y1) * (x2 - x1) {
                    c = !c;
                }
            }
            c
        };
        let (mut lo, mut hi) = ((i64::MAX, i64::MAX), (i64::MIN, i64::MIN));
        for p in &outline {
            let (i, j) = lattice(*p);
            lo = (lo.0.min(i), lo.1.min(j));
            hi = (hi.0.max(i), hi.1.max(j));
        }
        let mut cells = Vec::new();
        for j in lo.1..hi.1 {
            for i in lo.0..hi.0 {
                if inside((i as f64 + 0.5) * h, (j as f64 + 0.5) * h) {
                    cells.push((i, j));
                }
            }
        }
        let mut interior: BTreeMap<(i64, i64), u32> = BTreeMap::new();
        for (i, j) in &cells {
            for q in [(*i, *j), (i + 1, *j), (i + 1, j + 1), (*i, j + 1)] {
                if !boundary.contains_key(&q) {
                    interior.entry(q).or_insert(0);
                }
            }
        }
        // Raster order (j, i).
        let mut order: Vec<(i64, i64)> = interior.keys().copied().collect();
        order.sort_by_key(|(i, j)| (*j, *i));
        let mut pb = PatchBuilder::new(target);
        let mut idx: BTreeMap<(i64, i64), u32> = BTreeMap::new();
        for (n, q) in order.iter().enumerate() {
            let pt = Point3::new(q.0 as f64 * h, q.1 as f64 * h, z);
            let (pt, param) = geom::snap_to_face(t, f, pt)
                .map_err(|e| GenerateFailure::new(target, "gridface", FailureKind::Geometry, e.to_string()))?;
            idx.insert(*q, pb.node(NodeKey::new(f, n as u32), pt, param));
        }
        for (q, k) in &boundary {
            if cells.iter().any(|(i, j)| {
                [(*i, *j), (i + 1, *j), (i + 1, j + 1), (*i, j + 1)].contains(q)
            }) {
                idx.insert(*q, pb.refer(*k));
            }
        }
        let flip = normal_z(cx, f) < 0.0;
        for (i, j) in &cells {
            let mut quad = [
                idx[&(*i, *j)],
                idx[&(i + 1, *j)],
                idx[&(i + 1, j + 1)],
                idx[&(*i, j + 1)],
            ];
            if flip {
                quad[1..].reverse();
            }
            pb.push(ElemShape::Quad, &quad);
        }
        Ok(pb.finish())
    }
}

/// Triangle fan from a centre node `(face, 0)` at the centroid of the single loop's nodes.
pub struct Fan;

impl Scheme for Fan {
    fn name(&self) -> &str {
        "fan"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::TRI
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(95, "triangle fan")
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
        let fatal = |r: String| GenerateFailure::fatal(target, "fan", r);
        let f = face_of(target, "fan")?;
        let loops = cx.loops(f).map_err(|e| fatal(e.to_string()))?;
        let [lp] = loops.as_slice() else {
            return Err(fatal("fan needs a single loop".into()));
        };
        let pts: Vec<Point3> = lp
            .nodes
            .iter()
            .map(|k| cx.point(*k).ok_or_else(|| fatal(format!("node {k:?}"))))
            .collect::<GResult<_>>()?;
        let n = pts.len() as f64;
        let c = pts.iter().fold((0.0, 0.0, 0.0), |a, p| (a.0 + p.x, a.1 + p.y, a.2 + p.z));
        let c = Point3::new(c.0 / n, c.1 / n, c.2 / n);
        let (c, param) = geom::snap_to_face(cx.input.topo, f, c)
            .map_err(|e| GenerateFailure::new(target, "fan", FailureKind::Geometry, e.to_string()))?;
        let mut pb = PatchBuilder::new(target);
        let centre = pb.node(NodeKey::new(f, 0), c, param);
        let ring: Vec<u32> = lp.nodes.iter().map(|k| pb.refer(*k)).collect();
        // The loop runs with the face on its left, i.e. counter-clockwise about the normal.
        for i in 0..ring.len() {
            pb.push(ElemShape::Tri, &[centre, ring[i], ring[(i + 1) % ring.len()]]);
        }
        Ok(pb.finish())
    }
}

/// Quads between two loops with equal node counts (an annulus), pairing node `i` of the
/// outer loop with the node at the same angle on the inner loop.
pub struct Ring;

impl Scheme for Ring {
    fn name(&self) -> &str {
        "ring"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::QUAD | Caps::BLOCK_STRUCTURED
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(95, "ring quads")
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
        cx: &sm_scheme_api::IntervalCtx<'_>,
        target: Target,
        _p: &Params,
        out: &mut sm_scheme_api::IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            let edges: Vec<_> = cx.input.topo.face_edges(f).to_vec();
            if let [a, b] = edges.as_slice() {
                out.equal(&[*a], &[*b]);
            }
        }
        Ok(())
    }
    fn generate(&self, cx: &GenCtx<'_>, target: Target, _p: &Params) -> GResult<Patch> {
        let fatal = |r: String| GenerateFailure::fatal(target, "ring", r);
        let f = face_of(target, "ring")?;
        let loops = cx.loops(f).map_err(|e| fatal(e.to_string()))?;
        let [outer, inner] = loops.as_slice() else {
            return Err(fatal("ring needs two loops".into()));
        };
        if outer.nodes.len() != inner.nodes.len() {
            return Err(fatal("loops differ in node count".into()));
        }
        let n = outer.nodes.len();
        let ang = |k: NodeKey| cx.point(k).map(|p| p.y.atan2(p.x));
        let a0 = ang(outer.nodes[0]).ok_or_else(|| fatal("outer".into()))?;
        // The outer loop runs counter-clockwise, the inner one clockwise (hole): pair node i of
        // the outer with the inner node at the same angle.
        let mut pair = vec![0usize; n];
        for (i, k) in outer.nodes.iter().enumerate() {
            let a = ang(*k).ok_or_else(|| fatal("outer".into()))?;
            let (j, _) = inner
                .nodes
                .iter()
                .enumerate()
                .map(|(j, q)| {
                    let b = ang(*q).unwrap_or(f64::NAN);
                    let d = (b - a).rem_euclid(core::f64::consts::TAU);
                    (j, d.min(core::f64::consts::TAU - d))
                })
                .min_by(|x, y| x.1.total_cmp(&y.1))
                .ok_or_else(|| fatal("inner".into()))?;
            pair[i] = j;
        }
        let _ = a0;
        let mut pb = PatchBuilder::new(target);
        let o: Vec<u32> = outer.nodes.iter().map(|k| pb.refer(*k)).collect();
        let q: Vec<u32> = inner.nodes.iter().map(|k| pb.refer(*k)).collect();
        for i in 0..n {
            let j = (i + 1) % n;
            // Outer CCW (face on the left); the quad (o_i, o_j, i_j, i_i) is CCW about the normal.
            pb.push(ElemShape::Quad, &[o[i], o[j], q[pair[j]], q[pair[i]]]);
        }
        Ok(pb.finish())
    }
}

/// One `Polygon` element over the nodes of the single boundary loop.
pub struct OnePolygon;

impl Scheme for OnePolygon {
    fn name(&self) -> &str {
        "onepolygon"
    }
    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }
    fn caps(&self, _p: &Params) -> Caps {
        Caps::POLY
    }
    fn applicability(&self, _cx: &PlanCtx<'_>, _t: Target, _p: &Params) -> Applicability {
        Applicability::yes(95, "one polygon")
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
        let f = face_of(target, "onepolygon")?;
        let loops = cx
            .loops(f)
            .map_err(|e| GenerateFailure::fatal(target, "onepolygon", e.to_string()))?;
        let [lp] = loops.as_slice() else {
            return Err(GenerateFailure::fatal(target, "onepolygon", "needs a single loop"));
        };
        let mut pb = PatchBuilder::new(target);
        let ring: Vec<u32> = lp.nodes.iter().map(|k| pb.refer(*k)).collect();
        pb.push(ElemShape::Polygon, &ring);
        let _ = Param::None;
        Ok(pb.finish())
    }
}
