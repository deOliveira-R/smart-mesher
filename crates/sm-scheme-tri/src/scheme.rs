//! The `trimesh` / `tridelaunay` schemes: params, applicability and generation.

use crate::cdt::{
    Cdt, OptimizeOptions, Pslg, RefineOptions, Segment, SegmentKind,
};
use crate::chart::{ChartLoop, FaceChart};
use crate::error::TriError;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Point3};
use sm_scheme_api::{
    Applicability, Caps, ElemShape, FailureKind, GenCtx, GenerateFailure, NodeKey, Param, Params,
    Patch, PatchBuilder, PlanCtx, Requirements, Scheme, SchemeDim, Target, VertexRule, geom,
};
use sm_topo_api::TopologyViewExt;
use std::collections::BTreeMap;

/// Provisional score of `triadvance` (ruling S1: 20, below `trimesh`).
pub const FRONT_SCORE: u32 = 20;

/// Provisional automatic-selection score of the Delaunay schemes (decision 12: map 100,
/// submap 80, pave 40, trimesh 30). Only W3b auto selection reads it.
pub const SCORE: u32 = 30;

/// Parameters of `trimesh` and `tridelaunay`.
///
/// All fields are optional in the serialized form; unknown fields are rejected.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields, default)]
pub struct TriParams {
    /// Refinement target: triangles below this 3-D minimum angle (degrees) are refined where
    /// the fixed boundary allows it. Default 20.
    pub min_angle: f64,
    /// Quality gate: a face whose achieved minimum angle is below
    /// `min(fail_angle, 0.9 * smallest input corner angle)` fails with `Quality`. Default 12.
    pub fail_angle: f64,
    /// Scale of the target size (`1.0` = the size field, or the mean boundary segment length).
    pub size_factor: f64,
    /// Smooth and flip after refinement. Default true.
    pub optimize: bool,
    /// Budget on the number of points of one face. Default 2 000 000.
    pub max_points: u32,
}

impl Default for TriParams {
    fn default() -> Self {
        Self {
            min_angle: 20.0,
            fail_angle: 12.0,
            size_factor: 1.0,
            optimize: true,
            max_points: 2_000_000,
        }
    }
}

impl TriParams {
    /// Read and validate parameters; the error says what is wrong.
    pub fn from_params(p: &Params) -> Result<Self, String> {
        let v = if p.is_empty() {
            serde_json::Value::Object(Default::default())
        } else {
            p.0.clone()
        };
        let prm: TriParams = serde_json::from_value(v).map_err(|e| e.to_string())?;
        prm.validate()?;
        Ok(prm)
    }

    /// Check the ranges: `0 <= min_angle <= 34` (Ruppert's termination limit is about 33.8),
    /// `0 <= fail_angle < 60`, a positive finite `size_factor` and `max_points >= 4`.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.min_angle.is_finite() && (0.0..=34.0).contains(&self.min_angle)) {
            return Err(format!("min_angle {} must be in [0, 34] degrees", self.min_angle));
        }
        if !(self.fail_angle.is_finite() && (0.0..60.0).contains(&self.fail_angle)) {
            return Err(format!("fail_angle {} must be in [0, 60) degrees", self.fail_angle));
        }
        if !(self.size_factor.is_finite() && self.size_factor > 0.0) {
            return Err(format!("size_factor {} must be positive", self.size_factor));
        }
        if self.max_points < 4 {
            return Err("max_points must be at least 4".into());
        }
        Ok(())
    }
}

/// The Delaunay triangle scheme (`trimesh` and `tridelaunay` are the same algorithm).
///
/// Constrained Delaunay triangulation of the face's fixed boundary in a chart, then
/// Ruppert/Chew refinement of triangles that are bad in 3-D. The boundary is never changed:
/// no Steiner point is placed on a boundary segment (decision 12), so neighbouring faces and
/// the tet mesh on top stay conforming by construction.
///
/// **Guarantee.** The boundary is fixed. The Ruppert/Chew angle bound holds only where the
/// boundary spacing is compatible with the target size `h` and input corners are at least
/// 60 degrees. Elsewhere the scheme reports the achieved angle and either accepts (above the
/// gate `min(fail_angle, 0.9 * smallest input corner)`) or returns a `Quality` failure so the
/// driver can fall back. The documented minimum is 20 degrees where the fixed boundary allows
/// it; tetmesh calibrates against 20 degrees and accepts any valid closed surface.
#[derive(Clone, Copy, Debug)]
pub struct TriScheme {
    name: &'static str,
    method: Method,
    score: u32,
}

/// The algorithm behind a [`TriScheme`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Method {
    /// Constrained Delaunay plus Ruppert/Chew refinement.
    Delaunay,
    /// Advancing front plus smoothing and flips.
    Front,
}

impl TriScheme {
    /// The `trimesh` scheme (the default triangle scheme).
    pub const fn trimesh() -> Self {
        Self { name: "trimesh", method: Method::Delaunay, score: SCORE }
    }

    /// The `tridelaunay` scheme (same algorithm under Cubit's name).
    pub const fn tridelaunay() -> Self {
        Self { name: "tridelaunay", method: Method::Delaunay, score: SCORE }
    }

    /// The `triadvance` scheme: advancing front in the chart, then smoothing and flips. No angle
    /// guarantee beyond the gate; meant as the fallback of `trimesh` (provisional score 20).
    pub const fn triadvance() -> Self {
        Self { name: "triadvance", method: Method::Front, score: FRONT_SCORE }
    }
}

impl Scheme for TriScheme {
    fn name(&self) -> &str {
        self.name
    }

    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }

    fn caps(&self, _params: &Params) -> Caps {
        Caps::TRI
    }

    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, params: &Params) -> Applicability {
        let Target::Surface(f) = target else {
            return Applicability::no("not a surface");
        };
        if let Err(e) = TriParams::from_params(params) {
            return Applicability::no(e);
        }
        if cx.input.topo.loops(f).count() == 0 {
            return Applicability::no("face has no boundary loop");
        }
        Applicability::yes(self.score, match self.method {
            Method::Delaunay => "constrained Delaunay with refinement",
            Method::Front => "advancing front",
        })
    }

    fn requirements(
        &self,
        _cx: &PlanCtx<'_>,
        target: Target,
        _params: &Params,
        out: &mut Requirements<'_>,
    ) -> sm_scheme_api::Result<()> {
        if let Target::Surface(f) = target {
            out.vertex_rule(f, VertexRule::Free);
        }
        Ok(())
    }

    fn generate(
        &self,
        cx: &GenCtx<'_>,
        target: Target,
        params: &Params,
    ) -> Result<Patch, GenerateFailure> {
        let Target::Surface(face) = target else {
            return Err(GenerateFailure::fatal(target, self.name, "not a surface"));
        };
        let prm = TriParams::from_params(params).map_err(|e| {
            GenerateFailure::new(target, self.name, FailureKind::Constraint, e)
        })?;
        triangulate(cx, face, &prm, target, self.name, self.method)
    }
}

/// The input points of a face's PSLG with their node identities.
struct Assembly {
    pslg: Pslg,
    keys: Vec<NodeKey>,
    new: Vec<bool>,
    uv: Vec<[f64; 2]>,
}

fn fail_of(target: Target, name: &str, e: &TriError) -> GenerateFailure {
    GenerateFailure::new(target, name, e.kind(), e.to_string())
}

/// Boundary loops, then embedded chains, as a PSLG with keys.
fn assemble(
    cx: &GenCtx<'_>,
    face: EntityId,
    chart: &FaceChart<'_>,
    loops: &[ChartLoop],
) -> Result<Assembly, TriError> {
    let mut asm = Assembly {
        pslg: Pslg::default(),
        keys: Vec::new(),
        new: Vec::new(),
        uv: Vec::new(),
    };
    let mut by_key: BTreeMap<NodeKey, Vec<u32>> = BTreeMap::new();
    for lp in loops {
        let base = asm.pslg.points.len() as u32;
        let n = lp.nodes.len() as u32;
        for nd in &lp.nodes {
            let id = asm.pslg.points.len() as u32;
            asm.pslg.points.push(nd.q);
            asm.keys.push(nd.key);
            asm.new.push(nd.new);
            asm.uv.push(nd.uv);
            by_key.entry(nd.key).or_default().push(id);
        }
        for i in 0..n {
            asm.pslg.segments.push(Segment {
                a: base + i,
                b: base + (i + 1) % n,
                kind: SegmentKind::Boundary,
            });
        }
    }
    // Embedded chains: every chain node a vertex, every chain segment an edge.
    let chains = cx
        .embedded(face)
        .map_err(|e| TriError::Degenerate(e.to_string()))?;
    if !chains.is_empty() {
        let centroid_uv = {
            let o = &loops[0].nodes;
            let n = o.len() as f64;
            [
                o.iter().map(|c| c.uv[0]).sum::<f64>() / n,
                o.iter().map(|c| c.uv[1]).sum::<f64>() / n,
            ]
        };
        for chain in chains {
            let mut prev: Option<u32> = None;
            for key in chain {
                let hint = prev.map_or(centroid_uv, |p| asm.uv[p as usize]);
                let id = match by_key.get(&key).map(Vec::as_slice) {
                    Some([only]) => *only,
                    Some(many) if !many.is_empty() => *many
                        .iter()
                        .min_by(|a, b| {
                            let d = |i: &&u32| {
                                let u = asm.uv[**i as usize];
                                (u[0] - hint[0]).powi(2) + (u[1] - hint[1]).powi(2)
                            };
                            d(a).total_cmp(&d(b))
                        })
                        .unwrap_or(&many[0]),
                    _ => {
                        let p = cx.point(key).ok_or(TriError::Lift { key })?;
                        let uv = chart.uv_near(p, hint).ok_or(TriError::Lift { key })?;
                        let id = asm.pslg.points.len() as u32;
                        asm.pslg.points.push(chart.to_q(uv));
                        asm.keys.push(key);
                        asm.new.push(false);
                        asm.uv.push(uv);
                        by_key.entry(key).or_default().push(id);
                        id
                    }
                };
                if let Some(p) = prev
                    && p != id
                {
                    asm.pslg.segments.push(Segment {
                        a: p,
                        b: id,
                        kind: SegmentKind::Embedded,
                    });
                }
                prev = Some(id);
            }
        }
    }
    Ok(asm)
}

/// Smallest interior corner angle of the face's loops in degrees (180 if none is measurable).
///
/// Turn-back corners (the next real coedge uses the same edge, as at a pole where the seam goes
/// in and straight out again) are not corners of the boundary and are left out.
fn smallest_corner_deg(cx: &GenCtx<'_>, face: EntityId) -> f64 {
    let mut turn_back: Vec<_> = Vec::new();
    for lp in geom::face_loops(cx.input.topo, face) {
        for (i, k) in lp.iter().enumerate() {
            if lp[(i + 1) % lp.len()].edge == k.edge {
                turn_back.push(*k);
            }
        }
    }
    match geom::corners(cx.input.topo, face) {
        Ok(cs) => cs
            .iter()
            .filter(|c| !turn_back.contains(&c.key.incoming))
            .map(|c| c.angle.to_degrees())
            .fold(180.0, f64::min),
        Err(_) => 180.0,
    }
}

/// A face triangulated in its chart, before the quality gate and patch assembly.
///
/// Exposed so tests (and the stretch front) can inspect the exact chart-space triangulation.
pub struct FaceTriangulation<'a> {
    /// The chart (also the kernel's [`crate::cdt::Geometry2`]).
    pub chart: FaceChart<'a>,
    /// The lifted boundary loops (with pole fans).
    pub loops: Vec<ChartLoop>,
    /// The refined triangulation; input points `0..n_input` are the PSLG points.
    pub cdt: Cdt,
    /// Node key of each input point.
    pub keys: Vec<NodeKey>,
    /// True for input points this face creates (pole-ring nodes).
    pub new: Vec<bool>,
    /// What refinement reported.
    pub report: crate::cdt::RefineReport,
}

/// Lift `face`, triangulate, refine and (if `prm.optimize`) optimize in the chart.
///
/// # Errors
/// Any [`TriError`] of lifting, triangulation or the point budget.
pub fn triangulate_chart<'a>(
    cx: &'a GenCtx<'_>,
    face: EntityId,
    prm: &TriParams,
) -> Result<FaceTriangulation<'a>, TriError> {
    triangulate_chart_with(cx, face, prm, Method::Delaunay)
}

/// [`triangulate_chart`] with the advancing front instead of Delaunay refinement.
///
/// # Errors
/// Any [`TriError`], in particular [`TriError::Stuck`] when the front cannot be closed.
pub fn advance_chart<'a>(
    cx: &'a GenCtx<'_>,
    face: EntityId,
    prm: &TriParams,
) -> Result<FaceTriangulation<'a>, TriError> {
    triangulate_chart_with(cx, face, prm, Method::Front)
}

fn triangulate_chart_with<'a>(
    cx: &'a GenCtx<'_>,
    face: EntityId,
    prm: &TriParams,
    method: Method,
) -> Result<FaceTriangulation<'a>, TriError> {
    let (chart, loops) = FaceChart::lift(cx, face)?;
    let asm = assemble(cx, face, &chart, &loops)?;
    if method == Method::Front {
        let aopts = crate::front::AdvanceOptions {
            size_factor: prm.size_factor,
            optimize: prm.optimize,
            max_points: prm.max_points as usize,
            ..Default::default()
        };
        let cdt = crate::front::advance(&asm.pslg, &chart, &aopts)?;
        return Ok(FaceTriangulation {
            chart,
            loops,
            cdt,
            keys: asm.keys,
            new: asm.new,
            report: crate::cdt::RefineReport::default(),
        });
    }
    let mut cdt = Cdt::new(&asm.pslg).map_err(|e| {
        // Pole rings that collide or cross mean the boundary is too coarse for pole fans.
        if loops.iter().any(|l| !l.fans.is_empty())
            && matches!(
                e,
                TriError::DuplicatePoint { .. }
                    | TriError::SelfIntersection { .. }
                    | TriError::Degenerate(_)
            )
        {
            TriError::Unsupported(format!(
                "pole rings collide ({e}); the face needs a finer boundary (r >= 2h) or a model-level partition"
            ))
        } else {
            e
        }
    })?;
    let ropts = RefineOptions {
        min_angle: prm.min_angle,
        size_factor: prm.size_factor,
        max_points: prm.max_points as usize,
    };
    let report = cdt.refine(&chart, &ropts);
    if report.truncated {
        return Err(TriError::TooManyPoints { limit: prm.max_points });
    }
    if prm.optimize {
        cdt.optimize(&chart, &OptimizeOptions::default());
    }
    Ok(FaceTriangulation {
        chart,
        loops,
        cdt,
        keys: asm.keys,
        new: asm.new,
        report,
    })
}

/// Mesh `face`. Shared by `trimesh` and `tridelaunay`.
fn triangulate(
    cx: &GenCtx<'_>,
    face: EntityId,
    prm: &TriParams,
    target: Target,
    name: &str,
    method: Method,
) -> Result<Patch, GenerateFailure> {
    let mut ft = triangulate_chart_with(cx, face, prm, method).map_err(|e| fail_of(target, name, &e))?;
    ft.into_patch(cx, face, prm, target, name)
}

impl FaceTriangulation<'_> {
    fn key_of(&self, v: u32) -> NodeKey {
        let n_input = self.keys.len();
        let v = v as usize;
        if v < n_input {
            self.keys[v]
        } else {
            NodeKey::new(self.chart.face(), self.chart.next_index() + (v - n_input) as u32)
        }
    }

    /// Gate the pole-fan triangles `(b, a, pole)`: 3-D minimum angle at least `limit` degrees and
    /// a positive normal. `p3` holds the lifted kernel points.
    fn gate_fans(
        &self,
        cx: &GenCtx<'_>,
        p3: &[Point3],
        limit: f64,
        target: Target,
        name: &str,
    ) -> Result<(), GenerateFailure> {
        let pos: BTreeMap<NodeKey, usize> = (0..p3.len()).map(|i| (self.key_of(i as u32), i)).collect();
        for lp in &self.loops {
            for fan in &lp.fans {
                let Some(pole) = cx.point(fan.pole) else { continue };
                for w in fan.path.windows(2) {
                    let (Some(&ia), Some(&ib)) = (pos.get(&w[0]), pos.get(&w[1])) else { continue };
                    let (a, b) = (p3[ia], p3[ib]);
                    let ang = tri_min_angle_deg(b, a, pole);                    let mid = [0, 1].map(|k| 0.5 * (self.cdt.points()[ia][k] + self.cdt.points()[ib][k]));
                    let folded = self
                        .chart
                        .normal_at(self.chart.to_uv(mid))
                        .is_some_and(|f| (a - b).cross(pole - b).dot(f) <= 0.0);
                    if ang < limit || folded {
                        let worst = [w[1], w[0], fan.pole];
                        let msg = if folded {
                            "a pole-fan triangle folds over the surface (non-positive normal)".to_string()
                        } else {
                            format!("pole-fan triangle minimum angle {ang:.3} deg is below the gate {limit:.3} deg")
                        };
                        return Err(GenerateFailure::fallback(target, name, msg).with_nodes(worst.to_vec()));
                    }
                }
            }
        }
        Ok(())
    }

    /// Apply the quality gate and build the patch.
    ///
    /// The gate is `min_angle_3d >= min(fail_angle, 0.9 * smallest input corner angle)` and a
    /// positive 3-D normal on every triangle; a failure is `FailureKind::Quality` with the nodes
    /// of the worst triangle.
    ///
    /// # Errors
    /// A [`GenerateFailure`]: `Quality` below the gate, `Geometry` for a triangle that repeats
    /// a node.
    pub fn into_patch(
        &mut self,
        cx: &GenCtx<'_>,
        face: EntityId,
        prm: &TriParams,
        target: Target,
        name: &str,
    ) -> Result<Patch, GenerateFailure> {
        let n_input = self.keys.len();
        let limit = prm.fail_angle.min(0.9 * smallest_corner_deg(cx, face));
        let keys_of = |this: &Self, t: [u32; 3]| -> Vec<NodeKey> { t.iter().map(|&v| this.key_of(v)).collect() };
        if let Some((ang, worst)) = self.cdt.min_angle(&self.chart)
            && ang < limit
        {
            let e = TriError::Quality { min_angle: ang, limit, worst };
            return Err(GenerateFailure::fallback(target, name, e.to_string())
                .with_nodes(keys_of(self, worst)));
        }
        let chart = &self.chart;
        let tris = self.cdt.triangles();
        let pts = self.cdt.points();
        let mut pb = PatchBuilder::new(target);
        let mut local: Vec<u32> = Vec::with_capacity(pts.len());
        let mut by_key: BTreeMap<NodeKey, u32> = BTreeMap::new();
        let mut p3: Vec<Point3> = Vec::with_capacity(pts.len());
        for (i, q) in pts.iter().enumerate() {
            let key = self.key_of(i as u32);
            let is_new = i >= n_input || self.new[i];
            let idx = if let Some(&l) = by_key.get(&key) {
                l
            } else if is_new {
                let uv = chart.to_uv(*q);
                let l = pb.node(key, chart.point_at(uv), Param::Surface(chart.wrap_uv(uv)));
                by_key.insert(key, l);
                l
            } else {
                let l = pb.refer(key);
                by_key.insert(key, l);
                l
            };
            local.push(idx);
            p3.push(if is_new {
                chart.lift3(*q)
            } else {
                cx.point(key).unwrap_or_else(|| chart.lift3(*q))
            });
        }
        // The lift must not fold the surface: every triangle follows the face normal.
        for t in &tris {
            let (a, b, c) = (p3[t[0] as usize], p3[t[1] as usize], p3[t[2] as usize]);
            let n = (b - a).cross(c - a);
            let cen = [0, 1].map(|k| {
                (pts[t[0] as usize][k] + pts[t[1] as usize][k] + pts[t[2] as usize][k]) / 3.0
            });
            if let Some(f) = chart.normal_at(chart.to_uv(cen))
                && n.dot(f) <= 0.0
            {
                return Err(GenerateFailure::fallback(
                    target,
                    name,
                    "a triangle folds over the surface (non-positive normal)",
                )
                .with_nodes(keys_of(self, *t)));
            }
        }
        // Pole fans take the same gate (minimum angle and normal) as the kernel triangles.
        self.gate_fans(cx, &p3, limit, target, name)?;
        for t in &tris {
            let l = [local[t[0] as usize], local[t[1] as usize], local[t[2] as usize]];
            if l[0] == l[1] || l[1] == l[2] || l[0] == l[2] {
                return Err(GenerateFailure::new(
                    target,
                    name,
                    FailureKind::Geometry,
                    "a triangle repeats a node (seam or pole not separated)",
                )
                .with_nodes(keys_of(self, *t)));
            }
            pb.push(ElemShape::Tri, &l);
        }
        // Pole fans: (pole, ring_i, ring_{i+1}) counter-clockwise in the chart is (b, a, pole).
        for lp in &self.loops {
            for fan in &lp.fans {
                let pole = pb.refer(fan.pole);
                for w in fan.path.windows(2) {
                    let (Some(&a), Some(&b)) = (by_key.get(&w[0]), by_key.get(&w[1])) else {
                        return Err(GenerateFailure::new(
                            target,
                            name,
                            FailureKind::Internal,
                            "pole ring node missing from the patch",
                        ));
                    };
                    if a == b || a == pole || b == pole {
                        return Err(GenerateFailure::new(
                            target,
                            name,
                            FailureKind::Geometry,
                            "pole fan triangle repeats a node",
                        ));
                    }
                    pb.push(ElemShape::Tri, &[b, a, pole]);
                }
            }
        }
        Ok(pb.finish())
    }
}

/// Smallest angle of the triangle `(a, b, c)` in degrees (0 for a degenerate one).
fn tri_min_angle_deg(a: Point3, b: Point3, c: Point3) -> f64 {
    let ang = |p: Point3, q: Point3, r: Point3| {
        let (u, v) = (q - p, r - p);
        let (nu, nv) = (u.norm(), v.norm());
        if nu == 0.0 || nv == 0.0 {
            return 0.0;
        }
        u.cross(v).norm().atan2(u.dot(v)).to_degrees()
    };
    ang(a, b, c).min(ang(b, c, a)).min(ang(c, a, b))
}
