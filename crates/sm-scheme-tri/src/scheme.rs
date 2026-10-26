//! The `trimesh` / `tridelaunay` schemes: params, applicability and generation.

use crate::cdt::{
    Cdt, Geometry2, OptimizeOptions, Pslg, RefineOptions, Segment, SegmentKind,
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
}

impl TriScheme {
    /// The `trimesh` scheme (the default triangle scheme).
    pub const fn trimesh() -> Self {
        Self { name: "trimesh" }
    }

    /// The `tridelaunay` scheme (same algorithm under Cubit's name).
    pub const fn tridelaunay() -> Self {
        Self { name: "tridelaunay" }
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
        Applicability::yes(SCORE, "constrained Delaunay with refinement")
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
        triangulate(cx, face, &prm, target, self.name)
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
fn smallest_corner_deg(cx: &GenCtx<'_>, face: EntityId) -> f64 {
    match geom::corners(cx.input.topo, face) {
        Ok(cs) => cs
            .iter()
            .map(|c| c.angle.to_degrees())
            .fold(180.0, f64::min),
        Err(_) => 180.0,
    }
}

/// Mesh `face`. Shared by `trimesh` and `tridelaunay`.
pub(crate) fn triangulate(
    cx: &GenCtx<'_>,
    face: EntityId,
    prm: &TriParams,
    target: Target,
    name: &str,
) -> Result<Patch, GenerateFailure> {
    let fail = |e: TriError| fail_of(target, name, &e);
    let (chart, loops) = FaceChart::lift(cx, face).map_err(&fail)?;
    let asm = assemble(cx, face, &chart, &loops).map_err(&fail)?;
    let n_input = asm.pslg.points.len();
    let mut cdt = Cdt::new(&asm.pslg).map_err(&fail)?;
    let ropts = RefineOptions {
        min_angle: prm.min_angle,
        size_factor: prm.size_factor,
        max_points: prm.max_points as usize,
    };
    let report = cdt.refine(&chart, &ropts);
    if report.truncated {
        return Err(fail(TriError::TooManyPoints { limit: prm.max_points }));
    }
    if prm.optimize {
        cdt.optimize(&chart, &OptimizeOptions::default());
    }
    // Quality gate.
    let limit = prm.fail_angle.min(0.9 * smallest_corner_deg(cx, face));
    let keys_of = |t: [u32; 3]| -> Vec<NodeKey> {
        t.iter()
            .filter_map(|&v| key_of_point(&asm, &chart, n_input, v))
            .collect()
    };
    if let Some((ang, worst)) = cdt.min_angle(&chart)
        && ang < limit
    {
        let e = TriError::Quality {
            min_angle: ang,
            limit,
            worst,
        };
        return Err(GenerateFailure::fallback(target, name, e.to_string())
            .with_nodes(keys_of(worst)));
    }

    // Assemble the patch.
    let tris = cdt.triangles();
    let pts = cdt.points();
    let mut pb = PatchBuilder::new(target);
    let mut local: Vec<u32> = Vec::with_capacity(pts.len());
    let mut by_key: BTreeMap<NodeKey, u32> = BTreeMap::new();
    let mut p3: Vec<Point3> = Vec::with_capacity(pts.len());
    for (i, q) in pts.iter().enumerate() {
        let (key, is_new) = if i < n_input {
            (asm.keys[i], asm.new[i])
        } else {
            (
                NodeKey::new(face, chart.next_index() + (i - n_input) as u32),
                true,
            )
        };
        let idx = if let Some(&l) = by_key.get(&key) {
            l
        } else if is_new {
            let uv = chart.to_uv(*q);
            let p = chart.point_at(uv);
            let l = pb.node(key, p, Param::Surface(chart.wrap_uv(uv)));
            by_key.insert(key, l);
            l
        } else {
            let l = pb.refer(key);
            by_key.insert(key, l);
            l
        };
        local.push(idx);
        p3.push(if i < n_input && !asm.new[i] {
            cx.point(asm.keys[i]).unwrap_or_else(|| chart.lift3(*q))
        } else {
            chart.lift3(*q)
        });
    }
    // Normal check (the lift must not fold) and area bookkeeping.
    let mut worst_bad: Option<[u32; 3]> = None;
    for t in &tris {
        let (a, b, c) = (p3[t[0] as usize], p3[t[1] as usize], p3[t[2] as usize]);
        let n = (b - a).cross(c - a);
        let cen = [
            (pts[t[0] as usize][0] + pts[t[1] as usize][0] + pts[t[2] as usize][0]) / 3.0,
            (pts[t[0] as usize][1] + pts[t[1] as usize][1] + pts[t[2] as usize][1]) / 3.0,
        ];
        if let Some(fnrm) = chart.normal_at(chart.to_uv(cen))
            && n.dot(fnrm) <= 0.0
        {
            worst_bad = Some(*t);
            break;
        }
    }
    if let Some(t) = worst_bad {
        return Err(GenerateFailure::fallback(
            target,
            name,
            "a triangle folds over the surface (non-positive normal)",
        )
        .with_nodes(keys_of(t)));
    }
    for t in &tris {
        let l = [local[t[0] as usize], local[t[1] as usize], local[t[2] as usize]];
        if l[0] == l[1] || l[1] == l[2] || l[0] == l[2] {
            return Err(GenerateFailure::new(
                target,
                name,
                FailureKind::Geometry,
                "a triangle repeats a node (seam or pole not separated)",
            )
            .with_nodes(keys_of(*t)));
        }
        pb.push(ElemShape::Tri, &l);
    }
    // Pole fans: (pole, ring_i, ring_{i+1}) counter-clockwise in the chart is (b, a, pole).
    for lp in &loops {
        for fan in &lp.fans {
            let pole = pb.refer(fan.pole);
            for w in fan.path.windows(2) {
                let ia = by_key.get(&w[0]).copied();
                let ib = by_key.get(&w[1]).copied();
                let (Some(a), Some(b)) = (ia, ib) else {
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

fn key_of_point(asm: &Assembly, chart: &FaceChart<'_>, n_input: usize, v: u32) -> Option<NodeKey> {
    let v = v as usize;
    if v < n_input {
        Some(asm.keys[v])
    } else {
        Some(NodeKey::new(chart.face(), chart.next_index() + (v - n_input) as u32))
    }
}
