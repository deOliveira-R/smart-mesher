//! The surface `pave` scheme: unstructured all-quad meshing of a face.
//!
//! The face is meshed in its (metric-corrected) parameter space by [`pave_region`]:
//! constrained Delaunay triangulation of the given boundary nodes, frontal rows of interior
//! points, perfect matching of the triangles into quads, smoothing. The boundary nodes come
//! from the plan (never split); the only constraint on the interval solver is an even
//! boundary total. See `DESIGN.md`.
//!
//! Entry points: [`register`], [`Pave`], [`PaveParams`] and the pure kernel
//! [`pave_region`].

mod kernel;

pub use kernel::{PaveError, PaveOptions, QuadMesh2, pave_region};

use serde::Deserialize;
use sm_core::{EntityId, Point3};
use sm_geom_api::{ParametricSurface, Uv};
use sm_scheme_api::geom;
use sm_scheme_api::{
    Applicability, Caps, ElemShape, FailureKind, GenCtx, GenerateFailure, IntervalCtx,
    IntervalSink, NodeKey, Param, Params, Patch, PatchBuilder, PlanCtx, Rel, Requirements,
    Scheme, SchemeDim, SchemeRegistry, Target, VertexRule,
};
use sm_topo_api::{Carrier, CellFlags, Sense, TopologyView, TopologyViewExt};
use std::collections::BTreeMap;

/// Registered name of the scheme.
pub const NAME: &str = "pave";

/// A quad whose 3-D scaled Jacobian (corner sine) is at or below this is a `Quality` failure.
const MIN_SJ_3D: f64 = 0.05;

/// Register the surface paving scheme (`"pave"`, [`SchemeDim::Surface`]).
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    registry.register(Box::new(Pave))
}

/// Unstructured all-quad surface scheme. Produces [`Caps::QUAD`] only (never structured: a
/// sweep may take a paved source but must not use it as a linking face). Takes
/// [`PaveParams`].
#[derive(Clone, Copy, Debug, Default)]
pub struct Pave;

/// Parameters of `pave`. Unknown fields and out-of-range values make the scheme not
/// applicable.
#[derive(Clone, Debug, Default, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PaveParams {
    /// Maximum relative size change per unit length of the default size field, in `(0, 1]`
    /// (default 0.3).
    pub gradation: Option<f64>,
    /// Smoothing sweeps, at most 10000 (default 30).
    pub smooth_iters: Option<u32>,
}

impl PaveParams {
    /// Parse and validate scheme parameters into kernel options.
    pub fn options(params: &Params) -> Result<PaveOptions, String> {
        let p: PaveParams = if params.is_empty() {
            PaveParams::default()
        } else {
            serde_json::from_value(params.0.clone()).map_err(|e| format!("pave parameters: {e}"))?
        };
        let mut o = PaveOptions::default();
        if let Some(g) = p.gradation {
            if !(g.is_finite() && g > 0.0 && g <= 1.0) {
                return Err(format!("pave: gradation {g} is outside (0, 1]"));
            }
            o.gradation = g;
        }
        if let Some(n) = p.smooth_iters {
            if n > 10_000 {
                return Err(format!("pave: smooth_iters {n} exceeds 10000"));
            }
            o.smooth_iters = n;
        }
        Ok(o)
    }
}

/// The surface of the carrier of `f`, with the carrier's sense.
fn surface_of(topo: &dyn TopologyView, f: EntityId) -> Option<(&dyn ParametricSurface, Sense)> {
    match topo.carrier(f) {
        Carrier::Surface(sc) => topo.shapes().surface(sc.surface).map(|s| (s, sc.sense)),
        _ => None,
    }
}

impl Scheme for Pave {
    fn name(&self) -> &str {
        NAME
    }

    fn dim(&self) -> SchemeDim {
        SchemeDim::Surface
    }

    fn caps(&self, _params: &Params) -> Caps {
        Caps::QUAD
    }

    fn applicability(&self, cx: &PlanCtx<'_>, target: Target, params: &Params) -> Applicability {
        let Target::Surface(f) = target else {
            return Applicability::no("pave meshes surfaces");
        };
        if let Err(e) = PaveParams::options(params) {
            return Applicability::no(e);
        }
        let topo = cx.input.topo;
        let Some((surf, _)) = surface_of(topo, f) else {
            return Applicability::no("pave needs a face with a surface carrier");
        };
        if topo.face_edges(f).iter().any(|e| topo.flags(*e).contains(CellFlags::DEGENERATE)) {
            return Applicability::no("pave does not mesh faces with a degenerate (pole) edge yet");
        }
        if !cx.input.embedded_edges(f).is_empty() || !cx.input.embedded_vertices(f).is_empty() {
            return Applicability::no("pave cannot conform to embedded edges or vertices yet");
        }
        let dom = surf.domain();
        if (dom.u.is_periodic() || dom.v.is_periodic())
            && geom::face_loops(topo, f).iter().flatten().any(|k| topo.pcurve(*k).is_none())
        {
            return Applicability::no(
                "periodic surface with a coedge without pcurve: the seam cannot be unrolled",
            );
        }
        if topo.loops(f).count() == 0 {
            return Applicability::no("face without boundary loops");
        }
        Applicability::yes(40, "unstructured quad")
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

    fn interval_constraints(
        &self,
        cx: &IntervalCtx<'_>,
        target: Target,
        _params: &Params,
        out: &mut IntervalSink<'_>,
    ) -> sm_scheme_api::Result<()> {
        let Target::Surface(f) = target else {
            return Ok(());
        };
        let loops = geom::face_loops(cx.input.topo, f);
        let all: Vec<EntityId> = loops.iter().flatten().map(|k| k.edge).collect();
        if all.is_empty() {
            return Ok(());
        }
        // Even total over all loops (a seam used twice counts twice), and a polygon needs at
        // least four nodes per loop.
        out.even(&all);
        for lp in &loops {
            let mut terms: BTreeMap<EntityId, i32> = BTreeMap::new();
            for k in lp {
                *terms.entry(k.edge).or_insert(0) += 1;
            }
            let terms: Vec<(EntityId, i32)> = terms.into_iter().collect();
            out.linear(&terms, Rel::Ge, 4);
        }
        Ok(())
    }

    fn generate(
        &self,
        cx: &GenCtx<'_>,
        target: Target,
        params: &Params,
    ) -> Result<Patch, GenerateFailure> {
        let fail = |kind: FailureKind, r: String| GenerateFailure::new(target, NAME, kind, r);
        let Target::Surface(f) = target else {
            return Err(fail(FailureKind::Internal, "not a surface".into()));
        };
        let opts = PaveParams::options(params).map_err(|e| fail(FailureKind::NotApplicable, e))?;
        let topo = cx.input.topo;
        let (surf, sense) = surface_of(topo, f)
            .ok_or_else(|| fail(FailureKind::NotApplicable, "face without a surface carrier".into()))?;
        let loops = cx.loops(f).map_err(|e| fail(FailureKind::Internal, e.to_string()))?;
        if loops.is_empty() {
            return Err(fail(FailureKind::Internal, "face without boundary loops".into()));
        }

        // Boundary (u, v) per node from the pcurves (seam aware).
        let mut uv_loops: Vec<Vec<[f64; 2]>> = Vec::with_capacity(loops.len());
        let mut keys: Vec<NodeKey> = Vec::new();
        for lp in &loops {
            let uvs = cx.loop_uvs(f, lp).ok_or_else(|| {
                fail(
                    FailureKind::Geometry,
                    format!("loop {}: the boundary parameters (u, v) cannot be determined (missing pcurves, seam mismatch or no surface)", lp.ring),
                )
            })?;
            uv_loops.push(uvs);
            keys.extend_from_slice(&lp.nodes);
        }
        let nb = keys.len();
        if nb % 2 == 1 {
            return Err(fail(
                FailureKind::Constraint,
                format!("{nb} boundary nodes: an all-quad mesh needs an even total"),
            ));
        }

        let metric = |uv: [f64; 2]| -> [f64; 3] {
            let j = surf.jet(Uv::new(uv[0], uv[1]));
            [j.du.dot(j.du), j.du.dot(j.dv), j.dv.dot(j.dv)]
        };
        let size_fn = cx.size.map(|s| move |uv: [f64; 2]| s.size(surf.eval(Uv::new(uv[0], uv[1]))));
        let size_ref: Option<&dyn Fn([f64; 2]) -> f64> = size_fn.as_ref().map(|f| f as &dyn Fn([f64; 2]) -> f64);
        let mesh = pave_region(&uv_loops, &metric, size_ref, &opts).map_err(|e| {
            let kind = match e {
                PaveError::OddBoundary { .. } | PaveError::LoopTooSmall { .. } => FailureKind::Constraint,
                PaveError::Internal(_) => FailureKind::Internal,
                _ => FailureKind::Geometry,
            };
            fail(kind, e.to_string())
        })?;

        // Interior points on the surface.
        let interior_uv: Vec<Uv> = mesh.points[nb..].iter().map(|p| Uv::new(p[0], p[1])).collect();
        let mut interior_pt = vec![Point3::default(); interior_uv.len()];
        surf.eval_many(&interior_uv, &mut interior_pt);
        let mut pb = PatchBuilder::new(target);
        let mut local: Vec<u32> = Vec::with_capacity(mesh.points.len());
        let mut node_keys: Vec<NodeKey> = Vec::with_capacity(mesh.points.len());
        for key in &keys {
            local.push(pb.refer(*key));
            node_keys.push(*key);
        }
        for (k, (p, uv)) in interior_pt.iter().zip(&interior_uv).enumerate() {
            let key = NodeKey::new(f, k as u32);
            local.push(pb.node(key, *p, Param::Surface([uv.u, uv.v])));
            node_keys.push(key);
        }

        // Positions for the checks: boundary from the earlier waves, interior from the surface.
        let mut pos: Vec<Point3> = Vec::with_capacity(mesh.points.len());
        for key in &keys {
            pos.push(cx.point(*key).ok_or_else(|| {
                fail(FailureKind::Internal, format!("boundary node {key:?} of an earlier wave is missing"))
            })?);
        }
        pos.extend(interior_pt.iter().copied());

        for q in &mesh.quads {
            let qk: [NodeKey; 4] = std::array::from_fn(|k| node_keys[q[k] as usize]);
            let quality = |reason: String| fail(FailureKind::Quality, reason).with_nodes(qk.to_vec());
            if (0..4).any(|a| (a + 1..4).any(|b| qk[a] == qk[b])) {
                return Err(quality("a quad repeats a node".into()));
            }
            let p: [Point3; 4] = std::array::from_fn(|k| pos[q[k] as usize]);
            let uvc = {
                let c = [0, 1, 2, 3].map(|k| mesh.points[q[k] as usize]);
                Uv::new((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4.0, (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4.0)
            };
            let Ok(n) = surf.normal(uvc) else {
                return Err(quality("no surface normal at the quad".into()));
            };
            let n = if sense == Sense::Reversed { -n } else { n };
            let area_n = (p[2] - p[0]).cross(p[3] - p[1]);
            if area_n.dot(n) <= 0.0 {
                return Err(quality("a quad's normal disagrees with the face normal".into()));
            }
            let mut sjmin = f64::INFINITY;
            for k in 0..4 {
                let (a, b, c) = (p[(k + 3) % 4], p[k], p[(k + 1) % 4]);
                let (e1, e2) = (b - a, c - b);
                let l = e1.norm() * e2.norm();
                sjmin = sjmin.min(if l > 0.0 { e1.cross(e2).dot(n) / l } else { -1.0 });
            }
            if !(sjmin > MIN_SJ_3D) {
                return Err(quality(format!("a quad has 3-D scaled Jacobian {sjmin:.3}")));
            }
            pb.push(ElemShape::Quad, &[local[q[0] as usize], local[q[1] as usize], local[q[2] as usize], local[q[3] as usize]]);
        }
        Ok(pb.finish())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn params_reject_unknown_fields_and_out_of_range_values() {
        let p = |v: serde_json::Value| PaveParams::options(&Params(v));
        assert!(p(serde_json::json!({})).is_ok());
        assert_eq!(p(serde_json::json!({"gradation": 0.5, "smooth_iters": 3})).map(|o| (o.gradation, o.smooth_iters)), Ok((0.5, 3)));
        assert!(p(serde_json::json!({"bogus": 1})).is_err());
        assert!(p(serde_json::json!({"gradation": 0.0})).is_err());
        assert!(p(serde_json::json!({"gradation": 2.0})).is_err());
        assert!(p(serde_json::json!({"smooth_iters": 100000})).is_err());
        assert!(p(serde_json::json!({"smooth_iters": -1})).is_err());
    }

    #[test]
    fn register_is_callable_once() {
        let mut reg = SchemeRegistry::new();
        register(&mut reg).expect("register");
        assert!(register(&mut reg).is_err(), "double registration is rejected");
        let s = reg.get(SchemeDim::Surface, NAME).expect("pave");
        assert_eq!(s.caps(&Params::default()), Caps::QUAD);
        assert!(!s.supports_groups());
    }
}
