//! Geometry helpers schemes share: vertex points, edge evaluation in the edge's own direction,
//! face normals and corner angles. All read-only over `&dyn TopologyView`.

use crate::error::{Result, SchemeError};
use core::f64::consts::PI;
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{ParametricCurve, Uv};
use sm_mesh_api::Param;
use sm_topo_api::{
    Carrier, CoedgeKey, CoedgeKind, CornerKey, CurveCarrier, Sense, Side, TopologyView,
    TopologyViewExt,
};

/// Position of a topology vertex (its point carrier).
pub fn vertex_point(t: &dyn TopologyView, v: EntityId) -> Result<Point3> {
    match t.carrier(v) {
        Carrier::Point(p) => Ok(*p),
        _ => Err(SchemeError::Input(format!(
            "vertex {v} has no point carrier"
        ))),
    }
}

/// The curve and carrier of an edge.
pub fn edge_curve(
    t: &dyn TopologyView,
    e: EntityId,
) -> Result<(&dyn ParametricCurve, CurveCarrier)> {
    match t.carrier(e) {
        Carrier::Curve(cc) => {
            let c = t.shapes().curve(cc.curve).ok_or_else(|| {
                SchemeError::Input(format!("edge {e}: curve {} not in store", cc.curve))
            })?;
            Ok((c, *cc))
        }
        _ => Err(SchemeError::Input(format!("edge {e} has no curve carrier"))),
    }
}

/// Curve parameter at fraction `s` in `[0, 1]` of the carrier range, measured from the
/// edge's start vertex (honours the carrier sense).
pub fn edge_param(cc: &CurveCarrier, s: f64) -> f64 {
    let (lo, hi) = (cc.range.lo, cc.range.hi);
    match cc.sense {
        Sense::Forward => lo + s * (hi - lo),
        Sense::Reversed => hi - s * (hi - lo),
    }
}

/// Point at fraction `s` of the edge (from its start vertex, uniform in curve parameter).
pub fn edge_point(t: &dyn TopologyView, e: EntityId, s: f64) -> Result<Point3> {
    let (c, cc) = edge_curve(t, e)?;
    Ok(c.eval(edge_param(&cc, s)))
}

/// Unit tangent at fraction `s`, pointing along the edge direction.
pub fn edge_tangent(t: &dyn TopologyView, e: EntityId, s: f64) -> Result<Vector3> {
    let (c, cc) = edge_curve(t, e)?;
    let d = c.tangent(edge_param(&cc, s))?;
    Ok(if cc.sense == Sense::Reversed { -d } else { d })
}

/// Arc length of an edge.
pub fn edge_length(t: &dyn TopologyView, e: EntityId) -> Result<f64> {
    let (c, cc) = edge_curve(t, e)?;
    Ok(c.length(cc.range, t.tolerance_of(e))?)
}

/// Interval goal of an edge under a size field: `integral ds / h(x)` along the edge (the number
/// of elements of size `h` that fit), by composite 5-point Gauss-Legendre over the curve
/// parameter, doubling the panel count until two estimates agree to `1e-10` relative (at most
/// 2^14 panels; the last estimate is returned). Deterministic. A non-positive or non-finite
/// size is an input error.
pub fn edge_size_goal(
    t: &dyn TopologyView,
    e: EntityId,
    f: &dyn crate::sizing::SizeField,
) -> Result<f64> {
    const X: [f64; 5] = [
        0.0,
        -0.538_469_310_105_683_1,
        0.538_469_310_105_683_1,
        -0.906_179_845_938_664,
        0.906_179_845_938_664,
    ];
    const W: [f64; 5] = [
        0.568_888_888_888_888_9,
        0.478_628_670_499_366_5,
        0.478_628_670_499_366_5,
        0.236_926_885_056_189_1,
        0.236_926_885_056_189_1,
    ];
    let (c, cc) = edge_curve(t, e)?;
    let (lo, hi) = (cc.range.lo, cc.range.hi);
    if !(lo.is_finite() && hi.is_finite() && hi > lo) {
        return Err(SchemeError::Input(format!(
            "edge {e}: unbounded or empty curve range"
        )));
    }
    let estimate = |panels: u32| -> Result<f64> {
        let w = (hi - lo) / f64::from(panels);
        let mut sum = 0.0;
        for k in 0..panels {
            let mid = lo + (f64::from(k) + 0.5) * w;
            for (x, wt) in X.iter().zip(W) {
                let u = mid + 0.5 * w * x;
                let j = c.jet(u);
                let h = f.size(j.p);
                if !(h.is_finite() && h > 0.0) {
                    return Err(SchemeError::Input(format!("edge {e}: size field gave {h}")));
                }
                sum += wt * j.d1.norm() / h;
            }
        }
        Ok(sum * 0.5 * w)
    };
    let mut panels = 4u32;
    let mut prev = estimate(panels)?;
    while panels < (1 << 14) {
        panels *= 2;
        let next = estimate(panels)?;
        if (next - prev).abs() <= 1e-10 * next.abs() {
            return Ok(next);
        }
        prev = next;
    }
    Ok(prev)
}

/// `(start, end)` vertices of a coedge in loop direction.
pub fn coedge_ends(t: &dyn TopologyView, k: CoedgeKey) -> Option<(EntityId, EntityId)> {
    let (a, b) = t.edge_ends(k.edge)?;
    Some(if k.side == Side::Right {
        (b, a)
    } else {
        (a, b)
    })
}

/// Travel direction of a coedge at fraction `s` of the loop direction.
fn coedge_dir(t: &dyn TopologyView, k: CoedgeKey, s: f64) -> Result<Vector3> {
    if k.side == Side::Right {
        Ok(-edge_tangent(t, k.edge, 1.0 - s)?)
    } else {
        edge_tangent(t, k.edge, s)
    }
}

/// Unit normal of a face at (near) `p`, oriented left to right (the topology orientation
/// rule). Uses the surface carrier when present, else the Newell normal of the outline.
pub fn face_normal(t: &dyn TopologyView, f: EntityId, p: Point3) -> Result<Vector3> {
    if let Carrier::Surface(sc) = t.carrier(f)
        && let Some(s) = t.shapes().surface(sc.surface)
    {
        let uv: Uv = s.project(p, None)?.uv;
        let n = s.normal(uv)?;
        return Ok(if sc.sense == Sense::Reversed { -n } else { n });
    }
    let pts = t.face_outline(f);
    let mut n = Vector3::default();
    for i in 0..pts.len() {
        let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
        n = n + Vector3::new(
            (a.y - b.y) * (a.z + b.z),
            (a.z - b.z) * (a.x + b.x),
            (a.x - b.x) * (a.y + b.y),
        );
    }
    let l = n.norm();
    if l > 0.0 {
        Ok(n * (1.0 / l))
    } else {
        Err(SchemeError::Input(format!("face {f}: no normal")))
    }
}

/// One corner of a face loop with its measured interior angle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Corner {
    /// The corner key (face + incoming coedge).
    pub key: CornerKey,
    /// The vertex.
    pub vertex: EntityId,
    /// Loop index.
    pub ring: u16,
    /// Interior angle in radians, in `(0, 2 pi)`.
    pub angle: f64,
}

/// The corners of every loop of `f`, in ring then loop order. Pole coedges are skipped.
pub fn corners(t: &dyn TopologyView, f: EntityId) -> Result<Vec<Corner>> {
    let mut out = Vec::new();
    for lp in t.loops(f) {
        let keys: Vec<CoedgeKey> = lp
            .keys()
            .filter(|k| !t.flags(k.edge).contains(sm_topo_api::CellFlags::DEGENERATE))
            .collect();
        for i in 0..keys.len() {
            let (inc, out_k) = (keys[i], keys[(i + 1) % keys.len()]);
            let (_, v) = coedge_ends(t, inc)
                .ok_or_else(|| SchemeError::Input(format!("edge {} has no ends", inc.edge)))?;
            let p = vertex_point(t, v)?;
            let n = face_normal(t, f, p)?;
            let (a, b) = (coedge_dir(t, inc, 1.0)?, coedge_dir(t, out_k, 0.0)?);
            let turn = n.dot(a.cross(b)).atan2(a.dot(b));
            out.push(Corner {
                key: CornerKey {
                    face: f,
                    incoming: inc,
                },
                vertex: v,
                ring: lp.ring,
                angle: PI - turn,
            });
        }
    }
    Ok(out)
}

/// True if the coedge is a real (non-pole) edge use.
pub fn is_real(kind: CoedgeKind) -> bool {
    kind == CoedgeKind::Edge
}

/// The coedges of every loop of `f` (ring order, outer loop first), without pole coedges.
pub fn face_loops(t: &dyn TopologyView, f: EntityId) -> Vec<Vec<CoedgeKey>> {
    t.loops(f)
        .map(|lp| {
            lp.keys()
                .filter(|k| !t.flags(k.edge).contains(sm_topo_api::CellFlags::DEGENERATE))
                .collect()
        })
        .collect()
}

/// The interior angle (radians, in `(0, 2 pi)`) at one face corner.
pub fn corner_angle(t: &dyn TopologyView, key: CornerKey) -> Result<f64> {
    corners(t, key.face)?
        .into_iter()
        .find(|c| c.key == key)
        .map(|c| c.angle)
        .ok_or_else(|| SchemeError::Input(format!("{key:?} is not a corner of its face")))
}

/// Project `p` onto the carrier of face `f`. Returns the projected point and its surface
/// parameter; a face without a surface carrier returns `p` and [`Param::None`].
pub fn snap_to_face(t: &dyn TopologyView, f: EntityId, p: Point3) -> Result<(Point3, Param)> {
    if let Carrier::Surface(sc) = t.carrier(f)
        && let Some(s) = t.shapes().surface(sc.surface)
    {
        let pr = s.project(p, None)?;
        return Ok((pr.point, Param::Surface([pr.uv.u, pr.uv.v])));
    }
    Ok((p, Param::None))
}

/// The face uses of a volume piece: `(face, side)` per boundary incidence, in the topology's
/// canonical order.
pub fn volume_face_uses(t: &dyn TopologyView, v: EntityId) -> Vec<(EntityId, Side)> {
    t.boundary(v).iter().map(|i| (i.cell, i.side)).collect()
}
