//! Point-in-face classification that also works for faces with pole coedges.
//!
//! `TopologyView::classify_point_on_face` samples the loop by projection only and refuses a loop
//! that does not close in parameter space, which is what a pole (degenerate) coedge produces.
//! This module builds the same parameter-space polygon but reads a coedge's pcurve when it has
//! one (the pole coedge's pcurve carries the `u` jump) and projects otherwise, then runs the
//! same winding-number test with the periodic images of the query point. `On` is decided first
//! by the exact distance to the boundary curves.

use sm_core::{EntityId, Point3};
use sm_geom_api::{Domain2, Uv};
use sm_topo_api::{Carrier, CellFlags, PointClass, Side, TopologyView, TopologyViewExt};

use crate::error::BooleanError;
use crate::geo;

/// Samples per non-degenerate coedge of the parameter-space polygon.
const SAMPLES: usize = 32;

/// The parameter-space polygon of every loop of `f`, and the surface domain.
pub(crate) fn polygons<V: TopologyView + ?Sized>(view: &V, f: EntityId) -> Result<(Vec<Vec<Uv>>, Domain2), BooleanError> {
    let Some((surf, _)) = geo::face_surface(view, f) else { return Err(BooleanError::Unsupported("a face has no surface")) };
    let dom = surf.domain();
    let mut out = Vec::new();
    for lp in view.loops(f) {
        let mut poly: Vec<Uv> = Vec::new();
        let mut prev: Option<Uv> = None;
        for key in lp.keys() {
            let deg = view.flags(key.edge).contains(CellFlags::DEGENERATE);
            let pc = view.pcurve(key);
            if deg {
                if let Some(pc) = pc {
                    let rg = pc.range();
                    let (a, b) = if key.side == Side::Left { (pc.eval(rg.lo), pc.eval(rg.hi)) } else { (pc.eval(rg.hi), pc.eval(rg.lo)) };
                    poly.push(a);
                    poly.push(b);
                    prev = Some(b);
                }
                continue;
            }
            let Carrier::Curve(cc) = view.carrier(key.edge) else { return Err(BooleanError::Unsupported("an edge has no curve")) };
            let Some(curve) = view.shapes().curve(cc.curve) else { return Err(BooleanError::Unsupported("curve not in store")) };
            let along = (cc.sense == sm_topo_api::Sense::Forward) == (key.side == Side::Left);
            for k in 0..=SAMPLES {
                let frac = k as f64 / SAMPLES as f64;
                let t = geo::edge_param(&cc, if along { frac } else { 1.0 - frac });
                let uv = match pc {
                    Some(pc) => pc.eval(t),
                    None => {
                        let pr = surf.project(curve.eval(t), prev)?;
                        Uv::new(dom.u.nearest_to(pr.uv.u, prev.map_or(pr.uv.u, |p| p.u)), dom.v.nearest_to(pr.uv.v, prev.map_or(pr.uv.v, |p| p.v)))
                    }
                };
                poly.push(uv);
                prev = Some(uv);
            }
        }
        out.push(poly);
    }
    Ok((out, dom))
}

/// Winding-number test of `q` against the polygons (all loops together), trying the periodic
/// images of `q`.
fn inside(polys: &[Vec<Uv>], dom: &Domain2, q0: Uv) -> bool {
    let shifts = |d: &sm_geom_api::Domain1| match d.period {
        Some(p) => vec![-p, 0.0, p],
        None => vec![0.0],
    };
    for du in shifts(&dom.u) {
        for dv in shifts(&dom.v) {
            let q = Uv::new(q0.u + du, q0.v + dv);
            let mut winding = 0i64;
            for lp in polys {
                let n = lp.len();
                for k in 0..n {
                    let (a, b) = (lp[k], lp[(k + 1) % n]);
                    let cross = (b.u - a.u) * (q.v - a.v) - (q.u - a.u) * (b.v - a.v);
                    if a.v <= q.v {
                        if b.v > q.v && cross > 0.0 {
                            winding += 1;
                        }
                    } else if b.v <= q.v && cross < 0.0 {
                        winding -= 1;
                    }
                }
            }
            if winding != 0 {
                return true;
            }
        }
    }
    false
}

/// Distance from `p` to the boundary curves of `f` (restricted to the edge ranges).
pub(crate) fn boundary_distance<V: TopologyView + ?Sized>(view: &V, f: EntityId, p: Point3) -> f64 {
    let mut best = f64::INFINITY;
    for &e in view.face_edges(f) {
        if geo::is_degenerate(view, e) {
            if let Some((s, _)) = view.edge_ends(e)
                && let Some(q) = geo::vertex_point(view, s)
            {
                best = best.min(geo::dist(p, q));
            }
            continue;
        }
        if let Some((c, cc)) = geo::edge_curve(view, e)
            && let Some((d, _)) = crate::cc::dist_to_range(c, cc.range, p)
        {
            best = best.min(d);
        }
    }
    best
}

/// Classify the point `p` (assumed to lie on the surface of `f`; it is projected onto it) against
/// the trimmed face: `On` within `tol` of the boundary, else `In` or `Out`.
pub(crate) fn classify<V: TopologyView + ?Sized>(view: &V, f: EntityId, p: Point3, tol: f64) -> Result<(PointClass, Uv), BooleanError> {
    let Some((surf, _)) = geo::face_surface(view, f) else { return Err(BooleanError::Unsupported("a face has no surface")) };
    let pr = surf.project(p, None)?;
    classify_uv(view, f, pr.uv, tol)
}

/// As [`classify`], from parameters.
pub(crate) fn classify_uv<V: TopologyView + ?Sized>(view: &V, f: EntityId, uv: Uv, tol: f64) -> Result<(PointClass, Uv), BooleanError> {
    let Some((surf, _)) = geo::face_surface(view, f) else { return Err(BooleanError::Unsupported("a face has no surface")) };
    let p3 = surf.eval(uv);
    if boundary_distance(view, f, p3) <= tol {
        return Ok((PointClass::On, uv));
    }
    let (polys, dom) = polygons(view, f)?;
    Ok((if inside(&polys, &dom, uv) { PointClass::In } else { PointClass::Out }, uv))
}
