//! Pcurves of the coedges the kernel creates or moves.
//!
//! A coedge's pcurve is its edge curve in the face's `(u, v)` space, parametrized by the edge
//! curve parameter (that is how schemes read it: `pcurve.eval(edge_param)`). After a split or a
//! join the loops of a face on a periodic or pole-carrying surface need pcurves that are
//! continuous along each loop on one unrolled branch. This pass walks every loop of a face,
//! shifts existing pcurves by whole periods so they chain, and builds the missing ones as exact
//! `Line2` where the curve is an iso-line or an affine image of one (circles about a cylinder or
//! sphere axis, generators, meridians). A curve whose `(u, v)` is not affine in its parameter
//! gets no pcurve (a gap documented in `W3_REPORT.md`).

use sm_core::EntityId;
use sm_geom_api::{Interval, Line2, PCurve, ParametricSurface, SurfaceDef, Uv};
use sm_topo_api::{Carrier, CoedgeKey, Sense, Side, TopologyEdit, TopologyView, TopologyViewExt};

use crate::error::BooleanError;
use crate::geo;

fn shifted(pc: &PCurve, du: f64, dv: f64) -> PCurve {
    match pc {
        PCurve::Line2(l) => PCurve::Line2(Line2 { origin: Uv::new(l.origin.u + du, l.origin.v + dv), ..*l }),
        PCurve::Polyline2(p) => PCurve::Polyline2(p.iter().map(|q| Uv::new(q.u + du, q.v + dv)).collect()),
        PCurve::Nurbs2(n) => {
            let mut m = n.clone();
            for c in &mut m.control {
                c.u += du;
                c.v += dv;
            }
            PCurve::Nurbs2(m)
        }
        other => other.clone(),
    }
}

fn period_shift(period: Option<f64>, from: f64, to: f64) -> f64 {
    match period {
        Some(p) => p * ((to - from) / p).round(),
        None => 0.0,
    }
}

/// Make the pcurves of every loop of `face` continuous and fill the ones that can be exact.
pub(crate) fn fix_face<E: TopologyEdit + ?Sized>(topo: &mut E, face: EntityId) -> Result<(), BooleanError> {
    if !topo.is_alive(face) {
        return Ok(());
    }
    let Carrier::Surface(sc) = topo.carrier(face).clone() else { return Ok(()) };
    let Some(sdef) = topo.shapes().surface_def(sc.surface).cloned() else { return Ok(()) };
    if matches!(sdef, SurfaceDef::Plane(_)) {
        return Ok(());
    }
    let Some(surf) = sdef.as_surface() else { return Ok(()) };
    let dom = surf.domain();
    let (pu, pv) = (dom.u.period, dom.v.period);
    let loops: Vec<Vec<CoedgeKey>> = topo.loops(face).map(|l| l.keys().collect()).collect();
    for keys in loops {
        let mut cur: Option<Uv> = None;
        for key in keys {
            let deg = geo::is_degenerate(&*topo, key.edge);
            if deg {
                let Some(pc) = topo.pcurve(key).cloned() else { continue };
                let rg = pc.range();
                let (s, e) = if key.side == Side::Left { (pc.eval(rg.lo), pc.eval(rg.hi)) } else { (pc.eval(rg.hi), pc.eval(rg.lo)) };
                let (du, dv) = cur.map_or((0.0, 0.0), |c| (period_shift(pu, s.u, c.u), period_shift(pv, s.v, c.v)));
                if du != 0.0 || dv != 0.0 {
                    topo.set_pcurve(key, Some(shifted(&pc, du, dv)))?;
                }
                cur = Some(Uv::new(e.u + du, e.v + dv));
                continue;
            }
            let Carrier::Curve(cc) = topo.carrier(key.edge).clone() else { continue };
            let Some(cdef) = topo.shapes().curve_def(cc.curve).cloned() else { continue };
            let curve = cdef.as_curve();
            let along = (cc.sense == Sense::Forward) == (key.side == Side::Left);
            let (t_start, t_end) = if along { (cc.range.lo, cc.range.hi) } else { (cc.range.hi, cc.range.lo) };
            if let Some(pc) = topo.pcurve(key).cloned() {
                let (s, e) = (pc.eval(t_start), pc.eval(t_end));
                let (du, dv) = cur.map_or((0.0, 0.0), |c| (period_shift(pu, s.u, c.u), period_shift(pv, s.v, c.v)));
                if du != 0.0 || dv != 0.0 {
                    topo.set_pcurve(key, Some(shifted(&pc, du, dv)))?;
                }
                cur = Some(Uv::new(e.u + du, e.v + dv));
                continue;
            }
            // Sample the coedge in traversal order, unrolled from `cur`.
            const N: usize = 12;
            let mut pts: Vec<(f64, Uv)> = Vec::with_capacity(N + 1);
            let mut prev = cur;
            for k in 0..=N {
                let t = t_start + (t_end - t_start) * (k as f64 / N as f64);
                let pr = surf.project(curve.eval(t), prev)?;
                let at_pole = surf.jet(pr.uv).du.norm() < 1e-9;
                let u = if at_pole { prev.map_or(f64::NAN, |p| p.u) } else { dom.u.nearest_to(pr.uv.u, prev.map_or(pr.uv.u, |p| p.u)) };
                let v = dom.v.nearest_to(pr.uv.v, prev.map_or(pr.uv.v, |p| p.v));
                let uv = Uv::new(u, v);
                pts.push((t, uv));
                prev = if u.is_nan() { prev } else { Some(uv) };
            }
            // A start at a pole takes the longitude of the first sample after it.
            if pts[0].1.u.is_nan() {
                let first = pts.iter().find(|p| !p.1.u.is_nan()).map(|p| p.1.u);
                if let Some(u) = first {
                    for p in pts.iter_mut().take_while(|p| p.1.u.is_nan()) {
                        p.1.u = u;
                    }
                }
            }
            if pts.iter().any(|p| p.1.u.is_nan()) {
                continue;
            }
            // Re-anchor the whole run to `cur` (the first sample may have been unrolled from
            // the principal range when `cur` was unknown).
            if let Some(c) = cur {
                let du = period_shift(pu, pts[0].1.u, c.u);
                let dv = period_shift(pv, pts[0].1.v, c.v);
                for p in &mut pts {
                    p.1 = Uv::new(p.1.u + du, p.1.v + dv);
                }
            }
            let (t0, uv0) = pts[0];
            let (t1, uv1) = pts[N];
            let dt = t1 - t0;
            cur = Some(uv1);
            if dt == 0.0 {
                continue;
            }
            let dir = Uv::new((uv1.u - uv0.u) / dt, (uv1.v - uv0.v) / dt);
            let scale = 1.0 + uv0.u.abs() + uv0.v.abs() + uv1.u.abs() + uv1.v.abs();
            let affine = pts.iter().all(|&(t, uv)| {
                let want = Uv::new(uv0.u + dir.u * (t - t0), uv0.v + dir.v * (t - t0));
                (uv.u - want.u).abs() <= 1e-9 * scale && (uv.v - want.v).abs() <= 1e-9 * scale
            });
            if affine {
                let origin = Uv::new(uv0.u - dir.u * t0, uv0.v - dir.v * t0);
                let pc = PCurve::Line2(Line2 { origin, dir, range: Interval::new(cc.range.lo, cc.range.hi) });
                topo.set_pcurve(key, Some(pc))?;
            }
        }
    }
    Ok(())
}

/// [`fix_face`] on every face of `faces`.
pub(crate) fn fix_faces<E: TopologyEdit + ?Sized>(topo: &mut E, faces: &[EntityId]) -> Result<(), BooleanError> {
    for &f in faces {
        fix_face(topo, f)?;
    }
    Ok(())
}

#[allow(dead_code)]
fn unused(_: &dyn ParametricSurface) {}
