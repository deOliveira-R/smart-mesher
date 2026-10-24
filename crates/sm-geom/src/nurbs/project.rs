//! Closest-point projection by branch and bound over Bezier pieces (convex-hull boxes give
//! lower bounds) finished by safeguarded Newton, plus adaptive Gauss quadrature.

use super::curve::{PreparedCurve, SegPoles, box_dist, hp_point};
use super::spline1::{Hp, split_half_into};
use core::cmp::Ordering;
use sm_core::Point3;
use sm_geom_api::{CurveProjection, GeomError, ParametricCurve};
use std::collections::BinaryHeap;

/// A piece whose box diameter is below this fraction of its distance to the query is finished
/// by Newton instead of being subdivided further.
const LEAF_RATIO: f64 = 0.05;

#[derive(Clone, Copy, PartialEq)]
struct Key(f64, u64);
impl Eq for Key {}
impl PartialOrd for Key {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Key {
    // Reverse: BinaryHeap is a max-heap and we want the smallest bound (then the earliest seq).
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.total_cmp(&self.0).then(o.1.cmp(&self.1))
    }
}

struct Node {
    lo: f64,
    hi: f64,
    poles: SegPoles,
    bbox: [f64; 6],
    depth: u32,
}

fn node_box(poles: &[Hp]) -> [f64; 6] {
    super::curve::bbox_of(poles.iter().map(|h| hp_point(*h)))
}

/// Closest point on a prepared curve.
pub(crate) fn project_curve(c: &PreparedCurve, q: Point3, hint: Option<f64>) -> CurveProjection {
    let scale = {
        let b = c.control_bbox();
        ((b[3] - b[0]).powi(2) + (b[4] - b[1]).powi(2) + (b[5] - b[2]).powi(2)).sqrt().max(f64::MIN_POSITIVE)
    };
    let prune = 1e-11 * scale;
    let leaf = 1e-4 * scale;
    let mut best_t = c.range().lo;
    let mut best_d = f64::INFINITY;
    let consider = |t: f64, best_t: &mut f64, best_d: &mut f64| {
        let d = (c.eval(t) - q).norm();
        if d < *best_d {
            *best_d = d;
            *best_t = t;
        }
    };
    let mut heap: BinaryHeap<(Key, usize)> = BinaryHeap::new();
    let mut store: Vec<Node> = Vec::new();
    let mut seq = 0u64;
    for s in &c.segs {
        consider(s.lo, &mut best_t, &mut best_d);
        consider(0.5 * (s.lo + s.hi), &mut best_t, &mut best_d);
        consider(s.hi, &mut best_t, &mut best_d);
    }
    if let Some(h) = hint {
        consider(c.fold(h), &mut best_t, &mut best_d);
    }
    for s in &c.segs {
        let lb = box_dist(&s.bbox, q);
        if lb < best_d - prune {
            store.push(Node { lo: s.lo, hi: s.hi, poles: s.poles.clone(), bbox: s.bbox, depth: 0 });
            heap.push((Key(lb, seq), store.len() - 1));
            seq += 1;
        }
    }
    let mut budget = 1usize << 16;
    let mut polishes = 0usize;
    while let Some((Key(lb, _), idx)) = heap.pop() {
        if lb >= best_d - prune || budget == 0 {
            break;
        }
        budget -= 1;
        let (lo, hi, depth) = (store[idx].lo, store[idx].hi, store[idx].depth);
        // A converged local minimum from the most promising node tightens the upper bound, which
        // then discards almost every other node at once.
        if polishes < 8 {
            polishes += 1;
            consider(newton_curve(c, q, 0.5 * (lo + hi)), &mut best_t, &mut best_d);
            if lb >= best_d - prune {
                continue;
            }
        }
        let b = &store[idx].bbox;
        let diam = ((b[3] - b[0]).powi(2) + (b[4] - b[1]).powi(2) + (b[5] - b[2]).powi(2)).sqrt();
        // A piece that is small against its distance from the query has one local minimum at
        // most for any reasonable curve: start Newton from three points and drop the piece.
        if diam <= leaf.max(LEAF_RATIO * lb) || depth >= 48 {
            // Start from the closest of a few samples, then one Newton run.
            let mut start = (0.5 * (lo + hi), f64::INFINITY);
            for f in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let t = lo + f * (hi - lo);
                let d = (c.eval(t) - q).norm();
                if d < start.1 {
                    start = (t, d);
                }
            }
            consider(newton_curve(c, q, start.0), &mut best_t, &mut best_d);
            continue;
        }
        let np = store[idx].poles.len();
        let mut l: SegPoles = SegPoles::from_elem([0.0; 4], np);
        let mut r: SegPoles = SegPoles::from_elem([0.0; 4], np);
        split_half_into(&store[idx].poles, &mut l, &mut r);
        let mid = 0.5 * (lo + hi);
        for (pl, a, bnd) in [(l, lo, mid), (r, mid, hi)] {
            let bx = node_box(&pl);
            consider(0.5 * (a + bnd), &mut best_t, &mut best_d);
            let lb = box_dist(&bx, q);
            if lb < best_d - prune {
                store.push(Node { lo: a, hi: bnd, poles: pl, bbox: bx, depth: depth + 1 });
                heap.push((Key(lb, seq), store.len() - 1));
                seq += 1;
            }
        }
    }
    let t = newton_curve(c, q, best_t);
    consider(t, &mut best_t, &mut best_d);
    let point = c.eval(best_t);
    CurveProjection::new(best_t, point, (point - q).norm())
}

/// Safeguarded Newton on `f(t) = (C - q) . C'`; steps that do not reduce the distance are
/// halved, and the iteration stops at the domain end for a non-periodic curve.
pub(crate) fn newton_curve<C: ParametricCurve + ?Sized>(c: &C, q: Point3, start: f64) -> f64 {
    let dom = c.domain();
    let fix = |t: f64| if dom.is_periodic() { dom.wrap(t) } else { dom.range.clamp(t) };
    let mut t = fix(start);
    let mut d = (c.eval(t) - q).norm();
    for _ in 0..60 {
        let j = c.jet(t);
        let r = j.p - q;
        let g = r.dot(j.d1);
        let mut h = j.d1.dot(j.d1) + r.dot(j.d2);
        if h <= 1e-300 {
            h = j.d1.dot(j.d1).max(1e-300);
        }
        let mut step = -g / h;
        let mut accepted = false;
        for _ in 0..30 {
            let nt = fix(t + step);
            let jn = c.jet(nt);
            let nd = (jn.p - q).norm();
            // Far from the solution the distance must fall; near it the distance is flat to
            // rounding, so a smaller stationarity residual is the progress measure.
            let g_new = (jn.p - q).dot(jn.d1);
            if nd < d || (g_new.abs() < g.abs() && nd <= d * (1.0 + 1e-14)) {
                let moved = (nt - t).abs();
                t = nt;
                d = nd.min(d);
                accepted = true;
                if moved <= 1e-15 * (1.0 + t.abs()) {
                    return t;
                }
                break;
            }
            step *= 0.5;
        }
        if !accepted {
            break;
        }
    }
    t
}

/// 5-point Gauss-Legendre on `[a, b]` with panel doubling until two estimates agree to `tol`.
pub(crate) fn gauss_adaptive(f: &dyn Fn(f64) -> f64, a: f64, b: f64, tol: f64) -> Result<f64, GeomError> {
    const X: [f64; 5] = [0.0, 0.538_469_310_105_683_1, -0.538_469_310_105_683_1, 0.906_179_845_938_664, -0.906_179_845_938_664];
    const W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_1, 0.236_926_885_056_189_1];
    if b <= a {
        return Ok(0.0);
    }
    let est = |panels: usize| -> f64 {
        let h = (b - a) / panels as f64;
        let mut sum = 0.0;
        for k in 0..panels {
            let mid = a + (k as f64 + 0.5) * h;
            for (x, w) in X.iter().zip(W.iter()) {
                sum += w * f(mid + 0.5 * h * x);
            }
        }
        sum * 0.5 * h
    };
    let mut panels = 2;
    let mut prev = est(panels);
    while panels < 4096 {
        panels *= 2;
        let next = est(panels);
        if (next - prev).abs() <= tol.max(1e-15 * next.abs()) {
            return Ok(next);
        }
        prev = next;
    }
    Err(GeomError::NoConvergence("arc length did not reach the requested tolerance"))
}

// ------------------------------------------------------------------ surfaces

use super::surface::{PatchPoles, PreparedSurface};
use sm_geom_api::{Interval, ParametricSurface, SurfaceProjection, Uv};

struct PNode {
    u: Interval,
    v: Interval,
    poles: PatchPoles,
    bbox: [f64; 6],
    depth: u32,
}

/// Split a patch (poles `u`-major, `(pu+1) x (pv+1)`) in `u` (`along_u`) or `v` at the middle.
pub(crate) fn split_patch(poles: &[Hp], pu: usize, pv: usize, along_u: bool) -> (PatchPoles, PatchPoles) {
    let (mut l, mut r) = (PatchPoles::from_elem([0.0; 4], poles.len()), PatchPoles::from_elem([0.0; 4], poles.len()));
    let (nu, nv) = (pu + 1, pv + 1);
    if along_u {
        let mut col = [[0.0; 4]; 16];
        let (mut cl, mut cr) = ([[0.0; 4]; 16], [[0.0; 4]; 16]);
        for j in 0..nv {
            for i in 0..nu {
                col[i] = poles[i * nv + j];
            }
            split_half_into(&col[..nu], &mut cl[..nu], &mut cr[..nu]);
            for i in 0..nu {
                l[i * nv + j] = cl[i];
                r[i * nv + j] = cr[i];
            }
        }
    } else {
        for i in 0..nu {
            split_half_into(&poles[i * nv..(i + 1) * nv], &mut l[i * nv..(i + 1) * nv], &mut r[i * nv..(i + 1) * nv]);
        }
    }
    (l, r)
}

use smallvec::SmallVec;

/// Closest point on a prepared surface (branch and bound on Bezier patches, then Newton).
pub(crate) fn project_surface(s: &PreparedSurface, q: Point3, hint: Option<Uv>) -> SurfaceProjection {
    let b = s.control_bbox();
    let scale = ((b[3] - b[0]).powi(2) + (b[4] - b[1]).powi(2) + (b[5] - b[2]).powi(2)).sqrt().max(f64::MIN_POSITIVE);
    let prune = 1e-11 * scale;
    let leaf = 1e-4 * scale;
    let (pu, pv) = s.degrees();
    let mut best = Uv::new(s.ranges().0.lo, s.ranges().1.lo);
    let mut best_d = f64::INFINITY;
    let consider = |uv: Uv, best: &mut Uv, best_d: &mut f64| {
        let d = (s.eval(uv) - q).norm();
        if d < *best_d {
            *best_d = d;
            *best = uv;
        }
    };
    for p in &s.patches {
        for (u, v) in [(p.u.lo, p.v.lo), (p.u.hi, p.v.lo), (p.u.lo, p.v.hi), (p.u.hi, p.v.hi), (0.5 * (p.u.lo + p.u.hi), 0.5 * (p.v.lo + p.v.hi))] {
            consider(Uv::new(u, v), &mut best, &mut best_d);
        }
    }
    if let Some(h) = hint {
        consider(s.fold(h), &mut best, &mut best_d);
    }
    let mut heap: BinaryHeap<(Key, usize)> = BinaryHeap::new();
    let mut store: Vec<PNode> = Vec::new();
    let mut seq = 0u64;
    for p in &s.patches {
        let lb = box_dist(&p.bbox, q);
        if lb < best_d - prune {
            store.push(PNode { u: p.u, v: p.v, poles: p.poles.clone(), bbox: p.bbox, depth: 0 });
            heap.push((Key(lb, seq), store.len() - 1));
            seq += 1;
        }
    }
    let mut budget = 1usize << 16;
    let mut polishes = 0usize;
    while let Some((Key(lb, _), idx)) = heap.pop() {
        if lb >= best_d - prune || budget == 0 {
            break;
        }
        budget -= 1;
        let (u, v, depth) = (store[idx].u, store[idx].v, store[idx].depth);
        if polishes < 8 {
            polishes += 1;
            consider(newton_surface(s, q, Uv::new(0.5 * (u.lo + u.hi), 0.5 * (v.lo + v.hi))), &mut best, &mut best_d);
            if lb >= best_d - prune {
                continue;
            }
        }
        let bx = &store[idx].bbox;
        let (ex, ey, ez) = (bx[3] - bx[0], bx[4] - bx[1], bx[5] - bx[2]);
        let diam = (ex * ex + ey * ey + ez * ez).sqrt();
        if diam <= leaf.max(LEAF_RATIO * lb) || depth >= 48 {
            // Start from the closest of a 3 x 3 sample grid, then one Newton run.
            let mut start = (Uv::new(0.5 * (u.lo + u.hi), 0.5 * (v.lo + v.hi)), f64::INFINITY);
            for fu in [0.0, 0.5, 1.0] {
                for fv in [0.0, 0.5, 1.0] {
                    let uv = Uv::new(u.lo + fu * (u.hi - u.lo), v.lo + fv * (v.hi - v.lo));
                    let d = (s.eval(uv) - q).norm();
                    if d < start.1 {
                        start = (uv, d);
                    }
                }
            }
            consider(newton_surface(s, q, start.0), &mut best, &mut best_d);
            continue;
        }
        // Split the direction whose parameter interval is wider relative to the net extent.
        let along_u = depth % 2 == 0;
        let (l, r) = split_patch(&store[idx].poles, pu, pv, along_u);
        let halves = if along_u {
            let m = 0.5 * (u.lo + u.hi);
            [(l, Interval::new(u.lo, m), v), (r, Interval::new(m, u.hi), v)]
        } else {
            let m = 0.5 * (v.lo + v.hi);
            [(l, u, Interval::new(v.lo, m)), (r, u, Interval::new(m, v.hi))]
        };
        for (pl, cu, cv) in halves {
            let bx = super::curve::bbox_of(pl.iter().map(|h| hp_point(*h)));
            consider(Uv::new(0.5 * (cu.lo + cu.hi), 0.5 * (cv.lo + cv.hi)), &mut best, &mut best_d);
            let lb = box_dist(&bx, q);
            if lb < best_d - prune {
                store.push(PNode { u: cu, v: cv, poles: pl, bbox: bx, depth: depth + 1 });
                heap.push((Key(lb, seq), store.len() - 1));
                seq += 1;
            }
        }
    }
    let uv = newton_surface(s, q, best);
    consider(uv, &mut best, &mut best_d);
    let point = s.eval(best);
    SurfaceProjection::new(best, point, (point - q).norm())
}

/// Safeguarded 2D Newton for the closest point of a surface. At a non-periodic parameter bound
/// whose descent direction points outward the parameter is held (active set) and the other
/// one is solved alone; steps that do not reduce the distance are halved, and an indefinite
/// Hessian falls back to steepest descent.
pub(crate) fn newton_surface<S: ParametricSurface + ?Sized>(s: &S, q: Point3, start: Uv) -> Uv {
    let dom = s.domain();
    let fix = |uv: Uv| {
        let f = |d: &sm_geom_api::Domain1, t: f64| if d.is_periodic() { d.wrap(t) } else { d.range.clamp(t) };
        Uv::new(f(&dom.u, uv.u), f(&dom.v, uv.v))
    };
    let at_bound = |d: &sm_geom_api::Domain1, x: f64, g: f64| -> bool {
        if d.is_periodic() {
            return false;
        }
        let eps = 1e-14 * (1.0 + x.abs());
        (d.range.lo.is_finite() && x <= d.range.lo + eps && g > 0.0) || (d.range.hi.is_finite() && x >= d.range.hi - eps && g < 0.0)
    };
    let mut x = fix(start);
    let mut d = (s.eval(x) - q).norm();
    for _ in 0..100 {
        let j = s.jet(x);
        let r = j.p - q;
        let (g0, g1) = (r.dot(j.du), r.dot(j.dv));
        let h00 = j.du.dot(j.du) + r.dot(j.duu);
        let h01 = j.du.dot(j.dv) + r.dot(j.duv);
        let h11 = j.dv.dot(j.dv) + r.dot(j.dvv);
        let (act_u, act_v) = (at_bound(&dom.u, x.u, g0), at_bound(&dom.v, x.v, g1));
        if act_u && act_v {
            return x;
        }
        let (mut su, mut sv) = if act_u {
            (0.0, -g1 / if h11 > 1e-300 { h11 } else { j.dv.dot(j.dv).max(1e-300) })
        } else if act_v {
            (-g0 / if h00 > 1e-300 { h00 } else { j.du.dot(j.du).max(1e-300) }, 0.0)
        } else {
            let det = h00 * h11 - h01 * h01;
            if det > 1e-300 && h00 > 0.0 { (-(h11 * g0 - h01 * g1) / det, -(h00 * g1 - h01 * g0) / det) } else { (-g0 / j.du.dot(j.du).max(1e-300), -g1 / j.dv.dot(j.dv).max(1e-300)) }
        };
        let mut accepted = false;
        for _ in 0..40 {
            let nx = fix(Uv::new(x.u + su, x.v + sv));
            let jn = s.jet(nx);
            let nd = (jn.p - q).norm();
            let gn = ((jn.p - q).dot(jn.du)).abs().max(((jn.p - q).dot(jn.dv)).abs());
            let g_old = g0.abs().max(g1.abs());
            if nd < d || (gn < g_old && nd <= d * (1.0 + 1e-14)) {
                let moved = (nx.u - x.u).abs() + (nx.v - x.v).abs();
                x = nx;
                d = nd.min(d);
                accepted = true;
                if moved <= 1e-15 * (1.0 + x.u.abs() + x.v.abs()) {
                    return x;
                }
                break;
            }
            su *= 0.5;
            sv *= 0.5;
        }
        if !accepted {
            break;
        }
    }
    x
}
