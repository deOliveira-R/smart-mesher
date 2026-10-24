//! Closest-point projection by branch and bound over Bezier pieces (convex-hull boxes give
//! lower bounds) finished by safeguarded Newton, plus adaptive Gauss quadrature.

use super::curve::{PreparedCurve, SegPoles, box_dist, hp_point};
use super::spline1::{Hp, bezier_split};
use core::cmp::Ordering;
use sm_core::Point3;
use sm_geom_api::{CurveProjection, GeomError, ParametricCurve};
use std::collections::BinaryHeap;

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
    let leaf = 1e-3 * scale;
    let mut best_t = c.range().lo;
    let mut best_d = f64::INFINITY;
    let mut consider = |t: f64, best_t: &mut f64, best_d: &mut f64| {
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
            store.push(Node { lo: s.lo, hi: s.hi, poles: s.poles.clone(), depth: 0 });
            heap.push((Key(lb, seq), store.len() - 1));
            seq += 1;
        }
    }
    let mut budget = 1usize << 16;
    while let Some((Key(lb, _), idx)) = heap.pop() {
        if lb >= best_d - prune || budget == 0 {
            break;
        }
        budget -= 1;
        let (lo, hi, depth) = (store[idx].lo, store[idx].hi, store[idx].depth);
        let b = node_box(&store[idx].poles);
        let diam = ((b[3] - b[0]).powi(2) + (b[4] - b[1]).powi(2) + (b[5] - b[2]).powi(2)).sqrt();
        if diam <= leaf || depth >= 48 {
            let t = newton_curve(c, q, 0.5 * (lo + hi));
            consider(t, &mut best_t, &mut best_d);
            continue;
        }
        let (l, r) = bezier_split(&store[idx].poles, 0.5);
        let mid = 0.5 * (lo + hi);
        for (pl, a, bnd) in [(l, lo, mid), (r, mid, hi)] {
            let bx = node_box(&pl);
            consider(0.5 * (a + bnd), &mut best_t, &mut best_d);
            let lb = box_dist(&bx, q);
            if lb < best_d - prune {
                store.push(Node { lo: a, hi: bnd, poles: pl.into_iter().collect(), depth: depth + 1 });
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
            let nd = (c.eval(nt) - q).norm();
            if nd <= d {
                let moved = (nt - t).abs();
                t = nt;
                d = nd;
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
    pu: usize,
    pv: usize,
    poles: PatchPoles,
    depth: u32,
}

/// Split a patch (poles `u`-major, `(pu+1) x (pv+1)`) in `u` (`along_u`) or `v` at the middle.
fn split_patch(poles: &[Hp], pu: usize, pv: usize, along_u: bool) -> (PatchPoles, PatchPoles) {
    let (mut l, mut r) = (PatchPoles::from_elem([0.0; 4], poles.len()), PatchPoles::from_elem([0.0; 4], poles.len()));
    let (nu, nv) = (pu + 1, pv + 1);
    if along_u {
        for j in 0..nv {
            let col: SmallVec<[Hp; 16]> = (0..nu).map(|i| poles[i * nv + j]).collect();
            let (a, b) = bezier_split(&col, 0.5);
            for i in 0..nu {
                l[i * nv + j] = a[i];
                r[i * nv + j] = b[i];
            }
        }
    } else {
        for i in 0..nu {
            let (a, b) = bezier_split(&poles[i * nv..(i + 1) * nv], 0.5);
            l[i * nv..(i + 1) * nv].copy_from_slice(&a);
            r[i * nv..(i + 1) * nv].copy_from_slice(&b);
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
    let leaf = 1e-3 * scale;
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
            store.push(PNode { u: p.u, v: p.v, pu, pv, poles: p.poles.clone(), depth: 0 });
            heap.push((Key(lb, seq), store.len() - 1));
            seq += 1;
        }
    }
    let mut budget = 1usize << 16;
    while let Some((Key(lb, _), idx)) = heap.pop() {
        if lb >= best_d - prune || budget == 0 {
            break;
        }
        budget -= 1;
        let bx = super::curve::bbox_of(store[idx].poles.iter().map(|h| hp_point(*h)));
        let (ex, ey, ez) = (bx[3] - bx[0], bx[4] - bx[1], bx[5] - bx[2]);
        let diam = (ex * ex + ey * ey + ez * ez).sqrt();
        let (u, v, depth) = (store[idx].u, store[idx].v, store[idx].depth);
        if diam <= leaf || depth >= 48 {
            let uv = newton_surface(s, q, Uv::new(0.5 * (u.lo + u.hi), 0.5 * (v.lo + v.hi)));
            consider(uv, &mut best, &mut best_d);
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
                store.push(PNode { u: cu, v: cv, pu, pv, poles: pl, depth: depth + 1 });
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

/// Safeguarded 2D Newton for the closest point of a surface (steps that do not reduce the
/// distance are halved; an indefinite Hessian falls back to steepest descent).
pub(crate) fn newton_surface<S: ParametricSurface + ?Sized>(s: &S, q: Point3, start: Uv) -> Uv {
    let dom = s.domain();
    let fix = |uv: Uv| {
        let f = |d: &sm_geom_api::Domain1, t: f64| if d.is_periodic() { d.wrap(t) } else { d.range.clamp(t) };
        Uv::new(f(&dom.u, uv.u), f(&dom.v, uv.v))
    };
    let mut x = fix(start);
    let mut d = (s.eval(x) - q).norm();
    for _ in 0..80 {
        let j = s.jet(x);
        let r = j.p - q;
        let (g0, g1) = (r.dot(j.du), r.dot(j.dv));
        let h00 = j.du.dot(j.du) + r.dot(j.duu);
        let h01 = j.du.dot(j.dv) + r.dot(j.duv);
        let h11 = j.dv.dot(j.dv) + r.dot(j.dvv);
        let det = h00 * h11 - h01 * h01;
        let (mut su, mut sv) = if det > 1e-300 && h00 > 0.0 { (-(h11 * g0 - h01 * g1) / det, -(h00 * g1 - h01 * g0) / det) } else { (-g0 / j.du.dot(j.du).max(1e-300), -g1 / j.dv.dot(j.dv).max(1e-300)) };
        let mut accepted = false;
        for _ in 0..30 {
            let nx = fix(Uv::new(x.u + su, x.v + sv));
            let nd = (s.eval(nx) - q).norm();
            if nd <= d {
                let moved = (nx.u - x.u).abs() + (nx.v - x.v).abs();
                x = nx;
                d = nd;
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
