//! The [`ParametricCurve`] contract.

use crate::analytic::AnalyticCurve;
use crate::vecs::{ParamRange, unit};
use crate::{GeomError, ModelTolerance};
use sm_core::{Point3, Vector3};

/// Coarse classification of a curve, for scheme dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum CurveKind {
    /// Straight line.
    Line,
    /// Circle.
    Circle,
    /// Ellipse.
    Ellipse,
    /// NURBS (possibly rational).
    Nurbs,
    /// Anything else (procedural, offset, intersection curve).
    Other,
}

/// Result of projecting a point onto a curve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurveProjection {
    /// Curve parameter of the closest point.
    pub t: f64,
    /// The closest point.
    pub point: Point3,
    /// Distance from the query point.
    pub distance: f64,
}

/// An exact curve `C(t)`. Object-safe: schemes take `&dyn ParametricCurve`.
///
/// Contract:
/// - `domain()` is the natural parameter range; lines are unbounded.
/// - `period()` is `Some(p)` for closed periodic curves. `C(t) == C(t + p)`. Such curves are
///   parametrized over exactly one period, `domain().len() == p`.
/// - `d1`/`d2` are derivatives w.r.t. the parameter (not arc length).
/// - All methods are pure and deterministic.
pub trait ParametricCurve: Send + Sync + core::fmt::Debug {
    /// Coarse kind.
    fn kind(&self) -> CurveKind;
    /// Natural parameter range.
    fn domain(&self) -> ParamRange;
    /// Period for periodic curves, else `None`.
    fn period(&self) -> Option<f64> {
        None
    }
    /// Point `C(t)`.
    fn eval(&self, t: f64) -> Point3;
    /// First derivative `C'(t)`.
    fn d1(&self, t: f64) -> Vector3;
    /// Second derivative `C''(t)`.
    fn d2(&self, t: f64) -> Vector3;
    /// Unit tangent; `Err(Singular)` where `C'(t) = 0`.
    fn tangent(&self, t: f64) -> Result<Vector3, GeomError> {
        unit(self.d1(t)).ok_or(GeomError::Singular { u: t, v: 0.0 })
    }
    /// Closest point on the (untrimmed) curve to `p`. `hint` seeds iterative solvers and picks the
    /// period branch for periodic curves. Ties are broken toward the smaller parameter.
    fn project(&self, p: Point3, hint: Option<f64>, tol: &ModelTolerance) -> Result<CurveProjection, GeomError>;
    /// Arc length between parameters `t0 <= t1`.
    fn length(&self, t0: f64, t1: f64, tol: &ModelTolerance) -> f64 {
        integrate_speed(self, t0, t1, tol)
    }
    /// True for closed periodic curves.
    fn is_closed(&self) -> bool {
        self.period().is_some()
    }
    /// The analytic description if this curve is one of the analytic kinds.
    fn analytic(&self) -> Option<AnalyticCurve> {
        None
    }
}

const GL5_X: [f64; 5] = [0.0, 0.538_469_310_105_683_1, -0.538_469_310_105_683_1, 0.906_179_845_938_664, -0.906_179_845_938_664];
const GL5_W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_08, 0.236_926_885_056_189_08];

/// Arc length by composite 5-point Gauss-Legendre with panel doubling until converged.
pub fn integrate_speed<C: ParametricCurve + ?Sized>(c: &C, t0: f64, t1: f64, _tol: &ModelTolerance) -> f64 {
    if t1 <= t0 {
        return 0.0;
    }
    let run = |n: usize| -> f64 {
        let h = (t1 - t0) / n as f64;
        let mut s = 0.0;
        for i in 0..n {
            let mid = t0 + (i as f64 + 0.5) * h;
            for k in 0..5 {
                s += GL5_W[k] * c.d1(mid + 0.5 * h * GL5_X[k]).norm();
            }
        }
        0.5 * h * s
    };
    let mut n = 8;
    let mut prev = run(n);
    while n < 8192 {
        n *= 2;
        let cur = run(n);
        if (cur - prev).abs() <= 1e-13 * cur.abs().max(1e-300) {
            return cur;
        }
        prev = cur;
    }
    prev
}

/// Generic closest-point solver for any curve: Newton on `(C(t)-p).C'(t) = 0` from a fixed set of
/// seeds (the hint, or an even sampling of a bounded domain). Deterministic. Analytic kinds
/// override with closed forms; custom curves may call this from `project`.
pub fn project_by_newton<C: ParametricCurve + ?Sized>(c: &C, p: Point3, hint: Option<f64>, tol: &ModelTolerance) -> Result<CurveProjection, GeomError> {
    let dom = c.domain();
    let mut seeds: Vec<f64> = Vec::new();
    if let Some(h) = hint {
        seeds.push(h);
    } else if dom.is_bounded() {
        seeds.extend((0..=16).map(|i| dom.lo + dom.len() * f64::from(i) / 16.0));
    } else {
        seeds.push(0.0);
    }
    let mut best: Option<CurveProjection> = None;
    for s in seeds {
        let mut t = s;
        let mut ok = false;
        for _ in 0..60 {
            let r = c.eval(t) - p;
            let (d1, d2) = (c.d1(t), c.d2(t));
            let g = r.dot(d1);
            let h = d1.dot(d1) + r.dot(d2);
            if h.abs() < 1e-300 {
                break;
            }
            let mut tn = t - g / h;
            if c.period().is_none() {
                tn = dom.clamp(tn);
            }
            let moved = (tn - t).abs();
            t = tn;
            if moved <= tol.param_rel * dom.len().clamp(1e-3, 1.0) * 1e-3 {
                ok = true;
                break;
            }
        }
        if !ok && (c.eval(t) - p).dot(c.d1(t)).abs() > 1e-9 * (1.0 + (c.eval(t) - p).norm()) {
            continue;
        }
        let q = c.eval(t);
        let cand = CurveProjection { t, point: q, distance: (q - p).norm() };
        if best.is_none_or(|b| cand.distance < b.distance - 1e-15 || (cand.distance <= b.distance + 1e-15 && cand.t < b.t)) {
            best = Some(cand);
        }
    }
    best.ok_or(GeomError::NoConvergence)
}
