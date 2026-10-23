//! The [`ParametricCurve`] trait and generic numeric helpers.

use crate::frame::dist;
use crate::tol::Interval;
use core::fmt::Debug;
use sm_core::{Point3, Vector3};

/// Closed set of curve kinds (shape-identification for schemes and exporters).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum CurveKind {
    /// Straight line.
    Line,
    /// Circle.
    Circle,
    /// Ellipse.
    Ellipse,
    /// Rational B-spline.
    Nurbs,
}

/// Point and first two derivatives at a parameter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurveDerivs {
    /// `C(t)`.
    pub p: Point3,
    /// `C'(t)`.
    pub d1: Vector3,
    /// `C''(t)`.
    pub d2: Vector3,
}

/// Result of a closest-point query.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurveProjection {
    /// Parameter of the closest point (within the queried interval).
    pub t: f64,
    /// The closest point `C(t)`.
    pub point: Point3,
    /// Distance from the query point.
    pub distance: f64,
}

/// An exact parametric curve `C: [lo, hi] -> R^3`. Object-safe; `Send + Sync`.
///
/// Parameter speed is not normalized. Periodic curves report their period and accept any
/// parameter; non-periodic curves are evaluated by extension where defined (lines) and
/// otherwise clamp to the natural range.
pub trait ParametricCurve: Send + Sync + Debug {
    /// Which kind of curve this is.
    fn kind(&self) -> CurveKind;
    /// Natural parameter range (infinite for lines; one period for closed analytic curves).
    fn range(&self) -> Interval;
    /// Period if the curve is periodic (`C(t + period) = C(t)`).
    fn period(&self) -> Option<f64>;
    /// `C(t)`.
    fn eval(&self, t: f64) -> Point3;
    /// `C'(t)`.
    fn d1(&self, t: f64) -> Vector3;
    /// `C''(t)`.
    fn d2(&self, t: f64) -> Vector3;

    /// Point and first two derivatives together (override when sharing work).
    fn derivs(&self, t: f64) -> CurveDerivs {
        CurveDerivs { p: self.eval(t), d1: self.d1(t), d2: self.d2(t) }
    }

    /// True if the curve closes on itself over its natural range.
    fn is_closed(&self) -> bool {
        let r = self.range();
        self.period().is_some() || (r.is_finite() && dist(self.eval(r.lo), self.eval(r.hi)) < 1e-12)
    }

    /// Closest point to `p` among parameters in `within` (finite for non-lines).
    /// Ties resolve to the smaller parameter, so results are deterministic.
    fn closest_in(&self, p: Point3, within: Interval) -> CurveProjection;

    /// Closest point over the natural range.
    fn closest(&self, p: Point3) -> CurveProjection {
        self.closest_in(p, self.range())
    }

    /// Arc length over `[a, b]` to absolute accuracy `tol`.
    fn length(&self, a: f64, b: f64, tol: f64) -> f64 {
        integrate_speed(self, a, b, tol)
    }
}

const GL_X: [f64; 5] = [0.0, 0.538_469_310_105_683_1, -0.538_469_310_105_683_1, 0.906_179_845_938_664, -0.906_179_845_938_664];
const GL_W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_1, 0.236_926_885_056_189_1];

fn gl5<C: ParametricCurve + ?Sized>(c: &C, a: f64, b: f64) -> f64 {
    let (h, m) = (0.5 * (b - a), 0.5 * (a + b));
    let s: f64 = GL_X.iter().zip(GL_W.iter()).map(|(x, w)| w * c.d1(m + h * x).norm()).sum();
    s * h
}

fn adaptive<C: ParametricCurve + ?Sized>(c: &C, a: f64, b: f64, whole: f64, tol: f64, depth: u32) -> f64 {
    let m = 0.5 * (a + b);
    let (l, r) = (gl5(c, a, m), gl5(c, m, b));
    if depth == 0 || (l + r - whole).abs() <= tol {
        l + r
    } else {
        adaptive(c, a, m, l, 0.5 * tol, depth - 1) + adaptive(c, m, b, r, 0.5 * tol, depth - 1)
    }
}

/// Arc length by adaptive 5-point Gauss-Legendre over `|C'|` (finite `[a, b]`).
pub fn integrate_speed<C: ParametricCurve + ?Sized>(c: &C, a: f64, b: f64, tol: f64) -> f64 {
    let sign = if b < a { -1.0 } else { 1.0 };
    let (a, b) = if b < a { (b, a) } else { (a, b) };
    // Start from 8 panels so smooth-but-oscillatory speeds are not missed.
    let n = 8;
    let h = (b - a) / n as f64;
    let mut total = 0.0;
    for i in 0..n {
        let (x0, x1) = (a + h * i as f64, a + h * (i + 1) as f64);
        total += adaptive(c, x0, x1, gl5(c, x0, x1), tol / n as f64, 24);
    }
    sign * total
}

/// Generic closest-point: uniform sampling for a seed, then Newton on `(C - p) . C'`.
/// `within` must be finite (infinite bounds are clamped to `+-1e6`).
pub fn closest_generic<C: ParametricCurve + ?Sized>(c: &C, p: Point3, within: Interval, samples: usize) -> CurveProjection {
    let lo = within.lo.max(-1e6);
    let hi = within.hi.min(1e6);
    let n = samples.max(8);
    let eval_at = |t: f64| {
        let q = c.eval(t);
        CurveProjection { t, point: q, distance: dist(q, p) }
    };
    let mut best = eval_at(lo);
    for i in 0..=n {
        let t = lo + (hi - lo) * (i as f64 / n as f64);
        let cand = eval_at(t);
        if cand.distance < best.distance {
            best = cand;
        }
    }
    let mut t = best.t;
    for _ in 0..60 {
        let d = c.derivs(t);
        let r = d.p - p;
        let g = r.dot(d.d1);
        let h = d.d1.dot(d.d1) + r.dot(d.d2);
        if h.abs() < 1e-300 {
            break;
        }
        let nt = (t - g / h).max(lo).min(hi);
        let done = (nt - t).abs() <= 1e-15 * (1.0 + t.abs());
        t = nt;
        if done {
            break;
        }
    }
    let refined = eval_at(t);
    if refined.distance <= best.distance { refined } else { best }
}
