//! The [`ParametricCurve`] trait.

use crate::{Domain1, GeomError, Interval, util};
use sm_core::{Point3, Vector3};

/// Position and first two derivatives at one parameter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurveJet {
    /// `C(t)`.
    pub p: Point3,
    /// `C'(t)`.
    pub d1: Vector3,
    /// `C''(t)`.
    pub d2: Vector3,
}

/// Result of projecting a point onto a curve.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct CurveProjection {
    /// Parameter of the closest point (principal range if periodic).
    pub t: f64,
    /// The closest point.
    pub point: Point3,
    /// Euclidean distance from the query point.
    pub distance: f64,
}

impl CurveProjection {
    /// Construct (the type is `#[non_exhaustive]`).
    pub const fn new(t: f64, point: Point3, distance: f64) -> Self {
        Self { t, point, distance }
    }
}

/// An exact parametric curve `C: domain -> R^3`.
///
/// Object-safe: schemes take `&dyn ParametricCurve`. Evaluation is infallible: outside the
/// domain an implementor evaluates the analytic continuation (periodic curves wrap); callers
/// who care check [`Domain1`]. Operations that can genuinely fail return [`GeomError`].
pub trait ParametricCurve: Send + Sync + core::fmt::Debug {
    /// Parameter domain, including periodicity.
    fn domain(&self) -> Domain1;

    /// Position, first and second derivative.
    fn jet(&self, t: f64) -> CurveJet;

    /// Position `C(t)`.
    fn eval(&self, t: f64) -> Point3 {
        self.jet(t).p
    }

    /// Evaluate at many parameters: `out[i] = eval(t[i])`. Panics if the lengths differ.
    ///
    /// Contract: an override (for example a parallel batch evaluator) is bit-identical to this
    /// default loop for any thread count (determinism rule). Schemes holding a
    /// `&dyn ParametricCurve` call this instead of looping over `eval`.
    fn eval_many(&self, t: &[f64], out: &mut [Point3]) {
        assert_eq!(t.len(), out.len(), "eval_many: length mismatch");
        for (o, t) in out.iter_mut().zip(t) {
            *o = self.eval(*t);
        }
    }

    /// First derivative `C'(t)`.
    fn d1(&self, t: f64) -> Vector3 {
        self.jet(t).d1
    }

    /// Unit tangent, or [`GeomError::Singular`] where `C'` vanishes.
    fn tangent(&self, t: f64) -> Result<Vector3, GeomError> {
        util::unit(self.d1(t)).ok_or(GeomError::Singular("zero tangent"))
    }

    /// Closest point on the curve (within the domain) to `p`. `hint` is a starting parameter
    /// for iterative implementors and may be ignored. Returns [`GeomError::Unsupported`] if the
    /// implementor has no projection.
    fn project(&self, p: Point3, hint: Option<f64>) -> Result<CurveProjection, GeomError>;

    /// Arc length over a bounded parameter `range` (`lo <= hi`), accurate to about `tol` in
    /// model length units. The default refines composite 5-point Gauss-Legendre on `|C'|` until
    /// two successive estimates agree to `tol` ([`GeomError::NoConvergence`] otherwise;
    /// [`GeomError::Invalid`] for an unbounded or reversed range). Line and circle override it
    /// with the exact value.
    fn length(&self, range: Interval, tol: f64) -> Result<f64, GeomError> {
        gauss_length(self, range, tol)
    }

    /// True if `C(lo) == C(hi)` within `tol` or the curve is periodic.
    fn is_closed(&self, tol: f64) -> bool {
        let d = self.domain();
        d.is_periodic() || (d.range.is_bounded() && (self.eval(d.range.lo) - self.eval(d.range.hi)).norm() <= tol)
    }
}

/// Adaptive composite 5-point Gauss-Legendre arc length of `|C'|` over `range`: the panel count
/// doubles (from 8, at most 4096 panels) until two successive estimates differ by at most `tol`.
pub fn gauss_length<C: ParametricCurve + ?Sized>(c: &C, range: Interval, tol: f64) -> Result<f64, GeomError> {
    const X: [f64; 5] = [0.0, 0.538_469_310_105_683_1, -0.538_469_310_105_683_1, 0.906_179_845_938_664, -0.906_179_845_938_664];
    const W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_1, 0.236_926_885_056_189_1];
    if !(range.is_valid() && range.is_bounded()) {
        return Err(GeomError::Invalid("arc length needs a bounded, ordered range"));
    }
    let estimate = |panels: usize| -> f64 {
        let h = range.len() / panels as f64;
        let mut sum = 0.0;
        for k in 0..panels {
            let mid = range.lo + (k as f64 + 0.5) * h;
            for (x, w) in X.iter().zip(W.iter()) {
                sum += w * c.d1(mid + 0.5 * h * x).norm();
            }
        }
        sum * 0.5 * h
    };
    let mut panels = 8;
    let mut prev = estimate(panels);
    while panels < 4096 {
        panels *= 2;
        let next = estimate(panels);
        if (next - prev).abs() <= tol {
            return Ok(next);
        }
        prev = next;
    }
    Err(GeomError::NoConvergence("arc length did not reach the requested tolerance"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Circle, Frame};

    #[test]
    fn eval_many_default_matches_eval() {
        let c = Circle::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 2.0).unwrap();
        let t: Vec<f64> = (0..10).map(|i| 0.7 * i as f64).collect();
        let mut out = vec![Point3::default(); t.len()];
        c.eval_many(&t, &mut out);
        assert!(t.iter().zip(&out).all(|(t, p)| *p == c.eval(*t)));
    }
}
