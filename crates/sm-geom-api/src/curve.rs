//! The [`ParametricCurve`] trait.

use crate::{Domain1, GeomError, util};
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
pub struct CurveProjection {
    /// Parameter of the closest point (principal range if periodic).
    pub t: f64,
    /// The closest point.
    pub point: Point3,
    /// Euclidean distance from the query point.
    pub distance: f64,
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

    /// Arc length between parameters `a` and `b` (signed: negative if `b < a`).
    /// The default uses composite 5-point Gauss-Legendre on `|C'|`.
    fn length(&self, a: f64, b: f64) -> f64 {
        gauss_length(self, a, b)
    }

    /// True if `C(lo) == C(hi)` within `tol` or the curve is periodic.
    fn is_closed(&self, tol: f64) -> bool {
        let d = self.domain();
        d.is_periodic() || (d.range.is_bounded() && (self.eval(d.range.lo) - self.eval(d.range.hi)).norm() <= tol)
    }
}

/// Composite 5-point Gauss-Legendre arc length of `|C'|` over `[a, b]` (32 panels).
pub fn gauss_length<C: ParametricCurve + ?Sized>(c: &C, a: f64, b: f64) -> f64 {
    const X: [f64; 5] = [0.0, 0.538_469_310_105_683_1, -0.538_469_310_105_683_1, 0.906_179_845_938_664, -0.906_179_845_938_664];
    const W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_1, 0.236_926_885_056_189_1];
    const PANELS: usize = 32;
    let h = (b - a) / PANELS as f64;
    let mut sum = 0.0;
    for k in 0..PANELS {
        let mid = a + (k as f64 + 0.5) * h;
        for (x, w) in X.iter().zip(W.iter()) {
            sum += w * c.d1(mid + 0.5 * h * x).norm();
        }
    }
    sum * 0.5 * h
}
