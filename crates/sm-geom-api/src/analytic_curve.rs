//! Analytic curves: line, circle, ellipse.

use crate::curve::{CurveKind, CurveProjection, ParametricCurve, closest_generic};
use crate::error::GeomError;
use crate::frame::{Frame, dist, similarity_scale, unit, wrap_into};
use crate::tol::Interval;
use core::f64::consts::TAU;
use sm_core::{Point3, Transform, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Infinite line `P(t) = origin + t * dir` (unit `dir`, so `t` is arc length).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Line {
    /// A point on the line (`t = 0`).
    pub origin: Point3,
    /// Unit direction.
    pub dir: Vector3,
}

impl Line {
    /// Build from a point and a (non-zero) direction.
    pub fn new(origin: Point3, dir: Vector3) -> Result<Self, GeomError> {
        Ok(Self { origin, dir: unit(dir).ok_or(GeomError::Invalid("zero direction"))? })
    }

    /// Image under a similarity transform.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        similarity_scale(t).ok_or(GeomError::NotSimilarity)?;
        Self::new(t.apply_point(self.origin), t.apply_vector(self.dir))
    }
}

impl ParametricCurve for Line {
    fn kind(&self) -> CurveKind {
        CurveKind::Line
    }
    fn range(&self) -> Interval {
        Interval::FULL
    }
    fn period(&self) -> Option<f64> {
        None
    }
    fn eval(&self, t: f64) -> Point3 {
        self.origin + self.dir * t
    }
    fn d1(&self, _t: f64) -> Vector3 {
        self.dir
    }
    fn d2(&self, _t: f64) -> Vector3 {
        Vector3::new(0.0, 0.0, 0.0)
    }
    fn is_closed(&self) -> bool {
        false
    }
    fn closest_in(&self, p: Point3, within: Interval) -> CurveProjection {
        let t = within.clamp((p - self.origin).dot(self.dir));
        let q = self.eval(t);
        CurveProjection { t, point: q, distance: dist(q, p) }
    }
    fn length(&self, a: f64, b: f64, _tol: f64) -> f64 {
        b - a
    }
}

/// Circle `P(t) = O + r (cos t X + sin t Y)`, period `2 pi`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Circle {
    /// Supporting frame (the circle lies in its xy plane).
    pub frame: Frame,
    /// Radius (> 0).
    pub radius: f64,
}

impl Circle {
    /// Build a circle; `radius` must be positive and finite.
    pub fn new(frame: Frame, radius: f64) -> Result<Self, GeomError> {
        if !(radius > 0.0 && radius.is_finite()) {
            return Err(GeomError::Invalid("circle radius"));
        }
        Ok(Self { frame, radius })
    }

    /// Image under a similarity transform (radius scales).
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        let (f, s) = self.frame.transformed(t)?;
        Self::new(f, self.radius * s)
    }
}

impl ParametricCurve for Circle {
    fn kind(&self) -> CurveKind {
        CurveKind::Circle
    }
    fn range(&self) -> Interval {
        Interval::new(0.0, TAU)
    }
    fn period(&self) -> Option<f64> {
        Some(TAU)
    }
    fn eval(&self, t: f64) -> Point3 {
        self.frame.point(self.radius * t.cos(), self.radius * t.sin(), 0.0)
    }
    fn d1(&self, t: f64) -> Vector3 {
        (self.frame.y * t.cos() - self.frame.x * t.sin()) * self.radius
    }
    fn d2(&self, t: f64) -> Vector3 {
        (self.frame.x * t.cos() + self.frame.y * t.sin()) * -self.radius
    }
    fn closest_in(&self, p: Point3, within: Interval) -> CurveProjection {
        let l = self.frame.local(p);
        let t0 = if l.x.hypot(l.y) < 1e-300 { within.lo } else { l.y.atan2(l.x) };
        let mk = |t: f64| {
            let q = self.eval(t);
            CurveProjection { t, point: q, distance: dist(q, p) }
        };
        let t = wrap_into(t0, within.lo, TAU);
        let mut best = if t <= within.hi { mk(t) } else { mk(within.lo) };
        for e in [within.lo, within.hi] {
            let c = mk(e);
            if c.distance < best.distance {
                best = c;
            }
        }
        best
    }
    fn length(&self, a: f64, b: f64, _tol: f64) -> f64 {
        self.radius * (b - a)
    }
}

/// Ellipse `P(t) = O + a cos t X + b sin t Y`, period `2 pi`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Ellipse {
    /// Supporting frame; `x` is the `a`-axis.
    pub frame: Frame,
    /// Semi-axis along `x` (> 0).
    pub a: f64,
    /// Semi-axis along `y` (> 0).
    pub b: f64,
}

impl Ellipse {
    /// Build an ellipse; both semi-axes must be positive and finite.
    pub fn new(frame: Frame, a: f64, b: f64) -> Result<Self, GeomError> {
        if !(a > 0.0 && b > 0.0 && a.is_finite() && b.is_finite()) {
            return Err(GeomError::Invalid("ellipse semi-axes"));
        }
        Ok(Self { frame, a, b })
    }

    /// Image under a similarity transform.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        let (f, s) = self.frame.transformed(t)?;
        Self::new(f, self.a * s, self.b * s)
    }
}

impl ParametricCurve for Ellipse {
    fn kind(&self) -> CurveKind {
        CurveKind::Ellipse
    }
    fn range(&self) -> Interval {
        Interval::new(0.0, TAU)
    }
    fn period(&self) -> Option<f64> {
        Some(TAU)
    }
    fn eval(&self, t: f64) -> Point3 {
        self.frame.point(self.a * t.cos(), self.b * t.sin(), 0.0)
    }
    fn d1(&self, t: f64) -> Vector3 {
        self.frame.y * (self.b * t.cos()) - self.frame.x * (self.a * t.sin())
    }
    fn d2(&self, t: f64) -> Vector3 {
        -(self.frame.x * (self.a * t.cos()) + self.frame.y * (self.b * t.sin()))
    }
    fn closest_in(&self, p: Point3, within: Interval) -> CurveProjection {
        closest_generic(self, p, within, 64)
    }
}
