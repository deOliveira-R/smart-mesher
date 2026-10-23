//! Analytic curves: line, circle, ellipse. Surfaces are in [`crate::analytic_surf`].
//!
//! These are reference implementations of the contract and are what imports of STEP/SAT/CSG
//! produce for the common cases; `sm-geom` may add faster kernels behind the same traits.

use crate::curve::{CurveKind, CurveProjection, ParametricCurve, project_by_newton};
use crate::vecs::{Frame, ParamRange, unit, wrap_near, wrap_to};
use crate::{GeomError, ModelTolerance};
use core::f64::consts::TAU;
use sm_core::{Point3, Vector3};

/// Infinite line `origin + t * dir`, `dir` unit, so `t` is arc length.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Line {
    /// A point on the line (`t = 0`).
    pub origin: Point3,
    /// Unit direction.
    pub dir: Vector3,
}

/// Circle `origin + R (cos t x + sin t y)`, `t` in `[0, 2*pi]`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Circle {
    /// Frame: origin is the centre, `z` the plane normal.
    pub frame: Frame,
    /// Radius (> 0).
    pub radius: f64,
}

/// Ellipse `origin + a cos t x + b sin t y`, `t` in `[0, 2*pi]`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ellipse {
    /// Frame: origin is the centre, `z` the plane normal.
    pub frame: Frame,
    /// Semi-axis along `x` (> 0).
    pub a: f64,
    /// Semi-axis along `y` (> 0).
    pub b: f64,
}

impl Line {
    /// Line through `origin` along `dir` (normalized).
    pub fn new(origin: Point3, dir: Vector3) -> Result<Self, GeomError> {
        Ok(Self { origin, dir: unit(dir).ok_or(GeomError::Degenerate("zero direction"))? })
    }
    /// Line through two distinct points, `t = 0` at `a`.
    pub fn through(a: Point3, b: Point3) -> Result<Self, GeomError> {
        Self::new(a, b - a)
    }
}

impl Circle {
    /// Validated constructor.
    pub fn new(frame: Frame, radius: f64) -> Result<Self, GeomError> {
        if radius > 0.0 { Ok(Self { frame, radius }) } else { Err(GeomError::Degenerate("radius must be positive")) }
    }
}

impl Ellipse {
    /// Validated constructor.
    pub fn new(frame: Frame, a: f64, b: f64) -> Result<Self, GeomError> {
        if a > 0.0 && b > 0.0 { Ok(Self { frame, a, b }) } else { Err(GeomError::Degenerate("semi-axes must be positive")) }
    }
}

impl ParametricCurve for Line {
    fn kind(&self) -> CurveKind {
        CurveKind::Line
    }
    fn domain(&self) -> ParamRange {
        ParamRange::REAL
    }
    fn eval(&self, t: f64) -> Point3 {
        self.origin + self.dir * t
    }
    fn d1(&self, _t: f64) -> Vector3 {
        self.dir
    }
    fn d2(&self, _t: f64) -> Vector3 {
        Vector3::default()
    }
    fn project(&self, p: Point3, _hint: Option<f64>, _tol: &ModelTolerance) -> Result<CurveProjection, GeomError> {
        let t = (p - self.origin).dot(self.dir);
        let q = self.eval(t);
        Ok(CurveProjection { t, point: q, distance: (q - p).norm() })
    }
    fn length(&self, t0: f64, t1: f64, _tol: &ModelTolerance) -> f64 {
        (t1 - t0).max(0.0)
    }
    fn analytic(&self) -> Option<AnalyticCurve> {
        Some(AnalyticCurve::Line(*self))
    }
}

impl ParametricCurve for Circle {
    fn kind(&self) -> CurveKind {
        CurveKind::Circle
    }
    fn domain(&self) -> ParamRange {
        ParamRange::new(0.0, TAU)
    }
    fn period(&self) -> Option<f64> {
        Some(TAU)
    }
    fn eval(&self, t: f64) -> Point3 {
        self.frame.from_local([self.radius * t.cos(), self.radius * t.sin(), 0.0])
    }
    fn d1(&self, t: f64) -> Vector3 {
        self.frame.vec_from_local([-self.radius * t.sin(), self.radius * t.cos(), 0.0])
    }
    fn d2(&self, t: f64) -> Vector3 {
        self.frame.vec_from_local([-self.radius * t.cos(), -self.radius * t.sin(), 0.0])
    }
    fn project(&self, p: Point3, hint: Option<f64>, _tol: &ModelTolerance) -> Result<CurveProjection, GeomError> {
        let l = self.frame.to_local(p);
        if l[0].hypot(l[1]) < 1e-300 {
            return Err(GeomError::Singular { u: hint.unwrap_or(0.0), v: 0.0 });
        }
        let a = l[1].atan2(l[0]);
        let t = match hint {
            Some(h) => wrap_near(a, h, TAU),
            None => wrap_to(a, 0.0, TAU),
        };
        let q = self.eval(t);
        Ok(CurveProjection { t, point: q, distance: (q - p).norm() })
    }
    fn length(&self, t0: f64, t1: f64, _tol: &ModelTolerance) -> f64 {
        self.radius * (t1 - t0).max(0.0)
    }
    fn analytic(&self) -> Option<AnalyticCurve> {
        Some(AnalyticCurve::Circle(*self))
    }
}

impl ParametricCurve for Ellipse {
    fn kind(&self) -> CurveKind {
        CurveKind::Ellipse
    }
    fn domain(&self) -> ParamRange {
        ParamRange::new(0.0, TAU)
    }
    fn period(&self) -> Option<f64> {
        Some(TAU)
    }
    fn eval(&self, t: f64) -> Point3 {
        self.frame.from_local([self.a * t.cos(), self.b * t.sin(), 0.0])
    }
    fn d1(&self, t: f64) -> Vector3 {
        self.frame.vec_from_local([-self.a * t.sin(), self.b * t.cos(), 0.0])
    }
    fn d2(&self, t: f64) -> Vector3 {
        self.frame.vec_from_local([-self.a * t.cos(), -self.b * t.sin(), 0.0])
    }
    fn project(&self, p: Point3, hint: Option<f64>, tol: &ModelTolerance) -> Result<CurveProjection, GeomError> {
        let mut r = project_by_newton(self, p, hint, tol)?;
        r.t = match hint {
            Some(h) => wrap_near(r.t, h, TAU),
            None => wrap_to(r.t, 0.0, TAU),
        };
        Ok(r)
    }
    fn analytic(&self) -> Option<AnalyticCurve> {
        Some(AnalyticCurve::Ellipse(*self))
    }
}

/// Closed set of analytic curve kinds (plain data, `Copy`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum AnalyticCurve {
    /// Line.
    Line(Line),
    /// Circle.
    Circle(Circle),
    /// Ellipse.
    Ellipse(Ellipse),
}

impl AnalyticCurve {
    fn inner(&self) -> &dyn ParametricCurve {
        match self {
            Self::Line(c) => c,
            Self::Circle(c) => c,
            Self::Ellipse(c) => c,
        }
    }
}

impl ParametricCurve for AnalyticCurve {
    fn kind(&self) -> CurveKind {
        self.inner().kind()
    }
    fn domain(&self) -> ParamRange {
        self.inner().domain()
    }
    fn period(&self) -> Option<f64> {
        self.inner().period()
    }
    fn eval(&self, t: f64) -> Point3 {
        self.inner().eval(t)
    }
    fn d1(&self, t: f64) -> Vector3 {
        self.inner().d1(t)
    }
    fn d2(&self, t: f64) -> Vector3 {
        self.inner().d2(t)
    }
    fn project(&self, p: Point3, hint: Option<f64>, tol: &ModelTolerance) -> Result<CurveProjection, GeomError> {
        self.inner().project(p, hint, tol)
    }
    fn length(&self, t0: f64, t1: f64, tol: &ModelTolerance) -> f64 {
        self.inner().length(t0, t1, tol)
    }
    fn analytic(&self) -> Option<AnalyticCurve> {
        Some(*self)
    }
}
