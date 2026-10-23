//! Analytic surfaces: plane, cylinder, cone, sphere, torus.
//!
//! Parametrizations follow OCCT: `u` is the angle about the frame's z axis and is periodic
//! with period `2 pi` (seam at `u = 0 = 2 pi`).

use crate::error::GeomError;
use crate::frame::{Frame, dist, wrap_into};
use crate::surface::{Domain2, ParametricSurface, SingularLocus, Singularity, SurfaceKind, SurfaceProjection};
use crate::tol::Interval;
use core::f64::consts::{FRAC_PI_2, TAU};
use smallvec::{SmallVec, smallvec};
use sm_core::{Point3, Transform, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// `(w, t)` = radial and tangential unit vectors at longitude `u`.
fn wt(f: &Frame, u: f64) -> (Vector3, Vector3) {
    let (s, c) = u.sin_cos();
    (f.x * c + f.y * s, f.y * c - f.x * s)
}

/// Longitude of local coords; falls back to the hint (or 0) on the axis.
fn longitude(lx: f64, ly: f64, hint: Option<(f64, f64)>) -> f64 {
    if lx.hypot(ly) < 1e-300 { hint.map_or(0.0, |h| wrap_into(h.0, 0.0, TAU)) } else { wrap_into(ly.atan2(lx), 0.0, TAU) }
}

fn positive(x: f64) -> bool {
    x > 0.0 && x.is_finite()
}

const ZERO: Vector3 = Vector3::new(0.0, 0.0, 0.0);

/// Plane `S(u, v) = O + u X + v Y`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Plane {
    /// Supporting frame; the normal is `x × y` (= `z` when right-handed).
    pub frame: Frame,
}

impl Plane {
    /// Plane through `origin` with normal `n`.
    pub fn from_normal(origin: Point3, n: Vector3) -> Result<Self, GeomError> {
        Ok(Self { frame: Frame::from_z(origin, n)? })
    }
    /// Image under a similarity transform.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        Ok(Self { frame: self.frame.transformed(t)?.0 })
    }
}

impl ParametricSurface for Plane {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Plane
    }
    fn domain(&self) -> Domain2 {
        Domain2 { u: Interval::FULL, v: Interval::FULL }
    }
    fn period_u(&self) -> Option<f64> {
        None
    }
    fn period_v(&self) -> Option<f64> {
        None
    }
    fn singularities(&self) -> SmallVec<[Singularity; 2]> {
        SmallVec::new()
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        self.frame.point(u, v, 0.0)
    }
    fn d1(&self, _u: f64, _v: f64) -> (Vector3, Vector3) {
        (self.frame.x, self.frame.y)
    }
    fn d2(&self, _u: f64, _v: f64) -> (Vector3, Vector3, Vector3) {
        (ZERO, ZERO, ZERO)
    }
    fn project(&self, p: Point3, _hint: Option<(f64, f64)>) -> SurfaceProjection {
        let l = self.frame.local(p);
        SurfaceProjection { u: l.x, v: l.y, point: self.eval(l.x, l.y), distance: l.z.abs() }
    }
}

/// Cylinder `S(u, v) = O + r (cos u X + sin u Y) + v Z`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Cylinder {
    /// Supporting frame; `z` is the axis.
    pub frame: Frame,
    /// Radius (> 0).
    pub radius: f64,
}

impl Cylinder {
    /// Build a cylinder; radius must be positive.
    pub fn new(frame: Frame, radius: f64) -> Result<Self, GeomError> {
        if !positive(radius) {
            return Err(GeomError::Invalid("cylinder radius"));
        }
        Ok(Self { frame, radius })
    }
    /// Image under a similarity transform.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        let (f, s) = self.frame.transformed(t)?;
        Self::new(f, self.radius * s)
    }
}

impl ParametricSurface for Cylinder {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Cylinder
    }
    fn domain(&self) -> Domain2 {
        Domain2 { u: Interval::new(0.0, TAU), v: Interval::FULL }
    }
    fn period_u(&self) -> Option<f64> {
        Some(TAU)
    }
    fn period_v(&self) -> Option<f64> {
        None
    }
    fn singularities(&self) -> SmallVec<[Singularity; 2]> {
        SmallVec::new()
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        let (w, _) = wt(&self.frame, u);
        self.frame.origin + w * self.radius + self.frame.z * v
    }
    fn d1(&self, u: f64, _v: f64) -> (Vector3, Vector3) {
        let (_, t) = wt(&self.frame, u);
        (t * self.radius, self.frame.z)
    }
    fn d2(&self, u: f64, _v: f64) -> (Vector3, Vector3, Vector3) {
        let (w, _) = wt(&self.frame, u);
        (-(w * self.radius), ZERO, ZERO)
    }
    fn project(&self, p: Point3, hint: Option<(f64, f64)>) -> SurfaceProjection {
        let l = self.frame.local(p);
        let u = longitude(l.x, l.y, hint);
        let q = self.eval(u, l.z);
        SurfaceProjection { u, v: l.z, point: q, distance: dist(q, p) }
    }
}

/// Cone `S(u, v) = O + (R + v sin a)(cos u X + sin u Y) + v cos a Z`, semi-angle `a` in
/// `(0, pi/2)`. `R` is the radius at `v = 0`; the apex is at `v = -R / sin a`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Cone {
    /// Supporting frame; `z` is the axis.
    pub frame: Frame,
    /// Reference radius at `v = 0` (>= 0).
    pub radius: f64,
    /// Semi-angle in radians, in `(0, pi/2)`.
    pub semi_angle: f64,
}

impl Cone {
    /// Build a cone.
    pub fn new(frame: Frame, radius: f64, semi_angle: f64) -> Result<Self, GeomError> {
        if !(radius >= 0.0 && radius.is_finite()) || !(semi_angle > 0.0 && semi_angle < FRAC_PI_2) {
            return Err(GeomError::Invalid("cone radius/semi-angle"));
        }
        Ok(Self { frame, radius, semi_angle })
    }
    /// `v` of the apex.
    pub fn apex_v(&self) -> f64 {
        -self.radius / self.semi_angle.sin()
    }
    /// Image under a similarity transform.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        let (f, s) = self.frame.transformed(t)?;
        Self::new(f, self.radius * s, self.semi_angle)
    }
}

impl ParametricSurface for Cone {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Cone
    }
    fn domain(&self) -> Domain2 {
        Domain2 { u: Interval::new(0.0, TAU), v: Interval::FULL }
    }
    fn period_u(&self) -> Option<f64> {
        Some(TAU)
    }
    fn period_v(&self) -> Option<f64> {
        None
    }
    fn singularities(&self) -> SmallVec<[Singularity; 2]> {
        let v0 = self.apex_v();
        smallvec![Singularity { locus: SingularLocus::V(v0), point: self.eval(0.0, v0) }]
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        let (s, c) = self.semi_angle.sin_cos();
        let (w, _) = wt(&self.frame, u);
        self.frame.origin + w * (self.radius + v * s) + self.frame.z * (v * c)
    }
    fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let (s, c) = self.semi_angle.sin_cos();
        let (w, t) = wt(&self.frame, u);
        (t * (self.radius + v * s), w * s + self.frame.z * c)
    }
    fn d2(&self, u: f64, v: f64) -> (Vector3, Vector3, Vector3) {
        let s = self.semi_angle.sin();
        let (w, t) = wt(&self.frame, u);
        (-(w * (self.radius + v * s)), t * s, ZERO)
    }
    fn project(&self, p: Point3, hint: Option<(f64, f64)>) -> SurfaceProjection {
        let (s, c) = self.semi_angle.sin_cos();
        let l = self.frame.local(p);
        let rho = l.x.hypot(l.y);
        let u = longitude(l.x, l.y, hint);
        let v = (rho - self.radius) * s + l.z * c;
        let q = self.eval(u, v);
        SurfaceProjection { u, v, point: q, distance: dist(q, p) }
    }
}

/// Sphere `S(u, v) = O + r (cos v (cos u X + sin u Y) + sin v Z)`, `v in [-pi/2, pi/2]`.
/// The poles `v = +-pi/2` are singular.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Sphere {
    /// Supporting frame; `z` points to the north pole.
    pub frame: Frame,
    /// Radius (> 0).
    pub radius: f64,
}

impl Sphere {
    /// Build a sphere; radius must be positive.
    pub fn new(frame: Frame, radius: f64) -> Result<Self, GeomError> {
        if !positive(radius) {
            return Err(GeomError::Invalid("sphere radius"));
        }
        Ok(Self { frame, radius })
    }
    /// Image under a similarity transform.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        let (f, s) = self.frame.transformed(t)?;
        Self::new(f, self.radius * s)
    }
}

impl ParametricSurface for Sphere {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Sphere
    }
    fn domain(&self) -> Domain2 {
        Domain2 { u: Interval::new(0.0, TAU), v: Interval::new(-FRAC_PI_2, FRAC_PI_2) }
    }
    fn period_u(&self) -> Option<f64> {
        Some(TAU)
    }
    fn period_v(&self) -> Option<f64> {
        None
    }
    fn singularities(&self) -> SmallVec<[Singularity; 2]> {
        smallvec![
            Singularity { locus: SingularLocus::V(-FRAC_PI_2), point: self.frame.point(0.0, 0.0, -self.radius) },
            Singularity { locus: SingularLocus::V(FRAC_PI_2), point: self.frame.point(0.0, 0.0, self.radius) },
        ]
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        let (w, _) = wt(&self.frame, u);
        self.frame.origin + (w * v.cos() + self.frame.z * v.sin()) * self.radius
    }
    fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let (w, t) = wt(&self.frame, u);
        (t * (self.radius * v.cos()), (self.frame.z * v.cos() - w * v.sin()) * self.radius)
    }
    fn d2(&self, u: f64, v: f64) -> (Vector3, Vector3, Vector3) {
        let (w, t) = wt(&self.frame, u);
        let r = self.radius;
        (-(w * (r * v.cos())), -(t * (r * v.sin())), -((w * v.cos() + self.frame.z * v.sin()) * r))
    }
    /// Outward for right-handed frames; defined (by continuity) at the poles too.
    fn normal(&self, u: f64, v: f64) -> Result<Vector3, GeomError> {
        let (w, _) = wt(&self.frame, u);
        let n = w * v.cos() + self.frame.z * v.sin();
        Ok(if self.frame.is_direct() { n } else { -n })
    }
    fn project(&self, p: Point3, hint: Option<(f64, f64)>) -> SurfaceProjection {
        let l = self.frame.local(p);
        let rho = l.x.hypot(l.y);
        let u = longitude(l.x, l.y, hint);
        let v = if rho < 1e-300 && l.z.abs() < 1e-300 { hint.map_or(0.0, |h| h.1) } else { l.z.atan2(rho) };
        let q = self.eval(u, v);
        SurfaceProjection { u, v, point: q, distance: dist(q, p) }
    }
}

/// Ring torus `S(u, v) = O + (R + r cos v)(cos u X + sin u Y) + r sin v Z`, `R > r > 0`.
/// Periodic in both `u` and `v`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Torus {
    /// Supporting frame; `z` is the axis of revolution.
    pub frame: Frame,
    /// Major radius (centre of tube to axis).
    pub major: f64,
    /// Minor (tube) radius, `< major`.
    pub minor: f64,
}

impl Torus {
    /// Build a ring torus (`major > minor > 0`).
    pub fn new(frame: Frame, major: f64, minor: f64) -> Result<Self, GeomError> {
        if !(positive(major) && positive(minor) && major > minor) {
            return Err(GeomError::Invalid("torus radii (need major > minor > 0)"));
        }
        Ok(Self { frame, major, minor })
    }
    /// Image under a similarity transform.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        let (f, s) = self.frame.transformed(t)?;
        Self::new(f, self.major * s, self.minor * s)
    }
}

impl ParametricSurface for Torus {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Torus
    }
    fn domain(&self) -> Domain2 {
        Domain2 { u: Interval::new(0.0, TAU), v: Interval::new(0.0, TAU) }
    }
    fn period_u(&self) -> Option<f64> {
        Some(TAU)
    }
    fn period_v(&self) -> Option<f64> {
        Some(TAU)
    }
    fn singularities(&self) -> SmallVec<[Singularity; 2]> {
        SmallVec::new()
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        let (w, _) = wt(&self.frame, u);
        self.frame.origin + w * (self.major + self.minor * v.cos()) + self.frame.z * (self.minor * v.sin())
    }
    fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let (w, t) = wt(&self.frame, u);
        (t * (self.major + self.minor * v.cos()), (self.frame.z * v.cos() - w * v.sin()) * self.minor)
    }
    fn d2(&self, u: f64, v: f64) -> (Vector3, Vector3, Vector3) {
        let (w, t) = wt(&self.frame, u);
        let (big, r) = (self.major, self.minor);
        (-(w * (big + r * v.cos())), -(t * (r * v.sin())), -((w * v.cos() + self.frame.z * v.sin()) * r))
    }
    fn project(&self, p: Point3, hint: Option<(f64, f64)>) -> SurfaceProjection {
        let l = self.frame.local(p);
        let rho = l.x.hypot(l.y);
        let u = longitude(l.x, l.y, hint);
        let v = wrap_into(l.z.atan2(rho - self.major), 0.0, TAU);
        let q = self.eval(u, v);
        SurfaceProjection { u, v, point: q, distance: dist(q, p) }
    }
}
