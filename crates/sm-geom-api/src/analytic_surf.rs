//! Analytic surfaces: plane, cylinder, cone, sphere, torus.
//!
//! Parametrizations (local frame `x, y, z`, `c = cos u`, `s = sin u`):
//!
//! | Surface | `S(u, v)` | Domain | Normal |
//! |---|---|---|---|
//! | Plane | `(u, v, 0)` | R x R | `+z` |
//! | Cylinder | `(R c, R s, v)` | `[0,2pi]` (periodic) x R | outward |
//! | Cone (single nappe) | `(v k c, v k s, v)`, `k = tan(alpha)` | `[0,2pi]` (periodic) x `[0, inf)`, apex pole at `v = 0` | outward |
//! | Sphere | `R (cos v c, cos v s, sin v)` | `[0,2pi]` (periodic) x `[-pi/2, pi/2]`, poles at both `v` ends | outward |
//! | Torus | `((R + r cos v) c, (R + r cos v) s, r sin v)` | `[0,2pi]` x `[0,2pi]`, both periodic | outward |
//!
//! Each also implements [`ImplicitSurface`] (negative inside; the cone's implicit form is the
//! double cone) and [`RayIntersect`] with exact closed-form (quadric) or quartic (torus) roots.

use crate::csg::ImplicitSurface;
use crate::ray::{NormalCrossing, Ray, RayHit, RayIntersect, poly_roots, solve_quadratic};
use crate::surface::{DomainEdge, ParametricSurface, Pole, SecondDerivs, SurfaceDomain, SurfaceKind, SurfaceProjection};
use crate::vecs::{Frame, ParamRange, unit, wrap_near, wrap_to};
use crate::{GeomError, ModelTolerance};
use core::f64::consts::{FRAC_PI_2, TAU};
use sm_core::{Point3, Vector3};
use smallvec::{SmallVec, smallvec};

/// Plane `origin + u x + v y`, normal `z`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Plane {
    /// Frame (origin, in-plane axes, normal).
    pub frame: Frame,
}

/// Circular cylinder about the frame's z axis.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Cylinder {
    /// Frame (origin on the axis).
    pub frame: Frame,
    /// Radius (> 0).
    pub radius: f64,
}

/// Single-nappe circular cone: apex at the frame origin, opening toward `+z`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Cone {
    /// Frame (origin at the apex).
    pub frame: Frame,
    /// Half-angle between axis and generator, in `(0, pi/2)`.
    pub half_angle: f64,
}

/// Sphere about the frame origin.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Sphere {
    /// Frame (origin is the centre, `z` the polar axis).
    pub frame: Frame,
    /// Radius (> 0).
    pub radius: f64,
}

/// Torus about the frame's z axis.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Torus {
    /// Frame (origin is the centre).
    pub frame: Frame,
    /// Major radius (> minor).
    pub major: f64,
    /// Minor (tube) radius (> 0).
    pub minor: f64,
}

impl Plane {
    /// Plane through `origin` with normal `n`.
    pub fn new(origin: Point3, n: Vector3) -> Result<Self, GeomError> {
        Ok(Self { frame: Frame::from_z(origin, n)? })
    }
}
impl Cylinder {
    /// Validated constructor.
    pub fn new(frame: Frame, radius: f64) -> Result<Self, GeomError> {
        if radius > 0.0 { Ok(Self { frame, radius }) } else { Err(GeomError::Degenerate("radius must be positive")) }
    }
}
impl Cone {
    /// Validated constructor.
    pub fn new(frame: Frame, half_angle: f64) -> Result<Self, GeomError> {
        if half_angle > 0.0 && half_angle < FRAC_PI_2 { Ok(Self { frame, half_angle }) } else { Err(GeomError::Degenerate("half angle must be in (0, pi/2)")) }
    }
}
impl Sphere {
    /// Validated constructor.
    pub fn new(frame: Frame, radius: f64) -> Result<Self, GeomError> {
        if radius > 0.0 { Ok(Self { frame, radius }) } else { Err(GeomError::Degenerate("radius must be positive")) }
    }
}
impl Torus {
    /// Validated constructor (requires `major > minor > 0`, so the torus is non-self-intersecting).
    pub fn new(frame: Frame, major: f64, minor: f64) -> Result<Self, GeomError> {
        if minor > 0.0 && major > minor { Ok(Self { frame, major, minor }) } else { Err(GeomError::Degenerate("need major > minor > 0")) }
    }
}

fn angle_of(x: f64, y: f64, hint: Option<f64>, default: f64) -> f64 {
    let a = if x.hypot(y) < 1e-300 { default } else { y.atan2(x) };
    match hint {
        Some(h) => wrap_near(a, h, TAU),
        None => wrap_to(a, 0.0, TAU),
    }
}

fn proj_result(s: &dyn ParametricSurface, uv: [f64; 2], p: Point3) -> SurfaceProjection {
    let q = s.eval(uv[0], uv[1]);
    SurfaceProjection { uv, point: q, distance: (q - p).norm() }
}

const PERIODIC_U: SurfaceDomain = SurfaceDomain { u: ParamRange::new(0.0, TAU), v: ParamRange::REAL, u_period: Some(TAU), v_period: None };

impl ParametricSurface for Plane {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Plane
    }
    fn domain(&self) -> SurfaceDomain {
        SurfaceDomain { u: ParamRange::REAL, v: ParamRange::REAL, u_period: None, v_period: None }
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        self.frame.from_local([u, v, 0.0])
    }
    fn d1(&self, _u: f64, _v: f64) -> (Vector3, Vector3) {
        (self.frame.x, self.frame.y)
    }
    fn d2(&self, _u: f64, _v: f64) -> SecondDerivs {
        let z = Vector3::default();
        SecondDerivs { uu: z, uv: z, vv: z }
    }
    fn normal(&self, _u: f64, _v: f64) -> Result<Vector3, GeomError> {
        Ok(self.frame.z)
    }
    fn project(&self, p: Point3, _hint: Option<[f64; 2]>, _tol: &ModelTolerance) -> Result<SurfaceProjection, GeomError> {
        let l = self.frame.to_local(p);
        Ok(proj_result(self, [l[0], l[1]], p))
    }
    fn ray_hook(&self) -> Option<&dyn RayIntersect> {
        Some(self)
    }
    fn analytic(&self) -> Option<AnalyticSurface> {
        Some(AnalyticSurface::Plane(*self))
    }
}

impl ParametricSurface for Cylinder {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Cylinder
    }
    fn domain(&self) -> SurfaceDomain {
        PERIODIC_U
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        self.frame.from_local([self.radius * u.cos(), self.radius * u.sin(), v])
    }
    fn d1(&self, u: f64, _v: f64) -> (Vector3, Vector3) {
        (self.frame.vec_from_local([-self.radius * u.sin(), self.radius * u.cos(), 0.0]), self.frame.z)
    }
    fn d2(&self, u: f64, _v: f64) -> SecondDerivs {
        let z = Vector3::default();
        SecondDerivs { uu: self.frame.vec_from_local([-self.radius * u.cos(), -self.radius * u.sin(), 0.0]), uv: z, vv: z }
    }
    fn project(&self, p: Point3, hint: Option<[f64; 2]>, _tol: &ModelTolerance) -> Result<SurfaceProjection, GeomError> {
        let l = self.frame.to_local(p);
        let u = angle_of(l[0], l[1], hint.map(|h| h[0]), 0.0);
        Ok(proj_result(self, [u, l[2]], p))
    }
    fn ray_hook(&self) -> Option<&dyn RayIntersect> {
        Some(self)
    }
    fn analytic(&self) -> Option<AnalyticSurface> {
        Some(AnalyticSurface::Cylinder(*self))
    }
}

impl ParametricSurface for Cone {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Cone
    }
    fn domain(&self) -> SurfaceDomain {
        SurfaceDomain { v: ParamRange::new(0.0, f64::INFINITY), ..PERIODIC_U }
    }
    fn poles(&self) -> SmallVec<[Pole; 2]> {
        smallvec![Pole { edge: DomainEdge::VMin, point: self.frame.origin }]
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        let k = self.half_angle.tan();
        self.frame.from_local([v * k * u.cos(), v * k * u.sin(), v])
    }
    fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let k = self.half_angle.tan();
        (self.frame.vec_from_local([-v * k * u.sin(), v * k * u.cos(), 0.0]), self.frame.vec_from_local([k * u.cos(), k * u.sin(), 1.0]))
    }
    fn d2(&self, u: f64, v: f64) -> SecondDerivs {
        let k = self.half_angle.tan();
        SecondDerivs {
            uu: self.frame.vec_from_local([-v * k * u.cos(), -v * k * u.sin(), 0.0]),
            uv: self.frame.vec_from_local([-k * u.sin(), k * u.cos(), 0.0]),
            vv: Vector3::default(),
        }
    }
    fn normal(&self, u: f64, _v: f64) -> Result<Vector3, GeomError> {
        let k = self.half_angle.tan();
        let n = (1.0 + k * k).sqrt();
        Ok(self.frame.vec_from_local([u.cos() / n, u.sin() / n, -k / n]))
    }
    fn project(&self, p: Point3, hint: Option<[f64; 2]>, _tol: &ModelTolerance) -> Result<SurfaceProjection, GeomError> {
        let k = self.half_angle.tan();
        let l = self.frame.to_local(p);
        let rho = l[0].hypot(l[1]);
        let v = ((rho * k + l[2]) / (1.0 + k * k)).max(0.0);
        let u = angle_of(l[0], l[1], hint.map(|h| h[0]), 0.0);
        Ok(proj_result(self, [u, v], p))
    }
    fn ray_hook(&self) -> Option<&dyn RayIntersect> {
        Some(self)
    }
    fn analytic(&self) -> Option<AnalyticSurface> {
        Some(AnalyticSurface::Cone(*self))
    }
}

impl ParametricSurface for Sphere {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Sphere
    }
    fn domain(&self) -> SurfaceDomain {
        SurfaceDomain { v: ParamRange::new(-FRAC_PI_2, FRAC_PI_2), ..PERIODIC_U }
    }
    fn poles(&self) -> SmallVec<[Pole; 2]> {
        smallvec![
            Pole { edge: DomainEdge::VMin, point: self.frame.from_local([0.0, 0.0, -self.radius]) },
            Pole { edge: DomainEdge::VMax, point: self.frame.from_local([0.0, 0.0, self.radius]) },
        ]
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        let r = self.radius;
        self.frame.from_local([r * v.cos() * u.cos(), r * v.cos() * u.sin(), r * v.sin()])
    }
    fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let r = self.radius;
        (
            self.frame.vec_from_local([-r * v.cos() * u.sin(), r * v.cos() * u.cos(), 0.0]),
            self.frame.vec_from_local([-r * v.sin() * u.cos(), -r * v.sin() * u.sin(), r * v.cos()]),
        )
    }
    fn d2(&self, u: f64, v: f64) -> SecondDerivs {
        let r = self.radius;
        SecondDerivs {
            uu: self.frame.vec_from_local([-r * v.cos() * u.cos(), -r * v.cos() * u.sin(), 0.0]),
            uv: self.frame.vec_from_local([r * v.sin() * u.sin(), -r * v.sin() * u.cos(), 0.0]),
            vv: self.frame.vec_from_local([-r * v.cos() * u.cos(), -r * v.cos() * u.sin(), -r * v.sin()]),
        }
    }
    fn normal(&self, u: f64, v: f64) -> Result<Vector3, GeomError> {
        Ok(self.frame.vec_from_local([v.cos() * u.cos(), v.cos() * u.sin(), v.sin()]))
    }
    fn project(&self, p: Point3, hint: Option<[f64; 2]>, _tol: &ModelTolerance) -> Result<SurfaceProjection, GeomError> {
        let l = self.frame.to_local(p);
        let rho = l[0].hypot(l[1]);
        if rho.hypot(l[2]) < 1e-300 {
            return Err(GeomError::Singular { u: 0.0, v: 0.0 });
        }
        let v = l[2].atan2(rho);
        let u = angle_of(l[0], l[1], hint.map(|h| h[0]), 0.0);
        Ok(proj_result(self, [u, v], p))
    }
    fn ray_hook(&self) -> Option<&dyn RayIntersect> {
        Some(self)
    }
    fn analytic(&self) -> Option<AnalyticSurface> {
        Some(AnalyticSurface::Sphere(*self))
    }
}

impl ParametricSurface for Torus {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Torus
    }
    fn domain(&self) -> SurfaceDomain {
        SurfaceDomain { u: ParamRange::new(0.0, TAU), v: ParamRange::new(0.0, TAU), u_period: Some(TAU), v_period: Some(TAU) }
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        let w = self.major + self.minor * v.cos();
        self.frame.from_local([w * u.cos(), w * u.sin(), self.minor * v.sin()])
    }
    fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let (w, r) = (self.major + self.minor * v.cos(), self.minor);
        (
            self.frame.vec_from_local([-w * u.sin(), w * u.cos(), 0.0]),
            self.frame.vec_from_local([-r * v.sin() * u.cos(), -r * v.sin() * u.sin(), r * v.cos()]),
        )
    }
    fn d2(&self, u: f64, v: f64) -> SecondDerivs {
        let (w, r) = (self.major + self.minor * v.cos(), self.minor);
        SecondDerivs {
            uu: self.frame.vec_from_local([-w * u.cos(), -w * u.sin(), 0.0]),
            uv: self.frame.vec_from_local([r * v.sin() * u.sin(), -r * v.sin() * u.cos(), 0.0]),
            vv: self.frame.vec_from_local([-r * v.cos() * u.cos(), -r * v.cos() * u.sin(), -r * v.sin()]),
        }
    }
    fn normal(&self, u: f64, v: f64) -> Result<Vector3, GeomError> {
        Ok(self.frame.vec_from_local([v.cos() * u.cos(), v.cos() * u.sin(), v.sin()]))
    }
    fn project(&self, p: Point3, hint: Option<[f64; 2]>, _tol: &ModelTolerance) -> Result<SurfaceProjection, GeomError> {
        let l = self.frame.to_local(p);
        let rho = l[0].hypot(l[1]);
        let u = angle_of(l[0], l[1], hint.map(|h| h[0]), 0.0);
        let v = angle_of(rho - self.major, l[2], hint.map(|h| h[1]), 0.0);
        Ok(proj_result(self, [u, v], p))
    }
    fn ray_hook(&self) -> Option<&dyn RayIntersect> {
        Some(self)
    }
    fn analytic(&self) -> Option<AnalyticSurface> {
        Some(AnalyticSurface::Torus(*self))
    }
}

/// Closed set of analytic surface kinds (plain data, `Copy`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[non_exhaustive]
pub enum AnalyticSurface {
    /// Plane.
    Plane(Plane),
    /// Cylinder.
    Cylinder(Cylinder),
    /// Cone.
    Cone(Cone),
    /// Sphere.
    Sphere(Sphere),
    /// Torus.
    Torus(Torus),
}

impl AnalyticSurface {
    fn inner(&self) -> &dyn ParametricSurface {
        match self {
            Self::Plane(s) => s,
            Self::Cylinder(s) => s,
            Self::Cone(s) => s,
            Self::Sphere(s) => s,
            Self::Torus(s) => s,
        }
    }

    fn frame(&self) -> &Frame {
        match self {
            Self::Plane(s) => &s.frame,
            Self::Cylinder(s) => &s.frame,
            Self::Cone(s) => &s.frame,
            Self::Sphere(s) => &s.frame,
            Self::Torus(s) => &s.frame,
        }
    }

    /// Implicit value and gradient in the local frame.
    fn local_value_grad(&self, l: [f64; 3]) -> (f64, [f64; 3]) {
        let [x, y, z] = l;
        match self {
            Self::Plane(_) => (z, [0.0, 0.0, 1.0]),
            Self::Sphere(s) => (x * x + y * y + z * z - s.radius * s.radius, [2.0 * x, 2.0 * y, 2.0 * z]),
            Self::Cylinder(s) => (x * x + y * y - s.radius * s.radius, [2.0 * x, 2.0 * y, 0.0]),
            Self::Cone(s) => {
                let k2 = s.half_angle.tan().powi(2);
                (x * x + y * y - k2 * z * z, [2.0 * x, 2.0 * y, -2.0 * k2 * z])
            }
            Self::Torus(s) => {
                let (rr, r) = (s.major, s.minor);
                let q = x * x + y * y + z * z + rr * rr - r * r;
                (q * q - 4.0 * rr * rr * (x * x + y * y), [4.0 * q * x - 8.0 * rr * rr * x, 4.0 * q * y - 8.0 * rr * rr * y, 4.0 * q * z])
            }
        }
    }
}

impl ParametricSurface for AnalyticSurface {
    fn kind(&self) -> SurfaceKind {
        self.inner().kind()
    }
    fn domain(&self) -> SurfaceDomain {
        self.inner().domain()
    }
    fn poles(&self) -> SmallVec<[Pole; 2]> {
        self.inner().poles()
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        self.inner().eval(u, v)
    }
    fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        self.inner().d1(u, v)
    }
    fn d2(&self, u: f64, v: f64) -> SecondDerivs {
        self.inner().d2(u, v)
    }
    fn normal(&self, u: f64, v: f64) -> Result<Vector3, GeomError> {
        self.inner().normal(u, v)
    }
    fn project(&self, p: Point3, hint: Option<[f64; 2]>, tol: &ModelTolerance) -> Result<SurfaceProjection, GeomError> {
        self.inner().project(p, hint, tol)
    }
    fn ray_hook(&self) -> Option<&dyn RayIntersect> {
        Some(self)
    }
    fn analytic(&self) -> Option<AnalyticSurface> {
        Some(*self)
    }
}

impl ImplicitSurface for AnalyticSurface {
    fn value(&self, p: Point3) -> f64 {
        self.local_value_grad(self.frame().to_local(p)).0
    }
    fn gradient(&self, p: Point3) -> Vector3 {
        let g = self.local_value_grad(self.frame().to_local(p)).1;
        self.frame().vec_from_local(g)
    }
    fn ray_hook(&self) -> Option<&dyn RayIntersect> {
        Some(self)
    }
}

impl RayIntersect for AnalyticSurface {
    fn intersect_ray(&self, ray: &Ray, t_min: f64, t_max: f64, tol: &ModelTolerance, out: &mut Vec<RayHit>) {
        let f = self.frame();
        let o = f.to_local(ray.origin);
        let d = [ray.dir.dot(f.x), ray.dir.dot(f.y), ray.dir.dot(f.z)];
        let roots: SmallVec<[(f64, bool); 4]> = match self {
            Self::Plane(_) => {
                if d[2].abs() < 1e-300 {
                    SmallVec::new()
                } else {
                    smallvec![(-o[2] / d[2], false)]
                }
            }
            Self::Sphere(s) => solve_quadratic(d[0] * d[0] + d[1] * d[1] + d[2] * d[2], 2.0 * (o[0] * d[0] + o[1] * d[1] + o[2] * d[2]), o[0] * o[0] + o[1] * o[1] + o[2] * o[2] - s.radius * s.radius).into_iter().collect(),
            Self::Cylinder(s) => solve_quadratic(d[0] * d[0] + d[1] * d[1], 2.0 * (o[0] * d[0] + o[1] * d[1]), o[0] * o[0] + o[1] * o[1] - s.radius * s.radius).into_iter().collect(),
            Self::Cone(s) => {
                let k2 = s.half_angle.tan().powi(2);
                solve_quadratic(d[0] * d[0] + d[1] * d[1] - k2 * d[2] * d[2], 2.0 * (o[0] * d[0] + o[1] * d[1] - k2 * o[2] * d[2]), o[0] * o[0] + o[1] * o[1] - k2 * o[2] * o[2])
                    .into_iter()
                    .filter(|&(t, _)| o[2] + t * d[2] >= -tol.resolution)
                    .collect()
            }
            Self::Torus(s) => {
                let (rr, r) = (s.major, s.minor);
                let (a, b, c0) = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2], 2.0 * (o[0] * d[0] + o[1] * d[1] + o[2] * d[2]), o[0] * o[0] + o[1] * o[1] + o[2] * o[2]);
                let bound = rr + r;
                let hit = solve_quadratic(a, b, c0 - bound * bound);
                if hit.len() < 2 {
                    SmallVec::new()
                } else {
                    let (lo, hi) = (hit[0].0.max(t_min), hit[1].0.min(t_max));
                    let k = c0 + rr * rr - r * r;
                    let (a2, b2, c2) = (d[0] * d[0] + d[1] * d[1], 2.0 * (o[0] * d[0] + o[1] * d[1]), o[0] * o[0] + o[1] * o[1]);
                    let q = [k * k - 4.0 * rr * rr * c2, 2.0 * b * k - 4.0 * rr * rr * b2, b * b + 2.0 * a * k - 4.0 * rr * rr * a2, 2.0 * a * b, a * a];
                    if lo > hi { SmallVec::new() } else { poly_roots(&q, lo, hi).into_iter().map(|t| (t, false)).collect() }
                }
            }
        };
        let mut roots = roots;
        roots.sort_by(|a, b| a.0.total_cmp(&b.0));
        for (t, dbl) in roots {
            if t < t_min || t > t_max {
                continue;
            }
            let point = ray.at(t);
            let Some(normal) = unit(ImplicitSurface::gradient(self, point)) else { continue };
            let crossing = if dbl { NormalCrossing::Tangent } else { NormalCrossing::classify(ray.dir, normal, tol) };
            let uv = self.project(point, None, tol).ok().map(|p| p.uv);
            out.push(RayHit { t, point, normal, uv, crossing });
        }
    }
}

macro_rules! delegate_to_enum {
    ($($ty:ident),*) => {$(
        impl ImplicitSurface for $ty {
            fn value(&self, p: Point3) -> f64 { ImplicitSurface::value(&AnalyticSurface::$ty(*self), p) }
            fn gradient(&self, p: Point3) -> Vector3 { ImplicitSurface::gradient(&AnalyticSurface::$ty(*self), p) }
            fn ray_hook(&self) -> Option<&dyn RayIntersect> { Some(self) }
        }
        impl RayIntersect for $ty {
            fn intersect_ray(&self, ray: &Ray, t_min: f64, t_max: f64, tol: &ModelTolerance, out: &mut Vec<RayHit>) {
                AnalyticSurface::$ty(*self).intersect_ray(ray, t_min, t_max, tol, out)
            }
        }
        impl From<$ty> for AnalyticSurface { fn from(s: $ty) -> Self { Self::$ty(s) } }
    )*};
}
delegate_to_enum!(Plane, Cylinder, Cone, Sphere, Torus);
