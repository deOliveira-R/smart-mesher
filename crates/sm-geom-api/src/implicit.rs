//! Implicit (CSG) surfaces: signed quadrics, torus, half-spaces.
//!
//! Convention: `value(p) < 0` inside, `> 0` outside, `0` on the surface. For planes the
//! value is the signed distance along the normal.

use crate::error::GeomError;
use crate::frame::{Frame, similarity_scale, unit};
use crate::ray::{Ray, RayHit, RayIntersect, ray_roots};
use crate::tol::{Interval, TolerancePolicy};
use core::fmt::Debug;
use smallvec::SmallVec;
use sm_core::{Point3, Transform, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Which side of an implicit surface a point lies on (within tolerance).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sidedness {
    /// `value < 0`.
    Negative,
    /// Within tolerance of the surface.
    On,
    /// `value > 0`.
    Positive,
}

/// A signed implicit surface `f(p) = 0`. Object-safe.
pub trait ImplicitSurface: Send + Sync + Debug {
    /// `f(p)`.
    fn value(&self, p: Point3) -> f64;
    /// `grad f(p)`.
    fn gradient(&self, p: Point3) -> Vector3;

    /// Classify `p`; "on" means the first-order distance `|f| / |grad f|` is within `tol`.
    fn side(&self, p: Point3, tol: f64) -> Sidedness {
        let f = self.value(p);
        let g = self.gradient(p).norm().max(1e-300);
        if f.abs() <= tol * g {
            Sidedness::On
        } else if f < 0.0 {
            Sidedness::Negative
        } else {
            Sidedness::Positive
        }
    }
}

type Mat3 = [[f64; 3]; 3];

fn mat_vec(a: &Mat3, v: Vector3) -> Vector3 {
    Vector3::new(
        a[0][0] * v.x + a[0][1] * v.y + a[0][2] * v.z,
        a[1][0] * v.x + a[1][1] * v.y + a[1][2] * v.z,
        a[2][0] * v.x + a[2][1] * v.y + a[2][2] * v.z,
    )
}

fn vec_of(p: Point3) -> Vector3 {
    Vector3::new(p.x, p.y, p.z)
}

/// General quadric `f(p) = p^T A p + b . p + c` with symmetric `A` (covers planes, spheres,
/// cylinders, cones, ellipsoids, paraboloids, hyperboloids: MCNP `P/SO/S/C/K/SQ/GQ`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Quadric {
    /// Symmetric quadratic part.
    pub a: [[f64; 3]; 3],
    /// Linear part.
    pub b: Vector3,
    /// Constant.
    pub c: f64,
}

impl Quadric {
    /// `(p - o)^T A (p - o) + k`.
    pub fn centered(a: Mat3, o: Point3, k: f64) -> Self {
        let ov = vec_of(o);
        let ao = mat_vec(&a, ov);
        Self { a, b: ao * -2.0, c: ov.dot(ao) + k }
    }

    /// Plane through `point` with normal `n`; `value` is the signed distance.
    pub fn plane(point: Point3, n: Vector3) -> Result<Self, GeomError> {
        let n = unit(n).ok_or(GeomError::Invalid("zero plane normal"))?;
        Ok(Self { a: [[0.0; 3]; 3], b: n, c: -n.dot(vec_of(point)) })
    }

    /// Sphere of radius `r` about `center`.
    pub fn sphere(center: Point3, r: f64) -> Result<Self, GeomError> {
        if !(r > 0.0 && r.is_finite()) {
            return Err(GeomError::Invalid("sphere radius"));
        }
        Ok(Self::centered([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], center, -r * r))
    }

    /// Infinite circular cylinder about the axis through `point` with direction `axis`.
    pub fn cylinder(point: Point3, axis: Vector3, r: f64) -> Result<Self, GeomError> {
        let n = unit(axis).ok_or(GeomError::Invalid("zero axis"))?;
        if !(r > 0.0 && r.is_finite()) {
            return Err(GeomError::Invalid("cylinder radius"));
        }
        Ok(Self::centered(Self::diag_minus_outer(n, 1.0), point, -r * r))
    }

    /// Double cone with `apex`, `axis` and half-angle `a` in `(0, pi/2)`.
    pub fn cone(apex: Point3, axis: Vector3, half_angle: f64) -> Result<Self, GeomError> {
        let n = unit(axis).ok_or(GeomError::Invalid("zero axis"))?;
        if !(half_angle > 0.0 && half_angle < core::f64::consts::FRAC_PI_2) {
            return Err(GeomError::Invalid("cone half-angle"));
        }
        let c2 = half_angle.cos().powi(2);
        // cos^2 a |d|^2 - (d.n)^2  =  d^T (cos^2 a I - n n^T) d
        Ok(Self::centered(Self::diag_minus_outer(n, c2), apex, 0.0))
    }

    /// `alpha I - n n^T` (cylinder: `alpha = 1`; cone: `alpha = cos^2 a`).
    fn diag_minus_outer(n: Vector3, alpha: f64) -> Mat3 {
        let nn = [n.x, n.y, n.z];
        let mut m = [[0.0; 3]; 3];
        for (i, row) in m.iter_mut().enumerate() {
            for (j, e) in row.iter_mut().enumerate() {
                *e = if i == j { alpha } else { 0.0 } - nn[i] * nn[j];
            }
        }
        m
    }

    /// From the ten MCNP `GQ` coefficients `A x^2 + B y^2 + C z^2 + D xy + E yz + F zx + G x + H y + J z + K`.
    pub fn from_gq(k: [f64; 10]) -> Self {
        Self {
            a: [[k[0], 0.5 * k[3], 0.5 * k[5]], [0.5 * k[3], k[1], 0.5 * k[4]], [0.5 * k[5], 0.5 * k[4], k[2]]],
            b: Vector3::new(k[6], k[7], k[8]),
            c: k[9],
        }
    }

    /// Image under a similarity transform (uniform scale `s`, mirror allowed).
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        // f'(q) = f(T^-1 q). For a similarity T = s R: T^-1 q = R^T (q - t) / s.
        let s = similarity_scale(t).ok_or(GeomError::NotSimilarity)?;
        let m = &t.m;
        let r = |i: usize, j: usize| m[i][j] / s;
        // Q = R^T (columns of R are rows of Q): Q[i][j] = R[j][i]
        let q: Mat3 = [[r(0, 0), r(1, 0), r(2, 0)], [r(0, 1), r(1, 1), r(2, 1)], [r(0, 2), r(1, 2), r(2, 2)]];
        let mul = |a: &Mat3, b: &Mat3| {
            let mut o = [[0.0; 3]; 3];
            for (i, row) in o.iter_mut().enumerate() {
                for (j, e) in row.iter_mut().enumerate() {
                    *e = (0..3).map(|k| a[i][k] * b[k][j]).sum();
                }
            }
            o
        };
        let qt = [[q[0][0], q[1][0], q[2][0]], [q[0][1], q[1][1], q[2][1]], [q[0][2], q[1][2], q[2][2]]];
        // With x = Q (q - t) / s:  f' = x^T A x + b.x + c
        let a2 = mul(&mul(&qt, &self.a), &q);
        let inv_s2 = 1.0 / (s * s);
        let a_new = a2.map(|row| row.map(|e| e * inv_s2));
        let tv = t.t;
        let apply_q = |v: Vector3| Vector3::new(q[0][0] * v.x + q[0][1] * v.y + q[0][2] * v.z, q[1][0] * v.x + q[1][1] * v.y + q[1][2] * v.z, q[2][0] * v.x + q[2][1] * v.y + q[2][2] * v.z);
        // x = Q q / s - Q t / s
        let x0 = apply_q(tv) * (-1.0 / s);
        let ax0 = mat_vec(&self.a, x0);
        let lin_a = mat_vec(&qt, ax0) * (2.0 / s);
        let lin_b = mat_vec(&qt, self.b) * (1.0 / s);
        Ok(Self { a: a_new, b: lin_a + lin_b, c: x0.dot(ax0) + self.b.dot(x0) + self.c })
    }
}

impl ImplicitSurface for Quadric {
    fn value(&self, p: Point3) -> f64 {
        let v = vec_of(p);
        v.dot(mat_vec(&self.a, v)) + self.b.dot(v) + self.c
    }
    fn gradient(&self, p: Point3) -> Vector3 {
        mat_vec(&self.a, vec_of(p)) * 2.0 + self.b
    }
}

impl RayIntersect for Quadric {
    fn intersect_ray(&self, ray: &Ray, t_range: Interval, tol: &TolerancePolicy, hits: &mut SmallVec<[RayHit; 4]>) -> Result<(), GeomError> {
        let (o, d) = (vec_of(ray.origin), ray.dir);
        let ad = mat_vec(&self.a, d);
        let coef = [self.value(ray.origin), 2.0 * o.dot(ad) + self.b.dot(d), d.dot(ad)];
        for (t, grazing) in ray_roots(&coef, t_range, tol) {
            let point = ray.at(t);
            let normal = unit(self.gradient(point)).unwrap_or(d);
            hits.push(RayHit { t, point, normal, uv: None, grazing });
        }
        Ok(())
    }
}

/// Implicit torus `(|l|^2 + R^2 - r^2)^2 - 4 R^2 (lx^2 + ly^2)` in frame-local coordinates `l`.
/// (A quartic: not a quadric, but a first-class CSG surface in MCNP/OpenMC.)
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ImplicitTorus {
    /// Supporting frame; `z` is the axis.
    pub frame: Frame,
    /// Major radius.
    pub major: f64,
    /// Minor radius.
    pub minor: f64,
}

impl ImplicitTorus {
    /// Build; requires `major > minor > 0`.
    pub fn new(frame: Frame, major: f64, minor: f64) -> Result<Self, GeomError> {
        if !(minor > 0.0 && major > minor && major.is_finite()) {
            return Err(GeomError::Invalid("torus radii"));
        }
        Ok(Self { frame, major, minor })
    }
    /// Image under a similarity transform.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        let (f, s) = self.frame.transformed(t)?;
        Self::new(f, self.major * s, self.minor * s)
    }
}

impl ImplicitSurface for ImplicitTorus {
    fn value(&self, p: Point3) -> f64 {
        let l = self.frame.local(p);
        let s = l.dot(l) + self.major * self.major - self.minor * self.minor;
        s * s - 4.0 * self.major * self.major * (l.x * l.x + l.y * l.y)
    }
    fn gradient(&self, p: Point3) -> Vector3 {
        let l = self.frame.local(p);
        let s = l.dot(l) + self.major * self.major - self.minor * self.minor;
        let r2 = 8.0 * self.major * self.major;
        let g = Vector3::new(4.0 * s * l.x - r2 * l.x, 4.0 * s * l.y - r2 * l.y, 4.0 * s * l.z);
        self.frame.x * g.x + self.frame.y * g.y + self.frame.z * g.z
    }
}

impl RayIntersect for ImplicitTorus {
    fn intersect_ray(&self, ray: &Ray, t_range: Interval, tol: &TolerancePolicy, hits: &mut SmallVec<[RayHit; 4]>) -> Result<(), GeomError> {
        let o = self.frame.local(ray.origin);
        let d = Vector3::new(ray.dir.dot(self.frame.x), ray.dir.dot(self.frame.y), ray.dir.dot(self.frame.z));
        let (rr, k) = (self.major * self.major, self.major * self.major - self.minor * self.minor);
        let s = o.dot(d);
        let c0 = o.dot(o) + k;
        let (a2, b1, c_) = (d.x * d.x + d.y * d.y, o.x * d.x + o.y * d.y, o.x * o.x + o.y * o.y);
        let coef = [c0 * c0 - 4.0 * rr * c_, 4.0 * s * c0 - 8.0 * rr * b1, 4.0 * s * s + 2.0 * c0 - 4.0 * rr * a2, 4.0 * s, 1.0];
        for (t, grazing) in ray_roots(&coef, t_range, tol) {
            let point = ray.at(t);
            let normal = unit(self.gradient(point)).unwrap_or(ray.dir);
            hits.push(RayHit { t, point, normal, uv: None, grazing });
        }
        Ok(())
    }
}

/// Closed set of implicit surfaces usable as CSG surfaces.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[non_exhaustive]
pub enum ImplicitDef {
    /// Signed quadric.
    Quadric(Quadric),
    /// Quartic torus.
    Torus(ImplicitTorus),
}

impl ImplicitDef {
    /// Image under a similarity transform.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        Ok(match self {
            Self::Quadric(q) => Self::Quadric(q.transformed(t)?),
            Self::Torus(q) => Self::Torus(q.transformed(t)?),
        })
    }
}

impl ImplicitSurface for ImplicitDef {
    fn value(&self, p: Point3) -> f64 {
        match self {
            Self::Quadric(q) => q.value(p),
            Self::Torus(q) => q.value(p),
        }
    }
    fn gradient(&self, p: Point3) -> Vector3 {
        match self {
            Self::Quadric(q) => q.gradient(p),
            Self::Torus(q) => q.gradient(p),
        }
    }
}

impl RayIntersect for ImplicitDef {
    fn intersect_ray(&self, ray: &Ray, t_range: Interval, tol: &TolerancePolicy, hits: &mut SmallVec<[RayHit; 4]>) -> Result<(), GeomError> {
        match self {
            Self::Quadric(q) => q.intersect_ray(ray, t_range, tol, hits),
            Self::Torus(q) => q.intersect_ray(ray, t_range, tol, hits),
        }
    }
}

/// A CSG half-space `{ p : sign * f(p) <= 0 }`. The boundary [`ImplicitDef`] is the shape a
/// topology face can reference (shape provider); the half-space says which side is "in".
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct HalfSpace {
    /// Boundary surface.
    pub surface: ImplicitDef,
    /// If true the region is `f >= 0` (outside the surface), else `f <= 0`.
    pub positive: bool,
}

impl HalfSpace {
    /// Membership; points within `tol` of the boundary count as inside (closed region).
    pub fn contains(&self, p: Point3, tol: f64) -> bool {
        match self.surface.side(p, tol) {
            Sidedness::On => true,
            Sidedness::Negative => !self.positive,
            Sidedness::Positive => self.positive,
        }
    }
}
