//! CSG shape providers: signed implicit surfaces (quadrics incl. torus) and half-spaces.
//!
//! Convention (MCNP/OpenMC): the surface value is negative on the "inside"/negative side,
//! positive on the other side, zero on the surface. A [`HalfSpace`] picks a side with a [`Sense`].
//! Analytic surfaces ([`crate::analytic_surf`]) implement [`ImplicitSurface`] as well, including
//! the torus (quartic); [`Quadric`] covers the general second-degree surface.

use crate::ModelTolerance;
use crate::ray::{NormalCrossing, Ray, RayHit, RayIntersect, solve_quadratic};
use crate::sense::Sense;
use crate::vecs::{to_vec, unit};
use sm_core::{Point3, Transform, Vector3};
use std::sync::Arc;

/// Which side of an implicit surface a point lies on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Sidedness {
    /// Value < 0 beyond tolerance.
    Negative,
    /// Within tolerance of the surface.
    On,
    /// Value > 0 beyond tolerance.
    Positive,
}

/// A signed implicit surface `f(p) = 0`. Object-safe.
pub trait ImplicitSurface: Send + Sync + core::fmt::Debug {
    /// Signed value `f(p)`.
    fn value(&self, p: Point3) -> f64;
    /// Gradient `grad f(p)` (points toward positive values).
    fn gradient(&self, p: Point3) -> Vector3;
    /// Side of `p`, with "on" meaning within `tol.resolution` of the surface (first-order distance `f/|grad f|`).
    fn classify(&self, p: Point3, tol: &ModelTolerance) -> Sidedness {
        let f = self.value(p);
        let g = self.gradient(p).norm();
        let d = if g > 0.0 { f / g } else { f };
        if d.abs() <= tol.resolution {
            Sidedness::On
        } else if d < 0.0 {
            Sidedness::Negative
        } else {
            Sidedness::Positive
        }
    }
    /// Exact ray intersector, if available.
    fn ray_hook(&self) -> Option<&dyn RayIntersect> {
        None
    }
}

/// General quadric `f(p) = p^T A p + b . p + c` with symmetric `A`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Quadric {
    /// Symmetric quadratic part, row-major.
    pub a: [[f64; 3]; 3],
    /// Linear part.
    pub b: [f64; 3],
    /// Constant.
    pub c: f64,
}

type M3 = [[f64; 3]; 3];

fn mat_mul(x: &M3, y: &M3) -> M3 {
    let mut r = [[0.0; 3]; 3];
    for (i, row) in r.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = (0..3).map(|k| x[i][k] * y[k][j]).sum();
        }
    }
    r
}

fn transpose(m: &M3) -> M3 {
    let mut r = [[0.0; 3]; 3];
    for (i, row) in r.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = m[j][i];
        }
    }
    r
}

fn mat_vec(m: &M3, v: [f64; 3]) -> [f64; 3] {
    [0, 1, 2].map(|i| m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2])
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn inverse3(m: &M3) -> Option<M3> {
    let c = |r: usize, s: usize| {
        let (r1, r2) = ((r + 1) % 3, (r + 2) % 3);
        let (s1, s2) = ((s + 1) % 3, (s + 2) % 3);
        m[r1][s1] * m[r2][s2] - m[r1][s2] * m[r2][s1]
    };
    let det = m[0][0] * c(0, 0) + m[0][1] * c(0, 1) + m[0][2] * c(0, 2);
    if det.abs() < 1e-300 {
        return None;
    }
    let mut r = [[0.0; 3]; 3];
    for (i, row) in r.iter_mut().enumerate() {
        for (j, cell) in row.iter_mut().enumerate() {
            *cell = c(j, i) / det;
        }
    }
    Some(r)
}

impl Quadric {
    /// From the ten MCNP-style coefficients `[xx, yy, zz, xy, yz, zx, x, y, z, 1]`.
    pub fn from_coeffs(k: [f64; 10]) -> Self {
        Self {
            a: [[k[0], 0.5 * k[3], 0.5 * k[5]], [0.5 * k[3], k[1], 0.5 * k[4]], [0.5 * k[5], 0.5 * k[4], k[2]]],
            b: [k[6], k[7], k[8]],
            c: k[9],
        }
    }

    /// Plane through `p` with normal `n`; positive on the side `n` points to.
    pub fn plane(p: Point3, n: Vector3) -> Option<Self> {
        let n = unit(n)?;
        Some(Self { a: [[0.0; 3]; 3], b: [n.x, n.y, n.z], c: -n.dot(to_vec(p)) })
    }

    /// Sphere, negative inside.
    pub fn sphere(c: Point3, r: f64) -> Self {
        Self { a: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], b: [-2.0 * c.x, -2.0 * c.y, -2.0 * c.z], c: to_vec(c).dot(to_vec(c)) - r * r }
    }

    /// Infinite circular cylinder about the axis through `p` along `axis`, negative inside.
    pub fn cylinder(p: Point3, axis: Vector3, r: f64) -> Option<Self> {
        let n = unit(axis)?;
        let nn = [n.x, n.y, n.z];
        let mut a = [[0.0; 3]; 3];
        for (i, row) in a.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = f64::from(u8::from(i == j)) - nn[i] * nn[j];
            }
        }
        let pv = [p.x, p.y, p.z];
        let ap = mat_vec(&a, pv);
        Some(Self { a, b: [-2.0 * ap[0], -2.0 * ap[1], -2.0 * ap[2]], c: dot3(pv, ap) - r * r })
    }

    /// Double cone with apex `p`, axis `axis`, half-angle `alpha` (radians), negative inside.
    pub fn cone(p: Point3, axis: Vector3, alpha: f64) -> Option<Self> {
        let n = unit(axis)?;
        let nn = [n.x, n.y, n.z];
        let k2 = alpha.tan().powi(2);
        let mut a = [[0.0; 3]; 3];
        for (i, row) in a.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = f64::from(u8::from(i == j)) - (1.0 + k2) * nn[i] * nn[j];
            }
        }
        let pv = [p.x, p.y, p.z];
        let ap = mat_vec(&a, pv);
        Some(Self { a, b: [-2.0 * ap[0], -2.0 * ap[1], -2.0 * ap[2]], c: dot3(pv, ap) })
    }

    /// The same surface after applying `t` (proper or improper) to space: `f'(p) = f(t^-1 p)`.
    pub fn transformed(&self, t: &Transform) -> Option<Self> {
        let mi = inverse3(&t.m)?;
        let mit = transpose(&mi);
        let a2 = mat_mul(&mat_mul(&mit, &self.a), &mi);
        let tv = [t.t.x, t.t.y, t.t.z];
        let a2t = mat_vec(&a2, tv);
        let mib = mat_vec(&mit, self.b);
        let b2 = [0, 1, 2].map(|i| -2.0 * a2t[i] + mib[i]);
        let c2 = dot3(tv, a2t) - dot3(mib, tv) + self.c;
        Some(Self { a: a2, b: b2, c: c2 })
    }
}

impl ImplicitSurface for Quadric {
    fn value(&self, p: Point3) -> f64 {
        let v = [p.x, p.y, p.z];
        dot3(v, mat_vec(&self.a, v)) + dot3(self.b, v) + self.c
    }
    fn gradient(&self, p: Point3) -> Vector3 {
        let ap = mat_vec(&self.a, [p.x, p.y, p.z]);
        Vector3::new(2.0 * ap[0] + self.b[0], 2.0 * ap[1] + self.b[1], 2.0 * ap[2] + self.b[2])
    }
    fn ray_hook(&self) -> Option<&dyn RayIntersect> {
        Some(self)
    }
}

impl RayIntersect for Quadric {
    fn intersect_ray(&self, ray: &Ray, t_min: f64, t_max: f64, tol: &ModelTolerance, out: &mut Vec<RayHit>) {
        let (o, d) = ([ray.origin.x, ray.origin.y, ray.origin.z], [ray.dir.x, ray.dir.y, ray.dir.z]);
        let ad = mat_vec(&self.a, d);
        let a = dot3(d, ad);
        let b = 2.0 * dot3(o, ad) + dot3(self.b, d);
        let c = self.value(ray.origin);
        for (t, dbl) in solve_quadratic(a, b, c) {
            if t < t_min || t > t_max {
                continue;
            }
            let point = ray.at(t);
            let Some(normal) = unit(self.gradient(point)) else { continue };
            let crossing = if dbl { NormalCrossing::Tangent } else { NormalCrossing::classify(ray.dir, normal, tol) };
            out.push(RayHit { t, point, normal, uv: None, crossing });
        }
    }
}

/// Containment of a point relative to a half-space.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Containment {
    /// Strictly inside the region.
    Inside,
    /// On the bounding surface (within tolerance).
    Boundary,
    /// Strictly outside.
    Outside,
}

/// One side of an implicit surface: the CSG building block. `Forward` is the negative side
/// (`f < 0`), `Reversed` the positive side.
#[derive(Clone, Debug)]
pub struct HalfSpace {
    /// The bounding surface.
    pub surface: Arc<dyn ImplicitSurface>,
    /// Which side is "inside".
    pub sense: Sense,
}

impl HalfSpace {
    /// Containment of `p`.
    pub fn contains(&self, p: Point3, tol: &ModelTolerance) -> Containment {
        match (self.surface.classify(p, tol), self.sense) {
            (Sidedness::On, _) => Containment::Boundary,
            (Sidedness::Negative, Sense::Forward) | (Sidedness::Positive, Sense::Reversed) => Containment::Inside,
            _ => Containment::Outside,
        }
    }
}
