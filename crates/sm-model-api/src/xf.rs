//! Rigid-transform algebra on top of [`sm_core::Transform`]: composition, inverse, constructors for
//! the three deck operations, tolerance-based equality, planes and axes.
//!
//! `sm-core` keeps `Transform` minimal and its `PartialEq` is exact, so group identity in this crate
//! always goes through [`approx_eq`], never `==`.

use crate::error::{ModelError, Result};
use sm_core::{Point3, Transform, Vector3};

/// Linear tolerance policy for geometric comparisons in this crate.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Tolerance {
    /// Absolute length tolerance (model units).
    pub linear: f64,
    /// Relative tolerance for areas, volumes and moments.
    pub relative: f64,
    /// Tolerance on unit-vector components (direction/orientation tests).
    pub angular: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self { linear: 1e-8, relative: 1e-8, angular: 1e-8 }
    }
}

impl Tolerance {
    /// Scale-aware comparison: `|a-b| <= linear + relative * max(|a|,|b|)`.
    pub fn close(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.linear + self.relative * a.abs().max(b.abs())
    }
}

/// Tolerance used for group-element identity (closure, lookup). Fixed so the group structure never
/// depends on a user tolerance.
pub const GROUP_TOL: f64 = 1e-9;

/// `a ∘ b`: apply `b` first, then `a`.
pub fn compose(a: &Transform, b: &Transform) -> Transform {
    let mut m = [[0.0; 3]; 3];
    for (i, row) in m.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
        }
    }
    let t = a.apply_vector(b.t) + a.t;
    Transform { m, t }
}

/// Inverse of an affine transform, or `None` if the linear part is singular.
pub fn inverse(x: &Transform) -> Option<Transform> {
    let m = &x.m;
    let det = x.det();
    if det.abs() < 1e-14 {
        return None;
    }
    let c = |r0: usize, c0: usize, r1: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
    let inv = [
        [c(1, 1, 2, 2) / det, -c(0, 1, 2, 2) / det, c(0, 1, 1, 2) / det],
        [-c(1, 0, 2, 2) / det, c(0, 0, 2, 2) / det, -c(0, 0, 1, 2) / det],
        [c(1, 0, 2, 1) / det, -c(0, 0, 2, 1) / det, c(0, 0, 1, 1) / det],
    ];
    let lin = Transform { m: inv, t: Vector3::new(0.0, 0.0, 0.0) };
    let t = -lin.apply_vector(x.t);
    Some(Transform { m: inv, t })
}

/// Component-wise comparison with absolute tolerance `tol` on the matrix and `tol * (1 + |t|)` on
/// the translation.
pub fn approx_eq(a: &Transform, b: &Transform, tol: f64) -> bool {
    for i in 0..3 {
        for j in 0..3 {
            if (a.m[i][j] - b.m[i][j]).abs() > tol {
                return false;
            }
        }
    }
    let scale = 1.0 + a.t.norm().max(b.t.norm());
    (a.t - b.t).norm() <= tol * scale
}

/// Pure translation.
pub fn translation(v: Vector3) -> Transform {
    Transform { t: v, ..Transform::IDENTITY }
}

/// Unit vector, or an error for a (near-)zero input.
pub fn normalized(v: Vector3) -> Result<Vector3> {
    let n = v.norm();
    if !n.is_finite() || n < 1e-12 {
        return Err(ModelError::Degenerate("zero-length direction"));
    }
    Ok(v * (1.0 / n))
}

/// An oriented plane `n · (x - p) = 0`. The normal is unit length.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Plane {
    point: Point3,
    normal: Vector3,
}

impl Plane {
    /// Build a plane through `point` with the given (non-zero) normal.
    pub fn new(point: Point3, normal: Vector3) -> Result<Self> {
        Ok(Self { point, normal: normalized(normal)? })
    }
    /// A point on the plane.
    pub fn point(&self) -> Point3 {
        self.point
    }
    /// The unit normal.
    pub fn normal(&self) -> Vector3 {
        self.normal
    }
    /// Signed distance of `p` from the plane (positive on the normal side).
    pub fn signed_distance(&self, p: Point3) -> f64 {
        (p - self.point).dot(self.normal)
    }
    /// The same plane with the opposite orientation.
    pub fn flipped(&self) -> Self {
        Self { point: self.point, normal: -self.normal }
    }
    /// Reflection in this plane (improper, fixes the plane pointwise).
    pub fn reflection(&self) -> Transform {
        let n = self.normal;
        let nn = [n.x, n.y, n.z];
        let mut m = Transform::IDENTITY.m;
        for (i, row) in m.iter_mut().enumerate() {
            for (j, v) in row.iter_mut().enumerate() {
                *v -= 2.0 * nn[i] * nn[j];
            }
        }
        let p = Vector3::new(self.point.x, self.point.y, self.point.z);
        Transform { m, t: n * (2.0 * n.dot(p)) }
    }
}

/// An oriented line (rotation axis): a point and a unit direction.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Axis {
    point: Point3,
    dir: Vector3,
}

impl Axis {
    /// Build an axis through `point` with the given (non-zero) direction.
    pub fn new(point: Point3, dir: Vector3) -> Result<Self> {
        Ok(Self { point, dir: normalized(dir)? })
    }
    /// A point on the axis.
    pub fn point(&self) -> Point3 {
        self.point
    }
    /// The unit direction.
    pub fn dir(&self) -> Vector3 {
        self.dir
    }
    /// Rotation by `angle` radians (right-handed about `dir`).
    pub fn rotation(&self, angle: f64) -> Transform {
        let (s, c) = angle.sin_cos();
        let u = self.dir;
        let uu = [u.x, u.y, u.z];
        let k = [[0.0, -u.z, u.y], [u.z, 0.0, -u.x], [-u.y, u.x, 0.0]];
        let mut m = [[0.0; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let id = if i == j { 1.0 } else { 0.0 };
                m[i][j] = c * id + s * k[i][j] + (1.0 - c) * uu[i] * uu[j];
            }
        }
        let lin = Transform { m, t: Vector3::new(0.0, 0.0, 0.0) };
        let p = Point3::new(self.point.x, self.point.y, self.point.z);
        let rp = lin.apply_point(p);
        Transform { m, t: p - rp }
    }
}

/// A rational number of full turns: `num / den` of 2π. Keeps rotation angles exact so a 1/6-turn
/// has order exactly 6.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Turns {
    /// Numerator (non-zero).
    pub num: i32,
    /// Denominator (positive).
    pub den: u32,
}

impl Turns {
    /// `1/n` of a turn.
    pub fn one_over(n: u32) -> Result<Self> {
        Self::new(1, n)
    }
    /// `num/den` of a turn; both must be non-zero.
    pub fn new(num: i32, den: u32) -> Result<Self> {
        if num == 0 || den == 0 {
            return Err(ModelError::Degenerate("rotation by zero turns"));
        }
        Ok(Self { num, den })
    }
    /// Angle in radians.
    pub fn radians(&self) -> f64 {
        std::f64::consts::TAU * f64::from(self.num) / f64::from(self.den)
    }
    /// Multiplicative order of the rotation (smallest `k` with `k * num / den` an integer).
    pub fn order(&self) -> u32 {
        let (mut a, mut b) = (self.num.unsigned_abs(), self.den);
        while b != 0 {
            (a, b) = (b, a % b);
        }
        self.den / a
    }
}

/// 3x3 matrix helpers used by detection.
pub type Mat3 = [[f64; 3]; 3];

/// Eigen-decomposition of a symmetric 3x3 matrix by cyclic Jacobi. Returns eigenvalues in descending
/// order and the matching unit eigenvectors (sign-canonicalized so the first non-negligible
/// component is positive). Deterministic: fixed sweep order, no randomness.
#[allow(clippy::needless_range_loop)] // index-symmetric Jacobi rotation reads clearer with explicit indices
pub fn eigen_symmetric(a: &Mat3) -> ([f64; 3], [Vector3; 3]) {
    let mut m = *a;
    let mut v = Transform::IDENTITY.m;
    for _ in 0..64 {
        let off = m[0][1].abs() + m[0][2].abs() + m[1][2].abs();
        if off < 1e-15 {
            break;
        }
        for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
            if m[p][q].abs() < 1e-300 {
                continue;
            }
            let theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let t = if theta == 0.0 { 1.0 } else { t };
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (mkp, mkq) = (m[k][p], m[k][q]);
                m[k][p] = c * mkp - s * mkq;
                m[k][q] = s * mkp + c * mkq;
            }
            for k in 0..3 {
                let (mpk, mqk) = (m[p][k], m[q][k]);
                m[p][k] = c * mpk - s * mqk;
                m[q][k] = s * mpk + c * mqk;
            }
            for row in v.iter_mut() {
                let (vkp, vkq) = (row[p], row[q]);
                row[p] = c * vkp - s * vkq;
                row[q] = s * vkp + c * vkq;
            }
        }
    }
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| m[j][j].total_cmp(&m[i][i]));
    let vals = [m[idx[0]][idx[0]], m[idx[1]][idx[1]], m[idx[2]][idx[2]]];
    let mut vecs = [Vector3::default(); 3];
    for (o, &i) in idx.iter().enumerate() {
        vecs[o] = canonical_sign(Vector3::new(v[0][i], v[1][i], v[2][i]));
    }
    (vals, vecs)
}

/// Flip `v` so its first component larger than `1e-9` in magnitude is positive.
pub fn canonical_sign(v: Vector3) -> Vector3 {
    for c in [v.x, v.y, v.z] {
        if c.abs() > 1e-9 {
            return if c < 0.0 { -v } else { v };
        }
    }
    v
}
