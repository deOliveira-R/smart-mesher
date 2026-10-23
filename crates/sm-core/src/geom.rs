//! Basic geometric value types. Deliberately small; richer math lives in `sm-math`.

use core::ops::{Add, Mul, Neg, Sub};

/// A 3D vector.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Vector3 {
    /// x component.
    pub x: f64,
    /// y component.
    pub y: f64,
    /// z component.
    pub z: f64,
}

/// A 3D point.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Point3 {
    /// x coordinate.
    pub x: f64,
    /// y coordinate.
    pub y: f64,
    /// z coordinate.
    pub z: f64,
}

impl Vector3 {
    /// Construct a vector.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    /// Dot product.
    pub fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    /// Cross product.
    pub fn cross(self, o: Self) -> Self {
        Self::new(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }
    /// Euclidean norm.
    pub fn norm(self) -> f64 {
        self.dot(self).sqrt()
    }
}

impl Point3 {
    /// Construct a point.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

impl Add for Vector3 {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}
impl Sub for Vector3 {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Neg for Vector3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}
impl Mul<f64> for Vector3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Sub for Point3 {
    type Output = Vector3;
    fn sub(self, o: Self) -> Vector3 {
        Vector3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}
impl Add<Vector3> for Point3 {
    type Output = Self;
    fn add(self, v: Vector3) -> Self {
        Self::new(self.x + v.x, self.y + v.y, self.z + v.z)
    }
}

/// An affine transform `p -> M p + t`. May be improper (`det(M) < 0`, e.g. a mirror),
/// which reverses orientation. See docs/design/ir.md §5.1.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Transform {
    /// Row-major 3x3 linear part.
    pub m: [[f64; 3]; 3],
    /// Translation.
    pub t: Vector3,
}

impl Transform {
    /// The identity.
    pub const IDENTITY: Self = Self { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) };

    /// Apply to a point.
    pub fn apply_point(&self, p: Point3) -> Point3 {
        let m = &self.m;
        Point3::new(
            m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z + self.t.x,
            m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z + self.t.y,
            m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z + self.t.z,
        )
    }

    /// Apply the linear part to a vector.
    pub fn apply_vector(&self, v: Vector3) -> Vector3 {
        let m = &self.m;
        Vector3::new(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
        )
    }

    /// Determinant of the linear part.
    pub fn det(&self) -> f64 {
        let m = &self.m;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// True if the transform reverses orientation (mirror-like).
    pub fn is_improper(&self) -> bool {
        self.det() < 0.0
    }
}
