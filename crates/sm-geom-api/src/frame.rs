//! Right-handed orthonormal frames used by all analytic shapes.

use crate::{GeomError, util};
use sm_core::{Point3, Vector3};

/// A right-handed orthonormal frame: `x cross y == z`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Frame {
    /// Origin.
    pub origin: Point3,
    /// First axis (unit).
    pub x: Vector3,
    /// Second axis (unit).
    pub y: Vector3,
    /// Third axis, the shape axis (unit).
    pub z: Vector3,
}

impl Frame {
    /// The world frame at `origin`.
    pub const fn at(origin: Point3) -> Self {
        Self { origin, x: Vector3::new(1.0, 0.0, 0.0), y: Vector3::new(0.0, 1.0, 0.0), z: Vector3::new(0.0, 0.0, 1.0) }
    }

    /// Frame with axis `z` and `x` chosen as the world axis least aligned with `z`
    /// (deterministic), orthogonalized.
    pub fn from_z(origin: Point3, z: Vector3) -> Result<Self, GeomError> {
        let z = util::unit(z).ok_or(GeomError::Invalid("zero axis"))?;
        let hint = if z.x.abs() <= z.y.abs() && z.x.abs() <= z.z.abs() {
            Vector3::new(1.0, 0.0, 0.0)
        } else if z.y.abs() <= z.z.abs() {
            Vector3::new(0.0, 1.0, 0.0)
        } else {
            Vector3::new(0.0, 0.0, 1.0)
        };
        Self::from_zx(origin, z, hint)
    }

    /// Frame with axis `z` and reference direction `x_hint` (Gram-Schmidt).
    pub fn from_zx(origin: Point3, z: Vector3, x_hint: Vector3) -> Result<Self, GeomError> {
        let z = util::unit(z).ok_or(GeomError::Invalid("zero axis"))?;
        let x = util::unit(x_hint - z * x_hint.dot(z)).ok_or(GeomError::Invalid("x hint parallel to axis"))?;
        let y = z.cross(x);
        Ok(Self { origin, x, y, z })
    }

    /// Local coordinates of a world point.
    pub fn to_local(&self, p: Point3) -> Vector3 {
        let d = p - self.origin;
        Vector3::new(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }

    /// Local components of a world vector.
    pub fn to_local_vec(&self, v: Vector3) -> Vector3 {
        Vector3::new(v.dot(self.x), v.dot(self.y), v.dot(self.z))
    }

    /// World point from local coordinates.
    pub fn from_local(&self, l: Vector3) -> Point3 {
        self.origin + self.vec_from_local(l)
    }

    /// World vector from local components.
    pub fn vec_from_local(&self, l: Vector3) -> Vector3 {
        util::comb3(l.x, self.x, l.y, self.y, l.z, self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn right_handed_and_roundtrip() {
        let f = Frame::from_z(Point3::new(1.0, 2.0, 3.0), Vector3::new(1.0, 1.0, 1.0)).unwrap();
        assert!((f.x.cross(f.y) - f.z).norm() < 1e-14);
        assert!(f.x.dot(f.y).abs() < 1e-14 && f.x.dot(f.z).abs() < 1e-14);
        let p = Point3::new(-4.0, 0.5, 9.0);
        let q = f.from_local(f.to_local(p));
        assert!((p - q).norm() < 1e-13);
        assert!(Frame::from_z(Point3::default(), Vector3::default()).is_err());
    }
}
