//! Local coordinate frames and small vector helpers.

use crate::error::GeomError;
use sm_core::{Point3, Transform, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Unit vector, or `None` for (near-)zero input.
pub fn unit(v: Vector3) -> Option<Vector3> {
    let n = v.norm();
    (n > 1e-300 && n.is_finite()).then(|| v * (1.0 / n))
}

/// Position vector of a point.
pub fn to_vec(p: Point3) -> Vector3 {
    Vector3::new(p.x, p.y, p.z)
}

/// Point at a position vector.
pub fn to_point(v: Vector3) -> Point3 {
    Point3::new(v.x, v.y, v.z)
}

/// Distance between two points.
pub fn dist(a: Point3, b: Point3) -> f64 {
    (a - b).norm()
}

/// A deterministic unit vector perpendicular to the unit vector `z`
/// (cross with the world axis of smallest `|component|`).
pub fn perpendicular(z: Vector3) -> Vector3 {
    let (ax, ay, az) = (z.x.abs(), z.y.abs(), z.z.abs());
    let axis = if ax <= ay && ax <= az {
        Vector3::new(1.0, 0.0, 0.0)
    } else if ay <= az {
        Vector3::new(0.0, 1.0, 0.0)
    } else {
        Vector3::new(0.0, 0.0, 1.0)
    };
    unit(z.cross(axis)).unwrap_or(Vector3::new(1.0, 0.0, 0.0))
}

/// If `t` is a similarity (orthogonal columns of equal norm, i.e. rotation/mirror times a
/// uniform scale), return the scale factor; else `None`.
pub fn similarity_scale(t: &Transform) -> Option<f64> {
    let m = &t.m;
    let col = |j: usize| Vector3::new(m[0][j], m[1][j], m[2][j]);
    let (c0, c1, c2) = (col(0), col(1), col(2));
    let s = c0.norm();
    if s.is_nan() || s <= 0.0 || !s.is_finite() {
        return None;
    }
    let tol = 1e-9 * s * s;
    let ok = (c1.norm() - s).abs() <= 1e-9 * s
        && (c2.norm() - s).abs() <= 1e-9 * s
        && c0.dot(c1).abs() <= tol
        && c0.dot(c2).abs() <= tol
        && c1.dot(c2).abs() <= tol;
    ok.then_some(s)
}

/// Wrap `t` into `[lo, lo + period)`.
pub fn wrap_into(t: f64, lo: f64, period: f64) -> f64 {
    let r = lo + (t - lo).rem_euclid(period);
    if r >= lo + period { lo } else { r }
}

/// Shift `t` by a multiple of `period` so it lies within half a period of `reference`.
/// Used to keep pcurves continuous across a periodic seam.
pub fn unwrap_near(t: f64, reference: f64, period: f64) -> f64 {
    t - period * ((t - reference) / period).round()
}

/// An orthonormal local frame. Right-handed (`x × y = z`) unless produced by an improper
/// transform, in which case it is left-handed: surface normals computed from
/// derivatives then flip consistently with the mirror.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Frame {
    /// Origin.
    pub origin: Point3,
    /// Local x axis (unit).
    pub x: Vector3,
    /// Local y axis (unit).
    pub y: Vector3,
    /// Local z axis (unit); the axis of revolution for cylinders/cones/spheres/tori.
    pub z: Vector3,
}

impl Frame {
    /// World frame at the origin.
    pub const WORLD: Self = Self {
        origin: Point3::new(0.0, 0.0, 0.0),
        x: Vector3::new(1.0, 0.0, 0.0),
        y: Vector3::new(0.0, 1.0, 0.0),
        z: Vector3::new(0.0, 0.0, 1.0),
    };

    /// Right-handed frame from an origin and z axis; x is chosen deterministically.
    pub fn from_z(origin: Point3, z: Vector3) -> Result<Self, GeomError> {
        let z = unit(z).ok_or(GeomError::Invalid("zero axis"))?;
        let x = perpendicular(z);
        Ok(Self { origin, x, y: z.cross(x), z })
    }

    /// Right-handed frame from an origin, z axis and an approximate x direction
    /// (Gram-Schmidt against z).
    pub fn from_zx(origin: Point3, z: Vector3, x: Vector3) -> Result<Self, GeomError> {
        let z = unit(z).ok_or(GeomError::Invalid("zero axis"))?;
        let x = unit(x - z * x.dot(z)).ok_or(GeomError::Invalid("x parallel to z"))?;
        Ok(Self { origin, x, y: z.cross(x), z })
    }

    /// True if right-handed.
    pub fn is_direct(&self) -> bool {
        self.x.cross(self.y).dot(self.z) > 0.0
    }

    /// Local coordinates `(lx, ly, lz)` of a world point.
    pub fn local(&self, p: Point3) -> Vector3 {
        let d = p - self.origin;
        Vector3::new(d.dot(self.x), d.dot(self.y), d.dot(self.z))
    }

    /// World point at local coordinates.
    pub fn point(&self, lx: f64, ly: f64, lz: f64) -> Point3 {
        self.origin + self.x * lx + self.y * ly + self.z * lz
    }

    /// Image under a similarity transform, plus the uniform scale factor. An improper
    /// transform yields a left-handed frame (see type docs).
    pub fn transformed(&self, t: &Transform) -> Result<(Self, f64), GeomError> {
        let s = similarity_scale(t).ok_or(GeomError::NotSimilarity)?;
        let ax = |v: Vector3| unit(t.apply_vector(v)).ok_or(GeomError::NotSimilarity);
        Ok((Self { origin: t.apply_point(self.origin), x: ax(self.x)?, y: ax(self.y)?, z: ax(self.z)? }, s))
    }
}
