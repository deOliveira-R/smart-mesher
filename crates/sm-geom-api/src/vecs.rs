//! Small vector helpers, parameter ranges and orthonormal frames shared by all shapes.

use crate::GeomError;
use sm_core::{Point3, Vector3};

/// Position vector of a point.
pub fn to_vec(p: Point3) -> Vector3 {
    Vector3::new(p.x, p.y, p.z)
}

/// Point at a position vector.
pub fn to_pt(v: Vector3) -> Point3 {
    Point3::new(v.x, v.y, v.z)
}

/// Unit vector, or `None` if shorter than `1e-300`.
pub fn unit(v: Vector3) -> Option<Vector3> {
    let n = v.norm();
    (n > 1e-300).then(|| v * (1.0 / n))
}

/// Wrap `u` into `[lo, lo + period)`.
pub fn wrap_to(u: f64, lo: f64, period: f64) -> f64 {
    let w = u - period * ((u - lo) / period).floor();
    if w >= lo + period { lo } else { w }
}

/// Shift `u` by a multiple of `period` so it is closest to `hint`.
pub fn wrap_near(u: f64, hint: f64, period: f64) -> f64 {
    u + period * ((hint - u) / period).round()
}

/// Closed parameter interval `[lo, hi]`. Infinite ends are allowed (lines, planes).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ParamRange {
    /// Lower bound.
    pub lo: f64,
    /// Upper bound.
    pub hi: f64,
}

impl ParamRange {
    /// The whole real line.
    pub const REAL: Self = Self { lo: f64::NEG_INFINITY, hi: f64::INFINITY };
    /// Construct a range.
    pub const fn new(lo: f64, hi: f64) -> Self {
        Self { lo, hi }
    }
    /// Length `hi - lo` (infinite for unbounded ranges).
    pub fn len(&self) -> f64 {
        self.hi - self.lo
    }
    /// True if the range has no extent.
    pub fn is_empty(&self) -> bool {
        self.hi <= self.lo
    }
    /// True if both ends are finite.
    pub fn is_bounded(&self) -> bool {
        self.lo.is_finite() && self.hi.is_finite()
    }
    /// True if `t` lies inside, with slack `eps`.
    pub fn contains(&self, t: f64, eps: f64) -> bool {
        t >= self.lo - eps && t <= self.hi + eps
    }
    /// Clamp `t` into the range.
    pub fn clamp(&self, t: f64) -> f64 {
        t.max(self.lo).min(self.hi)
    }
}

/// Right-handed orthonormal frame (`x * y = z`) with an origin.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Frame {
    /// Origin.
    pub origin: Point3,
    /// Local x axis (unit).
    pub x: Vector3,
    /// Local y axis (unit).
    pub y: Vector3,
    /// Local z axis (unit); the axis of revolution / plane normal.
    pub z: Vector3,
}

impl Frame {
    /// Frame with the given z axis. The x axis is chosen deterministically (the world axis least
    /// aligned with `z`, orthogonalized).
    pub fn from_z(origin: Point3, z: Vector3) -> Result<Self, GeomError> {
        let z = unit(z).ok_or(GeomError::Degenerate("zero axis"))?;
        let h = if z.x.abs() <= z.y.abs() && z.x.abs() <= z.z.abs() {
            Vector3::new(1.0, 0.0, 0.0)
        } else if z.y.abs() <= z.z.abs() {
            Vector3::new(0.0, 1.0, 0.0)
        } else {
            Vector3::new(0.0, 0.0, 1.0)
        };
        Self::from_zx(origin, z, h)
    }

    /// Frame with z axis `z` and x axis the component of `x_hint` orthogonal to `z`.
    pub fn from_zx(origin: Point3, z: Vector3, x_hint: Vector3) -> Result<Self, GeomError> {
        let z = unit(z).ok_or(GeomError::Degenerate("zero axis"))?;
        let x = unit(x_hint - z * x_hint.dot(z)).ok_or(GeomError::Degenerate("x hint parallel to axis"))?;
        Ok(Self { origin, x, y: z.cross(x), z })
    }

    /// World point to local coordinates.
    pub fn to_local(&self, p: Point3) -> [f64; 3] {
        let d = p - self.origin;
        [d.dot(self.x), d.dot(self.y), d.dot(self.z)]
    }

    /// Local coordinates to world point.
    pub fn from_local(&self, l: [f64; 3]) -> Point3 {
        self.origin + self.vec_from_local(l)
    }

    /// Local vector components to world vector.
    pub fn vec_from_local(&self, l: [f64; 3]) -> Vector3 {
        self.x * l[0] + self.y * l[1] + self.z * l[2]
    }
}
