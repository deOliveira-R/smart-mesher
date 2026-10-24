//! Ray types for the tracking hook.

use crate::Uv;
use sm_core::{Point3, Vector3};

/// A ray `origin + t * dir`. `dir` need not be unit; `t` is then in units of `|dir|`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Ray {
    /// Start point.
    pub origin: Point3,
    /// Direction.
    pub dir: Vector3,
}

impl Ray {
    /// Point at parameter `t`.
    pub fn at(&self, t: f64) -> Point3 {
        self.origin + self.dir * t
    }
}

/// How a ray passes through a surface at a hit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum Crossing {
    /// The ray goes the way the hit normal points (`dir . normal > 0`): it leaves the region the
    /// normal points away from.
    AlongNormal,
    /// The ray goes against the hit normal (`dir . normal < 0`): it enters that region.
    AgainstNormal,
    /// The ray touches without crossing: a double root, a grazing contact or a singular point.
    /// A double root is reported once.
    Tangent,
}

/// One ray/surface crossing.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub struct RayHit {
    /// Ray parameter of the crossing.
    pub t: f64,
    /// The crossing point.
    pub point: Point3,
    /// Unit surface normal at the point (the surface's own orientation: outward for analytic
    /// closed shapes, gradient direction for implicit forms; outward from the cell for
    /// [`crate::CsgCell`] hits). Zero at a singular point.
    pub normal: Vector3,
    /// Surface parameters of the hit when the implementor knows them.
    pub uv: Option<Uv>,
    /// How the ray passes through the surface.
    pub crossing: Crossing,
}

impl RayHit {
    /// Construct (the type is `#[non_exhaustive]`; implementors of `ray_hits` build hits here).
    pub const fn new(t: f64, point: Point3, normal: Vector3, uv: Option<Uv>, crossing: Crossing) -> Self {
        Self { t, point, normal, uv, crossing }
    }

    /// True if the ray crosses in the direction the normal points (`dir . normal > 0`) and is
    /// not tangent.
    pub fn leaving(&self, ray: &Ray) -> bool {
        self.crossing != Crossing::Tangent && ray.dir.dot(self.normal) > 0.0
    }
}
