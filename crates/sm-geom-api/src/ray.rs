//! Ray types for the tracking hook.

use sm_core::{Point3, Vector3};

/// A ray `origin + t * dir`. `dir` need not be unit; `t` is then in units of `|dir|`.
#[derive(Clone, Copy, Debug, PartialEq)]
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

/// One ray/surface crossing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RayHit {
    /// Ray parameter of the crossing.
    pub t: f64,
    /// The crossing point.
    pub point: Point3,
    /// Unit surface normal at the point (the surface's own orientation: outward for analytic
    /// closed shapes, gradient direction for implicit forms).
    pub normal: Vector3,
}

impl RayHit {
    /// True if the ray crosses from the negative to the positive side of the surface
    /// (`dir . normal > 0`), i.e. it is leaving the region the normal points away from.
    pub fn leaving(&self, ray: &Ray) -> bool {
        ray.dir.dot(self.normal) > 0.0
    }
}
