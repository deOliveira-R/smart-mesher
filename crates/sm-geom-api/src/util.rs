//! Small private vector helpers (sm-core's `Vector3` is deliberately minimal).

use core::f64::consts::TAU;
use sm_core::{Point3, Vector3};

/// Normalize, or `None` if zero/non-finite.
pub(crate) fn unit(v: Vector3) -> Option<Vector3> {
    let n = v.norm();
    if n > 0.0 && n.is_finite() { Some(v * (1.0 / n)) } else { None }
}

/// Point as position vector.
pub(crate) fn pos(p: Point3) -> Vector3 {
    Vector3::new(p.x, p.y, p.z)
}

/// Wrap an angle into `[0, 2*pi)`.
pub(crate) fn wrap_tau(a: f64) -> f64 {
    let t = a.rem_euclid(TAU);
    if t >= TAU { 0.0 } else { t }
}

/// `a*x + b*y` helper for frame combinations.
pub(crate) fn comb2(a: f64, x: Vector3, b: f64, y: Vector3) -> Vector3 {
    x * a + y * b
}

/// `a*x + b*y + c*z`.
pub(crate) fn comb3(a: f64, x: Vector3, b: f64, y: Vector3, c: f64, z: Vector3) -> Vector3 {
    x * a + y * b + z * c
}

/// Deterministic LCG in `[0,1)` for tests.
#[cfg(test)]
pub(crate) struct Lcg(pub u64);
#[cfg(test)]
impl Lcg {
    pub fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }
}
