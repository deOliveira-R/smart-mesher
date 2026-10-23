//! Tolerance policy.

use sm_core::Point3;

/// The tolerance policy of a model.
///
/// Topology stores one global [`Tolerance`] plus sparse per-entity linear overrides
/// (see `sm-topo-api`). All geometric "is it the same?" decisions use these numbers, never
/// ad-hoc epsilons.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Tolerance {
    /// Model resolution: two points closer than this are the same point (model length units).
    pub linear: f64,
    /// Two unit directions closer than this angle (radians) are parallel.
    pub angular: f64,
    /// Resolution in parameter space for periodic wrap and seam matching.
    pub parametric: f64,
}

impl Tolerance {
    /// Default policy: 1e-6 length, 1e-9 rad, 1e-9 parameter.
    pub const DEFAULT: Self = Self { linear: 1e-6, angular: 1e-9, parametric: 1e-9 };

    /// True if the two points coincide within [`Self::linear`].
    pub fn coincident(&self, a: Point3, b: Point3) -> bool {
        (a - b).norm() <= self.linear
    }

    /// True if `len` is below the linear resolution.
    pub fn is_zero_length(&self, len: f64) -> bool {
        len.abs() <= self.linear
    }

    /// True if all components are finite and strictly positive.
    pub fn is_valid(&self) -> bool {
        [self.linear, self.angular, self.parametric].iter().all(|v| v.is_finite() && *v > 0.0)
    }

    /// A copy with the linear tolerance replaced (used for per-entity overrides).
    pub fn with_linear(self, linear: f64) -> Self {
        Self { linear, ..self }
    }
}

impl Default for Tolerance {
    fn default() -> Self {
        Self::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coincidence() {
        let t = Tolerance::default();
        assert!(t.is_valid());
        assert!(t.coincident(Point3::new(0.0, 0.0, 0.0), Point3::new(1e-7, 0.0, 0.0)));
        assert!(!t.coincident(Point3::new(0.0, 0.0, 0.0), Point3::new(1e-5, 0.0, 0.0)));
    }
}
