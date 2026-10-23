//! Tolerance policy.
//!
//! One global [`ModelTolerance`] per model (ACIS style). Topological identity (same vertex, same
//! edge) is decided against it. Merge and imprint ops take their own, larger, *operation* tolerance
//! as an argument; it is never stored on entities.

use sm_core::Point3;

/// Global modeling tolerance, fixed for the lifetime of a model and part of its state hash.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ModelTolerance {
    /// Linear resolution in model units: points closer than this are the same point.
    pub resolution: f64,
    /// Angular resolution in radians: directions closer than this are parallel.
    pub angular: f64,
    /// Relative parametric tolerance (fraction of a parameter range).
    pub param_rel: f64,
}

impl Default for ModelTolerance {
    fn default() -> Self {
        Self { resolution: 1e-6, angular: 1e-9, param_rel: 1e-12 }
    }
}

impl ModelTolerance {
    /// Tolerance scaled to a model of the given characteristic extent (resolution = extent * 1e-9,
    /// never below 1e-12).
    pub fn for_extent(extent: f64) -> Self {
        Self { resolution: (extent.abs() * 1e-9).max(1e-12), ..Self::default() }
    }

    /// True if two points coincide within [`Self::resolution`].
    pub fn same_point(&self, a: Point3, b: Point3) -> bool {
        (a - b).norm() <= self.resolution
    }

    /// Grid size used when quantizing coordinates for cross-platform state hashes
    /// (one hundredth of the resolution).
    pub fn hash_quantum(&self) -> f64 {
        self.resolution * 0.01
    }
}
