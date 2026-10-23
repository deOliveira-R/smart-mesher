//! Size fields: the sizing layer's continuous answer to "how big should elements be here".
//!
//! Interval counts come from the interval solve; size fields shape distributions inside a curve
//! (curvature, bias towards a small neighbour) and drive unstructured schemes (pave, tri, tet).
//! Implementations (constant, curvature, linear, skeleton, field-from-file) live in `sm-sizing`.

use sm_core::Point3;

/// A symmetric 3x3 metric tensor, row-major upper triangle `[m00, m01, m02, m11, m12, m22]`.
pub type SymMat3 = [f64; 6];

/// Target element size as a function of position. Must be deterministic and thread-safe.
pub trait SizeField: Send + Sync {
    /// Isotropic target edge length at `p` (finite, positive).
    fn size_at(&self, p: Point3) -> f64;

    /// Anisotropic metric at `p` (unit edges in the metric have the target length). The default
    /// is isotropic: `I / size^2`.
    fn metric_at(&self, p: Point3) -> SymMat3 {
        let h = self.size_at(p);
        let m = 1.0 / (h * h);
        [m, 0.0, 0.0, m, 0.0, m]
    }
}

/// The same size everywhere.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConstantSize(pub f64);

impl SizeField for ConstantSize {
    fn size_at(&self, _p: Point3) -> f64 {
        self.0
    }
}
