//! Typed errors.

use crate::Metric;
use sm_mesh_api::CellShape;

/// Errors of the mesh-level quality drivers.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum QualityError {
    /// The metric is not defined for any cell of the mesh (e.g. `Skew` on a tet mesh).
    #[error("metric {metric:?} is not defined for any cell shape present in the mesh")]
    Unsupported {
        /// The requested metric.
        metric: Metric,
    },
    /// A metric was requested for one shape where it is undefined (element-level API).
    #[error("metric {metric:?} is not defined for {shape:?}")]
    NotApplicable {
        /// The requested metric.
        metric: Metric,
        /// The shape it was requested for.
        shape: CellShape,
    },
    /// A caller-supplied option or argument is invalid.
    #[error("invalid argument: {0}")]
    BadArgument(String),
}

/// Result alias for this crate.
pub type Result<T> = core::result::Result<T, QualityError>;
