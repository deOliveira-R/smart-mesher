//! Errors of the triangle schemes.

use sm_scheme_api::{FailureKind, NodeKey};

/// Why triangulating a face failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum TriError {
    /// A lifted boundary loop does not close.
    #[error("boundary loop {ring} does not close (gap {gap:e} of the uv extent)")]
    OpenLoop {
        /// Loop index (0 = outer).
        ring: u16,
        /// Closing gap relative to the uv extent.
        gap: f64,
    },
    /// Two input segments cross (indices into `Pslg::segments`).
    #[error("segments {a} and {b} cross")]
    SelfIntersection {
        /// First segment.
        a: u32,
        /// Second segment.
        b: u32,
    },
    /// A boundary node could not be lifted into the chart.
    #[error("node {key:?} cannot be lifted into the face chart")]
    Lift {
        /// The node.
        key: NodeKey,
    },
    /// The point budget was exhausted before the mesh met its criteria.
    #[error("point budget of {limit} exhausted")]
    TooManyPoints {
        /// The budget (`max_points`).
        limit: u32,
    },
    /// The mesh is below the quality gate.
    #[error("minimum angle {min_angle:.3} deg is below the gate {limit:.3} deg")]
    Quality {
        /// Achieved minimum 3-D angle, degrees.
        min_angle: f64,
        /// The gate, degrees.
        limit: f64,
        /// Chart point ids of the worst triangle.
        worst: [u32; 3],
    },
    /// The advancing front got stuck (stretch scheme).
    #[error("advancing front stuck with {front_edges} front edges")]
    Stuck {
        /// Front edges left.
        front_edges: usize,
    },
    /// Malformed input (indices out of range, degenerate segment, non-finite value).
    #[error("bad input: {0}")]
    BadInput(String),
    /// A coordinate is outside the range in which the exact predicates are valid.
    #[error("point {point} is non-finite or outside [2^-100, 2^100]")]
    OutOfRange {
        /// The point.
        point: u32,
    },
    /// Two input points coincide.
    #[error("points {a} and {b} coincide")]
    DuplicatePoint {
        /// First point.
        a: u32,
        /// Second point.
        b: u32,
    },
    /// The boundary segments do not bound a consistent region (an open chain).
    #[error("inconsistent boundary: {0}")]
    Inconsistent(String),
    /// The face needs a feature this scheme does not implement.
    #[error("unsupported: {0}")]
    Unsupported(String),
    /// The input is geometrically degenerate (collinear points, zero-area face, repeated key).
    #[error("degenerate input: {0}")]
    Degenerate(String),
}

impl TriError {
    /// The driver failure kind this error maps to.
    pub fn kind(&self) -> FailureKind {
        match self {
            TriError::Unsupported(_) => FailureKind::Unsupported,
            TriError::Quality { .. } | TriError::TooManyPoints { .. } | TriError::Stuck { .. } => {
                FailureKind::Quality
            }
            _ => FailureKind::Geometry,
        }
    }
}
