//! Typed errors of the spacing math.

use sm_scheme_api::FailureKind;

/// Why a spacing could not be computed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum SpacingError {
    /// A parameter is not finite and positive.
    #[error("{what} must be finite and positive, got {value}")]
    BadValue {
        /// Which parameter.
        what: &'static str,
        /// The offending value.
        value: f64,
    },
    /// The interval count is zero.
    #[error("interval count must be at least 1")]
    ZeroCount,
    /// No spacing of this family satisfies the sizes on this curve.
    #[error("{0}")]
    Unreachable(String),
    /// The spacing exists but a consecutive-segment ratio exceeds the allowed maximum.
    #[error("segment ratio {needed:.6} exceeds the allowed maximum {allowed:.6}")]
    RatioLimit {
        /// The ratio the spacing needs.
        needed: f64,
        /// The limit.
        allowed: f64,
    },
    /// The values are degenerate in double precision (coincident nodes, overflow).
    #[error("{0}")]
    Degenerate(String),
    /// The curve defeated the arc-length machinery (zero speed, bad range).
    #[error("geometry: {0}")]
    Geometry(String),
}

impl SpacingError {
    /// The driver failure kind this error maps to.
    pub fn failure_kind(&self) -> FailureKind {
        match self {
            SpacingError::Geometry(_) | SpacingError::Degenerate(_) => FailureKind::Geometry,
            SpacingError::BadValue { .. } | SpacingError::ZeroCount => FailureKind::NotApplicable,
            SpacingError::Unreachable(_) | SpacingError::RatioLimit { .. } => FailureKind::Constraint,
        }
    }
}

/// Result alias of the spacing math.
pub type Result<T> = core::result::Result<T, SpacingError>;
