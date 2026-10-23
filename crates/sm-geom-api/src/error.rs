//! Errors returned by shape queries.

use thiserror::Error;

/// Error from a geometric query. Library code never panics on bad input; it returns one of these.
#[derive(Clone, Debug, PartialEq, Error)]
#[non_exhaustive]
pub enum GeomError {
    /// The query hit a singular point (pole, apex, zero tangent) where the result is undefined.
    #[error("singular point at parameter ({u}, {v})")]
    Singular {
        /// First parameter.
        u: f64,
        /// Second parameter (0 for curves).
        v: f64,
    },
    /// Parameter outside the domain of a non-periodic direction.
    #[error("parameter {0} outside domain")]
    OutOfDomain(f64),
    /// An iterative solver did not converge within its budget.
    #[error("iteration did not converge")]
    NoConvergence,
    /// Degenerate input (zero-length axis, non-positive radius, ...).
    #[error("degenerate input: {0}")]
    Degenerate(&'static str),
    /// Malformed NURBS data.
    #[error("invalid NURBS: {0}")]
    InvalidNurbs(String),
}
