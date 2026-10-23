//! Errors for shape operations.

/// Failure of a shape operation. Inputs never panic; they return one of these.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum GeomError {
    /// A parameter lies outside the (non-periodic) domain.
    #[error("parameter {0} is outside the domain")]
    OutOfDomain(f64),
    /// The operation is ill-defined at this location (pole, apex, zero-length tangent).
    #[error("singular configuration: {0}")]
    Singular(&'static str),
    /// An iterative solver did not converge.
    #[error("iteration did not converge: {0}")]
    NoConvergence(&'static str),
    /// This implementor does not provide the operation (callers fall back, e.g. to facets).
    #[error("operation not supported: {0}")]
    Unsupported(&'static str),
    /// A constructor received degenerate or inconsistent data.
    #[error("invalid geometry: {0}")]
    Invalid(&'static str),
}
