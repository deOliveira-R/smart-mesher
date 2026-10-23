//! Errors for geometry construction and evaluation.

/// Errors from shape construction, evaluation and intersection.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum GeomError {
    /// Construction parameters are invalid (non-positive radius, bad knots, ...).
    #[error("invalid geometry: {0}")]
    Invalid(&'static str),
    /// The query is undefined at a singular point (pole, apex, cusp).
    #[error("singular point: {0}")]
    Singular(&'static str),
    /// A transform that is not a similarity (rotation/mirror/translation + uniform scale).
    #[error("transform is not a similarity")]
    NotSimilarity,
    /// The shape does not support this query (e.g. exact ray hits on NURBS).
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
}
