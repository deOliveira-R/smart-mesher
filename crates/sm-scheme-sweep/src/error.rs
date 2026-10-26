//! Errors of the numerical helpers and of column discovery.

use sm_math::linalg::LinAlgError;

/// Failure of [`crate::fit_affine`], [`crate::place_layers`] or column discovery.
///
/// The scheme maps these to `SchemeError`/`GenerateFailure`; the helpers themselves return them
/// so that they stay usable (and testable) without a driver.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum SweepError {
    /// Fewer points than an affine fit with a well-defined in-plane map needs.
    #[error("affine fit needs at least {need} points, got {have}")]
    TooFewPoints {
        /// Points given.
        have: usize,
        /// Points needed.
        need: usize,
    },
    /// The two point sets differ in length.
    #[error("point sets differ in length ({src} source, {dst} destination)")]
    LengthMismatch {
        /// Source points.
        src: usize,
        /// Destination points.
        dst: usize,
    },
    /// The source points are collinear or coincident (rank of the moment matrix below 2).
    #[error("source points are degenerate (moment-matrix rank {rank}, need at least 2)")]
    Degenerate {
        /// Numerical rank found.
        rank: usize,
    },
    /// A coordinate was NaN or infinite.
    #[error("non-finite coordinate")]
    NonFinite,
    /// Inconsistent arguments to a helper (lengths, layer counts, parameter ranges).
    #[error("invalid input: {0}")]
    Input(String),
    /// Column discovery found no valid sweep between the faces.
    #[error("{0}")]
    NoColumns(String),
    /// A linear-algebra routine failed.
    #[error("linear algebra: {0}")]
    LinAlg(#[from] LinAlgError),
}
