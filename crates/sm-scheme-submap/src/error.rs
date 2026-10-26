//! The typed error of the pure lattice and block layers.

use thiserror::Error;

/// Why a lattice polygon or a block decomposition could not be built. Every message starts
/// with the layer that rejected the input (`lattice:`), which is also what the driver shows in
/// the attempt history.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum SubmapError {
    /// Fewer than four sides.
    #[error("lattice: {0} sides; a rectilinear polygon needs at least four")]
    TooFewSides(usize),
    /// A side has no intervals.
    #[error("lattice: side {side} has zero length")]
    ZeroLength {
        /// The side.
        side: usize,
    },
    /// The side lengths do not close the loop.
    #[error("lattice: sides do not close (residual ({dx}, {dy}))")]
    NotClosed {
        /// Residual along `i`.
        dx: i64,
        /// Residual along `j`.
        dy: i64,
    },
    /// The signed area is not positive: the face is not on the left of the loop.
    #[error(
        "lattice: not positively oriented (twice the signed area is {area2}); the face must lie on the left of the loop"
    )]
    NotPositive {
        /// Twice the signed lattice area.
        area2: i128,
    },
    /// Two sides that are not neighbours intersect or touch, or two neighbours fold back.
    #[error("lattice: polygon is not simple: side {a} and side {b} intersect or touch")]
    NotSimple {
        /// First side.
        a: usize,
        /// Second side.
        b: usize,
    },
    /// The lattice coordinates overflow `i64`.
    #[error("lattice: coordinates overflow")]
    Overflow,
}
