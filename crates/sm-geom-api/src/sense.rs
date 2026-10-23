//! Orientation sense shared by geometry and topology.

/// Relative orientation of two oriented things (an edge vs its curve, a face vs its surface,
/// a coedge vs its edge).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Sense {
    /// Same direction.
    Forward,
    /// Opposite direction.
    Reversed,
}

impl Sense {
    /// The opposite sense.
    pub const fn flip(self) -> Self {
        match self {
            Self::Forward => Self::Reversed,
            Self::Reversed => Self::Forward,
        }
    }
    /// `+1.0` for `Forward`, `-1.0` for `Reversed`.
    pub const fn sign(self) -> f64 {
        match self {
            Self::Forward => 1.0,
            Self::Reversed => -1.0,
        }
    }
    /// Composition: `Forward` is the identity.
    pub const fn compose(self, other: Self) -> Self {
        match (self, other) {
            (Self::Forward, s) | (s, Self::Forward) => s,
            _ => Self::Forward,
        }
    }
}
