//! Firmness: how strongly a mesh attribute was asked for (Cubit's default/soft/hard).

use serde::{Deserialize, Serialize};

/// How strongly an attribute (scheme, interval count, vertex type) was requested.
///
/// The solver may change `Default` and `Soft` values to satisfy other constraints (`Soft`
/// is preferred over `Default` when two requests disagree); a `Hard` value is never changed,
/// and two disagreeing `Hard` values make the problem infeasible.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Default, Serialize, Deserialize,
)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub enum Firmness {
    /// Set automatically (sizing, auto selection, corner picking).
    #[default]
    Default,
    /// A preference: kept unless it conflicts with a hard request or an imposition.
    Soft,
    /// A requirement: never changed; conflicts are errors.
    Hard,
}

/// A value together with its firmness.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct Firm<T> {
    /// The requested value.
    pub value: T,
    /// How strongly it was requested.
    pub firmness: Firmness,
}

impl<T> Firm<T> {
    /// A hard request.
    pub const fn hard(value: T) -> Self {
        Self {
            value,
            firmness: Firmness::Hard,
        }
    }
    /// A soft request.
    pub const fn soft(value: T) -> Self {
        Self {
            value,
            firmness: Firmness::Soft,
        }
    }
    /// An automatic (default-firmness) value.
    pub const fn auto(value: T) -> Self {
        Self {
            value,
            firmness: Firmness::Default,
        }
    }
    /// True if the value must not be changed.
    pub fn is_hard(&self) -> bool {
        self.firmness == Firmness::Hard
    }
}
