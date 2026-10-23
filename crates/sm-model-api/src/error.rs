//! Typed errors for the model-semantics layer.

use sm_core::{EntityId, EntityKind};

/// Everything that can go wrong when editing or deriving from the model tables.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum ModelError {
    /// An id was used that is not present in the relevant table.
    #[error("unknown entity {0}")]
    Unknown(EntityId),
    /// An id had the wrong [`EntityKind`] for the slot it was used in.
    #[error("entity {id} has kind {found:?}, expected {expected:?}")]
    WrongKind {
        /// Offending id.
        id: EntityId,
        /// Kind the slot requires.
        expected: EntityKind,
        /// Kind the id actually has.
        found: EntityKind,
    },
    /// A color of the wrong domain (volume vs boundary) was used.
    #[error("color {0} has the wrong domain for this use")]
    ColorDomain(EntityId),
    /// An entity already has a conflicting assignment.
    #[error("{0} is already assigned")]
    Duplicate(EntityId),
    /// A generated group exceeded the allowed element count (or is infinite).
    #[error("group has more than {limit} elements (infinite or too large)")]
    GroupTooLarge {
        /// The element budget that was exceeded.
        limit: usize,
    },
    /// A translation generator needs an explicit repeat count to unfold.
    #[error("generator {0} is a translation and needs an explicit repeat count")]
    MissingRepeats(EntityId),
    /// A transform could not be inverted.
    #[error("transform is singular")]
    Singular,
    /// A prototype/lattice/group reference cycle.
    #[error("reference cycle through {0}")]
    Cycle(EntityId),
    /// A structurally invalid request, with an explanation.
    #[error("invalid request: {0}")]
    Invalid(String),
    /// The operation needs a symmetry to be declared first.
    #[error("no symmetry declared")]
    NoSymmetry,
    /// The 56-bit serial space of a kind is exhausted.
    #[error("id space exhausted for {0:?}")]
    IdExhausted(EntityKind),
}

/// Result alias for this crate.
pub type Result<T> = core::result::Result<T, ModelError>;
