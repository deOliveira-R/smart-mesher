//! Typed errors for the model-semantics layer.

use sm_core::{EntityId, EntityKind};

/// Errors returned by model-semantics operations. Library code never panics on bad input.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum ModelError {
    /// An id was not found in the structure it was looked up in.
    #[error("unknown entity {0}")]
    UnknownEntity(EntityId),
    /// An id had the wrong [`EntityKind`] for the slot it was used in.
    #[error("entity {id} has kind {found:?}, expected {expected}")]
    WrongKind {
        /// The offending id.
        id: EntityId,
        /// Its actual kind.
        found: EntityKind,
        /// What was expected.
        expected: &'static str,
    },
    /// A name that must be unique was reused.
    #[error("duplicate name `{0}`")]
    DuplicateName(String),
    /// A geometric input was degenerate (zero normal, zero axis, non-invertible transform...).
    #[error("degenerate geometry: {0}")]
    Degenerate(&'static str),
    /// Group closure did not terminate within the element cap.
    #[error("group closure exceeded {limit} elements")]
    GroupTooLarge {
        /// The cap that was hit.
        limit: usize,
    },
    /// A lattice definition is inconsistent.
    #[error("invalid lattice: {0}")]
    InvalidLattice(String),
    /// Instancing (or group nesting) contains a cycle through this id.
    #[error("cycle through {0}")]
    Cycle(EntityId),
    /// The operation is not valid in the current symmetry state.
    #[error("invalid symmetry state: {0}")]
    State(&'static str),
    /// Any other invalid argument.
    #[error("invalid argument: {0}")]
    Invalid(String),
}

/// Result alias for this crate.
pub type Result<T> = core::result::Result<T, ModelError>;
