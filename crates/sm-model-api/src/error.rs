//! Typed errors for the model-semantics layer.

use sm_core::{EntityId, EntityKind};

/// Errors returned by model-semantics operations.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum ModelError {
    /// An id of the wrong kind was given for this selection or slot.
    #[error("expected a {expected:?} id, found {found}")]
    KindMismatch {
        /// The kind that was required.
        expected: EntityKind,
        /// The offending id.
        found: EntityId,
    },
    /// An id is not known to the model or to the topology.
    #[error("unknown entity {0}")]
    UnknownEntity(EntityId),
    /// A member already belongs to an exclusive selection (colors partition, boundary kinds are exclusive).
    #[error("{member} already belongs to {existing}; cannot also add it to {wanted}")]
    Conflict {
        /// The contested member.
        member: EntityId,
        /// The selection that already owns it.
        existing: EntityId,
        /// The selection that tried to claim it.
        wanted: EntityId,
    },
    /// A name is already in use.
    #[error("duplicate name {0:?}")]
    DuplicateName(String),
    /// Two selections with different lifetimes (geometry-bound vs mesh-bound, or different meshes) were combined.
    #[error("selection lifetimes differ")]
    LifetimeMismatch,
    /// The selection operation is not valid for this selection kind or lifetime.
    #[error("invalid selector: {0}")]
    InvalidSelector(String),
    /// The 56-bit serial space of a kind is exhausted.
    #[error("serial overflow for {0:?}")]
    SerialOverflow(EntityKind),
    /// A transform has no inverse.
    #[error("singular transform")]
    SingularTransform,
    /// A generator is malformed (zero axis, order < 2, zero vector...).
    #[error("bad generator: {0}")]
    BadGenerator(String),
    /// The group closure exceeded the configured element cap.
    #[error("group has more than {limit} elements")]
    GroupTooLarge {
        /// The cap.
        limit: usize,
    },
    /// A translation generator needs an explicit repeat count to unfold.
    #[error("translation generator {index} has no repeat count")]
    UnboundedGenerator {
        /// Index into the generator list.
        index: usize,
    },
    /// A symmetry candidate was declared without being verified.
    #[error("symmetry candidate is not verified")]
    NotVerified,
    /// Deck faces could not be paired under the transform.
    #[error("cannot pair deck face {face}")]
    PairingFailed {
        /// A face with no partner.
        face: EntityId,
    },
    /// No domain-boundary face was found on a cut plane.
    #[error("no domain-boundary faces lie on cut plane {index}")]
    NoCutFaces {
        /// Index of the half-space.
        index: usize,
    },
    /// An operation needs a declared symmetry.
    #[error("model has no symmetry")]
    NoSymmetry,
    /// A reference cycle (nested lattices, nested groups).
    #[error("reference cycle through {0}")]
    Cycle(EntityId),
    /// A decomposition request is inconsistent.
    #[error("decomposition: {0}")]
    Decomposition(String),
    /// Anything else that is a caller error.
    #[error("{0}")]
    Invalid(String),
}
