//! Errors from applying an op.

use sm_core::EntityKind;

use crate::hash::CanonError;
use crate::id::Id;

/// Why an op failed. Any error leaves the state exactly as it was before the op.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum OpError {
    /// The op referenced an id that does not exist (never created, or removed).
    #[error("unknown entity {0}")]
    UnknownEntity(Id),
    /// The op referenced an id of the wrong kind.
    #[error("entity {id} has wrong kind, expected {expected:?}")]
    WrongKind {
        /// The offending id.
        id: Id,
        /// The kind the op needed.
        expected: EntityKind,
    },
    /// The op's arguments are invalid (message explains why).
    #[error("invalid op: {0}")]
    Invalid(String),
    /// The op needs an upstream layer store that the state does not provide.
    #[error("upstream layer `{0}` unavailable")]
    MissingUpstream(&'static str),
    /// The caller cancelled a long-running op.
    #[error("operation cancelled")]
    Cancelled,
    /// The 56-bit serial space of a kind is exhausted.
    #[error("id space exhausted for {0:?}")]
    IdSpaceExhausted(EntityKind),
    /// The state could not be canonically hashed (for example it holds a NaN).
    #[error("state not hashable: {0}")]
    Hash(#[from] CanonError),
}
