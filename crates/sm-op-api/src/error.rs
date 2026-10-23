//! Error types for the op and timeline APIs.

use crate::eid::Eid;
use crate::hash::Hash;
use crate::toml::TomlError;
use crate::value::ValueError;

/// A layer rejected a patch. A layer must leave itself unchanged when it returns this.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum LayerError {
    /// The patch is not valid for the layer's current content.
    #[error("invalid patch: {0}")]
    Invalid(String),
    /// The patch could not be (de)serialized.
    #[error(transparent)]
    Value(#[from] ValueError),
}

/// An op failed. The applying machinery rolls back everything the op did.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum OpError {
    /// Parameters are semantically invalid (e.g. negative size).
    #[error("invalid op: {0}")]
    Invalid(String),
    /// An id parameter does not exist in the state.
    #[error("unknown entity {0:?}")]
    UnknownEntity(Eid),
    /// The state has no layer with this name (concept not installed).
    #[error("no such layer `{0}`")]
    NoSuchLayer(String),
    /// The layer exists but is not of the requested Rust type.
    #[error("layer `{0}` has a different type")]
    LayerType(String),
    /// Cancelled through the cancel token.
    #[error("cancelled")]
    Cancelled,
    /// A layer rejected a patch.
    #[error(transparent)]
    Layer(#[from] LayerError),
    /// Serialization problem.
    #[error(transparent)]
    Value(#[from] ValueError),
}

impl OpError {
    /// Shorthand for [`OpError::Invalid`].
    pub fn invalid(msg: impl Into<String>) -> Self {
        OpError::Invalid(msg.into())
    }
}

/// Applying an op record to a state failed.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum ApplyError {
    /// No handler registered for this op kind.
    #[error("unknown op kind `{0}`")]
    UnknownOp(String),
    /// Params do not match the op's type.
    #[error("bad params for `{kind}`: {source}")]
    BadParams {
        /// Op kind.
        kind: String,
        /// Underlying error.
        source: ValueError,
    },
    /// Record is newer than this build understands.
    #[error("op `{kind}` version {found} is newer than supported {supported}")]
    UnsupportedVersion {
        /// Op kind.
        kind: String,
        /// Version in the record.
        found: u32,
        /// Highest version registered.
        supported: u32,
    },
    /// Record is older and no migration step exists.
    #[error("no migration for `{kind}` from version {from}")]
    NoMigration {
        /// Op kind.
        kind: String,
        /// Version without a migration.
        from: u32,
    },
    /// The op itself failed (state was rolled back).
    #[error(transparent)]
    Op(#[from] OpError),
    /// Rolling back after a failure itself failed. The state must be discarded.
    #[error("rollback failed (state is poisoned): {0}")]
    RollbackFailed(String),
    /// A recorded delta does not apply to this state.
    #[error("delta mismatch: {0}")]
    Delta(String),
}

/// Registering ops or layers failed.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum RegistryError {
    /// An op kind is already registered.
    #[error("op kind `{0}` already registered")]
    DuplicateOp(String),
    /// A layer name is already registered.
    #[error("layer `{0}` already registered")]
    DuplicateLayer(String),
    /// Migration refers to an unknown op kind or bad version.
    #[error("bad migration for `{0}`")]
    BadMigration(String),
}

/// Timeline, store or workspace failure.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum TimelineError {
    /// Applying an op failed (nothing was changed or recorded).
    #[error(transparent)]
    Apply(#[from] ApplyError),
    /// No node with this hash.
    #[error("unknown node {0}")]
    UnknownNode(Hash),
    /// No ref with this name.
    #[error("unknown ref `{0}`")]
    UnknownRef(String),
    /// A ref or branch already exists.
    #[error("ref `{0}` already exists")]
    RefExists(String),
    /// Backing store failure.
    #[error("store: {0}")]
    Store(String),
    /// Stored data is inconsistent (bad hash, malformed file).
    #[error("corrupt: {0}")]
    Corrupt(String),
    /// TOML problem.
    #[error(transparent)]
    Toml(#[from] TomlError),
    /// Value conversion problem.
    #[error(transparent)]
    Value(#[from] ValueError),
}
