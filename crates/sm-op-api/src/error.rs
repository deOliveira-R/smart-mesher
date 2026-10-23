//! Error types.

use sm_core::EntityId;

use crate::hash::Hash256;
use crate::value::CodecError;

/// Why a single op's `apply` failed. Any `OpError` discards the whole op's effects.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum OpError {
    /// The parameters are semantically invalid (e.g. negative length).
    #[error("invalid op parameters: {0}")]
    Invalid(String),
    /// A referenced entity does not exist in the state.
    #[error("unknown entity {0:?}")]
    UnknownEntity(EntityId),
    /// The op needs a layer that the registry did not install.
    #[error("layer '{0}' is not registered in this state")]
    MissingLayer(&'static str),
    /// The cancellation token fired.
    #[error("operation cancelled")]
    Cancelled,
    /// The 56-bit serial space of an entity kind is exhausted.
    #[error("entity serial space exhausted")]
    IdExhausted,
    /// Any other op-specific failure.
    #[error("{0}")]
    Failed(String),
}

/// Registry construction and lookup errors.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum RegistryError {
    /// Two registrations used the same op name.
    #[error("op '{0}' is already registered")]
    DuplicateOp(String),
    /// Two registrations used the same layer name.
    #[error("layer '{0}' is already registered")]
    DuplicateLayer(&'static str),
    /// No op of this name is registered.
    #[error("unknown op '{0}'")]
    UnknownOp(String),
    /// The stored op version is newer than this build understands.
    #[error("op '{name}' version {found} is newer than supported version {supported}")]
    UnsupportedVersion {
        /// Op name.
        name: String,
        /// Version found in the record.
        found: u32,
        /// Highest version this registry supports.
        supported: u32,
    },
    /// No migration step exists from this version.
    #[error("op '{name}' has no migration from version {from}")]
    MissingMigration {
        /// Op name.
        name: String,
        /// Version without a migration step.
        from: u32,
    },
    /// Parameters could not be decoded.
    #[error("op '{name}' parameters: {source}")]
    Decode {
        /// Op name.
        name: String,
        /// Underlying error.
        source: CodecError,
    },
}

/// Failure applying an op through the [`Engine`](crate::Engine).
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum ApplyError {
    /// The op could not be resolved or decoded.
    #[error(transparent)]
    Registry(#[from] RegistryError),
    /// Encoding the op's parameters failed.
    #[error("encoding op '{name}': {source}")]
    Encode {
        /// Op name.
        name: String,
        /// Underlying error.
        source: CodecError,
    },
    /// The op itself failed; state is unchanged.
    #[error("op '{name}' failed: {source}")]
    Op {
        /// Op name.
        name: String,
        /// Underlying error.
        source: OpError,
    },
}

/// Timeline, replay and storage errors.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum TimelineError {
    /// A node references a parent that is not in the timeline.
    #[error("unknown parent node {0}")]
    UnknownParent(Hash256),
    /// No node with this hash.
    #[error("unknown node {0}")]
    UnknownNode(Hash256),
    /// No ref with this name.
    #[error("unknown ref '{0}'")]
    UnknownRef(String),
    /// The ref already exists.
    #[error("ref '{0}' already exists")]
    RefExists(String),
    /// The ref name is not allowed.
    #[error("invalid ref name '{0}'")]
    BadRefName(String),
    /// There is no current node to branch from.
    #[error("nothing to branch from: the timeline position is empty")]
    EmptyPosition,
    /// Replaying a node produced a different state hash than recorded.
    #[error("replay of node {node} diverged: recorded state {recorded}, got {actual}")]
    Diverged {
        /// Node being replayed.
        node: Hash256,
        /// State hash stored in the node.
        recorded: Hash256,
        /// State hash after replay.
        actual: Hash256,
    },
    /// A stored node's content does not match its hash.
    #[error("node file {expected} has content hashing to {actual}")]
    HashMismatch {
        /// Hash from the file name.
        expected: Hash256,
        /// Hash of the content.
        actual: Hash256,
    },
    /// A node could not be parsed or encoded.
    #[error("node format: {0}")]
    Format(String),
    /// Applying an op during replay or append failed.
    #[error(transparent)]
    Apply(#[from] ApplyError),
    /// Filesystem failure.
    #[error("io: {0}")]
    Io(String),
}

impl From<CodecError> for TimelineError {
    fn from(e: CodecError) -> Self {
        TimelineError::Format(e.0)
    }
}

impl From<std::io::Error> for TimelineError {
    fn from(e: std::io::Error) -> Self {
        TimelineError::Io(e.to_string())
    }
}
