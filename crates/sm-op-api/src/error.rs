//! Error types.

use sm_core::{EntityId, EntityKind};

use crate::codec::CodecError;
use crate::hash::Hash256;

/// Derived-cache misuse.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum CacheError {
    /// An entry with this `(kind, key)` already holds a value of a different Rust type.
    #[error("derived cache entry '{kind}' already holds a value of another type")]
    TypeConflict {
        /// The cache kind.
        kind: String,
    },
}

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
    /// A referenced entity exists but has the wrong kind.
    #[error("entity {id:?} is not a {expected:?}")]
    WrongKind {
        /// The offending id.
        id: EntityId,
        /// The kind the op required.
        expected: EntityKind,
    },
    /// The op needs a layer that the registry did not install.
    #[error("layer '{0}' is not registered in this state")]
    MissingLayer(String),
    /// The op needs a service that the engine was not given.
    #[error("service '{0}' is not available")]
    MissingService(&'static str),
    /// The cancellation token fired.
    #[error("operation cancelled")]
    Cancelled,
    /// The 56-bit serial space of an entity kind is exhausted.
    #[error("entity serial space exhausted")]
    IdExhausted,
    /// Derived-cache misuse.
    #[error(transparent)]
    Cache(#[from] CacheError),
    /// A nested op (e.g. inside `core.batch`) could not be resolved.
    #[error(transparent)]
    Registry(#[from] RegistryError),
    /// Parameter encoding failed.
    #[error(transparent)]
    Codec(#[from] CodecError),
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
    /// A layer name is not registered.
    #[error("layer '{0}' is not registered")]
    MissingLayer(String),
    /// A layer lists a dependency that is not registered.
    #[error("layer '{layer}' depends on unregistered layer '{dep}'")]
    UnknownLayerDependency {
        /// The layer with the dangling dependency.
        layer: &'static str,
        /// The missing layer name.
        dep: &'static str,
    },
    /// Layer dependencies form a cycle.
    #[error("layer dependencies form a cycle through: {0:?}")]
    LayerCycle(Vec<&'static str>),
    /// A migration for this `(op, from)` already exists.
    #[error("op '{name}' already has a migration from version {from}")]
    DuplicateMigration {
        /// Op name.
        name: String,
        /// Source version.
        from: u32,
    },
    /// The migration's source version is not below the op's current version.
    #[error("op '{name}': cannot migrate from version {from}; current version is {current}")]
    BadMigration {
        /// Op name.
        name: String,
        /// Source version given.
        from: u32,
        /// Current version of the op.
        current: u32,
    },
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
    /// The op's JSON schema could not be produced.
    #[error("op '{name}' schema: {reason}")]
    Schema {
        /// Op name.
        name: String,
        /// What went wrong.
        reason: String,
    },
    /// A persisted layer snapshot could not be restored.
    #[error("layer '{layer}' snapshot: {source}")]
    Snapshot {
        /// Layer name.
        layer: String,
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

impl ApplyError {
    /// True if the op stopped because cancellation was requested.
    pub fn is_cancelled(&self) -> bool {
        matches!(self, ApplyError::Op { source: OpError::Cancelled, .. })
    }
}

/// Timeline, replay and storage-format errors.
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
    /// The ref name is not allowed (bad characters, or it collides with another ref as file/directory).
    #[error("invalid ref name '{0}'")]
    BadRefName(String),
    /// There is no current node to branch from.
    #[error("nothing to branch from: the timeline position is empty")]
    EmptyPosition,
    /// Replaying a node produced a different quantized state hash than recorded.
    #[error("replay of node {node} diverged: recorded state {recorded}, got {actual}")]
    Diverged {
        /// Node being replayed.
        node: Hash256,
        /// State hash stored in the node's check.
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
    /// A stored file could not be parsed or a value could not be encoded.
    #[error("format: {0}")]
    Format(String),
    /// The session moved since the op was prepared.
    #[error("stale base: op was prepared at {expected:?} but the session is at {found:?}")]
    StaleBase {
        /// Position the op was prepared against.
        expected: Option<Hash256>,
        /// Current position.
        found: Option<Hash256>,
    },
    /// A long replay was cancelled.
    #[error("operation cancelled")]
    Cancelled,
    /// The operation needs HEAD attached to a branch.
    #[error("HEAD is detached")]
    NotAttached,
    /// Nothing to undo.
    #[error("nothing to undo")]
    NothingToUndo,
    /// Nothing to redo.
    #[error("nothing to redo")]
    NothingToRedo,
    /// Applying an op during replay or append failed.
    #[error(transparent)]
    Apply(#[from] ApplyError),
}

impl From<CodecError> for TimelineError {
    fn from(e: CodecError) -> Self {
        TimelineError::Format(e.0)
    }
}

impl From<RegistryError> for TimelineError {
    fn from(e: RegistryError) -> Self {
        TimelineError::Apply(e.into())
    }
}
