//! Typed errors for building, editing and validating meshes.

use sm_core::EntityId;
use thiserror::Error;

use crate::index::Dim;

/// Everything that can go wrong in the mesh contract. Library paths never panic on input.
#[derive(Debug, Error, Clone, PartialEq)]
#[non_exhaustive]
pub enum MeshError {
    /// A shaped cell was given the wrong number of corner vertices.
    #[error("shape {shape} needs {expected} corner vertices, got {got}")]
    BadArity {
        /// Shape name.
        shape: &'static str,
        /// Required corner count.
        expected: usize,
        /// Provided corner count.
        got: usize,
    },
    /// A face loop has fewer than 3 vertices or repeats a vertex.
    #[error("degenerate face loop: {0}")]
    DegenerateLoop(&'static str),
    /// An index points outside its table.
    #[error("{dim:?} index {index} out of range (len {len})")]
    OutOfRange {
        /// Table dimension.
        dim: Dim,
        /// Offending index.
        index: u32,
        /// Table length.
        len: usize,
    },
    /// A face is used by more than two cells.
    #[error("face shared by more than two cells (cell {0})")]
    NonManifoldFace(u32),
    /// Two cells use a shared face with loops that are not mutually reversed.
    #[error("cells {0} and {1} traverse a shared face in the same direction")]
    InconsistentOrientation(u32, u32),
    /// A tag or set name is unknown.
    #[error("unknown name `{0}`")]
    UnknownName(String),
    /// A tag or set name is already defined.
    #[error("duplicate name `{0}`")]
    DuplicateName(String),
    /// A tag was accessed with the wrong type or width.
    #[error("tag `{0}` type or width mismatch")]
    TagMismatch(String),
    /// A classification target has the wrong kind or dimension for the mesh entity.
    #[error("cannot classify {dim:?} entity onto {target:?}")]
    BadClassification {
        /// Mesh entity dimension.
        dim: Dim,
        /// Offending model/partition entity.
        target: EntityId,
    },
    /// The id has the wrong kind for the role it was given.
    #[error("entity id {0:?} has the wrong kind")]
    WrongKind(EntityId),
    /// Hierarchy map does not fit the meshes it joins.
    #[error("level map mismatch: {0}")]
    LevelMap(String),
    /// The operation is not supported for this input.
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
    /// A mesh failed validation (see [`crate::validate`]).
    #[error("invalid mesh: {0}")]
    Invalid(String),
}

/// Result alias for the crate.
pub type MeshResult<T> = Result<T, MeshError>;
