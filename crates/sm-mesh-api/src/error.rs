//! Typed errors for builders, edits and algorithms.

use crate::index::{CellIx, FaceIx, MeshEntity, VertexIx};
use sm_core::EntityId;

/// Errors from building, editing, finishing and combining meshes.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum MeshError {
    /// A face loop has fewer than three vertices or repeats a vertex.
    #[error("invalid face loop: {0}")]
    InvalidLoop(String),
    /// An index does not exist.
    #[error("index out of range: {0}")]
    OutOfRange(String),
    /// A face is referenced by no cell.
    #[error("face {0:?} is attached to no cell")]
    DanglingFace(FaceIx),
    /// A face is referenced by more than two cells.
    #[error("face {0:?} is attached to more than two cells")]
    NonManifoldFace(FaceIx),
    /// The two cells (or the single cell) of a face disagree about orientation.
    #[error("face {0:?} has inconsistent orientation between its cells")]
    InconsistentOrientation(FaceIx),
    /// Two faces have the same cells and the same loop.
    #[error("duplicate face {0:?}")]
    DuplicateFace(FaceIx),
    /// A cell lists the same face twice.
    #[error("cell {0:?} lists a face twice")]
    RepeatedCellFace(CellIx),
    /// An entity has no classification at `finish`.
    #[error("{0:?} is unclassified")]
    Unclassified(MeshEntity),
    /// A shape tag and the topology disagree.
    #[error("shape mismatch for cell {0:?}: {1}")]
    ShapeMismatch(CellIx, String),
    /// A named set or tag already exists.
    #[error("duplicate name '{0}'")]
    DuplicateName(String),
    /// A builder or edit call was not valid in the current state.
    #[error("invalid state: {0}")]
    State(String),
    /// A mesh in a catalog or view is missing.
    #[error("unknown mesh {0:?}")]
    UnknownMesh(EntityId),
    /// A hierarchy map is inconsistent with its meshes.
    #[error("hierarchy: {0}")]
    Hierarchy(String),
    /// A vertex was referenced that does not exist.
    #[error("unknown vertex {0:?}")]
    UnknownVertex(VertexIx),
    /// The finished mesh failed validation.
    #[error("mesh failed validation: {0}")]
    Invalid(String),
    /// Serial space exhausted for an entity kind.
    #[error("serial space exhausted")]
    SerialOverflow,
}

/// Result alias.
pub type Result<T> = core::result::Result<T, MeshError>;
