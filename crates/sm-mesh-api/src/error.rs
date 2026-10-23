//! Typed errors for builders, edits and algorithms.

use crate::index::{CellIx, FaceIx, MeshEntity, VertexIx};
use crate::validate::Violation;
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
    /// An entity has no classification at `finish` and none could be derived.
    #[error("{0:?} is unclassified")]
    Unclassified(MeshEntity),
    /// An edge (named by its vertex pair, since no table exists yet) has no explicit class and
    /// none could be derived.
    #[error("edge {0:?}-{1:?} is unclassified")]
    UnclassifiedEdge(VertexIx, VertexIx),
    /// A classification names a non-model entity or carries a parameter of the wrong kind.
    #[error("bad classification: {0}")]
    BadClassification(String),
    /// Two classifications had to be merged and no rule resolved them.
    #[error("classification conflict: {0}")]
    ClassConflict(String),
    /// A shape tag and the topology disagree.
    #[error("shape mismatch for cell {0:?}: {1}")]
    ShapeMismatch(CellIx, String),
    /// A named set or tag already exists.
    #[error("duplicate name '{0}'")]
    DuplicateName(String),
    /// A builder or edit call was not valid in the current state.
    #[error("invalid state: {0}")]
    State(String),
    /// A face and a cell are not incident.
    #[error("cell {1:?} is not incident to face {0:?}")]
    NotIncident(FaceIx, CellIx),
    /// The requested feature is outside v1 (for example high-order degree above 2).
    #[error("unsupported: {0}")]
    Unsupported(String),
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
    #[error("mesh failed validation ({} violation(s)); first: {}", .0.len(), .0.first().map_or(String::new(), |v| v.to_string()))]
    Invalid(Vec<Violation>),
    /// A size limit was exceeded (faces above 2^31, indices above `u32::MAX - 1`).
    #[error("mesh too large")]
    TooLarge,
    /// Serial space exhausted for an entity kind.
    #[error("serial space exhausted")]
    SerialOverflow,
}

/// Result alias.
pub type Result<T> = core::result::Result<T, MeshError>;
