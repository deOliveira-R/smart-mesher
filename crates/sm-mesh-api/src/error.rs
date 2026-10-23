//! Typed errors of the mesh contract.

use crate::point::Point;
use sm_core::EntityId;

/// Errors returned by builders, hierarchy maps and instancing.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum MeshError {
    /// A point is out of range or of the wrong stratum for the operation.
    #[error("invalid point {0:?}")]
    InvalidPoint(Point),
    /// A face loop has fewer than 3 vertices or repeats a vertex.
    #[error("degenerate face loop")]
    DegenerateLoop,
    /// A cell lists the same face twice.
    #[error("cell lists the same face twice")]
    DuplicateFace,
    /// A cell's shape tag disagrees with its topology.
    #[error("shape tag {tag} does not match topology (detected {detected})")]
    ShapeMismatch {
        /// Declared tag (debug name).
        tag: String,
        /// Detected tag (debug name).
        detected: String,
    },
    /// A face with the same vertex set exists but with a different vertex cycle.
    #[error("face shares a vertex set with an existing face but not its vertex cycle")]
    FaceCycleMismatch,
    /// The cell sees an existing face from the same side as its other cell: a loop is
    /// wound the wrong way (the orientation rule would be violated).
    #[error("face loop winding is inconsistent with the neighbouring cell")]
    InconsistentOrientation,
    /// A face would get a third cell.
    #[error("face already has two cells (non-manifold)")]
    NonManifoldFace,
    /// An edge named by endpoints does not exist.
    #[error("no edge between vertices {0} and {1}")]
    NoSuchEdge(u32, u32),
    /// A classification names something that is not a model entity.
    #[error("{0:?} is not a model entity")]
    NotAModelEntity(EntityId),
    /// A mesh id is not part of the collection / instanced view.
    #[error("unknown mesh {0:?}")]
    UnknownMesh(EntityId),
    /// A fine->coarse map is inconsistent.
    #[error("invalid hierarchy map: {0}")]
    InvalidMap(String),
    /// An instance description is inconsistent.
    #[error("invalid instance: {0}")]
    InvalidInstance(String),
    /// The operation is outside what this implementation supports.
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
}
