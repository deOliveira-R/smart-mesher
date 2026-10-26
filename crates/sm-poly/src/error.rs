//! Errors of the tet -> polyhedron conversion.

use sm_core::EntityId;
use sm_mesh_api::{CellIx, FaceIx, MeshError, VertexIx};

/// Why a mesh could not be dualized. All variants are typed input problems except
/// [`PolyError::Mesh`], which wraps a failure of the mesh builder.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum PolyError {
    /// A cell is not a tetrahedron (hex, wedge, pyramid, polyhedron, ...). A mesh that is
    /// already polyhedral fails here.
    #[error("cell {cell:?} is not a tetrahedron")]
    NotTet {
        /// The first offending cell.
        cell: CellIx,
    },
    /// The mesh is a dimension-2 (surface) mesh.
    #[error("a dimension-2 mesh cannot be dualized into polyhedra")]
    Dim2,
    /// A face is neither `Interior` nor `Boundary` (for example `NonConformal`).
    #[error("face {face:?} is not an Interior or Boundary face")]
    UnsupportedFaceKind {
        /// The first offending face.
        face: FaceIx,
    },
    /// The tets of one region around a primal edge do not form a single closed ring or a single
    /// open path (non-manifold or non-conformal input).
    #[error("the tets around edge ({}, {}) form neither one closed ring nor one open path", edge[0].0, edge[1].0)]
    OpenRing {
        /// The primal edge, as its two (ascending) vertices.
        edge: [VertexIx; 2],
    },
    /// A boundary edge between two different model faces could not be mapped to one feature
    /// curve.
    #[error("cannot resolve the feature edge between faces {a:?} and {b:?}")]
    AmbiguousEdge {
        /// The lower face id (or the first of two disagreeing edge classes).
        a: EntityId,
        /// The higher face id (or the second of two disagreeing edge classes).
        b: EntityId,
    },
    /// A boundary vertex has exactly one feature (split) edge around it in one region (a seam
    /// or a dangling curve), which this version does not handle.
    #[error("vertex {vertex:?} has exactly one feature edge in a region (seam or dangling curve)")]
    UnsupportedFeature {
        /// The vertex.
        vertex: VertexIx,
    },
    /// A region-boundary face is classified on something that is not a model face.
    #[error("face {face:?} separates regions but is not classified on a model face")]
    BadClassification {
        /// The face.
        face: FaceIx,
    },
    /// The mesh builder refused something.
    #[error(transparent)]
    Mesh(#[from] MeshError),
}
