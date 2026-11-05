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
    /// open path: the region is not edge-manifold (a checkerboard where two volumes alternate
    /// around the edge, two tets of one volume touching only along it) or the input is not
    /// conformal.
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
    /// A dual cell is not star-shaped (or has non-positive volume) and the wedge repair could
    /// not fix it, so the output would fail `validate_geometry`. The cell is the dual of
    /// `vertex` in `region`; `count` is the number of such cells (this is the first, by output
    /// order).
    #[error(
        "dual cell of vertex {} in region {region:?} near ({:.6e}, {:.6e}, {:.6e}) is not star-shaped and could not be repaired ({count} such cell(s))",
        vertex.0, at[0], at[1], at[2]
    )]
    UnrepairedCell {
        /// The primal vertex whose dual cell failed.
        vertex: VertexIx,
        /// The model region of the cell.
        region: EntityId,
        /// Centroid of the failing cell (the cell's vertex mean if its volume is not positive).
        at: [f64; 3],
        /// Number of unrepaired cells.
        count: usize,
    },
    /// After agglomeration a merged cell is not star-shaped or has non-positive volume, so the
    /// output would fail `validate_geometry`. `region` and `at` describe the first such cell
    /// (by output order); `count` is the number of such cells. Lower the merge ratio or the
    /// concavity limit, or disable agglomeration.
    #[error(
        "agglomerated cell in region {region:?} near ({:.6e}, {:.6e}, {:.6e}) is not star-shaped ({count} such cell(s)); reduce the agglomeration ratio or concavity limit",
        at[0], at[1], at[2]
    )]
    InvalidAgglomeration {
        /// The model region of the first failing cell.
        region: EntityId,
        /// Centroid of the first failing cell.
        at: [f64; 3],
        /// Number of failing merged cells.
        count: usize,
    },
    /// The mesh builder refused something.
    #[error(transparent)]
    Mesh(#[from] MeshError),
}
