//! Errors of lowering and export.

use sm_core::EntityId;
use sm_mesh_api::{CellShape, MeshError};
use sm_meshio::MeshioError;

use crate::element::ElementType;

/// Errors of [`lower`](crate::lower) and [`export`](crate::export).
///
/// [`FemError::code`] gives a stable kebab-case code for protocols and front ends.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum FemError {
    /// The state has no `meshes` layer.
    #[error("the state has no `meshes` layer")]
    NoMeshLayer,
    /// The state holds no mesh.
    #[error("the state holds no mesh to export")]
    NoMesh,
    /// No mesh was named and the state holds several; name one.
    #[error("the state holds {0} meshes; name the mesh to export")]
    AmbiguousMesh(usize),
    /// The requested mesh is not in the layer.
    #[error("unknown mesh {0}")]
    UnknownMesh(EntityId),
    /// The mesh no longer matches the model (its volumes were changed or deleted), so block and
    /// sideset membership cannot be derived from it.
    #[error("mesh {0} is stale; delete it and mesh again")]
    StaleMesh(EntityId),
    /// A block declares an element type that does not match the shape of its cells.
    #[error("block '{block}' declares {declared} but cell {cell} is a {found:?}")]
    ElementMismatch {
        /// The block (color) name.
        block: String,
        /// The declared element type.
        declared: ElementType,
        /// The index of the first offending cell.
        cell: u32,
        /// Its shape.
        found: CellShape,
    },
    /// The layer holds something the lowering cannot express.
    #[error("unsupported: {0}")]
    Unsupported(String),
    /// A mesh edit failed (for example two sets with the same name).
    #[error("mesh: {0}")]
    Mesh(#[from] MeshError),
    /// The writer failed.
    #[error("mesh writer: {0}")]
    Meshio(#[from] MeshioError),
    /// Writing to the sink failed.
    #[error("i/o: {0}")]
    Io(#[from] std::io::Error),
}

impl FemError {
    /// Stable kebab-case code for protocols and front ends.
    pub fn code(&self) -> &'static str {
        match self {
            Self::NoMeshLayer => "no-mesh-layer",
            Self::NoMesh => "no-mesh",
            Self::AmbiguousMesh(_) => "ambiguous-mesh",
            Self::UnknownMesh(_) => "unknown-mesh",
            Self::StaleMesh(_) => "stale-mesh",
            Self::ElementMismatch { .. } => "element-mismatch",
            Self::Unsupported(_) => "unsupported",
            Self::Mesh(_) => "mesh",
            Self::Meshio(_) => "meshio",
            Self::Io(_) => "io",
        }
    }
}
