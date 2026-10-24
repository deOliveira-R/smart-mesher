//! Errors of the layer contract.

use sm_core::EntityKind;
use sm_geom_api::GeomError;
use sm_mesh_api::MeshError;
use sm_model_api::ModelError;
use sm_op_api::OpError;
use sm_topo_api::TopoError;
use sm_topo_api::fixtures::FixtureError;

/// Why a layer operation failed.
///
/// Converts into [`OpError`] (`OpError::Failed` with the message), so an op body can use `?` on a
/// `Result<_, LayerError>`. Foreign errors ([`TopoError`], [`ModelError`], [`MeshError`],
/// [`GeomError`], [`FixtureError`]) convert into `LayerError`; closures passed to the
/// transaction helpers return `Result<_, OpError>`, so convert with
/// [`LayerResultExt::or_op`] (or `.map_err(LayerError::from)?` inside a function that returns
/// `Result<_, LayerError>`).
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum LayerError {
    /// A topology edit or query failed.
    #[error("topology: {0}")]
    Topo(#[from] TopoError),
    /// A model-table edit failed.
    #[error("model: {0}")]
    Model(#[from] ModelError),
    /// A mesh edit or build failed.
    #[error("mesh: {0}")]
    Mesh(#[from] MeshError),
    /// A geometry store insertion failed.
    #[error("geometry: {0}")]
    Geom(#[from] GeomError),
    /// A reference fixture failed (it wraps topology and geometry errors).
    #[error("fixture: {0}")]
    Fixture(#[from] FixtureError),
    /// A transaction helper was used incorrectly (a previous `topo_txn` was never committed,
    /// a role names an id the transaction did not create, an id is in two roles, ...).
    #[error("layer transaction: {0}")]
    Txn(String),
    /// The id passed to [`MeshLayer::insert`](crate::MeshLayer::insert) is not a `Mesh` id, is
    /// already present, or differs from the id of the mesh it carries.
    #[error("mesh entry id {0}")]
    BadMeshId(String),
    /// The allocator handed to [`MeshLayer::insert`](crate::MeshLayer::insert) is behind the
    /// layer's current one for this kind (serials are never reused).
    #[error("mesh serial allocator moved backwards for {0:?}")]
    AllocRegressed(EntityKind),
    /// The `before` allocator given to [`insert_mesh`](crate::insert_mesh) is not the layer's
    /// current allocator (the caller read it at the wrong time or cloned another one).
    #[error("mesh allocator mismatch: {0}")]
    AllocMismatch(String),
    /// The ids minted between `before` and `after` are not exactly the ids the mesh holds
    /// (a builder over-minted and discarded ids, or the mesh lost some): such ids would be
    /// adopted but never retired.
    #[error("minted ids do not match the mesh: {0}")]
    MintedNotInMesh(String),
}

impl From<LayerError> for OpError {
    fn from(e: LayerError) -> Self {
        OpError::Failed(e.to_string())
    }
}

/// `Result` adapter from any error that converts into [`LayerError`] to [`OpError`].
pub trait LayerResultExt<T> {
    /// Convert the error to an [`OpError`] through [`LayerError`].
    fn or_op(self) -> Result<T, OpError>;
}

impl<T, E: Into<LayerError>> LayerResultExt<T> for Result<T, E> {
    fn or_op(self) -> Result<T, OpError> {
        self.map_err(|e| OpError::from(e.into()))
    }
}
