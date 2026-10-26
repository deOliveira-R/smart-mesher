//! Tet to polyhedral conversion: the barycentric dual of a conformal all-tetrahedron mesh,
//! driven by the model classification (see `DESIGN.md`).
//!
//! | Item | What |
//! |---|---|
//! | [`dualize`] | the library entry point over any [`MeshBuild`](sm_mesh_api::MeshBuild) |
//! | [`PolyOptions`], [`FaceMode`], [`DualReport`], [`PolyError`] | options, report, errors |
//! | [`EdgeResolver`], [`VertexClassResolver`], [`TopologyResolver`] | feature-curve identity of boundary edges |
//! | [`poly_stats`], [`PolyStats`] | statistics of a polyhedral mesh |
//! | [`PolyPlugin`] | registers op `mesh.to_poly` and query `mesh.poly_info` (no layers) |
//!
//! Install [`PolyPlugin`] after `sm_layer_api::CoreLayers`.

mod agglom;
mod dual;
mod error;
mod geom;
mod ops;
mod options;
mod polydata;
mod prep;
mod resolver;
mod stats;

pub use dual::dualize;
pub use error::PolyError;
pub use ops::{PolyInfo, ToPoly};
pub use options::{Agglomerate, DualReport, FaceMode, PolyOptions};
pub use resolver::{EdgeResolver, TopologyResolver, VertexClassResolver};
pub use stats::{PolyStats, Spread, face_nonplanarity, poly_stats};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `poly` plugin: op `mesh.to_poly` and query `mesh.poly_info`. It registers no layers
/// (`CoreLayers` does).
#[derive(Clone, Copy, Debug, Default)]
pub struct PolyPlugin;

impl Plugin for PolyPlugin {
    fn id(&self) -> &str {
        "poly"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register::<ToPoly>()?;
        registry.register_query::<PolyInfo>()?;
        Ok(())
    }
}
