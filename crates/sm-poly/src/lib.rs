//! Tet to polyhedral conversion: the barycentric dual of a conformal tetrahedron mesh, driven by
//! the model classification (see `DESIGN.md`). A mixed mesh (tets plus hexes, wedges, pyramids)
//! keeps its non-tet cells as polyhedra and cuts each transition triangle into dual sub-faces.
//!
//! | Item | What |
//! |---|---|
//! | [`dualize`] | the library entry point over any [`MeshBuild`](sm_mesh_api::MeshBuild) |
//! | [`PolyOptions`], [`FaceMode`], [`DualPoints`], [`DualReport`], [`PolyError`] | options, report, errors |
//! | [`EdgeResolver`], [`VertexClassResolver`], [`TopologyResolver`] | feature-curve identity of boundary edges |
//! | [`GENERATOR_TAGS`] | cell tags holding the generator point of a circumcentric dual |
//! | [`poly_stats`], [`PolyStats`] | statistics of a polyhedral mesh |
//! | [`PolyPlugin`] | registers op `mesh.to_poly` and query `mesh.poly_info` (no layers) |
//!
//! Install [`PolyPlugin`] after `sm_layer_api::CoreLayers`.

mod agglom;
mod circum;
mod concave;
mod dual;
mod error;
mod geom;
mod measure;
mod mixed;
mod ops;
mod options;
mod polydata;
mod prep;
mod resolver;
mod stats;

#[cfg(feature = "test-hooks")]
pub use agglom::with_unchecked_merges;
pub use dual::dualize;
#[cfg(feature = "test-hooks")]
#[doc(hidden)]
pub use dual::{with_gate_off_circumcentric, with_ungated_circumcentric};
#[doc(hidden)]
pub use dual::dualize_with_rounds;
pub use error::PolyError;
pub use measure::{DualEligibility, dual_eligibility};
pub use ops::{PolyInfo, ToPoly};
pub use options::{Agglomerate, DualPoints, DualReport, FaceMode, PolyOptions};
pub use resolver::{EdgeResolver, TopologyResolver, VertexClassResolver};
pub use stats::{GENERATOR_TAGS, PolyStats, Spread, WEDGE_TAG, face_nonplanarity, poly_stats};

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
