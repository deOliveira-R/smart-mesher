//! View-data buffers for the web viewer (WebGL2 client lives in `web/`, Q8).
//!
//! * [`skin`] turns a [`MeshRead`](sm_mesh_api::MeshRead) into an indexed triangle soup of its
//!   visible boundary, with an exact cell-based clip, volume filter, interface double-siding and
//!   polygon outline. No encoding: the library form used by tests and native renderers.
//! * [`ViewPlugin`] registers three read-only queries over it: `view.scene`, `view.mesh_skin`
//!   (base64 buffers) and `view.mesh_scalar` (per-cell `sm-quality` values as `f32`).
//! * [`assets`] is the built web client, for the server to serve.
//!
//! Nothing here mutates state or judges validity: quality values come from `sm-quality` and
//! findings from `mesh.validate` / `mesh.quality`. See `DESIGN.md` for buffer layouts and rules.
#![forbid(unsafe_code)]

mod assets;
pub mod b64;
mod error;
mod queries;
mod skin;

pub use assets::{Asset, assets};
pub use error::{TOO_LARGE_HINT, ViewError};
pub use queries::{
    BodyOut, CellRef, ClipIn, MeshOut, PALETTE_LEN, SCHEMA_VERSION, SceneQuery, SceneReply, ScalarQuery, ScalarReply,
    SkinQuery, SkinReply, VolumeOut, WorstCell, mesh_digest, metric_names, palette_rgb,
};
pub use skin::{Clip, NO_FACE, Skin, SkinOptions, skin};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The view plugin: registers `view.scene`, `view.mesh_skin` and `view.mesh_scalar`. It registers
/// no ops and no layers; it reads the `model` and `meshes` layers that
/// `sm_layer_api::CoreLayers` installs.
#[derive(Clone, Copy, Debug, Default)]
pub struct ViewPlugin;

impl Plugin for ViewPlugin {
    fn id(&self) -> &str {
        "view"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register_query::<SceneQuery>()?;
        registry.register_query::<SkinQuery>()?;
        registry.register_query::<ScalarQuery>()?;
        Ok(())
    }
}
