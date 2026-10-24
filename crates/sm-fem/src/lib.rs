//! Blocks, sidesets, deck lowering and export (W2 task T8).
//!
//! | Item | What |
//! |---|---|
//! | [`FemPlugin`] | registers the [`FemLayer`] and the `fem.*` ops |
//! | [`FemLayer`] | element type per block color, side preference per sideset face |
//! | [`lower`] | blocks, sidesets and decks to mesh sets (export time, never stored) |
//! | [`export`] | `(state, request)` to `.smsh` / VTU bytes, a pure function |
//!
//! See `DESIGN.md` for the positioning and the rules.

mod element;
mod error;
mod export;
mod layer;
mod lower;
mod ops;

pub use element::ElementType;
pub use error::FemError;
pub use export::{ExportFormat, ExportReport, ExportRequest, export};
pub use layer::{FEM_LAYER, FemLayer};
pub use lower::{Lowered, SetOrigin, SetSummary, lower};
pub use ops::{BlockAdd, BlockElementType, BlockRemove, SidesetAdd};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The FEM plugin: registers [`FemLayer`] and the ops `fem.block_add`, `fem.block_remove`,
/// `fem.block_element_type` and `fem.sideset_add`.
///
/// The `model` and `meshes` layers come from `sm_layer_api::CoreLayers`, which must be installed
/// as well.
#[derive(Clone, Copy, Debug, Default)]
pub struct FemPlugin;

impl Plugin for FemPlugin {
    fn id(&self) -> &str {
        "fem"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register_layer::<FemLayer>()?;
        registry.register::<BlockAdd>()?;
        registry.register::<BlockRemove>()?;
        registry.register::<BlockElementType>()?;
        registry.register::<SidesetAdd>()?;
        Ok(())
    }
}
