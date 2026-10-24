//! The intent and meshes plugin and the mesh-driver node (W2 task T7).
//!
//! | Item | What |
//! |---|---|
//! | [`MeshPlugin`] | registers [`IntentLayer`], the `mesh.*` ops and the `mesh.info` query. It does **not** register the `meshes` layer (`sm_layer_api::CoreLayers` does) |
//! | [`MesherConfig`] | the service: scheme registry, interval solver, corner picker, size-field and auto-size hooks, injected by the distribution |
//! | [`IntentLayer`] | the `intent` layer (depends on `model`) |
//! | [`RayonExec`] | the driver [`Executor`](sm_scheme_api::Executor) on the rayon pool |
//! | ops | `mesh.size`, `mesh.intervals`, `mesh.scheme`, `mesh.default_size` (intent), `mesh.run`, `mesh.delete` (meshes) |
//! | query | `mesh.info` |
//!
//! This crate names no scheme crate and no sizing crate: schemes, the solver and the picker all
//! arrive through [`MesherConfig`]. See `DESIGN.md`.
//!
//! # Install order
//! The `intent` and `meshes` layers depend on `model`, so install `CoreLayers` before
//! [`MeshPlugin`], and inject a [`MesherConfig`] service
//! (`Services::new().with_versioned("mesher", 1, cfg)`).

mod config;
mod exec;
mod info;
mod intent_layer;
mod intent_ops;
mod mesh_ops;

pub use config::{AutoSizeFn, DEFAULT_MAX_CELLS, MesherConfig, SizeFieldFn};
pub use exec::RayonExec;
pub use info::{MeshInfo, MeshInfoQuery};
pub use intent_layer::IntentLayer;
pub use intent_ops::{FirmnessArg, MeshDefaultSize, MeshIntervals, MeshScheme, MeshSize};
pub use mesh_ops::{MeshDelete, MeshRun};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The mesher plugin: registers [`IntentLayer`], the `mesh.*` ops and the `mesh.info` query.
#[derive(Clone, Copy, Debug, Default)]
pub struct MeshPlugin;

impl Plugin for MeshPlugin {
    fn id(&self) -> &str {
        "mesher"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register_layer::<IntentLayer>()?;
        registry.register::<MeshSize>()?;
        registry.register::<MeshIntervals>()?;
        registry.register::<MeshScheme>()?;
        registry.register::<MeshDefaultSize>()?;
        registry.register::<MeshRun>()?;
        registry.register::<MeshDelete>()?;
        registry.register_query::<MeshInfoQuery>()?;
        Ok(())
    }
}
