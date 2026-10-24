//! The plugin that registers the shared layers.

use std::sync::Arc;

use sm_op_api::{Plugin, Registry, RegistryError};
use sm_topo_api::CellComplex;

use crate::host::TopoHost;
use crate::mesh::MeshLayer;
use crate::model::ModelLayer;

type TopoFactory = dyn Fn() -> Box<dyn TopoHost> + Send + Sync;

/// Registers [`ModelLayer`] and [`MeshLayer`], so a layer is never registered twice
/// (`DuplicateLayer`) and every crate can build a test state without its siblings.
///
/// The topology store is chosen by the factory: `|| Box::new(sm_topo::TopoStore::new())` in the
/// standard distribution, the reference `CellComplex` by [`Default`].
#[derive(Clone)]
pub struct CoreLayers {
    topo: Arc<TopoFactory>,
}

impl CoreLayers {
    /// A plugin whose fresh states get a topology host from `topo`.
    pub fn new(topo: impl Fn() -> Box<dyn TopoHost> + Send + Sync + 'static) -> Self {
        Self { topo: Arc::new(topo) }
    }
}

impl Default for CoreLayers {
    /// The reference topology store ([`CellComplex`]).
    fn default() -> Self {
        Self::new(|| Box::new(CellComplex::new()))
    }
}

impl Plugin for CoreLayers {
    fn id(&self) -> &str {
        "core-layers"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        let topo = Arc::clone(&self.topo);
        registry.register_layer_with(move || ModelLayer::new(topo()))?;
        registry.register_layer::<MeshLayer>()?;
        Ok(())
    }
}
