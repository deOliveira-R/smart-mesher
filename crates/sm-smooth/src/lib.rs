//! Mesh smoothing suite: Laplacian, smart Laplacian, centroid-area, Winslow, mean-ratio optimization; nodes stay on their geometry.
//!
//! W3a skeleton (orchestrator, stage 0). Keep [`SmoothPlugin`] (unit struct, `Plugin` id `"smooth"`): `sm-server` installs it.

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `smooth` plugin. The skeleton registers nothing.
#[derive(Clone, Copy, Debug, Default)]
pub struct SmoothPlugin;

impl Plugin for SmoothPlugin {
    fn id(&self) -> &str {
        "smooth"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        let _ = registry;
        Ok(())
    }
}
