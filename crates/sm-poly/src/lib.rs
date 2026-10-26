//! Tet to polyhedral conversion (median dual with boundary and feature handling; optional agglomeration).
//!
//! W3a skeleton (orchestrator, stage 0). Keep [`PolyPlugin`] (unit struct, `Plugin` id `"poly"`): `sm-server` installs it.

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `poly` plugin. The skeleton registers nothing.
#[derive(Clone, Copy, Debug, Default)]
pub struct PolyPlugin;

impl Plugin for PolyPlugin {
    fn id(&self) -> &str {
        "poly"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        let _ = registry;
        Ok(())
    }
}
