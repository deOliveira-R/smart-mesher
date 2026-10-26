//! Mesh validation and quality diagnostics as queries and ops: mesh.validate, mesh.quality, mesh.flag_cells.
//!
//! W3a skeleton (orchestrator, stage 0). Keep [`DiagPlugin`] (unit struct, `Plugin` id `"diag"`): `sm-server` installs it.

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `diag` plugin. The skeleton registers nothing.
#[derive(Clone, Copy, Debug, Default)]
pub struct DiagPlugin;

impl Plugin for DiagPlugin {
    fn id(&self) -> &str {
        "diag"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        let _ = registry;
        Ok(())
    }
}
