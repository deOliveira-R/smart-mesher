//! CAD geometry import: STEP (AP203/AP214/AP242) B-rep reader (and writer), content-addressed.
//!
//! W3a skeleton (orchestrator, stage 0). Keep [`CadioPlugin`] (unit struct, `Plugin` id `"cadio"`): `sm-server` installs it.

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `cadio` plugin. The skeleton registers nothing.
#[derive(Clone, Copy, Debug, Default)]
pub struct CadioPlugin;

impl Plugin for CadioPlugin {
    fn id(&self) -> &str {
        "cadio"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        let _ = registry;
        Ok(())
    }
}
