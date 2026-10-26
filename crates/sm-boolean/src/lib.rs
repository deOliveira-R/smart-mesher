//! Booleans (unite, subtract, intersect), imprint, merge and the webcut family on the cell complex.
//!
//! W3a skeleton (orchestrator, stage 0). Keep [`BooleanPlugin`] (unit struct, `Plugin` id `"boolean"`): `sm-server` installs it.

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `boolean` plugin. The skeleton registers nothing.
#[derive(Clone, Copy, Debug, Default)]
pub struct BooleanPlugin;

impl Plugin for BooleanPlugin {
    fn id(&self) -> &str {
        "boolean"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        let _ = registry;
        Ok(())
    }
}
