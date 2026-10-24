//! B-rep tessellation for display. Skeleton: see W2 plan §2 task T2 (`docs/design/w2/plan.md`);
//! that task fills in the bodies.

use sm_op_api::{Plugin, Registry, RegistryError};

/// The tessellation plugin (the `tess.*` queries). Registers nothing yet.
#[derive(Clone, Copy, Debug, Default)]
pub struct TessPlugin;

impl Plugin for TessPlugin {
    fn id(&self) -> &str {
        "tess"
    }

    fn register(&self, _registry: &mut Registry) -> Result<(), RegistryError> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plugin_installs() {
        let mut reg = Registry::new();
        reg.install(&TessPlugin).expect("install");
        assert_eq!(TessPlugin.id(), "tess");
    }
}
