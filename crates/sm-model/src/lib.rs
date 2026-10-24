//! Primitives, transforms and topology queries. Skeleton: see W2 plan §2 task T1
//! (`docs/design/w2/plan.md`); that task fills in the bodies.

use sm_op_api::{Plugin, Registry, RegistryError};

/// The model plugin (ops and queries of the `model` and `topo` namespaces). Registers nothing yet.
#[derive(Clone, Copy, Debug, Default)]
pub struct ModelPlugin;

impl Plugin for ModelPlugin {
    fn id(&self) -> &str {
        "model"
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
        reg.install(&ModelPlugin).expect("install");
        assert_eq!(ModelPlugin.id(), "model");
    }
}
