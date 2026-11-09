//! The `modify` plugin: mesh modification ops (W4 tasks MOD-HTET, MOD-REFINE-U, MOD-REFINE-L): `mesh.htet` and
//! `mesh.qtri` (hex to tet with polyhedral closure, quad to tri), uniform `mesh.refine` with `LevelMap`, and local
//! refinement with polyhedral hanging faces. This skeleton (W4-0.2) registers no op yet.

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `modify` plugin. In the W4-0.2 skeleton it registers no op, query or layer.
#[derive(Clone, Copy, Debug, Default)]
pub struct ModifyPlugin;

impl Plugin for ModifyPlugin {
    fn id(&self) -> &str {
        "modify"
    }

    fn register(&self, _registry: &mut Registry) -> Result<(), RegistryError> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn install_twice_into_fresh_registries_changes_nothing() {
        for _ in 0..2 {
            let mut r = Registry::new();
            let ops: Vec<String> = r.op_names().into_iter().map(String::from).collect();
            let queries: Vec<String> = r.query_names().into_iter().map(String::from).collect();
            r.install(&ModifyPlugin).unwrap();
            assert_eq!(r.op_names(), ops);
            assert_eq!(r.query_names(), queries);
            assert_eq!(ModifyPlugin.id(), "modify");
        }
    }
}
