//! The `csgbrep` plugin (W4 tasks CSG-*). This skeleton (W4-0.2) registers no op yet.

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `csgbrep` plugin. In the W4-0.2 skeleton it registers no op, query or layer.
#[derive(Clone, Copy, Debug, Default)]
pub struct CsgbrepPlugin;

impl Plugin for CsgbrepPlugin {
    fn id(&self) -> &str {
        "csgbrep"
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
            r.install(&CsgbrepPlugin).unwrap();
            assert_eq!(r.op_names(), ops);
            assert_eq!(r.query_names(), queries);
            assert_eq!(CsgbrepPlugin.id(), "csgbrep");
        }
    }
}
