//! The `modify` plugin: mesh modification ops (W4 tasks MOD-HTET, MOD-REFINE-U, MOD-REFINE-L): `mesh.htet` and
//! `mesh.qtri` (hex to tet with polyhedral closure, quad to tri), uniform `mesh.refine` with `LevelMap`, and local
//! refinement with polyhedral hanging faces.
//!
//! Registered so far: `mesh.refine` (MOD-REFINE-U). Every op is a pure read-source, build-new-mesh pass that keeps
//! classification, sets and tags, and links the result to its source with a `LevelMap` where it is a hierarchy.

// The passes index several parallel arrays by cell, face or node id; iterator forms would obscure them.
#![allow(clippy::needless_range_loop)]

mod ops;
mod project;
mod refine;
mod template;

pub use ops::{MAX_LEVELS, Refine};
pub use refine::{RefineError, RefineStats, Refined, refine};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `modify` plugin. It registers the op `mesh.refine`.
#[derive(Clone, Copy, Debug, Default)]
pub struct ModifyPlugin;

impl Plugin for ModifyPlugin {
    fn id(&self) -> &str {
        "modify"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register::<Refine>().map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn install_twice_into_fresh_registries_registers_the_refine_op() {
        for _ in 0..2 {
            let mut r = Registry::new();
            let before: Vec<String> = r.op_names().into_iter().map(String::from).collect();
            r.install(&ModifyPlugin).expect("install");
            let names: Vec<String> = r.op_names().into_iter().map(String::from).collect();
            let added: Vec<&String> = names.iter().filter(|n| !before.contains(n)).collect();
            assert_eq!(added, [&"mesh.refine".to_string()]);
            assert_eq!(ModifyPlugin.id(), "modify");
        }
    }
}
