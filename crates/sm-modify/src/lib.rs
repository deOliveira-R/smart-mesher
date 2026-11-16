//! The `modify` plugin: mesh modification ops (W4 tasks MOD-HTET, MOD-REFINE-U, MOD-REFINE-L): `mesh.htet` and
//! `mesh.qtri` (hex to tet with polyhedral closure, quad to tri), uniform `mesh.refine` with `LevelMap`, and local
//! refinement with polyhedral hanging faces.
//!
//! Registered so far: `mesh.refine` (MOD-REFINE-U), `mesh.htet` and `mesh.qtri` (MOD-HTET), `mesh.thex` and
//! `mesh.tquad` (MOD-CONV2), `mesh.refine_local` (MOD-REFINE-L). Every op is a pure read-source, build-new-mesh pass that keeps
//! classification, sets and tags, and links the result to its source with a `LevelMap` where it is a hierarchy.

// The passes index several parallel arrays by cell, face or node id; iterator forms would obscure them.
#![allow(clippy::needless_range_loop)]

mod carry;
mod convert;
mod deck;
mod htet;
mod local;
mod local_op;
mod ops;
mod project;
mod refine;
mod template;

pub use convert::{ClosureMode, Htet, Qtri, QtriSplit};
pub use htet::{Closure, Converted, HtetError, HtetOptions, HtetStats, QtriMode, hex_tets, htet, qtri};
pub use local::{LocalError, LocalOptions, LocalRefined, LocalStats, MAX_LOCAL_LEVELS, refine_local};
pub use local_op::RefineLocal;
pub use ops::{MAX_LEVELS, Refine, Thex, Tquad};
pub use refine::{RefineError, RefineStats, Refined, Rule, pass, refine};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `modify` plugin. It registers the ops `mesh.refine`, `mesh.htet`, `mesh.qtri`, `mesh.thex`, `mesh.tquad` and `mesh.refine_local`.
#[derive(Clone, Copy, Debug, Default)]
pub struct ModifyPlugin;

impl Plugin for ModifyPlugin {
    fn id(&self) -> &str {
        "modify"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register::<Refine>()?;
        registry.register::<Htet>()?;
        registry.register::<Qtri>()?;
        registry.register::<Thex>()?;
        registry.register::<Tquad>()?;
        registry.register::<RefineLocal>().map(|_| ())
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
            let want: Vec<String> = ["mesh.refine", "mesh.htet", "mesh.qtri", "mesh.thex", "mesh.tquad", "mesh.refine_local"].map(String::from).to_vec();
            let mut got: Vec<String> = added.into_iter().cloned().collect();
            got.sort();
            let mut want = want;
            want.sort();
            assert_eq!(got, want);
            assert_eq!(ModifyPlugin.id(), "modify");
        }
    }
}
