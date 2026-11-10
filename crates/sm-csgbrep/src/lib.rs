//! The `csgbrep` plugin (W4 tasks CSG-*): OpenMC CSG decks to B-rep volumes.
//!
//! * [`ImportOpenmc`] (`model.import_openmc`): one history node that reads a deck and builds the
//!   volumes by space partition ([`partition`]), with blocks per region name, sidesets per
//!   boundary kind and lattice frames and region tags per deck lattice.
//! * [`deck`]: the deck placed in the global frame (canonical surfaces, cell regions, origins).
//! * [`quadric`]: the classifier that turns an OpenMC quadric into an exact plane or cylinder.

pub mod deck;
mod error;
mod op;
pub mod partition;
pub mod quadric;
pub mod reuse;

pub use error::CsgbrepError;
pub use op::{ImportOpenmc, OpenmcOptions, OverlapMode};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The `csgbrep` plugin: registers `model.import_openmc`. It needs the `model` and `fem` layers
/// (`sm_layer_api::CoreLayers` and `sm_fem::FemPlugin`) and the `StepInputs` service for
/// `{sha256}` sources.
#[derive(Clone, Copy, Debug, Default)]
pub struct CsgbrepPlugin;

impl Plugin for CsgbrepPlugin {
    fn id(&self) -> &str {
        "csgbrep"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register::<ImportOpenmc>().map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn install_registers_the_import_op_and_only_that() {
        let mut names = Vec::new();
        for _ in 0..2 {
            let mut r = Registry::new();
            let before: Vec<String> = r.op_names().into_iter().map(String::from).collect();
            let queries: Vec<String> = r.query_names().into_iter().map(String::from).collect();
            r.install(&CsgbrepPlugin).unwrap();
            let after: Vec<String> = r.op_names().into_iter().map(String::from).collect();
            assert_eq!(after.len(), before.len() + 1);
            assert!(after.iter().any(|n| n == "model.import_openmc"));
            assert_eq!(r.query_names(), queries);
            assert_eq!(CsgbrepPlugin.id(), "csgbrep");
            names.push(after);
        }
        assert_eq!(names[0], names[1]);
    }
}
