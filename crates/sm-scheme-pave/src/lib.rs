//! Unstructured all-quad surface scheme (paving).

mod kernel;

pub use kernel::{PaveError, PaveOptions, QuadMesh2, pave_region};

use sm_scheme_api::SchemeRegistry;

/// Register this crate's schemes. The skeleton registers nothing.
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    let _ = registry;
    Ok(())
}
