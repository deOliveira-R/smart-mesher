//! Unstructured all-quad surface scheme (paving).
//!
//! W3a skeleton (orchestrator, stage 0). Keep the [`register`] signature: `sm-server` calls it.

use sm_scheme_api::SchemeRegistry;

/// Register this crate's schemes. The skeleton registers nothing.
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    let _ = registry;
    Ok(())
}
