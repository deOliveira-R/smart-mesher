//! Triangle surface schemes: constrained Delaunay with refinement (trimesh, tridelaunay) and advancing front (triadvance).

pub mod cdt;
pub mod error;

pub use error::TriError;

use sm_scheme_api::SchemeRegistry;

/// Register this crate's schemes. The skeleton registers nothing.
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    let _ = registry;
    Ok(())
}
