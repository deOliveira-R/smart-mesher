//! Triangle surface schemes: constrained Delaunay with refinement (`trimesh`, `tridelaunay`)
//! and, as a stretch, an advancing front (`triadvance`).
//!
//! * [`cdt`]: the pure 2-D kernel (exact predicates, Sloan segment recovery, Ruppert/Chew
//!   refinement, smoothing and flips), usable and tested without any topology.
//! * [`chart`]: lifts a face's boundary loops into a plane (seams, poles, Newell planes).
//! * [`scheme`]: the `trimesh`/`tridelaunay` schemes and [`TriParams`].
//!
//! See `DESIGN.md` for the positioning, the pole-fan rationale and the guarantee statement.

pub mod cdt;
pub mod chart;
pub mod error;
pub mod scheme;
mod util;

pub use error::TriError;
pub use scheme::{TriParams, TriScheme};

use sm_scheme_api::SchemeRegistry;

/// Register `trimesh` and `tridelaunay` (surface schemes, `Caps::TRI`).
///
/// Never registers `tri` (the stub's name), a fallback chain or an auto list: those stay
/// unchanged in W3a (decision 12).
///
/// # Errors
/// A name is already registered.
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    registry.register(Box::new(TriScheme::trimesh()))?;
    registry.register(Box::new(TriScheme::tridelaunay()))?;
    Ok(())
}
