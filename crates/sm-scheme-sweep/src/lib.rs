//! Volume sweep scheme.
//!
//! Temporary header; see DESIGN.md.

mod discover;
mod emit;
mod error;
mod fit;
mod params;
mod place;
mod scheme;

/// The scheme name.
pub const NAME: &str = "sweep";

pub use discover::{Column, Columns, discover_columns};
pub use error::SweepError;
pub use fit::{fit_affine, fit_target_transform};
pub use params::{Placement, SweepParams, TargetMode};
pub use place::{Boundary, Interior, LayerMode, PAR_MIN, PlaceOptions, Placed, place_layers};
pub use scheme::Sweep;

use sm_scheme_api::SchemeRegistry;

/// Register the `"sweep"` volume scheme.
///
/// # Errors
/// A `BadParams` error if `"sweep"` is already registered (for example by
/// `sm_scheme_api::stubs::registry()`, which must not be combined with this).
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    registry.register(Box::new(Sweep))
}
