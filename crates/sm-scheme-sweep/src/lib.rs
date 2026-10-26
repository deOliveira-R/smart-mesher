//! The `"sweep"` volume scheme: a 1-to-1 sweep of a source face along columns to a target face.
//!
//! One source face, one target face and the linking faces between them become layers of
//! prismatic cells: each quad of the source gives a stack of hexahedra, each triangle a stack
//! of 6-node wedges, and each polygon of five or more sides a stack of prismatic polyhedra.
//! Translation, twist, arbitrary linking surfaces and rotation about an axis outside the
//! source are all the same code: the layers are placed from the boundary columns the linking
//! faces already carry.
//!
//! The scheme is declarative inside the `sm-scheme-api` driver:
//! 1. [`Sweep::applicability`](sm_scheme_api::Scheme::applicability) finds the columns
//!    ([`discover_columns`]) and tests whether the target is an affine image of the source
//!    ([`fit_target_transform`]).
//! 2. The requirements declare the source/target/linking roles, require `QUAD |
//!    BLOCK_STRUCTURED` on every linking face, register the target as a copy of the source
//!    and store the columns as plan data.
//! 3. The interval constraints make all columns equal step by step and add the optional
//!    `layers` row.
//! 4. Generation ([`place_layers`] and the cell emission) is a pure function of the plan and
//!    of the patches of earlier waves.
//!
//! Parameters are [`SweepParams`]; register the scheme with [`register`]. See `DESIGN.md`.

#![forbid(unsafe_code)]

mod discover;
mod emit;
mod error;
mod fit;
mod params;
mod place;
mod scheme;

/// The registered scheme name.
pub const NAME: &str = "sweep";

pub use discover::{Column, Columns, discover_columns};
pub use error::SweepError;
pub use fit::{fit_affine, fit_target_transform};
pub use params::{Placement, SweepParams, TargetMode};
pub use place::{Boundary, Interior, LayerMode, PAR_MIN, PlaceOptions, Placed, place_layers};
pub use scheme::Sweep;

use sm_scheme_api::SchemeRegistry;

/// Register the `"sweep"` volume scheme (capabilities `HEX | WEDGE | POLY`).
///
/// # Errors
/// A `BadParams` error if `"sweep"` is already registered, for example by
/// `sm_scheme_api::stubs::registry()`, which must not be combined with this function.
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    registry.register(Box::new(Sweep))
}
