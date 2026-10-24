//! Curve meshing schemes: `equal`, `bias`, `dualbias`, `stretch`, `curvature`.
//!
//! Every scheme computes node positions as arc-length fractions of the edge, converts them to
//! **parameter** fractions with an adaptive Gauss-Legendre arc-length map and a monotone inverse,
//! and hands those to `GenCtx::curve_patch`, which owns node keys, positions and parameters.
//! See `DESIGN.md` for the design and the binding decisions on bias solving, dualbias with odd
//! counts and the curvature density.
//!
//! - [`spacing`]: the spacing math in normalized arc length (pure, unit-tested against closed forms).
//! - [`engine`]: a validated [`engine::Spec`] to parameter fractions on a concrete curve.
//! - [`params`]: the typed parameter structs carried in `SchemeChoice::params`.
//! - [`CurveScheme`]: the `Scheme` implementation; [`register`] adds all five to a registry.

mod arc;
pub mod engine;
pub mod error;
pub mod params;
mod schemes;
pub mod spacing;

pub use error::SpacingError;
pub use schemes::{CurveKind, CurveScheme};

use sm_scheme_api::SchemeRegistry;

/// Register the five curve schemes (`equal`, `bias`, `dualbias`, `stretch`, `curvature`).
/// Does not set auto-selection lists (the composer does).
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    for kind in [CurveKind::Equal, CurveKind::Bias, CurveKind::Dualbias, CurveKind::Stretch, CurveKind::Curvature] {
        registry.register(Box::new(CurveScheme::new(kind)))?;
    }
    Ok(())
}
