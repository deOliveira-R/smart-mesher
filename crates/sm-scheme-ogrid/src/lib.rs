//! O-grid surface scheme (W4 tasks HEX-OGRID-K, HEX-OGRID-S): structured ring decomposition of
//! circular faces into all-quad meshes.
//!
//! This crate holds the pure kernels (task HEX-OGRID-K): [`ogrid_disk`] (five-block O-grid of a
//! disk), [`ogrid_annulus_quad`] (circle inside a square), [`ring`] (between two circles) and
//! [`polar_disk`] (central n-gon plus rings, for polyhedral/FV transport meshes). They take counts
//! and reals and return a [`PlanarMesh`]; they depend on no topology type and not on `sm-decomp`.
//! The `Scheme` adapter and its registration are HEX-OGRID-S; until then [`register`] adds nothing.

#![allow(clippy::needless_range_loop)] // index-based grid loops read clearer here

mod kernels;
mod mesh;

pub use kernels::{
    AnnulusQuadParams, DiskParams, PolarParams, RingParams, ogrid_annulus_quad, ogrid_disk, polar_disk, ring,
};
pub use mesh::{OgridError, PHASE_MAX_DEG, PlanarMesh, RATIO_MAX, Result as KernelResult, inscribed_area};

use sm_scheme_api::SchemeRegistry;

/// Register the O-grid scheme. Until HEX-OGRID-S it registers nothing and always succeeds.
pub fn register(_registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_scheme_api::SchemeDim;

    fn listing(r: &SchemeRegistry) -> Vec<String> {
        [SchemeDim::Curve, SchemeDim::Surface, SchemeDim::Volume]
            .into_iter()
            .flat_map(|d| r.of_dim(d).map(|s| s.name().to_string()).collect::<Vec<_>>())
            .collect()
    }

    #[test]
    fn register_twice_into_fresh_registries_changes_nothing() {
        for _ in 0..2 {
            let mut r = SchemeRegistry::new();
            let before = listing(&r);
            register(&mut r).unwrap();
            assert_eq!(listing(&r), before);
            assert!(before.is_empty());
        }
    }
}
