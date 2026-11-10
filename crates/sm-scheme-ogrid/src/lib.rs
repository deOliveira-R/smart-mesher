//! O-grid surface scheme (W4 tasks HEX-OGRID-K, HEX-OGRID-S): structured ring decomposition of
//! circular faces into all-quad meshes.
//!
//! This crate holds the pure kernels (task HEX-OGRID-K): [`ogrid_disk`] (five-block O-grid of a
//! disk), [`ogrid_annulus_quad`] (circle inside a square), [`ring`] (between two circles) and
//! [`polar_disk`] (central n-gon plus rings, for polyhedral/FV transport meshes). They take counts
//! and reals and return a [`PlanarMesh`]; they depend on no topology type and not on `sm-decomp`.
//! The [`Ogrid`] scheme adapter (task HEX-OGRID-S) recognises exact disks, circle-in-square annuli and
//! two-circle rings and emits these kernels on the face.

#![allow(clippy::needless_range_loop)] // index-based grid loops read clearer here

mod fit;
mod kernels;
mod mesh;
mod scheme;

pub use kernels::{
    AnnulusQuadParams, DiskParams, PolarParams, RingParams, ogrid_annulus_quad, ogrid_disk, polar_disk, ring,
};
pub use mesh::{OgridError, PHASE_MAX_DEG, PlanarMesh, RATIO_MAX, Result as KernelResult, inscribed_area};

pub use scheme::{NAME, Ogrid, OgridParams};

use sm_scheme_api::SchemeRegistry;

/// Register the `ogrid` surface scheme ([`Ogrid`]). No automatic list names it yet; select it
/// with an explicit `mesh.scheme`.
pub fn register(registry: &mut SchemeRegistry) -> sm_scheme_api::Result<()> {
    registry.register(Box::new(Ogrid))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_scheme_api::SchemeDim;

    #[test]
    fn register_adds_ogrid_once_and_rejects_a_duplicate() {
        let mut r = SchemeRegistry::new();
        register(&mut r).unwrap();
        let names: Vec<String> = r.of_dim(SchemeDim::Surface).map(|s| s.name().to_string()).collect();
        assert_eq!(names, ["ogrid"]);
        assert!(register(&mut r).is_err());
    }
}
