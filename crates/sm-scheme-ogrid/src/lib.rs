//! O-grid surface scheme (W4 task HEX-OGRID): structured ring decomposition of circular faces into all-quad meshes.
//! A self-contained kernel (ogrid and polar modes on a disk, an annulus with a 4-sided loop, a two-circle ring);
//! it does not depend on `sm-decomp`.
//! This skeleton (W4-0.2) registers nothing yet.

use sm_scheme_api::SchemeRegistry;

/// Register the O-grid scheme. In the W4-0.2 skeleton it registers nothing and always succeeds.
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
