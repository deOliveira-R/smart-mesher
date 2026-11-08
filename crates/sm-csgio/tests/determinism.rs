//! Determinism placeholder: two runs give equal results (and equal under `RAYON_NUM_THREADS=1`,
//! which `tools/ci.sh` sets for the whole suite). W3c-CSG-CORE replaces the body with hashes of
//! the imported IR.

use sm_csgio::{ImportError, openmc};

#[test]
fn import_is_deterministic_across_runs() {
    let a = openmc::import("<geometry/>");
    let b = openmc::import("<geometry/>");
    assert_eq!(a, b);
    assert!(matches!(a, Err(ImportError::Unsupported(_))));
}
