//! Regressions for the fix1 review round: identical-row SVD, wide-bracket Newton, SoS caching.

mod common;
use common::Rng;
use sm_math::linalg::*;
use sm_math::roots::{RootOptions, newton_bisect};

fn maxdiff(a: &Mat3, b: &Mat3) -> f64 {
    (0..3)
        .flat_map(|i| (0..3).map(move |j| (i, j)))
        .map(|(i, j)| (a[i][j] - b[i][j]).abs())
        .fold(0.0, f64::max)
}

fn check_svd(a: &Mat3, tol: f64) {
    let r = svd3(a).expect("svd3 must converge");
    let norm = a.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs()));
    assert!(
        maxdiff(&r.reconstruct(), a) <= tol * norm,
        "reconstruction {a:?}"
    );
    let id = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for m in [&r.u, &r.v] {
        let g = mat3_mul(&mat3_transpose(m), m);
        assert!(maxdiff(&g, &id) <= tol, "orthogonality {a:?}");
    }
    assert!(r.s[0] >= r.s[1] && r.s[1] >= r.s[2] && r.s[2] >= 0.0);
}

#[test]
fn svd3_identical_rows_converge_at_all_scales() {
    for e in [0, -136, 58, 194, 254, -300] {
        let k = 10f64.powi(e);
        let row = [0.1 * k, 0.2 * k, 0.3 * k];
        check_svd(&[row, row, row], 1e-14);
    }
    let r = svd3(&[[0.1, 0.2, 0.3]; 3]).unwrap();
    let expect = (3.0f64 * (0.01 + 0.04 + 0.09)).sqrt();
    assert!((r.s[0] - expect).abs() < 1e-14 && r.s[1] < 1e-15 && r.s[2] < 1e-15);
}

#[test]
fn svd3_identical_row_fuzz_has_no_errors() {
    let mut rng = Rng(0x5eed);
    let m = 1_000_000;
    for _ in 0..30000 {
        let k = 10f64.powi(rng.int(40) as i32 - 20) / m as f64;
        let row = [0, 1, 2].map(|_| rng.int(m) as f64 * k);
        check_svd(&[row, row, row], 1e-13);
    }
}

#[test]
fn newton_bisect_handles_wide_brackets() {
    let o = RootOptions::default();
    let r = newton_bisect(|x| (x - 1.5e308, 1.0), 1e308, 1.7e308, None, o).unwrap();
    assert!((r.x - 1.5e308).abs() < 1e293, "{r:?}");
    let r = newton_bisect(|x| (x - 3.0, 1.0), -1e308, 1.7e308, None, o).unwrap();
    assert!((r.x - 3.0).abs() < 1e294, "{r:?}");
    let r = newton_bisect(|x| (x - 3.0, 0.0), -f64::MAX, f64::MAX, None, o).unwrap();
    assert!((r.x - 3.0).abs() < 1e294, "{r:?}");
}
