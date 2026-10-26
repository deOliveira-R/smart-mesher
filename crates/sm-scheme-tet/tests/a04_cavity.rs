//! Acceptance 4: a cube with a cavity.

mod common;
use common::*;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::surf3;

#[test]
fn cube_with_cavity_volume_and_both_shells() {
    let f = surf3::cube_with_cavity(4);
    assert_eq!(f.volumes[0].n_shells, 2);
    let inp = input_of(&f, 0);
    let m = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    let v = total_volume(&m);
    assert!((v - 7.0).abs() < 1e-12 * 7.0, "volume {v}");
    // No tet centroid inside the cavity (the inner cube has side 1, centred at the origin).
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    // The inner shell is the set of vertices with all |coordinates| <= 0.5.
    for p in &f.points {
        if p.iter().all(|c| c.abs() <= 0.5 + 1e-12) {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
    }
    assert!((hi[0] - lo[0] - 1.0).abs() < 1e-12);
    for t in &m.tets {
        let q = t.map(|i| m.points[i as usize]);
        let c: Vec<f64> = (0..3).map(|k| (q[0][k] + q[1][k] + q[2][k] + q[3][k]) / 4.0).collect();
        let inside = (0..3).all(|k| c[k] > lo[k] + 1e-12 && c[k] < hi[k] - 1e-12);
        assert!(!inside, "tet centroid {c:?} inside the cavity");
    }
    // Both shells' triangles are faces of exactly one tet.
    check_boundary(&m, &inp.tris).unwrap();
    assert!(all_exact_positive(&m));
}
