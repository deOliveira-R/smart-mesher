mod common;
use common::*;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::surf3;

#[test]
fn cube4_basic() {
    let f = surf3::tri_cube(4);
    let inp = input_of(&f, 0);
    let opts = TetOptions { optimize: 0, ..TetOptions::default() };
    let m = tetrahedralize(&inp, &opts).unwrap();
    assert_eq!(m.n_input, 98);
    assert_eq!(&m.points[..98], &f.points[..]);
    assert!(all_exact_positive(&m));
    let v = total_volume(&m);
    assert!((v - 1.0).abs() < 1e-12, "volume {v}");
    check_boundary(&m, &inp.tris).unwrap();
}
