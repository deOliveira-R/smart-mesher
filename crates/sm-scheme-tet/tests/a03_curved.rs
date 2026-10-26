//! Acceptance 3: icosphere and torus.

mod common;
use common::*;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::surf3;

#[test]
fn icosphere_level3_volume_boundary() {
    let f = surf3::icosphere(1.0, 3);
    assert_eq!((f.points.len(), f.tris.len()), (642, 1280));
    let inp = input_of(&f, 0);
    let m = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    let want = f.volumes[0].enclosed_volume;
    let v = total_volume(&m);
    assert!(((v - want) / want).abs() < 1e-12, "{v} vs {want}");
    let ball = 4.0 / 3.0 * std::f64::consts::PI;
    assert!(v / ball >= 0.95 && v / ball <= 1.0, "ratio {}", v / ball);
    assert!(all_exact_positive(&m));
    check_boundary(&m, &inp.tris).unwrap();
}

#[test]
fn torus_nonconvex_genus1() {
    let f = surf3::tri_torus(2.0, 0.7, 48, 24);
    assert_eq!((f.points.len(), f.tris.len()), (1152, 2304));
    let inp = input_of(&f, 0);
    let m = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    let want = f.volumes[0].enclosed_volume;
    let v = total_volume(&m);
    assert!(((v - want) / want).abs() < 1e-12, "{v} vs {want}");
    let smooth = 2.0 * std::f64::consts::PI.powi(2) * 2.0 * 0.7 * 0.7;
    assert!(((v - smooth) / smooth).abs() < 0.03, "{v} vs smooth {smooth}");
    assert!(all_exact_positive(&m));
    check_boundary(&m, &inp.tris).unwrap();
    for (a, b) in m.points[..1152].iter().zip(&f.points) {
        assert_eq!(a.map(f64::to_bits), b.map(f64::to_bits));
    }
}
