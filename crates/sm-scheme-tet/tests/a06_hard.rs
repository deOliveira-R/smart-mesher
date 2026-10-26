//! Acceptance 6: Schönhardt and a non-Delaunay surface.

mod common;
use common::*;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::surf3;

#[test]
fn schonhardt_needs_an_interior_steiner_point() {
    let f = surf3::schonhardt();
    let inp = input_of(&f, 0);
    let m = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    assert!(m.stats.n_steiner >= 1, "steiner {}", m.stats.n_steiner);
    assert_eq!(m.points.len(), 6 + m.stats.n_steiner);
    check_boundary(&m, &inp.tris).unwrap();
    let want = f.volumes[0].enclosed_volume;
    let v = total_volume(&m);
    assert!(((v - want) / want).abs() < 1e-12, "{v} vs {want}");
    assert!(all_exact_positive(&m));
}

#[test]
fn noisy_sphere_keeps_all_triangles() {
    let f = surf3::noisy_sphere(1);
    let inp = input_of(&f, 0);
    let m = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    check_boundary(&m, &inp.tris).unwrap();
    let want = f.volumes[0].enclosed_volume;
    let v = total_volume(&m);
    assert!(((v - want) / want).abs() < 1e-12, "{v} vs {want}");
    assert!(all_exact_positive(&m));
    eprintln!("noisy_sphere: n_steiner {} unrefinable {}", m.stats.n_steiner, m.stats.unrefinable);
    assert!(m.stats.n_steiner > 0);
}
