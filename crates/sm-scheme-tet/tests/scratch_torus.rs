mod common;
use common::*;
use sm_scheme_tet::{TetOptions, tet_quality, tetrahedralize};
use sm_testkit::surf3;

#[test]
fn torus_bad() {
    let f = surf3::tri_torus(2.0, 0.7, 48, 24);
    let inp = input_of(&f, 0);
    let m = tetrahedralize(&inp, &TetOptions { optimize: 0, ..TetOptions::default() }).unwrap();
    let mut n = 0;
    for t in &m.tets {
        let p = t.map(|i| m.points[i as usize]);
        let q = tet_quality(p);
        if q.radius_edge > 2.0 && n < 5 {
            n += 1;
            eprintln!("tet {t:?} re={} mr={} dih=[{},{}] vol={:e}", q.radius_edge, q.mean_ratio, q.min_dihedral, q.max_dihedral, tet_vol(&m, *t));
        }
    }
    let bad = m.tets.iter().filter(|t| tet_quality(t.map(|i| m.points[i as usize])).radius_edge > 2.0).count();
    eprintln!("bad radius-edge tets {bad} of {}", m.tets.len());
}
