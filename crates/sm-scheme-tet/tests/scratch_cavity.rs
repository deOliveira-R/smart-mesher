mod common;
use common::*;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::surf3;

#[test]
fn cavity() {
    let f = surf3::cube_with_cavity(4);
    let inp = input_of(&f, 0);
    let m = tetrahedralize(&inp, &TetOptions { optimize: 0, ..TetOptions::default() }).unwrap();
    eprintln!("cavity tets {} unref {}", m.tets.len(), m.stats.unrefinable);
}
