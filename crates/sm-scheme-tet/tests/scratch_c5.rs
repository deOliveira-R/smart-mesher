mod common;
use common::*;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::surf3;
#[test]
fn c5() {
    let f = surf3::cube_with_cavity(5);
    let inp = input_of(&f, 0);
    eprintln!("edge pts {:?} {:?}", inp.points[249], inp.points[285]);
    let r = tetrahedralize(&inp, &TetOptions::default());
    eprintln!("{:?}", r.map(|m| m.tets.len()));
}
