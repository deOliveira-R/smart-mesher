//! Performance records (acceptance 12): `#[ignore]` release tests, no gate.
//!
//! `cargo test -p sm-scheme-tet --release --test perf -- --ignored --nocapture`

mod common;
use common::*;
use sm_scheme_tet::{SizeSpec, TetOptions, delaunay, tetrahedralize};
use sm_testkit::surf3;
use std::time::Instant;

#[test]
#[ignore = "performance record"]
fn record_delaunay_200k() {
    let mut r = surf3::SplitMix64(1);
    let pts: Vec<[f64; 3]> = (0..200_000).map(|_| [r.unit(), r.unit(), r.unit()]).collect();
    let t = Instant::now();
    let m = delaunay(&pts).unwrap();
    let s = t.elapsed().as_secs_f64();
    eprintln!("delaunay: {} points -> {} tets in {s:.3} s = {:.0} points/s", pts.len(), m.tets.len(), pts.len() as f64 / s);
}

fn ball(level: u32, h: f64, optimize: u8) {
    let f = surf3::icosphere(1.0, level);
    let inp = input_of(&f, 0);
    let t = Instant::now();
    let m = tetrahedralize(&inp, &TetOptions { size: SizeSpec::Constant(h), optimize, max_steiner: 500_000, ..TetOptions::default() }).unwrap();
    let s = t.elapsed().as_secs_f64();
    let st = &m.stats;
    eprintln!(
        "ball level {level}, h={h}, optimize={optimize}: {} tets ({} steiner, {} unrefinable) in {s:.2} s = {:.0} tets/s; stage ms {:?}; min dihedral {:.1}, min mean ratio {:.3}",
        m.tets.len(),
        st.n_steiner,
        st.unrefinable,
        m.tets.len() as f64 / s,
        st.stage_ms.map(|x| x.round()),
        st.min_dihedral_deg,
        st.min_mean_ratio
    );
    assert!((total_volume(&m) - f.volumes[0].enclosed_volume).abs() < 1e-9);
}

#[test]
#[ignore = "performance record"]
fn record_ball_pipeline() {
    ball(5, 0.033, 2);
}

#[test]
#[ignore = "performance record"]
fn record_ball_small() {
    ball(5, 0.07, 2);
}
