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
        "ball level {level}, h={h}, optimize={optimize}: {} tets ({} steiner, {} unrefinable) in {s:.2} s = {:.0} tets/s; stage ms {:?}; min dihedral {:.1}, min mean ratio {:.3}, mean mean ratio {:.4}, max R/l {:.2}",
        m.tets.len(),
        st.n_steiner,
        st.unrefinable,
        m.tets.len() as f64 / s,
        st.stage_ms.map(|x| x.round()),
        st.min_dihedral_deg,
        st.min_mean_ratio,
        st.mean_mean_ratio,
        st.max_radius_edge
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

/// Complete recovery scaling (review round 3): a cube with an 8 x 8 hard relief patch against the
/// plain cube of the same size (3 072, 12 288 and 27 648 triangles).
#[test]
#[ignore = "performance record"]
fn record_hard_patch_scaling() {
    for n in [16u32, 32, 48] {
        for (label, inp) in [("plain", input_of(&surf3::tri_cube(n), 0)), ("patch", patch_cube(n, 8, 2, 0.9))] {
            let t = Instant::now();
            let m = tetrahedralize(&inp, &TetOptions::default()).unwrap_or_else(|e| panic!("{label} n{n}: {e}"));
            let s = t.elapsed().as_secs_f64();
            let st = &m.stats;
            eprintln!(
                "{label} n{n}: {} tris, {} tets in {s:.2} s; stage ms {:?}; recovery Steiner {} (temporary boundary {}); constraint updates {}; S4 margin {:.4}; min dihedral {:.2}",
                inp.tris.len(),
                m.tets.len(),
                st.stage_ms.map(|x| x.round()),
                st.n_steiner_recovery,
                st.n_boundary_steiner,
                st.n_constraint_updates,
                min_s4_margin(&m, &inp),
                st.min_dihedral_deg
            );
            check_boundary(&m, &inp.tris).unwrap_or_else(|e| panic!("{label} n{n}: {e}"));
        }
    }
}
