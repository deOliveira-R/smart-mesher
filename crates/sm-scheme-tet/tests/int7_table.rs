//! Round 4 before/after tables (ruling INT-7): `#[ignore]` release records using only the
//! public API that existed before (independent spacing `r`, minimum dihedral, S4 margin,
//! wall time), so the same file runs on the old and the new code.
//!
//! `cargo test -p sm-scheme-tet --release --test int7_table -- --ignored --nocapture <name>`
//! prints one `ROW|...` line per case. `table_sweep` is the 49-case release sweep of review
//! round 3 (it replaces `a15::relief_sweep_s4`): every case keeps the exact boundary and S4 is
//! met or disclosed.

mod common;
use common::*;
use sm_scheme_tet::{SurfaceInput, TetOptions, tetrahedralize};
use sm_testkit::surf3::{self, SplitMix64};

fn relief_cube(n: u32, seed: u64, depth: f64) -> SurfaceInput {
    let f = surf3::tri_cube(n);
    let mut inp = input_of(&f, 0);
    let mut rng = SplitMix64(seed);
    for p in &mut inp.points {
        if p[2] == 1.0 && p[0] > 0.0 && p[0] < 1.0 && p[1] > 0.0 && p[1] < 1.0 {
            p[2] = 1.0 - depth * rng.unit().abs();
        }
    }
    inp
}

fn row(set: &str, label: &str, inp: &SurfaceInput) {
    let t = std::time::Instant::now();
    let m = tetrahedralize(inp, &TetOptions::default()).unwrap_or_else(|e| panic!("{label}: {e}"));
    let secs = t.elapsed().as_secs_f64();
    check_boundary(&m, &inp.tris).unwrap_or_else(|e| panic!("{label}: {e}"));
    let r = spacing_r(&m, inp);
    let (c, mean, sd, p01, min) = r_summary(&r);
    let rms = (r.iter().map(|x| (x - 0.816_496_580_9).powi(2)).sum::<f64>() / c.max(1) as f64).sqrt();
    let margin = min_s4_margin(&m, inp);
    // Ruling S4 is met, or its violation is disclosed (never silent).
    assert!(margin >= 0.3 - 1e-9 || m.stats.n_s4_violations > 0, "{label}: S4 violation not disclosed (margin {margin})");
    println!(
        "ROW|{set}|{label}|{} tris|{secs:.1} s|{} steiner|{} tets|r n={c} mean={mean:.3} sd={sd:.3} p01={p01:.3} min={min:.3} rms_dev={rms:.3}|min dihedral {:.3}|mean ratio min {:.4} mean {:.4}|max R/l {:.2}|S4 margin {margin:.4} ({} viol)",
        inp.tris.len(),
        m.stats.n_steiner,
        m.tets.len(),
        m.stats.min_dihedral_deg,
        m.stats.min_mean_ratio,
        m.stats.mean_mean_ratio,
        m.stats.max_radius_edge,
        m.stats.n_s4_violations
    );
}

#[test]
#[ignore = "round 4 table: the eleven hard relief cases"]
fn table_a15() {
    let mut cases: Vec<(u32, u64, f64)> = (1..=3).map(|s| (6, s, 0.9)).collect();
    for depth in [0.6, 0.9] {
        cases.extend((1..=4).map(|s| (8, s, depth)));
    }
    for (n, seed, depth) in cases {
        row("a15", &format!("relief n{n} seed{seed} d{depth}"), &relief_cube(n, seed, depth));
    }
}

#[test]
#[ignore = "round 4 table: the 49-case sweep (minutes)"]
fn table_sweep() {
    let mut cases: Vec<(u32, u64, f64)> = vec![(12, 7, 0.995)];
    for n in [6u32, 8, 10, 12] {
        for seed in 1..=4u64 {
            for depth in [0.6, 0.9, 0.97] {
                cases.push((n, seed, depth));
            }
        }
    }
    for (n, seed, depth) in cases {
        row("sweep", &format!("relief n{n} seed{seed} d{depth}"), &relief_cube(n, seed, depth));
    }
}
