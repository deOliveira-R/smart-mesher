//! Review round 1: valid closed surfaces that need interior Steiner points for recovery
//! (height-field relief on a cube face, stacked and alternating twisted prisms) must be accepted
//! with the boundary preserved exactly (decision 12).
//!
//! Known limit (see W3_REPORT): relief at depth 0.9 on 6 x 6 and finer grids, and 0.6 on 8 x 8
//! grids, can still fail with a typed `RecoveryFailed`. `relief_known_hard_fail_typed` records
//! that they fail cleanly and in bounded time.

mod common;
use common::*;
use sm_scheme_tet::{SurfaceInput, TetError, TetOptions, tetrahedralize};
use sm_testkit::surf3::{self, SplitMix64};

fn check(inp: &SurfaceInput, label: &str) {
    let t = std::time::Instant::now();
    let m = tetrahedralize(inp, &TetOptions::default()).unwrap_or_else(|e| panic!("{label}: {e}"));
    eprintln!("{label}: {:.2}s steiner {} tets {}", t.elapsed().as_secs_f64(), m.stats.n_steiner, m.tets.len());
    check_boundary(&m, &inp.tris).unwrap_or_else(|e| panic!("{label}: {e}"));
    assert!(all_exact_positive(&m), "{label}: non-positive tet");
    assert_eq!(m.points[..inp.points.len()], inp.points[..], "{label}: input points moved");
    let vol = total_volume(&m);
    let want = enclosed_volume(inp);
    assert!((vol - want).abs() <= 1e-10 * want.abs().max(1.0), "{label}: volume {vol} vs {want}");
}

/// Enclosed volume of the closed surface (divergence theorem about the first point).
fn enclosed_volume(inp: &SurfaceInput) -> f64 {
    let o = inp.points[0];
    let mut v = 0.0;
    for t in &inp.tris {
        let q = t.map(|i| {
            let p = inp.points[i as usize];
            [p[0] - o[0], p[1] - o[1], p[2] - o[2]]
        });
        let c = [q[1][1] * q[2][2] - q[1][2] * q[2][1], q[1][2] * q[2][0] - q[1][0] * q[2][2], q[1][0] * q[2][1] - q[1][1] * q[2][0]];
        v += (q[0][0] * c[0] + q[0][1] * c[1] + q[0][2] * c[2]) / 6.0;
    }
    v
}

/// A cube with `n x n` quads per face whose top face (z = 1, interior vertices) is pushed down
/// by up to `depth` with seeded random relief.
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

/// Stacked twisted prisms: ring `k` at height `z_k`, rotated by `rot_k` degrees.
fn tower(rings: &[(f64, f64)]) -> SurfaceInput {
    let mut points = Vec::new();
    for &(z, rot) in rings {
        for i in 0..3 {
            let a = std::f64::consts::TAU * f64::from(i) / 3.0 + rot.to_radians();
            points.push([a.cos(), a.sin(), z]);
        }
    }
    let top = 3 * (rings.len() as u32 - 1);
    let mut tris: Vec<[u32; 3]> = vec![[0, 2, 1], [top, top + 1, top + 2]];
    for l in 0..rings.len() as u32 - 1 {
        let (lo, hi) = (3 * l, 3 * l + 3);
        for i in 0..3u32 {
            let j = (i + 1) % 3;
            tris.push([lo + i, lo + j, hi + j]);
            tris.push([lo + i, hi + j, hi + i]);
        }
    }
    SurfaceInput { points, tris }
}

/// Review case (a): the height-field box that used to fail after about 10 s.
#[test]
fn relief_cube_seed5_depth09_n4() {
    check(&relief_cube(4, 5, 0.9), "relief n4 seed5 d0.9");
}

/// The relief family on the 4 x 4 grid, up to depth 0.9 (seeds 1..=6).
#[test]
fn relief_family_n4() {
    for seed in 1..=6u64 {
        for depth in [0.3, 0.6, 0.9] {
            check(&relief_cube(4, seed, depth), &format!("relief n4 seed{seed} d{depth}"));
        }
    }
}

/// Finer grids at moderate depth (a lighter sweep in debug builds).
#[test]
fn relief_family_n6_n8_moderate() {
    // (n6 seed2 d0.6 recovers but takes about 20 s in release: left out.)
    let seeds: &[u64] = if cfg!(debug_assertions) { &[1] } else { &[1, 3, 4] };
    for &seed in seeds {
        for depth in [0.3, 0.6] {
            check(&relief_cube(6, seed, depth), &format!("relief n6 seed{seed} d{depth}"));
        }
        check(&relief_cube(8, seed, 0.3), &format!("relief n8 seed{seed} d0.3"));
    }
}

/// Review case (b): two stacked twisted prisms (a two-layer Schönhardt tower).
#[test]
fn two_layer_schonhardt_tower() {
    check(&tower(&[(0.0, 0.0), (1.0, 30.0), (2.0, 0.0)]), "tower2");
}

/// Review case (c): five rings, alternating twist (needs two interior points).
#[test]
fn five_ring_tower() {
    check(&tower(&[(0.0, 0.0), (0.2, 40.0), (0.4, 0.0), (0.6, 40.0), (0.8, 0.0)]), "tower5");
}

#[test]
fn alternating_towers() {
    for (k, tw) in [(3usize, 25.0), (4, 35.0), (3, 45.0)] {
        let rings: Vec<(f64, f64)> = (0..k).map(|i| (0.5 * i as f64, if i % 2 == 1 { tw } else { 0.0 })).collect();
        check(&tower(&rings), &format!("alt k{k} tw{tw}"));
    }
}

/// Cases the recovery still cannot do (release only, about 15 s each): they must fail with
/// a typed `RecoveryFailed`, never panic, never return a wrong mesh.
#[test]
#[ignore = "known limit; about 15 s each in release"]
fn relief_known_hard_fail_typed() {
    for (n, seed, depth) in [(6u32, 1u64, 0.9), (6, 2, 0.9), (8, 1, 0.6)] {
        let inp = relief_cube(n, seed, depth);
        match tetrahedralize(&inp, &TetOptions::default()) {
            Ok(m) => {
                check_boundary(&m, &inp.tris).unwrap();
                assert!(all_exact_positive(&m));
            }
            Err(TetError::RecoveryFailed { .. }) => {}
            Err(e) => panic!("n{n} seed{seed} d{depth}: unexpected {e}"),
        }
    }
}
