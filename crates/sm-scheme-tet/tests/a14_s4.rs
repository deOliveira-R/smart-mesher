//! Ruling S4 on the recovery and smoothing paths: interior Steiner points keep a distance from
//! the input triangles (measured against the mean edge length `h` of each triangle) whenever the
//! geometry allows it, and recovery prefers the deepest point when it does not.

mod common;
use common::*;
use sm_scheme_tet::{SurfaceInput, TetOptions, TetMesh, tetrahedralize};
use sm_testkit::surf3;

fn run(label: &str, inp: &SurfaceInput) -> (TetMesh, f64) {
    let m = tetrahedralize(inp, &TetOptions::default()).unwrap_or_else(|e| panic!("{label}: {e}"));
    check_boundary(&m, &inp.tris).unwrap();
    assert!(all_exact_positive(&m));
    let margin = min_s4_margin(&m, inp);
    eprintln!("{label}: steiner {} min margin {margin:.3} h", m.stats.n_steiner);
    (m, margin)
}

/// The Schönhardt fixture needs one interior point, and no interior point is 0.3 h away from all
/// eight triangles (a grid search inside the solid finds at best 0.18 h at (0, 0, 0.68)): the
/// recovery point must be that single point, at a depth close to the best possible one (before
/// the review round it sat at 0.064 h).
#[test]
fn schonhardt_steiner_point_is_deep() {
    let inp = input_of(&surf3::schonhardt(), 0);
    let (m, margin) = run("schonhardt", &inp);
    assert_eq!(m.stats.n_steiner, 1);
    assert!(margin >= 0.14, "margin {margin}");
}

/// Where 0.3 h is feasible the bound holds for every Steiner point, recovery ones included.
#[test]
fn feasible_fixtures_keep_the_bound() {
    for (name, f) in [("cube_with_cavity(4)", surf3::cube_with_cavity(4)), ("two_cubes_shared(3)", surf3::two_cubes_shared(3)), ("tri_cube(4)", surf3::tri_cube(4))] {
        for v in 0..f.volumes.len() {
            let (m, margin) = run(&format!("{name} v{v}"), &input_of(&f, v));
            assert!(m.stats.n_steiner > 0);
            assert!(margin >= 0.3 - 1e-9, "{name} v{v}: margin {margin}");
        }
    }
}

/// A 12-triangle box `sx x sy x sz`; the diagonal of quad `k` is flipped when bit `k` of `flips`
/// is set.
fn slab(sx: f64, sy: f64, sz: f64, flips: u32) -> SurfaceInput {
    let mut points = Vec::new();
    for k in 0..8u32 {
        points.push([sx * f64::from(k & 1), sy * f64::from((k >> 1) & 1), sz * f64::from((k >> 2) & 1)]);
    }
    // Outward-wound quads (x-, x+, y-, y+, z-, z+).
    let quads: [[u32; 4]; 6] = [[0, 4, 6, 2], [1, 3, 7, 5], [0, 1, 5, 4], [2, 6, 7, 3], [0, 2, 3, 1], [4, 5, 7, 6]];
    let mut tris = Vec::new();
    for (k, q) in quads.iter().enumerate() {
        if (flips >> k) & 1 == 0 {
            tris.push([q[0], q[1], q[2]]);
            tris.push([q[0], q[2], q[3]]);
        } else {
            tris.push([q[1], q[2], q[3]]);
            tris.push([q[1], q[3], q[0]]);
        }
    }
    SurfaceInput { points, tris }
}

/// A convex slab (10 x 10 x 0.02) is thinner than 0.3 h everywhere, so S4 cannot hold; every
/// diagonal pattern must still mesh exactly with only a handful of Steiner points, all inside
/// the slab (the best possible margin is 0.002 h; recovery reaches about a third of it).
#[test]
fn thin_slab_all_diagonal_patterns() {
    let mut worst = f64::INFINITY;
    for flips in 0..64u32 {
        let inp = slab(10.0, 10.0, 0.02, flips);
        let m = tetrahedralize(&inp, &TetOptions::default()).unwrap_or_else(|e| panic!("pattern {flips}: {e}"));
        check_boundary(&m, &inp.tris).unwrap();
        assert!(all_exact_positive(&m));
        let vol = total_volume(&m);
        assert!((vol - 2.0).abs() < 1e-12, "pattern {flips}: volume {vol}");
        assert!(m.stats.n_steiner <= 6, "pattern {flips}: {} Steiner points", m.stats.n_steiner);
        for p in &m.points[m.n_input..] {
            assert!(p[2] > 0.0 && p[2] < 0.02, "pattern {flips}: Steiner point {p:?} outside the slab");
        }
        worst = worst.min(min_s4_margin(&m, &inp));
    }
    eprintln!("thin slab: worst margin {worst:.4} h (best possible about 0.002 h)");
}
