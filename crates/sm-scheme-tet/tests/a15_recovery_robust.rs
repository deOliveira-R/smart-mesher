//! Review round 1: valid closed surfaces that need interior Steiner points for recovery
//! (height-field relief on a cube face, stacked and alternating twisted prisms) must be accepted
//! with the boundary preserved exactly (decision 12).
//!
//! Round 2: the relief cases at depth 0.9 on 6 x 6 and 8 x 8 grids and 0.6 on 8 x 8 grids, which
//! used to end in `RecoveryFailed`, recover through complete recovery
//! (`relief_hard_cases_recover_exactly`).
//!
//! Round 3: those cases also meet ruling S4 (every interior point at least `0.3 h_T` from every
//! input triangle) with a dihedral floor, and the work of complete recovery is linear in the
//! surface size (`complete_recovery_work_is_linear`, a work counter, not wall clock).

mod common;
use common::*;
use sm_scheme_tet::{SurfaceInput, TetMesh, TetOptions, tetrahedralize};
use sm_testkit::surf3::{self, SplitMix64};

fn check(inp: &SurfaceInput, label: &str) -> TetMesh {
    let t = std::time::Instant::now();
    let m = tetrahedralize(inp, &TetOptions::default()).unwrap_or_else(|e| panic!("{label}: {e}"));
    eprintln!("{label}: {:.2}s steiner {} tets {}", t.elapsed().as_secs_f64(), m.stats.n_steiner, m.tets.len());
    check_boundary(&m, &inp.tris).unwrap_or_else(|e| panic!("{label}: {e}"));
    assert!(all_exact_positive(&m), "{label}: non-positive tet");
    assert_eq!(m.points[..inp.points.len()], inp.points[..], "{label}: input points moved");
    let vol = total_volume(&m);
    let want = enclosed_volume(inp);
    assert!((vol - want).abs() <= 1e-10 * want.abs().max(1.0), "{label}: volume {vol} vs {want}");
    m
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

/// Hash of a mesh: point bits, tets and the statistics (not `stage_ms`).
fn mesh_hash(m: &TetMesh) -> u64 {
    use core::hash::Hasher;
    let mut h = sm_testkit::determinism::Fnv::default();
    for p in &m.points {
        for c in p {
            h.write(&c.to_bits().to_le_bytes());
        }
    }
    for t in &m.tets {
        for i in t {
            h.write(&i.to_le_bytes());
        }
    }
    for x in [m.stats.n_steiner, m.stats.n_steiner_recovery, m.stats.n_boundary_steiner, m.stats.flips, m.stats.smooth_passes, m.stats.unrefinable, m.stats.n_s4_violations] {
        h.write(&(x as u64).to_le_bytes());
    }
    h.write(&m.stats.n_constraint_updates.to_le_bytes());
    h.write(&m.stats.spacing.mean.to_bits().to_le_bytes());
    h.write(&(m.stats.n_quality_violations as u64).to_le_bytes());
    h.write(&m.stats.worst_s4_margin.to_bits().to_le_bytes());
    h.finish()
}

/// Review round 2: relief cases the interior-point recovery cannot finish (decision 12). They
/// go through complete recovery (temporary boundary Steiner points on the stuck edges and
/// triangles, moved off the surface afterwards) and must come out with the exact boundary.
/// Round 3: every interior point meets ruling S4 (measured here independently of the mesher's
/// own statistic, which must agree), and the smallest dihedral angle stays above a floor.
/// Debug builds run three of the eleven cases.
#[test]
fn relief_hard_cases_recover_exactly() {
    let mut cases: Vec<(u32, u64, f64)> = (1..=3).map(|s| (6, s, 0.9)).collect();
    for depth in [0.6, 0.9] {
        cases.extend((1..=4).map(|s| (8, s, depth)));
    }
    if cfg!(debug_assertions) {
        cases = vec![(6, 1, 0.9), (6, 2, 0.9), (8, 2, 0.6)];
    }
    for (n, seed, depth) in cases {
        let label = format!("relief n{n} seed{seed} d{depth}");
        let inp = relief_cube(n, seed, depth);
        let m = check(&inp, &label);
        let margin = min_s4_margin(&m, &inp);
        let (d_steiner, d_input) = min_dihedral_split(&m);
        eprintln!(
            "{label}: recovery Steiner {} (temporary boundary {}), refinement {}, tets {}, S4 margin {margin:.4} ({} violations), min dihedral {:.3} (tets with a Steiner vertex {d_steiner:.3}, input vertices only {d_input:.3}, sharpest surface wedge {:.3}), stage ms {:?}",
            m.stats.n_steiner_recovery,
            m.stats.n_boundary_steiner,
            m.stats.n_steiner_refine,
            m.tets.len(),
            m.stats.n_s4_violations,
            m.stats.min_dihedral_deg,
            min_solid_wedge_deg(&inp),
            m.stats.stage_ms.map(|x| x.round())
        );
        // Ruling S4, and the statistic never hides a violation.
        assert_eq!(m.stats.n_s4_violations, 0, "{label}: S4 violations (worst {})", m.stats.worst_s4_margin);
        assert!(margin >= 0.3 - 1e-9, "{label}: S4 margin {margin}");
        // Dihedral floors (review round 3, measured 0.563 (n8 seed4 d0.6) .. 5.24 degrees
        // overall, the worst tets having input vertices only: they span thin walls next to the
        // steepest relief, where S4 leaves no room for an interior point; tets with a Steiner
        // vertex 1.99 (n8 seed2 d0.9) .. 11.9 degrees).
        assert!(m.stats.min_dihedral_deg >= 0.5, "{label}: min dihedral {}", m.stats.min_dihedral_deg);
        assert!(d_steiner >= 1.5, "{label}: min dihedral at Steiner tets {d_steiner}");
    }
}

/// Review round 3: the constraint lookup of complete recovery is updated incrementally (it was
/// rebuilt from scratch after every recovered triangle, quadratic in the surface size). The
/// number of lookup entries touched is about six per input triangle, independent of the size.
#[test]
fn complete_recovery_work_is_linear() {
    let mut per_tri: Vec<f64> = Vec::new();
    for n in [16u32, 32] {
        let inp = patch_cube(n, 8, 2, 0.9);
        let m = check(&inp, &format!("patch n{n}"));
        let st = &m.stats;
        let tris = inp.tris.len() as u64;
        eprintln!("patch n{n}: {tris} triangles, {} temporary boundary points, {} constraint updates", st.n_boundary_steiner, st.n_constraint_updates);
        assert!(st.n_boundary_steiner > 0, "patch n{n}: complete recovery did not run");
        assert!(st.n_constraint_updates <= 7 * tris + 64 * st.n_boundary_steiner as u64, "patch n{n}: {} updates", st.n_constraint_updates);
        per_tri.push(st.n_constraint_updates as f64 / tris as f64);
        assert_eq!(st.n_s4_violations, 0, "patch n{n}");
        // Round 4: the lookup is only ever updated, never rebuilt.
        assert_eq!(st.n_lookup_rebuilds, 0, "patch n{n}");
    }
    // Four times the triangles: at most about four times the work (a rebuild per triangle
    // would be sixteen).
    assert!(per_tri[1] <= 1.25 * per_tri[0], "updates per triangle {per_tri:?}");
    let plain = tetrahedralize(&input_of(&surf3::tri_cube(16), 0), &TetOptions::default()).unwrap();
    assert_eq!(plain.stats.n_constraint_updates, 0, "complete recovery ran on the plain cube");
    assert_eq!(plain.stats.n_lookup_rebuilds, 0);
}

/// Complete recovery is bit-identical over runs and thread counts.
#[test]
fn relief_hard_case_is_deterministic() {
    let inp = relief_cube(6, 2, 0.9);
    let h = sm_testkit::determinism::check(2, 4, |threads| {
        rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(|| mesh_hash(&tetrahedralize(&inp, &TetOptions::default()).unwrap()))
    })
    .unwrap_or_else(|e| panic!("relief n6 seed2 d0.9: {e}"));
    assert_ne!(h, 0);
}
