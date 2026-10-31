//! Round 4 (ruling INT-7): the spacing statistic, the quality floor as located diagnostics, and
//! bounded work on tiled relief. Counters, not wall clock; timings are in `#[ignore]` records.

mod common;
use common::*;
use sm_scheme_tet::{MIN_DIHEDRAL_DEG, MIN_MEAN_RATIO, QualityMetric, SurfaceInput, TetMesh, TetOptions, tetrahedralize};
use sm_testkit::surf3::{self, SplitMix64};
use std::time::Instant;

fn mesh(inp: &SurfaceInput, label: &str) -> TetMesh {
    let m = tetrahedralize(inp, &TetOptions::default()).unwrap_or_else(|e| panic!("{label}: {e}"));
    check_boundary(&m, &inp.tris).unwrap_or_else(|e| panic!("{label}: {e}"));
    assert!(all_exact_positive(&m), "{label}: non-positive tet");
    m
}

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

/// The in-crate statistic agrees with an independent implementation (bucket grid, brute-force
/// nearest triangle) on meshes with interior points.
#[test]
fn spacing_stats_match_an_independent_measure() {
    let cases = [("cube 6", input_of(&surf3::tri_cube(6), 0)), ("ico", input_of(&surf3::icosphere(1.0, 2), 0)), ("relief n6", relief_cube(6, 2, 0.9))];
    for (label, inp) in &cases {
        let m = mesh(inp, label);
        let s = m.stats.spacing;
        let r = spacing_r(&m, inp);
        let (c, mean, sd, p01, min) = r_summary(&r);
        eprintln!("{label}: r n={c} mean={mean:.4} sd={sd:.4} p01={p01:.4} min={min:.4}; in-crate {s:?}");
        assert_eq!(s.count, c, "{label}");
        assert_eq!(s.count, m.stats.n_steiner, "{label}");
        assert!(c > 0, "{label}: no interior points to measure");
        for (name, a, b) in [("mean", s.mean, mean), ("sd", s.std_dev, sd), ("p01", s.p01, p01), ("min", s.min, min)] {
            assert!((a - b).abs() <= 1e-9, "{label}: {name} {a} vs {b}");
        }
        assert!(s.min <= s.p01 && s.p01 <= s.mean + 1e-12 && s.mean <= s.cap + 1e-12, "{label}: order {s:?}");
        assert!((s.target - sm_scheme_tet::TARGET_R).abs() < 1e-15);
        assert!(s.rms_dev >= 0.0 && s.rms_dev.is_finite());
    }
}

/// No interior points: all-zero statistic.
#[test]
fn spacing_stats_without_steiner_points_are_zero() {
    let inp = SurfaceInput { points: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], tris: vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]] };
    let m = mesh(&inp, "single tet");
    assert_eq!(m.stats.n_steiner, 0);
    assert_eq!(m.stats.spacing.count, 0);
    assert_eq!(m.stats.spacing.mean, 0.0);
    assert_eq!(m.stats.n_quality_violations, 0);
    assert!(m.stats.quality_diagnostics.is_empty());
}

/// A clean mesh has no quality violation; a hard relief mesh has located ones, worst first.
#[test]
fn quality_floor_violations_are_located_and_sorted() {
    let clean = mesh(&input_of(&surf3::icosphere(1.0, 2), 0), "ico");
    assert_eq!(clean.stats.n_quality_violations, 0, "{:?}", clean.stats.quality_diagnostics.first());
    let inp = relief_cube(6, 3, 0.9);
    let m = mesh(&inp, "relief n6 seed3 d0.9");
    let st = &m.stats;
    assert!(st.n_quality_violations > 0, "min dihedral {}", st.min_dihedral_deg);
    assert!(!st.quality_diagnostics.is_empty() && st.quality_diagnostics.len() <= sm_scheme_tet::MAX_DIAGNOSTICS);
    assert!(st.quality_diagnostics.len() <= st.n_quality_violations);
    let lo = [0.0; 3];
    let hi = [1.0; 3];
    for d in &st.quality_diagnostics {
        assert!(d.value < d.threshold, "{d:?}");
        let want = match d.metric {
            QualityMetric::MinDihedral => MIN_DIHEDRAL_DEG,
            QualityMetric::MeanRatio => MIN_MEAN_RATIO,
        };
        assert_eq!(d.threshold, want);
        assert!(!d.triangles.is_empty() && d.triangles.len() <= 6, "{d:?}");
        assert!(d.triangles.windows(2).all(|w| w[0] < w[1]), "ids sorted and distinct: {d:?}");
        assert!(d.triangles.iter().all(|&t| (t as usize) < inp.tris.len()), "{d:?}");
        assert!((0..3).all(|k| d.position[k] >= lo[k] && d.position[k] <= hi[k]), "{d:?}");
        // The listed triangles are near the position (a few local sizes).
        let near = d.triangles.iter().map(|&t| {
            let [a, b, c] = inp.tris[t as usize].map(|i| inp.points[i as usize]);
            point_tri_dist(d.position, a, b, c)
        });
        assert!(near.fold(f64::INFINITY, f64::min) < 1.0, "{d:?}");
    }
    // Worst first within each metric; the very worst dihedral is the mesh minimum.
    for metric in [QualityMetric::MinDihedral, QualityMetric::MeanRatio] {
        let v: Vec<f64> = st.quality_diagnostics.iter().filter(|d| d.metric == metric).map(|d| d.value).collect();
        assert!(v.windows(2).all(|w| w[0] <= w[1]), "{metric:?} not sorted: {v:?}");
    }
    let worst = st.quality_diagnostics.iter().filter(|d| d.metric == QualityMetric::MinDihedral).map(|d| d.value).fold(f64::INFINITY, f64::min);
    assert!((worst - st.min_dihedral_deg).abs() < 1e-9, "{worst} vs {}", st.min_dihedral_deg);
}

/// Triangle ids in the diagnostics are the caller's: listing the input triangles in another
/// order changes the ids, but each id still names the same triangle (owner), and everything
/// else is unchanged.
#[test]
fn diagnostic_triangle_ids_follow_the_callers_order() {
    let inp = relief_cube(6, 2, 0.9);
    let a = mesh(&inp, "ordered");
    // Reverse the triangle list.
    let rev = SurfaceInput { points: inp.points.clone(), tris: inp.tris.iter().rev().copied().collect() };
    let b = mesh(&rev, "reversed");
    let nt = inp.tris.len() as u32;
    assert_eq!(a.tets, b.tets);
    assert_eq!(a.stats.quality_diagnostics.len(), b.stats.quality_diagnostics.len());
    for (x, y) in a.stats.quality_diagnostics.iter().zip(&b.stats.quality_diagnostics) {
        assert_eq!(x.position.map(f64::to_bits), y.position.map(f64::to_bits));
        assert_eq!(x.value.to_bits(), y.value.to_bits());
        let tris_of = |ids: &[u32], from: &SurfaceInput| -> Vec<[u32; 3]> {
            let mut v: Vec<[u32; 3]> = ids.iter().map(|&i| sm_scheme_tet_canon(from.tris[i as usize])).collect();
            v.sort_unstable();
            v
        };
        assert_eq!(tris_of(&x.triangles, &inp), tris_of(&y.triangles, &rev));
        assert!(y.triangles.iter().all(|&t| t < nt));
    }
}

fn sm_scheme_tet_canon(t: [u32; 3]) -> [u32; 3] {
    let r = (0..3).min_by_key(|&i| t[i]).unwrap();
    [t[r], t[(r + 1) % 3], t[(r + 2) % 3]]
}

/// Bit-identical statistics (spacing and diagnostics included) over runs and thread counts.
#[test]
fn int7_statistics_are_deterministic() {
    let inp = relief_cube(6, 2, 0.9);
    let h = sm_testkit::determinism::check(2, 4, |threads| {
        rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(|| {
            use core::hash::Hasher;
            let m = tetrahedralize(&inp, &TetOptions::default()).unwrap();
            let mut h = sm_testkit::determinism::Fnv::default();
            let s = &m.stats;
            for x in [s.spacing.mean, s.spacing.std_dev, s.spacing.p01, s.spacing.min, s.spacing.rms_dev] {
                h.write(&x.to_bits().to_le_bytes());
            }
            h.write(&(s.n_quality_violations as u64).to_le_bytes());
            for d in &s.quality_diagnostics {
                for c in d.position {
                    h.write(&c.to_bits().to_le_bytes());
                }
                h.write(&d.value.to_bits().to_le_bytes());
                for t in &d.triangles {
                    h.write(&t.to_le_bytes());
                }
            }
            h.write(&(s.n_removal_deferred as u64).to_le_bytes());
            h.finish()
        })
    })
    .unwrap_or_else(|e| panic!("int7 statistics: {e}"));
    assert_ne!(h, 0);
}

/// Tiled relief (`(n/8)^2` hard patches on the top face): the work counters per input triangle
/// stay bounded as the mesh grows (review round 4; the patches are independent, so the work per
/// triangle must not depend on how many there are).
#[test]
fn tiled_relief_work_per_triangle_is_bounded() {
    let mut per_tri: Vec<(f64, f64)> = Vec::new();
    // Debug builds are slow (n = 32 takes minutes unoptimized): they run n = 16 and 24 (4 and 9
    // patches); release runs n = 16 and 32 (4 and 16 patches).
    let sizes = if cfg!(debug_assertions) { [16u32, 24] } else { [16u32, 32] };
    for n in sizes {
        let inp = tiled_cube(n, 8, 2, 0.9);
        let t = Instant::now();
        let m = mesh(&inp, &format!("tiled n{n}"));
        let st = &m.stats;
        let tris = inp.tris.len() as f64;
        eprintln!(
            "tiled n{n}: {} tris, {:.2}s, boundary Steiner {}, updates {}, flips {}, deferred {}, rebuilds {}, steiner {}, min dihedral {:.3}",
            inp.tris.len(),
            t.elapsed().as_secs_f64(),
            st.n_boundary_steiner,
            st.n_constraint_updates,
            st.flips,
            st.n_removal_deferred,
            st.n_lookup_rebuilds,
            st.n_steiner,
            st.min_dihedral_deg
        );
        assert!(st.n_boundary_steiner > 0, "tiled n{n}: complete recovery did not run");
        assert_eq!(st.n_lookup_rebuilds, 0, "tiled n{n}");
        assert!(st.n_constraint_updates as f64 <= 7.0 * tris + 64.0 * st.n_boundary_steiner as f64, "tiled n{n}");
        per_tri.push((st.n_constraint_updates as f64 / tris, st.flips as f64 / tris));
    }
    // 2.25 .. 4 times the triangles and patches: per-triangle work within 25 % for constraint
    // updates and 50 % for flips.
    assert!(per_tri[1].0 <= 1.25 * per_tri[0].0, "updates per triangle {per_tri:?}");
    assert!(per_tri[1].1 <= 1.5 * per_tri[0].1 + 0.05, "flips per triangle {per_tri:?}");
}

/// Release timings of tiled relief (record only): `n = 16 .. 64`.
#[test]
#[ignore = "performance record"]
fn record_tiled_relief() {
    let ns: Vec<u32> = std::env::var("TILED_N").map(|s| s.split(',').filter_map(|x| x.parse().ok()).collect()).unwrap_or_else(|_| vec![16, 32, 48, 64]);
    for n in ns {
        let inp = tiled_cube(n, 8, 2, 0.9);
        let t = Instant::now();
        let m = mesh(&inp, &format!("tiled n{n}"));
        let s = t.elapsed().as_secs_f64();
        let st = &m.stats;
        let sp = st.spacing;
        eprintln!(
            "tiled n{n}: {} tris, {} tets in {s:.2} s; stage ms {:?}; temporary boundary {}; updates {}; flips {}; deferred {}; r mean {:.3} sd {:.3} p01 {:.3} min {:.3}; min dihedral {:.3}; {} quality violations; S4 {} violations (worst {:.4})",
            inp.tris.len(),
            m.tets.len(),
            st.stage_ms.map(|x| x.round()),
            st.n_boundary_steiner,
            st.n_constraint_updates,
            st.flips,
            st.n_removal_deferred,
            sp.mean,
            sp.std_dev,
            sp.p01,
            sp.min,
            st.min_dihedral_deg,
            st.n_quality_violations,
            st.n_s4_violations,
            st.worst_s4_margin
        );
    }
}
