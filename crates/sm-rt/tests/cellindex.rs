//! W4-RT-CELLINDEX: the kd tiling of the analytic leaf.
//!
//! The oracle is the same tracker without the index (`with_index_threshold(usize::MAX)`: every
//! cell scanned, every surface tested) and `locate_checked(.., indexed = false)`.
#[path = "common/csg.rs"]
mod csg;

use csg::*;
use proptest::prelude::*;
use sm_core::Point3;
use sm_geom_api::Frame;
use sm_rt::{AnalyticTracker, CsgWorldOptions, MAX_FLAT_CELLS, csg_world};
use sm_rt_api::{BoundarySpec, BuildOptions, CellId, Scratch, StepEnd, Tracker};
use sm_testkit::strategies::config;
use sm_testkit::topo::pin_assembly;

fn u01(r: &mut SplitMix64) -> f64 {
    0.5 * (r.unit() + 1.0)
}

/// A jittered `g^3` grid of boxes (shared planes), each cut into an inner sphere and a shell,
/// plus `overlaps` spheres that straddle two boxes (so some points are claimed twice). Inside
/// bounds of half-width 20, the grid spans `[-16, 16]`, so a complement shell surrounds it.
fn random_model(seed: &str, g: usize, overlaps: usize) -> Fx {
    random_model_balls(seed, g, overlaps).0
}

/// [`random_model`] and the `(centre, radius)` of every sphere.
fn random_model_balls(seed: &str, g: usize, overlaps: usize) -> (Fx, Vec<(Point3, f64)>) {
    let mut balls = Vec::new();
    let mut r = rng(seed);
    let mut fx = Fx::new(20.0);
    let span = 32.0 / g as f64;
    let mut axes = [Vec::new(), Vec::new(), Vec::new()];
    for ax in &mut axes {
        for i in 0..=g {
            let jit = if i == 0 || i == g { 0.0 } else { 0.3 * span * r.unit() };
            ax.push(-16.0 + span * i as f64 + jit);
        }
    }
    let n = [v3(1.0, 0.0, 0.0), v3(0.0, 1.0, 0.0), v3(0.0, 0.0, 1.0)];
    let planes: Vec<Vec<_>> = (0..3).map(|a| axes[a].iter().map(|&c| fx.plane(p3(c * n[a].x, c * n[a].y, c * n[a].z), n[a])).collect()).collect();
    for i in 0..g {
        for j in 0..g {
            for k in 0..g {
                let b = [hs(planes[0][i], false), hs(planes[0][i + 1], true), hs(planes[1][j], false), hs(planes[1][j + 1], true), hs(planes[2][k], false), hs(planes[2][k + 1], true)];
                let c = p3(0.5 * (axes[0][i] + axes[0][i + 1]), 0.5 * (axes[1][j] + axes[1][j + 1]), 0.5 * (axes[2][k] + axes[2][k + 1]));
                let rad = 0.3 * span * (0.5 + u01(&mut r));
                let s = fx.sphere(c, rad);
                balls.push((c, rad));
                let mut inner = b.to_vec();
                inner.push(hs(s, true));
                fx.cell1("ball", &inner);
                let mut shell = b.to_vec();
                shell.push(hs(s, false));
                fx.cell1("shell", &shell);
            }
        }
    }
    for _ in 0..overlaps {
        let c = p3(32.0 * u01(&mut r) - 16.0, 32.0 * u01(&mut r) - 16.0, 32.0 * u01(&mut r) - 16.0);
        let s = fx.sphere(c, 0.6 * span);
        balls.push((c, 0.6 * span));
        fx.cell1("lump", &[hs(s, true)]);
    }
    (fx, balls)
}

fn pair(fx: &Fx) -> (AnalyticTracker, AnalyticTracker) {
    (fx.build().with_index_threshold(0), fx.build().with_index_threshold(usize::MAX))
}

/// Locate agreement: same cell, region, face and overlap outcome. Returns the number of overlaps seen.
fn check_locates(ix: &AnalyticTracker, or: &AnalyticTracker, pts: &[(Point3, sm_core::Vector3)]) -> usize {
    let (mut s1, mut s2) = (Scratch::new(), Scratch::new());
    let mut overlaps = 0;
    for (k, &(p, d)) in pts.iter().enumerate() {
        let a = ix.locate_checked(p, Some(d), true, &mut s1).unwrap();
        let b = or.locate_checked(p, Some(d), false, &mut s2).unwrap();
        assert_eq!(a, b, "point {k} {p:?} dir {d:?}");
        overlaps += usize::from(a.1);
    }
    overlaps
}

/// Random points over the bounds plus points within `tol` of surfaces: the points where seeded
/// rays cross a surface, moved by `0, +-tol/2, +-tol, +-2 tol` along the ray.
fn points(or: &AnalyticTracker, half: f64, n_rand: usize, n_rays: usize, seed: &str) -> Vec<(Point3, sm_core::Vector3)> {
    let mut r = rng(seed);
    let mut pts = Vec::new();
    for _ in 0..n_rand {
        pts.push((p3((2.0 * u01(&mut r) - 1.0) * half, (2.0 * u01(&mut r) - 1.0) * half, (2.0 * u01(&mut r) - 1.0) * half), random_dir(&mut r)));
    }
    let tol = or.info().tol.linear;
    let mut sc = Scratch::new();
    for _ in 0..n_rays {
        let o = p3((2.0 * u01(&mut r) - 1.0) * half, (2.0 * u01(&mut r) - 1.0) * half, (2.0 * u01(&mut r) - 1.0) * half);
        let d = random_dir(&mut r);
        let Ok(mut st) = or.init_state(o, d, None, &mut sc) else { continue };
        for _ in 0..40 {
            let Ok(s) = or.step(&mut st, 1e9, &mut sc) else { break };
            for f in [0.0, 0.5, -0.5, 1.0, -1.0, 2.0, -2.0] {
                pts.push((st.pos + d * (f * tol), d));
            }
            if matches!(s.end, StepEnd::Boundary { .. } | StepEnd::Lost(_)) {
                break;
            }
        }
    }
    pts
}

fn scale() -> usize {
    std::env::var("SMRT_CELLINDEX_N").ok().and_then(|v| v.parse().ok()).unwrap_or(if cfg!(debug_assertions) { 1 } else { 5 })
}

#[test]
fn points_in_the_on_face_band_beyond_a_cell_box_are_still_found() {
    // The extreme points of each sphere, within +-2 tol of its surface, along and across the axes.
    let (fx, balls) = random_model_balls("cellindex-band", 4, 6);
    let (ix, or) = pair(&fx);
    let tol = or.info().tol.linear;
    let mut pts = Vec::new();
    for (c, r) in balls {
        for ax in 0..3 {
            for sgn in [-1.0, 1.0] {
                for f in [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0] {
                    let mut e = [0.0; 3];
                    e[ax] = sgn;
                    let e = v3(e[0], e[1], e[2]);
                    for d in [e, -e, v3(0.0, 0.0, 1.0)] {
                        pts.push((c + e * (r + f * tol), d));
                    }
                }
            }
        }
    }
    check_locates(&ix, &or, &pts);
}

#[test]
fn locate_cold_with_the_index_equals_the_scan_on_three_models() {
    let k = scale();
    // 1. Random cells, with overlaps.
    let fx = random_model("cellindex-random", 4, 6);
    let (ix, or) = pair(&fx);
    assert!(ix.index_tiles().is_some_and(|t| t > 1) && or.index_tiles().is_none());
    let pts = points(&or, 22.0, 40_000 * k, 400 * k, "cellindex-random-pts");
    let ov = check_locates(&ix, &or, &pts);
    assert!(ov > 0, "the random model must produce overlap outcomes");
    // 2. The 12-ring pin cell with the threshold forced to 0.
    let mut fx = Fx::new(10.0);
    let bx = fx.box_planes(p3(-1.26, -1.26, -50.0), p3(1.26, 1.26, 50.0));
    let f = Frame::at(p3(0.0, 0.0, 0.0));
    let cyl: Vec<_> = (1..=12).map(|k| fx.cylinder(f, 0.1 * f64::from(k))).collect();
    let b: Vec<_> = bx.iter().map(|&p| hs(p, true)).collect();
    let mut c0 = b.clone();
    c0.push(hs(cyl[0], true));
    fx.cell1("r0", &c0);
    for k in 1..12 {
        let mut h = b.clone();
        h.push(hs(cyl[k - 1], false));
        h.push(hs(cyl[k], true));
        fx.cell1(&format!("r{k}"), &h);
    }
    let mut h = b.clone();
    h.push(hs(cyl[11], false));
    fx.cell1("mod", &h);
    // The default build has no index (13 cells): acceptance 6.
    assert!(fx.build().index_tiles().is_none());
    let (ix, or) = pair(&fx);
    assert!(ix.index_tiles().is_some());
    let pts = points(&or, 2.0, 40_000 * k, 400 * k, "cellindex-pin-pts");
    check_locates(&ix, &or, &pts);
    // 3. P2 flattened, with a complement region.
    let fix = pin_assembly(3, 1.26, 0.4096, 0.475, 100.0).unwrap();
    let w = csg_world(&fix.model, &fix.complex, &CsgWorldOptions { complement: Some("out".into()), ..CsgWorldOptions::default() }).unwrap();
    let flat = || w.analytic_tracker(&BoundarySpec::default(), &BuildOptions::default(), MAX_FLAT_CELLS).unwrap().0;
    assert!(flat().index_tiles().is_some(), "P2 (162 cells) is above the default threshold");
    let (ix, or) = (flat().with_index_threshold(0), flat().with_index_threshold(usize::MAX));
    let pts = points(&or, 7.0, 40_000 * k, 400 * k, "cellindex-p2-pts");
    check_locates(&ix, &or, &pts);
}

fn run(t: &AnalyticTracker, o: Point3, d: sm_core::Vector3, sc: &mut Scratch) -> Vec<String> {
    let mut out = Vec::new();
    let Ok(mut st) = t.init_state(o, d, None, sc) else { return out };
    for _ in 0..4000 {
        let Ok(s) = t.step(&mut st, 1e9, sc) else {
            out.push("err".into());
            break;
        };
        out.push(format!("{s:?}"));
        if matches!(s.end, StepEnd::Boundary { .. } | StepEnd::Lost(_)) {
            break;
        }
    }
    out
}

/// Rays from the complement shell: the event sequences (surface, distance bits, next cell) are equal.
fn check_rays(ix: &AnalyticTracker, or: &AnalyticTracker, half: f64, n: usize, seed: &str) -> usize {
    let mut r = rng(seed);
    let (mut s1, mut s2) = (Scratch::new(), Scratch::new());
    let mut compared = 0;
    for k in 0..n {
        let o = p3((2.0 * u01(&mut r) - 1.0) * half, (2.0 * u01(&mut r) - 1.0) * half, (2.0 * u01(&mut r) - 1.0) * half);
        let d = random_dir(&mut r);
        let (a, b) = (run(ix, o, d, &mut s1), run(or, o, d, &mut s2));
        assert_eq!(a, b, "ray {k} from {o:?} along {d:?}");
        compared += a.len();
    }
    compared
}

#[test]
fn complement_rays_have_the_events_of_the_all_surfaces_path() {
    let fx = random_model("cellindex-rays", 4, 0);
    let (ix, or) = pair(&fx);
    assert!(check_rays(&ix, &or, 19.0, 1000, "cellindex-rays-a") > 3000);
    // Axis-parallel rays (the traversal's d == 0 branch) and rays along grid planes.
    let mut r = rng("cellindex-axis");
    let (mut s1, mut s2) = (Scratch::new(), Scratch::new());
    for k in 0..300 {
        let ax = k % 3;
        let mut d = [0.0; 3];
        d[ax] = if k % 2 == 0 { 1.0 } else { -1.0 };
        let o = p3(19.0 * (2.0 * u01(&mut r) - 1.0), 19.0 * (2.0 * u01(&mut r) - 1.0), 19.0 * (2.0 * u01(&mut r) - 1.0));
        let d = v3(d[0], d[1], d[2]);
        assert_eq!(run(&ix, o, d, &mut s1), run(&or, o, d, &mut s2), "axis ray {k}");
    }
    // P2 flattened with the complement.
    let fix = pin_assembly(3, 1.26, 0.4096, 0.475, 100.0).unwrap();
    let w = csg_world(&fix.model, &fix.complex, &CsgWorldOptions { complement: Some("out".into()), ..CsgWorldOptions::default() }).unwrap();
    let flat = || w.analytic_tracker(&BoundarySpec::default(), &BuildOptions::default(), MAX_FLAT_CELLS).unwrap().0;
    let (ix, or) = (flat().with_index_threshold(0), flat().with_index_threshold(usize::MAX));
    assert!(check_rays(&ix, &or, 6.0, 1000, "cellindex-rays-p2") > 2000);
}

#[test]
fn complement_steps_on_p2_test_a_fraction_of_the_surfaces() {
    p2_surface_evals(3, 500, 4.0);
}

/// Full-scale P2 (17 x 17 pins, 10404 surfaces): the ratio is pinned, not only measured.
#[test]
#[ignore = "full-scale P2: cargo test --release -p sm-rt --test cellindex -- --ignored"]
fn complement_steps_on_p2_full_test_a_hundredth_of_the_surfaces() {
    p2_surface_evals(17, 200, 100.0);
}

/// Surface evaluations per complement step on `pin_assembly(n)`: index against the oracle.
/// The index must test at most `1 / ratio` of the surfaces.
fn p2_surface_evals(n: usize, rays: usize, ratio: f64) {
    let fix = pin_assembly(n, 1.26, 0.4096, 0.475, 100.0).unwrap();
    let w = csg_world(&fix.model, &fix.complex, &CsgWorldOptions { complement: Some("out".into()), ..CsgWorldOptions::default() }).unwrap();
    let flat = || w.analytic_tracker(&BoundarySpec::default(), &BuildOptions::default(), MAX_FLAT_CELLS).unwrap().0;
    let (ix, or) = (flat(), flat().with_index_threshold(usize::MAX));
    let ns = or.surface_kinds().len() as f64;
    let mut r = rng("cellindex-counters");
    let mut per = [(0u64, 0u64); 2];
    let half = 1.26 * n as f64 / 2.0 + 1.0;
    for _ in 0..rays {
        let o = p3(half * (2.0 * u01(&mut r) - 1.0), half * (2.0 * u01(&mut r) - 1.0), 40.0 + 20.0 * u01(&mut r));
        let d = random_dir(&mut r);
        for (i, t) in [&ix, &or].into_iter().enumerate() {
            let mut sc = Scratch::new();
            let Ok(mut st) = t.init_state(o, d, None, &mut sc) else { continue };
            for _ in 0..400 {
                let comp = st.cell == CellId::COMPLEMENT;
                let before = sc.counters.surface_evals;
                let Ok(s) = t.step(&mut st, 1e9, &mut sc) else { break };
                if comp {
                    per[i].0 += sc.counters.surface_evals - before;
                    per[i].1 += 1;
                }
                if matches!(s.end, StepEnd::Boundary { .. } | StepEnd::Lost(_)) {
                    break;
                }
            }
        }
    }
    let (a, b) = (per[0].0 as f64 / per[0].1 as f64, per[1].0 as f64 / per[1].1 as f64);
    eprintln!("surface evals per complement step on P2: index {a:.1}, all-surfaces {b:.1}, n_surfaces {ns}");
    assert!(per[0].1 > 100 && per[1].1 > 100);
    assert!(b >= ns, "the oracle tests every surface");
    assert!(a * ratio <= ns, "index {a} vs {ns} surfaces");
}

/// Tori: the quartic is bracketed by its `t_max`, so the tile walk must not change the roots.
#[test]
fn complement_rays_agree_on_a_model_with_tori() {
    let (mut fx, _) = random_model_balls("cellindex-tori", 4, 0);
    let mut r = rng("cellindex-tori-frames");
    for _ in 0..24 {
        let c = p3(28.0 * u01(&mut r) - 14.0, 28.0 * u01(&mut r) - 14.0, 28.0 * u01(&mut r) - 14.0);
        let f = random_frame(&mut r, c);
        let t = fx.torus(f, 2.0 + 2.0 * u01(&mut r), 0.5 + 0.5 * u01(&mut r));
        fx.cell1("torus", &[hs(t, true)]);
    }
    let (ix, or) = pair(&fx);
    assert!(ix.index_tiles().is_some());
    assert!(check_rays(&ix, &or, 19.0, 1500, "cellindex-tori-rays") > 4000);
    let pts = points(&or, 21.0, 2000, 40, "cellindex-tori-pts");
    check_locates(&ix, &or, &pts);
}

/// Heavily overlapping boxes cannot blow the tile lists up.
#[test]
fn split_has_a_total_size_budget() {
    let mut fx = Fx::new(20.0);
    let mut r = rng("cellindex-budget");
    let n = 600;
    for _ in 0..n {
        let c = p3(30.0 * u01(&mut r) - 15.0, 30.0 * u01(&mut r) - 15.0, 30.0 * u01(&mut r) - 15.0);
        let s = fx.sphere(c, 4.0 + 2.0 * u01(&mut r));
        fx.cell1("lump", &[hs(s, true)]);
    }
    let (ix, or) = pair(&fx);
    let listed = ix.index_listed_cells().unwrap();
    eprintln!("budget listed {listed} of {n}");
    assert!(listed <= 16 * n, "listed {listed} for {n} cells");
    assert!(ix.index_tiles().unwrap() > 1);
    // The cut-off tiles are still correct.
    let pts = points(&or, 21.0, 1500, 0, "cellindex-budget-pts");
    check_locates(&ix, &or, &pts);
}

#[test]
fn memory_bytes_counts_the_index() {
    let fx = random_model("cellindex-mem", 4, 0);
    let (ix, or) = pair(&fx);
    assert!(ix.info().memory_bytes > or.info().memory_bytes);
    let by_default = fx.build();
    assert_eq!(by_default.info().memory_bytes, ix.info().memory_bytes);
    assert_eq!(by_default.with_index_threshold(usize::MAX).info().memory_bytes, or.info().memory_bytes);
}

#[test]
fn index_is_bit_identical_across_builds_and_thread_counts() {
    let fx = random_model("cellindex-det", 4, 3);
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap().install(|| fx.build().with_index_threshold(0).index_digest());
    let many = rayon::ThreadPoolBuilder::new().num_threads(4).build().unwrap().install(|| fx.build().with_index_threshold(0).index_digest());
    assert!(one.is_some());
    assert_eq!(one, many);
    assert_eq!(one, fx.build().with_index_threshold(0).index_digest());
    // Different cells give a different index.
    assert_ne!(one, random_model("cellindex-det", 5, 3).build().with_index_threshold(0).index_digest());
    // The default build of a 128-cell model has the index; a tiny one does not.
    assert_eq!(fx.build().index_digest(), one);
}

proptest! {
    #![proptest_config(config(12))]

    /// Random cell sets: same locates and the same complement events.
    #[test]
    fn random_cell_sets_agree_with_the_scan(g in 2usize..5, overlaps in 0usize..5, seed in 0u32..1000) {
        let fx = random_model(&format!("cellindex-prop-{seed}"), g, overlaps);
        let (ix, or) = pair(&fx);
        let pts = points(&or, 21.0, 600, 12, &format!("cellindex-prop-pts-{seed}"));
        check_locates(&ix, &or, &pts);
        check_rays(&ix, &or, 19.0, 20, &format!("cellindex-prop-rays-{seed}"));
    }
}

/// Candidate cells tested per segment on `pin_assembly(n)` (all steps, interior and complement).
/// Returns `(candidates, segments)` of the indexed tracker.
fn p2_candidates(n: usize, rays: usize) -> (u64, u64) {
    let fix = pin_assembly(n, 1.26, 0.4096, 0.475, 100.0).unwrap();
    let w = csg_world(&fix.model, &fix.complex, &CsgWorldOptions { complement: Some("out".into()), ..CsgWorldOptions::default() }).unwrap();
    let ix = w.analytic_tracker(&BoundarySpec::default(), &BuildOptions::default(), MAX_FLAT_CELLS).unwrap().0;
    let mut r = rng("cellindex-candidates");
    let half = 1.26 * n as f64 / 2.0 + 1.0;
    let (mut cand, mut segs) = (0u64, 0u64);
    for _ in 0..rays {
        let o = p3(half * (2.0 * u01(&mut r) - 1.0), half * (2.0 * u01(&mut r) - 1.0), 40.0 + 20.0 * u01(&mut r));
        let d = random_dir(&mut r);
        let mut sc = Scratch::new();
        let Ok(mut st) = ix.init_state(o, d, None, &mut sc) else { continue };
        for _ in 0..400 {
            let Ok(s) = ix.step(&mut st, 1e9, &mut sc) else { break };
            segs += 1;
            if matches!(s.end, StepEnd::Boundary { .. } | StepEnd::Lost(_)) {
                break;
            }
        }
        cand += sc.counters.locate_candidates;
    }
    (cand, segs)
}

/// Reduced P2: the shared top and bottom z-planes must not list every cell as a candidate.
#[test]
fn p2_candidates_per_segment_stay_small() {
    let (c, s) = p2_candidates(3, 300);
    let per = c as f64 / s as f64;
    eprintln!("P2 reduced (n=3): {per:.2} locate candidates per segment over {s} segments");
    assert!(per < 25.0, "{per}");
}

/// Full-scale P2: was about 977 candidates per segment (shared z-planes list every cell).
#[test]
#[ignore = "full-scale P2: cargo test --release -p sm-rt --test cellindex -- --ignored"]
fn p2_full_candidates_per_segment_stay_small() {
    let (c, s) = p2_candidates(17, 200);
    let per = c as f64 / s as f64;
    eprintln!("P2 full (n=17): {per:.2} locate candidates per segment over {s} segments");
    assert!(per < 25.0, "{per}");
}
