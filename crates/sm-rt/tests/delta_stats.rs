//! W3b-RT-DELTA acceptance 5, 6, 6b and 7: unbiasedness of surface (track-length), delta
//! (collision) and mixed flights against analytic results. Cross sections live in the tests
//! (keyed by region name); sm-rt never sees one. The full runs are `#[ignore]` (release, fixed
//! seeds, 4 sigma); the `6b` tests are their small forms and run on every `cargo test`.
#![allow(clippy::needless_range_loop, clippy::neg_cmp_op_on_partial_ord, clippy::too_many_arguments)]
mod common;
mod dcommon;
use common::*;
use dcommon::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, SerialAlloc};
use sm_rt::MeshTracker;
use sm_rt_api::{CellId, ModePolicy, NodeId, RegionRule, Scratch, Tracker};
use sm_testkit::meshes::{MeshFixture, SetSpec, hex_block_two_volumes, with_sets};

/// Slab of thickness `L = 20` (x), lateral mirrors, vacuum ends.
fn slab(p: u32) -> MeshTracker {
    let mesh = hex_block([20, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    build_part(&mesh, &RegionRule::ByVolumeClassification, &mirror_lateral(), popts(p, false))
}

fn slab_source(r: &mut SplitMix64) -> (sm_core::Point3, [f64; 3]) {
    (p3([20.0 * unit01(r), 2.0 * unit01(r), 2.0 * unit01(r)]), iso_dir(r))
}

fn within(what: &str, got: f64, want: f64, se: f64, k: f64) {
    assert!((got - want).abs() <= k * se.max(1e-300), "{what}: {got} vs {want} (se {se}, {k} sigma = {})", k * se);
}

/// Leakage and the rejection fraction of the slab at optical thickness `tau`.
fn slab_run(tau: f64, scale: f64, n: usize, seed: u64, mode: ModePolicy) -> Score {
    let t = slab(8);
    let sigma = vec![tau / 20.0; t.regions().0.len()];
    let maj = node_majorants(&t, &sigma, scale);
    let run = Run { t: &t, sigma: &sigma, scatter: None, maj: &maj, block: &[], mode };
    run.batch(n, seed, &slab_source)
}

#[test]
fn acc6b_i_slab_leakage_and_rejection_fraction() {
    let (tau, n) = (1.0, 10_000);
    let s = slab_run(tau, 3.0, n, 1, ModePolicy::Delta);
    assert_eq!(s.lost, 0);
    assert_eq!(s.histories, n as u64);
    let want = slab_leak(tau);
    let p = s.leaked as f64 / n as f64;
    within("leakage", p, want, binom_se(want, n as f64), 10.0);
    let rej = s.rejected as f64 / s.tentative as f64;
    within("rejection fraction", rej, 2.0 / 3.0, binom_se(2.0 / 3.0, s.tentative as f64), 10.0);
    // Surface (track-length) gives the same leakage with no tentative collisions at all.
    let s2 = slab_run(tau, 1.0, n, 1, ModePolicy::Surface);
    within("surface leakage", s2.leaked as f64 / n as f64, want, binom_se(want, n as f64), 10.0);
    assert_eq!(s2.tentative, 0);
}

/// Two-region beam: `S1` on `x < a`, `S2` beyond.
fn beam(mode: ModePolicy, mixed: bool, s1: f64, s2: f64, a: u32, l: u32, n: usize, seed: u64) -> (f64, f64) {
    let fx = hex_block_two_volumes([l, 1, 1], [1.0; 3], a).unwrap();
    let t = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(1, false));
    let mut sc = Scratch::new();
    let low = t.locate(p3([0.5, 0.5, 0.5]), None, None, &mut sc).unwrap().region;
    let sigma: Vec<f64> = (0..t.regions().0.len()).map(|r| if r == low.get() { s1 } else { s2 }).collect();
    let maj = node_majorants(&t, &sigma, 1.0);
    // The partition separates the low-sigma leaf: a leaf holding only that region exists and
    // its majorant is exactly s1 (the global majorant would give s2).
    let h = t.majorant_view().unwrap().hierarchy();
    let pure_low = (0..h.node_count()).filter(|&i| h.node(NodeId::new(i)).unwrap().is_leaf()).filter(|&i| h.set_regions(h.node(NodeId::new(i)).unwrap().set).unwrap() == [low]).count();
    assert!(pure_low > 0, "the low-sigma region must have a leaf of its own");
    assert!(maj.contains(&s1));
    let block: Vec<NodeId> = if mixed {
        // Mixed policy: even-numbered leaves delta, odd-numbered surface (blocked).
        (0..h.node_count()).filter(|&i| h.node(NodeId::new(i)).unwrap().is_leaf() && i % 2 == 1).map(NodeId::new).collect()
    } else {
        Vec::new()
    };
    let run = Run { t: &t, sigma: &sigma, scatter: None, maj: &maj, block: &block, mode };
    let s = run.batch(n, seed, &|_r| (p3([0.0, 0.5, 0.5]), [1.0, 0.0, 0.0]));
    assert_eq!(s.lost, 0);
    (s.leaked as f64 / n as f64, (-(s1 * f64::from(a) + s2 * f64::from(l - a))).exp())
}

#[test]
fn acc6b_ii_two_region_beam_in_surface_delta_and_mixed() {
    let n = 10_000;
    for (mode, mixed) in [(ModePolicy::Surface, false), (ModePolicy::Delta, false), (ModePolicy::Delta, true)] {
        let (got, want) = beam(mode, mixed, 0.5, 2.0, 1, 2, n, 2);
        within(&format!("transmission {mode:?} mixed={mixed}"), got, want, binom_se(want, n as f64), 10.0);
    }
}

#[test]
#[ignore = "full statistical run: cargo test --release -p sm-rt --test delta_stats -- --ignored"]
fn acc5_homogeneous_slab_full() {
    let n = 200_000;
    for tau in [0.1, 1.0, 10.0] {
        let want = slab_leak(tau);
        // Surface: leakage and the track-length flux (1 - P_esc) / sigma per source particle.
        let t = slab(8);
        let sg = tau / 20.0;
        let sigma = vec![sg; t.regions().0.len()];
        let maj = node_majorants(&t, &sigma, 1.0);
        let run = Run { t: &t, sigma: &sigma, scatter: None, maj: &maj, block: &[], mode: ModePolicy::Surface };
        let s = run.batch(n, 11, &slab_source);
        let nf = n as f64;
        within(&format!("tau {tau} surface leakage"), s.leaked as f64 / nf, want, binom_se(want, nf), 4.0);
        let flux = (1.0 - want) / sg;
        let mean = s.sum_track_total / nf;
        let se = ((s.sq_track_total / nf - mean * mean).max(0.0) / nf).sqrt();
        within(&format!("tau {tau} track-length flux"), mean, flux, se, 4.0);
        let tl: f64 = s.track.iter().sum::<f64>() / nf;
        assert!((tl - mean).abs() < 1e-9 * (1.0 + mean));
        // Delta with the majorant scaled: the collision flux and the rejection fraction.
        for scale in [1.0, 3.0, 10.0] {
            let maj = node_majorants(&t, &sigma, scale);
            let run = Run { t: &t, sigma: &sigma, scatter: None, maj: &maj, block: &[], mode: ModePolicy::Delta };
            let d = run.batch(n, 12, &slab_source);
            within(&format!("tau {tau} x{scale} delta leakage"), d.leaked as f64 / nf, want, binom_se(want, nf), 4.0);
            let coll: f64 = d.coll.iter().sum::<f64>() / nf;
            within(&format!("tau {tau} x{scale} collision flux"), coll, flux, binom_se(1.0 - want, nf) / sg, 4.0);
            let rej = d.rejected as f64 / d.tentative as f64;
            within(&format!("tau {tau} x{scale} rejection"), rej, 1.0 - 1.0 / scale, binom_se(1.0 - 1.0 / scale, d.tentative as f64), 4.0);
        }
    }
}

#[test]
#[ignore = "full statistical run: cargo test --release -p sm-rt --test delta_stats -- --ignored"]
fn acc6_two_region_beam_full() {
    let n = 200_000;
    for (s1, s2, a, l) in [(0.5, 2.0, 1, 2), (0.1, 1.5, 3, 8), (1.0, 0.2, 2, 6)] {
        for (mode, mixed) in [(ModePolicy::Surface, false), (ModePolicy::Delta, false), (ModePolicy::Delta, true)] {
            let (got, want) = beam(mode, mixed, s1, s2, a, l, n, 3);
            within(&format!("beam {s1},{s2},{a},{l} {mode:?} mixed={mixed}"), got, want, binom_se(want, n as f64), 4.0);
        }
    }
}

/// 4 materials by `(i + j + k) mod 4`, `n^3` cells, x fastest.
fn checker(n: u32) -> MeshFixture {
    let base = MeshFixture {
        name: "checker",
        mesh: hex_block([n; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap(),
        cell_volumes: vec![1.0; (n * n * n) as usize],
        volume: f64::from(n * n * n),
        area: 6.0 * f64::from(n * n),
        boundary_faces: (6 * n * n) as usize,
    };
    let mut sets: Vec<Vec<CellIx>> = vec![Vec::new(); 4];
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                sets[((i + j + k) % 4) as usize].push(CellIx::new((i + n * (j + n * k)) as usize));
            }
        }
    }
    let specs: Vec<SetSpec> = sets.into_iter().enumerate().map(|(m, cs)| SetSpec::new(&format!("m{m}"), sm_mesh_api::SetMembers::Cells(cs))).collect();
    with_sets(base, &specs).unwrap()
}

struct Het {
    t: MeshTracker,
    sigma: Vec<f64>,
    scat: Vec<f64>,
    maj: Vec<f64>,
    odd: Vec<NodeId>,
    n: u32,
}

fn het(n: u32, leaf: u32) -> Het {
    let fx = checker(n);
    let rule = RegionRule::ByCellSets((0..4).map(|m| format!("m{m}")).collect());
    let t = build_part(&fx.mesh, &rule, &mirror_all(), popts(leaf, false));
    let c = [0.2, 0.5, 0.7, 0.9];
    let sg = [0.4, 0.8, 1.2, 1.6];
    let mat = |name: &str| usize::from(name.as_bytes()[1] - b'0');
    let sigma = sigma_by_name(&t, |nm| sg[mat(nm)]);
    let scat = sigma_by_name(&t, |nm| c[mat(nm)]);
    let maj = node_majorants(&t, &sigma, 1.0);
    let h = t.majorant_view().unwrap().hierarchy();
    let odd: Vec<NodeId> = (0..h.node_count()).filter(|&i| h.node(NodeId::new(i)).unwrap().is_leaf() && i % 2 == 1).map(NodeId::new).collect();
    Het { t, sigma, scat, maj, odd, n }
}

/// Per-batch per-cell integrated flux and the balance `sum sigma_a * flux`, over `batches`
/// fixed batches (seeds from the batch index).
struct Est {
    cell: Vec<Vec<f64>>,
    balance: Vec<f64>,
}

fn estimate(h: &Het, mode: ModePolicy, block: &[NodeId], use_coll: bool, per_batch: usize, batches: usize) -> Est {
    let run = Run { t: &h.t, sigma: &h.sigma, scatter: Some(&h.scat), maj: &h.maj, block, mode };
    let nn = f64::from(h.n);
    let src = move |r: &mut SplitMix64| (p3([nn * unit01(r), nn * unit01(r), nn * unit01(r)]), iso_dir(r));
    let mut cell = Vec::new();
    let mut balance = Vec::new();
    for b in 0..batches {
        let s = run.batch(per_batch, 1000 + b as u64, &src);
        assert_eq!(s.lost, 0);
        // Mixed: both tallies hold their share; pure modes have one of them empty.
        let flux: Vec<f64> = (0..s.track.len()).map(|c| (s.track[c] + if use_coll { s.coll[c] } else { 0.0 }) / per_batch as f64).collect();
        let mut sc = Scratch::new();
        let _ = &mut sc;
        let bal: f64 = flux
            .iter()
            .enumerate()
            .map(|(c, f)| {
                let r = h.t.cell_region(CellId::new(c)).unwrap().get();
                h.sigma[r] * (1.0 - h.scat[r]) * f
            })
            .sum();
        cell.push(flux);
        balance.push(bal);
    }
    Est { cell, balance }
}

fn mean_se(xs: &[f64]) -> (f64, f64) {
    let n = xs.len() as f64;
    let m = xs.iter().sum::<f64>() / n;
    let v = xs.iter().map(|x| (x - m) * (x - m)).sum::<f64>() / (n - 1.0);
    (m, (v / n).sqrt())
}

fn check_het(h: &Het, per_batch: usize, batches: usize, k_bal: f64, k_cell: f64) {
    let surf = estimate(h, ModePolicy::Surface, &[], false, per_batch, batches);
    let delta = estimate(h, ModePolicy::Delta, &[], true, per_batch, batches);
    let mixed = estimate(h, ModePolicy::Delta, &h.odd, true, per_batch, batches);
    for (name, e) in [("track-length", &surf), ("collision", &delta), ("mixed", &mixed)] {
        let (m, se) = mean_se(&e.balance);
        within(&format!("balance {name}"), m, 1.0, se, k_bal);
    }
    // Per-cell agreement, sigma from the batches.
    let ncell = surf.cell[0].len();
    let mut worst: f64 = 0.0;
    for (an, a, bn, b) in [("track", &surf, "collision", &delta), ("track", &surf, "mixed", &mixed)] {
        for c in 0..ncell {
            let diff: Vec<f64> = (0..batches).map(|i| a.cell[i][c] - b.cell[i][c]).collect();
            let (m, se) = mean_se(&diff);
            let z = m.abs() / se.max(1e-300);
            worst = worst.max(z);
            assert!(z <= k_cell, "cell {c}: {an} vs {bn} differ by {m} ({z} sigma)");
        }
    }
    eprintln!("worst per-cell deviation: {worst:.2} sigma");
}

#[test]
fn acc7_small_heterogeneous_block_estimators_agree() {
    // CI form: hex_block(4), few histories, loose bounds (the balance identity must hold).
    let h = het(4, 2);
    let per = if cfg!(debug_assertions) { 600 } else { 5_000 };
    let surf = estimate(&h, ModePolicy::Surface, &[], false, per, 10);
    let delta = estimate(&h, ModePolicy::Delta, &[], true, per, 10);
    let mixed = estimate(&h, ModePolicy::Delta, &h.odd, true, per, 10);
    for (name, e) in [("track-length", &surf), ("collision", &delta), ("mixed", &mixed)] {
        let (m, se) = mean_se(&e.balance);
        within(&format!("balance {name}"), m, 1.0, se, 10.0);
    }
}

#[test]
#[ignore = "full statistical run: cargo test --release -p sm-rt --test delta_stats -- --ignored"]
fn acc7_heterogeneous_block_full() {
    let h = het(8, 2);
    check_het(&h, 10_000, 20, 4.0, 5.0);
}
