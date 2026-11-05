//! W3b-BOOL: candidate-pair BVH. Pairs equal the brute-force scan; the work counter
//! `candidate_tests` stays within a fixed bound on a 1e4-face assembly; results do not depend on
//! the thread count.

mod common;

use common::*;
use sm_boolean::{CandidateStats, candidate_pair_stats};
use sm_core::EntityId;
use sm_topo_api::{CellComplex, Dim, TopologyView, TopologyViewExt};

/// `nx * ny * nz` unit bricks on a grid of the given pitch (pitch 1 = touching).
fn grid(nx: usize, ny: usize, nz: usize, pitch: f64) -> CellComplex {
    let mut t = CellComplex::new();
    let mut op = 1;
    for i in 0..nx {
        for j in 0..ny {
            for k in 0..nz {
                brick(&mut t, op, [i as f64 * pitch, j as f64 * pitch, k as f64 * pitch], [1.0; 3]);
                op += 1;
            }
        }
    }
    t
}

fn faces(t: &CellComplex) -> Vec<EntityId> {
    t.cells(Dim::Face).to_vec()
}

fn brute(t: &CellComplex, fa: &[EntityId], fb: &[EntityId], pad: f64) -> Vec<(EntityId, EntityId)> {
    // Boxes of planar axis-aligned bricks: derive from the vertices of each face.
    let bx = |f: EntityId| {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for &e in t.face_edges(f) {
            let (s, e2) = t.edge_ends(e).unwrap();
            for v in [s, e2] {
                if let sm_topo_api::Carrier::Point(p) = t.carrier(v) {
                    for (k, c) in [p.x, p.y, p.z].into_iter().enumerate() {
                        lo[k] = lo[k].min(c - pad);
                        hi[k] = hi[k].max(c + pad);
                    }
                }
            }
        }
        (lo, hi)
    };
    let mut out = Vec::new();
    for &a in fa {
        let (al, ah) = bx(a);
        for &b in fb {
            let (bl, bh) = bx(b);
            if a != b && (0..3).all(|k| al[k] <= bh[k] && bl[k] <= ah[k]) {
                out.push((a, b));
            }
        }
    }
    out.sort();
    out
}

#[test]
fn pairs_equal_the_brute_force_scan() {
    for pitch in [1.0, 1.0 + 1e-3, 0.6] {
        let t = grid(4, 4, 3, pitch);
        let all = faces(&t);
        let half: Vec<EntityId> = all.iter().copied().step_by(2).collect();
        let pad = t.tolerance().linear_max;
        for (fa, fb) in [(&all, &all), (&half, &all), (&all, &half)] {
            let (got, _) = candidate_pair_stats(&t, fa, fb, pad);
            assert_eq!(got, brute(&t, fa, fb, pad), "pitch {pitch}");
        }
    }
}

#[test]
fn candidate_tests_within_the_fixed_bound_on_a_1e4_face_assembly() {
    // 11 x 11 x 14 bricks = 10164 faces.
    let t = grid(11, 11, 14, 2.0);
    let all = faces(&t);
    let f = all.len();
    assert!(f >= 10_000);
    let (pairs, stats) = candidate_pair_stats(&t, &all, &all, t.tolerance().linear_max);
    // Separated bricks: only the faces of one brick meet each other (6 faces, 5 partners each...
    // adjacent faces share edges), nothing else.
    assert!(!pairs.is_empty());
    let bound = 500_000u64;
    assert!(stats.candidate_tests <= bound, "candidate_tests {} > {bound}", stats.candidate_tests);
    let c = stats.candidate_tests as f64 / (f as f64 * (f as f64).log2());
    eprintln!("F = {f}, candidate_tests = {}, node_visits = {}, c = {c:.4}", stats.candidate_tests, stats.node_visits);
}

#[test]
fn counters_and_pairs_do_not_depend_on_threads() {
    let t = grid(5, 5, 5, 1.0);
    let all = faces(&t);
    let pad = t.tolerance().linear_max;
    let run = |n: usize| -> (Vec<(EntityId, EntityId)>, CandidateStats) {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(n).build().unwrap();
        pool.install(|| candidate_pair_stats(&t, &all, &all, pad))
    };
    let one = run(1);
    assert_eq!(one, run(4));
    assert_eq!(one, run(1));
}
