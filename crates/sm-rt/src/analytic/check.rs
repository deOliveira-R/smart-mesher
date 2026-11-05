//! Sampled overlap and gap check of CSG leaves (`csg.overlap`, `csg.gap`).
//!
//! Completeness is the job of this check, not of tracking: it classifies stratified points and
//! points along random chords against every cell of the leaf. The samples are a pure function of
//! `(seed, leaf hash, index)`, so the findings are the same for any thread count.

use super::leaf::Leaf;
use rayon::prelude::*;
use sm_core::Point3;
use sm_geom_api::ModelTolerance;
use sm_rt_api::{BuildIssue, ComponentKind, HierInput, LeafInput, WorkCounters, hashed_direction};
use std::collections::BTreeSet;

/// Points tested along each random chord.
const PER_RAY: usize = 8;
/// Findings reported per code and leaf.
const MAX_REPORT: usize = 16;

fn mix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// Uniform in `[0, 1)` from the stream `(base, k)`.
fn unit(base: u64, k: u64) -> f64 {
    (mix(base ^ mix(k)) >> 11) as f64 / (1u64 << 53) as f64
}

enum Finding {
    Overlap([f64; 3], u32, u32),
    Gap([f64; 3]),
}

fn classify(leaf: &Leaf, q: [f64; 3]) -> Option<Finding> {
    let h = hashed_direction(Point3::new(q[0], q[1], q[2]));
    let d = [h.x, h.y, h.z];
    let mut cnt = WorkCounters::default();
    let mut hit: [u32; 2] = [0; 2];
    let mut n = 0;
    for c in 0..leaf.n_cells() {
        if leaf.contains(c, q, d, None, &mut cnt) {
            if n < 2 {
                hit[n] = c as u32;
            }
            n += 1;
        }
    }
    match n {
        0 => Some(Finding::Gap(q)),
        1 => None,
        _ => Some(Finding::Overlap(q, hit[0], hit[1])),
    }
}

/// Sample the CSG leaves of `input` for overlapping cells (`csg.overlap`: two cells contain the
/// point) and gaps (`csg.gap`: no cell contains a point of the leaf bounds). `n_points`
/// stratified points and `n_rays` random chords (each tested at 8 stratified points) per leaf;
/// every sample tests every cell. The samples depend only on `seed` and the leaf's hash.
///
/// A leaf whose bounds are not finite cannot be sampled and gives one `csg.unbounded` issue.
/// Each finding names the sample point, the cells, and the seed.
pub fn check_overlaps(input: &HierInput<'_>, n_points: usize, n_rays: usize, seed: u64) -> Vec<BuildIssue> {
    let mut out = Vec::new();
    let mut used: BTreeSet<usize> = BTreeSet::new();
    for c in &input.spec.components {
        if let ComponentKind::Leaf { input: i } = c {
            used.insert(*i);
        }
    }
    for i in used {
        let Some(LeafInput::Csg(csg)) = input.leaves.get(i) else { continue };
        let leaf = match Leaf::compile(csg, &ModelTolerance::DEFAULT) {
            Ok((l, _)) => l,
            Err(sm_rt_api::RtError::Build(v)) => {
                out.extend(v);
                continue;
            }
            Err(_) => continue,
        };
        let (lo, hi) = ([leaf.bounds.lo.x, leaf.bounds.lo.y, leaf.bounds.lo.z], [leaf.bounds.hi.x, leaf.bounds.hi.y, leaf.bounds.hi.z]);
        if (0..3).any(|k| !(lo[k].is_finite() && hi[k].is_finite() && hi[k] - lo[k] < 1e29)) {
            out.push(BuildIssue::new("csg.unbounded", format!("leaf {i}: the bounds are not finite, so the leaf cannot be sampled")).suggest("give the leaf bounds"));
            continue;
        }
        let lh = csg.content_hash().0;
        let base = mix(seed ^ lh as u64 ^ (lh >> 64) as u64);
        let g = ((n_points.max(1) as f64).cbrt().ceil() as usize).max(1);
        let strata = g * g * g;
        let n_total = n_points + n_rays * PER_RAY;
        let leaf_ref = &leaf;
        let findings: Vec<Option<Finding>> = (0..n_total)
            .into_par_iter()
            .map(|s| {
                let k = s as u64;
                let q = if s < n_points {
                    let st = s * strata / n_points.max(1);
                    let ix = [st % g, (st / g) % g, st / (g * g)];
                    let mut q = [0.0; 3];
                    for a in 0..3 {
                        let u = unit(base, k * 3 + a as u64);
                        q[a] = lo[a] + (hi[a] - lo[a]) * ((ix[a] as f64 + u) / g as f64);
                    }
                    q
                } else {
                    let (j, m) = ((s - n_points) / PER_RAY, (s - n_points) % PER_RAY);
                    let rb = mix(base ^ 0xA5A5 ^ (j as u64));
                    let o = [0, 1, 2].map(|a| lo[a] + (hi[a] - lo[a]) * unit(rb, a as u64));
                    let z = 2.0 * unit(rb, 3) - 1.0;
                    let phi = std::f64::consts::TAU * unit(rb, 4);
                    let sn = ((1.0 - z) * (1.0 + z)).max(0.0).sqrt();
                    let (sp, cp) = crate::vec3::sin_cos(phi);
                    let d = [sn * cp, sn * sp, z];
                    // Chord through the box.
                    let (mut t0, mut t1) = (f64::NEG_INFINITY, f64::INFINITY);
                    for a in 0..3 {
                        if d[a] != 0.0 {
                            let (a0, a1) = ((lo[a] - o[a]) / d[a], (hi[a] - o[a]) / d[a]);
                            t0 = t0.max(a0.min(a1));
                            t1 = t1.min(a0.max(a1));
                        }
                    }
                    let t = t0 + (t1 - t0) * ((m as f64 + unit(rb, 8 + m as u64)) / PER_RAY as f64);
                    [o[0] + d[0] * t, o[1] + d[1] * t, o[2] + d[2] * t]
                };
                classify(leaf_ref, q)
            })
            .collect();
        let (mut pairs, mut n_gap) = (BTreeSet::new(), 0usize);
        for f in findings.into_iter().flatten() {
            match f {
                Finding::Overlap(q, a, b) => {
                    if pairs.len() < MAX_REPORT && pairs.insert((a, b)) {
                        out.push(
                            BuildIssue::new("csg.overlap", format!("leaf {i}: cells {a} and {b} both contain the point (seed {seed}); cells must be disjoint"))
                                .at(Point3::new(q[0], q[1], q[2]))
                                .suggest("make the cells disjoint (subtract one from the other)"),
                        );
                    }
                }
                Finding::Gap(q) => {
                    if n_gap < MAX_REPORT {
                        out.push(BuildIssue::new("csg.gap", format!("leaf {i}: no cell contains the point (seed {seed})")).at(Point3::new(q[0], q[1], q[2])).suggest("add a cell for the gap or shrink the leaf bounds"));
                    }
                    n_gap += 1;
                }
            }
        }
    }
    out
}
