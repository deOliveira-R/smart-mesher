//! Reader-side stitching: merge vertices within a tolerance and duplicate edges.
//!
//! Everything here is ordered by index (STEP id order), never by hash order, so the result is
//! independent of thread count and platform.

use std::collections::BTreeMap;

use sm_core::Point3;
use sm_topo_api::Sense;

use crate::model::{CurveRec, Edge};
use sm_geom_api::ParametricCurve;

fn cell(p: f64, h: f64) -> i64 {
    // Saturating cast: non-finite or huge values land in a boundary cell, which is harmless.
    (p / h).floor() as i64
}

/// Weld points: `out[i]` is the index of the representative of point `i`.
///
/// Points are visited in index order. Point `i` joins the lowest-index representative within
/// `tol` (Euclidean) of it, else it becomes a representative itself, so the lowest index of a
/// group always wins. Representatives live in a sorted grid of cell size `tol`
/// (`BTreeMap`, O(n log n)); the result depends only on the points and the tolerance.
/// `tol <= 0` (or not finite) leaves every point alone.
pub fn weld_vertices(points: &[Point3], tol: f64) -> Vec<usize> {
    let mut rep: Vec<usize> = (0..points.len()).collect();
    if !(tol > 0.0 && tol.is_finite()) {
        return rep;
    }
    let mut grid: BTreeMap<(i64, i64, i64), Vec<usize>> = BTreeMap::new();
    for (i, p) in points.iter().enumerate() {
        let lo = (cell(p.x - tol, tol), cell(p.y - tol, tol), cell(p.z - tol, tol));
        let hi = (cell(p.x + tol, tol), cell(p.y + tol, tol), cell(p.z + tol, tol));
        let mut best: Option<usize> = None;
        for ix in lo.0..=hi.0 {
            for iy in lo.1..=hi.1 {
                for iz in lo.2..=hi.2 {
                    let Some(reps) = grid.get(&(ix, iy, iz)) else { continue };
                    for &j in reps {
                        if best.is_some_and(|b| b < j) {
                            break;
                        }
                        if (points[j] - *p).norm() <= tol {
                            best = Some(j);
                            break;
                        }
                    }
                }
            }
        }
        match best {
            Some(j) => rep[i] = j,
            None => grid.entry((cell(p.x, tol), cell(p.y, tol), cell(p.z, tol))).or_default().push(i),
        }
    }
    rep
}

/// Five points of an edge, from its start vertex to its end vertex.
fn samples(e: &Edge, curve: &dyn ParametricCurve) -> [Point3; 5] {
    let (lo, hi) = (e.range.lo, e.range.hi);
    core::array::from_fn(|k| {
        let f = k as f64 / 4.0;
        let t = if e.sense == Sense::Forward { lo + f * (hi - lo) } else { hi - f * (hi - lo) };
        curve.eval(t)
    })
}

/// Merge duplicate edges: same (welded) end vertices and five sampled curve points within
/// `tol`. Returns, for each input edge, the index of the kept edge (the lowest of its group)
/// and the direction of the input edge relative to the kept one.
pub(crate) fn weld_edges(edges: &[Edge], curves: &[CurveRec], tol: f64) -> Vec<(usize, Sense)> {
    let mut out: Vec<(usize, Sense)> = (0..edges.len()).map(|i| (i, Sense::Forward)).collect();
    if tol.is_nan() || tol <= 0.0 {
        return out;
    }
    let mut groups: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
    let mut cache: Vec<Option<[Point3; 5]>> = vec![None; edges.len()];
    for (i, e) in edges.iter().enumerate() {
        let key = (e.start.min(e.end), e.start.max(e.end));
        let kept = groups.entry(key).or_default();
        let si = *cache[i].get_or_insert_with(|| samples(e, curves[e.curve].def.as_curve()));
        let mut found = None;
        for &k in kept.iter() {
            let sk = *cache[k].get_or_insert_with(|| samples(&edges[k], curves[edges[k].curve].def.as_curve()));
            let close = |a: &[Point3; 5], b: &[Point3; 5], rev: bool| (0..5).all(|j| (a[j] - b[if rev { 4 - j } else { j }]).norm() <= tol);
            let closed = e.start == e.end;
            // Same direction when the start vertices agree (closed edges: whichever samples match).
            let same_dir = e.start == edges[k].start;
            if closed {
                if close(&si, &sk, false) {
                    found = Some((k, Sense::Forward));
                } else if close(&si, &sk, true) {
                    found = Some((k, Sense::Reversed));
                }
            } else if close(&si, &sk, !same_dir) {
                found = Some((k, if same_dir { Sense::Forward } else { Sense::Reversed }));
            }
            if found.is_some() {
                break;
            }
        }
        match found {
            Some(f) => out[i] = f,
            None => kept.push(i),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }

    #[test]
    fn the_lowest_index_wins_and_distant_points_stay() {
        let pts = [p(0.0, 0.0, 0.0), p(5.0, 0.0, 0.0), p(1e-7, 0.0, 0.0), p(5.0, 1e-7, 0.0), p(0.0, 0.0, 1e-5)];
        let r = weld_vertices(&pts, 1e-6);
        assert_eq!(r, [0, 1, 0, 1, 4]);
    }

    #[test]
    fn points_across_a_cell_boundary_weld() {
        // Straddle a grid line (multiples of the tolerance).
        let tol = 1e-3;
        let pts = [p(2.0 * tol - 1e-9, 0.0, 0.0), p(2.0 * tol + 1e-9, 0.0, 0.0)];
        assert_eq!(weld_vertices(&pts, tol), [0, 0]);
    }

    #[test]
    fn welding_matches_brute_force_on_a_jittered_lattice() {
        // Deterministic pseudo-random points on a lattice with duplicates inside the tolerance.
        let mut s = 12345u64;
        let mut rnd = move || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (s >> 33) as f64 / (1u64 << 31) as f64
        };
        let tol = 0.05;
        let pts: Vec<Point3> = (0..400).map(|_| p((rnd() * 8.0).floor() * 0.1 + rnd() * 0.04, (rnd() * 8.0).floor() * 0.1 + rnd() * 0.04, 0.0)).collect();
        let got = weld_vertices(&pts, tol);
        let mut want: Vec<usize> = (0..pts.len()).collect();
        let mut reps: Vec<usize> = Vec::new();
        for i in 0..pts.len() {
            match reps.iter().copied().find(|&j| (pts[j] - pts[i]).norm() <= tol) {
                Some(j) => want[i] = j,
                None => reps.push(i),
            }
        }
        assert_eq!(got, want);
    }

    #[test]
    fn nonpositive_tolerance_welds_nothing() {
        let pts = [p(0.0, 0.0, 0.0), p(0.0, 0.0, 0.0)];
        assert_eq!(weld_vertices(&pts, 0.0), [0, 1]);
    }
}
