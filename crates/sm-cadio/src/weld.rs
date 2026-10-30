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

/// Cells overlapping `[p - tol, p + tol]`, padded by a few ulps so that a neighbour at exactly
/// `|d| == tol` is found whatever the rounding of the subtraction and of the division.
fn span(p: f64, tol: f64) -> (i64, i64) {
    let pad = tol + 4.0 * f64::EPSILON * (p.abs() + tol);
    (cell(p - pad, tol), cell(p + pad, tol))
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
        let (sx, sy, sz) = (span(p.x, tol), span(p.y, tol), span(p.z, tol));
        let mut best: Option<usize> = None;
        for ix in sx.0..=sx.1 {
            for iy in sy.0..=sy.1 {
                for iz in sz.0..=sz.1 {
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
///
/// Kept edges are bucketed by their end vertices and the grid cell (size `tol`) of their middle
/// sample, which is the same point in either direction; a match must lie in one of the cells
/// within `tol` (3 per axis, a ulp-padded span), so many distinct edges between the same two vertices do not compare all pairs.
pub(crate) fn weld_edges(edges: &[Edge], curves: &[CurveRec], tol: f64) -> Vec<(usize, Sense)> {
    let mut out: Vec<(usize, Sense)> = (0..edges.len()).map(|i| (i, Sense::Forward)).collect();
    if tol.is_nan() || tol <= 0.0 {
        return out;
    }
    let smp: Vec<[Point3; 5]> = edges.iter().map(|e| samples(e, curves[e.curve].def.as_curve())).collect();
    let mut kept: BTreeMap<(usize, usize, i64, i64, i64), Vec<usize>> = BTreeMap::new();
    let close = |a: &[Point3; 5], b: &[Point3; 5], rev: bool| (0..5).all(|j| (a[j] - b[if rev { 4 - j } else { j }]).norm() <= tol);
    for (i, e) in edges.iter().enumerate() {
        let (a, b) = (e.start.min(e.end), e.start.max(e.end));
        let m = smp[i][2];
        let c = (cell(m.x, tol), cell(m.y, tol), cell(m.z, tol));
        let closed = e.start == e.end;
        // The lowest-index kept edge that matches, as a scan of all kept edges in index order.
        let mut found: Option<(usize, Sense)> = None;
        let (sx, sy, sz) = (span(m.x, tol), span(m.y, tol), span(m.z, tol));
        for ix in sx.0..=sx.1 {
            for iy in sy.0..=sy.1 {
                for iz in sz.0..=sz.1 {
                    let key = (a, b, ix, iy, iz);
                    let Some(list) = kept.get(&key) else { continue };
                    for &k in list {
                        if found.is_some_and(|f| f.0 < k) {
                            break;
                        }
                        // Same direction when the start vertices agree (closed edges: whichever samples match).
                        let same_dir = e.start == edges[k].start;
                        let hit = if closed {
                            if close(&smp[i], &smp[k], false) {
                                Some((k, Sense::Forward))
                            } else if close(&smp[i], &smp[k], true) {
                                Some((k, Sense::Reversed))
                            } else {
                                None
                            }
                        } else if close(&smp[i], &smp[k], !same_dir) {
                            Some((k, if same_dir { Sense::Forward } else { Sense::Reversed }))
                        } else {
                            None
                        };
                        if hit.is_some() {
                            found = hit;
                            break;
                        }
                    }
                }
            }
        }
        match found {
            Some(f) => out[i] = f,
            None => kept.entry((a, b, c.0, c.1, c.2)).or_default().push(i),
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
    fn points_exactly_one_tolerance_apart_weld_whatever_the_rounding() {
        // `b = a + tol` computed in floating point: a cell could be missed without the padded span.
        for tol in [1e-3, 0.1, 1e-6 / 3.0] {
            for k in 0..2000 {
                let x = k as f64 * tol * 0.99973 - 7.3;
                let pts = [p(x, 0.0, 0.0), p(x + tol, 0.0, 0.0)];
                let (a, b) = (pts[0], pts[1]);
                let want = if (b - a).norm() <= tol { [0, 0] } else { [0, 1] };
                assert_eq!(weld_vertices(&pts, tol), want, "x={x:e} tol={tol:e}");
            }
        }
    }

    #[test]
    fn nonpositive_tolerance_welds_nothing() {
        let pts = [p(0.0, 0.0, 0.0), p(0.0, 0.0, 0.0)];
        assert_eq!(weld_vertices(&pts, 0.0), [0, 1]);
    }

    /// Edges between vertices `a` and `b` (points `pa`, `pb`) bowed through `mid`; `rev` stores
    /// the curve from `b` to `a`.
    fn bowed(curves: &mut Vec<CurveRec>, a: (usize, Point3), b: (usize, Point3), mid: Point3, rev: bool) -> Edge {
        use sm_geom_api::{CurveDef, Interval, NurbsCurve};
        let ctrl = if rev { vec![b.1, mid, a.1] } else { vec![a.1, mid, b.1] };
        let c = NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], ctrl, None).expect("curve");
        curves.push(CurveRec { id: curves.len() as u64, def: CurveDef::Nurbs(c) });
        let (start, end) = if rev { (b.0, a.0) } else { (a.0, b.0) };
        Edge { id: curves.len() as u64, start, end, curve: curves.len() - 1, range: Interval::new(0.0, 1.0), sense: Sense::Forward, merged: Vec::new() }
    }

    /// The pre-bucketing algorithm: every kept edge of the vertex pair, in index order.
    fn weld_edges_scan(edges: &[Edge], curves: &[CurveRec], tol: f64) -> Vec<(usize, Sense)> {
        let mut out: Vec<(usize, Sense)> = (0..edges.len()).map(|i| (i, Sense::Forward)).collect();
        let smp: Vec<[Point3; 5]> = edges.iter().map(|e| samples(e, curves[e.curve].def.as_curve())).collect();
        let close = |a: &[Point3; 5], b: &[Point3; 5], rev: bool| (0..5).all(|j| (a[j] - b[if rev { 4 - j } else { j }]).norm() <= tol);
        let mut groups: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
        for (i, e) in edges.iter().enumerate() {
            let kept = groups.entry((e.start.min(e.end), e.start.max(e.end))).or_default();
            let mut found = None;
            for &k in kept.iter() {
                let same_dir = e.start == edges[k].start;
                if e.start == e.end {
                    if close(&smp[i], &smp[k], false) {
                        found = Some((k, Sense::Forward));
                    } else if close(&smp[i], &smp[k], true) {
                        found = Some((k, Sense::Reversed));
                    }
                } else if close(&smp[i], &smp[k], !same_dir) {
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

    #[test]
    fn edge_welding_matches_the_full_scan() {
        let mut s = 777u64;
        let mut rnd = move || {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            (s >> 33) as f64 / (1u64 << 31) as f64
        };
        let tol = 0.05;
        let verts = [(0, p(0.0, 0.0, 0.0)), (1, p(1.0, 0.0, 0.0)), (2, p(0.0, 1.0, 0.0))];
        let mut curves = Vec::new();
        let mut edges = Vec::new();
        for _ in 0..600 {
            let (ia, ib) = ((rnd() * 3.0) as usize % 3, (rnd() * 3.0) as usize % 3);
            // Bows on a coarse lattice plus jitter of the order of the tolerance (closed edges included).
            let mid = p((rnd() * 4.0).floor() * 0.1 + rnd() * 0.03, (rnd() * 4.0).floor() * 0.1 + rnd() * 0.03, rnd() * 0.06);
            edges.push(bowed(&mut curves, verts[ia], verts[ib], mid, rnd() < 0.5));
        }
        let got = weld_edges(&edges, &curves, tol);
        assert_eq!(got, weld_edges_scan(&edges, &curves, tol));
        assert!(got.iter().enumerate().filter(|(i, g)| g.0 != *i).count() > 100, "the test should merge many edges");
    }

    #[test]
    fn many_distinct_edges_on_one_vertex_pair_weld_in_near_linear_time() {
        let (a, b) = ((0, p(0.0, 0.0, 0.0)), (1, p(1.0, 0.0, 0.0)));
        let mut curves = Vec::new();
        let n = 20_000;
        let edges: Vec<Edge> = (0..n).map(|k| bowed(&mut curves, a, b, p(0.5, 0.01 * k as f64, 0.0), false)).collect();
        let t = std::time::Instant::now();
        let got = weld_edges(&edges, &curves, 1e-6);
        // A full scan compares 2e8 pairs (minutes in a debug build).
        assert!(t.elapsed().as_secs() < 10, "{:?}", t.elapsed());
        assert!(got.iter().enumerate().all(|(i, g)| g.0 == i));
    }
}
