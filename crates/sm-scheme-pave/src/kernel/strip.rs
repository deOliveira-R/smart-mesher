//! Structured strip fill: a single loop whose boundary nodes split into four logical sides
//! with equal opposite counts is meshed as a transfinite-interpolation grid.
//!
//! This covers the thin regions (cladding gaps, narrow rectangles, slivers with two or more
//! intervals across) for which the triangle route cannot find a perfect matching: the
//! Delaunay triangles of such a region do not pair up, while the grid trivially exists. The
//! four corners are chosen among all boundary nodes by minimising the deviation of the
//! turning angle from a right angle; the opposite-count condition fixes the rest, since
//! `a + b = nb / 2` for the two side lengths.

use super::metric::Met;
use super::smooth::{quad_quality, quad_valid};
use core::f64::consts::FRAC_PI_2;

/// Largest boundary the corner search handles (it is quadratic in the node count).
const MAX_NODES: usize = 8192;

/// Worst corner sine below which a grid counts as degenerate (a flat corner).
const MIN_QUALITY: f64 = 1e-3;

/// A strip grid: the interior points (frame coordinates, row-major in `(t, s)`) and the quads
/// (indices into boundary-then-interior), with their worst corner sine.
pub(crate) struct Strip {
    /// Interior points.
    pub interior: Vec<[f64; 2]>,
    /// Quads, counter-clockwise.
    pub quads: Vec<[u32; 4]>,
    /// Smallest corner sine over the quads in the Euclidean frame (before smoothing).
    pub quality: f64,
}

/// Turning angle at every node of the counter-clockwise loop `w`.
fn turns(w: &[[f64; 2]]) -> Vec<f64> {
    let n = w.len();
    (0..n)
        .map(|i| {
            let (a, b, c) = (w[(i + n - 1) % n], w[i], w[(i + 1) % n]);
            let (e1, e2) = ([b[0] - a[0], b[1] - a[1]], [c[0] - b[0], c[1] - b[1]]);
            (e1[0] * e2[1] - e1[1] * e2[0]).atan2(e1[0] * e2[0] + e1[1] * e2[1])
        })
        .collect()
}

/// The corner pair `(i, a)` of the four corners `i, i+a, i+n/2, i+n/2+a` with the smallest
/// total deviation from right angles, ties by the smaller `(i, a)`.
fn best_corners(w: &[[f64; 2]]) -> Option<(usize, usize)> {
    let n = w.len();
    let half = n / 2;
    let f: Vec<f64> = turns(w).iter().map(|t| (t - FRAC_PI_2) * (t - FRAC_PI_2)).collect();
    let mut best: Option<(f64, usize, usize)> = None;
    for i in 0..half {
        for a in 1..half {
            let c = f[i] + f[(i + a) % n] + f[i + half] + f[(i + half + a) % n];
            if best.is_none_or(|b| c < b.0) {
                best = Some((c, i, a));
            }
        }
    }
    best.map(|b| (b.1, b.2))
}

/// Mesh the counter-clockwise loop `w` (frame coordinates, even length) as one grid, or
/// `None` when there is no valid one (too few nodes, a degenerate or folded grid).
pub(crate) fn strip_fill(w: &[[f64; 2]]) -> Option<Strip> {
    let n = w.len();
    if n < 4 || n % 2 == 1 || n > MAX_NODES {
        return None;
    }
    let half = n / 2;
    let (i0, a) = best_corners(w)?;
    let b = half - a;
    // Grid index of (s, t), 0 <= s <= a, 0 <= t <= b, into the loop nodes or the interior.
    let node = |s: usize, t: usize| -> Option<usize> {
        if t == 0 {
            Some((i0 + s) % n)
        } else if s == a {
            Some((i0 + a + t) % n)
        } else if t == b {
            Some((i0 + half + a - s) % n)
        } else if s == 0 {
            Some((i0 + n - t) % n)
        } else {
            None
        }
    };
    let interior_id = |s: usize, t: usize| n + (t - 1) * (a - 1) + (s - 1);
    let id = |s: usize, t: usize| node(s, t).unwrap_or_else(|| interior_id(s, t)) as u32;
    // Transfinite interpolation on index fractions.
    let mut pts: Vec<[f64; 2]> = w.to_vec();
    for t in 1..b {
        for s in 1..a {
            let (al, be) = (s as f64 / a as f64, t as f64 / b as f64);
            let at = |s: usize, t: usize| w[node(s, t).unwrap_or(0)];
            let (l, r, bo, to) = (at(0, t), at(a, t), at(s, 0), at(s, b));
            let (c00, c10, c01, c11) = (at(0, 0), at(a, 0), at(0, b), at(a, b));
            let p: [f64; 2] = core::array::from_fn(|k| {
                (1.0 - al) * l[k] + al * r[k] + (1.0 - be) * bo[k] + be * to[k]
                    - ((1.0 - al) * (1.0 - be) * c00[k] + al * (1.0 - be) * c10[k] + (1.0 - al) * be * c01[k] + al * be * c11[k])
            });
            if !(p[0].is_finite() && p[1].is_finite()) {
                return None;
            }
            pts.push(p);
        }
    }
    let mut quads = Vec::with_capacity(a * b);
    for t in 0..b {
        for s in 0..a {
            quads.push([id(s, t), id(s + 1, t), id(s + 1, t + 1), id(s, t + 1)]);
        }
    }
    if !quads.iter().all(|q| quad_valid(&pts, q)) {
        return None;
    }
    let ident = Met::from_sym([1.0, 0.0, 1.0])?;
    let met = vec![ident; pts.len()];
    let quality = quads.iter().map(|q| quad_quality(&pts, &met, q)).fold(f64::INFINITY, f64::min);
    // A grid with a (numerically) flat corner is a degenerate quad, not a mesh.
    if quality.is_nan() || quality < MIN_QUALITY {
        return None;
    }
    Some(Strip { interior: pts.split_off(n), quads, quality })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rectangle_is_a_regular_grid() {
        // 6 x 2 intervals, counter-clockwise.
        let mut w = Vec::new();
        for i in 0..6 {
            w.push([i as f64, 0.0]);
        }
        for j in 0..2 {
            w.push([6.0, j as f64]);
        }
        for i in 0..6 {
            w.push([6.0 - i as f64, 2.0]);
        }
        for j in 0..2 {
            w.push([0.0, 2.0 - j as f64]);
        }
        let s = strip_fill(&w).expect("grid");
        assert_eq!(s.quads.len(), 12);
        assert_eq!(s.interior.len(), 5);
        assert!((s.quality - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_two_by_one_grid_and_too_few_nodes() {
        let w = vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [2.0, 1.0], [1.0, 1.0], [0.0, 1.0]];
        let s = strip_fill(&w).expect("2 x 1 grid");
        assert_eq!(s.quads.len(), 2);
        assert!(strip_fill(&w[..3]).is_none());
    }
}
