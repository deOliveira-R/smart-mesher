//! Discrete transfinite (Coons) interpolation on a logical `ni x nj` grid, in any dimension.
//!
//! The four boundary lines are given as node coordinates (`bottom`, `top` along +i; `left`,
//! `right` along +j). Each interior node `(i, j)` blends them with the weights `(u, v)` taken
//! from the normalized chord-length fractions of the *nodes* of opposite boundary lines,
//! averaged:
//!
//! ```text
//! u(i,j) = (ub[i] + ut[i]) / 2        v(i,j) = (vl[j] + vr[j]) / 2
//! P = (1-v) B[i] + v T[i] + (1-u) L[j] + u R[j]
//!       - [(1-u)(1-v) P00 + u(1-v) P10 + (1-u) v P01 + u v P11]
//! ```
//!
//! At a boundary node this reproduces the boundary exactly. For uniformly spaced straight sides
//! the fractions are `i/(ni-1)` and `j/(nj-1)` and the result is the bilinear map of the
//! corners; for graded sides the interior follows the grading of the boundary. The weights are
//! kept apart from the coordinates ([`Weights`]) so the blend can run in parameter space while
//! the fractions come from 3-D distances.

use thiserror::Error;

/// Why an interpolation could not run.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
#[non_exhaustive]
pub enum TfiError {
    /// A boundary line has the wrong number of nodes, or the grid is smaller than 2 x 2.
    #[error(
        "boundary lines have {bottom}/{top}/{left}/{right} nodes (bottom/top/left/right); need ni >= 2, nj >= 2 and matching opposite sides"
    )]
    Shape {
        /// Nodes on the bottom line.
        bottom: usize,
        /// Nodes on the top line.
        top: usize,
        /// Nodes on the left line.
        left: usize,
        /// Nodes on the right line.
        right: usize,
    },
}

/// Normalized chord-length fractions of the nodes along a polyline: `f[0] = 0`, `f[n-1] = 1`.
/// A polyline of zero length (all nodes coincide) gets uniform fractions.
pub fn chord_fractions<const D: usize>(line: &[[f64; D]]) -> Vec<f64> {
    let n = line.len();
    let mut acc = vec![0.0; n];
    for q in 1..n {
        let s: f64 = line[q]
            .iter()
            .zip(&line[q - 1])
            .map(|(a, b)| (a - b) * (a - b))
            .sum();
        acc[q] = acc[q - 1] + s.sqrt();
    }
    let total = acc.last().copied().unwrap_or(0.0);
    if total > 0.0 && total.is_finite() {
        for a in &mut acc {
            *a /= total;
        }
    } else {
        let last = n.saturating_sub(1).max(1) as f64;
        for (q, a) in acc.iter_mut().enumerate() {
            *a = q as f64 / last;
        }
    }
    acc
}

/// Blending fractions of the four boundary lines.
#[derive(Clone, Debug, PartialEq)]
pub struct Weights {
    /// Fractions along the bottom line (`ni` values).
    pub bottom: Vec<f64>,
    /// Fractions along the top line (`ni` values).
    pub top: Vec<f64>,
    /// Fractions along the left line (`nj` values).
    pub left: Vec<f64>,
    /// Fractions along the right line (`nj` values).
    pub right: Vec<f64>,
}

impl Weights {
    /// Chord-length fractions of four boundary lines (in whatever space the caller measures).
    pub fn from_lines<const D: usize>(
        bottom: &[[f64; D]],
        top: &[[f64; D]],
        left: &[[f64; D]],
        right: &[[f64; D]],
    ) -> Self {
        Self {
            bottom: chord_fractions(bottom),
            top: chord_fractions(top),
            left: chord_fractions(left),
            right: chord_fractions(right),
        }
    }
}

/// The interior nodes of the Coons patch of four boundary lines, row-major over the interior
/// (`(j-1) * (ni-2) + (i-1)`), `(ni-2) * (nj-2)` of them.
///
/// `bottom[i]` and `top[i]` are the nodes at `j = 0` and `j = nj-1`; `left[j]` and `right[j]`
/// the nodes at `i = 0` and `i = ni-1`. The corners are read from `bottom` and `top`.
#[allow(clippy::needless_range_loop)]
pub fn coons<const D: usize>(
    bottom: &[[f64; D]],
    top: &[[f64; D]],
    left: &[[f64; D]],
    right: &[[f64; D]],
    w: &Weights,
) -> Result<Vec<[f64; D]>, TfiError> {
    let (ni, nj) = (bottom.len(), left.len());
    let shape = || TfiError::Shape {
        bottom: bottom.len(),
        top: top.len(),
        left: left.len(),
        right: right.len(),
    };
    if ni < 2 || nj < 2 || top.len() != ni || right.len() != nj {
        return Err(shape());
    }
    if w.bottom.len() != ni || w.top.len() != ni || w.left.len() != nj || w.right.len() != nj {
        return Err(shape());
    }
    let (p00, p10, p01, p11) = (bottom[0], bottom[ni - 1], top[0], top[ni - 1]);
    let mut out = Vec::with_capacity((ni - 2) * (nj - 2));
    for j in 1..nj - 1 {
        let v = 0.5 * (w.left[j] + w.right[j]);
        for i in 1..ni - 1 {
            let u = 0.5 * (w.bottom[i] + w.top[i]);
            let mut p = [0.0; D];
            for d in 0..D {
                let edges = (1.0 - v) * bottom[i][d] + v * top[i][d];
                let sides = (1.0 - u) * left[j][d] + u * right[j][d];
                let corners = (1.0 - u) * (1.0 - v) * p00[d]
                    + u * (1.0 - v) * p10[d]
                    + (1.0 - u) * v * p01[d]
                    + u * v * p11[d];
                p[d] = edges + sides - corners;
            }
            out.push(p);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fractions_of_uniform_and_graded_lines() {
        let line: Vec<[f64; 2]> = (0..5).map(|i| [f64::from(i), 0.0]).collect();
        assert_eq!(chord_fractions(&line), vec![0.0, 0.25, 0.5, 0.75, 1.0]);
        let graded = [[0.0, 0.0], [1.0, 0.0], [3.0, 0.0], [7.0, 0.0]];
        let f = chord_fractions(&graded);
        assert_eq!(f, vec![0.0, 1.0 / 7.0, 3.0 / 7.0, 1.0]);
        let point = [[1.0, 1.0]; 3];
        assert_eq!(chord_fractions(&point), vec![0.0, 0.5, 1.0]);
    }

    #[test]
    fn bilinear_corners_for_uniform_straight_sides() {
        // Trapezoid corners: (0,0) (10,0) (2,6) (8,6).
        let (ni, nj) = (6usize, 4usize);
        let lerp =
            |a: [f64; 2], b: [f64; 2], t: f64| [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
        let c = [[0.0, 0.0], [10.0, 0.0], [2.0, 6.0], [8.0, 6.0]];
        let bottom: Vec<_> = (0..ni)
            .map(|i| lerp(c[0], c[1], i as f64 / (ni - 1) as f64))
            .collect();
        let top: Vec<_> = (0..ni)
            .map(|i| lerp(c[2], c[3], i as f64 / (ni - 1) as f64))
            .collect();
        let left: Vec<_> = (0..nj)
            .map(|j| lerp(c[0], c[2], j as f64 / (nj - 1) as f64))
            .collect();
        let right: Vec<_> = (0..nj)
            .map(|j| lerp(c[1], c[3], j as f64 / (nj - 1) as f64))
            .collect();
        let w = Weights::from_lines(&bottom, &top, &left, &right);
        let inner = coons(&bottom, &top, &left, &right, &w).expect("coons");
        assert_eq!(inner.len(), (ni - 2) * (nj - 2));
        for j in 1..nj - 1 {
            for i in 1..ni - 1 {
                let (s, t) = (i as f64 / (ni - 1) as f64, j as f64 / (nj - 1) as f64);
                let want = lerp(lerp(c[0], c[1], s), lerp(c[2], c[3], s), t);
                let got = inner[(j - 1) * (ni - 2) + (i - 1)];
                assert!((got[0] - want[0]).abs() < 1e-13 && (got[1] - want[1]).abs() < 1e-13);
            }
        }
    }

    #[test]
    fn graded_opposite_sides_give_graded_columns() {
        // Unit square, bottom and top graded identically, sides uniform: every interior column
        // sits at the graded x of its boundary nodes and rows stay uniform in y.
        let xs = [0.0, 0.1, 0.3, 0.6, 1.0];
        let bottom: Vec<[f64; 2]> = xs.iter().map(|x| [*x, 0.0]).collect();
        let top: Vec<[f64; 2]> = xs.iter().map(|x| [*x, 1.0]).collect();
        let left: Vec<[f64; 2]> = (0..4).map(|j| [0.0, f64::from(j) / 3.0]).collect();
        let right: Vec<[f64; 2]> = (0..4).map(|j| [1.0, f64::from(j) / 3.0]).collect();
        let w = Weights::from_lines(&bottom, &top, &left, &right);
        let inner = coons(&bottom, &top, &left, &right, &w).expect("coons");
        for j in 1..3 {
            for i in 1..4 {
                let p = inner[(j - 1) * 3 + (i - 1)];
                assert!(
                    (p[0] - xs[i]).abs() < 1e-15 && (p[1] - j as f64 / 3.0).abs() < 1e-15,
                    "{p:?}"
                );
            }
        }
    }

    #[test]
    fn shape_errors_are_typed() {
        let l: [[f64; 2]; 3] = [[0.0; 2]; 3];
        let m: [[f64; 2]; 2] = [[0.0; 2]; 2];
        let w = Weights::from_lines(&l, &l, &m, &m);
        assert!(coons(&l, &m, &m, &m, &w).is_err());
        let one: [[f64; 2]; 1] = [[0.0; 2]];
        let w1 = Weights::from_lines(&one, &one, &one, &one);
        assert!(coons(&one, &one, &one, &one, &w1).is_err());
    }
}
