//! Discrete transfinite (Coons) interpolation of one rectangular block in 3-D.
//!
//! Private twin of `sm-scheme-map`'s kernel: the blending fractions are the normalized
//! chord-length fractions of the boundary nodes, averaged over opposite lines,
//! `u = (ub[i] + ut[i]) / 2`, `v = (vl[j] + vr[j]) / 2`, and
//! `P = (1-v) B + v T + (1-u) L + u R - bilinear(corners)`.

/// Normalized chord-length fractions of the nodes along a polyline (`f[0] = 0`, `f[n-1] = 1`);
/// a polyline of zero length gets uniform fractions.
pub(crate) fn chord_fractions(line: &[[f64; 3]]) -> Vec<f64> {
    let n = line.len();
    let mut acc = vec![0.0; n];
    for q in 1..n {
        let s: f64 = (0..3).map(|d| (line[q][d] - line[q - 1][d]).powi(2)).sum();
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

/// Interior nodes of the Coons patch of four boundary lines, row-major over the interior
/// (`(j-1) * (ni-2) + (i-1)`). `bottom`/`top` have `ni` nodes, `left`/`right` have `nj`; the
/// corners are read from `bottom` and `top`. Returns `None` if the shapes disagree or the
/// block is smaller than 2 x 2 nodes.
pub(crate) fn coons(
    bottom: &[[f64; 3]],
    top: &[[f64; 3]],
    left: &[[f64; 3]],
    right: &[[f64; 3]],
) -> Option<Vec<[f64; 3]>> {
    let (ni, nj) = (bottom.len(), left.len());
    if ni < 2 || nj < 2 || top.len() != ni || right.len() != nj {
        return None;
    }
    let (wb, wt) = (chord_fractions(bottom), chord_fractions(top));
    let (wl, wr) = (chord_fractions(left), chord_fractions(right));
    let (p00, p10, p01, p11) = (bottom[0], bottom[ni - 1], top[0], top[ni - 1]);
    let mut out = Vec::with_capacity((ni - 2) * (nj - 2));
    for j in 1..nj - 1 {
        let v = 0.5 * (wl[j] + wr[j]);
        for i in 1..ni - 1 {
            let u = 0.5 * (wb[i] + wt[i]);
            let p: [f64; 3] = std::array::from_fn(|d| {
                let edges = (1.0 - v) * bottom[i][d] + v * top[i][d];
                let sides = (1.0 - u) * left[j][d] + u * right[j][d];
                let corners = (1.0 - u) * (1.0 - v) * p00[d]
                    + u * (1.0 - v) * p10[d]
                    + (1.0 - u) * v * p01[d]
                    + u * v * p11[d];
                edges + sides - corners
            });
            out.push(p);
        }
    }
    Some(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fractions_of_uniform_and_graded_lines() {
        let line: Vec<[f64; 3]> = (0..5).map(|i| [f64::from(i), 0.0, 0.0]).collect();
        assert_eq!(chord_fractions(&line), vec![0.0, 0.25, 0.5, 0.75, 1.0]);
        let graded = [[0.0; 3], [1.0, 0.0, 0.0], [3.0, 0.0, 0.0], [7.0, 0.0, 0.0]];
        assert_eq!(
            chord_fractions(&graded),
            vec![0.0, 1.0 / 7.0, 3.0 / 7.0, 1.0]
        );
        assert_eq!(chord_fractions(&[[1.0; 3]; 3]), vec![0.0, 0.5, 1.0]);
    }

    #[test]
    fn uniform_straight_sides_give_the_bilinear_map() {
        let (ni, nj) = (6usize, 4usize);
        let lerp = |a: [f64; 3], b: [f64; 3], t: f64| -> [f64; 3] {
            std::array::from_fn(|d| a[d] + t * (b[d] - a[d]))
        };
        let c = [[0.0; 3], [10.0, 0.0, 0.0], [2.0, 6.0, 0.0], [8.0, 6.0, 0.0]];
        let line = |a, b, n: usize| -> Vec<[f64; 3]> {
            (0..n)
                .map(|i| lerp(a, b, i as f64 / (n - 1) as f64))
                .collect()
        };
        let (bottom, top) = (line(c[0], c[1], ni), line(c[2], c[3], ni));
        let (left, right) = (line(c[0], c[2], nj), line(c[1], c[3], nj));
        let inner = coons(&bottom, &top, &left, &right).expect("coons");
        for j in 1..nj - 1 {
            for i in 1..ni - 1 {
                let (s, t) = (i as f64 / (ni - 1) as f64, j as f64 / (nj - 1) as f64);
                let want = lerp(lerp(c[0], c[1], s), lerp(c[2], c[3], s), t);
                let got = inner[(j - 1) * (ni - 2) + (i - 1)];
                assert!(
                    (0..3).all(|d| (got[d] - want[d]).abs() < 1e-13),
                    "{got:?} {want:?}"
                );
            }
        }
    }

    #[test]
    fn graded_opposite_sides_give_graded_columns() {
        let xs = [0.0, 0.1, 0.3, 0.6, 1.0];
        let bottom: Vec<[f64; 3]> = xs.iter().map(|x| [*x, 0.0, 0.0]).collect();
        let top: Vec<[f64; 3]> = xs.iter().map(|x| [*x, 1.0, 0.0]).collect();
        let left: Vec<[f64; 3]> = (0..4).map(|j| [0.0, f64::from(j) / 3.0, 0.0]).collect();
        let right: Vec<[f64; 3]> = (0..4).map(|j| [1.0, f64::from(j) / 3.0, 0.0]).collect();
        let inner = coons(&bottom, &top, &left, &right).expect("coons");
        for j in 1..3 {
            for i in 1..4 {
                let p = inner[(j - 1) * 3 + (i - 1)];
                assert!((p[0] - xs[i]).abs() < 1e-15 && (p[1] - j as f64 / 3.0).abs() < 1e-15);
            }
        }
    }

    #[test]
    fn bad_shapes_are_none() {
        let l = [[0.0; 3]; 3];
        let m = [[0.0; 3]; 2];
        assert!(coons(&l, &m, &m, &m).is_none());
        assert!(coons(&[[0.0; 3]], &[[0.0; 3]], &[[0.0; 3]], &[[0.0; 3]]).is_none());
    }
}
