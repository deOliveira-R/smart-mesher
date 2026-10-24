//! Private numeric helpers: bracketed Brent and a small dense solver. Polynomial and quadratic
//! roots live in [`sm_geom_api::roots`].

/// Horner evaluation of ascending coefficients.
#[cfg(test)]
pub(crate) fn horner(c: &[f64], t: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, v| acc * t + v)
}

/// Brent's method on a sign-changing bracket `[a, b]`. `fa` and `fb` must have opposite signs
/// (or one must be zero). Converges to a few ulp of the root.
pub(crate) fn brent(f: &mut impl FnMut(f64) -> f64, mut a: f64, mut b: f64, mut fa: f64, mut fb: f64) -> f64 {
    if fa == 0.0 {
        return a;
    }
    if fb == 0.0 {
        return b;
    }
    let (mut c, mut fc) = (a, fa);
    let (mut d, mut e) = (b - a, b - a);
    for _ in 0..200 {
        if fb * fc > 0.0 {
            c = a;
            fc = fa;
            d = b - a;
            e = d;
        }
        if fc.abs() < fb.abs() {
            a = b;
            b = c;
            c = a;
            fa = fb;
            fb = fc;
            fc = fa;
        }
        let tol = 2.0 * f64::EPSILON * b.abs() + 1e-300;
        let m = 0.5 * (c - b);
        if m.abs() <= tol || fb == 0.0 {
            return b;
        }
        if e.abs() >= tol && fa.abs() > fb.abs() {
            let s = fb / fa;
            let (mut p, mut q);
            if a == c {
                p = 2.0 * m * s;
                q = 1.0 - s;
            } else {
                let (qq, r) = (fa / fc, fb / fc);
                p = s * (2.0 * m * qq * (qq - r) - (b - a) * (r - 1.0));
                q = (qq - 1.0) * (r - 1.0) * (s - 1.0);
            }
            if p > 0.0 {
                q = -q;
            } else {
                p = -p;
            }
            if 2.0 * p < (3.0 * m * q - (tol * q).abs()).min((e * q).abs()) {
                e = d;
                d = p / q;
            } else {
                d = m;
                e = m;
            }
        } else {
            d = m;
            e = m;
        }
        a = b;
        fa = fb;
        b += if d.abs() > tol { d } else { tol.copysign(m) };
        fb = f(b);
    }
    b
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn brent_converges_on_hard_function() {
        let mut f = |x: f64| x.powi(3) - 2.0 * x - 5.0;
        let (fa, fb) = (f(2.0), f(3.0));
        let r = brent(&mut f, 2.0, 3.0, fa, fb);
        assert!((r - 2.094_551_481_542_326_6).abs() < 1e-14);
        assert!(horner(&[-5.0, -2.0, 0.0, 1.0], r).abs() < 1e-13);
    }
}

/// Solve the dense `N x N` system `a x = b` by Gaussian elimination with partial pivoting.
/// `None` when the matrix is singular to working precision.
pub(crate) fn solve_dense<const N: usize>(mut a: [[f64; N]; N], mut b: [f64; N]) -> Option<[f64; N]> {
    let scale = a.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs()));
    if scale == 0.0 {
        return None;
    }
    for col in 0..N {
        let piv = (col..N).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[piv][col].abs() <= 1e-14 * scale {
            return None;
        }
        a.swap(col, piv);
        b.swap(col, piv);
        for r in col + 1..N {
            let f = a[r][col] / a[col][col];
            for c in col..N {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = [0.0; N];
    for r in (0..N).rev() {
        let mut s = b[r];
        for c in r + 1..N {
            s -= a[r][c] * x[c];
        }
        x[r] = s / a[r][r];
    }
    Some(x)
}

#[cfg(test)]
mod dense_tests {
    use super::solve_dense;

    #[test]
    fn solves_and_detects_singular() {
        let x = solve_dense([[2.0, 1.0, 0.0], [1.0, 3.0, 1.0], [0.0, 1.0, 4.0]], [3.0, 5.0, 5.0]).unwrap();
        assert!((x[0] - 1.0).abs() < 1e-14 && (x[1] - 1.0).abs() < 1e-14 && (x[2] - 1.0).abs() < 1e-14);
        assert!(solve_dense([[1.0, 2.0], [2.0, 4.0]], [1.0, 2.0]).is_none());
    }
}
