//! Private real-root machinery: stable quadratic, bracketed Brent, and polynomial roots on an
//! interval with a double-root flag (the `Tangent` semantics of the contract's ray hook).
//!
//! `sm-math` was empty when W1 started; this module should migrate there.

/// A real root and whether it is a double (tangential) root.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Root {
    pub t: f64,
    pub double: bool,
}

/// Horner evaluation of ascending coefficients.
pub(crate) fn horner(c: &[f64], t: f64) -> f64 {
    c.iter().rev().fold(0.0, |acc, v| acc * t + v)
}

/// Horner evaluation of the absolute-value polynomial (a rounding-error scale).
fn horner_abs(c: &[f64], t: f64) -> f64 {
    let t = t.abs();
    c.iter().rev().fold(0.0, |acc, v| acc * t + v.abs())
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

/// Real roots of `a t^2 + b t + c` (stable form). Linear when `a` is negligible. A double root
/// counts once and is flagged.
pub(crate) fn quadratic(a: f64, b: f64, c: f64) -> smallvec::SmallVec<[Root; 2]> {
    let mut out = smallvec::SmallVec::new();
    let scale = a.abs().max(b.abs()).max(c.abs());
    if scale == 0.0 {
        return out;
    }
    if a.abs() <= 1e-14 * scale {
        if b.abs() > 1e-14 * scale {
            out.push(Root { t: -c / b, double: false });
        }
        return out;
    }
    let disc = b * b - 4.0 * a * c;
    let dscale = b * b + (4.0 * a * c).abs();
    if disc < -1e-13 * dscale {
        return out;
    }
    if disc <= 1e-13 * dscale {
        out.push(Root { t: -b / (2.0 * a), double: true });
        return out;
    }
    let sq = disc.sqrt();
    let q = -0.5 * (b + b.signum() * sq);
    let (r1, r2) = (q / a, if q != 0.0 { c / q } else { q / a });
    out.push(Root { t: r1.min(r2), double: false });
    out.push(Root { t: r1.max(r2), double: false });
    out
}

/// Real roots of the polynomial with ascending coefficients `c` inside `[lo, hi]` (either may be
/// infinite), sorted. Monotone pieces are isolated with the roots of the derivative and refined
/// by Brent; a critical point whose value is within rounding of zero is reported as a double
/// root (and replaces any simple root next to it).
pub(crate) fn poly_roots(c: &[f64], lo: f64, hi: f64) -> Vec<Root> {
    let maxabs = c.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    if maxabs == 0.0 || !maxabs.is_finite() {
        return Vec::new();
    }
    let mut n = c.len();
    while n > 1 && c[n - 1].abs() <= 1e-14 * maxabs {
        n -= 1;
    }
    let c = &c[..n];
    match n {
        0 | 1 => return Vec::new(),
        2 => {
            let r = -c[0] / c[1];
            return if r >= lo && r <= hi { vec![Root { t: r, double: false }] } else { Vec::new() };
        }
        3 => return quadratic(c[2], c[1], c[0]).into_iter().filter(|r| r.t >= lo && r.t <= hi).collect(),
        _ => {}
    }
    // Cauchy bound on all roots.
    let lead = c[n - 1].abs();
    let bound = 1.0 + c[..n - 1].iter().fold(0.0f64, |m, v| m.max(v.abs())) / lead;
    let (lo, hi) = (lo.max(-bound), hi.min(bound));
    if lo > hi {
        return Vec::new();
    }
    let deriv: Vec<f64> = c.iter().enumerate().skip(1).map(|(i, v)| v * i as f64).collect();
    let crit: Vec<f64> = poly_roots(&deriv, lo, hi).into_iter().map(|r| r.t).collect();
    let mut pts = Vec::with_capacity(crit.len() + 2);
    pts.push(lo);
    pts.extend(crit.iter().copied());
    pts.push(hi);
    let mut roots: Vec<Root> = Vec::new();
    let mut f = |t: f64| horner(c, t);
    for w in pts.windows(2) {
        let (a, b) = (w[0], w[1]);
        let (fa, fb) = (f(a), f(b));
        if fa == 0.0 {
            roots.push(Root { t: a, double: false });
        } else if fa * fb < 0.0 {
            roots.push(Root { t: brent(&mut f, a, b, fa, fb), double: false });
        }
        if fb == 0.0 && w[1] == hi {
            roots.push(Root { t: b, double: false });
        }
    }
    // Interior critical points that are roots to rounding: tangential contacts.
    for &t in &crit {
        if f(t).abs() <= 1e-11 * horner_abs(c, t) {
            // Drop simple roots that are the same contact seen from either side.
            let near = 1e-5 * (1.0 + t.abs());
            roots.retain(|r| (r.t - t).abs() > near);
            roots.push(Root { t, double: true });
        }
    }
    roots.sort_by(|a, b| a.t.total_cmp(&b.t));
    roots.dedup_by(|a, b| a.t == b.t);
    roots
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quadratic_cases() {
        let r = quadratic(1.0, -3.0, 2.0);
        assert_eq!(r.len(), 2);
        assert!((r[0].t - 1.0).abs() < 1e-14 && (r[1].t - 2.0).abs() < 1e-14);
        let r = quadratic(1.0, -2.0, 1.0);
        assert_eq!(r.len(), 1);
        assert!(r[0].double && (r[0].t - 1.0).abs() < 1e-12);
        assert!(quadratic(1.0, 0.0, 1.0).is_empty());
        let r = quadratic(0.0, 2.0, -4.0);
        assert!((r[0].t - 2.0).abs() < 1e-14);
        // Cancellation-prone case: roots 1e-8 and 1e8.
        let r = quadratic(1.0, -(1e8 + 1e-8), 1.0);
        assert!((r[0].t - 1e-8).abs() < 1e-20 && (r[1].t / 1e8 - 1.0).abs() < 1e-15);
    }

    #[test]
    fn simple_and_multiple_polynomial_roots() {
        // (t-1)(t-2)(t-3)(t+4) = t^4 - 2 t^3 - 13 t^2 + 38 t - 24
        let r = poly_roots(&[-24.0, 38.0, -13.0, -2.0, 1.0], f64::NEG_INFINITY, f64::INFINITY);
        let ts: Vec<f64> = r.iter().map(|r| r.t).collect();
        assert_eq!(ts.len(), 4);
        for (a, b) in ts.iter().zip([-4.0, 1.0, 2.0, 3.0]) {
            assert!((a - b).abs() < 1e-12, "{ts:?}");
        }
        assert!(r.iter().all(|r| !r.double));
        // (t-1)^2 (t-2)(t+3): double root at 1.
        let r = poly_roots(&[-6.0, 13.0, -7.0, -1.0, 1.0], -10.0, 10.0);
        assert!(r.iter().any(|r| r.double && (r.t - 1.0).abs() < 1e-5), "{r:?}");
        // Interval restriction.
        let r = poly_roots(&[-24.0, 38.0, -13.0, -2.0, 1.0], 1.5, 2.5);
        assert_eq!(r.len(), 1);
        assert!((r[0].t - 2.0).abs() < 1e-12);
    }

    #[test]
    fn brent_converges_on_hard_function() {
        let mut f = |x: f64| x.powi(3) - 2.0 * x - 5.0;
        let (fa, fb) = (f(2.0), f(3.0));
        let r = brent(&mut f, 2.0, 3.0, fa, fb);
        assert!((r - 2.094_551_481_542_326_6).abs() < 1e-14);
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
