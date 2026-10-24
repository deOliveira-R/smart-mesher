//! Real-root finders for low-degree polynomials: the stable quadratic and an interval
//! polynomial solver that flags double (tangential) roots. They back the ray hooks of the
//! implicit surfaces and are public so implementors share one definition.

/// A real root and whether it is a double (tangential) root.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Root {
    /// The root.
    pub t: f64,
    /// True for a double (tangential) root, reported once.
    pub double: bool,
}


/// Real roots of `a t^2 + b t + c` (stable form). Returns the count written to `out`
/// (a double root counts once and is flagged). Degenerates to the linear case when `a` is
/// negligible.
pub fn solve_quadratic(a: f64, b: f64, c: f64, out: &mut [Root; 2]) -> usize {
    let scale = a.abs().max(b.abs()).max(c.abs());
    if scale == 0.0 {
        return 0;
    }
    if a.abs() <= 1e-14 * scale {
        if b.abs() <= 1e-14 * scale {
            return 0;
        }
        out[0] = Root { t: -c / b, double: false };
        return 1;
    }
    let disc = b * b - 4.0 * a * c;
    let dscale = b * b + (4.0 * a * c).abs();
    if disc < -1e-13 * dscale {
        return 0;
    }
    if disc <= 1e-13 * dscale {
        out[0] = Root { t: -b / (2.0 * a), double: true };
        return 1;
    }
    let sq = disc.sqrt();
    let q = -0.5 * (b + b.signum() * sq);
    let (r1, r2) = (q / a, if q != 0.0 { c / q } else { q / a });
    out[0] = Root { t: r1.min(r2), double: false };
    out[1] = Root { t: r1.max(r2), double: false };
    2
}

/// Real roots of the polynomial with ascending coefficients `c` inside `[lo, hi]` (either may
/// be infinite). Isolates monotone pieces with the roots of the derivative, then bisects;
/// double roots are detected from near-zero critical values and flagged. Sorted, deduplicated
/// (a simple root that coincides with a flagged critical root becomes that double root).
pub fn real_roots(c: &[f64], lo: f64, hi: f64) -> Vec<Root> {
    let maxabs = c.iter().fold(0.0f64, |m, v| m.max(v.abs()));
    let mut n = c.len();
    while n > 1 && c[n - 1].abs() <= 1e-14 * maxabs {
        n -= 1;
    }
    let c = &c[..n];
    if n <= 1 {
        return Vec::new();
    }
    if n == 2 {
        let r = -c[0] / c[1];
        return if r >= lo && r <= hi { vec![Root { t: r, double: false }] } else { Vec::new() };
    }
    let lead = c[n - 1].abs();
    let bound = 1.0 + c[..n - 1].iter().fold(0.0f64, |m, v| m.max(v.abs())) / lead;
    let (lo, hi) = (lo.max(-bound), hi.min(bound));
    if lo > hi {
        return Vec::new();
    }
    let deriv: Vec<f64> = c.iter().enumerate().skip(1).map(|(i, v)| v * i as f64).collect();
    let crit: Vec<f64> = real_roots(&deriv, lo, hi).into_iter().map(|r| r.t).collect();
    let eval = |t: f64| c.iter().rev().fold(0.0, |acc, v| acc * t + v);
    let abs_eval = |t: f64| c.iter().rev().fold(0.0, |acc, v| acc * t.abs() + v.abs());
    let mut pts = Vec::with_capacity(crit.len() + 2);
    pts.push(lo);
    pts.extend(crit.iter().copied());
    pts.push(hi);
    let mut roots: Vec<Root> = Vec::new();
    for w in pts.windows(2) {
        let (mut a, mut b) = (w[0], w[1]);
        let (mut fa, fb) = (eval(a), eval(b));
        if fa == 0.0 {
            roots.push(Root { t: a, double: false });
        } else if fa * fb < 0.0 {
            for _ in 0..200 {
                let m = 0.5 * (a + b);
                if m <= a || m >= b {
                    break;
                }
                let fm = eval(m);
                if fm == 0.0 {
                    a = m;
                    b = m;
                    break;
                }
                if fa * fm < 0.0 {
                    b = m;
                } else {
                    a = m;
                    fa = fm;
                }
            }
            roots.push(Root { t: 0.5 * (a + b), double: false });
        }
        if fb == 0.0 {
            roots.push(Root { t: b, double: false });
        }
    }
    for &t in &crit {
        if eval(t).abs() <= 1e-12 * abs_eval(t) {
            // Rounding splits a multiple root into spurious simple neighbours: drop them.
            let near = 1e-5 * (1.0 + t.abs());
            roots.retain(|r| (r.t - t).abs() > near);
            roots.push(Root { t, double: true });
        }
    }
    roots.sort_by(|a, b| a.t.total_cmp(&b.t));
    let mut merged: Vec<Root> = Vec::with_capacity(roots.len());
    for r in roots {
        match merged.last_mut() {
            Some(l) if (r.t - l.t).abs() <= 1e-9 * (1.0 + l.t.abs()) => {
                if r.double && !l.double {
                    l.t = r.t;
                }
                l.double |= r.double;
            }
            _ => merged.push(r),
        }
    }
    merged
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quadratic_cases() {
        let mut o = [Root::default(); 2];
        assert_eq!(solve_quadratic(1.0, -3.0, 2.0, &mut o), 2);
        assert!((o[0].t - 1.0).abs() < 1e-14 && (o[1].t - 2.0).abs() < 1e-14);
        assert_eq!(solve_quadratic(1.0, -2.0, 1.0, &mut o), 1);
        assert!(o[0].double && (o[0].t - 1.0).abs() < 1e-12);
        assert_eq!(solve_quadratic(1.0, 0.0, 1.0, &mut o), 0);
        assert_eq!(solve_quadratic(0.0, 2.0, -4.0, &mut o), 1);
        assert!((o[0].t - 2.0).abs() < 1e-14);
        // Cancellation-prone case: roots 1e-8 and 1e8.
        assert_eq!(solve_quadratic(1.0, -(1e8 + 1e-8), 1.0, &mut o), 2);
        assert!((o[0].t - 1e-8).abs() < 1e-20 && (o[1].t / 1e8 - 1.0).abs() < 1e-15);
        assert_eq!(solve_quadratic(0.0, 0.0, 0.0, &mut o), 0);
    }

    #[test]
    fn simple_and_multiple_polynomial_roots() {
        // (t-1)(t-2)(t-3)(t+4)
        let c = [-24.0, 38.0, -13.0, -2.0, 1.0];
        let r = real_roots(&c, f64::NEG_INFINITY, f64::INFINITY);
        assert_eq!(r.len(), 4);
        for (a, b) in r.iter().zip([-4.0, 1.0, 2.0, 3.0]) {
            assert!((a.t - b).abs() < 1e-12 && !a.double, "{r:?}");
        }
        // (t-1)^2 (t-2)(t+3): double root at 1.
        let r = real_roots(&[-6.0, 13.0, -7.0, -1.0, 1.0], -10.0, 10.0);
        assert!(r.iter().any(|r| r.double && (r.t - 1.0).abs() < 1e-5), "{r:?}");
        // Regression: rounding must not leave spurious simple roots next to the double root.
        for c in [[0.686_647_121_972_279, 2.002_959_241_948_647_5, 1.460_665_018_657_671_3].as_slice(), &[-6.0, 13.0, -7.0, -1.0, 1.0]] {
            let r = real_roots(c, -10.0, 10.0);
            for (i, a) in r.iter().enumerate() {
                assert!(r.iter().skip(i + 1).all(|b| (b.t - a.t).abs() > 1e-5), "{r:?}");
            }
        }
        assert_eq!(real_roots(&[0.686_647_121_972_279, 2.002_959_241_948_647_5, 1.460_665_018_657_671_3], -10.0, 10.0).len(), 1);
        // Interval restriction.
        let r = real_roots(&c, 1.5, 2.5);
        assert_eq!(r.len(), 1);
        assert!((r[0].t - 2.0).abs() < 1e-12);
        // Degenerate input.
        assert!(real_roots(&[0.0, 0.0], -1.0, 1.0).is_empty());
        assert!(real_roots(&[], -1.0, 1.0).is_empty());
    }
}
