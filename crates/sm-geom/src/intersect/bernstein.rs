//! Real roots of a polynomial given by Bernstein coefficients on `[0, 1]`, by variation-
//! diminishing subdivision: no sign change means no root, one means exactly one (refined by
//! Brent), more means bisect. Roots that never separate (tangential contacts) are reported once,
//! flagged, when the coefficients collapse to within `eps` of zero.

use crate::roots::brent;
use smallvec::SmallVec;

/// A root in `[0, 1]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct BRoot {
    pub s: f64,
    pub tangent: bool,
}

type Coef = SmallVec<[f64; 16]>;

/// de Casteljau value at `s`.
pub(crate) fn eval(c: &[f64], s: f64) -> f64 {
    let n = c.len();
    let mut w: Coef = c.iter().copied().collect();
    for level in 1..n {
        for i in 0..n - level {
            w[i] = (1.0 - s) * w[i] + s * w[i + 1];
        }
    }
    w[0]
}

fn split(c: &[f64]) -> (Coef, Coef) {
    let n = c.len();
    let mut w: Coef = c.iter().copied().collect();
    let mut l = Coef::new();
    let mut r: Coef = SmallVec::from_elem(0.0, n);
    l.push(w[0]);
    r[n - 1] = w[n - 1];
    for level in 1..n {
        for i in 0..n - level {
            w[i] = 0.5 * (w[i] + w[i + 1]);
        }
        l.push(w[0]);
        r[n - 1 - level] = w[n - 1 - level];
    }
    (l, r)
}

fn sign_changes(c: &[f64]) -> usize {
    let mut n = 0;
    let mut prev = 0.0f64;
    for &v in c {
        if v != 0.0 {
            if prev != 0.0 && (v > 0.0) != (prev > 0.0) {
                n += 1;
            }
            prev = v;
        }
    }
    n
}

/// Roots of the Bernstein polynomial `c` on `[0, 1]`, ascending. `eps` is the coefficient
/// magnitude under which a polynomial that never changes sign counts as touching zero.
pub(crate) fn roots(c: &[f64], eps: f64) -> Vec<BRoot> {
    let mut out: Vec<BRoot> = Vec::new();
    // Stretches where the polynomial stays within `eps` of zero without crossing it.
    let mut flat: Vec<(f64, f64)> = Vec::new();
    // Monotone pieces that end within `eps` of zero without crossing: (parameter, sign). Two
    // pieces of the same sign meeting at a parameter form a grazing contact there.
    let mut weak: Vec<(f64, bool)> = Vec::new();
    // Zero coefficients at a piece end without a sign change inside the piece: (parameter,
    // piece lies to the right of the parameter, sign of the piece). A matching pair of the same
    // sign on both sides of a split point is an even-multiplicity root.
    let mut ends: Vec<(f64, bool, Option<bool>)> = Vec::new();
    let mut stack: Vec<(Coef, f64, f64, u32)> = vec![(c.iter().copied().collect(), 0.0, 1.0, 0)];
    while let Some((co, s0, s1, depth)) = stack.pop() {
        let k = sign_changes(&co);
        let (first, last) = (co[0], co[co.len() - 1]);
        let width = s1 - s0;
        let subdivide = |co: &Coef, stack: &mut Vec<(Coef, f64, f64, u32)>| {
            let (l, r) = split(co);
            let m = 0.5 * (s0 + s1);
            stack.push((r, m, s1, depth + 1));
            stack.push((l, s0, m, depth + 1));
        };
        match k {
            0 => {
                if first == 0.0 || last == 0.0 {
                    let sign = co.iter().find(|v| **v != 0.0).map(|v| *v > 0.0);
                    if first == 0.0 {
                        ends.push((s0, true, sign));
                    }
                    if last == 0.0 {
                        ends.push((s1, false, sign));
                    }
                    continue;
                }
                let mn = co.iter().fold(f64::INFINITY, |m, v| m.min(v.abs()));
                if mn > 4.0 * eps {
                    continue;
                }
                // Near zero without a crossing: only an interior extremum can touch zero. The
                // derivative's control polygon says whether there is one.
                let dco: Coef = co.windows(2).map(|w| w[1] - w[0]).collect();
                if sign_changes(&dco) == 0 {
                    // Monotone: a crossing belongs to the neighbouring piece. A minimum of |c|
                    // at an end within `eps` is a candidate contact, kept if the neighbour
                    // approaches it from the same side (decided after the loop).
                    let (at, v) = if first.abs() <= last.abs() { (s0, first) } else { (s1, last) };
                    if v.abs() <= eps {
                        weak.push((at, v > 0.0));
                    }
                    continue;
                }
                let mx = co.iter().fold(0.0f64, |m, v| m.max(v.abs()));
                if mx <= 4.0 * eps || width <= 1e-9 || depth >= 60 {
                    flat.push((s0, s1));
                } else {
                    subdivide(&co, &mut stack);
                }
            }
            1 if first != 0.0 && last != 0.0 => {
                let mut f = |x: f64| eval(&co, x);
                let x = brent(&mut f, 0.0, 1.0, first, last);
                push_unique(&mut out, s0 + x * width, false);
            }
            _ => {
                if width < 1e-12 || depth >= 60 {
                    push_unique(&mut out, 0.5 * (s0 + s1), k >= 2);
                } else {
                    subdivide(&co, &mut stack);
                }
            }
        }
    }
    ends.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut i = 0;
    while i < ends.len() {
        let mut j = i + 1;
        while j < ends.len() && (ends[j].0 - ends[i].0).abs() <= 1e-13 {
            j += 1;
        }
        let grp = &ends[i..j];
        let left = grp.iter().find(|e| !e.1).and_then(|e| e.2);
        let right = grp.iter().find(|e| e.1).and_then(|e| e.2);
        let tangent = matches!((left, right), (Some(a), Some(b)) if a == b);
        if tangent {
            out.push(BRoot { s: ends[i].0, tangent: true });
        } else {
            push_unique(&mut out, ends[i].0, false);
        }
        i = j;
    }
    weak.sort_by(|a, b| a.0.total_cmp(&b.0));
    for w in weak.windows(2) {
        if (w[0].0 - w[1].0).abs() <= 1e-12 && w[0].1 == w[1].1 {
            out.push(BRoot { s: w[0].0, tangent: true });
        }
    }
    // One contact per run of flat stretches.
    flat.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut runs: Vec<(f64, f64)> = Vec::new();
    for (a, b) in flat {
        match runs.last_mut() {
            Some(r) if a <= r.1 + 1e-12 => r.1 = r.1.max(b),
            _ => runs.push((a, b)),
        }
    }
    for (a, b) in runs {
        out.push(BRoot { s: 0.5 * (a + b), tangent: true });
    }
    out.sort_by(|a, b| a.s.total_cmp(&b.s));
    let mut merged: Vec<BRoot> = Vec::new();
    for r in out {
        match merged.last_mut() {
            // Two simple roots this close are one grazing contact (below any model tolerance).
            Some(l) if r.s - l.s <= 1e-7 => {
                l.tangent = true;
                l.s = 0.5 * (l.s + r.s);
            }
            _ => merged.push(r),
        }
    }
    merged
}

fn push_unique(out: &mut Vec<BRoot>, s: f64, tangent: bool) {
    if out.iter().any(|r| (r.s - s).abs() <= 1e-13) {
        return;
    }
    out.push(BRoot { s, tangent });
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Bernstein coefficients of a monomial-form polynomial of degree n by the standard
    /// transformation (test helper).
    fn from_monomial(a: &[f64]) -> Vec<f64> {
        let n = a.len() - 1;
        let binom = |n: usize, k: usize| (0..k).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64);
        (0..=n).map(|i| (0..=i).map(|j| binom(i, j) / binom(n, j) * a[j]).sum()).collect()
    }

    #[test]
    fn simple_roots() {
        // (s - 0.2)(s - 0.5)(s - 0.9) = s^3 - 1.6 s^2 + 0.73 s - 0.09
        let c = from_monomial(&[-0.09, 0.73, -1.6, 1.0]);
        let r = roots(&c, 1e-13);
        assert_eq!(r.len(), 3, "{r:?}");
        for (a, b) in r.iter().zip([0.2, 0.5, 0.9]) {
            assert!((a.s - b).abs() < 1e-12 && !a.tangent);
        }
    }

    #[test]
    fn double_root_is_flagged_once() {
        // (s - 0.4)^2 = s^2 - 0.8 s + 0.16
        let c = from_monomial(&[0.16, -0.8, 1.0]);
        let r = roots(&c, 1e-9);
        assert_eq!(r.len(), 1, "{r:?}");
        assert!(r[0].tangent && (r[0].s - 0.4).abs() < 1e-3);
    }

    #[test]
    fn touching_cubic_and_near_misses() {
        // Bernstein [1, -1/3, -1/3, 1] + h touches (h = 0) or just misses zero at s = 1/2.
        for h in [0.0, 1e-9, 1e-7, 5e-7] {
            let c = [1.0 + h, -1.0 / 3.0 + h, -1.0 / 3.0 + h, 1.0 + h];
            let r = roots(&c, 1e-6);
            assert_eq!(r.len(), 1, "h={h}: {r:?}");
            assert!(r[0].tangent && (r[0].s - 0.5).abs() < 1e-3, "h={h}: {r:?}");
        }
        assert!(roots(&[1.0 + 1e-4, -1.0 / 3.0 + 1e-4, -1.0 / 3.0 + 1e-4, 1.0 + 1e-4], 1e-6).is_empty());
    }

    #[test]
    fn double_root_at_a_split_point_is_tangent() {
        for sgn in [1.0, -1.0] {
            // sgn (s - 0.5)^2
            let c: Vec<f64> = from_monomial(&[0.25, -1.0, 1.0]).into_iter().map(|v| sgn * v).collect();
            let r = roots(&c, 1e-9);
            assert_eq!(r.len(), 1, "{r:?}");
            assert!(r[0].tangent && (r[0].s - 0.5).abs() < 1e-9, "{r:?}");
        }
        // Odd multiplicity at the split stays a crossing: (s-.5)^3, (s-.5)(s^2+1), linear.
        for c in [from_monomial(&[-0.125, 0.75, -1.5, 1.0]), from_monomial(&[-0.5, 1.0, -0.5, 1.0]), from_monomial(&[-0.5, 1.0])] {
            let r = roots(&c, 1e-9);
            assert_eq!(r.len(), 1, "{r:?}");
            assert!(!r[0].tangent && (r[0].s - 0.5).abs() < 1e-6, "{r:?}");
        }
    }

    #[test]
    fn end_roots() {
        let c = from_monomial(&[0.0, 1.0]); // s
        let r = roots(&c, 1e-13);
        assert_eq!(r.len(), 1);
        assert!(r[0].s.abs() < 1e-14);
        let c = from_monomial(&[-1.0, 1.0]); // s - 1
        let r = roots(&c, 1e-13);
        assert_eq!(r.len(), 1);
        assert!((r[0].s - 1.0).abs() < 1e-14);
    }
}

