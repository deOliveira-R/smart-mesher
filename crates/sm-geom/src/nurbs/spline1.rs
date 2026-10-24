//! One-dimensional spline algorithms on homogeneous poles `[w x, w y, w z, w]`: knot insertion
//! (Boehm, A5.1), the periodic variant, clamping an unclamped/periodic spline, and Bezier
//! extraction. Curves use them directly; surfaces apply them to every row or column of the net.

use super::basis::find_span;
use sm_geom_api::GeomError;

/// Homogeneous pole.
pub(crate) type Hp = [f64; 4];

/// A 1D spline's structure: degree, extended knots, periodicity.
#[derive(Clone, Debug)]
pub(crate) struct Layout {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub periodic: bool,
}

fn lerp(a: Hp, b: Hp, s: f64) -> Hp {
    // s*a + (1-s)*b
    [s * a[0] + (1.0 - s) * b[0], s * a[1] + (1.0 - s) * b[1], s * a[2] + (1.0 - s) * b[2], s * a[3] + (1.0 - s) * b[3]]
}

/// Multiplicity of `t` in `knots` (exact comparison).
pub(crate) fn multiplicity(knots: &[f64], t: f64) -> usize {
    knots.iter().filter(|k| **k == t).count()
}

impl Layout {
    /// Number of poles for a pole array (`knots.len() - degree - 1`).
    pub(crate) fn npoles(&self) -> usize {
        self.knots.len() - self.degree - 1
    }

    /// Parameter range `[k_p, k_n]`.
    pub(crate) fn range(&self) -> (f64, f64) {
        (self.knots[self.degree], self.knots[self.npoles()])
    }

    /// Insert `t` once; the result keeps the periodicity of `self`.
    pub(crate) fn insert_once(&self, poles: &[Hp], t: f64) -> Result<(Layout, Vec<Hp>), GeomError> {
        if self.periodic { self.insert_periodic(poles, t) } else { self.insert_open(poles, t) }
    }

    /// Boehm insertion into a non-periodic spline. `t` must lie in the range; inserting at the
    /// very ends is allowed (it is how clamping is done).
    fn insert_open(&self, poles: &[Hp], t: f64) -> Result<(Layout, Vec<Hp>), GeomError> {
        let p = self.degree;
        let n = poles.len();
        let (lo, hi) = self.range();
        if !(t >= lo && t <= hi) {
            return Err(GeomError::OutOfDomain(t));
        }
        let k = &self.knots;
        let s = if t >= hi { n - 1 } else { find_span(n, p, t, k) };
        let mut kn = Vec::with_capacity(k.len() + 1);
        kn.extend_from_slice(&k[..=s]);
        kn.push(t);
        kn.extend_from_slice(&k[s + 1..]);
        let mut q = Vec::with_capacity(n + 1);
        for i in 0..=n {
            if i + p <= s {
                q.push(poles[i]);
            } else if i > s {
                q.push(poles[i - 1]);
            } else {
                let a = (t - k[i]) / (k[i + p] - k[i]);
                q.push(lerp(poles[i], poles[i - 1], a));
            }
        }
        Ok((Layout { degree: p, knots: kn, periodic: false }, q))
    }

    /// Insertion into a periodic spline: the knot and all its period copies. Needs more unique
    /// poles than the degree (otherwise the support of one basis function wraps onto itself).
    fn insert_periodic(&self, poles: &[Hp], t: f64) -> Result<(Layout, Vec<Hp>), GeomError> {
        let p = self.degree;
        let np = poles.len();
        let m = np - p;
        if m <= p {
            return Err(GeomError::Unsupported("knot insertion into a periodic nurbs needs more unique poles than the degree"));
        }
        let (lo, hi) = self.range();
        let period = hi - lo;
        let t = lo + (t - lo).rem_euclid(period);
        let t = if t >= hi { lo } else { t };
        let k = &self.knots;
        let kn_at = |i: isize| -> f64 {
            let mi = m as isize;
            k[i.rem_euclid(mi) as usize] + (i.div_euclid(mi)) as f64 * period
        };
        let pole_at = |i: isize| -> Hp { poles[i.rem_euclid(m as isize) as usize] };
        // Span s in [p, p+m) with kn(s) <= t < kn(s+1).
        let mut s = p;
        while s + 1 < p + m && kn_at(s as isize + 1) <= t {
            s += 1;
        }
        let si = s as isize;
        let pi = p as isize;
        let mp = (m + 1) as isize;
        // Canonical window W = [s-p+1, s-p+m+1] of the first-copy formulas.
        let w0 = si - pi + 1;
        let f = |j: isize| -> Hp {
            if j > si {
                pole_at(j - 1)
            } else {
                let a = (t - kn_at(j)) / (kn_at(j + pi) - kn_at(j));
                lerp(pole_at(j), pole_at(j - 1), a)
            }
        };
        let newp = m + 1 + p;
        let mut q = Vec::with_capacity(newp);
        for j in 0..newp as isize {
            q.push(f(w0 + (j - w0).rem_euclid(mp)));
        }
        // New knots: periodic extension of one sorted window.
        let mut b: Vec<f64> = (p..p + m).map(|i| kn_at(i as isize)).collect();
        let pos = b.partition_point(|x| *x <= t);
        b.insert(pos, t);
        let total = newp + p + 1;
        let kn: Vec<f64> = (0..total as isize).map(|i| b[(i - pi).rem_euclid(mp) as usize] + ((i - pi).div_euclid(mp)) as f64 * period).collect();
        Ok((Layout { degree: p, knots: kn, periodic: true }, q))
    }

    /// Convert to a clamped, non-periodic spline over the same parameter range (a periodic
    /// spline is cut at its seam).
    pub(crate) fn clamped(&self, poles: &[Hp]) -> Result<(Layout, Vec<Hp>), GeomError> {
        let p = self.degree;
        let (lo, hi) = self.range();
        let mut lay = Layout { degree: p, knots: self.knots.clone(), periodic: false };
        let mut pts = poles.to_vec();
        for t in [lo, hi] {
            while multiplicity(&lay.knots, t) < p + 1 {
                let (l, q) = lay.insert_open(&pts, t)?;
                lay = l;
                pts = q;
            }
        }
        // Keep the blocks [lo x (p+1)] .. [hi x (p+1)].
        let a = lay.knots.partition_point(|k| *k < lo);
        let b = lay.knots.partition_point(|k| *k <= hi);
        let knots = lay.knots[a..b].to_vec();
        let npoles = knots.len() - p - 1;
        let poles = pts[a..a + npoles].to_vec();
        Ok((Layout { degree: p, knots, periodic: false }, poles))
    }

    /// Clamp (if needed) and raise every interior knot to multiplicity `degree`: the result is
    /// a chain of Bezier segments sharing end poles (`segments * degree + 1` poles).
    pub(crate) fn bezierized(&self, poles: &[Hp]) -> Result<(Layout, Vec<Hp>), GeomError> {
        let p = self.degree;
        let k = &self.knots;
        let is_clamped = !self.periodic && k[0] == k[p] && k[k.len() - 1] == k[self.npoles()] && multiplicity(k, k[p]) > p && multiplicity(k, k[self.npoles()]) > p;
        let (mut l, mut cur) = if is_clamped { (self.clone(), poles.to_vec()) } else { self.clamped(poles)? };
        let mut distinct: Vec<f64> = Vec::new();
        for kk in &l.knots[p + 1..l.knots.len() - p - 1] {
            if distinct.last() != Some(kk) {
                distinct.push(*kk);
            }
        }
        for t in distinct {
            while multiplicity(&l.knots, t) < p {
                let (nl, q) = l.insert_open(&cur, t)?;
                l = nl;
                cur = q;
            }
        }
        Ok((l, cur))
    }

    /// Bezier extraction: for each non-empty knot span, its parameter interval and the `p + 1`
    /// homogeneous poles of the segment.
    pub(crate) fn bezier(&self, poles: &[Hp]) -> Result<Vec<(f64, f64, Vec<Hp>)>, GeomError> {
        let p = self.degree;
        let (l, cur) = self.bezierized(poles)?;
        let nseg = (cur.len() - 1) / p;
        let mut out = Vec::with_capacity(nseg);
        for s in 0..nseg {
            let (a, b) = (l.knots[(s + 1) * p], l.knots[(s + 1) * p + 1]);
            out.push((a, b, cur[s * p..=(s + 1) * p].to_vec()));
        }
        Ok(out)
    }
}

/// de Casteljau split at the middle of `poles` (`n <= 16` homogeneous poles) into `left` and
/// `right` (both of length `n`), without allocation.
pub(crate) fn split_half_into(poles: &[Hp], left: &mut [Hp], right: &mut [Hp]) {
    let n = poles.len();
    let mut work = [[0.0f64; 4]; 16];
    work[..n].copy_from_slice(poles);
    left[0] = work[0];
    right[n - 1] = work[n - 1];
    for level in 1..n {
        for i in 0..n - level {
            for c in 0..4 {
                work[i][c] = 0.5 * (work[i][c] + work[i + 1][c]);
            }
        }
        left[level] = work[0];
        right[n - 1 - level] = work[n - 1 - level];
    }
}

/// de Casteljau split of a homogeneous Bezier segment at `s` in `(0, 1)`: the left and right
/// halves, each with the same number of poles (reference form used by the tests).
#[cfg(test)]
pub(crate) fn bezier_split(poles: &[Hp], s: f64) -> (Vec<Hp>, Vec<Hp>) {
    let n = poles.len();
    let mut work = poles.to_vec();
    let mut left = Vec::with_capacity(n);
    let mut right = vec![[0.0; 4]; n];
    left.push(work[0]);
    right[n - 1] = work[n - 1];
    for level in 1..n {
        for i in 0..n - level {
            work[i] = lerp(work[i + 1], work[i], s);
        }
        left.push(work[0]);
        right[n - 1 - level] = work[n - 1 - level];
    }
    (left, right)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn eval(l: &Layout, poles: &[Hp], t: f64) -> [f64; 3] {
        let p = l.degree;
        let (lo, hi) = l.range();
        let t = if l.periodic { lo + (t - lo).rem_euclid(hi - lo) } else { t };
        let s = find_span(poles.len(), p, t, &l.knots);
        let d = super::super::basis::ders(s, t, p, &l.knots);
        let mut acc = [0.0; 4];
        for j in 0..=p {
            for c in 0..4 {
                acc[c] += d[0][j] * poles[s - p + j][c];
            }
        }
        [acc[0] / acc[3], acc[1] / acc[3], acc[2] / acc[3]]
    }

    fn cubic() -> (Layout, Vec<Hp>) {
        let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.5, 3.5, 3.5, 3.5];
        let pts: Vec<Hp> = [[0.0, 0.0, 0.0], [1.0, 2.0, 0.5], [3.0, 3.0, -1.0], [4.0, 0.5, 2.0], [6.0, 1.0, 0.0], [7.0, 4.0, 1.0]]
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let w = 1.0 + 0.3 * i as f64;
                [p[0] * w, p[1] * w, p[2] * w, w]
            })
            .collect();
        (Layout { degree: 3, knots, periodic: false }, pts)
    }

    #[test]
    fn open_insertion_preserves_shape() {
        let (l, p) = cubic();
        let (l2, p2) = l.insert_once(&p, 1.3).unwrap();
        let (l3, p3) = l2.insert_once(&p2, 1.3).unwrap();
        let (l4, p4) = l3.insert_once(&p3, 2.0).unwrap();
        assert_eq!(p4.len(), p.len() + 3);
        for i in 0..=70 {
            let t = 3.5 * i as f64 / 70.0;
            let (a, b) = (eval(&l, &p, t), eval(&l4, &p4, t));
            assert!((0..3).all(|c| (a[c] - b[c]).abs() < 1e-13), "t={t}");
        }
    }

    /// Uniform periodic cubic B-spline with 6 unique poles.
    fn periodic_cubic() -> (Layout, Vec<Hp>) {
        let u = [[0.0, 0.0, 0.0], [2.0, 0.5, 0.2], [3.0, 2.0, -0.3], [1.5, 3.0, 0.4], [-0.5, 2.0, 0.0], [-1.0, 0.7, 0.1]];
        let m = u.len();
        let p = 3;
        let pts: Vec<Hp> = (0..m + p).map(|i| {
            let q = u[i % m];
            let w = 1.0 + 0.2 * ((i % m) as f64);
            [q[0] * w, q[1] * w, q[2] * w, w]
        }).collect();
        let knots: Vec<f64> = (0..m + 2 * p + 1).map(|i| i as f64 - p as f64).collect();
        (Layout { degree: p, knots, periodic: true }, pts)
    }

    #[test]
    fn periodic_insertion_preserves_shape_and_periodicity() {
        let (l, p) = periodic_cubic();
        let (l1, p1) = l.insert_once(&p, 2.4).unwrap();
        let (l2, p2) = l1.insert_once(&p1, 0.5).unwrap();
        let (l3, p3) = l2.insert_once(&p2, 5.9).unwrap();
        // Same domain start and period.
        assert_eq!(l.range().0, l3.range().0);
        assert!(((l.range().1 - l.range().0) - (l3.range().1 - l3.range().0)).abs() < 1e-14);
        // First `degree` poles repeat at the end (the contract's periodic rule).
        for i in 0..3 {
            assert_eq!(p3[i], p3[p3.len() - 3 + i]);
        }
        for i in 0..=120 {
            let t = -1.0 + 7.3 * i as f64 / 120.0;
            let (a, b) = (eval(&l, &p, t), eval(&l3, &p3, t));
            assert!((0..3).all(|c| (a[c] - b[c]).abs() < 1e-13), "t={t}: {a:?} {b:?}");
        }
    }

    #[test]
    fn clamping_a_periodic_spline_and_bezier_extraction() {
        let (l, p) = periodic_cubic();
        let (cl, cp) = l.clamped(&p).unwrap();
        assert_eq!(cl.knots[0], cl.knots[3]);
        for i in 0..=100 {
            let t = l.range().0 + (l.range().1 - l.range().0) * i as f64 / 100.0;
            let (a, b) = (eval(&l, &p, t), eval(&cl, &cp, t));
            assert!((0..3).all(|c| (a[c] - b[c]).abs() < 1e-13), "t={t}");
        }
        let segs = cl.bezier(&cp).unwrap();
        assert_eq!(segs.len(), 6);
        for (a, b, bp) in &segs {
            let bl = Layout { degree: 3, knots: vec![*a, *a, *a, *a, *b, *b, *b, *b], periodic: false };
            for i in 0..=10 {
                let t = a + (b - a) * i as f64 / 10.0;
                let (x, y) = (eval(&bl, bp, t), eval(&l, &p, t));
                assert!((0..3).all(|c| (x[c] - y[c]).abs() < 1e-12));
            }
        }
    }

    #[test]
    fn de_casteljau_split_reproduces_segment() {
        let (l, p) = cubic();
        let segs = l.bezier(&p).unwrap();
        let (a, b, bp) = &segs[1];
        let (left, right) = bezier_split(bp, 0.3);
        let mid = a + 0.3 * (b - a);
        let ll = Layout { degree: 3, knots: vec![*a, *a, *a, *a, mid, mid, mid, mid], periodic: false };
        let rl = Layout { degree: 3, knots: vec![mid, mid, mid, mid, *b, *b, *b, *b], periodic: false };
        for i in 0..=10 {
            let s = i as f64 / 10.0;
            let t1 = a + (mid - a) * s;
            let t2 = mid + (b - mid) * s;
            let (x1, y1) = (eval(&ll, &left, t1), eval(&l, &p, t1));
            let (x2, y2) = (eval(&rl, &right, t2), eval(&l, &p, t2));
            assert!((0..3).all(|c| (x1[c] - y1[c]).abs() < 1e-12 && (x2[c] - y2[c]).abs() < 1e-12));
        }
    }
}
