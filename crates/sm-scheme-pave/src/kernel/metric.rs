//! Metric correction: the global affine map to an (almost) isotropic frame, the residual
//! metric in it, and the default size field interpolated from the boundary.

use super::PaveError;
use super::cdt::{Cdt, Loc, NONE};
use std::cmp::Reverse;
use std::collections::BinaryHeap;

/// `f64` with a total order, for heaps.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Ord64(pub f64);
impl Eq for Ord64 {}
impl PartialOrd for Ord64 {
    fn partial_cmp(&self, o: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Ord64 {
    fn cmp(&self, o: &Self) -> core::cmp::Ordering {
        self.0.total_cmp(&o.0)
    }
}

/// A symmetric positive definite 2x2 metric `M = B^T B` with `B` upper triangular.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Met {
    b11: f64,
    b12: f64,
    b22: f64,
}

impl Met {
    /// Factor `[m11, m12, m22]`; `None` unless finite and positive definite.
    pub fn from_sym(m: [f64; 3]) -> Option<Met> {
        let [e, f, g] = m;
        if !(e.is_finite() && f.is_finite() && g.is_finite() && e > 0.0) {
            return None;
        }
        let b11 = e.sqrt();
        let b12 = f / b11;
        let r = g - b12 * b12;
        if !(r.is_finite() && r > 1e-13 * g.abs().max(f64::MIN_POSITIVE)) {
            return None;
        }
        Some(Met { b11, b12, b22: r.sqrt() })
    }
    /// `B d`: the vector in the metric-orthonormal frame.
    #[inline]
    pub fn apply(&self, d: [f64; 2]) -> [f64; 2] {
        [self.b11 * d[0] + self.b12 * d[1], self.b22 * d[1]]
    }
    /// `B^-1 v`.
    #[inline]
    pub fn unapply(&self, v: [f64; 2]) -> [f64; 2] {
        let y = v[1] / self.b22;
        [(v[0] - self.b12 * y) / self.b11, y]
    }
    /// `sqrt(d^T M d)`.
    #[inline]
    pub fn len(&self, d: [f64; 2]) -> f64 {
        let v = self.apply(d);
        v[0].hypot(v[1])
    }
}

/// The affine frame `w = R A uv` (`A` upper triangular with `A^T A` the mean first
/// fundamental form of the loop, `R` the x-reflection when the outer loop is clockwise).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Frame {
    a11: f64,
    a12: f64,
    a22: f64,
    pub reflect: bool,
}

impl Frame {
    pub fn new(g_mean: [f64; 3], reflect: bool) -> Option<Frame> {
        let m = Met::from_sym(g_mean)?;
        Some(Frame { a11: m.b11, a12: m.b12, a22: m.b22, reflect })
    }
    pub fn to_w(self, uv: [f64; 2]) -> [f64; 2] {
        let x = self.a11 * uv[0] + self.a12 * uv[1];
        [if self.reflect { -x } else { x }, self.a22 * uv[1]]
    }
    pub fn to_uv(self, w: [f64; 2]) -> [f64; 2] {
        let x = if self.reflect { -w[0] } else { w[0] };
        let v = w[1] / self.a22;
        [(x - self.a12 * v) / self.a11, v]
    }
    /// Residual metric `J^T G J` in the frame, `J = A^-1 R`.
    pub fn residual(&self, g: [f64; 3]) -> [f64; 3] {
        let s = if self.reflect { -1.0 } else { 1.0 };
        let j1 = [s / self.a11, 0.0];
        let j2 = [-self.a12 / (self.a11 * self.a22), 1.0 / self.a22];
        let gm = |a: [f64; 2], b: [f64; 2]| a[0] * (g[0] * b[0] + g[1] * b[1]) + a[1] * (g[1] * b[0] + g[2] * b[1]);
        [gm(j1, j1), gm(j1, j2), gm(j2, j2)]
    }
}

/// The problem as the kernel sees it: callbacks in the original `(u, v)` space plus the frame.
pub(crate) struct Space<'a> {
    pub frame: Frame,
    pub metric: &'a dyn Fn([f64; 2]) -> [f64; 3],
    pub size: Option<&'a dyn Fn([f64; 2]) -> f64>,
}

impl Space<'_> {
    /// Residual metric factor at the frame point `w`.
    pub fn met(&self, w: [f64; 2]) -> Result<Met, PaveError> {
        let g = (self.metric)(self.frame.to_uv(w));
        Met::from_sym(self.frame.residual(g)).ok_or(PaveError::BadMetric)
    }
}

/// Default size field: boundary-segment metric lengths, interpolated over the boundary-node
/// triangulation and limited by `gradation`.
#[derive(Clone)]
pub(crate) struct SizeMap {
    cdt: Cdt,
    s: Vec<f64>,
    hint: u32,
}

impl SizeMap {
    /// `seg_len[i]` is the metric length of the segment leaving boundary node `i` in its
    /// loop; `prev[i]` is the index of the preceding node of the loop.
    pub fn new(cdt: &Cdt, nb: usize, seg_len: &[f64], prev: &[usize], next: &[usize], gradation: f64) -> SizeMap {
        let mut s: Vec<f64> = (0..nb).map(|i| 0.5 * (seg_len[prev[i]] + seg_len[i])).collect();
        // Edge list of the boundary triangulation, Dijkstra-style gradient limiting.
        let mut nbrs: Vec<Vec<u32>> = vec![Vec::new(); nb];
        for t in cdt.triangles() {
            let v = cdt.tv[t as usize];
            for k in 0..3 {
                let (a, b) = (v[k], v[(k + 1) % 3]);
                nbrs[a as usize].push(b);
                nbrs[b as usize].push(a);
            }
        }
        for l in &mut nbrs {
            l.sort_unstable();
            l.dedup();
        }
        let _ = next;
        let mut heap: BinaryHeap<Reverse<(Ord64, u32)>> = (0..nb).map(|i| Reverse((Ord64(s[i]), i as u32))).collect();
        while let Some(Reverse((Ord64(si), i))) = heap.pop() {
            if si > s[i as usize] {
                continue;
            }
            for &j in &nbrs[i as usize] {
                let (pi, pj) = (cdt.pts[i as usize], cdt.pts[j as usize]);
                let d = (pi[0] - pj[0]).hypot(pi[1] - pj[1]);
                let cand = si + gradation * d;
                if cand < s[j as usize] {
                    s[j as usize] = cand;
                    heap.push(Reverse((Ord64(cand), j)));
                }
            }
        }
        SizeMap { cdt: cdt.clone(), s, hint: NONE }
    }

    /// Size at the frame point `w` (linear interpolation; nearest node outside).
    pub fn at(&mut self, w: [f64; 2]) -> f64 {
        let loc = match self.cdt.locate(w, self.hint) {
            Loc::Outside => self.cdt.locate_scan(w),
            l => l,
        };
        let t = match loc {
            Loc::In(t) | Loc::OnEdge(t, _) => t,
            Loc::OnVertex(v) => return self.s[v as usize],
            Loc::Outside => {
                let mut best = (f64::INFINITY, 0usize);
                for (i, p) in self.cdt.pts.iter().enumerate() {
                    let d = (p[0] - w[0]).hypot(p[1] - w[1]);
                    if d < best.0 {
                        best = (d, i);
                    }
                }
                return self.s[best.1];
            }
        };
        self.hint = t;
        let v = self.cdt.tv[t as usize];
        let (a, b, c) = (self.cdt.pts[v[0] as usize], self.cdt.pts[v[1] as usize], self.cdt.pts[v[2] as usize]);
        let area = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
        let tot = area(a, b, c);
        if tot <= 0.0 {
            return self.s[v[0] as usize];
        }
        let (wa, wb, wc) = (area(w, b, c) / tot, area(a, w, c) / tot, area(a, b, w) / tot);
        let (wa, wb, wc) = (wa.max(0.0), wb.max(0.0), wc.max(0.0));
        let sum = wa + wb + wc;
        (wa * self.s[v[0] as usize] + wb * self.s[v[1] as usize] + wc * self.s[v[2] as usize]) / sum
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frame_round_trips_and_identity_is_exact() {
        let f = Frame::new([1.0, 0.0, 1.0], false).expect("frame");
        let p = [0.123456789, -7.5];
        assert_eq!(f.to_w(p), p);
        assert_eq!(f.to_uv(p), p);
        let g = [4.0, 1.2, 2.0];
        let f = Frame::new(g, true).expect("frame");
        let w = f.to_w([0.3, 0.7]);
        let uv = f.to_uv(w);
        assert!((uv[0] - 0.3).abs() < 1e-14 && (uv[1] - 0.7).abs() < 1e-14);
        // A constant metric is exactly isotropic in its own frame.
        let m = f.residual(g);
        assert!((m[0] - 1.0).abs() < 1e-14 && m[1].abs() < 1e-14 && (m[2] - 1.0).abs() < 1e-14, "{m:?}");
    }

    #[test]
    fn bad_metrics_are_rejected() {
        assert!(Met::from_sym([1.0, 2.0, 1.0]).is_none());
        assert!(Met::from_sym([f64::NAN, 0.0, 1.0]).is_none());
        assert!(Met::from_sym([-1.0, 0.0, 1.0]).is_none());
        let m = Met::from_sym([4.0, 0.0, 1.0]).expect("met");
        assert!((m.len([1.0, 0.0]) - 2.0).abs() < 1e-15);
        let v = m.apply([0.3, 0.4]);
        let d = m.unapply(v);
        assert!((d[0] - 0.3).abs() < 1e-15 && (d[1] - 0.4).abs() < 1e-15);
    }
}
