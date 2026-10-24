//! Arc-length parametrization of an edge: adaptive composite Gauss-Legendre plus a monotone
//! inverse (safeguarded Newton inside one panel).

use crate::error::{Result, SpacingError};
use rayon::prelude::*;
use sm_geom_api::ParametricCurve;
use sm_scheme_api::geom::edge_param;
use sm_topo_api::CurveCarrier;

const GL_X: [f64; 5] = [0.0, -0.538_469_310_105_683_1, 0.538_469_310_105_683_1, -0.906_179_845_938_664, 0.906_179_845_938_664];
const GL_W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_1, 0.236_926_885_056_189_1];

const MIN_PANELS: usize = 8;
const MAX_PANELS: usize = 1 << 14;
/// Panel count from which panels are integrated in parallel (results are order-independent).
const PAR_PANELS: usize = 1 << 11;
/// Node count from which the inverse is evaluated in parallel.
const PAR_NODES: usize = 1 << 11;
const CONVERGED: f64 = 1e-13;
/// Relative speed spread below which the parametrization counts as arc-length proportional.
const UNIFORM_SPREAD: f64 = 1e-13;

/// Arc length as a function of the edge fraction `tau` in `[0, 1]` (from the edge's start).
pub(crate) struct ArcMap<'a> {
    curve: &'a dyn ParametricCurve,
    cc: CurveCarrier,
    span: f64,
    cum: Vec<f64>,
    total: f64,
    uniform: bool,
}

impl<'a> ArcMap<'a> {
    pub(crate) fn new(curve: &'a dyn ParametricCurve, cc: CurveCarrier) -> Result<Self> {
        let span = (cc.range.hi - cc.range.lo).abs();
        if !(cc.range.lo.is_finite() && cc.range.hi.is_finite() && span > 0.0) {
            return Err(SpacingError::Geometry("unbounded or empty curve range".into()));
        }
        let mut map = ArcMap { curve, cc, span, cum: Vec::new(), total: 0.0, uniform: false };
        let panels = |p: usize| -> Vec<(f64, f64, f64)> {
            let one = |j: usize| map.gl(j as f64 / p as f64, (j + 1) as f64 / p as f64);
            if p >= PAR_PANELS { (0..p).into_par_iter().map(one).collect() } else { (0..p).map(one).collect() }
        };
        let sum = |v: &[(f64, f64, f64)]| v.iter().map(|t| t.0).sum::<f64>();
        let mut p = MIN_PANELS;
        let mut cur = panels(p);
        let mut tot = sum(&cur);
        while p < MAX_PANELS {
            let next = panels(2 * p);
            let tot2 = sum(&next);
            let done = (tot2 - tot).abs() <= CONVERGED * tot2.abs();
            p *= 2;
            cur = next;
            tot = tot2;
            if done {
                break;
            }
        }
        let (mut smin, mut smax) = (f64::INFINITY, 0.0f64);
        for t in &cur {
            smin = smin.min(t.1);
            smax = smax.max(t.2);
        }
        if !(tot.is_finite() && tot > 0.0 && smin.is_finite() && smin > 0.0 && smax.is_finite()) {
            return Err(SpacingError::Geometry("curve has zero or non-finite speed".into()));
        }
        let mut cum = Vec::with_capacity(cur.len() + 1);
        let mut acc = 0.0;
        cum.push(0.0);
        for t in &cur {
            acc += t.0;
            cum.push(acc);
        }
        map.total = acc;
        map.cum = cum;
        map.uniform = smax - smin <= UNIFORM_SPREAD * smax;
        Ok(map)
    }

    /// Total arc length.
    pub(crate) fn length(&self) -> f64 {
        self.total
    }

    /// `|d(point)/d(tau)|` at edge fraction `tau`.
    pub(crate) fn speed(&self, tau: f64) -> f64 {
        self.curve.d1(edge_param(&self.cc, tau)).norm() * self.span
    }

    /// The curve and carrier (curvature sampling).
    pub(crate) fn curve(&self) -> (&dyn ParametricCurve, CurveCarrier) {
        (self.curve, self.cc)
    }

    /// Composite 5-point rule on `[a, b]`: `(integral, min speed, max speed)`.
    fn gl(&self, a: f64, b: f64) -> (f64, f64, f64) {
        let (h, mid) = (0.5 * (b - a), 0.5 * (a + b));
        let (mut s, mut lo, mut hi) = (0.0, f64::INFINITY, 0.0f64);
        for (x, w) in GL_X.iter().zip(GL_W) {
            let v = self.speed(mid + h * x);
            s += w * v;
            lo = lo.min(v);
            hi = hi.max(v);
        }
        (s * h, lo, hi)
    }

    /// Edge fraction at arc length `s` from the start.
    fn tau_at(&self, s: f64) -> f64 {
        let p = self.cum.len() - 1;
        let j = self.cum.partition_point(|c| *c <= s).saturating_sub(1).min(p - 1);
        let (lo, hi) = (j as f64 / p as f64, (j + 1) as f64 / p as f64);
        let target = s - self.cum[j];
        let cell = self.cum[j + 1] - self.cum[j];
        if target <= 0.0 {
            return lo;
        }
        if target >= cell {
            return hi;
        }
        let (mut bl, mut bh) = (lo, hi);
        let mut x = lo + (hi - lo) * target / cell;
        for _ in 0..64 {
            let f = self.gl(lo, x).0 - target;
            if f == 0.0 {
                return x;
            }
            if f > 0.0 {
                bh = x;
            } else {
                bl = x;
            }
            let mut nx = x - f / self.speed(x);
            if !(nx >= bl && nx <= bh) {
                nx = 0.5 * (bl + bh);
            }
            let done = (nx - x).abs() <= 4.0 * f64::EPSILON * nx.abs();
            x = nx;
            if done {
                break;
            }
        }
        x
    }

    /// Edge fractions for increasing arc-length fractions `a_k` of the total length.
    pub(crate) fn taus(&self, arc: &[f64]) -> Vec<f64> {
        if self.uniform {
            return arc.to_vec();
        }
        let f = |a: &f64| self.tau_at(a * self.total);
        if arc.len() >= PAR_NODES { arc.par_iter().map(f).collect() } else { arc.iter().map(f).collect() }
    }
}
