//! Arc-length parametrization of an edge: locally adaptive Gauss-Legendre plus a monotone
//! inverse (safeguarded Newton inside one panel).

use crate::error::{Result, SpacingError};
use rayon::prelude::*;
use sm_geom_api::ParametricCurve;
use sm_scheme_api::geom::edge_param;
use sm_topo_api::CurveCarrier;

const GL_X: [f64; 5] = [0.0, -0.538_469_310_105_683_1, 0.538_469_310_105_683_1, -0.906_179_845_938_664, 0.906_179_845_938_664];
const GL_W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_1, 0.236_926_885_056_189_1];

/// Initial uniform panels before local refinement.
const MIN_PANELS: usize = 8;
/// Most panels the refinement may produce before the map is refused.
const MAX_PANELS: usize = 1 << 16;
/// A panel is accepted when its whole and split estimates differ by at most this times the
/// initial length estimate.
const PANEL_TOL: f64 = 1e-14;
/// Panels narrower than this (in edge fraction) are accepted as they are: a kink cannot be
/// resolved further in double precision.
const MIN_WIDTH: f64 = 1e-13;
/// Node count from which the inverse is evaluated in parallel.
const PAR_NODES: usize = 1 << 11;
/// Relative speed spread below which the parametrization counts as arc-length proportional.
const UNIFORM_SPREAD: f64 = 1e-13;

/// Arc length as a function of the edge fraction `tau` in `[0, 1]` (from the edge's start).
///
/// Panels are refined locally where the speed is not smooth (repeated knots, kinks), so the map
/// is accurate on non-uniformly parametrized NURBS without knowing the knots.
pub(crate) struct ArcMap<'a> {
    curve: &'a dyn ParametricCurve,
    cc: CurveCarrier,
    span: f64,
    /// Panel breakpoints in the edge fraction (`0 ..= 1`).
    tb: Vec<f64>,
    /// Arc length at each breakpoint.
    cum: Vec<f64>,
    total: f64,
    uniform: bool,
}

/// One panel `[a, b]` and its 5-point integral.
#[derive(Clone, Copy)]
struct Panel {
    a: f64,
    b: f64,
    i: f64,
}

impl<'a> ArcMap<'a> {
    pub(crate) fn new(curve: &'a dyn ParametricCurve, cc: CurveCarrier) -> Result<Self> {
        let span = (cc.range.hi - cc.range.lo).abs();
        if !(cc.range.lo.is_finite() && cc.range.hi.is_finite() && span > 0.0) {
            return Err(SpacingError::Geometry("unbounded or empty curve range".into()));
        }
        let mut map = ArcMap { curve, cc, span, tb: Vec::new(), cum: Vec::new(), total: 0.0, uniform: false };
        let (mut smin, mut smax) = (f64::INFINITY, 0.0f64);
        let mut eval = |map: &ArcMap<'_>, a: f64, b: f64| -> Panel {
            let (i, lo, hi) = map.gl(a, b);
            smin = smin.min(lo);
            smax = smax.max(hi);
            Panel { a, b, i }
        };
        let init: Vec<Panel> = (0..MIN_PANELS).map(|j| eval(&map, j as f64 / MIN_PANELS as f64, (j + 1) as f64 / MIN_PANELS as f64)).collect();
        let l0: f64 = init.iter().map(|p| p.i).sum();
        if !(l0.is_finite() && l0 > 0.0) {
            return Err(SpacingError::Geometry("curve has zero or non-finite speed".into()));
        }
        let tol = PANEL_TOL * l0;
        // Depth-first refinement in a fixed order, so the result is deterministic.
        let mut out: Vec<Panel> = Vec::with_capacity(4 * MIN_PANELS);
        let mut stack: Vec<Panel> = init.into_iter().rev().collect();
        while let Some(p) = stack.pop() {
            let m = 0.5 * (p.a + p.b);
            let (l, r) = (eval(&map, p.a, m), eval(&map, m, p.b));
            let diff = (p.i - (l.i + r.i)).abs();
            if !diff.is_finite() {
                return Err(SpacingError::Geometry("curve has non-finite speed".into()));
            }
            if diff <= tol || p.b - p.a <= MIN_WIDTH {
                out.push(l);
                out.push(r);
            } else {
                stack.push(r);
                stack.push(l);
            }
            if out.len() + stack.len() > MAX_PANELS {
                return Err(SpacingError::Geometry("arc length did not converge: speed is not integrable to double precision on this curve".into()));
            }
        }
        if !(smin.is_finite() && smin > 0.0 && smax.is_finite()) {
            return Err(SpacingError::Geometry("curve has zero or non-finite speed".into()));
        }
        let mut tb = Vec::with_capacity(out.len() + 1);
        let mut cum = Vec::with_capacity(out.len() + 1);
        let mut acc = 0.0;
        tb.push(0.0);
        cum.push(0.0);
        for p in &out {
            acc += p.i;
            tb.push(p.b);
            cum.push(acc);
        }
        if !(acc.is_finite() && acc > 0.0) {
            return Err(SpacingError::Geometry("curve has zero or non-finite speed".into()));
        }
        map.total = acc;
        map.tb = tb;
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

    /// 5-point rule on `[a, b]`: `(integral, min speed, max speed)`.
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
        let (lo, hi) = (self.tb[j], self.tb[j + 1]);
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
