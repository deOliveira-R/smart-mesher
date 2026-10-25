//! [`PreparedCurve`]: a NURBS curve compiled for fast evaluation, projection and editing.

use super::basis::{ders_n, find_span, values_n};
use super::project;
use super::spline1::{Hp, Layout};
use sm_core::{Point3, Vector3};
use sm_geom_api::{CurveJet, CurveProjection, Domain1, GeomError, Interval, NurbsCurve, ParametricCurve};
use smallvec::SmallVec;

/// Inline pole storage for one Bezier segment (degree <= 7 stays on the stack).
pub(crate) type SegPoles = SmallVec<[Hp; 8]>;

/// One Bezier segment of a curve: parameter interval, homogeneous poles and the box of its
/// (Cartesian) control polygon, which bounds the segment by the convex-hull property.
#[derive(Clone, Debug)]
pub struct BezierSegment {
    /// Parameter interval `[lo, hi]` of the segment in the curve's parameter.
    pub range: Interval,
    /// Control points (Cartesian), `degree + 1` of them.
    pub control: Vec<Point3>,
    /// Weights (all `1` for a polynomial curve).
    pub weights: Vec<f64>,
}

/// Cached Bezier decomposition used by projection and intersection.
#[derive(Clone, Debug)]
pub(crate) struct Segment {
    pub lo: f64,
    pub hi: f64,
    pub poles: SegPoles,
    pub bbox: [f64; 6],
}

/// A NURBS curve prepared for the hot path: homogeneous poles in one contiguous array, a
/// pre-decomposed Bezier chain for bounding-box culling, and allocation-free evaluation.
///
/// Built from the contract's [`NurbsCurve`] (which stays the serialized form, see
/// [`Self::to_def`]); evaluation agrees with the reference evaluator to rounding.
#[derive(Clone, Debug)]
pub struct PreparedCurve {
    lay: Layout,
    poles: Vec<Hp>,
    rational: bool,
    lo: f64,
    hi: f64,
    pub(crate) segs: Vec<Segment>,
    bbox: [f64; 6],
}

pub(crate) fn hp_from(p: Point3, w: f64) -> Hp {
    [p.x * w, p.y * w, p.z * w, w]
}

pub(crate) fn hp_point(h: Hp) -> Point3 {
    Point3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3])
}

pub(crate) fn bbox_of(points: impl Iterator<Item = Point3>) -> [f64; 6] {
    let mut b = [f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for p in points {
        b[0] = b[0].min(p.x);
        b[1] = b[1].min(p.y);
        b[2] = b[2].min(p.z);
        b[3] = b[3].max(p.x);
        b[4] = b[4].max(p.y);
        b[5] = b[5].max(p.z);
    }
    b
}

/// Distance from `p` to the axis-aligned box (zero inside).
pub(crate) fn box_dist(b: &[f64; 6], p: Point3) -> f64 {
    let dx = (b[0] - p.x).max(0.0).max(p.x - b[3]);
    let dy = (b[1] - p.y).max(0.0).max(p.y - b[4]);
    let dz = (b[2] - p.z).max(0.0).max(p.z - b[5]);
    (dx * dx + dy * dy + dz * dz).sqrt()
}

fn fold(lo: f64, hi: f64, periodic: bool, t: f64) -> f64 {
    if periodic {
        let w = lo + (t - lo).rem_euclid(hi - lo);
        if w >= hi { lo } else { w }
    } else {
        t.max(lo).min(hi)
    }
}

impl PreparedCurve {
    /// Prepare a validated curve.
    pub fn new(c: &NurbsCurve) -> Result<Self, GeomError> {
        c.validate()?;
        let rational = c.weights.is_some();
        let poles: Vec<Hp> = c.control.iter().enumerate().map(|(i, p)| hp_from(*p, c.weights.as_ref().map_or(1.0, |w| w[i]))).collect();
        Self::from_parts(Layout { degree: c.degree, knots: c.knots.clone(), periodic: c.periodic }, poles, rational)
    }

    pub(crate) fn from_parts(lay: Layout, poles: Vec<Hp>, rational: bool) -> Result<Self, GeomError> {
        let (lo, hi) = lay.range();
        let segs = lay
            .bezier(&poles)?
            .into_iter()
            .map(|(a, b, pp)| {
                let bbox = bbox_of(pp.iter().map(|h| hp_point(*h)));
                Segment { lo: a, hi: b, poles: pp.into_iter().collect(), bbox }
            })
            .collect::<Vec<_>>();
        let mut bbox = bbox_of(poles.iter().map(|h| hp_point(*h)));
        if let Some(first) = segs.first() {
            bbox = first.bbox;
            for s in &segs[1..] {
                for k in 0..3 {
                    bbox[k] = bbox[k].min(s.bbox[k]);
                    bbox[k + 3] = bbox[k + 3].max(s.bbox[k + 3]);
                }
            }
        }
        Ok(Self { lay, poles, rational, lo, hi, segs, bbox })
    }

    /// The contract data this curve was built from (or the result of an edit).
    pub fn to_def(&self) -> Result<NurbsCurve, GeomError> {
        let control: Vec<Point3> = self.poles.iter().map(|h| hp_point(*h)).collect();
        let weights = self.rational.then(|| self.poles.iter().map(|h| h[3]).collect());
        if self.lay.periodic {
            NurbsCurve::new_periodic(self.lay.degree, self.lay.knots.clone(), control, weights)
        } else {
            NurbsCurve::new(self.lay.degree, self.lay.knots.clone(), control, weights)
        }
    }

    /// Polynomial degree.
    pub fn degree(&self) -> usize {
        self.lay.degree
    }

    /// Number of control points.
    pub fn pole_count(&self) -> usize {
        self.poles.len()
    }

    /// True if the curve carries weights.
    pub fn is_rational(&self) -> bool {
        self.rational
    }

    /// The parameter range `[knots[p], knots[n]]`.
    pub fn range(&self) -> Interval {
        Interval::new(self.lo, self.hi)
    }

    /// Axis-aligned box `[xmin, ymin, zmin, xmax, ymax, zmax]` of the control polygon (contains
    /// the curve by the convex-hull property).
    pub fn control_bbox(&self) -> [f64; 6] {
        self.bbox
    }

    pub(crate) fn fold(&self, t: f64) -> f64 {
        fold(self.lo, self.hi, self.lay.periodic, t)
    }

    /// Homogeneous derivative sums `A_k = sum N_j^(k) P^w_j`, `k = 0..=2`.
    #[inline]
    fn homogeneous(&self, t: f64) -> [Hp; 3] {
        let p = self.lay.degree;
        let span = find_span(self.poles.len(), p, t, &self.lay.knots);
        // Scratch width by degree keeps the basis arrays small for the common low degrees.
        if p < 4 {
            self.homogeneous_n::<4>(t, span)
        } else if p < 8 {
            self.homogeneous_n::<8>(t, span)
        } else {
            self.homogeneous_n::<16>(t, span)
        }
    }

    #[inline]
    fn homogeneous_n<const N: usize>(&self, t: f64, span: usize) -> [Hp; 3] {
        let p = self.lay.degree;
        let d = ders_n::<N>(span, t, p, &self.lay.knots);
        let mut a = [[0.0f64; 4]; 3];
        for j in 0..=p {
            let h = self.poles[span - p + j];
            for (k, ak) in a.iter_mut().enumerate() {
                let n = d[k][j];
                ak[0] += n * h[0];
                ak[1] += n * h[1];
                ak[2] += n * h[2];
                ak[3] += n * h[3];
            }
        }
        a
    }

    /// Position and derivatives of arbitrary order up to the degree is not needed by callers;
    /// the contract's jet (order 2) is [`ParametricCurve::jet`].
    #[inline]
    fn jet_at(&self, t: f64) -> CurveJet {
        let a = self.homogeneous(t);
        let iw = 1.0 / a[0][3];
        let c0 = Vector3::new(a[0][0] * iw, a[0][1] * iw, a[0][2] * iw);
        let v = |k: usize| Vector3::new(a[k][0], a[k][1], a[k][2]);
        let c1 = (v(1) - c0 * a[1][3]) * iw;
        let c2 = (v(2) - c1 * (2.0 * a[1][3]) - c0 * a[2][3]) * iw;
        CurveJet { p: Point3::new(c0.x, c0.y, c0.z), d1: c1, d2: c2 }
    }

    /// Bezier decomposition of the curve (periodic curves are cut at their seam).
    pub fn bezier_segments(&self) -> Vec<BezierSegment> {
        self.segs
            .iter()
            .map(|s| BezierSegment {
                range: Interval::new(s.lo, s.hi),
                control: s.poles.iter().map(|h| hp_point(*h)).collect(),
                weights: s.poles.iter().map(|h| h[3]).collect(),
            })
            .collect()
    }

    /// Insert the knot `t` `times` times (keeps the shape; a periodic curve keeps its period
    /// copies). `t` must lie in the open parameter interval: the ends, including the seam of a
    /// periodic curve, are refused with [`GeomError::OutOfDomain`] (a knot already sits there;
    /// wrap `t` into the interior if a seam knot is wanted). Fails if the multiplicity would
    /// exceed the degree.
    pub fn insert_knot(&self, t: f64, times: usize) -> Result<PreparedCurve, GeomError> {
        let (lo, hi) = (self.lo, self.hi);
        if !(t > lo && t < hi) {
            return Err(GeomError::OutOfDomain(t));
        }
        let mut lay = self.lay.clone();
        let mut poles = self.poles.clone();
        let existing = super::spline1::multiplicity(&lay.knots, t);
        if existing + times > lay.degree {
            return Err(GeomError::Invalid("knot multiplicity would exceed the degree"));
        }
        for _ in 0..times {
            let (l, q) = lay.insert_once(&poles, t)?;
            lay = l;
            poles = q;
        }
        Self::from_parts(lay, poles, self.rational)
    }

    /// Insert several knots (ascending or not); equivalent to repeated [`Self::insert_knot`].
    pub fn refine(&self, knots: &[f64]) -> Result<PreparedCurve, GeomError> {
        let mut cur = self.clone();
        for &t in knots {
            cur = cur.insert_knot(t, 1)?;
        }
        Ok(cur)
    }

    /// The same curve as a clamped, non-periodic spline over the same range.
    pub fn clamped(&self) -> Result<PreparedCurve, GeomError> {
        let (lay, poles) = self.lay.clamped(&self.poles)?;
        Self::from_parts(lay, poles, self.rational)
    }

    /// Split at an interior parameter into two clamped curves covering `[lo, t]` and `[t, hi]`.
    pub fn split(&self, t: f64) -> Result<(PreparedCurve, PreparedCurve), GeomError> {
        if !(t > self.lo && t < self.hi) {
            return Err(GeomError::OutOfDomain(t));
        }
        let p = self.lay.degree;
        let base = self.clamped()?;
        let mut lay = base.lay.clone();
        let mut poles = base.poles.clone();
        while super::spline1::multiplicity(&lay.knots, t) < p + 1 {
            let (l, q) = lay.insert_once(&poles, t)?;
            lay = l;
            poles = q;
        }
        let a = lay.knots.partition_point(|k| *k < t);
        let b = lay.knots.partition_point(|k| *k <= t);
        // Left: knots up to the block, closed by one more copy so it is clamped at t.
        let mut lk = lay.knots[..b].to_vec();
        let lp = poles[..lk.len() - p - 1].to_vec();
        lk.truncate(lp.len() + p + 1);
        let mut rk = lay.knots[a..].to_vec();
        let rp_start = a;
        let rp = poles[rp_start..].to_vec();
        rk.truncate(rp.len() + p + 1);
        let left = Self::from_parts(Layout { degree: p, knots: lk, periodic: false }, lp, self.rational)?;
        let right = Self::from_parts(Layout { degree: p, knots: rk, periodic: false }, rp, self.rational)?;
        Ok((left, right))
    }

    /// Closest points of many query points, in parallel; result `i` belongs to `points[i]`
    /// (order-preserving, so the output does not depend on the thread count).
    pub fn project_many(&self, points: &[Point3]) -> Vec<CurveProjection> {
        use rayon::prelude::*;
        points.par_iter().map(|p| project::project_curve(self, *p, None)).collect()
    }
}

impl PreparedCurve {
    #[inline]
    fn point_n<const N: usize>(&self, t: f64, span: usize) -> Point3 {
        let p = self.lay.degree;
        let n = values_n::<N>(span, t, p, &self.lay.knots);
        let mut a = [0.0f64; 4];
        for j in 0..=p {
            let h = self.poles[span - p + j];
            a[0] += n[j] * h[0];
            a[1] += n[j] * h[1];
            a[2] += n[j] * h[2];
            a[3] += n[j] * h[3];
        }
        let iw = 1.0 / a[3];
        Point3::new(a[0] * iw, a[1] * iw, a[2] * iw)
    }
}

impl ParametricCurve for PreparedCurve {
    fn breakpoints(&self) -> Vec<f64> {
        sm_geom_api::nurbs::interior_breakpoints(&self.lay.knots, self.lo, self.hi)
    }

    fn domain(&self) -> Domain1 {
        if self.lay.periodic { Domain1::periodic(self.lo, self.hi - self.lo) } else { Domain1::open(self.lo, self.hi) }
    }

    fn jet(&self, t: f64) -> CurveJet {
        self.jet_at(self.fold(t))
    }

    /// Position only: skips the derivative basis.
    fn eval(&self, t: f64) -> Point3 {
        let t = self.fold(t);
        let p = self.lay.degree;
        let span = find_span(self.poles.len(), p, t, &self.lay.knots);
        if p < 4 {
            self.point_n::<4>(t, span)
        } else if p < 8 {
            self.point_n::<8>(t, span)
        } else {
            self.point_n::<16>(t, span)
        }
    }

    /// Parallel batch evaluation; element-independent writes, so the output is bit-identical to
    /// the default loop for any thread count.
    fn eval_many(&self, params: &[f64], out: &mut [Point3]) {
        use rayon::prelude::*;
        assert_eq!(params.len(), out.len(), "eval_many: length mismatch");
        out.par_chunks_mut(256).zip(params.par_chunks(256)).for_each(|(o, t)| {
            for (o, t) in o.iter_mut().zip(t) {
                *o = self.eval(*t);
            }
        });
    }

    fn project(&self, p: Point3, hint: Option<f64>) -> Result<CurveProjection, GeomError> {
        Ok(project::project_curve(self, p, hint))
    }

    /// Composite Gauss-Legendre per knot span (the integrand is smooth inside a span), refined
    /// until the estimate is stable to `tol`. On a periodic curve, whole periods inside the range
    /// are not integrated again: the length of one period is multiplied by their count, so the
    /// cost does not depend on how many periods the range spans.
    fn length(&self, range: Interval, tol: f64) -> Result<f64, GeomError> {
        if !(range.is_valid() && range.is_bounded()) {
            return Err(GeomError::Invalid("arc length needs a bounded, ordered range"));
        }
        if !self.lay.periodic {
            return self.length_pieces(range, tol);
        }
        let period = self.hi - self.lo;
        let j0 = ((range.lo - self.lo) / period).floor();
        let j1 = ((range.hi - self.lo) / period).floor();
        if (j1 - j0).is_nan() || j1 - j0 < 2.0 {
            return self.length_pieces(range, tol);
        }
        let nfull = j1 - j0 - 1.0;
        let first_end = self.lo + (j0 + 1.0) * period;
        let last_start = self.lo + j1 * period;
        let one = self.length_pieces(Interval::new(self.lo, self.hi), tol / nfull.max(1.0))?;
        Ok(self.length_pieces(Interval::new(range.lo, first_end), tol)? + nfull * one + self.length_pieces(Interval::new(last_start, range.hi), tol)?)
    }
}

impl PreparedCurve {
    /// Arc length over `range`, which touches at most two periods of a periodic curve: cuts at
    /// every knot value (collected, then sorted and deduplicated), Gauss quadrature per piece.
    fn length_pieces(&self, range: Interval, tol: f64) -> Result<f64, GeomError> {
        let per = self.lay.periodic;
        let period = self.hi - self.lo;
        let k0 = if per { ((range.lo - self.lo) / period).floor() as i64 } else { 0 };
        let k1 = if per { ((range.hi - self.lo) / period).floor() as i64 } else { 0 };
        let mut cuts: Vec<f64> = vec![range.lo, range.hi];
        for k in k0..=k1 {
            for s in &self.segs {
                for v in [s.lo, s.hi] {
                    let v = v + k as f64 * period;
                    if v > range.lo && v < range.hi {
                        cuts.push(v);
                    }
                }
            }
        }
        cuts.sort_by(f64::total_cmp);
        cuts.dedup();
        if cuts.len() < 2 {
            return Ok(0.0);
        }
        let pieces = (cuts.len() - 1) as f64;
        let mut total = 0.0;
        for w in cuts.windows(2) {
            total += project::gauss_adaptive(&|t| self.jet_at(self.fold(t)).d1.norm(), w[0], w[1], tol / pieces)?;
        }
        Ok(total)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rat_arc() -> NurbsCurve {
        let w = core::f64::consts::FRAC_1_SQRT_2;
        NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)], Some(vec![1.0, w, 1.0])).unwrap()
    }

    #[test]
    fn breakpoints_match_the_contract_curve() {
        assert!(PreparedCurve::new(&rat_arc()).unwrap().breakpoints().is_empty());
        let pts: Vec<Point3> = (0..6).map(|i| Point3::new(i as f64, (i * i) as f64, 0.0)).collect();
        let c = NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 0.25, 0.25, 0.75, 1.0, 1.0, 1.0], pts, None).unwrap();
        let pc = PreparedCurve::new(&c).unwrap();
        assert_eq!(pc.breakpoints(), vec![0.25, 0.75]);
        assert_eq!(pc.breakpoints(), c.breakpoints());
    }

    #[test]
    fn matches_reference_evaluator() {
        let c = rat_arc();
        let pc = PreparedCurve::new(&c).unwrap();
        for i in 0..=50 {
            let t = i as f64 / 50.0;
            let (a, b) = (c.jet(t), pc.jet(t));
            assert!((a.p - b.p).norm() < 1e-14 && (a.d1 - b.d1).norm() < 1e-13 && (a.d2 - b.d2).norm() < 1e-12);
        }
    }

    #[test]
    fn split_and_knot_ops_keep_the_shape() {
        let c = NurbsCurve::new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.5, 3.5, 3.5, 3.5],
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.5), Point3::new(3.0, 3.0, -1.0), Point3::new(4.0, 0.5, 2.0), Point3::new(6.0, 1.0, 0.0), Point3::new(7.0, 4.0, 1.0)],
            Some(vec![1.0, 1.4, 0.8, 2.0, 1.1, 1.0]),
        )
        .unwrap();
        let pc = PreparedCurve::new(&c).unwrap();
        let (l, r) = pc.split(1.7).unwrap();
        assert_eq!(l.range(), Interval::new(0.0, 1.7));
        assert_eq!(r.range(), Interval::new(1.7, 3.5));
        let refined = pc.refine(&[0.4, 0.4, 2.9]).unwrap();
        for i in 0..=60 {
            let t = 3.5 * i as f64 / 60.0;
            let want = pc.eval(t);
            assert!((refined.eval(t) - want).norm() < 1e-13);
            if t <= 1.7 {
                assert!((l.eval(t) - want).norm() < 1e-13, "left t={t}");
            }
            if t >= 1.7 {
                assert!((r.eval(t) - want).norm() < 1e-13, "right t={t}");
            }
        }
        // Defs round trip and validate.
        assert!(refined.to_def().unwrap().validate().is_ok());
        assert!(l.to_def().is_ok() && r.to_def().is_ok());
        assert!(pc.insert_knot(2.0, 3).is_err(), "multiplicity beyond the degree");
    }

    #[test]
    fn periodic_length_matches_brute_force_over_shifted_ranges() {
        let u = [Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.5, 0.2), Point3::new(3.0, 2.0, -0.3), Point3::new(1.5, 3.0, 0.4), Point3::new(-0.5, 2.0, 0.0), Point3::new(-1.0, 0.7, 0.1)];
        let c = NurbsCurve::new_periodic(3, (0..13).map(|i| i as f64 - 3.0).collect(), (0..9).map(|i| u[i % 6]).collect(), None).unwrap();
        let pc = PreparedCurve::new(&c).unwrap();
        let brute = |a: f64, b: f64| {
            let n = 40_000;
            let h = (b - a) / n as f64;
            (0..n).map(|i| pc.d1(a + (i as f64 + 0.5) * h).norm() * h).sum::<f64>()
        };
        for (a, b) in [(0.0, 6.0), (-0.7, 6.9), (2.5, 14.1), (-8.0, -1.25)] {
            let got = pc.length(Interval::new(a, b), 1e-9).unwrap();
            let want = brute(a, b);
            assert!((got - want).abs() < 1e-7, "[{a}, {b}]: {got} vs {want}");
        }
    }
}
