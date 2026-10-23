//! NURBS curves and surfaces as validated data with a small reference evaluator
//! (de Boor / Piegl & Tiller A2.1, A2.3) and a numeric closest-point projector.
//!
//! Non-periodic only: closed NURBS are represented by coincident end control points, not by a
//! period. Weights are optional (`None` = all ones, a polynomial B-spline). The evaluator is
//! deliberately small and allocation-free (degree is capped at [`MAX_DEGREE`]); `sm-geom` may
//! provide faster implementations behind the same traits.

#![allow(clippy::needless_range_loop)] // index-heavy numeric kernels

use crate::{CurveJet, CurveProjection, Domain1, Domain2, GeomError, Interval, ParamAxis, ParametricCurve, ParametricSurface, Singularity, SurfaceJet, SurfaceProjection, Uv};
use sm_core::{Point3, Vector3};
use smallvec::SmallVec;

/// Highest supported degree (keeps evaluation allocation-free).
pub const MAX_DEGREE: usize = 15;
const W: usize = MAX_DEGREE + 1;

fn find_span(n: usize, p: usize, u: f64, k: &[f64]) -> usize {
    if u >= k[n + 1] {
        // Last non-empty span.
        let mut s = n;
        while s > p && k[s] >= k[n + 1] {
            s -= 1;
        }
        return s;
    }
    if u <= k[p] {
        return p;
    }
    let (mut lo, mut hi) = (p, n + 1);
    let mut mid = (lo + hi) / 2;
    while u < k[mid] || u >= k[mid + 1] {
        if u < k[mid] {
            hi = mid
        } else {
            lo = mid
        }
        mid = (lo + hi) / 2;
    }
    mid
}

/// Basis functions and first two derivatives: `ders[k][j]` = k-th derivative of `N_{span-p+j}`.
fn ders_basis(span: usize, u: f64, p: usize, k: &[f64]) -> [[f64; W]; 3] {
    let mut ndu = [[0.0f64; W]; W];
    let (mut left, mut right) = ([0.0f64; W], [0.0f64; W]);
    ndu[0][0] = 1.0;
    for j in 1..=p {
        left[j] = u - k[span + 1 - j];
        right[j] = k[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }
    let mut ders = [[0.0f64; W]; 3];
    for j in 0..=p {
        ders[0][j] = ndu[j][p];
    }
    let nd = 2.min(p);
    let mut a = [[0.0f64; W]; 2];
    for r in 0..=p {
        let (mut s1, mut s2) = (0usize, 1usize);
        a[0][0] = 1.0;
        for kk in 1..=nd {
            let mut d = 0.0;
            let rk = r as i64 - kk as i64;
            let pk = (p - kk) as i64;
            if rk >= 0 {
                a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                d = a[s2][0] * ndu[rk as usize][pk as usize];
            }
            let j1 = if rk >= -1 { 1 } else { -rk };
            let j2 = if r as i64 - 1 <= pk { kk as i64 - 1 } else { (p - r) as i64 };
            for j in j1..=j2 {
                let ju = j as usize;
                let idx = (rk + j) as usize;
                a[s2][ju] = (a[s1][ju] - a[s1][ju - 1]) / ndu[(pk + 1) as usize][idx];
                d += a[s2][ju] * ndu[idx][pk as usize];
            }
            if r as i64 <= pk {
                a[s2][kk] = -a[s1][kk - 1] / ndu[(pk + 1) as usize][r];
                d += a[s2][kk] * ndu[r][pk as usize];
            }
            ders[kk][r] = d;
            core::mem::swap(&mut s1, &mut s2);
        }
    }
    let mut f = p as f64;
    for kk in 1..=nd {
        for j in 0..=p {
            ders[kk][j] *= f;
        }
        f *= (p - kk) as f64;
    }
    ders
}

fn check_knots(knots: &[f64], npoles: usize, degree: usize) -> Result<(), GeomError> {
    if degree == 0 || degree > MAX_DEGREE {
        return Err(GeomError::Invalid("nurbs degree must be in 1..=15"));
    }
    if npoles < degree + 1 || knots.len() != npoles + degree + 1 {
        return Err(GeomError::Invalid("knot count must be control points + degree + 1"));
    }
    if knots.iter().any(|k| !k.is_finite()) || knots.windows(2).any(|w| w[0] > w[1]) {
        return Err(GeomError::Invalid("knots must be finite and non-decreasing"));
    }
    let mut run = 1;
    for w in knots.windows(2) {
        run = if w[0] == w[1] { run + 1 } else { 1 };
        if run > degree + 1 {
            return Err(GeomError::Invalid("knot multiplicity exceeds degree + 1"));
        }
    }
    if knots[degree] >= knots[npoles] {
        return Err(GeomError::Invalid("empty parameter range"));
    }
    Ok(())
}

fn check_weights(weights: &Option<Vec<f64>>, n: usize) -> Result<(), GeomError> {
    if let Some(w) = weights {
        if w.len() != n {
            return Err(GeomError::Invalid("weight count must equal control point count"));
        }
        if w.iter().any(|v| !(v.is_finite() && *v > 0.0)) {
            return Err(GeomError::Invalid("weights must be positive"));
        }
    }
    Ok(())
}

fn vec3(p: Point3) -> Vector3 {
    Vector3::new(p.x, p.y, p.z)
}

/// A NURBS curve: degree, knot vector, control points, optional weights (absent = polynomial B-spline).
///
/// The fields are public for reading and serialization. Construct with `new()` (or add through a
/// `GeomAppend` store, which validates): evaluating an unvalidated value may index out of bounds.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct NurbsCurve {
    /// Polynomial degree `p` in `1..=15`.
    pub degree: usize,
    /// Knots, length `control.len() + degree + 1`, non-decreasing.
    pub knots: Vec<f64>,
    /// Control points.
    pub control: Vec<Point3>,
    /// Positive weights, one per control point; `None` for non-rational.
    pub weights: Option<Vec<f64>>,
}

impl NurbsCurve {
    /// Build and validate.
    pub fn new(degree: usize, knots: Vec<f64>, control: Vec<Point3>, weights: Option<Vec<f64>>) -> Result<Self, GeomError> {
        let c = Self { degree, knots, control, weights };
        c.validate()?;
        Ok(c)
    }

    /// Validate the data (counts, knot order and multiplicity, positive weights). Data obtained
    /// by deserialization must be validated before evaluation.
    pub fn validate(&self) -> Result<(), GeomError> {
        check_knots(&self.knots, self.control.len(), self.degree)?;
        check_weights(&self.weights, self.control.len())
    }

    /// The valid parameter range `[knots[p], knots[n]]`.
    pub fn range(&self) -> Interval {
        Interval::new(self.knots[self.degree], self.knots[self.control.len()])
    }

    fn w(&self, i: usize) -> f64 {
        self.weights.as_ref().map_or(1.0, |w| w[i])
    }
}

impl ParametricCurve for NurbsCurve {
    fn domain(&self) -> Domain1 {
        let r = self.range();
        Domain1::open(r.lo, r.hi)
    }

    fn jet(&self, t: f64) -> CurveJet {
        let p = self.degree;
        let t = self.range().clamp(t);
        let span = find_span(self.control.len() - 1, p, t, &self.knots);
        let d = ders_basis(span, t, p, &self.knots);
        let mut a = [Vector3::default(); 3];
        let mut w = [0.0f64; 3];
        for (k, (ak, wk)) in a.iter_mut().zip(w.iter_mut()).enumerate() {
            for j in 0..=p {
                let i = span - p + j;
                let wi = self.w(i) * d[k][j];
                *ak = *ak + vec3(self.control[i]) * wi;
                *wk += wi;
            }
        }
        let c0 = a[0] * (1.0 / w[0]);
        let c1 = (a[1] - c0 * w[1]) * (1.0 / w[0]);
        let c2 = (a[2] - c1 * (2.0 * w[1]) - c0 * w[2]) * (1.0 / w[0]);
        CurveJet { p: Point3::new(c0.x, c0.y, c0.z), d1: c1, d2: c2 }
    }

    fn project(&self, p: Point3, hint: Option<f64>) -> Result<CurveProjection, GeomError> {
        Ok(project_curve_numeric(self, p, hint, 16 * self.control.len()))
    }
}

/// Closest point on any bounded curve by dense sampling followed by safeguarded Newton.
pub(crate) fn project_curve_numeric<C: ParametricCurve + ?Sized>(c: &C, p: Point3, hint: Option<f64>, samples: usize) -> CurveProjection {
    let r = c.domain().range;
    let n = samples.max(8);
    let mut best_t = r.lo;
    let mut best_d = f64::INFINITY;
    let consider = |t: f64, best_t: &mut f64, best_d: &mut f64| {
        let d = (c.eval(t) - p).norm();
        if d < *best_d {
            *best_d = d;
            *best_t = t;
        }
    };
    for i in 0..=n {
        let t = r.lo + r.len() * (i as f64 / n as f64);
        consider(t, &mut best_t, &mut best_d);
    }
    if let Some(h) = hint {
        consider(r.clamp(h), &mut best_t, &mut best_d);
    }
    let mut t = best_t;
    for _ in 0..40 {
        let j = c.jet(t);
        let diff = j.p - p;
        let g = diff.dot(j.d1);
        let h = j.d1.dot(j.d1) + diff.dot(j.d2);
        if h.abs() < 1e-300 {
            break;
        }
        let nt = r.clamp(t - g / h);
        if (nt - t).abs() <= 1e-15 * (1.0 + t.abs()) {
            t = nt;
            break;
        }
        // Only accept steps that do not increase the distance.
        if (c.eval(nt) - p).norm() <= (c.eval(t) - p).norm() {
            t = nt;
        } else {
            break;
        }
    }
    if (c.eval(t) - p).norm() > best_d {
        t = best_t;
    }
    let point = c.eval(t);
    CurveProjection { t, point, distance: (point - p).norm() }
}

/// A 2D NURBS curve in a surface's parameter space (used by [`crate::PCurve`]).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct NurbsCurve2 {
    /// Polynomial degree in `1..=15`.
    pub degree: usize,
    /// Knots, length `control.len() + degree + 1`.
    pub knots: Vec<f64>,
    /// Control points in `(u, v)`.
    pub control: Vec<Uv>,
    /// Positive weights or `None`.
    pub weights: Option<Vec<f64>>,
}

impl NurbsCurve2 {
    /// Validate the data.
    pub fn validate(&self) -> Result<(), GeomError> {
        check_knots(&self.knots, self.control.len(), self.degree)?;
        check_weights(&self.weights, self.control.len())
    }

    fn as_3d(&self) -> NurbsCurve {
        NurbsCurve { degree: self.degree, knots: self.knots.clone(), control: self.control.iter().map(|c| Point3::new(c.u, c.v, 0.0)).collect(), weights: self.weights.clone() }
    }

    /// Parameter range.
    pub fn range(&self) -> Interval {
        Interval::new(self.knots[self.degree], self.knots[self.control.len()])
    }

    /// Point and first derivative at `t` (clamped to the range).
    pub fn eval_d1(&self, t: f64) -> (Uv, Uv) {
        let j = self.as_3d().jet(t);
        (Uv::new(j.p.x, j.p.y), Uv::new(j.d1.x, j.d1.y))
    }
}

/// A tensor-product NURBS surface; control points are stored `u`-major: index `iu * nv + iv`.
///
/// The fields are public for reading and serialization. Call `validate()` (a `GeomAppend` store
/// does) before evaluating: an unvalidated value may index out of bounds.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub struct NurbsSurface {
    /// Degree in `u`.
    pub degree_u: usize,
    /// Degree in `v`.
    pub degree_v: usize,
    /// Knots in `u`, length `nu + degree_u + 1`.
    pub knots_u: Vec<f64>,
    /// Knots in `v`, length `nv + degree_v + 1`.
    pub knots_v: Vec<f64>,
    /// Number of control points in `u`.
    pub nu: usize,
    /// Number of control points in `v`.
    pub nv: usize,
    /// `nu * nv` control points, `u`-major.
    pub control: Vec<Point3>,
    /// Positive weights (same layout) or `None`.
    pub weights: Option<Vec<f64>>,
}

impl NurbsSurface {
    /// Validate the data.
    pub fn validate(&self) -> Result<(), GeomError> {
        check_knots(&self.knots_u, self.nu, self.degree_u)?;
        check_knots(&self.knots_v, self.nv, self.degree_v)?;
        if self.control.len() != self.nu * self.nv {
            return Err(GeomError::Invalid("control net must be nu * nv"));
        }
        check_weights(&self.weights, self.control.len())
    }

    fn w(&self, i: usize) -> f64 {
        self.weights.as_ref().map_or(1.0, |w| w[i])
    }

    fn param_domain(&self) -> (Interval, Interval) {
        (Interval::new(self.knots_u[self.degree_u], self.knots_u[self.nu]), Interval::new(self.knots_v[self.degree_v], self.knots_v[self.nv]))
    }
}

impl ParametricSurface for NurbsSurface {
    fn domain(&self) -> Domain2 {
        let (du, dv) = self.param_domain();
        let mut d = Domain2::regular(Domain1::open(du.lo, du.hi), Domain1::open(dv.lo, dv.hi));
        // A boundary row of coincident control points collapses to a pole.
        let same = |r: core::ops::Range<usize>| -> Option<Point3> {
            let first = self.control[r.start];
            self.control[r].iter().all(|q| *q == first).then_some(first)
        };
        let mut s: SmallVec<[Singularity; 2]> = SmallVec::new();
        if let Some(point) = same(0..self.nv) {
            s.push(Singularity { along: ParamAxis::U, at: du.lo, point });
        }
        if let Some(point) = same((self.nu - 1) * self.nv..self.nu * self.nv) {
            s.push(Singularity { along: ParamAxis::U, at: du.hi, point });
        }
        d.singularities = s;
        d
    }

    fn jet(&self, uv: Uv) -> SurfaceJet {
        let (pu, pv) = (self.degree_u, self.degree_v);
        let (du, dv) = self.param_domain();
        let (u, v) = (du.clamp(uv.u), dv.clamp(uv.v));
        let su = find_span(self.nu - 1, pu, u, &self.knots_u);
        let sv = find_span(self.nv - 1, pv, v, &self.knots_v);
        let nu_d = ders_basis(su, u, pu, &self.knots_u);
        let nv_d = ders_basis(sv, v, pv, &self.knots_v);
        let mut a = [[Vector3::default(); 3]; 3];
        let mut w = [[0.0f64; 3]; 3];
        for i in 0..=pu {
            for j in 0..=pv {
                let idx = (su - pu + i) * self.nv + (sv - pv + j);
                let qv = vec3(self.control[idx]);
                let wij = self.w(idx);
                for k in 0..3 {
                    for l in 0..3 {
                        let c = nu_d[k][i] * nv_d[l][j] * wij;
                        a[k][l] = a[k][l] + qv * c;
                        w[k][l] += c;
                    }
                }
            }
        }
        let iw = 1.0 / w[0][0];
        let s = a[0][0] * iw;
        let s_u = (a[1][0] - s * w[1][0]) * iw;
        let s_v = (a[0][1] - s * w[0][1]) * iw;
        let s_uu = (a[2][0] - s_u * (2.0 * w[1][0]) - s * w[2][0]) * iw;
        let s_vv = (a[0][2] - s_v * (2.0 * w[0][1]) - s * w[0][2]) * iw;
        let s_uv = (a[1][1] - s_u * w[0][1] - s_v * w[1][0] - s * w[1][1]) * iw;
        SurfaceJet { p: Point3::new(s.x, s.y, s.z), du: s_u, dv: s_v, duu: s_uu, duv: s_uv, dvv: s_vv }
    }

    fn project(&self, p: Point3, hint: Option<Uv>) -> Result<SurfaceProjection, GeomError> {
        let (du, dv) = self.param_domain();
        let (gu, gv) = ((8 * self.nu).max(8), (8 * self.nv).max(8));
        let mut best = (du.lo, dv.lo);
        let mut best_d = f64::INFINITY;
        let consider = |u: f64, v: f64, best: &mut (f64, f64), best_d: &mut f64| {
            let d = (self.jet(Uv::new(u, v)).p - p).norm();
            if d < *best_d {
                *best_d = d;
                *best = (u, v);
            }
        };
        for i in 0..=gu {
            for j in 0..=gv {
                consider(du.lo + du.len() * i as f64 / gu as f64, dv.lo + dv.len() * j as f64 / gv as f64, &mut best, &mut best_d);
            }
        }
        if let Some(h) = hint {
            consider(du.clamp(h.u), dv.clamp(h.v), &mut best, &mut best_d);
        }
        let (mut u, mut v) = best;
        for _ in 0..60 {
            let j = self.jet(Uv::new(u, v));
            let r = j.p - p;
            let (g0, g1) = (r.dot(j.du), r.dot(j.dv));
            let h00 = j.du.dot(j.du) + r.dot(j.duu);
            let h01 = j.du.dot(j.dv) + r.dot(j.duv);
            let h11 = j.dv.dot(j.dv) + r.dot(j.dvv);
            let det = h00 * h11 - h01 * h01;
            if det.abs() < 1e-300 {
                break;
            }
            let (su, sv) = ((h11 * g0 - h01 * g1) / det, (h00 * g1 - h01 * g0) / det);
            let (nu_, nv_) = (du.clamp(u - su), dv.clamp(v - sv));
            let moved = (nu_ - u).abs() + (nv_ - v).abs();
            if (self.jet(Uv::new(nu_, nv_)).p - p).norm() > r.norm() {
                break;
            }
            u = nu_;
            v = nv_;
            if moved <= 1e-15 * (1.0 + u.abs() + v.abs()) {
                break;
            }
        }
        let point = self.jet(Uv::new(u, v)).p;
        if (point - p).norm() > best_d {
            (u, v) = best;
        }
        let point = self.jet(Uv::new(u, v)).p;
        Ok(SurfaceProjection { uv: Uv::new(u, v), point, distance: (point - p).norm() })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line() -> NurbsCurve {
        NurbsCurve { degree: 1, knots: vec![0.0, 0.0, 1.0, 1.0], control: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)], weights: None }
    }

    /// Exact rational quarter circle of radius 1 (degree 2, weights 1, sqrt2/2, 1).
    fn quarter_circle() -> NurbsCurve {
        let w = core::f64::consts::FRAC_1_SQRT_2;
        NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)], Some(vec![1.0, w, 1.0])).unwrap()
    }

    #[test]
    fn validation() {
        let mut c = line();
        assert!(c.validate().is_ok());
        assert_eq!(c.range(), Interval::new(0.0, 1.0));
        c.knots = vec![0.0, 1.0, 0.5, 1.0];
        assert!(c.validate().is_err());
        let mut c = line();
        c.weights = Some(vec![1.0, -1.0]);
        assert!(c.validate().is_err());
        let mut c = line();
        c.knots = vec![0.0, 0.0, 0.0, 1.0];
        assert!(c.validate().is_err(), "multiplicity 3 > degree + 1");
    }

    #[test]
    fn surface_validation() {
        let s = NurbsSurface {
            degree_u: 1,
            degree_v: 1,
            knots_u: vec![0.0, 0.0, 1.0, 1.0],
            knots_v: vec![0.0, 0.0, 1.0, 1.0],
            nu: 2,
            nv: 2,
            control: vec![Point3::default(); 4],
            weights: None,
        };
        assert!(s.validate().is_ok());
        let bad = NurbsSurface { control: vec![Point3::default(); 3], ..s };
        assert!(bad.validate().is_err());
    }

    #[test]
    fn rational_quarter_circle_is_exact() {
        let c = quarter_circle();
        for k in 0..=20 {
            let t = k as f64 / 20.0;
            let j = c.jet(t);
            // Points of the rational quadratic lie exactly on the unit circle; the speed is
            // positive and the second derivative points inwards.
            assert!((j.p.x.hypot(j.p.y) - 1.0).abs() < 1e-14, "t={t}");
            assert!(j.d1.norm() > 0.0);
            assert!(j.d2.dot(vec3(j.p)) < 0.0);
        }
        // Total arc length is pi/2.
        assert!((c.length(Interval::new(0.0, 1.0), 1e-10).unwrap() - core::f64::consts::FRAC_PI_2).abs() < 1e-9);
        // Derivatives against central differences.
        let (t, h) = (0.37, 1e-6);
        let fd = (c.eval(t + h) - c.eval(t - h)) * (1.0 / (2.0 * h));
        assert!((fd - c.d1(t)).norm() < 1e-8);
        let fd2 = (c.d1(t + h) - c.d1(t - h)) * (1.0 / (2.0 * h));
        assert!((fd2 - c.jet(t).d2).norm() < 1e-6);
    }

    #[test]
    fn curve_projection_is_closest() {
        let c = quarter_circle();
        let q = Point3::new(2.0, 2.0, 0.5);
        let pr = c.project(q, None).unwrap();
        // Closest point of the arc to (2,2,.) is at 45 degrees.
        assert!((pr.point.x - core::f64::consts::FRAC_1_SQRT_2).abs() < 1e-7 && (pr.point.y - core::f64::consts::FRAC_1_SQRT_2).abs() < 1e-7);
        assert!((pr.distance - (2.0f64 * (2.0 - core::f64::consts::FRAC_1_SQRT_2).powi(2) + 0.25).sqrt()).abs() < 1e-7);
    }

    #[test]
    fn bilinear_surface_eval_and_projection() {
        let s = NurbsSurface {
            degree_u: 1,
            degree_v: 1,
            knots_u: vec![0.0, 0.0, 1.0, 1.0],
            knots_v: vec![0.0, 0.0, 1.0, 1.0],
            nu: 2,
            nv: 2,
            control: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 2.0, 0.0), Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0)],
            weights: None,
        };
        let j = s.jet(Uv::new(0.25, 0.5));
        assert!((j.p - Point3::new(0.5, 1.0, 0.0)).norm() < 1e-14);
        assert!((j.du - Vector3::new(2.0, 0.0, 0.0)).norm() < 1e-14 && (j.dv - Vector3::new(0.0, 2.0, 0.0)).norm() < 1e-14);
        let n = s.normal(Uv::new(0.5, 0.5)).unwrap();
        assert!((n - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-14);
        let pr = s.project(Point3::new(0.5, 1.5, 3.0), None).unwrap();
        assert!((pr.distance - 3.0).abs() < 1e-9 && (pr.point - Point3::new(0.5, 1.5, 0.0)).norm() < 1e-9);
        // Last-knot evaluation stays on the surface.
        assert!((s.jet(Uv::new(1.0, 1.0)).p - Point3::new(2.0, 2.0, 0.0)).norm() < 1e-14);
    }
}
