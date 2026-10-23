//! NURBS curves and surfaces as plain data with a reference evaluator
//! (Piegl & Tiller, *The NURBS Book*, A2.1/A2.3).
//!
//! Non-periodic only: closed NURBS are represented by coincident end poles, not by a
//! period. Weights are optional (`None` = all ones, i.e. a polynomial B-spline).

#![allow(clippy::needless_range_loop)] // index-heavy numeric kernels

use crate::curve::{CurveDerivs, CurveKind, CurveProjection, ParametricCurve, closest_generic};
use crate::error::GeomError;
use crate::surface::{Domain2, ParametricSurface, SingularLocus, Singularity, SurfaceDerivs, SurfaceKind, SurfaceProjection, project_generic};
use crate::tol::Interval;
use smallvec::SmallVec;
use sm_core::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Highest supported degree (keeps evaluation allocation-free).
pub const MAX_DEGREE: usize = 15;
const W: usize = MAX_DEGREE + 1;

fn find_span(n: usize, p: usize, u: f64, k: &[f64]) -> usize {
    if u >= k[n + 1] {
        return n;
    }
    if u <= k[p] {
        return p;
    }
    let (mut lo, mut hi) = (p, n + 1);
    let mut mid = (lo + hi) / 2;
    while u < k[mid] || u >= k[mid + 1] {
        if u < k[mid] { hi = mid } else { lo = mid }
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
        return Err(GeomError::Invalid("nurbs degree"));
    }
    if npoles < degree + 1 || knots.len() != npoles + degree + 1 {
        return Err(GeomError::Invalid("nurbs knot/pole count"));
    }
    if knots.iter().any(|k| !k.is_finite()) || knots.windows(2).any(|w| w[0] > w[1]) {
        return Err(GeomError::Invalid("nurbs knots must be finite and non-decreasing"));
    }
    if knots[degree] >= knots[npoles] {
        return Err(GeomError::Invalid("nurbs empty parameter range"));
    }
    Ok(())
}

fn check_weights(w: &Option<Vec<f64>>, n: usize) -> Result<(), GeomError> {
    match w {
        Some(w) if w.len() != n || w.iter().any(|x| !(*x > 0.0 && x.is_finite())) => Err(GeomError::Invalid("nurbs weights")),
        _ => Ok(()),
    }
}

/// Rational B-spline curve data.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct NurbsCurve {
    /// Polynomial degree (1..=[`MAX_DEGREE`]).
    pub degree: usize,
    /// Knot vector, length `poles.len() + degree + 1`, non-decreasing.
    pub knots: Vec<f64>,
    /// Control points.
    pub poles: Vec<Point3>,
    /// Positive weights, one per pole; `None` means all ones.
    pub weights: Option<Vec<f64>>,
}

impl NurbsCurve {
    /// Validate the data (counts, knot order, weights).
    pub fn validate(&self) -> Result<(), GeomError> {
        check_knots(&self.knots, self.poles.len(), self.degree)?;
        check_weights(&self.weights, self.poles.len())
    }

    /// Build and validate. Deserialized curves must be `validate()`d before evaluation.
    pub fn new(degree: usize, knots: Vec<f64>, poles: Vec<Point3>, weights: Option<Vec<f64>>) -> Result<Self, GeomError> {
        let c = Self { degree, knots, poles, weights };
        c.validate()?;
        Ok(c)
    }

    fn w(&self, i: usize) -> f64 {
        self.weights.as_ref().map_or(1.0, |w| w[i])
    }
}

impl ParametricCurve for NurbsCurve {
    fn kind(&self) -> CurveKind {
        CurveKind::Nurbs
    }
    fn range(&self) -> Interval {
        Interval::new(self.knots[self.degree], self.knots[self.poles.len()])
    }
    fn period(&self) -> Option<f64> {
        None
    }
    fn eval(&self, t: f64) -> Point3 {
        self.derivs(t).p
    }
    fn d1(&self, t: f64) -> Vector3 {
        self.derivs(t).d1
    }
    fn d2(&self, t: f64) -> Vector3 {
        self.derivs(t).d2
    }
    fn derivs(&self, t: f64) -> CurveDerivs {
        let p = self.degree;
        let r = self.range();
        let t = r.clamp(t);
        let span = find_span(self.poles.len() - 1, p, t, &self.knots);
        let d = ders_basis(span, t, p, &self.knots);
        let mut a = [Vector3::new(0.0, 0.0, 0.0); 3];
        let mut w = [0.0f64; 3];
        for (k, (ak, wk)) in a.iter_mut().zip(w.iter_mut()).enumerate() {
            for j in 0..=p {
                let i = span - p + j;
                let wi = self.w(i) * d[k][j];
                let q = self.poles[i];
                *ak = *ak + Vector3::new(q.x, q.y, q.z) * wi;
                *wk += wi;
            }
        }
        let c0 = a[0] * (1.0 / w[0]);
        let c1 = (a[1] - c0 * w[1]) * (1.0 / w[0]);
        let c2 = (a[2] - c1 * (2.0 * w[1]) - c0 * w[2]) * (1.0 / w[0]);
        CurveDerivs { p: Point3::new(c0.x, c0.y, c0.z), d1: c1, d2: c2 }
    }
    fn closest_in(&self, p: Point3, within: Interval) -> CurveProjection {
        let within = within.intersect(&self.range()).unwrap_or(self.range());
        closest_generic(self, p, within, 16 * self.poles.len())
    }
}

/// Rational B-spline surface data. Poles are row-major in `u`: pole `(i, j)` is
/// `poles[i * nv + j]`.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct NurbsSurface {
    /// Degree in `u`.
    pub degree_u: usize,
    /// Degree in `v`.
    pub degree_v: usize,
    /// Knots in `u`.
    pub knots_u: Vec<f64>,
    /// Knots in `v`.
    pub knots_v: Vec<f64>,
    /// Number of poles in `u`.
    pub nu: usize,
    /// Number of poles in `v`.
    pub nv: usize,
    /// Control net, `nu * nv` points.
    pub poles: Vec<Point3>,
    /// Positive weights (`nu * nv`) or `None`.
    pub weights: Option<Vec<f64>>,
}

impl NurbsSurface {
    /// Build and validate. Data obtained by deserialization must be passed through
    /// [`NurbsSurface::validate`] (or rebuilt with `new`) before evaluation.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
        nu: usize,
        nv: usize,
        poles: Vec<Point3>,
        weights: Option<Vec<f64>>,
    ) -> Result<Self, GeomError> {
        let s = Self { degree_u, degree_v, knots_u, knots_v, nu, nv, poles, weights };
        s.validate()?;
        Ok(s)
    }

    /// Validate the data.
    pub fn validate(&self) -> Result<(), GeomError> {
        check_knots(&self.knots_u, self.nu, self.degree_u)?;
        check_knots(&self.knots_v, self.nv, self.degree_v)?;
        if self.poles.len() != self.nu * self.nv {
            return Err(GeomError::Invalid("nurbs control net size"));
        }
        check_weights(&self.weights, self.poles.len())
    }

    fn w(&self, i: usize) -> f64 {
        self.weights.as_ref().map_or(1.0, |w| w[i])
    }
}

impl ParametricSurface for NurbsSurface {
    fn kind(&self) -> SurfaceKind {
        SurfaceKind::Nurbs
    }
    fn domain(&self) -> Domain2 {
        Domain2 {
            u: Interval::new(self.knots_u[self.degree_u], self.knots_u[self.nu]),
            v: Interval::new(self.knots_v[self.degree_v], self.knots_v[self.nv]),
        }
    }
    fn period_u(&self) -> Option<f64> {
        None
    }
    fn period_v(&self) -> Option<f64> {
        None
    }
    fn singularities(&self) -> SmallVec<[Singularity; 2]> {
        // Degenerate edges (all poles of a boundary row coincident) are reported as collapsed lines.
        let mut out = SmallVec::new();
        let d = self.domain();
        let same = |r: core::ops::Range<usize>| -> Option<Point3> {
            let first = self.poles[r.start];
            self.poles[r].iter().all(|q| *q == first).then_some(first)
        };
        if let Some(pt) = same(0..self.nv) {
            out.push(Singularity { locus: SingularLocus::U(d.u.lo), point: pt });
        }
        if let Some(pt) = same((self.nu - 1) * self.nv..self.nu * self.nv) {
            out.push(Singularity { locus: SingularLocus::U(d.u.hi), point: pt });
        }
        out
    }
    fn eval(&self, u: f64, v: f64) -> Point3 {
        self.derivs(u, v).p
    }
    fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3) {
        let d = self.derivs(u, v);
        (d.du, d.dv)
    }
    fn d2(&self, u: f64, v: f64) -> (Vector3, Vector3, Vector3) {
        let d = self.derivs(u, v);
        (d.duu, d.duv, d.dvv)
    }
    fn derivs(&self, u: f64, v: f64) -> SurfaceDerivs {
        let (pu, pv) = (self.degree_u, self.degree_v);
        let dom = self.domain();
        let (u, v) = (dom.u.clamp(u), dom.v.clamp(v));
        let su = find_span(self.nu - 1, pu, u, &self.knots_u);
        let sv = find_span(self.nv - 1, pv, v, &self.knots_v);
        let nu_d = ders_basis(su, u, pu, &self.knots_u);
        let nv_d = ders_basis(sv, v, pv, &self.knots_v);
        // a[k][l], w[k][l]: homogeneous partials.
        let mut a = [[Vector3::new(0.0, 0.0, 0.0); 3]; 3];
        let mut w = [[0.0f64; 3]; 3];
        for i in 0..=pu {
            for j in 0..=pv {
                let idx = (su - pu + i) * self.nv + (sv - pv + j);
                let q = self.poles[idx];
                let qv = Vector3::new(q.x, q.y, q.z);
                let wij = self.w(idx);
                for (k, (ak, wk)) in a.iter_mut().zip(w.iter_mut()).enumerate() {
                    for l in 0..3 {
                        let c = nu_d[k][i] * nv_d[l][j] * wij;
                        ak[l] = ak[l] + qv * c;
                        wk[l] += c;
                    }
                }
            }
        }
        let iw = 1.0 / w[0][0];
        let s = a[0][0] * iw;
        let su_ = (a[1][0] - s * w[1][0]) * iw;
        let sv_ = (a[0][1] - s * w[0][1]) * iw;
        let suu = (a[2][0] - su_ * (2.0 * w[1][0]) - s * w[2][0]) * iw;
        let svv = (a[0][2] - sv_ * (2.0 * w[0][1]) - s * w[0][2]) * iw;
        let suv = (a[1][1] - su_ * w[0][1] - sv_ * w[1][0] - s * w[1][1]) * iw;
        SurfaceDerivs { p: Point3::new(s.x, s.y, s.z), du: su_, dv: sv_, duu: suu, duv: suv, dvv: svv }
    }
    fn project(&self, p: Point3, hint: Option<(f64, f64)>) -> SurfaceProjection {
        project_generic(self, p, hint)
    }
}
