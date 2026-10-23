//! The [`ParametricSurface`] contract.

use crate::analytic_surf::AnalyticSurface;
use crate::ray::RayIntersect;
use crate::vecs::{ParamRange, unit, wrap_near, wrap_to};
use crate::{GeomError, ModelTolerance};
use sm_core::{Point3, Vector3};
use smallvec::SmallVec;

/// Coarse surface classification, for scheme dispatch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum SurfaceKind {
    /// Plane.
    Plane,
    /// Cylinder.
    Cylinder,
    /// Cone (single nappe).
    Cone,
    /// Sphere.
    Sphere,
    /// Torus.
    Torus,
    /// NURBS.
    Nurbs,
    /// Anything else.
    Other,
}

/// Parameter domain of a surface, with periodic seams.
///
/// A periodic direction has `*_period = Some(p)` and a range of length exactly `p`; the two
/// boundary iso-curves of that direction are one geometric curve (the *seam*). Topology represents
/// a seam as an edge used twice by the same face.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SurfaceDomain {
    /// Range of `u`.
    pub u: ParamRange,
    /// Range of `v`.
    pub v: ParamRange,
    /// Period of `u`, if periodic.
    pub u_period: Option<f64>,
    /// Period of `v`, if periodic.
    pub v_period: Option<f64>,
}

/// A side of the rectangular parameter domain.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DomainEdge {
    /// `u = u.lo`.
    UMin,
    /// `u = u.hi`.
    UMax,
    /// `v = v.lo`.
    VMin,
    /// `v = v.hi`.
    VMax,
}

/// A singular pole: a whole domain edge that collapses to one point (sphere poles, cone apex).
/// First derivatives are linearly dependent there; `normal` returns the limit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pole {
    /// The collapsed domain edge.
    pub edge: DomainEdge,
    /// The 3D point it maps to.
    pub point: Point3,
}

/// Second partial derivatives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SecondDerivs {
    /// `S_uu`.
    pub uu: Vector3,
    /// `S_uv`.
    pub uv: Vector3,
    /// `S_vv`.
    pub vv: Vector3,
}

/// Result of inverse projection onto a surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceProjection {
    /// Parameters of the closest point (periodic directions wrapped near the hint, or into the domain).
    pub uv: [f64; 2],
    /// Closest point.
    pub point: Point3,
    /// Distance from the query point.
    pub distance: f64,
}

/// An exact surface `S(u, v)`. Object-safe: schemes take `&dyn ParametricSurface`.
///
/// Contract:
/// - The natural normal is `S_u x S_v` normalized; topology faces carry a `Sense` against it.
/// - `normal` is defined at poles by its limit when the limit exists.
/// - `project` returns parameters; periodic directions are wrapped to the branch nearest `hint`,
///   or into `[lo, lo+period)` without a hint.
/// - All methods are pure and deterministic.
pub trait ParametricSurface: Send + Sync + core::fmt::Debug {
    /// Coarse kind.
    fn kind(&self) -> SurfaceKind;
    /// Parameter domain incl. periodicity.
    fn domain(&self) -> SurfaceDomain;
    /// Singular poles (usually none, up to two).
    fn poles(&self) -> SmallVec<[Pole; 2]> {
        SmallVec::new()
    }
    /// Point `S(u, v)`.
    fn eval(&self, u: f64, v: f64) -> Point3;
    /// First partials `(S_u, S_v)`.
    fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3);
    /// Second partials.
    fn d2(&self, u: f64, v: f64) -> SecondDerivs;
    /// Unit normal. Default: `S_u x S_v` normalized; `Err(Singular)` where it vanishes.
    fn normal(&self, u: f64, v: f64) -> Result<Vector3, GeomError> {
        let (su, sv) = self.d1(u, v);
        let c = su.cross(sv);
        let scale = su.norm().max(sv.norm());
        if c.norm() <= 1e-12 * scale * scale {
            return Err(GeomError::Singular { u, v });
        }
        unit(c).ok_or(GeomError::Singular { u, v })
    }
    /// Closest point on the untrimmed surface to `p`.
    fn project(&self, p: Point3, hint: Option<[f64; 2]>, tol: &ModelTolerance) -> Result<SurfaceProjection, GeomError>;
    /// Principal curvatures `(k1, k2)` with `k1 >= k2` w.r.t. [`Self::normal`]. Default from the
    /// fundamental forms.
    fn principal_curvatures(&self, u: f64, v: f64) -> Result<(f64, f64), GeomError> {
        let (su, sv) = self.d1(u, v);
        let n = self.normal(u, v)?;
        let d = self.d2(u, v);
        let (e, f, g) = (su.dot(su), su.dot(sv), sv.dot(sv));
        let (l, m, nn) = (d.uu.dot(n), d.uv.dot(n), d.vv.dot(n));
        let det = e * g - f * f;
        if det <= 0.0 {
            return Err(GeomError::Singular { u, v });
        }
        let k = (l * nn - m * m) / det;
        let h = 0.5 * (e * nn - 2.0 * f * m + g * l) / det;
        let s = (h * h - k).max(0.0).sqrt();
        Ok((h + s, h - s))
    }
    /// Wrap `(u, v)` into the domain along periodic directions (near `hint` if given).
    fn wrap(&self, uv: [f64; 2], hint: Option<[f64; 2]>) -> [f64; 2] {
        let d = self.domain();
        let one = |x: f64, r: ParamRange, per: Option<f64>, h: Option<f64>| match (per, h) {
            (Some(p), Some(h)) => wrap_near(x, h, p),
            (Some(p), None) => wrap_to(x, r.lo, p),
            (None, _) => x,
        };
        [one(uv[0], d.u, d.u_period, hint.map(|h| h[0])), one(uv[1], d.v, d.v_period, hint.map(|h| h[1]))]
    }
    /// Ray intersection hook for tracking; `None` if the surface has no exact intersector (the
    /// tracker then falls back to facets).
    fn ray_hook(&self) -> Option<&dyn RayIntersect> {
        None
    }
    /// Analytic description if this is one of the analytic kinds.
    fn analytic(&self) -> Option<AnalyticSurface> {
        None
    }
}

/// Generic closest-point solver: damped Newton on the two orthogonality conditions from a fixed
/// grid of seeds (or the hint). Analytic kinds override with closed forms.
pub fn project_by_newton<S: ParametricSurface + ?Sized>(s: &S, p: Point3, hint: Option<[f64; 2]>, _tol: &ModelTolerance) -> Result<SurfaceProjection, GeomError> {
    let dom = s.domain();
    let span = |r: ParamRange, per: Option<f64>| if r.is_bounded() { r.len() } else { per.unwrap_or(1.0) };
    let mut seeds: Vec<[f64; 2]> = Vec::new();
    if let Some(h) = hint {
        seeds.push(h);
    } else {
        for i in 0..5 {
            for j in 0..5 {
                let fu = f64::from(i) / 4.0;
                let fv = f64::from(j) / 4.0;
                let u = if dom.u.is_bounded() { dom.u.lo + fu * span(dom.u, dom.u_period) } else { 0.0 };
                let v = if dom.v.is_bounded() { dom.v.lo + fv * span(dom.v, dom.v_period) } else { 0.0 };
                seeds.push([u, v]);
            }
        }
    }
    let mut best: Option<SurfaceProjection> = None;
    for seed in seeds {
        let [mut u, mut v] = seed;
        let mut conv = false;
        for _ in 0..80 {
            let r = s.eval(u, v) - p;
            let (su, sv) = s.d1(u, v);
            let d = s.d2(u, v);
            let (g0, g1) = (r.dot(su), r.dot(sv));
            let (a, b, c) = (su.dot(su) + r.dot(d.uu), su.dot(sv) + r.dot(d.uv), sv.dot(sv) + r.dot(d.vv));
            let det = a * c - b * b;
            if det.abs() < 1e-300 {
                break;
            }
            let (du, dv) = ((c * g0 - b * g1) / det, (a * g1 - b * g0) / det);
            let (mut un, mut vn) = (u - du, v - dv);
            if dom.u_period.is_none() {
                un = dom.u.clamp(un);
            }
            if dom.v_period.is_none() {
                vn = dom.v.clamp(vn);
            }
            let moved = (un - u).abs() + (vn - v).abs();
            (u, v) = (un, vn);
            if moved < 1e-14 * (1.0 + u.abs() + v.abs()) {
                conv = true;
                break;
            }
        }
        if !conv {
            continue;
        }
        let q = s.eval(u, v);
        let cand = SurfaceProjection { uv: s.wrap([u, v], hint), point: q, distance: (q - p).norm() };
        if best.is_none_or(|b| cand.distance < b.distance - 1e-15) {
            best = Some(cand);
        }
    }
    best.ok_or(GeomError::NoConvergence)
}
