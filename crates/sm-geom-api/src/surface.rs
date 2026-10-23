//! The [`ParametricSurface`] trait and generic numeric helpers.

use crate::error::GeomError;
use crate::frame::{dist, unit};
use crate::tol::Interval;
use core::fmt::Debug;
use smallvec::SmallVec;
use sm_core::{Point3, Vector3};

/// Closed set of surface kinds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum SurfaceKind {
    /// Plane.
    Plane,
    /// Right circular cylinder.
    Cylinder,
    /// Right circular cone.
    Cone,
    /// Sphere.
    Sphere,
    /// Torus (ring torus, `R > r`).
    Torus,
    /// Rational B-spline surface.
    Nurbs,
    /// Implicit-only (CSG quadric) surface without a parametrization.
    Implicit,
}

/// Rectangular parameter domain `[u.lo, u.hi] x [v.lo, v.hi]`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Domain2 {
    /// `u` range.
    pub u: Interval,
    /// `v` range.
    pub v: Interval,
}

/// A parameter line along which the map collapses to a single point (pole / apex).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Singularity {
    /// Which parameter is held constant, and its value: the whole line collapses.
    pub locus: SingularLocus,
    /// The point in space the line collapses to.
    pub point: Point3,
}

/// The collapsed parameter line of a [`Singularity`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SingularLocus {
    /// The line `u = const` collapses.
    U(f64),
    /// The line `v = const` collapses (sphere poles, cone apex).
    V(f64),
}

/// Point and derivatives up to second order.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceDerivs {
    /// `S(u, v)`.
    pub p: Point3,
    /// `dS/du`.
    pub du: Vector3,
    /// `dS/dv`.
    pub dv: Vector3,
    /// `d2S/du2`.
    pub duu: Vector3,
    /// `d2S/dudv`.
    pub duv: Vector3,
    /// `d2S/dv2`.
    pub dvv: Vector3,
}

/// Result of an inverse projection.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceProjection {
    /// Parameter `u` (periodic parameters are normalized into the domain).
    pub u: f64,
    /// Parameter `v`.
    pub v: f64,
    /// The surface point `S(u, v)`.
    pub point: Point3,
    /// Distance from the query point.
    pub distance: f64,
}

/// An exact parametric surface `S: D -> R^3`. Object-safe; `Send + Sync`.
///
/// Orientation: the surface normal is `S_u x S_v` (normalized). Topology stores a separate
/// sense per face so a face can use the reversed normal.
pub trait ParametricSurface: Send + Sync + Debug {
    /// Which kind of surface this is.
    fn kind(&self) -> SurfaceKind;
    /// Natural parameter domain (infinite where the surface is unbounded).
    fn domain(&self) -> Domain2;
    /// Period in `u`, if the surface wraps (seam at `u.lo`/`u.hi`).
    fn period_u(&self) -> Option<f64>;
    /// Period in `v`, if the surface wraps (torus).
    fn period_v(&self) -> Option<f64>;
    /// Collapsed parameter lines (poles, apexes). Empty for regular surfaces.
    fn singularities(&self) -> SmallVec<[Singularity; 2]>;
    /// `S(u, v)`.
    fn eval(&self, u: f64, v: f64) -> Point3;
    /// First partials `(S_u, S_v)`.
    fn d1(&self, u: f64, v: f64) -> (Vector3, Vector3);
    /// Second partials `(S_uu, S_uv, S_vv)`.
    fn d2(&self, u: f64, v: f64) -> (Vector3, Vector3, Vector3);

    /// Point and partials up to second order.
    fn derivs(&self, u: f64, v: f64) -> SurfaceDerivs {
        let (du, dv) = self.d1(u, v);
        let (duu, duv, dvv) = self.d2(u, v);
        SurfaceDerivs { p: self.eval(u, v), du, dv, duu, duv, dvv }
    }

    /// Unit normal `S_u x S_v / |.|`. Fails at singular points (poles, apex).
    fn normal(&self, u: f64, v: f64) -> Result<Vector3, GeomError> {
        let (du, dv) = self.d1(u, v);
        unit(du.cross(dv)).ok_or(GeomError::Singular("degenerate tangent plane"))
    }

    /// Inverse projection: closest surface point to `p`. `hint` is a nearby `(u, v)` used
    /// to pick a branch or the longitude at poles. Ties resolve deterministically.
    fn project(&self, p: Point3, hint: Option<(f64, f64)>) -> SurfaceProjection;
}

/// Generic closest-point by grid seeding + damped Newton on the 2x2 normal equations.
/// Needs a finite domain (infinite bounds are clamped to `+-1e6`).
pub fn project_generic<S: ParametricSurface + ?Sized>(s: &S, p: Point3, hint: Option<(f64, f64)>) -> SurfaceProjection {
    let d = s.domain();
    let (ulo, uhi) = (d.u.lo.max(-1e6), d.u.hi.min(1e6));
    let (vlo, vhi) = (d.v.lo.max(-1e6), d.v.hi.min(1e6));
    let mk = |u: f64, v: f64| {
        let q = s.eval(u, v);
        SurfaceProjection { u, v, point: q, distance: dist(q, p) }
    };
    let n = 24;
    let mut best = mk(ulo, vlo);
    for i in 0..=n {
        for j in 0..=n {
            let c = mk(ulo + (uhi - ulo) * i as f64 / n as f64, vlo + (vhi - vlo) * j as f64 / n as f64);
            if c.distance < best.distance {
                best = c;
            }
        }
    }
    if let Some((hu, hv)) = hint {
        let c = mk(hu.max(ulo).min(uhi), hv.max(vlo).min(vhi));
        if c.distance < best.distance {
            best = c;
        }
    }
    let (mut u, mut v) = (best.u, best.v);
    let mut cur = best;
    for _ in 0..80 {
        let k = s.derivs(u, v);
        let r = k.p - p;
        let (g1, g2) = (r.dot(k.du), r.dot(k.dv));
        let a = k.du.dot(k.du) + r.dot(k.duu);
        let b = k.du.dot(k.dv) + r.dot(k.duv);
        let c = k.dv.dot(k.dv) + r.dot(k.dvv);
        let det = a * c - b * b;
        if det.abs() < 1e-300 {
            break;
        }
        let (du, dv) = (-(c * g1 - b * g2) / det, -(a * g2 - b * g1) / det);
        let mut step = 1.0;
        let mut moved = false;
        for _ in 0..8 {
            let cand = mk((u + step * du).max(ulo).min(uhi), (v + step * dv).max(vlo).min(vhi));
            if cand.distance <= cur.distance {
                moved = (cand.u - u).abs() + (cand.v - v).abs() > 1e-15;
                (u, v, cur) = (cand.u, cand.v, cand);
                break;
            }
            step *= 0.5;
        }
        if !moved {
            break;
        }
    }
    cur
}
