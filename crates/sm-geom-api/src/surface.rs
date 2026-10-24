//! The [`ParametricSurface`] trait.

use crate::{Domain2, GeomError, ImplicitDef, Interval, Ray, RayHit, ImplicitSurface, Uv, util};
use sm_core::{Point3, Vector3};

/// Position and partial derivatives up to second order at one `(u, v)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceJet {
    /// `S(u,v)`.
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

/// Result of projecting a point onto a surface.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct SurfaceProjection {
    /// Parameters of the closest point (principal range for periodic parameters).
    pub uv: Uv,
    /// The closest point.
    pub point: Point3,
    /// Euclidean distance from the query point.
    pub distance: f64,
}

impl SurfaceProjection {
    /// Construct (the type is `#[non_exhaustive]`).
    pub const fn new(uv: Uv, point: Point3, distance: f64) -> Self {
        Self { uv, point, distance }
    }
}

/// An exact parametric surface `S: domain -> R^3`.
///
/// Object-safe. Conventions:
/// - Evaluation is infallible and continues analytically outside the domain.
/// - The surface normal is `du x dv` normalized. The *face* normal is this times the face's
///   `Sense` on its carrier (that is `sm-topo-api`'s job, not the surface's).
/// - Periodic parameters have a seam at the end of the principal range; singular iso-lines
///   (poles) are listed in [`Domain2::singularities`].
pub trait ParametricSurface: Send + Sync + core::fmt::Debug {
    /// Parameter domain, periodicity and poles.
    fn domain(&self) -> Domain2;

    /// Position and partials up to second order.
    fn jet(&self, uv: Uv) -> SurfaceJet;

    /// Position `S(u, v)`.
    fn eval(&self, uv: Uv) -> Point3 {
        self.jet(uv).p
    }

    /// Unit normal `du x dv / |du x dv|`. At a singular pole an analytic implementor returns
    /// the exact limit normal for the given `uv`; the default returns [`GeomError::Singular`].
    fn normal(&self, uv: Uv) -> Result<Vector3, GeomError> {
        let j = self.jet(uv);
        let c = j.du.cross(j.dv);
        let scale = j.du.norm().max(j.dv.norm());
        if c.norm() <= 1e-12 * scale * scale {
            return Err(GeomError::Singular("degenerate tangent plane"));
        }
        util::unit(c).ok_or(GeomError::Singular("degenerate tangent plane"))
    }

    /// Closest point on the surface (within the domain) to `p`; `hint` seeds iterative
    /// implementors. [`GeomError::Unsupported`] if the implementor cannot project.
    fn project(&self, p: Point3, hint: Option<Uv>) -> Result<SurfaceProjection, GeomError>;

    /// Implicit (signed) form if the surface is a quadric or torus. It is the "positive side =
    /// side the normal points to" function. Used for CSG half-spaces and the default ray hook.
    fn implicit(&self) -> Option<ImplicitDef> {
        None
    }

    /// Ray hook for tracking: append every crossing with `range.lo <= t <= range.hi` to `out`
    /// in ascending `t`. `out` is the caller's reusable buffer (no per-call allocation).
    /// The default intersects the [`Self::implicit`] form; implementors without one return
    /// [`GeomError::Unsupported`] and the tracker falls back to facets.
    fn ray_hits(&self, ray: &Ray, range: Interval, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        let Some(f) = self.implicit() else { return Err(GeomError::Unsupported("ray_hits")) };
        let first = out.len();
        f.ray_hits(ray, range, out)?;
        for h in &mut out[first..] {
            h.uv = self.project(h.point, None).ok().map(|p| p.uv);
        }
        Ok(())
    }
}
