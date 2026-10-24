//! The [`ParametricSurface`] trait.

use crate::roots::real_roots;
use crate::{Crossing, Domain2, GeomError, ImplicitDef, ImplicitSurface, Interval, ModelTolerance, Ray, RayHit, Uv, util};
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

    /// Evaluate at many parameters: `out[i] = eval(uv[i])`. Panics if the lengths differ.
    ///
    /// Contract: an override (for example a parallel batch evaluator) is bit-identical to this
    /// default loop for any thread count (determinism rule). Schemes holding a
    /// `&dyn ParametricSurface` call this instead of looping over `eval`.
    fn eval_many(&self, uv: &[Uv], out: &mut [Point3]) {
        assert_eq!(uv.len(), out.len(), "eval_many: length mismatch");
        for (o, q) in out.iter_mut().zip(uv) {
            *o = self.eval(*q);
        }
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

    /// [`Self::ray_hits`] plus near-tangent contacts. First calls `ray_hits`. Then, when the
    /// surface has an [`Self::implicit`] form, it takes each stationary point of the implicit
    /// along the ray (roots of the derivative of the ray polynomial, within `range`), projects
    /// it onto the surface, and if the distance is at most `tol.linear` and no hit of this
    /// call lies within `tol.linear` of the projected point, inserts one
    /// [`Crossing::Tangent`] hit (`point = ray.at(t)` at the stationary `t`, `normal` the unit
    /// implicit gradient at the projected point, `uv` the projection's) keeping `out` ascending
    /// in `t`. A line that misses by less than `tol.linear` thus counts as touching.
    /// Surfaces without an implicit form return exactly what `ray_hits` returns.
    fn ray_hits_tol(&self, ray: &Ray, range: Interval, tol: &ModelTolerance, out: &mut Vec<RayHit>) -> Result<(), GeomError> {
        let first = out.len();
        self.ray_hits(ray, range, out)?;
        let Some(f) = self.implicit() else { return Ok(()) };
        let poly = f.ray_poly(ray);
        let deriv: Vec<f64> = poly.iter().enumerate().skip(1).map(|(i, c)| c * i as f64).collect();
        for r in real_roots(&deriv, range.lo, range.hi) {
            let p = ray.at(r.t);
            let Ok(pr) = self.project(p, None) else { continue };
            if pr.distance > tol.linear || out[first..].iter().any(|h| (h.point - pr.point).norm() <= tol.linear) {
                continue;
            }
            let normal = util::unit(f.gradient(pr.point)).unwrap_or_default();
            let at = first + out[first..].partition_point(|h| h.t < r.t);
            out.insert(at, RayHit::new(r.t, p, normal, Some(pr.uv), Crossing::Tangent));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Cylinder, Frame, Sphere, Torus};

    fn xray(y: f64, z: f64) -> Ray {
        Ray { origin: Point3::new(-5.0, y, z), dir: Vector3::new(1.0, 0.0, 0.0) }
    }

    fn hits(s: &dyn ParametricSurface, ray: &Ray, tol: &ModelTolerance) -> Vec<RayHit> {
        let mut out = Vec::new();
        s.ray_hits_tol(ray, Interval::new(0.0, 10.0), tol, &mut out).unwrap();
        out
    }

    #[test]
    fn near_tangent_lines_give_one_tangent_hit_within_tolerance() {
        let tol = ModelTolerance::DEFAULT;
        let f = Frame::at(Point3::new(0.0, 0.0, 0.0));
        let sphere = Sphere::new(f, 1.0).unwrap();
        let cyl = Cylinder::new(f, 1.0).unwrap();
        // Line along x passing at gap `g` from the top (sphere) or the side (cylinder, axis z).
        for (s, y, z) in [(&sphere as &dyn ParametricSurface, 0.0f64, 1.0f64), (&cyl, 1.0, 0.0)] {
            let h = hits(s, &xray(y + 1e-9 * y, z + 1e-9 * z), &tol);
            assert_eq!(h.len(), 1, "{s:?}");
            assert_eq!(h[0].crossing, Crossing::Tangent);
            assert!((h[0].t - 5.0).abs() < 1e-6);
            assert!(h[0].uv.is_some());
            assert!(hits(s, &xray(y + 1e-3 * y, z + 1e-3 * z), &tol).is_empty());
        }
        // The plain hook is unchanged: it reports nothing for the 1e-9 miss.
        let mut plain = Vec::new();
        sphere.ray_hits(&xray(0.0, 1.0 + 1e-9), Interval::new(0.0, 10.0), &mut plain).unwrap();
        assert!(plain.is_empty());
        // Torus (R = 3, r = 1): the tube top at x = -3 and x = 3.
        let torus = Torus::new(f, 3.0, 1.0).unwrap();
        let h = hits(&torus, &xray(0.0, 1.0 + 1e-9), &tol);
        assert_eq!(h.len(), 2);
        assert!(h.iter().all(|h| h.crossing == Crossing::Tangent));
        assert!((h[0].t - 2.0).abs() < 1e-6 && (h[1].t - 8.0).abs() < 1e-6);
        assert!(hits(&torus, &xray(0.0, 1.0 + 1e-3), &tol).is_empty());
    }

    #[test]
    fn genuine_hits_are_not_duplicated_and_stay_sorted() {
        let tol = ModelTolerance::DEFAULT;
        let sphere = Sphere::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap();
        let h = hits(&sphere, &xray(0.0, 0.0), &tol);
        assert_eq!(h.len(), 2);
        assert!(h[0].t < h[1].t && h[0].crossing != Crossing::Tangent);
        // An exactly tangent line: ray_hits reports it, ray_hits_tol adds nothing.
        let h = hits(&sphere, &xray(0.0, 1.0), &tol);
        assert_eq!(h.len(), 1);
        // Existing entries in `out` are preserved and not considered.
        let mut out = vec![RayHit::new(-1.0, Point3::new(0.0, 0.0, 0.0), Vector3::default(), None, Crossing::Tangent)];
        sphere.ray_hits_tol(&xray(0.0, 1.0 + 1e-9), Interval::new(0.0, 10.0), &tol, &mut out).unwrap();
        assert_eq!(out.len(), 2);
        assert_eq!(out[0].t, -1.0);
    }

    #[test]
    fn eval_many_default_matches_eval() {
        let sphere = Sphere::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 2.0).unwrap();
        let uv: Vec<Uv> = (0..10).map(|i| Uv::new(0.3 * i as f64, 0.1 * i as f64 - 0.4)).collect();
        let mut out = vec![Point3::default(); uv.len()];
        sphere.eval_many(&uv, &mut out);
        assert!(uv.iter().zip(&out).all(|(q, p)| *p == sphere.eval(*q)));
    }

    #[test]
    #[should_panic(expected = "length mismatch")]
    fn eval_many_checks_lengths() {
        let sphere = Sphere::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 2.0).unwrap();
        sphere.eval_many(&[Uv::new(0.0, 0.0)], &mut []);
    }
}
