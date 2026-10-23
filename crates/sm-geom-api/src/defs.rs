//! Closed, serializable shape definitions ([`CurveDef`], [`SurfaceDef`]) that implement the
//! object-safe traits by delegation.

use crate::analytic_curve::{Circle, Ellipse, Line};
use crate::analytic_surface::{Cone, Cylinder, Plane, Sphere, Torus};
use crate::curve::{CurveDerivs, CurveKind, CurveProjection, ParametricCurve};
use crate::error::GeomError;
use crate::implicit::{ImplicitDef, ImplicitTorus, Quadric};
use crate::nurbs::{NurbsCurve, NurbsSurface};
use crate::ray::{Ray, RayHit, RayIntersect};
use crate::surface::ParametricSurface;
use crate::tol::{Interval, TolerancePolicy};
use smallvec::SmallVec;
use sm_core::{Point3, Transform, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A curve as data: what a geometry store holds and what a file stores.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[non_exhaustive]
pub enum CurveDef {
    /// Line.
    Line(Line),
    /// Circle.
    Circle(Circle),
    /// Ellipse.
    Ellipse(Ellipse),
    /// NURBS.
    Nurbs(NurbsCurve),
}

impl CurveDef {
    /// Borrow as the object-safe trait.
    pub fn as_parametric(&self) -> &dyn ParametricCurve {
        match self {
            Self::Line(c) => c,
            Self::Circle(c) => c,
            Self::Ellipse(c) => c,
            Self::Nurbs(c) => c,
        }
    }

    /// Image under a transform. Analytic kinds need a similarity; NURBS accept any affine map
    /// (affine invariance), including improper ones.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        Ok(match self {
            Self::Line(c) => Self::Line(c.transformed(t)?),
            Self::Circle(c) => Self::Circle(c.transformed(t)?),
            Self::Ellipse(c) => Self::Ellipse(c.transformed(t)?),
            Self::Nurbs(c) => {
                let mut n = c.clone();
                n.poles = c.poles.iter().map(|p| t.apply_point(*p)).collect();
                Self::Nurbs(n)
            }
        })
    }
}

impl ParametricCurve for CurveDef {
    fn kind(&self) -> CurveKind {
        self.as_parametric().kind()
    }
    fn range(&self) -> Interval {
        self.as_parametric().range()
    }
    fn period(&self) -> Option<f64> {
        self.as_parametric().period()
    }
    fn eval(&self, t: f64) -> Point3 {
        self.as_parametric().eval(t)
    }
    fn d1(&self, t: f64) -> Vector3 {
        self.as_parametric().d1(t)
    }
    fn d2(&self, t: f64) -> Vector3 {
        self.as_parametric().d2(t)
    }
    fn derivs(&self, t: f64) -> CurveDerivs {
        self.as_parametric().derivs(t)
    }
    fn closest_in(&self, p: Point3, within: Interval) -> CurveProjection {
        self.as_parametric().closest_in(p, within)
    }
    fn length(&self, a: f64, b: f64, tol: f64) -> f64 {
        self.as_parametric().length(a, b, tol)
    }
}

/// A surface as data. `Implicit` variants are CSG shape providers without a parametrization.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[non_exhaustive]
pub enum SurfaceDef {
    /// Plane.
    Plane(Plane),
    /// Cylinder.
    Cylinder(Cylinder),
    /// Cone.
    Cone(Cone),
    /// Sphere.
    Sphere(Sphere),
    /// Torus.
    Torus(Torus),
    /// NURBS.
    Nurbs(NurbsSurface),
    /// Implicit CSG surface (quadric / quartic torus), no `(u, v)`.
    Implicit(ImplicitDef),
}

impl SurfaceDef {
    /// Borrow as the object-safe parametric trait, if the surface has a parametrization.
    pub fn as_parametric(&self) -> Option<&dyn ParametricSurface> {
        match self {
            Self::Plane(s) => Some(s),
            Self::Cylinder(s) => Some(s),
            Self::Cone(s) => Some(s),
            Self::Sphere(s) => Some(s),
            Self::Torus(s) => Some(s),
            Self::Nurbs(s) => Some(s),
            Self::Implicit(_) => None,
        }
    }

    /// The signed implicit form `f(p)` (negative on the side opposite the parametric normal),
    /// when one exists in closed form. This is what CSG half-spaces and the tracker use.
    pub fn to_implicit(&self) -> Option<ImplicitDef> {
        match self {
            Self::Plane(s) => Quadric::plane(s.frame.origin, s.frame.x.cross(s.frame.y)).ok().map(ImplicitDef::Quadric),
            Self::Cylinder(s) => Quadric::cylinder(s.frame.origin, s.frame.z, s.radius).ok().map(ImplicitDef::Quadric),
            Self::Cone(s) => {
                let apex = s.frame.point(0.0, 0.0, s.apex_v() * s.semi_angle.cos());
                Quadric::cone(apex, s.frame.z, s.semi_angle).ok().map(ImplicitDef::Quadric)
            }
            Self::Sphere(s) => Quadric::sphere(s.frame.origin, s.radius).ok().map(ImplicitDef::Quadric),
            Self::Torus(s) => ImplicitTorus::new(s.frame, s.major, s.minor).ok().map(ImplicitDef::Torus),
            Self::Nurbs(_) => None,
            Self::Implicit(d) => Some(*d),
        }
    }

    /// Image under a transform. Analytic kinds need a similarity; NURBS accept any affine map.
    pub fn transformed(&self, t: &Transform) -> Result<Self, GeomError> {
        Ok(match self {
            Self::Plane(s) => Self::Plane(s.transformed(t)?),
            Self::Cylinder(s) => Self::Cylinder(s.transformed(t)?),
            Self::Cone(s) => Self::Cone(s.transformed(t)?),
            Self::Sphere(s) => Self::Sphere(s.transformed(t)?),
            Self::Torus(s) => Self::Torus(s.transformed(t)?),
            Self::Nurbs(s) => {
                let mut n = s.clone();
                n.poles = s.poles.iter().map(|p| t.apply_point(*p)).collect();
                Self::Nurbs(n)
            }
            Self::Implicit(d) => Self::Implicit(d.transformed(t)?),
        })
    }
}

impl RayIntersect for SurfaceDef {
    fn intersect_ray(&self, ray: &Ray, t_range: Interval, tol: &TolerancePolicy, hits: &mut SmallVec<[RayHit; 4]>) -> Result<(), GeomError> {
        let imp = self.to_implicit().ok_or(GeomError::Unsupported("exact ray hits on NURBS; use facets"))?;
        let start = hits.len();
        imp.intersect_ray(ray, t_range, tol, hits)?;
        if let Some(s) = self.as_parametric() {
            // Replace the gradient normal by the surface's own oriented normal and add (u, v).
            for h in &mut hits[start..] {
                let pr = s.project(h.point, None);
                h.uv = Some((pr.u, pr.v));
                if let Ok(n) = s.normal(pr.u, pr.v) {
                    h.normal = n;
                }
            }
        }
        Ok(())
    }
}
