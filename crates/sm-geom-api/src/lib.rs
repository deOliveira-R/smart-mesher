//! Exact-shape contract for smart-mesh (frozen once reviewed; orchestrator-owned).
//!
//! - [`ParametricCurve`] / [`ParametricSurface`]: object-safe traits for exact curves and surfaces.
//! - [`analytic`] and [`analytic_surf`]: line, circle, ellipse, plane, cylinder, cone, sphere, torus.
//! - [`nurbs`]: NURBS as data.
//! - [`csg`]: signed implicit surfaces (general quadric, plus the torus quartic) and half-spaces.
//! - [`ray`]: ray/surface intersection hook for tracking.
//! - [`ModelTolerance`]: the single global tolerance policy.
//! - [`predicates`]: exact `orient2d` / `orient3d`.
//! - [`ShapeStore`]: how topology resolves `Curve` / `Surface` handles to geometry.
//!
//! See `DESIGN.md` for the model and its tradeoffs.

pub mod analytic;
pub mod analytic_surf;
pub mod csg;
pub mod curve;
pub mod error;
pub mod nurbs;
pub mod pcurve;
pub mod predicates;
pub mod ray;
pub mod sense;
pub mod store;
pub mod surface;
pub mod tol;
pub mod vecs;

pub use analytic::{AnalyticCurve, Circle, Ellipse, Line};
pub use analytic_surf::{AnalyticSurface, Cone, Cylinder, Plane, Sphere, Torus};
pub use csg::{Containment, HalfSpace, ImplicitSurface, Quadric, Sidedness};
pub use curve::{CurveKind, CurveProjection, ParametricCurve};
pub use error::GeomError;
pub use nurbs::{NurbsCurve, NurbsSurface};
pub use pcurve::PCurve;
pub use predicates::{Sign, orient2d, orient3d};
pub use ray::{NormalCrossing, Ray, RayHit, RayIntersect};
pub use sense::Sense;
pub use store::{ShapeLibrary, ShapeStore};
pub use surface::{DomainEdge, ParametricSurface, Pole, SecondDerivs, SurfaceDomain, SurfaceKind, SurfaceProjection};
pub use tol::ModelTolerance;
pub use vecs::{Frame, ParamRange};

#[cfg(test)]
mod tests;
