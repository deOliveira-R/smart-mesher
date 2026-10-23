//! Exact-shape contract for smart-mesh.
//!
//! - [`ParametricCurve`] / [`ParametricSurface`]: object-safe evaluation, derivatives, periodicity,
//!   singularities and projection.
//! - Analytic kinds ([`Line`], [`Circle`], [`Ellipse`], [`Plane`], [`Cylinder`], [`Cone`], [`Sphere`],
//!   [`Torus`]) and NURBS data ([`NurbsCurve`], [`NurbsSurface`]) with a reference evaluator.
//! - CSG: [`Quadric`], [`ImplicitTorus`], [`HalfSpace`] behind [`ImplicitSurface`].
//! - [`RayIntersect`] hook for tracking; [`TolerancePolicy`]; [`GeomStore`] handles for topology.
//!
//! See `DESIGN.md` in this crate for the model and tradeoffs.

pub mod analytic_curve;
pub mod analytic_surface;
pub mod curve;
pub mod defs;
pub mod error;
pub mod frame;
pub mod implicit;
pub mod nurbs;
pub mod pcurve;
pub mod ray;
pub mod store;
pub mod surface;
pub mod tol;

pub use analytic_curve::{Circle, Ellipse, Line};
pub use analytic_surface::{Cone, Cylinder, Plane, Sphere, Torus};
pub use curve::{CurveDerivs, CurveKind, CurveProjection, ParametricCurve};
pub use defs::{CurveDef, SurfaceDef};
pub use error::GeomError;
pub use frame::Frame;
pub use implicit::{HalfSpace, ImplicitDef, ImplicitSurface, ImplicitTorus, Quadric, Sidedness};
pub use nurbs::{NurbsCurve, NurbsSurface};
pub use pcurve::{Line2, PCurve};
pub use ray::{Ray, RayHit, RayIntersect};
pub use store::{CurveId, GeomStore, SimpleGeomStore, SurfaceId};
pub use surface::{Domain2, ParametricSurface, SingularLocus, Singularity, SurfaceDerivs, SurfaceKind, SurfaceProjection};
pub use tol::{Interval, TolerancePolicy};
