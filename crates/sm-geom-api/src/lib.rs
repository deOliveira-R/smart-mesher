//! Exact shape contract for smart-mesh.
//!
//! Layers of this crate:
//! - [`ParametricCurve`] / [`ParametricSurface`]: object-safe evaluation, derivatives,
//!   projection, periodicity, singular poles and a ray hook for tracking.
//! - [`analytic`]: line, circle, ellipse, plane, cylinder, cone, sphere, torus.
//! - [`nurbs`]: NURBS curves and surfaces as validated *data* (evaluation lives in `sm-geom`).
//! - [`implicit`]: signed quadrics and the quartic torus as CSG half-space providers.
//! - [`store`]: [`GeometryStore`], the GMI-like handle resolver used by topology.
//! - [`Tolerance`]: the one tolerance policy value shared by shape and topology.
//!
//! See `DESIGN.md` in this crate for the rationale.

pub mod analytic;
pub mod curve;
pub mod domain;
pub mod error;
pub mod frame;
pub mod implicit;
pub mod nurbs;
pub mod ray;
pub mod store;
pub mod surface;
pub mod tol;
mod util;

pub use analytic::{Circle, Cone, Cylinder, Ellipse, Line, Plane, Sphere, Torus};
pub use curve::{CurveJet, CurveProjection, ParametricCurve};
pub use domain::{Domain1, Domain2, Interval, ParamAxis, Singularity, Uv};
pub use error::GeomError;
pub use frame::Frame;
pub use implicit::{HalfSpace, ImplicitForm, Quadric, SignedSurface, Sign, TorusImplicit};
pub use nurbs::{NurbsCurve, NurbsSurface};
pub use ray::{Ray, RayHit};
pub use store::{CurveData, GeometryStore, MemoryStore, SurfaceData};
pub use surface::{ParametricSurface, SurfaceJet, SurfaceProjection};
pub use tol::Tolerance;
