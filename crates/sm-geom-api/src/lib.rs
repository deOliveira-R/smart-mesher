//! Exact shape contract for smart-mesh (frozen after W0c).
//!
//! Layers of this crate:
//! - [`ParametricCurve`] / [`ParametricSurface`] / [`ImplicitSurface`]: object-safe evaluation,
//!   derivatives, projection, periodicity, singular poles and a ray hook for tracking.
//! - [`analytic`]: line, circle, ellipse, plane, cylinder, cone, sphere, torus.
//! - [`elliptic`]: elliptic cylinder and elliptic cone (affine images of the circular kinds).
//! - [`nurbs`]: NURBS curves and surfaces with a small reference de Boor evaluator.
//! - [`implicit`]: signed quadrics (incl. general quadrics), the quartic torus, [`HalfSpace`] and
//!   [`CsgCell`].
//! - [`pcurve`]: [`PCurve`], the curve of a coedge in its face's parameter space.
//! - [`store`]: closed serializable [`CurveDef`] / [`SurfaceDef`] and [`GeomStore`] /
//!   [`GeomStoreMut`], the handle resolver used by topology.
//! - [`roots`]: the shared real-root finders (stable quadratic, interval polynomial roots).
//! - [`ModelTolerance`]: the one tolerance policy and the cross-platform hash quantization rule.
//!
//! See `DESIGN.md` in this crate for the rationale.

pub mod analytic;
pub mod curve;
pub mod domain;
pub mod elliptic;
pub mod error;
pub mod frame;
pub mod implicit;
pub mod nurbs;
pub mod pcurve;
pub mod ray;
pub mod roots;
pub mod store;
pub mod surface;
pub mod tol;
pub mod transform;
mod util;

pub use analytic::{Circle, Cone, Cylinder, Ellipse, Line, Plane, Sphere, Torus};
pub use curve::{CurveJet, CurveProjection, ParametricCurve};
pub use elliptic::{EllipticCone, EllipticCylinder};
pub use domain::{Domain1, Domain2, Interval, ParamAxis, Singularity, Uv};
pub use error::GeomError;
pub use frame::Frame;
pub use implicit::{Containment, CsgCell, HalfSpace, ImplicitDef, ImplicitSurface, ImplicitTorus, Quadric, Sign};
pub use nurbs::{NurbsCurve, NurbsCurve2, NurbsSurface};
pub use pcurve::{Line2, PCurve};
pub use ray::{Crossing, Ray, RayHit};
pub use store::{CurveDef, GeomAppend, GeomStore, GeomStoreMut, MemoryStore, ShapeValue, StoreData, SurfaceDef};
pub use surface::{ParametricSurface, SurfaceJet, SurfaceProjection};
pub use tol::ModelTolerance;
pub use transform::Sense;
