//! Geometry kernel: analytic and NURBS curves and surfaces, evaluation, projection and
//! intersection, implementing the frozen `sm-geom-api` contract.
//!
//! - [`PreparedCurve`] / [`PreparedSurface`]: NURBS compiled for fast evaluation (position-only
//!   fast path, degree-sized scratch, homogeneous poles), exact editing (knot insertion,
//!   refinement, clamping, splitting, Bezier extraction; periodic directions supported) and
//!   global closest-point projection (Bezier boxes plus Newton). They implement the contract
//!   traits, including the surface ray hook the reference NURBS evaluator lacks.
//! - [`exact`]: exact rational NURBS forms of circles, ellipses, cylinders, cones, spheres,
//!   tori, planes and conic arcs.
//! - [`CurveOnSurface`]: a pcurve lifted to space with second derivatives by the chain rule.
//! - [`KernelStore`]: the performance-oriented [`sm_geom_api::GeomStore`] (same handles and
//!   semantics as the reference `MemoryStore`, prepared NURBS behind the trait objects).
//! - [`intersect`]: curve-plane, curve-surface and surface-surface intersection (closed forms,
//!   Bezier subdivision, and a marching tracer with documented limits), plus [`fit`] to turn
//!   marched polylines into NURBS.
//!
//! The contract crate already ships exact closed-form evaluators for every analytic kind; this
//! crate resolves analytic handles to them rather than duplicating them (the orphan rule would
//! forbid re-implementing the traits on those types anyway). See `DESIGN.md`.

// Index-heavy numeric kernels (as in sm-geom-api): explicit indices mirror the textbook
// algorithms (Piegl and Tiller) and keep the stack-array loops simple.
#![allow(clippy::needless_range_loop)]

pub mod fit;
pub mod intersect;
mod nurbs;
mod pcurve;
mod roots;
mod store;

pub use intersect::{CurveGeometry, CurveSurface, CurveSurfaceHit, IntersectionCurve, MarchEnd, MarchedCurve, SsiOptions, SsiResult, curve_plane, curve_prepared_surface, curve_surface, intersect_surfaces};
pub use nurbs::{BezierSegment, PreparedCurve, PreparedSurface, exact};
pub use pcurve::CurveOnSurface;
pub use store::KernelStore;
