//! Geometry kernel: analytic and NURBS curves and surfaces, evaluation, projection and intersection.
//!
//! Skeleton created in W0; implemented in W1 against the frozen `*-api` contracts.

pub mod intersect;
mod nurbs;
mod pcurve;
mod roots;
mod store;

pub use nurbs::{BezierSegment, PreparedCurve, PreparedSurface, exact};
pub use pcurve::CurveOnSurface;
pub use intersect::{CurveGeometry, CurveSurface, CurveSurfaceHit, IntersectionCurve, MarchEnd, MarchedCurve, SsiOptions, SsiResult, curve_plane, curve_prepared_surface, curve_surface, intersect_surfaces};
pub use store::KernelStore;
