//! Geometry kernel: analytic and NURBS curves and surfaces, evaluation, projection and intersection.
//!
//! Skeleton created in W0; implemented in W1 against the frozen `*-api` contracts.

mod intersect;
mod nurbs;
mod roots;

pub use nurbs::{BezierSegment, PreparedCurve, PreparedSurface};
