//! Prepared NURBS curves and surfaces and their algorithms.

pub(crate) mod basis;
pub(crate) mod curve;
pub(crate) mod project;
pub(crate) mod spline1;
pub(crate) mod surface;

pub use curve::{BezierSegment, PreparedCurve};
pub use surface::PreparedSurface;
