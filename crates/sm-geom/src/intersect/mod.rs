//! Intersections: curve-plane, curve-surface and surface-surface.
//!
//! Closed forms are used wherever the geometry allows (line/circle/ellipse against planes and
//! quadrics, plane against every quadric and the torus' symmetric cuts). NURBS use bounding-box
//! subdivision of the Bezier decomposition followed by Newton. The general surface-surface
//! intersection marches along the curve from seeds found with iso-curves; see
//! [`crate::SsiOptions`] for its documented limits.

mod bernstein;
mod curve_surface;
mod interval;

pub use curve_surface::{curve_plane, curve_prepared_surface, curve_surface};
pub(crate) use curve_surface::ray_nurbs_surface;

use sm_core::Point3;
use sm_geom_api::Uv;

/// One point where a curve meets a surface.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurveSurfaceHit {
    /// Curve parameter of the hit.
    pub t: f64,
    /// The point.
    pub point: Point3,
    /// Surface parameters when known (analytic and NURBS surfaces; `None` for implicit-only).
    pub uv: Option<Uv>,
    /// True for a grazing contact (double root): the curve touches without crossing.
    pub tangent: bool,
}

/// Result of a curve/surface intersection.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub enum CurveSurface {
    /// Isolated hits ordered by curve parameter (possibly none).
    Points(Vec<CurveSurfaceHit>),
    /// The curve lies on the surface (detected for plane and exact-analytic cases).
    Coincident,
}
