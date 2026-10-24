//! Intersections.

use sm_geom_api::{GeomError, Interval, Ray, RayHit};

use crate::nurbs::PreparedSurface;

/// Placeholder until the line/surface intersector lands.
pub(crate) fn ray_nurbs_surface(_s: &PreparedSurface, _ray: &Ray, _range: Interval, _out: &mut Vec<RayHit>) -> Result<(), GeomError> {
    Err(GeomError::Unsupported("ray_hits"))
}
