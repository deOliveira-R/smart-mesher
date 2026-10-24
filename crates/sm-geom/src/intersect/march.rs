//! General surface-surface intersection by marching (placeholder).

use super::ssi::{SsiOptions, SsiResult};
use sm_geom_api::{GeomError, SurfaceDef};

pub(crate) fn intersect_general(_a: &SurfaceDef, _b: &SurfaceDef, _o: &SsiOptions) -> Result<SsiResult, GeomError> {
    Err(GeomError::Unsupported("general surface-surface intersection"))
}
