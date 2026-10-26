//! Error model. Queries never panic and never return a silent wrong answer.

use crate::handle::{SurfHandle, VolHandle};
use sm_core::Point3;

/// Result alias.
pub type Result<T> = core::result::Result<T, RtError>;

/// Everything a ray-tracing call can report.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum RtError {
    /// A volume handle is out of range.
    #[error("no such volume {0:?}")]
    BadVolume(VolHandle),
    /// A surface handle is out of range.
    #[error("no such surface {0:?}")]
    BadSurface(SurfHandle),
    /// The surface does not bound the volume.
    #[error("surface {surface:?} does not bound volume {volume:?}")]
    NotIncident {
        /// The surface.
        surface: SurfHandle,
        /// The volume.
        volume: VolHandle,
    },
    /// A point or direction has a NaN or infinite component.
    #[error("non-finite input")]
    NonFinite,
    /// The direction is not a unit vector (tolerance 1e-9) or is zero.
    #[error("direction is not a unit vector (norm {0})")]
    NotUnit(f64),
    /// The measure of an unbounded volume (the implicit complement) was requested.
    #[error("volume {0:?} is unbounded")]
    Unbounded(VolHandle),
    /// The point is claimed by zero or by several volumes (a gap or an overlap in the geometry).
    #[error("point {point:?} is in {count} volumes (expected exactly 1)")]
    Ambiguous {
        /// The point.
        point: Point3,
        /// How many volumes claim it.
        count: u32,
    },
    /// [`crate::World::trace`] exceeded `max_crossings`.
    #[error("more than {0} surface crossings in one trace")]
    TooManyCrossings(u32),
    /// Build input is inconsistent (message names the offending entity).
    #[error("invalid build input: {0}")]
    BadInput(String),
    /// Build found open edges and `BuildParams::require_sealed` was set.
    #[error("geometry is not watertight: {0} open edges (see BuildReport)")]
    NotWatertight(usize),
    /// The backend does not support this input kind or query.
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
    /// A cache blob is corrupt, or was built for other input, parameters or backend.
    #[error("cache rejected: {0}")]
    Cache(String),
}

impl RtError {
    /// Stable negative code for the C ABI (`0` is success, `1` is "no hit").
    pub const fn code(&self) -> i32 {
        match self {
            RtError::BadVolume(_) => -1,
            RtError::BadSurface(_) => -2,
            RtError::NotIncident { .. } => -3,
            RtError::NonFinite => -4,
            RtError::NotUnit(_) => -5,
            RtError::Unbounded(_) => -6,
            RtError::Ambiguous { .. } => -7,
            RtError::TooManyCrossings(_) => -8,
            RtError::BadInput(_) => -9,
            RtError::NotWatertight(_) => -10,
            RtError::Unsupported(_) => -11,
            RtError::Cache(_) => -12,
        }
    }
}
