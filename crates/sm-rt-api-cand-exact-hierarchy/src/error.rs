//! Error model: typed errors, and a typed, non-panicking "lost particle" report.

use crate::types::{FaceSide, FlatIx};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::GeomError;

/// Why a particle could not be advanced.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum LostReason {
    /// No region contains the point (outside the model with no implicit complement, or a gap).
    NoRegionAtPoint,
    /// The particle is in a bounded region but no exit surface was found along its ray.
    NoExitFound,
    /// The surface sides disagree: the neighbour across a face does not list this region.
    InconsistentSides,
    /// More events than `RtOptions::max_events` in one trace (reflection trap).
    EventLimit,
    /// A coordinate or direction was NaN or infinite.
    NonFinite,
}

/// Everything needed to reproduce and debug a lost particle (DAGMC prints only a message).
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct LostReport {
    /// What went wrong.
    pub reason: LostReason,
    /// Position when the problem was detected.
    pub pos: Point3,
    /// Unit direction.
    pub dir: Vector3,
    /// The face the particle last crossed, with its side.
    pub last_face: Option<FaceSide>,
    /// Copy the particle believed it was in.
    pub copy: Option<FlatIx>,
    /// Faces within `tol.linear` of the position (candidates for a gap or overlap), ascending.
    pub nearby_faces: Vec<EntityId>,
    /// Recovery attempts made before giving up (see [`crate::RecoveryPolicy`]).
    pub retries: u8,
}

impl LostReport {
    /// A report with no extra diagnostics.
    pub fn new(reason: LostReason, pos: Point3, dir: Vector3) -> Self {
        Self { reason, pos, dir, last_face: None, copy: None, nearby_faces: Vec::new(), retries: 0 }
    }
}

/// Errors of the tracking contract.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum RtError {
    /// The build input is unusable (open shell, missing carrier, bad hierarchy).
    #[error("invalid tracking input: {0}")]
    Input(String),
    /// A requested capability is not provided by this implementor.
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
    /// Exact geometry failed (singular surface, root finder).
    #[error("geometry: {0}")]
    Geom(#[from] GeomError),
    /// The particle is lost. Always a value, never a panic or abort.
    #[error("particle lost: {:?}", .0.reason)]
    Lost(Box<LostReport>),
    /// A cache blob is corrupt or for a different content key.
    #[error("cache: {0}")]
    Cache(String),
}

/// Result alias.
pub type Result<T, E = RtError> = core::result::Result<T, E>;
