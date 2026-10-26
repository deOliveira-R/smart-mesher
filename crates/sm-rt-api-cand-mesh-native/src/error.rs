//! Error model.
//!
//! Two channels. *Contract violations* (NaN direction, unknown id, bad input, stale cache)
//! are `Err(RtError)`. *Geometric conditions* a transport code must handle in its inner loop
//! (a particle reached the domain boundary, or is lost) are values in [`crate::StepEnd`], so
//! the hot path never allocates an error and a lost particle carries enough data to be
//! diagnosed (`LostInfo`).

use crate::ids::{CellId, FaceId, SurfaceId};
use sm_core::{EntityId, Point3, Vector3};

/// Why a particle could not be tracked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub enum LostCause {
    /// No cell (and not the complement) contains the point.
    NoCellContainsPoint,
    /// The ray is inside a cell but no face is exited (inconsistent cell; mesh should fail
    /// `validate`, so this is a bug in the mesh or the tracker).
    NoExitFace,
    /// The face just entered through is not a face of the entered cell.
    InconsistentEntryFace,
    /// Position or direction became non-finite.
    NonFinite,
    /// The per-ray step cap was reached (`TraceOptions::max_steps`): a loop in the walk.
    MaxStepsExceeded,
    /// Recovery by nudge + re-location was tried and failed.
    RecoveryFailed,
}

/// A located, linked description of a lost particle. This is what `mesh.validate` /
/// diagnostics tooling reports to a user: where, which cell, which geometry entity.
#[derive(Clone, Copy, Debug, PartialEq)]
#[non_exhaustive]
pub struct LostInfo {
    /// Why.
    pub cause: LostCause,
    /// Position when lost.
    pub pos: Point3,
    /// Direction when lost.
    pub dir: Vector3,
    /// Last cell the particle was validly in.
    pub last_cell: CellId,
    /// Last face crossed.
    pub last_face: Option<FaceId>,
    /// Nearest boundary surface, when known.
    pub nearest_surface: Option<SurfaceId>,
    /// Model entity (volume/face) the last cell is classified on, when known.
    pub classified_on: Option<EntityId>,
}

impl LostInfo {
    /// Construct (the type is `#[non_exhaustive]`).
    pub const fn new(cause: LostCause, pos: Point3, dir: Vector3, last_cell: CellId) -> Self {
        Self { cause, pos, dir, last_cell, last_face: None, nearest_surface: None, classified_on: None }
    }
}

/// One actionable build problem (a located finding, in the style of `sm_mesh_api::Violation`).
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct BuildIssue {
    /// Stable machine code, e.g. `"facet.open_edge"`, `"mesh.nonplanar_face"`.
    pub code: &'static str,
    /// Offending entity, when there is one.
    pub entity: Option<EntityId>,
    /// Where, when known.
    pub location: Option<Point3>,
    /// Human text.
    pub message: String,
    /// Suggested fix (an op or a setting), when known.
    pub suggestion: Option<String>,
}

impl BuildIssue {
    /// Construct (the type is `#[non_exhaustive]`).
    pub fn new(code: &'static str, message: impl Into<String>) -> Self {
        Self { code, entity: None, location: None, message: message.into(), suggestion: None }
    }
    /// Attach an entity.
    pub fn with_entity(mut self, e: EntityId) -> Self {
        self.entity = Some(e);
        self
    }
    /// Attach a location.
    pub fn at(mut self, p: Point3) -> Self {
        self.location = Some(p);
        self
    }
    /// Attach a suggestion.
    pub fn suggest(mut self, s: impl Into<String>) -> Self {
        self.suggestion = Some(s.into());
        self
    }
}

/// Errors from building, loading and querying a tracker.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum RtError {
    /// Input failed validation (the issues are located).
    #[error("invalid tracking input: {} issue(s), first: {}", .0.len(), .0.first().map(|i| i.message.as_str()).unwrap_or(""))]
    Build(Vec<BuildIssue>),
    /// A direction was zero, non-finite or not unit within tolerance.
    #[error("direction must be finite and unit length (got norm {norm})")]
    BadDirection {
        /// The norm that was supplied.
        norm: f64,
    },
    /// A query argument was NaN or infinite.
    #[error("non-finite query argument: {0}")]
    NonFinite(&'static str),
    /// An id does not exist in this tracker.
    #[error("unknown {kind} id {id}")]
    UnknownId {
        /// "region", "cell", "surface", "face" or "component".
        kind: &'static str,
        /// The raw id.
        id: u32,
    },
    /// The implementor does not support this feature (see `TrackerCaps`).
    #[error("unsupported by this tracker: {0}")]
    Unsupported(&'static str),
    /// Hierarchy deeper than `MAX_LEVELS`.
    #[error("hierarchy deeper than {max} levels")]
    LevelOverflow {
        /// The cap.
        max: usize,
    },
    /// A cache image is for different inputs, options or builder version.
    #[error("tracker cache is stale: {0}")]
    CacheStale(&'static str),
    /// A cache image is malformed or fails its checks.
    #[error("tracker cache is corrupt: {0}")]
    CacheCorrupt(String),
}

/// Result alias.
pub type Result<T, E = RtError> = core::result::Result<T, E>;
