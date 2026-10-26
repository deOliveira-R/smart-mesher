//! Error model.
//!
//! Two channels. *Contract violations* (NaN direction, unknown id, bad input, stale cache)
//! are `Err(RtError)`. *Geometric conditions* a transport code must handle in its inner loop
//! (a particle reached the domain boundary, or is lost) are values in [`crate::StepEnd`], so
//! the hot path never allocates an error and a lost particle carries enough data to be
//! diagnosed (`LostInfo`).

use crate::ids::{CellId, FaceId, SurfaceId};
use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::GeomError;

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
    /// Gap magnitude in model length units, when the issue is a leak (an open edge: the edge
    /// length, an upper bound of the opening).
    pub gap: Option<f64>,
    /// Human text.
    pub message: String,
    /// Suggested fix (an op or a setting), when known.
    pub suggestion: Option<String>,
}

impl BuildIssue {
    /// Construct (the type is `#[non_exhaustive]`).
    pub fn new(code: &'static str, message: impl Into<String>) -> Self {
        Self { code, entity: None, location: None, gap: None, message: message.into(), suggestion: None }
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
    /// Attach a gap magnitude.
    pub fn with_gap(mut self, gap: f64) -> Self {
        self.gap = Some(gap);
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
    /// An albedo outside `[0, 1]` (or non-finite).
    #[error("albedo {value} is outside [0, 1]")]
    BadAlbedo {
        /// The value supplied.
        value: f64,
    },
    /// Two tallies with different bin counts were merged.
    #[error("tally bin counts differ ({left} vs {right})")]
    BinCountMismatch {
        /// Bins in the receiving tally.
        left: usize,
        /// Bins in the merged tally.
        right: usize,
    },
    /// An argument is outside its documented domain (for example a random number outside
    /// `[0, 1]`).
    #[error("invalid argument: {0}")]
    InvalidInput(&'static str),
    /// A cache image is for different inputs, options or builder version.
    #[error("tracker cache is stale: {0}")]
    CacheStale(&'static str),
    /// A cache image is malformed or fails its checks.
    #[error("tracker cache is corrupt: {0}")]
    CacheCorrupt(String),
    /// A geometry operation failed (analytic refine, ray hits).
    #[error(transparent)]
    Geom(#[from] GeomError),
}

impl RtError {
    /// Stable negative code for the C ABI (`smrt_*` functions return it as their status).
    /// `0` is success and `1` "no hit" in the ABI; `-100` null argument and `-101` panic are
    /// reserved by the ABI layer. These values are frozen:
    ///
    /// | variant | code |
    /// |---|---|
    /// | `Build` | -1 |
    /// | `BadDirection` | -2 |
    /// | `NonFinite` | -3 |
    /// | `UnknownId` | -4 |
    /// | `Unsupported` | -5 |
    /// | `CacheStale` | -6 |
    /// | `CacheCorrupt` | -7 |
    /// | `Geom` | -8 |
    /// | `BadAlbedo` | -9 |
    /// | `BinCountMismatch` | -10 |
    /// | `InvalidInput` | -11 |
    ///
    /// A variant added later gets the next free code (never reused).
    pub fn code(&self) -> i32 {
        match self {
            Self::Build(_) => -1,
            Self::BadDirection { .. } => -2,
            Self::NonFinite(_) => -3,
            Self::UnknownId { .. } => -4,
            Self::Unsupported(_) => -5,
            Self::CacheStale(_) => -6,
            Self::CacheCorrupt(_) => -7,
            Self::Geom(_) => -8,
            Self::BadAlbedo { .. } => -9,
            Self::BinCountMismatch { .. } => -10,
            Self::InvalidInput(_) => -11,
        }
    }
}

/// Result alias.
pub type Result<T, E = RtError> = core::result::Result<T, E>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codes_are_frozen() {
        let all: [(RtError, i32); 11] = [
            (RtError::Build(vec![]), -1),
            (RtError::BadDirection { norm: 0.0 }, -2),
            (RtError::NonFinite("x"), -3),
            (RtError::UnknownId { kind: "cell", id: 0 }, -4),
            (RtError::Unsupported("x"), -5),
            (RtError::CacheStale("x"), -6),
            (RtError::CacheCorrupt(String::new()), -7),
            (RtError::Geom(GeomError::Invalid("x")), -8),
            (RtError::BadAlbedo { value: 2.0 }, -9),
            (RtError::BinCountMismatch { left: 1, right: 2 }, -10),
            (RtError::InvalidInput("x"), -11),
        ];
        for (e, c) in &all {
            assert_eq!(e.code(), *c, "{e}");
        }
        let mut codes: Vec<i32> = all.iter().map(|x| x.1).collect();
        codes.sort_unstable();
        codes.dedup();
        assert_eq!(codes.len(), all.len());
        assert!(codes.iter().all(|&c| c < 0 && c > -100));
    }

    #[test]
    fn issue_builders() {
        let i = BuildIssue::new("a.b", "m").at(Point3::new(1.0, 0.0, 0.0)).with_gap(0.5).suggest("fix");
        assert_eq!((i.gap, i.suggestion.as_deref()), (Some(0.5), Some("fix")));
    }
}
