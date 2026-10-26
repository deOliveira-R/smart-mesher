//! Candidate contract for ray tracing and particle tracking (future `sm-rt-api`).
//!
//! Angle: **exact geometry and hierarchy first.**
//! - Surfaces are intersected analytically through `sm-geom-api` (`ParametricSurface::ray_hits`,
//!   `ImplicitSurface`, `CsgCell`); facets exist only as a fallback ([`FacetFallback`]).
//! - Reactor geometry is a hierarchy (pin, assembly, core). Copies are traced in their
//!   prototype's frame, so one structure serves N copies ([`TrackerInfo::prototypes`] vs
//!   [`TrackerInfo::copies`]); lattice neighbours come from index arithmetic
//!   (`sm_model_api::Layout::offset`), not from a search.
//! - After a crossing the next region is **read** from the face sides (`FaceSides`), not
//!   searched. A tree/search is used only by [`Tracker::locate`].
//! - Deck faces (mirror, translation, rotation) are applied inside [`Tracker::step`]
//!   ([`FaceAction`]); response boundaries are reported with no RNG involved.
//! - The structure is derived data with a content-hash key and a `.smsh`-compatible blob
//!   ([`RtCacheKey`], [`RtCacheBlob`]).
//!
//! Entry points: [`Tracker`] (queries), [`TrackerBuilder`] + [`BuildInput`] (construction),
//! [`stub::AnalyticWorld`] (reference semantics), `capi` (feature `capi`, C ABI sketch).
//!
//! See `DESIGN.md` for the baseline comparison (DAGMC, double-down, OpenMC).

pub mod boundary;
pub mod build;
pub mod cache;
#[cfg(feature = "capi")]
pub mod capi;
pub mod error;
pub mod stub;
pub mod tracker;
pub mod types;

pub use boundary::{BoundarySpec, FaceAction, ResponseKind, wrap_transform};
pub use build::{BuildInput, BuildIssue, BuildIssueCode, BuildReport, FacetFace, FacetSoup, Geometry, TrackerBuilder};
pub use cache::{CACHE_VERSION, Dtype, Fnv64, RtArray, RtCacheBlob, RtCacheKey, boundary_hash};
pub use error::{LostReason, LostReport, Result, RtError};
pub use tracker::{FacetFallback, RecoveryPolicy, RtOptions, Tracker, TrackerInfo};
pub use types::{
    BoundaryHit, DeckAction, Event, FaceSide, FlatIx, LEVEL_INLINE, Level, Location, Nearest, Scratch, Segment, SegmentSink, Step, TraceBatch, TraceEnd, TrackState,
};

#[cfg(test)]
mod tests;
