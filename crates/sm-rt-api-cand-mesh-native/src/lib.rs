//! Candidate frozen contract `sm-rt-api` (angle: mesh-native).
//!
//! # Thesis
//! The polyhedral volume mesh *is* the tracking structure. A particle walks cell to cell
//! through oriented face adjacency (`sm_mesh_api`'s `across`), so crossing is a table lookup,
//! planar faces make every chord exact, and there is nothing to be "watertight" about: a face
//! belongs to exactly two cells by construction, which `sm_mesh_api::validate` already checks.
//! A BVH exists only for (a) point location without a hint and (b) surface-only geometry
//! (facet worlds, and the implicit complement outside the mesh).
//!
//! The same structure serves Monte Carlo (`fire`, `step`, boundary conditions), Sn/MOC
//! characteristics (`trace`, `trace_bundle` into a CSR track database) and tallies (`CellId` is
//! the transport mesh's cell index, so flux, track-length scores and Sn sweeps share indexing;
//! `bin_map` coarsens).
//!
//! # Map
//! * [`Tracker`]: the query surface; [`StepEnd`]/[`Step`]/[`TrackState`]: events and per-particle state.
//! * [`BuildInput`], [`TrackerBuilder`], [`FacetInput`]: how a tracker is built from a
//!   mesh, facets or a B-rep; [`FacetInput::check_watertight`] is the located diagnostic that
//!   replaces DAGMC's repair step.
//! * [`WorldSpec`]: universes, placements, lattices.
//! * [`BoundaryKind`], [`Tracker::resolve_boundary`]: vacuum / reflective / white / periodic.
//! * [`CacheImage`]: the rebuildable `.smsh` cache keyed by content hash.
//! * [`ffi`]: the C ABI. [`stub::AnalyticWorld`]: a reference implementor with exact answers.
//!
//! # Determinism (binding on implementors)
//! Queries are pure; ties go to the lowest id; building is thread-count independent;
//! `trace_bundle` output order is input order; tallies reduce over a fixed block partition in
//! block order. No wall-clock or thread-id dependence anywhere.

pub mod cache;
pub mod error;
pub mod ffi;
pub mod ids;
pub mod input;
pub mod state;
pub mod stub;
pub mod tally;
pub mod tracker;
pub mod world;

pub use cache::{CacheImage, CacheKey, ContentHash, ContentHasher, SectionTag, TrackerKind};
pub use error::{BuildIssue, LostCause, LostInfo, Result, RtError};
pub use ids::{CellId, ComponentId, FaceId, InstanceId, RegionId, SurfaceId};
pub use input::{
    BoundarySelector, BoundarySpec, BrepInput, BuildInput, BuildOptions, ComplementPolicy, FacetInput, MeshInput,
    RegionRule, SurfaceRefine, TrackerBuilder, facet_sides_from_topo,
};
pub use state::{
    BoundaryKind, Closest, Level, Located, MAX_LEVELS, RegionInfo, RegionTable, Step, StepEnd, SurfaceInfo,
    SurfaceTable, TrackState, TrackerCaps,
};
pub use tally::{
    BinMap, EndCode, FireResult, Flow, Segment, SegmentSink, TallySpec, TraceEnd, TraceOptions, TrackBundle,
    TrackLengthTally,
};
pub use tracker::{BoundaryAction, PeriodicImage, Tracker, TrackerInfo, check_direction, lost};
pub use world::{ComponentKind, InstancePolicy, Lattice, LatticeKind, Placement, WorldSpec};
