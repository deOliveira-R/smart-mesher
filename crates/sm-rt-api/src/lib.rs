//! Frozen contract `sm-rt-api` (layer C): ray tracing and particle tracking.
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
//! * [`Tracker`]: the query surface; [`StepEnd`]/[`Step`]/[`TrackState`]/[`Scratch`]: events,
//!   per-particle state and caller-owned working memory.
//! * [`BuildInput`], [`TrackerBuilder`], [`BuildOptions`], [`FacetInput`]: how a tracker is built
//!   from a mesh, facets, a B-rep or a world; [`FacetInput::check_watertight`] is the located
//!   diagnostic that replaces DAGMC's repair step.
//! * [`WorldSpec`]: universes, placements, lattices ([`StepEnd::SlotChange`] at their walls).
//! * [`mod@boundary`]: [`FaceAction`], [`ResponseKind`], [`BoundarySpec`], derived from
//!   `sm_model_api` decks; [`Tracker::resolve_boundary`].
//! * [`compat`]: DAGMC-shaped `ray_fire`, `point_in_volume`, `next_vol`, `surface_sense`,
//!   `get_angle`.
//! * [`CacheImage`]: the rebuildable `.smrt` sidecar keyed by composite content hash.
//! * `stub` (feature `stubs`): `stub::AnalyticWorld`, a reference implementor with exact
//!   answers, and its builder.
//!
//! # Orientation convention (binding, one rule everywhere)
//! A face or surface has a normal (right-hand rule on its triangles; `Tracker::face_normal`).
//! `front` is the region the normal points **out of** (`sm_topo_api::FaceSides::left`, DAGMC
//! forward volume, `surface_sense` `+1`); `back` is the region it points **into**
//! (`FaceSides::right`, sense `-1`). This holds for `FacetInput::front/back`,
//! `SurfaceInfo::front/back`, `SurfaceTable::{sense, next_region}` and `compat`. At a model
//! boundary the normal is outward, so `front` is inside and `back` is `RegionId::NONE`.
//!
//! # Determinism (binding on implementors)
//! Queries are pure; ties go to the lowest id; a point on a face with no direction uses
//! [`hashed_direction`]; building is thread-count independent; `trace_bundle` output order is
//! input order; tallies reduce over a fixed block partition in block order. No wall-clock or
//! thread-id dependence anywhere. `build` and `load` of its cache give the same
//! `structure_hash`.
//!
//! # Tolerances
//! The tracker has no tolerance constants. [`BuildOptions::tol`] is an
//! `sm_geom_api::ModelTolerance` (the model's policy, as the topology and geometry layers use):
//! `linear` for on-face tests and nudges, `angular` for the unit-direction check.
//!
//! # The C surface (documented here; implemented by `sm-rt-capi`, not by this crate)
//! This crate has no `unsafe` and no `extern "C"`. The C ABI is the `sm-rt-capi` worker's job; the
//! intended surface, so the contract can be judged for ABI-friendliness, is:
//!
//! ```text
//! typedef struct SmrtTracker SmrtTracker;   /* opaque, Arc<dyn Tracker>, shared read-only */
//! typedef struct SmrtCtx     SmrtCtx;       /* per thread: owns Scratch + TrackState */
//! typedef struct { double x, y, z; } SmrtVec3;
//! typedef struct { uint32_t component, instance; int32_t lattice[3]; } SmrtLevel;
//! typedef struct {                          /* mirrors TrackState; levels beyond 8 spill in Rust */
//!     SmrtVec3 pos, dir; uint32_t cell, region, entered_via; uint32_t n_levels; SmrtLevel level[8];
//! } SmrtState;
//! typedef struct {                          /* mirrors Step */
//!     double dist; uint32_t cell, region; int32_t end; uint32_t surface, face, to_region, to_cell;
//!     int32_t action; double albedo;
//! } SmrtStep;
//!
//! int smrt_open(const char *smrt_path, SmrtTracker **out);
//! void smrt_tracker_free(SmrtTracker *);
//! SmrtCtx *smrt_ctx_new(const SmrtTracker *); void smrt_ctx_free(SmrtCtx *);
//! int smrt_init_state(SmrtCtx *, SmrtVec3 pos, SmrtVec3 dir, uint32_t hint, SmrtState *out);
//! int smrt_step(SmrtCtx *, SmrtState *, double max_dist, SmrtStep *out);
//! int smrt_fire(SmrtCtx *, SmrtState *, double dist_limit, uint32_t max_steps, SmrtStep *out);
//! int smrt_peek(SmrtCtx *, const SmrtState *, double limit, SmrtStep *out);
//! int smrt_resolve_boundary(SmrtCtx *, SmrtState *, uint32_t face, int32_t response,
//!                           double albedo, double u0, double u1);
//! /* DAGMC shims over the same calls: smrt_ray_fire, smrt_point_in_volume, smrt_next_vol,
//!    smrt_surface_sense (+1/-1), smrt_get_angle. */
//! const char *smrt_last_error_message(void);   /* thread_local storage; no global lock */
//! ```
//!
//! Every function returns `int`: `0` ok, `1` no hit, a negative [`RtError::code`] on error,
//! `-100` null argument, `-101` caught panic (`catch_unwind` at every entry). There is no
//! global registry and no global mutex; trackers are created by explicit constructors. A
//! Fortran `iso_c_binding` block (`bind(C)` interfaces with `c_int`, `c_double`, `c_ptr`) is
//! provided next to the C header by `sm-rt-capi`.

#![forbid(unsafe_code)]

pub mod boundary;
pub mod cache;
pub mod compat;
pub mod error;
pub mod ids;
pub mod input;
pub mod state;
#[cfg(feature = "stubs")]
pub mod stub;
pub mod tally;
pub mod tracker;
pub mod world;

pub use boundary::{BoundaryDef, BoundarySelector, BoundarySpec, FaceAction, ResponseKind, ResponseMap, wrap_transform};
pub use cache::{CacheImage, CacheKey, ContentHash, ContentHasher, SectionTag, TrackerKind};
pub use error::{BuildIssue, LostCause, LostInfo, Result, RtError};
pub use ids::{CellId, ComponentId, FaceId, InstanceId, PrototypeId, RegionId, SurfaceId};
pub use input::{
    BrepInput, BuildInput, BuildOptions, BuildReport, ComplementPolicy, FacetFallback, FacetInput, MeshInput, Precision,
    RecoveryPolicy, RegionRule, SurfaceRefine, TrackerBuilder, UnsealedEdge, facet_sides_from_topo,
};
pub use state::{
    BoundaryHit, Closest, LEVEL_INLINE, Level, Located, Peek, RegionInfo, RegionTable, Scratch, Step, StepEnd, SurfaceInfo,
    SurfaceTable, TrackState, TrackerCaps,
};
pub use tally::{
    BinMap, EndCode, FireResult, Flow, Segment, SegmentSink, TallySpec, TraceEnd, TraceOptions, TrackBundle,
    TrackLengthTally,
};
pub use tracker::{BoundaryAction, PeriodicImage, Tracker, TrackerInfo, check_direction, hashed_direction, lost};
pub use world::{ComponentKind, InstancePolicy, Lattice, LatticeKind, Placement, WorldSpec};
