//! Candidate contract for ray tracing and particle tracking (becomes `sm-rt-api`).
//!
//! Angle: **DAGMC-compatible first.** The [`World`] trait mirrors the `DagMC` class of
//! DAGMC/MOAB so a transport code can swap DAGMC for smart-mesh with a mechanical port:
//!
//! | DagMC (C++)                    | here                                   | difference |
//! |--------------------------------|----------------------------------------|------------|
//! | `ray_fire(vol, xyz, uvw, surf, dist, hist, limit, orient)` | [`World::ray_fire`] | `Option<RayHit>` instead of `surf == 0`; `dist_limit` is `Option` |
//! | `point_in_volume(vol, xyz, r, uvw, hist)` | [`World::point_in_volume`] | no random direction: the default direction is a hash of the point |
//! | `point_in_volume_slow`         | [`World::point_in_volume_exact`]       | |
//! | `find_volume`                  | [`World::find_volume`]                 | |
//! | `test_volume_boundary`         | [`World::test_volume_boundary`]        | |
//! | `closest_to_location`          | [`World::closest_to_location`]         | |
//! | `get_angle`                    | [`World::get_angle`]                   | named `normal` alias exists |
//! | `surface_sense`, `next_vol`    | [`World::surface_sense`], [`World::next_vol`] | |
//! | `measure_volume/area`          | [`World::measure_volume`], [`World::measure_area`] | |
//! | implicit complement            | [`World::implicit_complement`]         | always present |
//! | `RayHistory`                   | [`RayHistory`]                         | same operations |
//!
//! Beyond DAGMC (the "beats DAGMC" part): [`World::trace`] walks a ray through many volumes in
//! one call (MOC segments, flux tallies), [`World::ray_fire_batch`] fires many rays, boundary
//! conditions are first-class ([`Boundary`], [`World::apply_boundary`]), lost particles are
//! typed values ([`TraceEnd::Lost`]) rather than silent errors, build reports name leaking
//! edges ([`BuildReport`]), and results are bit-identical for any thread count.
//!
//! # Determinism (binding)
//! * Handles are dense indices assigned in ascending `EntityId` order, so they do not depend on
//!   hash iteration order or thread count.
//! * Every query is a pure function of `(world, arguments)`. There is no hidden RNG: the default
//!   test direction of [`World::point_in_volume`] is derived from the bits of the point.
//! * Ties (equal hit distances) go to the lowest [`SurfHandle`].
//! * `BuildParams::threads` may change build time, never results or [`World::content_hash`].
//!
//! # Layering
//! Depends on `sm-core`, `sm-geom-api`, `sm-topo-api`, `sm-mesh-api` (L1 contracts only). It is
//! a contract crate (layer C). Backends (OBB tree, BVH, Embree-style wide BVH, analytic CSG)
//! live in L3 crates and implement [`World`] and [`Backend`].

#![cfg_attr(not(feature = "capi"), forbid(unsafe_code))]

pub mod analytic;
pub mod boundary;
pub mod build;
pub mod cache;
#[cfg(feature = "capi")]
pub mod capi;
pub mod error;
pub mod handle;
pub mod query;
pub mod world;

pub use analytic::{AnalyticBackend, AnalyticShape, AnalyticSpec, AnalyticWorld, SolidSpec};
pub use boundary::{Boundary, BoundaryAction, reflect};
pub use build::{
    Backend, BrepInput, BuildInput, BuildParams, BuildReport, FacetSoup, InstanceSpec, InstancedInput, LatticeSpec,
    MeshInput, Precision, SurfaceGeometry, SurfaceSpec, UnsealedEdge, VolumeSpec,
};
pub use cache::{CacheBlob, CacheHeader, ContentHash, ContentHasher, SectionEntry};
pub use error::{Result, RtError};
pub use handle::{FacetId, SurfHandle, SurfSense, VolHandle};
pub use query::{
    Closest, FireOptions, RayHistory, RayHit, RayIn, RayOrientation, Segment, TraceEnd, TraceOptions, TraceOutcome,
};
pub use world::{BackendInfo, CellExit, MeshTrack, check_dir, check_point, SurfaceInfo, SurfaceVolumes, VolumeInfo, World};
