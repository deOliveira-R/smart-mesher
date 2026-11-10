//! Ray tracing / particle tracking: implements the `sm-rt-api` contract (cell walking on the
//! polyhedral mesh, facet input, BVH locate, `.smrt` cache).
//!
//! See `DESIGN.md` for the positioning, the data layout and the robustness rules.
//!
//! * [`MeshTrackerBuilder`] builds a [`MeshTracker`] from a mesh ([`sm_rt_api::BuildInput::Mesh`])
//!   or a watertight facet soup ([`sm_rt_api::BuildInput::Facets`]).
//! * [`WorldTracker`] is the hierarchical walker built from [`sm_rt_api::BuildInput::Hier`]
//!   (placements, rectangular lattices, global instance-numbered cells).
//! * [`AnalyticTrackerBuilder`] builds an [`AnalyticTracker`] for a CSG leaf (exact quadric and
//!   torus surface tracking); [`check_overlaps`] is the sampled overlap and gap check.
//! * With `BuildOptions::partition.enabled` a [`MeshTracker`] also implements the majorant view
//!   (kd partition, surface and delta flight segments, safety distance, region-jump `fire`); see
//!   `DESIGN.md`, "Majorant partition and delta flights".
//! * [`check_facets`] is the grouped, located watertightness diagnostic.
//! * [`write_smrt`], [`read_smrt`], [`load_image`] handle the `.smrt` sidecar.
//! * With `BuildOptions::partition.enabled` a [`WorldTracker`] also implements the majorant view: the
//!   nodes are the prototype tree (a lattice is one node; elements of one universe share its subtree),
//!   and delta flights cross element walls inside the segment stream.
//! * [`csg_world`] bridges a model (volumes, prototypes, lattices, clusters) to CSG leaves and a
//!   [`sm_rt_api::WorldSpec`] ([`CsgWorld`]); [`CsgWorld::world_tracker`] tracks it exactly without flattening (CSG prototypes under
//!   `Union`/`Lattice`); [`CsgWorld::analytic_tracker`] flattens and is the oracle.
//! * [`RtPlugin`] registers the read-only queries `rt.info`, `rt.check`, `rt.locate`, `rt.partition`,
//!   `rt.check_overlaps`.

#![forbid(unsafe_code)]

mod analytic;
mod bridge;
mod build;
mod builder;
mod bvh;
mod core;
mod delta;
mod facets;
mod image;
mod overlaps_query;
mod plugin;
mod tracker;
mod vec3;
mod walk;
mod world;

pub use analytic::{ANALYTIC_BUILDER_VERSION, AnalyticTracker, AnalyticTrackerBuilder, check_overlaps};
pub use bridge::{BridgeError, CellInfo, CheckScope, CsgWorld, CsgWorldOptions, MAX_FLAT_CELLS, WorldFinding, check_world_overlaps, csg_world};
pub use builder::{BUILDER_VERSION, BuildExtras, MeshTrackerBuilder, load_image};
pub use facets::check_facets;
pub use image::{read_smrt, write_smrt, write_smrt_file};
pub use overlaps_query::{OverlapCellOut, OverlapFindingOut, RtCheckOverlaps, RtCheckOverlapsOut};
pub use plugin::RtPlugin;
pub use tracker::MeshTracker;
pub use world::WorldTracker;
