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
//! * [`RtPlugin`] registers the read-only queries `rt.info`, `rt.check`, `rt.locate`.

#![forbid(unsafe_code)]

mod analytic;
mod build;
mod builder;
mod bvh;
mod core;
mod delta;
mod facets;
mod image;
mod plugin;
mod tracker;
mod vec3;
mod walk;
mod world;

pub use analytic::{ANALYTIC_BUILDER_VERSION, AnalyticTracker, AnalyticTrackerBuilder, check_overlaps};
pub use builder::{BUILDER_VERSION, BuildExtras, MeshTrackerBuilder, load_image};
pub use facets::check_facets;
pub use image::{read_smrt, write_smrt, write_smrt_file};
pub use plugin::RtPlugin;
pub use tracker::MeshTracker;
pub use world::WorldTracker;
