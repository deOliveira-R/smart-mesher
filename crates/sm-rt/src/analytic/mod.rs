//! Analytic CSG leaf: exact surface tracking on planes, spheres, cylinders, cones, tori and
//! general quadrics (no mesh). See `DESIGN.md`, "Analytic leaf".
//!
//! * [`AnalyticTrackerBuilder`] builds an [`AnalyticTracker`] from a `BuildInput::Hier` whose
//!   root is a CSG leaf.
//! * [`check_overlaps`] is the sampled overlap and gap check.

mod builder;
mod check;
mod kernel;
mod leaf;
mod tracker;

pub(crate) use builder::{build_proto_csg, is_bare_csg};
pub use builder::AnalyticTrackerBuilder;
pub use check::check_overlaps;
pub(crate) use check::check_located;
pub use tracker::{ANALYTIC_BUILDER_VERSION, AnalyticTracker};
