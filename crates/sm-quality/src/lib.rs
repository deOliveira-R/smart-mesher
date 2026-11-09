//! Mesh quality metrics (Verdict-equivalent set) and validity checks.
//!
//! * [`eval`] computes one [`Metric`] on one element from its corner coordinates (pure,
//!   allocation free, usable without a mesh).
//! * [`compute`], [`compute_many`] and [`report`] run metrics over every cell of a
//!   [`sm_mesh_api::MeshRead`] in parallel; [`validity`] classifies cells as valid, degenerate or
//!   inverted; [`summarize`] and [`histogram`] reduce columns deterministically.
//!
//! * [`fv_geometry`], [`fv_compute`] and [`face_to_cell`] add the finite-volume metrics of OpenFOAM
//!   `checkMesh` and Fluent ([`FvMetric`]); [`Profile`] and [`rules`] hold the graded limits.
//!
//! Output is bit-identical for any number of threads. See `DESIGN.md` for every formula, its
//! range and the conventions for degenerate input.

mod check;
mod convexity;
mod driver;
mod error;
mod extra;
mod faces;
mod fv;
mod metric;
mod planar;
mod polyhedron;
mod profile;
mod solid;
mod stats;
mod vec;

pub use driver::{
    CellStatus, Options, QualityTable, ReferenceSize, SurfaceNormals, Validity, compute, compute_many, report,
    validity,
};
pub use check::check_indices;
pub use convexity::{
    CONVEX_CHECK_MAX, CellConvexity, ConvexityReport, RT_PLANARITY_TOL, cell_convexity, convexity_column, convexity_report,
    fast_path_eligible,
};
pub use error::{QualityError, Result};
pub use metric::{Context, Metric, eval};
pub use stats::{CHUNK, Histogram, Summary, histogram, histogram_auto, summarize};
pub use vec::{DBL_MAX, TINY, V};
pub use fv::{FvColumn, FvEntity, FvGeometry, FvMetric, Reduce, face_to_cell, fv_compute, fv_geometry};
pub use profile::{Grade, Measure, Profile, Rule, Sense, rules, transport_extra_rules};
