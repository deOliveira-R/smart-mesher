//! Mesh quality metrics (Verdict-equivalent set) and validity checks.
//!
//! * [`eval`] computes one [`Metric`] on one element from its corner coordinates (pure,
//!   allocation free, usable without a mesh).
//! * [`compute`], [`compute_many`] and [`report`] run metrics over every cell of a
//!   [`sm_mesh_api::MeshRead`] in parallel; [`validity`] classifies cells as valid, degenerate or
//!   inverted; [`summarize`] and [`histogram`] reduce columns deterministically.
//!
//! Output is bit-identical for any number of threads. See `DESIGN.md` for every formula, its
//! range and the conventions for degenerate input.

mod driver;
mod error;
mod faces;
mod metric;
mod planar;
mod polyhedron;
mod solid;
mod stats;
mod vec;

pub use driver::{
    CellStatus, Options, QualityTable, ReferenceSize, SurfaceNormals, Validity, compute, compute_many, report,
    validity,
};
pub use error::{QualityError, Result};
pub use metric::{Context, Metric, eval};
pub use stats::{CHUNK, Histogram, Summary, histogram, histogram_auto, summarize};
pub use vec::{DBL_MAX, TINY, V};
