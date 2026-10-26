//! Mesh smoothing suite: Laplacian, smart Laplacian, centroid-area and mean-ratio optimization;
//! nodes stay on their geometry and no cell is made worse than the guard allows.
//!
//! See `DESIGN.md`. [`smooth`] is the pure library entry; [`SmoothPlugin`] registers the
//! `mesh.smooth` op and the `mesh.smooth_preview` query.

mod kernel;
mod moved;
mod normals;
mod op;
mod prep;
mod smooth;
mod types;

pub use op::{MeshSmooth, MeshSmoothPreview, SmoothPlugin};
pub use smooth::{apply_outcome, smooth};
pub use types::{
    Method, NodeSelect, Objective, QualityStats, Schedule, SmoothError, SmoothInput, SmoothOutcome, SmoothReport,
    SmoothRequest,
};
