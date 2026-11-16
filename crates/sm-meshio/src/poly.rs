//! The polyhedra policy shared by the exporters whose format has no general polyhedron.

use serde::{Deserialize, Serialize};

/// What to do with polyhedral cells in a format that cannot carry them (Gmsh msh4, Abaqus).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum PolyPolicy {
    /// Fail with the writer's polyhedra error.
    #[default]
    Error,
    /// Split each polyhedron into tetrahedra around a cell centroid and face centroids (new
    /// nodes after the original ones). Allowed only when every neighbour across a polygonal
    /// face is also decomposed, else the writer's non-conforming-decomposition error.
    Decompose,
    /// Omit polyhedra and report `<format>.poly_dropped`.
    Drop,
}
