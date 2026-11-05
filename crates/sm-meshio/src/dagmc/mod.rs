//! DAGMC `.h5m` export: a faceted B-rep (triangles, surfaces with a forward and a backward
//! volume, volumes, `mat:` groups) written as a MOAB HDF5 file that DAGMC and OpenMC read.
//!
//! * [`model_from_mesh`]: skin and interface faces of a classified [`MeshRead`] to a
//!   [`DagmcModel`] (quads split on the shorter diagonal, polygons fanned from a centroid).
//! * [`model_from_facets`]: the same model from sm-tess style facets ([`FacetInput`]).
//! * [`write_h5m`], [`export_h5m`]: the byte-exact writer (private pure-Rust HDF5 subset).
//!
//! The file layout, with the MOAB sources it follows, is in `H5M_FORMAT.md`. Names are fixed:
//! a cell set `X` becomes the group `mat:X`, the optional graveyard is `mat:Graveyard`.
//! Geometry is never repaired: a volume whose triangles do not close is [`DagmcError::OpenSurface`].

mod model;
mod write;

#[cfg(test)]
mod tests;

pub use crate::h5::H5Error;
pub use model::{model_from_facets, model_from_mesh};
pub use write::{export_h5m, write_h5m};

use crate::report::LossReport;
use serde::{Deserialize, Serialize};
use sm_mesh_api::MeshError;

/// A faceted surface between two volumes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DagmcSurface {
    /// Model id (the `TopoFace` serial; unique, ascending in [`DagmcModel::surfaces`]).
    pub id: u64,
    /// Index into [`DagmcModel::volumes`] of the volume the triangle normals point OUT of
    /// (DAGMC forward; IR `face_left`; rt-api `front`).
    pub forward: Option<u32>,
    /// Index of the volume the normals point into (`None` at the model boundary).
    pub backward: Option<u32>,
}

/// A volume (a closed set of surfaces).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DagmcVolume {
    /// Model id (the `TopoVolume` serial; unique, ascending in [`DagmcModel::volumes`]).
    pub id: u64,
}

/// A material group: written as the set `mat:<name>` holding its volumes.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DagmcGroup {
    /// Set name without the `mat:` prefix.
    pub name: String,
    /// Indices into [`DagmcModel::volumes`], ascending.
    pub volumes: Vec<u32>,
}

/// A faceted model: shared vertices, triangles grouped by surface, senses and groups.
///
/// Triangles are counter-clockwise seen from the backward side: the right-hand normal points
/// out of the surface's `forward` volume. `triangles` are contiguous per surface in
/// `surfaces` order (`tri_surface` is non-decreasing).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct DagmcModel {
    /// Vertex positions.
    pub vertices: Vec<[f64; 3]>,
    /// Triangles as vertex indices.
    pub triangles: Vec<[u32; 3]>,
    /// Surface index of each triangle.
    pub tri_surface: Vec<u32>,
    /// Surfaces, ascending by id.
    pub surfaces: Vec<DagmcSurface>,
    /// Volumes, ascending by id.
    pub volumes: Vec<DagmcVolume>,
    /// Material groups, ascending by name.
    pub groups: Vec<DagmcGroup>,
}

/// The graveyard: two nested axis-aligned boxes around the model, their own volume.
///
/// The inner box is the model's bounding box scaled by `scale` about its centre; the outer box
/// is `thickness_frac` times the longest inner edge further out. Normals point out of the
/// graveyard volume (into the model at the inner box, away at the outer).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[non_exhaustive]
pub struct Graveyard {
    /// Scale of the inner box relative to the model bounding box (> 1).
    pub scale: f64,
    /// Wall thickness as a fraction of the longest inner edge (> 0).
    pub thickness_frac: f64,
}

impl Graveyard {
    /// A graveyard with the given inner-box scale and wall thickness fraction.
    pub fn new(scale: f64, thickness_frac: f64) -> Self {
        Self { scale, thickness_frac }
    }
}

impl Default for Graveyard {
    fn default() -> Self {
        Self { scale: 1.1, thickness_frac: 0.05 }
    }
}

/// Options of the DAGMC exporter.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[non_exhaustive]
pub struct DagmcOptions {
    /// Add the graveyard (`None`: do not). Skipped when the model already has a group named
    /// `Graveyard`.
    pub graveyard: Option<Graveyard>,
    /// Faceting tolerance recorded as the `FACETING_TOL` tag of a file set (`None`: no tag).
    pub faceting_tol: Option<f64>,
}

impl DagmcOptions {
    /// These options with a `FACETING_TOL` tag of `tol`.
    pub fn with_faceting_tol(mut self, tol: f64) -> Self {
        self.faceting_tol = Some(tol);
        self
    }

    /// The default options with a graveyard.
    pub fn with_graveyard(graveyard: Graveyard) -> Self {
        Self { graveyard: Some(graveyard), ..Self::default() }
    }
}

/// What a write produced.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct DagmcReport {
    /// Triangles written.
    pub triangles: u64,
    /// Surface sets written.
    pub surfaces: u64,
    /// Volume sets written.
    pub volumes: u64,
    /// Group sets written.
    pub groups: u64,
    /// File size in bytes.
    pub bytes: u64,
    /// FNV-1a 64 of the file bytes.
    pub hash: u64,
    /// What the format could not carry (`h5m.*` codes).
    pub loss: LossReport,
}

/// Errors of the DAGMC exporter.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum DagmcError {
    /// An I/O error from the output.
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
    /// A mesh-API call failed.
    #[error(transparent)]
    Mesh(#[from] MeshError),
    /// The mesh or model uses something the exporter does not handle (partition-classified
    /// entities, a dimension-2 mesh, inactive cells, a face with fewer than 3 vertices, ...).
    #[error("unsupported: {0}")]
    Unsupported(String),
    /// A face on a volume boundary is not classified on a `TopoFace`.
    #[error("face {face} bounds a volume but is not classified on a TopoFace")]
    Unclassified {
        /// Face index.
        face: u32,
    },
    /// The triangles of one surface disagree about which volumes are on its two sides.
    #[error("surface {surface}: triangles disagree about their forward and backward volumes")]
    InconsistentSense {
        /// Surface id.
        surface: u64,
    },
    /// An edge is used by more than two triangles of one volume.
    #[error("surface {surface}: edge {edge:?} is used by more than two triangles of a volume")]
    NonManifoldSurface {
        /// Surface id of one triangle using the edge.
        surface: u64,
        /// Model vertex indices of the edge.
        edge: [u32; 2],
    },
    /// The triangles of a volume do not close; the first open edge is reported, nothing is
    /// repaired.
    #[error("volume {volume} is not closed at edge {edge:?} (near {at:?})")]
    OpenSurface {
        /// Volume id.
        volume: u64,
        /// Model vertex indices of the first open edge.
        edge: [u32; 2],
        /// Midpoint of that edge.
        at: [f64; 3],
    },
    /// A group name does not fit the 31 bytes of a MOAB `NAME` tag value (never truncated).
    #[error("group name {name:?} does not fit in 31 bytes")]
    NameTooLong {
        /// The name with its `mat:` prefix.
        name: String,
    },
    /// The model or an option is inconsistent (index out of range, non-finite value, ...).
    #[error("invalid model: {0}")]
    Invalid(String),
    /// The HDF5 layer refused the tree.
    #[error(transparent)]
    H5(#[from] H5Error),
}
