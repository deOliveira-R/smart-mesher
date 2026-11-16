//! Abaqus `.inp` export: `*NODE`, `*ELEMENT` blocks per element type, `*ELSET`, `*NSET` and
//! element-based `*SURFACE`.
//!
//! [`write_abaqus`] writes a flat model-data deck (no `*PART`, no sections or materials, which
//! belong to the analysis, not to the mesh). Tets are `C3D4`, hexahedra `C3D8`, wedges `C3D6`;
//! pyramids follow [`PyramidPolicy`] because Abaqus has no pyramid element it documents;
//! polyhedra follow [`PolyPolicy`]. Cell sets become `*ELSET`, face and side sets become
//! element-based `*SURFACE`s (`S1`..`S6` by the Abaqus face tables), vertex sets `*NSET`; with
//! [`AbaqusOptions::nsets`] every cell and face set also gets a node set of the same name.
//!
//! Node `n` is vertex `n - 1`; element ids are the source cell index + 1 (gaps where cells are
//! dropped), decomposition tets follow after the last cell id. The node order per type, the face
//! tables and the limits are in `ABAQUS_FORMAT.md`. Output is a pure function of the mesh and
//! options: bit-identical for any thread count.

mod write;

pub use write::write_abaqus;

pub use crate::poly::PolyPolicy;
use crate::report::LossReport;
use serde::{Deserialize, Serialize};
use sm_mesh_api::MeshError;
use std::collections::BTreeMap;

/// What to do with pyramids.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum PyramidPolicy {
    /// Fail with [`AbaqusError::PyramidsUnsupported`].
    #[default]
    Error,
    /// Write `C3D5` (nodes 1-4 the base counter-clockwise seen from the apex side, node 5 the
    /// apex). The element name is not confirmed by Abaqus documentation available to us and no
    /// reader in the conformance environment accepts it (meshio does not), so the report carries
    /// `abaqus.pyramid_type_unverified`.
    C3d5,
    /// Write a `C3D8` with nodes 5 to 8 all at the apex. Abaqus accepts it; the Jacobian is
    /// singular at the apex corner, reported as `abaqus.pyramid_degenerate_hex`.
    DegenerateHex,
}

/// Options of [`write_abaqus`]. JSON shape:
/// `{"polyhedra":"error"|"decompose"|"drop","pyramids":"error"|"c3d5"|"degenerate_hex","nsets":true}`;
/// every field is optional and unknown keys are refused.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[non_exhaustive]
pub struct AbaqusOptions {
    /// Policy for polyhedral cells.
    pub polyhedra: PolyPolicy,
    /// Policy for pyramids.
    pub pyramids: PyramidPolicy,
    /// Also write a `*NSET` (the nodes of its elements or faces) for every cell, face and side
    /// set, under the set's own name.
    pub nsets: bool,
}

impl AbaqusOptions {
    /// These options with the given polyhedra policy.
    pub fn with_polyhedra(mut self, polyhedra: PolyPolicy) -> Self {
        self.polyhedra = polyhedra;
        self
    }

    /// These options with the derived node sets on or off.
    pub fn with_nsets(mut self, nsets: bool) -> Self {
        self.nsets = nsets;
        self
    }

    /// These options with the given pyramid policy.
    pub fn with_pyramids(mut self, pyramids: PyramidPolicy) -> Self {
        self.pyramids = pyramids;
        self
    }
}

impl Default for AbaqusOptions {
    fn default() -> Self {
        Self { polyhedra: PolyPolicy::Error, pyramids: PyramidPolicy::Error, nsets: true }
    }
}

/// What a write produced.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct AbaqusReport {
    /// Nodes written (mesh vertices plus any added by decomposition).
    pub nodes: u64,
    /// Elements written, by Abaqus element type name.
    pub elements_by_type: BTreeMap<String, u64>,
    /// `*ELSET` blocks written.
    pub elsets: u64,
    /// `*NSET` blocks written.
    pub nsets: u64,
    /// `*SURFACE` blocks written.
    pub surfaces: u64,
    /// What the format could not carry (`abaqus.*` codes).
    pub loss: LossReport,
    /// File size in bytes.
    pub bytes: u64,
    /// FNV-1a 64 of the file bytes.
    pub hash: u64,
}

/// Errors of the Abaqus writer.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum AbaqusError {
    /// An I/O error from the output.
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
    /// A mesh-API call failed.
    #[error(transparent)]
    Mesh(#[from] MeshError),
    /// The mesh uses something the writer does not handle (dimension-2 meshes, inactive cells,
    /// non-finite coordinates, ...).
    #[error("unsupported: {0}")]
    Unsupported(String),
    /// The mesh has polyhedral cells and the policy is [`PolyPolicy::Error`].
    #[error("{count} polyhedral cells cannot be written to an Abaqus deck (set `polyhedra` to decompose or drop)")]
    PolyhedraUnsupported {
        /// Number of polyhedra.
        count: u64,
    },
    /// The mesh has pyramids and the policy is [`PyramidPolicy::Error`].
    #[error("{count} pyramid cells have no confirmed Abaqus element type (set `pyramids` to c3d5 or degenerate_hex)")]
    PyramidsUnsupported {
        /// Number of pyramids.
        count: u64,
    },
    /// A decomposed polyhedron shares a polygonal face with a cell that is not decomposed.
    #[error("face {face}: a decomposed polyhedron meets a cell that is not decomposed")]
    NonconformingDecomposition {
        /// Face index.
        face: u32,
    },
    /// More nodes or elements than the writer's 32-bit ids allow.
    #[error("the mesh is too large for the Abaqus writer: {0}")]
    TooLarge(&'static str),
}
