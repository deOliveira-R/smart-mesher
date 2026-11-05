//! Gmsh MSH 4.1 export (ASCII or binary) and a small parser for round-trip tests.
//!
//! [`write_msh4`] writes `$MeshFormat`, `$PhysicalNames`, `$Entities`, `$Nodes` and `$Elements`
//! from a classified [`sm_mesh_api::MeshRead`]. Gmsh attaches physical groups to ENTITIES, not
//! elements, so an entity is one classification entity plus one set-membership signature:
//! a cell set that holds part of a `TopoVolume` splits it into several Gmsh volumes
//! (`msh.entity_split`). Tetrahedra, hexahedra, wedges and pyramids, triangles, quads, lines
//! and points are written; polyhedra follow [`PolyPolicy`] (Gmsh's polyhedron types 34 and 35
//! are dropped by msh4 and unreadable, see `docs/design/w3/gmsh-polyhedra.md`).
//!
//! Output is a pure function of the mesh and options: ASCII reals are shortest round-trip,
//! binary is little-endian with `size_t` = 8.

mod parse;
mod write;

#[cfg(test)]
mod tests;

pub use parse::{Msh4Data, Msh4ElementBlock, Msh4Entity, Msh4NodeBlock, parse_msh4};
pub use write::write_msh4;

use crate::report::LossReport;
use serde::{Deserialize, Serialize};
use sm_mesh_api::MeshError;
use std::collections::BTreeMap;

/// What to do with polyhedral cells, which msh4 cannot carry.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum PolyPolicy {
    /// Fail with [`GmshError::PolyhedraUnsupported`].
    #[default]
    Error,
    /// Split each polyhedron into tetrahedra around a cell centroid and face centroids (new
    /// nodes after the original ones). Allowed only when every neighbour across a polygonal
    /// face is also decomposed, else [`GmshError::NonconformingDecomposition`].
    Decompose,
    /// Omit polyhedra and report `msh.poly_dropped`.
    Drop,
}

/// Options of the msh4 writer.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[non_exhaustive]
pub struct GmshOptions {
    /// Binary instead of ASCII.
    pub binary: bool,
    /// Policy for polyhedral cells.
    pub polyhedra: PolyPolicy,
    /// Write boundary and interface faces (dimension 2 elements).
    pub boundary_elements: bool,
    /// Write classified edges and vertices (dimension 1 and 0 elements).
    pub lines_and_points: bool,
}

impl GmshOptions {
    /// These options with binary output on or off.
    pub fn with_binary(mut self, binary: bool) -> Self {
        self.binary = binary;
        self
    }

    /// These options with the given polyhedra policy.
    pub fn with_polyhedra(mut self, polyhedra: PolyPolicy) -> Self {
        self.polyhedra = polyhedra;
        self
    }
}

impl Default for GmshOptions {
    fn default() -> Self {
        Self { binary: false, polyhedra: PolyPolicy::Error, boundary_elements: true, lines_and_points: true }
    }
}

/// What a write produced.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct GmshReport {
    /// Nodes written (mesh vertices plus any added by decomposition).
    pub nodes: u64,
    /// Elements written by Gmsh element type.
    pub elements_by_type: BTreeMap<u8, u64>,
    /// Entities written by dimension (points, curves, surfaces, volumes).
    pub entities: [u64; 4],
    /// Physical groups written.
    pub physical_groups: u64,
    /// What the format could not carry (`msh.*` codes).
    pub loss: LossReport,
    /// File size in bytes.
    pub bytes: u64,
    /// FNV-1a 64 of the file bytes.
    pub hash: u64,
}

/// Errors of the msh4 writer and parser.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum GmshError {
    /// An I/O error from the output.
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
    /// A mesh-API call failed.
    #[error(transparent)]
    Mesh(#[from] MeshError),
    /// The mesh uses something the writer does not handle (dimension-2 meshes, partition
    /// classification, inactive cells, ...).
    #[error("unsupported: {0}")]
    Unsupported(String),
    /// The mesh has polyhedral cells and the policy is [`PolyPolicy::Error`].
    #[error("{count} polyhedral cells cannot be written to msh4 ({hint})")]
    PolyhedraUnsupported {
        /// Number of polyhedra.
        count: u64,
        /// Where to export instead.
        hint: &'static str,
    },
    /// A decomposed polyhedron shares a polygonal face with a cell that is not decomposed.
    #[error("face {face}: a decomposed polyhedron meets a cell that is not decomposed")]
    NonconformingDecomposition {
        /// Face index.
        face: u32,
    },
    /// A set label is not a positive 32-bit integer or repeats within a dimension.
    #[error("set {set:?}: label {label} is not usable as a physical tag")]
    BadLabel {
        /// Set name.
        set: String,
        /// The label.
        label: i64,
    },
    /// A set name contains a double quote or a newline, which `$PhysicalNames` cannot carry.
    #[error("set name {0:?} cannot be written to $PhysicalNames")]
    BadName(String),
    /// The input of [`parse_msh4`] is not valid msh4.
    #[error("malformed msh4: {0}")]
    Malformed(String),
}
