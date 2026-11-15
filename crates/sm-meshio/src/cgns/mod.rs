//! CGNS 4.x export (HDF5 encoding): one unstructured zone per file.
//!
//! [`write_cgns`] builds the face-first [`crate::foam::PolyMesh`] and writes a CGNS/HDF5 tree with
//! the crate's private `h5` writer: `CGNSLibraryVersion`, one `Base`, one unstructured `Zone`,
//! coordinates, element sections, `BC_t` nodes from the face sets (grid location `FaceCenter`, a
//! `PointList`, a `Family_t` that carries the BC type), and a `ZoneSubRegion_t` plus `Family_t`
//! per cell set. When every cell is a tetrahedron, pyramid, wedge or hexahedron the zone uses
//! standard element sections (`TETRA_4`, `PYRA_5`, `PENTA_6`, `HEXA_8`, one per block and type)
//! plus `TRI_3` / `QUAD_4` sections for the boundary faces; otherwise (or on request) it uses
//! `NGON_n` and `NFACE_n` with `ElementStartOffset`, and signed face references in the cells.
//!
//! The node tree, the data types and the limits are in `CGNS_FORMAT.md`. The CGNS library
//! (`cgnscheck`, `cgnslist`) is the oracle. Output is a pure function of the mesh and options:
//! bit-identical for any thread count.

mod write;

pub use write::write_cgns;

use crate::foam::{PatchSource, ZoneSelect};
use crate::h5::H5Error;
use crate::report::LossReport;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// CGNS `BCType_t` of a boundary condition, carried by the `FamilyBC` of its family.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
#[non_exhaustive]
pub enum CgnsBc {
    /// `BCTypeUserDefined`: the default, a face set carries no physics.
    #[default]
    UserDefined,
    /// `BCWall`.
    Wall,
    /// `BCInflow`.
    Inflow,
    /// `BCOutflow`.
    Outflow,
    /// `BCSymmetryPlane`.
    SymmetryPlane,
    /// `BCFarfield`.
    Farfield,
}

impl CgnsBc {
    /// The enumeration word stored in the file.
    pub const fn word(self) -> &'static str {
        match self {
            CgnsBc::UserDefined => "UserDefined",
            CgnsBc::Wall => "BCWall",
            CgnsBc::Inflow => "BCInflow",
            CgnsBc::Outflow => "BCOutflow",
            CgnsBc::SymmetryPlane => "BCSymmetryPlane",
            CgnsBc::Farfield => "BCFarfield",
        }
    }
}

/// How the cells of the zone are stored.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum CgnsLayout {
    /// Standard sections when every cell is a tet, pyramid, wedge or hex, else `Ngon`.
    #[default]
    Auto,
    /// `NGON_n` faces and `NFACE_n` cells for every cell, whatever its shape.
    Ngon,
}

/// Options of [`write_cgns`]. JSON shape:
/// `{"layout":"auto"|"ngon","cell_zones":"all"|"none"|{"names":["b1"]},"boundary_types":{"inlet":"inflow"}}`;
/// every field is optional.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[non_exhaustive]
pub struct CgnsOptions {
    /// Element layout of the zone.
    pub layout: CgnsLayout,
    /// Which cell sets become sections / sub-regions and families. A cell in two sets goes to the
    /// first.
    pub cell_zones: ZoneSelect,
    /// Where the boundary conditions come from (face and side sets by default).
    pub patches: PatchSource,
    /// BC type per boundary condition, by the face set's name as in the mesh. BCs not listed are
    /// `UserDefined`.
    pub boundary_types: BTreeMap<String, CgnsBc>,
}

impl Default for CgnsOptions {
    fn default() -> Self {
        Self { layout: CgnsLayout::Auto, cell_zones: ZoneSelect::All, patches: PatchSource::Sets, boundary_types: BTreeMap::new() }
    }
}

/// One `Elements_t` section of the zone.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct CgnsSection {
    /// Node name after sanitizing.
    pub name: String,
    /// The `ElementType_t` word (`HEXA_8`, `NGON_n`, ...).
    pub kind: &'static str,
    /// First element id (1-based, inclusive).
    pub first: u64,
    /// Last element id (inclusive).
    pub last: u64,
}

/// What a write produced.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct CgnsReport {
    /// Layout actually written (never `Auto`).
    pub layout: CgnsLayout,
    /// Vertices written.
    pub nodes: u64,
    /// Cells written.
    pub cells: u64,
    /// Face elements written: every face under `Ngon`, the boundary faces under standard sections.
    pub faces: u64,
    /// Interior faces of the mesh.
    pub interior_faces: u64,
    /// Cells by CGNS element type word of their face census (`NFACE_n` for polyhedra), also
    /// under `Ngon`.
    pub cells_by_type: BTreeMap<&'static str, u64>,
    /// Element sections in file order.
    pub sections: Vec<CgnsSection>,
    /// Names of the BC nodes (and their families), in file order.
    pub bcs: Vec<String>,
    /// Names of the cell-set sub-regions (and their families), in file order.
    pub blocks: Vec<String>,
    /// Exported cell (0-based, element order) to source mesh cell.
    pub cell_of_cgns: Vec<u32>,
    /// What the format could not carry (`cgns.*` codes).
    pub loss: LossReport,
    /// File size in bytes.
    pub bytes: u64,
    /// FNV-1a 64 of the file bytes.
    pub hash: u64,
}

/// Errors of the CGNS writer.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum CgnsError {
    /// An I/O error from the output.
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
    /// The mesh could not be put in face-first order (dimension, inactive cells, sets, names,
    /// non-finite coordinates); the message is the underlying reason.
    #[error("{0}")]
    Mesh(#[from] crate::error::MeshioError),
    /// A `boundary_types` key is no boundary condition of this export.
    #[error("boundary_types names '{0}', which is no boundary condition of this mesh")]
    UnknownBoundary(String),
    /// The HDF5 writer refused the tree.
    #[error("{0}")]
    Hdf5(#[from] H5Error),
}
