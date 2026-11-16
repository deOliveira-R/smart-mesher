//! Ansys Fluent ASCII `.msh` export (legacy text format, sections 10, 12, 13 and 39).
//!
//! [`write_fluent`] builds the face-first [`crate::foam::PolyMesh`] (faces ordered interior first,
//! then one zone per boundary face set, owner = `left`), renumbers cells by cell zone, and writes
//! nodes, cells and faces with Fluent's "right-hand normal points toward c0" rule. Tets, hexes,
//! wedges and pyramids are labelled only when the cell has exactly the canonical face set; every
//! other cell is a polyhedron (type 7). No 58/59 hanging-node sections and no binary variant.
//!
//! The byte-level conventions and the limits (no Fluent licence: OpenFOAM `fluent3DMeshToFoam`
//! and `checkMesh` are the oracle) are in `FLUENT_FORMAT.md`. Output is a pure function of the
//! mesh and options: bit-identical for any thread count.

mod write;

pub use write::write_fluent;

use crate::foam::{PatchSource, ZoneSelect};
use crate::report::LossReport;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Fluent boundary condition type of a face zone: the bc-type code of section 13 and the word of
/// section 39.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
#[non_exhaustive]
pub enum FluentBc {
    /// `wall` (bc 3). The default of every boundary zone.
    #[default]
    Wall,
    /// `symmetry` (bc 7).
    Symmetry,
    /// `pressure-inlet` (bc 4).
    PressureInlet,
    /// `pressure-outlet` (bc 5).
    PressureOutlet,
    /// `velocity-inlet` (bc 10).
    VelocityInlet,
    /// `mass-flow-inlet` (bc 20).
    MassFlowInlet,
    /// `outflow` (bc 36).
    Outflow,
    /// `pressure-far-field` (bc 9).
    PressureFarField,
}

impl FluentBc {
    /// The bc-type code (decimal; the file writes it in hex).
    pub const fn code(self) -> u32 {
        match self {
            FluentBc::Wall => 3,
            FluentBc::Symmetry => 7,
            FluentBc::PressureInlet => 4,
            FluentBc::PressureOutlet => 5,
            FluentBc::VelocityInlet => 10,
            FluentBc::MassFlowInlet => 20,
            FluentBc::Outflow => 36,
            FluentBc::PressureFarField => 9,
        }
    }

    /// The zone-type word of section 39.
    pub const fn word(self) -> &'static str {
        match self {
            FluentBc::Wall => "wall",
            FluentBc::Symmetry => "symmetry",
            FluentBc::PressureInlet => "pressure-inlet",
            FluentBc::PressureOutlet => "pressure-outlet",
            FluentBc::VelocityInlet => "velocity-inlet",
            FluentBc::MassFlowInlet => "mass-flow-inlet",
            FluentBc::Outflow => "outflow",
            FluentBc::PressureFarField => "pressure-far-field",
        }
    }
}

/// Options of [`write_fluent`]. JSON shape:
/// `{"cell_zones":"all"|"none"|{"names":["b1"]},"boundary_types":{"inlet":"pressure-inlet"}}`;
/// every field is optional.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
#[non_exhaustive]
pub struct FluentOptions {
    /// Which cell sets become cell zones. A cell in two sets goes to the first; cells in none go
    /// to the zone `fluid`.
    pub cell_zones: ZoneSelect,
    /// Where the boundary zones come from (face and side sets by default).
    pub patches: PatchSource,
    /// Boundary type per boundary zone, by the face set's name as in the mesh. Zones not listed
    /// are `wall`.
    pub boundary_types: BTreeMap<String, FluentBc>,
}

impl Default for FluentOptions {
    fn default() -> Self {
        Self { cell_zones: ZoneSelect::All, patches: PatchSource::Sets, boundary_types: BTreeMap::new() }
    }
}

/// One zone of the file.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct FluentZone {
    /// Zone id (unique over node, cell and face zones; decimal in section 39).
    pub id: u32,
    /// Zone name after sanitizing.
    pub name: String,
    /// The section-39 type word (`fluid`, `interior`, `wall`, ...).
    pub kind: &'static str,
    /// First entity id (1-based).
    pub first: u64,
    /// Number of cells or faces.
    pub count: u64,
}

/// What a write produced.
#[derive(Clone, Debug, PartialEq)]
#[non_exhaustive]
pub struct FluentReport {
    /// Nodes written.
    pub nodes: u64,
    /// Cells written.
    pub cells: u64,
    /// Faces written.
    pub faces: u64,
    /// Interior faces (the first zone of faces).
    pub interior_faces: u64,
    /// Cells by Fluent element type code (2 tet, 4 hex, 5 pyramid, 6 wedge, 7 polyhedron).
    pub cells_by_type: BTreeMap<u8, u64>,
    /// Cell zones in file order.
    pub cell_zones: Vec<FluentZone>,
    /// Face zones in file order: the interior zone, then the boundary zones.
    pub face_zones: Vec<FluentZone>,
    /// Exported cell (0-based, file order) to source mesh cell.
    pub cell_of_fluent: Vec<u32>,
    /// What the format could not carry (`fluent.*` codes).
    pub loss: LossReport,
    /// File size in bytes.
    pub bytes: u64,
    /// FNV-1a 64 of the file bytes.
    pub hash: u64,
}

/// Errors of the Fluent writer.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum FluentError {
    /// An I/O error from the output.
    #[error("i/o error: {0}")]
    Io(#[from] std::io::Error),
    /// The mesh could not be put in face-first zone order (dimension, inactive cells, sets,
    /// names, non-finite coordinates); the message is the underlying reason.
    #[error("{0}")]
    Mesh(#[from] crate::error::MeshioError),
    /// A `boundary_types` key is no boundary zone of this export.
    #[error("boundary_types names '{0}', which is no boundary zone of this mesh")]
    UnknownBoundary(String),
    /// More entities than the writer's 32-bit ids allow.
    #[error("the mesh is too large for the Fluent writer: {0}")]
    TooLarge(&'static str),
}
