//! [`BrepModel`]: the plain, ordered result of reading a STEP file, independent of any
//! topology store.
//!
//! Every list is in ascending STEP id order. Coordinates are in the solid's target length unit
//! (the file unit unless `to_unit` was given). Each [`Solid`] is self-contained (own vertices,
//! edges, curves and surfaces): the reader never merges cells across solids.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::Point3;
use sm_geom_api::{CurveDef, Interval, SurfaceDef};
use sm_topo_api::{Sense, ShellKind};

/// A length unit the importer converts to and from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize, JsonSchema)]
pub enum LengthUnit {
    /// Micrometre.
    #[serde(rename = "um")]
    Um,
    /// Millimetre.
    #[serde(rename = "mm")]
    Mm,
    /// Centimetre.
    #[serde(rename = "cm")]
    Cm,
    /// Metre.
    #[serde(rename = "m")]
    M,
    /// Inch.
    #[serde(rename = "in")]
    In,
    /// Foot.
    #[serde(rename = "ft")]
    Ft,
}

impl LengthUnit {
    /// Micrometres per unit (exact integers, so conversions such as inch to millimetre are
    /// correctly rounded divisions).
    pub const fn micrometres(self) -> f64 {
        match self {
            Self::Um => 1.0,
            Self::Mm => 1000.0,
            Self::Cm => 10_000.0,
            Self::M => 1_000_000.0,
            Self::In => 25_400.0,
            Self::Ft => 304_800.0,
        }
    }

    /// The short name used in op parameters.
    pub const fn symbol(self) -> &'static str {
        match self {
            Self::Um => "um",
            Self::Mm => "mm",
            Self::Cm => "cm",
            Self::M => "m",
            Self::In => "in",
            Self::Ft => "ft",
        }
    }
}

/// Severity of a [`Note`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoteSeverity {
    /// Informational.
    Info,
    /// Worth a look; the import still succeeded.
    Warning,
}

/// Codes of the diagnostics the reader and the op emit (`step.*`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[non_exhaustive]
pub enum NoteCode {
    /// `step.units`: the detected and applied units.
    Units,
    /// `step.inverted_solid`: a solid with inward normals was flipped.
    InvertedSolid,
    /// `step.welded`: vertices and duplicate edges merged.
    Welded,
    /// `step.sheets_skipped`: surface models and open shells not imported.
    SheetsSkipped,
    /// `step.unsupported_entity`: solid-like entities that are not imported.
    UnsupportedEntity,
    /// `step.validate`: an issue found by our validators.
    Validate,
    /// `step.edge_gap`: a vertex off its edge curve by more than the weld tolerance.
    EdgeGap,
    /// `step.edge_sense`: an `EDGE_CURVE.same_sense` flag contradicted the vertices and was
    /// repaired from the geometry.
    EdgeSense,
    /// `step.assembly_transforms_ignored`: the file is an assembly whose placements could not be
    /// applied; those solids stay in local coordinates.
    AssemblyTransformsIgnored,
    /// `step.assembly_flattened`: assembly placements were applied, one solid per instance.
    AssemblyFlattened,
    /// `step.attributes`: counts of colors, PMI, tessellation and similar entities not imported.
    Attributes,
}

impl NoteCode {
    /// The stable code string.
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Units => "step.units",
            Self::InvertedSolid => "step.inverted_solid",
            Self::Welded => "step.welded",
            Self::SheetsSkipped => "step.sheets_skipped",
            Self::UnsupportedEntity => "step.unsupported_entity",
            Self::Validate => "step.validate",
            Self::EdgeGap => "step.edge_gap",
            Self::EdgeSense => "step.edge_sense",
            Self::AssemblyTransformsIgnored => "step.assembly_transforms_ignored",
            Self::AssemblyFlattened => "step.assembly_flattened",
            Self::Attributes => "step.attributes",
        }
    }
}

/// A diagnostic from reading.
#[derive(Clone, Debug, PartialEq)]
pub struct Note {
    /// Severity.
    pub severity: NoteSeverity,
    /// Code.
    pub code: NoteCode,
    /// Message.
    pub message: String,
    /// STEP ids the message is about.
    pub step_ids: Vec<u64>,
}

impl Note {
    pub(crate) fn info(code: NoteCode, message: String, step_ids: Vec<u64>) -> Self {
        Self { severity: NoteSeverity::Info, code, message, step_ids }
    }

    pub(crate) fn warn(code: NoteCode, message: String, step_ids: Vec<u64>) -> Self {
        Self { severity: NoteSeverity::Warning, code, message, step_ids }
    }
}

/// Options of [`crate::read_step`].
#[derive(Clone, Debug, PartialEq)]
pub struct ReadOptions {
    /// Convert coordinates to this unit. `None` keeps the file unit (no scaling).
    pub to_unit: Option<LengthUnit>,
    /// The unit to use when the file declares none. `None` makes a file without units an error.
    pub assume_unit: Option<LengthUnit>,
    /// Vertices closer than this (target units) are welded.
    pub weld_tol: f64,
    /// Import only these solids (0-based ordinals in ascending STEP id). `None` imports all.
    pub solids: Option<Vec<u32>>,
    /// Import sheets (`SHELL_BASED_SURFACE_MODEL`): free faces grouped in one body per model,
    /// listed after the solids. Their shells need not be closed.
    pub sheets: bool,
    /// Apply assembly placements (`MAPPED_ITEM`, `ITEM_DEFINED_TRANSFORMATION`): one solid per
    /// instance, in the root representation's coordinates. When `false`, or when a placement
    /// cannot be applied, solids stay local and `step.assembly_transforms_ignored` is raised.
    pub flatten_assemblies: bool,
}

impl Default for ReadOptions {
    fn default() -> Self {
        Self { to_unit: None, assume_unit: None, weld_tol: 1e-6, solids: None, sheets: false, flatten_assemblies: true }
    }
}

/// The units a solid was read with.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitReport {
    /// Name of the file's length unit (`millimetre`, `inch`, ...).
    pub length_name: String,
    /// Micrometres per file length unit.
    pub length_um: f64,
    /// False when the file declared no length unit and `assume_unit` was used.
    pub declared: bool,
    /// Name of the file's plane angle unit.
    pub angle_name: String,
    /// Radians per file angle unit.
    pub angle_rad: f64,
    /// The file's distance uncertainty in file length units, if declared.
    pub uncertainty: Option<f64>,
    /// Factor applied to file lengths (1 when no conversion).
    pub scale: f64,
}

/// A vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct Vertex {
    /// STEP id of the `VERTEX_POINT` (the lowest of the welded group).
    pub id: u64,
    /// Position in target units.
    pub point: Point3,
}

/// A decoded curve.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveRec {
    /// STEP id of the geometry entity.
    pub id: u64,
    /// The curve.
    pub def: CurveDef,
}

/// A decoded surface.
#[derive(Clone, Debug, PartialEq)]
pub struct SurfaceRec {
    /// STEP id of the geometry entity.
    pub id: u64,
    /// The surface.
    pub def: SurfaceDef,
}

/// An edge: vertices, carrier curve and parameter range.
#[derive(Clone, Debug, PartialEq)]
pub struct Edge {
    /// STEP id of the `EDGE_CURVE` (the lowest of a merged group).
    pub id: u64,
    /// Start vertex index.
    pub start: usize,
    /// End vertex index (equal to `start` for a closed edge).
    pub end: usize,
    /// Curve index.
    pub curve: usize,
    /// Parameter range on the curve, ascending.
    pub range: Interval,
    /// `Forward` if the edge starts at `range.lo`.
    pub sense: Sense,
    /// Ids of duplicate `EDGE_CURVE`s merged into this edge, ascending.
    pub merged: Vec<u64>,
}

/// One use of an edge in a loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Coedge {
    /// Edge index.
    pub edge: usize,
    /// `Forward` traverses the edge from its start to its end.
    pub sense: Sense,
}

/// A face: surface, carrier sense and loops (loop 0 is the outer loop). Loops are counter-
/// clockwise seen from outside the material, as in the cell complex.
#[derive(Clone, Debug, PartialEq)]
pub struct Face {
    /// STEP id of the `ADVANCED_FACE`.
    pub id: u64,
    /// Surface index.
    pub surface: usize,
    /// Carrier sense (`ADVANCED_FACE.same_sense`, flipped for reversed shells).
    pub sense: Sense,
    /// Loops of coedges.
    pub loops: Vec<Vec<Coedge>>,
    /// Index of the shell holding the face.
    pub shell: usize,
}

/// A shell of faces.
#[derive(Clone, Debug, PartialEq)]
pub struct Shell {
    /// STEP id of the `CLOSED_SHELL`.
    pub id: u64,
    /// Outer or void.
    pub kind: ShellKind,
    /// Face indices, ascending.
    pub faces: Vec<usize>,
}

/// Which STEP entity a solid came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolidKind {
    /// `MANIFOLD_SOLID_BREP`.
    Manifold,
    /// `BREP_WITH_VOIDS`.
    WithVoids,
    /// `FACETED_BREP`: planar facets bounded by `POLY_LOOP`s. Planes and straight edges are
    /// synthesized from the points.
    Faceted,
    /// `SHELL_BASED_SURFACE_MODEL`: faces with no volume (only with `ReadOptions::sheets`).
    Sheet,
}

/// One solid with its own cells and geometry.
#[derive(Clone, Debug, PartialEq)]
pub struct Solid {
    /// STEP id of the solid entity.
    pub id: u64,
    /// Instance ordinal when assembly flattening places the solid more than once (0 otherwise).
    pub instance: u32,
    /// Its name attribute.
    pub name: String,
    /// Entity kind.
    pub kind: SolidKind,
    /// Units and scale.
    pub units: UnitReport,
    /// Vertices.
    pub vertices: Vec<Vertex>,
    /// Curves.
    pub curves: Vec<CurveRec>,
    /// Surfaces.
    pub surfaces: Vec<SurfaceRec>,
    /// Edges.
    pub edges: Vec<Edge>,
    /// Faces, ascending by id.
    pub faces: Vec<Face>,
    /// Shells; the outer shell is first.
    pub shells: Vec<Shell>,
}

/// The decoded B-rep content of a STEP file.
#[derive(Clone, Debug, PartialEq)]
pub struct BrepModel {
    /// First schema identifier of the header.
    pub schema: String,
    /// The imported solids, ascending by STEP id.
    pub solids: Vec<Solid>,
    /// Diagnostics, in the order they were produced.
    pub notes: Vec<Note>,
}
