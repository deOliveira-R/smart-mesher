//! OpenFOAM `polyMesh` (ASCII) from the face-first IR, an in-process parser and a topology checker.
//!
//! The IR's rule (the loop's right-hand normal points from `left` to `right`; `left` is the lower
//! cell) is OpenFOAM's owner / neighbour / outward-normal rule, so [`build_polymesh`] is a face
//! reordering plus patch and zone assignment: no face matching, no cell renumbering. polyMesh
//! stores no cell shapes, so a true 6-node prism is five faces and polyhedra stay exact.
//!
//! * [`build_polymesh`]: [`FoamOptions`] + `&dyn MeshRead` -> [`FoamBuild`] ([`PolyMesh`], index maps,
//!   [`FoamReport`] with a [`LossReport`](crate::LossReport)).
//! * [`PolyMesh::files`], [`PolyMesh::write_dir`], [`PolyMesh::write_tar`], [`write_polymesh`].
//! * [`parse_polymesh`], [`read_polymesh_dir`]: ASCII only, hostile-input safe.
//! * [`check_polymesh`]: checkMesh's topology and orientation checks, in process.
//!
//! The byte-level conventions are in `FOAM_FORMAT.md`.

mod build;
mod check;
mod parse;
mod tar;
mod write;

pub use build::build_polymesh;
pub use check::{FoamIssue, check_polymesh};
pub use parse::{parse_polymesh, read_polymesh_dir};
pub use write::{FoamFile, write_polymesh};

use crate::report::LossReport;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;

use crate::smsh::DeckRecord;

/// Errors specific to the OpenFOAM exporter, parser and checker.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
pub enum FoamError {
    /// The mesh is not a volume (dimension 3) mesh. Dimension-2 meshes (`empty` patches) are W3b.
    #[error("OpenFOAM export needs a dimension-3 mesh")]
    NotThreeDimensional,
    /// The mesh has refined parents retained inactive; extracting the leaf mesh is not done.
    #[error("{count} inactive (refined parent) cells: extract the leaf mesh first")]
    InactiveCells {
        /// Number of inactive cells.
        count: usize,
    },
    /// A coordinate is NaN or infinite.
    #[error("point {point} has a non-finite coordinate")]
    NonFinite {
        /// Index of the first offending point in the mesh.
        point: u32,
    },
    /// An interior face has `left >= right`, which the mesh contract forbids; it is not flipped.
    #[error("interior face {face} has left >= right")]
    Orientation {
        /// Source face index.
        face: u32,
    },
    /// A named set does not exist in the mesh.
    #[error("no set named '{0}'")]
    UnknownSet(String),
    /// A set cannot be used in the requested role (not a face / side set, holds interior faces,
    /// or a side set with right-side entries, for a patch).
    #[error("set '{set}' cannot be used here: {detail}")]
    IneligibleSet {
        /// Set name.
        set: String,
        /// What is wrong.
        detail: String,
    },
    /// Two explicitly listed patch sets claim the same face.
    #[error("face {face} is in both '{a}' and '{b}'")]
    PatchOverlap {
        /// Source face index.
        face: u32,
        /// The set listed first.
        a: String,
        /// The set listed later.
        b: String,
    },
    /// A patch or zone name is empty after sanitizing.
    #[error("name '{0}' is empty after sanitizing (allowed: A-Z a-z 0-9 _ . + -)")]
    BadName(String),
    /// Two patches (or two zones of one kind) have the same name after sanitizing.
    #[error("two entries are called '{0}' after sanitizing")]
    DuplicateName(String),
    /// A syntax error in a polyMesh file.
    #[error("{file}:{line}: {detail}")]
    Parse {
        /// File name.
        file: String,
        /// 1-based line.
        line: usize,
        /// What was expected.
        detail: String,
    },
    /// Valid OpenFOAM that this reader does not handle (binary format, cyclic patches, ...).
    #[error("unsupported: {0}")]
    Unsupported(String),
    /// Well-formed text that is not a consistent polyMesh (counts, index ranges, missing files).
    #[error("malformed polyMesh: {0}")]
    Malformed(String),
}

/// OpenFOAM patch types this crate writes and reads.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum FoamPatchType {
    /// A plain `patch`.
    #[default]
    Patch,
    /// A `wall` (written with `inGroups List<word> 1(wall)`).
    Wall,
    /// A `symmetryPlane` (needs exactly planar faces).
    SymmetryPlane,
    /// A `symmetry` patch.
    Symmetry,
    /// An `empty` patch (user-selected only; never inferred).
    Empty,
}

impl FoamPatchType {
    /// The OpenFOAM type word: `patch`, `wall`, `symmetryPlane`, `symmetry`, `empty`.
    pub const fn as_str(self) -> &'static str {
        match self {
            FoamPatchType::Patch => "patch",
            FoamPatchType::Wall => "wall",
            FoamPatchType::SymmetryPlane => "symmetryPlane",
            FoamPatchType::Symmetry => "symmetry",
            FoamPatchType::Empty => "empty",
        }
    }

    /// Inverse of [`FoamPatchType::as_str`].
    pub fn from_word(w: &str) -> Option<Self> {
        Some(match w {
            "patch" => FoamPatchType::Patch,
            "wall" => FoamPatchType::Wall,
            "symmetryPlane" => FoamPatchType::SymmetryPlane,
            "symmetry" => FoamPatchType::Symmetry,
            "empty" => FoamPatchType::Empty,
            _ => return None,
        })
    }
}

/// One explicitly requested patch: a face (or side) set, an optional patch name and a type.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PatchSpec {
    /// Name of a `Faces` or `Sides` set of the mesh.
    pub set: String,
    /// Patch name; the set name when absent.
    #[serde(default)]
    pub name: Option<String>,
    /// Patch type.
    #[serde(default)]
    pub patch_type: FoamPatchType,
}

impl PatchSpec {
    /// A patch named after `set`, of type `patch`.
    pub fn new(set: impl Into<String>) -> Self {
        Self { set: set.into(), name: None, patch_type: FoamPatchType::Patch }
    }
    /// Rename the patch.
    pub fn named(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }
    /// Set the patch type.
    pub fn of_type(mut self, t: FoamPatchType) -> Self {
        self.patch_type = t;
        self
    }
}

/// Where the patches come from.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PatchSource {
    /// Every eligible `Faces` / `Sides` set becomes a patch (set id order, first claim wins).
    #[default]
    Sets,
    /// Only these sets, in this order; an overlap is an error.
    Explicit(Vec<PatchSpec>),
    /// One patch `topo_face_<serial>` per model face of the boundary classification.
    Classification,
}

/// Which cell sets become `cellZones`.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ZoneSelect {
    /// Every non-empty cell set.
    #[default]
    All,
    /// No cell zones.
    None,
    /// The named cell sets (written in set id order).
    Names(Vec<String>),
}

/// Options of [`build_polymesh`]. JSON shape:
/// `{"patches":"sets"|"classification"|{"explicit":[{"set":"xmin","name":"inlet","patch_type":"patch"}]},
/// "cell_zones":"all"|"none"|{"names":["b1"]},"face_zones":["s"],"decks":[...]}`; every field
/// is optional.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
#[non_exhaustive]
pub struct FoamOptions {
    /// Patch source.
    pub patches: PatchSource,
    /// Name of the patch collecting boundary faces no set claims.
    pub default_patch: String,
    /// Type of that patch.
    pub default_patch_type: FoamPatchType,
    /// Cell zones.
    pub cell_zones: ZoneSelect,
    /// Names of `Faces` / `Sides` sets written as `faceZones` (opt-in, never automatic).
    pub face_zones: Vec<String>,
    /// Deck records of the model (a mirror deck's face set becomes a `symmetryPlane` when planar).
    pub decks: Vec<DeckRecord>,
}

impl Default for FoamOptions {
    fn default() -> Self {
        Self {
            patches: PatchSource::Sets,
            default_patch: "defaultFaces".into(),
            default_patch_type: FoamPatchType::Patch,
            cell_zones: ZoneSelect::All,
            face_zones: Vec::new(),
            decks: Vec::new(),
        }
    }
}

impl FoamOptions {
    /// The defaults (`Sets`, `defaultFaces`, all cell zones, no face zones, no decks).
    pub fn new() -> Self {
        Self::default()
    }
    /// Choose the patch source.
    pub fn patches(mut self, p: PatchSource) -> Self {
        self.patches = p;
        self
    }
    /// Name the default patch.
    pub fn default_patch(mut self, name: impl Into<String>) -> Self {
        self.default_patch = name.into();
        self
    }
    /// Type of the default patch.
    pub fn default_patch_type(mut self, t: FoamPatchType) -> Self {
        self.default_patch_type = t;
        self
    }
    /// Choose the cell zones.
    pub fn cell_zones(mut self, z: ZoneSelect) -> Self {
        self.cell_zones = z;
        self
    }
    /// Write these face / side sets as `faceZones`.
    pub fn face_zones(mut self, names: Vec<String>) -> Self {
        self.face_zones = names;
        self
    }
    /// Attach all deck records.
    pub fn decks(mut self, d: Vec<DeckRecord>) -> Self {
        self.decks = d;
        self
    }
    /// Attach one deck record.
    pub fn deck(mut self, d: DeckRecord) -> Self {
        self.decks.push(d);
        self
    }
}

/// Where a patch came from.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum PatchOrigin {
    /// A face or side set of that name.
    Set(String),
    /// The boundary classification on this model face.
    Classification(#[serde(with = "entity_text")] EntityId),
    /// The `defaultFaces` patch.
    Default,
    /// The `nonConformal` patch.
    NonConformal,
    /// The `partitionCut` patch.
    PartitionCut,
    /// Read from a `boundary` file: the origin is not recorded there.
    Parsed,
}

/// Serde for [`EntityId`] as its `Kind#serial` text.
mod entity_text {
    use serde::{Deserialize, Deserializer, Serializer};
    use sm_core::EntityId;

    pub fn serialize<S: Serializer>(id: &EntityId, s: S) -> Result<S::Ok, S::Error> {
        s.collect_str(id)
    }
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<EntityId, D::Error> {
        let t = String::deserialize(d)?;
        t.parse::<EntityId>().map_err(|_| serde::de::Error::custom(format!("bad entity id '{t}'")))
    }
}

/// One boundary patch of a [`PolyMesh`]: the faces `start_face..start_face + n_faces`.
///
/// Equality compares name, type, start and size only: [`PolyPatch::origin`] is provenance that a
/// `boundary` file cannot hold, so a parsed mesh equals the mesh that was written.
#[derive(Clone, Debug)]
pub struct PolyPatch {
    /// Patch name (sanitized).
    pub name: String,
    /// Patch type.
    pub patch_type: FoamPatchType,
    /// First face (foam numbering).
    pub start_face: u32,
    /// Number of faces.
    pub n_faces: u32,
    /// Where the faces came from (provenance only).
    pub origin: PatchOrigin,
}

impl PartialEq for PolyPatch {
    fn eq(&self, o: &Self) -> bool {
        self.name == o.name && self.patch_type == o.patch_type && self.start_face == o.start_face && self.n_faces == o.n_faces
    }
}

/// A `cellZone`: ascending cell labels.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PolyZone {
    /// Zone name.
    pub name: String,
    /// Cell indices, ascending.
    pub labels: Vec<u32>,
}

/// A `faceZone`: ascending foam face labels with a flip flag per face (`true` = the zone's
/// orientation is the face's reverse).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PolyFaceZone {
    /// Zone name.
    pub name: String,
    /// Foam face indices, ascending.
    pub labels: Vec<u32>,
    /// One flag per label.
    pub flips: Vec<bool>,
}

/// An OpenFOAM polyMesh: points, a face CSR, owner / neighbour, patches and zones.
///
/// Faces are ordered `[internal][patch 0][patch 1]...`; internal faces are sorted by
/// `(owner, neighbour)` (upper triangular order); every face loop is outward from its owner.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PolyMesh {
    /// Points.
    pub points: Vec<[f64; 3]>,
    /// CSR offsets into `face_points`, `n_faces + 1` entries.
    pub face_offsets: Vec<u64>,
    /// Concatenated face loops (point indices).
    pub face_points: Vec<u32>,
    /// Owner cell of every face.
    pub owner: Vec<u32>,
    /// Neighbour cell of every internal face (`n_internal` entries).
    pub neighbour: Vec<u32>,
    /// Number of cells.
    pub n_cells: u32,
    /// Boundary patches, in file order.
    pub patches: Vec<PolyPatch>,
    /// Cell zones.
    pub cell_zones: Vec<PolyZone>,
    /// Face zones.
    pub face_zones: Vec<PolyFaceZone>,
}

impl PolyMesh {
    /// Number of faces.
    pub fn n_faces(&self) -> usize {
        self.owner.len()
    }
    /// Number of internal faces.
    pub fn n_internal(&self) -> usize {
        self.neighbour.len()
    }
    /// Point indices of face `i`. Panics if `i` is out of range or the CSR is inconsistent
    /// (meshes from [`build_polymesh`] and [`parse_polymesh`] never are).
    pub fn face(&self, i: usize) -> &[u32] {
        &self.face_points[self.face_offsets[i] as usize..self.face_offsets[i + 1] as usize]
    }
}

/// Cell counts under OpenFOAM `checkMesh` names, derived from [`sm_mesh_api::CellShape`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ShapeCounts {
    /// `CellShape::Hex`.
    pub hexahedra: u64,
    /// `CellShape::Wedge` (two triangles and three quads).
    pub prisms: u64,
    /// `CellShape::Pyramid`.
    pub pyramids: u64,
    /// `CellShape::Tet`.
    pub tetrahedra: u64,
    /// `CellShape::Polyhedron`.
    pub polyhedra: u64,
}

/// A patch as reported.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct PatchSummary {
    /// Patch name.
    pub name: String,
    /// Patch type.
    pub patch_type: FoamPatchType,
    /// First face (foam numbering).
    pub start_face: u32,
    /// Number of faces.
    pub n_faces: u32,
    /// Where the faces came from.
    pub origin: PatchOrigin,
}

/// What an export produced and lost. Serialized as `expected.json` by the `foam_fixtures` example.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FoamReport {
    /// Points written.
    pub n_points: u64,
    /// Cells.
    pub n_cells: u64,
    /// Faces.
    pub n_faces: u64,
    /// Internal faces.
    pub n_internal_faces: u64,
    /// Cell census under checkMesh names.
    pub shape_counts: ShapeCounts,
    /// Patches in file order.
    pub patches: Vec<PatchSummary>,
    /// What the format could not carry.
    pub loss: LossReport,
}

/// The result of [`build_polymesh`].
#[derive(Clone, Debug)]
pub struct FoamBuild {
    /// The polyMesh.
    pub mesh: PolyMesh,
    /// Source face index of every foam face.
    pub face_of_foam: Vec<u32>,
    /// Source vertex index of every foam point.
    pub point_of_foam: Vec<u32>,
    /// Counts, patches and losses.
    pub report: FoamReport,
}

/// Replace every character outside `[A-Za-z0-9_.+-]` by `_`. Returns `BadName` for a result that
/// is empty. Used for patch and zone names.
pub(crate) fn sanitize(name: &str) -> Result<String, FoamError> {
    let s: String = name.chars().map(|c| if c.is_ascii_alphanumeric() || matches!(c, '_' | '.' | '+' | '-') { c } else { '_' }).collect();
    if s.is_empty() { Err(FoamError::BadName(name.to_string())) } else { Ok(s) }
}
