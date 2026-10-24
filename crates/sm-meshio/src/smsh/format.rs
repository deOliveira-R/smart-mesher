//! The `.smsh` header schema and file-layout constants (see `SMSH_FORMAT.md`).

use crate::error::{MeshioError, Result};
use serde::{Deserialize, Serialize};
use sm_core::Transform;
use sm_model_api::Deck;

/// File magic: `SMSH\r\n\x1a\n` (the PNG trick: detects text-mode transfer damage).
pub const MAGIC: [u8; 8] = *b"SMSH\r\n\x1a\n";
/// Alignment of the data region and of every array, in bytes.
pub const ALIGN: u64 = 64;
/// Bytes before the JSON header: magic plus the `u64` header length.
pub const PREAMBLE: u64 = 16;
/// Format major version written by this crate. A reader rejects a different major.
pub const VERSION_MAJOR: u32 = 1;
/// Format minor version written by this crate. Minor bumps only add optional fields and arrays.
pub const VERSION_MINOR: u32 = 0;

/// Round `x` up to a multiple of [`ALIGN`].
pub(crate) const fn align_up(x: u64) -> u64 {
    x.div_ceil(ALIGN) * ALIGN
}

/// Version pair of the file format.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Version {
    /// Major version (breaking changes).
    pub major: u32,
    /// Minor version (additive changes).
    pub minor: u32,
}

/// Scalar type of an array.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Dtype {
    /// Unsigned 8-bit.
    U8,
    /// Unsigned 32-bit little-endian.
    U32,
    /// Unsigned 64-bit little-endian.
    U64,
    /// Signed 64-bit little-endian.
    I64,
    /// IEEE-754 binary64 little-endian.
    F64,
}

impl Dtype {
    /// Size of one element in bytes.
    pub const fn size(self) -> usize {
        match self {
            Dtype::U8 => 1,
            Dtype::U32 => 4,
            Dtype::U64 | Dtype::I64 | Dtype::F64 => 8,
        }
    }
}

/// One row of the array table.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArrayInfo {
    /// Unique array name.
    pub name: String,
    /// Element type.
    pub dtype: Dtype,
    /// Shape: `[n]` or `[n, k]` (row-major).
    pub shape: Vec<u64>,
    /// Byte offset from the start of the data region (a multiple of 64).
    pub offset: u64,
    /// Length in bytes (`product(shape) * size`).
    pub nbytes: u64,
    /// CRC-32 (IEEE, zlib-compatible) of the array bytes.
    pub crc32: u32,
}

/// Content hashes of the mesh (`sm-mesh-api` hashes, 16 lowercase hex digits each).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct HashInfo {
    /// `connectivity_hash`.
    pub connectivity: String,
    /// `topology_hash`.
    pub topology: String,
    /// `geometry_hash`.
    pub geometry: String,
}

/// Counts and identity of the mesh.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MeshInfo {
    /// Mesh id in `Kind#serial` text form.
    pub id: String,
    /// Topological dimension, 2 or 3.
    pub dim: u8,
    /// Geometry rule id (`MeshRead::geometry_rule`).
    pub geometry_rule: String,
    /// Number of vertices.
    pub n_vertices: u64,
    /// Number of points in the coordinate arrays (vertices plus high-order extra nodes).
    pub n_points: u64,
    /// Number of faces (dimension 2: facets).
    pub n_faces: u64,
    /// Number of cells.
    pub n_cells: u64,
    /// Number of edges in the edge table (0 = no table).
    pub n_edges: u64,
    /// Content hashes; absent when the writer was told to skip them.
    #[serde(default)]
    pub hashes: Option<HashInfo>,
}

/// Header entry of a named set.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SetInfo {
    /// Set id in `Kind#serial` text form.
    pub id: String,
    /// Unique name.
    pub name: String,
    /// Optional integer id (block / side-set / node-set number).
    pub label: Option<i64>,
    /// Members variant code (`SetMembers::code`): 0 vertices, 1 faces, 2 cells, 3 sides,
    /// 4 shell sides, 5 points.
    pub members: u8,
    /// Role name: `node_set`, `face_set`, `cell_set`, `side_set`, `shell_side_set`, `point_set`.
    pub role: String,
    /// Name of the `u32` array holding the member words.
    pub array: String,
}

/// Header entry of a tag column.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TagInfo {
    /// Column name.
    pub name: String,
    /// Target entity kind: `vertex`, `edge`, `face` or `cell`.
    pub target: String,
    /// Name of the `i64` or `f64` array with one value per entity.
    pub array: String,
}

/// Header entry of a fine-to-coarse level map.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LevelMapInfo {
    /// Fine mesh id (`Kind#serial`).
    pub fine: String,
    /// Coarse mesh id (`Kind#serial`).
    pub coarse: String,
    /// `connectivity_hash` of the fine mesh when the map was built (hex).
    pub fine_hash: String,
    /// `connectivity_hash` of the coarse mesh when the map was built (hex).
    pub coarse_hash: String,
    /// Array `u32[n_fine_cells]`: coarse cell of every fine cell.
    pub coarse_of: String,
    /// Array `u32[n_fine_faces]`: coarse face carrying each fine face (`0xFFFFFFFF` = none).
    pub face_map: Option<String>,
}

/// Header entry of an extrusion map (base mesh x layers).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExtrusionInfo {
    /// Base (dimension-2) mesh id.
    pub base: String,
    /// Extruded (dimension-3) mesh id.
    pub extruded: String,
    /// `connectivity_hash` of the base mesh when the map was built (hex).
    pub base_hash: String,
    /// `connectivity_hash` of the extruded mesh when the map was built (hex).
    pub extruded_hash: String,
    /// Number of layers.
    pub n_layers: u64,
    /// Array `f64[n_layers + 1]`: level heights.
    pub z: String,
    /// Array `u32[n_extruded_cells]`: base cell of every extruded cell.
    pub cell_base: String,
    /// Array `u32[n_extruded_cells]`: layer of every extruded cell.
    pub cell_layer: String,
}

/// Provenance: the project state that produced the mesh (ir.md section 2 item H).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Provenance {
    /// Op-timeline node hash (opaque text, normally hex) the mesh was generated from.
    pub timeline_hash: String,
}

/// Kind and parameters of a deck transformation (the source of truth; the matrix is derived).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum DeckParams {
    /// Reflection in the plane `normal . x = offset`.
    Mirror {
        /// Unit normal.
        normal: [f64; 3],
        /// Signed offset along the normal.
        offset: f64,
    },
    /// Translation pairing `B = A + vector`.
    Translation {
        /// Shift taking side A to side B.
        vector: [f64; 3],
    },
    /// Rotation by `turns[0] / turns[1]` turns about an axis.
    Rotation {
        /// A point on the axis.
        axis_point: [f64; 3],
        /// Unit direction of the axis.
        axis_dir: [f64; 3],
        /// Numerator and denominator of the angle in turns.
        turns: [i64; 2],
    },
}

/// Deck-transformation metadata carried from the model (ir.md section 5.1): consumers such as
/// the tracker need it to reflect or wrap on a deck boundary.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DeckRecord {
    /// Name of the boundary record.
    pub name: String,
    /// Kind and parameters.
    pub params: DeckParams,
    /// Row-major 3x3 linear part of the transform taking side A to side B.
    pub matrix: [f64; 9],
    /// Translation part.
    pub translation: [f64; 3],
    /// True if the transform reverses orientation (mirror).
    pub improper: bool,
    /// Order of the generated cyclic group, `None` = infinite.
    pub order: Option<u32>,
    /// Name of the set of this file holding side A of the boundary, if any.
    pub face_set: Option<String>,
    /// Name of the set holding the mate side B (`None` for a self-paired mirror).
    pub mate_set: Option<String>,
}

impl DeckRecord {
    /// Build from a model [`Deck`]. Fails with [`MeshioError::Unsupported`] for a deck variant
    /// added to the model API after this crate, and for non-finite parameters.
    pub fn from_deck(name: impl Into<String>, deck: &Deck, face_set: Option<String>, mate_set: Option<String>) -> Result<Self> {
        let v3 = |v: sm_core::Vector3| [v.x, v.y, v.z];
        let params = match deck {
            Deck::Mirror { plane } => DeckParams::Mirror { normal: v3(plane.normal), offset: plane.offset },
            Deck::Translation { vector } => DeckParams::Translation { vector: v3(*vector) },
            Deck::Rotation { axis, angle, .. } => DeckParams::Rotation {
                axis_point: [axis.point.x, axis.point.y, axis.point.z],
                axis_dir: v3(axis.dir),
                turns: [angle.num as i64, angle.den as i64],
            },
            _ => return Err(MeshioError::Unsupported("unknown deck variant".into())),
        };
        let t: Transform = deck.transform();
        Ok(Self {
            name: name.into(),
            params,
            matrix: [t.m[0][0], t.m[0][1], t.m[0][2], t.m[1][0], t.m[1][1], t.m[1][2], t.m[2][0], t.m[2][1], t.m[2][2]],
            translation: [t.t.x, t.t.y, t.t.z],
            improper: deck.is_improper(),
            order: deck.order(),
            face_set,
            mate_set,
        })
    }

    /// True if every stored number is finite (JSON cannot carry NaN or infinity).
    pub(crate) fn is_finite(&self) -> bool {
        let p = match &self.params {
            DeckParams::Mirror { normal, offset } => normal.iter().all(|v| v.is_finite()) && offset.is_finite(),
            DeckParams::Translation { vector } => vector.iter().all(|v| v.is_finite()),
            DeckParams::Rotation { axis_point, axis_dir, .. } => axis_point.iter().chain(axis_dir).all(|v| v.is_finite()),
        };
        p && self.matrix.iter().chain(&self.translation).all(|v| v.is_finite())
    }
}

/// Header entry of the cached geometry (derived data).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct GeometryCacheInfo {
    /// Geometry rule id the values were computed under.
    pub rule: String,
    /// `connectivity_hash` the cache was computed for (hex).
    pub connectivity_hash: String,
    /// `coordinate_hash` the cache was computed for (hex).
    pub coordinate_hash: String,
}

/// The JSON header of a `.smsh` file.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Header {
    /// Always `"smsh"`.
    pub format: String,
    /// Format version.
    pub version: Version,
    /// Always `"little"`.
    pub endian: String,
    /// Writer identification (informational).
    pub producer: String,
    /// Mesh identity, counts and hashes.
    pub mesh: MeshInfo,
    /// Array table, in file order.
    pub arrays: Vec<ArrayInfo>,
    /// Named sets, ascending by id.
    #[serde(default)]
    pub sets: Vec<SetInfo>,
    /// Tag columns, in mesh order.
    #[serde(default)]
    pub tags: Vec<TagInfo>,
    /// Cached geometry, if present.
    #[serde(default)]
    pub geometry_cache: Option<GeometryCacheInfo>,
    /// Fine-to-coarse maps touching this mesh.
    #[serde(default)]
    pub level_maps: Vec<LevelMapInfo>,
    /// Extrusion maps touching this mesh.
    #[serde(default)]
    pub extrusions: Vec<ExtrusionInfo>,
    /// Deck transformations from the model.
    #[serde(default)]
    pub decks: Vec<DeckRecord>,
    /// Provenance, if recorded.
    #[serde(default)]
    pub provenance: Option<Provenance>,
}

impl Header {
    /// The array table row named `name`.
    pub fn array(&self, name: &str) -> Option<&ArrayInfo> {
        self.arrays.iter().find(|a| a.name == name)
    }
}

/// Lowercase 16-digit hex of a hash.
pub(crate) fn hex(h: u64) -> String {
    format!("{h:016x}")
}

/// Cell shape code table (`cell_shape` array).
pub(crate) fn shape_code(s: sm_mesh_api::CellShape) -> Option<u8> {
    use sm_mesh_api::CellShape::*;
    Some(match s {
        Tet => 0,
        Hex => 1,
        Wedge => 2,
        Pyramid => 3,
        Polyhedron => 4,
        Tri => 5,
        Quad => 6,
        Polygon => 7,
        _ => return None,
    })
}

/// Face kind code table (`face_kind` array).
pub(crate) fn face_kind_code(k: sm_mesh_api::FaceKind) -> Option<u8> {
    use sm_mesh_api::FaceKind::*;
    Some(match k {
        Interior => 0,
        Boundary => 1,
        NonConformal => 2,
        PartitionCut => 3,
        _ => return None,
    })
}
