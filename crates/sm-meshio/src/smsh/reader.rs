//! The `.smsh` reader: zero-copy typed views over the bytes, and a full decode used by the
//! round-trip tests.

use super::format::*;
use crate::crc::crc32;
use crate::error::{MeshioError, Result};
use crate::le::{Le, decode};
use rayon::prelude::*;
use sm_core::EntityId;
use sm_mesh_api::{
    Basis, CellIx, CellShape, Classification, EdgeIx, EdgeTable, FaceEdge, FaceIx, FaceKind, GeometryCache, HighOrderCell, MeshDim, MeshEntityKind, MeshSet, NO_CELL, Param,
    PointIx, SetMembers, SignedFace, TagColumn, TagValues, VertexIx,
};
use std::borrow::Cow;

fn bad<T>(s: impl Into<String>) -> Result<T> {
    Err(MeshioError::Malformed(s.into()))
}

/// A parsed `.smsh` file borrowing the caller's bytes (a `Vec`, or a memory map).
///
/// Typed accessors return `Cow::Borrowed` slices straight into the bytes when the host is
/// little-endian and the array is aligned for its type (always true for a mapped file, whose
/// data region starts at a multiple of 64 from a page boundary), and a decoded copy otherwise.
pub struct SmshFile<'a> {
    header: Header,
    data: &'a [u8],
}

impl<'a> SmshFile<'a> {
    /// Parse the preamble and header and validate the array table (bounds, alignment, sizes).
    /// Array contents are not read; call [`Self::verify_checksums`] for that.
    pub fn parse(bytes: &'a [u8]) -> Result<Self> {
        if bytes.len() < PREAMBLE as usize || bytes[..8] != MAGIC {
            return bad("bad magic");
        }
        let hl = u64::from_le_bytes(bytes[8..16].try_into().map_err(|_| MeshioError::Malformed("short preamble".into()))?);
        let start = PREAMBLE.checked_add(hl).filter(|s| *s <= bytes.len() as u64 && s % ALIGN == 0);
        let Some(start) = start else { return bad("header length is out of range or the data region is misaligned") };
        let header: Header = serde_json::from_slice(&bytes[PREAMBLE as usize..start as usize])?;
        if header.format != "smsh" {
            return bad(format!("format tag '{}'", header.format));
        }
        if header.version.major != VERSION_MAJOR {
            return Err(MeshioError::UnsupportedVersion { major: header.version.major, minor: header.version.minor });
        }
        if header.endian != "little" {
            return bad("only little-endian files exist in v1");
        }
        let data = &bytes[start as usize..];
        let mut seen = std::collections::BTreeSet::new();
        for a in &header.arrays {
            let n = a.shape.iter().try_fold(1u64, |p, d| p.checked_mul(*d));
            if n.and_then(|n| n.checked_mul(a.dtype.size() as u64)) != Some(a.nbytes) {
                return bad(format!("array '{}': shape and size disagree", a.name));
            }
            if a.offset % ALIGN != 0 || a.offset.checked_add(a.nbytes).is_none_or(|e| e > data.len() as u64) {
                return bad(format!("array '{}': offset out of range or misaligned", a.name));
            }
            if !seen.insert(a.name.as_str()) {
                return bad(format!("duplicate array '{}'", a.name));
            }
        }
        Ok(Self { header, data })
    }

    /// The header.
    pub fn header(&self) -> &Header {
        &self.header
    }

    /// Raw little-endian bytes of an array.
    pub fn raw(&self, name: &str) -> Result<&'a [u8]> {
        let a = self.header.array(name).ok_or_else(|| MeshioError::Malformed(format!("missing array '{name}'")))?;
        Ok(&self.data[a.offset as usize..(a.offset + a.nbytes) as usize])
    }

    fn typed<T: Le + bytemuck::Pod>(&self, name: &str, want: Dtype) -> Result<Cow<'a, [T]>> {
        let a = self.header.array(name).ok_or_else(|| MeshioError::Malformed(format!("missing array '{name}'")))?;
        if a.dtype != want {
            return bad(format!("array '{name}' is {:?}, expected {want:?}", a.dtype));
        }
        Ok(decode::<T>(self.raw(name)?))
    }

    /// A `u8` array.
    pub fn u8s(&self, name: &str) -> Result<Cow<'a, [u8]>> {
        self.typed(name, Dtype::U8)
    }
    /// A `u32` array.
    pub fn u32s(&self, name: &str) -> Result<Cow<'a, [u32]>> {
        self.typed(name, Dtype::U32)
    }
    /// A `u64` array.
    pub fn u64s(&self, name: &str) -> Result<Cow<'a, [u64]>> {
        self.typed(name, Dtype::U64)
    }
    /// An `i64` array.
    pub fn i64s(&self, name: &str) -> Result<Cow<'a, [i64]>> {
        self.typed(name, Dtype::I64)
    }
    /// An `f64` array.
    pub fn f64s(&self, name: &str) -> Result<Cow<'a, [f64]>> {
        self.typed(name, Dtype::F64)
    }

    /// Check every array against its CRC-32 (parallel, reads all data).
    pub fn verify_checksums(&self) -> Result<()> {
        self.header.arrays.par_iter().try_for_each(|a| {
            let b = &self.data[a.offset as usize..(a.offset + a.nbytes) as usize];
            if crc32(b) == a.crc32 { Ok(()) } else { bad(format!("array '{}': checksum mismatch", a.name)) }
        })
    }

    /// Decode the whole file into owned vectors and API value types.
    pub fn decode(&self) -> Result<SmshMesh> {
        decode_mesh(self)
    }
}

/// A memory-mapped `.smsh` file (feature `mmap`).
#[cfg(feature = "mmap")]
pub struct SmshMap {
    map: memmap2::Mmap,
}

#[cfg(feature = "mmap")]
impl SmshMap {
    /// Map `path` read-only. The file must not be truncated or modified while mapped.
    pub fn open(path: impl AsRef<std::path::Path>) -> Result<Self> {
        let f = std::fs::File::open(path)?;
        // SAFETY: the map is read-only and the contract above forbids concurrent truncation.
        let map = unsafe { memmap2::Mmap::map(&f)? };
        Ok(Self { map })
    }
    /// Parse the mapped bytes.
    pub fn file(&self) -> Result<SmshFile<'_>> {
        SmshFile::parse(&self.map)
    }
}

/// Decoded fine-to-coarse map.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SmshLevelMap {
    /// Fine mesh id.
    pub fine: EntityId,
    /// Coarse mesh id.
    pub coarse: EntityId,
    /// Connectivity hash of the fine mesh at build time.
    pub fine_hash: u64,
    /// Connectivity hash of the coarse mesh at build time.
    pub coarse_hash: u64,
    /// Coarse cell of every fine cell.
    pub coarse_of: Vec<u32>,
    /// Coarse face carrying each fine face, `u32::MAX` for none.
    pub face_map: Option<Vec<u32>>,
}

/// Decoded extrusion map.
#[derive(Clone, Debug, PartialEq)]
pub struct SmshExtrusion {
    /// Base mesh id.
    pub base: EntityId,
    /// Extruded mesh id.
    pub extruded: EntityId,
    /// Connectivity hash of the base mesh at build time.
    pub base_hash: u64,
    /// Connectivity hash of the extruded mesh at build time.
    pub extruded_hash: u64,
    /// Level heights, `n_layers + 1`.
    pub z: Vec<f64>,
    /// Base cell of every extruded cell.
    pub cell_base: Vec<u32>,
    /// Layer of every extruded cell.
    pub cell_layer: Vec<u32>,
}

/// A fully decoded `.smsh` mesh: the file's arrays as owned vectors, in the layout of
/// `SMSH_FORMAT.md`. Used by the round-trip tests and as a convenient in-memory form for tools.
#[derive(Clone, Debug)]
#[non_exhaustive]
pub struct SmshMesh {
    /// The header as read.
    pub header: Header,
    /// Mesh id.
    pub mesh_id: EntityId,
    /// Topological dimension.
    pub dim: MeshDim,
    /// x coordinates of all points.
    pub x: Vec<f64>,
    /// y coordinates.
    pub y: Vec<f64>,
    /// z coordinates.
    pub z: Vec<f64>,
    /// Vertex ids.
    pub vertex_ids: Vec<EntityId>,
    /// Face ids.
    pub face_ids: Vec<EntityId>,
    /// Cell ids.
    pub cell_ids: Vec<EntityId>,
    /// Face loop CSR offsets.
    pub face_offsets: Vec<u64>,
    /// Face loop vertices.
    pub face_vertices: Vec<u32>,
    /// Left cell of every face.
    pub face_left: Vec<u32>,
    /// Right cell of every face (`u32::MAX` = none).
    pub face_right: Vec<u32>,
    /// Face kind codes (0 interior, 1 boundary, 2 non-conformal, 3 partition cut).
    pub face_kind: Vec<u8>,
    /// Cell-face CSR offsets.
    pub cell_face_offsets: Vec<u64>,
    /// Signed cell faces, packed `face << 1 | side`.
    pub cell_faces: Vec<u32>,
    /// Cell shape codes (see the spec).
    pub cell_shape: Vec<u8>,
    /// Template-corner CSR offsets (empty rows for polyhedra).
    pub cell_corner_offsets: Vec<u64>,
    /// Template corners.
    pub cell_corners: Vec<u32>,
    /// Parent of every cell (`u32::MAX` = root).
    pub cell_parent: Vec<u32>,
    /// Active flag of every cell.
    pub cell_active: Vec<bool>,
    /// Sorted table of model / partition entities used by the classifications.
    pub class_entities: Vec<EntityId>,
    /// Classification of every vertex.
    pub vertex_class: Vec<Classification>,
    /// Classification of every face.
    pub face_class: Vec<Classification>,
    /// Classification of every cell.
    pub cell_class: Vec<Classification>,
    /// Edge table, if the mesh had one.
    pub edges: Option<EdgeTable>,
    /// High-order cell records `(cell, record)`.
    pub high_order: Vec<(CellIx, HighOrderCell)>,
    /// Face-curve records `(face, degree, basis, nodes)`.
    pub face_curves: Vec<(FaceIx, u8, Basis, Vec<PointIx>)>,
    /// Cached geometry, if present.
    pub geometry_cache: Option<GeometryCache>,
    /// Sets in header order.
    pub sets: Vec<MeshSet>,
    /// Tag columns in header order.
    pub tags: Vec<TagColumn>,
    /// Level maps.
    pub level_maps: Vec<SmshLevelMap>,
    /// Extrusion maps.
    pub extrusions: Vec<SmshExtrusion>,
}

impl SmshMesh {
    /// Vertices of face `f`.
    pub fn face_loop(&self, f: usize) -> &[u32] {
        &self.face_vertices[self.face_offsets[f] as usize..self.face_offsets[f + 1] as usize]
    }
    /// Signed faces of cell `c`.
    pub fn cell_face_list(&self, c: usize) -> impl Iterator<Item = SignedFace> + '_ {
        self.cell_faces[self.cell_face_offsets[c] as usize..self.cell_face_offsets[c + 1] as usize].iter().map(|r| SignedFace::from_raw(*r))
    }
}

fn eid(raw: u64) -> Result<EntityId> {
    EntityId::from_raw(raw).map_or_else(|| bad(format!("unknown entity kind in id {raw:#x}")), Ok)
}

fn parse_id(s: &str) -> Result<EntityId> {
    s.parse::<EntityId>().map_or_else(|_| bad(format!("bad entity id '{s}'")), Ok)
}

fn parse_hash(s: &str) -> Result<u64> {
    u64::from_str_radix(s, 16).or_else(|_| bad(format!("bad hash '{s}'")))
}

fn ids(v: &[u64]) -> Result<Vec<EntityId>> {
    v.iter().map(|r| eid(*r)).collect()
}

fn check_csr(name: &str, off: &[u64], rows: usize, len: usize) -> Result<()> {
    if off.len() != rows + 1 || off.first() != Some(&0) || off.last().copied() != Some(len as u64) || off.windows(2).any(|w| w[0] > w[1]) {
        return bad(format!("'{name}' is not a valid CSR offset array"));
    }
    Ok(())
}

fn len_is(name: &str, got: usize, want: usize) -> Result<()> {
    if got == want { Ok(()) } else { bad(format!("'{name}' has {got} entries, expected {want}")) }
}

/// Every word is below `n`.
fn below(name: &str, v: &[u32], n: usize) -> Result<()> {
    match v.iter().position(|w| *w as usize >= n) {
        None => Ok(()),
        Some(i) => bad(format!("'{name}'[{i}] = {} is out of range (limit {n})", v[i])),
    }
}

/// Every word is below `n` or `u32::MAX` (none).
fn below_or_none(name: &str, v: &[u32], n: usize) -> Result<()> {
    match v.iter().position(|w| *w != NO_CELL && *w as usize >= n) {
        None => Ok(()),
        Some(i) => bad(format!("'{name}'[{i}] = {} is out of range (limit {n})", v[i])),
    }
}

/// Every entry `w >> 1` is below `n` (packed `index << 1 | side` words).
fn below_packed(name: &str, v: &[u32], n: usize) -> Result<()> {
    match v.iter().position(|w| (*w >> 1) as usize >= n) {
        None => Ok(()),
        Some(i) => bad(format!("'{name}'[{i}] = {} names an index out of range (limit {n})", v[i])),
    }
}

/// Strictly ascending and below `n`.
fn ascending_below(name: &str, v: &[u32], n: usize) -> Result<()> {
    below(name, v, n)?;
    if v.windows(2).any(|w| w[0] >= w[1]) {
        return bad(format!("'{name}' is not strictly ascending"));
    }
    Ok(())
}

fn basis_of(code: u8) -> Result<Basis> {
    Basis::from_code(code).map_or_else(|| bad(format!("unknown basis code {code}")), Ok)
}

fn class_of(f: &SmshFile, prefix: &str, n: usize, table: &[EntityId]) -> Result<Vec<Classification>> {
    let idx = f.u32s(&format!("{prefix}_class"))?;
    len_is(&format!("{prefix}_class"), idx.len(), n)?;
    let kinds = f.header.array(&format!("{prefix}_param_kind")).map(|_| f.u8s(&format!("{prefix}_param_kind"))).transpose()?;
    let par = f.header.array(&format!("{prefix}_param")).map(|_| f.f64s(&format!("{prefix}_param"))).transpose()?;
    if let (Some(k), Some(p)) = (&kinds, &par) {
        len_is(&format!("{prefix}_param_kind"), k.len(), n)?;
        len_is(&format!("{prefix}_param"), p.len(), 3 * n)?;
    } else if kinds.is_some() != par.is_some() {
        return bad(format!("'{prefix}_param_kind' and '{prefix}_param' must come together"));
    }
    (0..n)
        .map(|i| {
            let entity = *table.get(idx[i] as usize).ok_or_else(|| MeshioError::Malformed(format!("'{prefix}_class' index out of range")))?;
            let param = match (&kinds, &par) {
                (Some(k), Some(p)) => {
                    let q = &p[3 * i..3 * i + 3];
                    match k[i] {
                        0 => Param::None,
                        1 => Param::Curve(q[0]),
                        2 => Param::Surface([q[0], q[1]]),
                        3 => Param::Volume([q[0], q[1], q[2]]),
                        c => return bad(format!("unknown param code {c}")),
                    }
                }
                _ => Param::None,
            };
            Ok(Classification { entity, param })
        })
        .collect()
}

fn decode_mesh(f: &SmshFile) -> Result<SmshMesh> {
    let h = &f.header;
    let mi = &h.mesh;
    let (nv, np, nf, nc, ne) = (mi.n_vertices as usize, mi.n_points as usize, mi.n_faces as usize, mi.n_cells as usize, mi.n_edges as usize);
    if np < nv {
        return bad(format!("n_points {np} is below n_vertices {nv}"));
    }
    let dim = match mi.dim {
        2 => MeshDim::Two,
        3 => MeshDim::Three,
        d => return bad(format!("dimension {d}")),
    };
    let x = f.f64s("x")?.into_owned();
    let (y, z) = (f.f64s("y")?.into_owned(), f.f64s("z")?.into_owned());
    len_is("x", x.len(), np)?;
    len_is("y", y.len(), np)?;
    len_is("z", z.len(), np)?;
    let vertex_ids = ids(&f.u64s("vertex_ids")?)?;
    let face_ids = ids(&f.u64s("face_ids")?)?;
    let cell_ids = ids(&f.u64s("cell_ids")?)?;
    len_is("vertex_ids", vertex_ids.len(), nv)?;
    len_is("face_ids", face_ids.len(), nf)?;
    len_is("cell_ids", cell_ids.len(), nc)?;
    let face_offsets = f.u64s("face_offsets")?.into_owned();
    let face_vertices = f.u32s("face_vertices")?.into_owned();
    check_csr("face_offsets", &face_offsets, nf, face_vertices.len())?;
    let (face_left, face_right) = (f.u32s("face_left")?.into_owned(), f.u32s("face_right")?.into_owned());
    let face_kind = f.u8s("face_kind")?.into_owned();
    len_is("face_left", face_left.len(), nf)?;
    len_is("face_right", face_right.len(), nf)?;
    len_is("face_kind", face_kind.len(), nf)?;
    let cell_face_offsets = f.u64s("cell_face_offsets")?.into_owned();
    let cell_faces = f.u32s("cell_faces")?.into_owned();
    check_csr("cell_face_offsets", &cell_face_offsets, nc, cell_faces.len())?;
    let cell_shape = f.u8s("cell_shape")?.into_owned();
    len_is("cell_shape", cell_shape.len(), nc)?;
    let cell_corner_offsets = f.u64s("cell_corner_offsets")?.into_owned();
    let cell_corners = f.u32s("cell_corners")?.into_owned();
    check_csr("cell_corner_offsets", &cell_corner_offsets, nc, cell_corners.len())?;
    let cell_parent = f.u32s("cell_parent")?.into_owned();
    let cell_active: Vec<bool> = f.u8s("cell_active")?.iter().map(|b| *b != 0).collect();
    len_is("cell_parent", cell_parent.len(), nc)?;
    len_is("cell_active", cell_active.len(), nc)?;
    if face_vertices.iter().any(|v| *v as usize >= nv) || cell_corners.iter().any(|v| *v as usize >= nv) {
        return bad("vertex index out of range");
    }
    below_packed("cell_faces", &cell_faces, nf)?;
    below("face_left", &face_left, nc)?;
    below_or_none("face_right", &face_right, nc)?;
    below_or_none("cell_parent", &cell_parent, nc)?;
    if let Some(i) = face_kind.iter().position(|k| FaceKind::from_code(*k).is_none()) {
        return bad(format!("face_kind[{i}] = {} is not a known face kind code", face_kind[i]));
    }
    if let Some(i) = cell_shape.iter().position(|k| CellShape::from_code(*k).is_none()) {
        return bad(format!("cell_shape[{i}] = {} is not a known cell shape code", cell_shape[i]));
    }

    let class_entities = ids(&f.u64s("class_entities")?)?;
    let vertex_class = class_of(f, "vertex", nv, &class_entities)?;
    let face_class = class_of(f, "face", nf, &class_entities)?;
    let cell_class = class_of(f, "cell", nc, &class_entities)?;

    let edges = if ne > 0 {
        let eids = ids(&f.u64s("edge_ids")?)?;
        let ev = f.u32s("edge_vertices")?;
        len_is("edge_ids", eids.len(), ne)?;
        len_is("edge_vertices", ev.len(), 2 * ne)?;
        let face_offsets = f.u64s("edge_face_offsets")?.into_owned();
        let face_edge = f.u32s("edge_face_edge")?.into_owned();
        let fwd: Vec<bool> = f.u8s("edge_face_forward")?.iter().map(|b| *b != 0).collect();
        check_csr("edge_face_offsets", &face_offsets, nf, face_edge.len())?;
        len_is("edge_face_forward", fwd.len(), face_edge.len())?;
        below("edge_vertices", &ev, nv)?;
        below("edge_face_edge", &face_edge, ne)?;
        Some(EdgeTable {
            ids: eids,
            vertices: ev.as_chunks::<2>().0.iter().map(|p| [VertexIx(p[0]), VertexIx(p[1])]).collect(),
            face_off: face_offsets,
            face_edges: face_edge.iter().zip(&fwd).map(|(e, f)| FaceEdge { edge: EdgeIx(*e), forward: *f }).collect(),
            class: class_of(f, "edge", ne, &class_entities)?,
        })
    } else {
        None
    };

    let mut high_order = Vec::new();
    if h.array("ho_cell").is_some() {
        let (cells, deg, bas) = (f.u32s("ho_cell")?, f.u8s("ho_degree")?, f.u8s("ho_basis")?);
        let off = f.u64s("ho_node_offsets")?;
        let nodes = f.u32s("ho_nodes")?;
        check_csr("ho_node_offsets", &off, cells.len(), nodes.len())?;
        len_is("ho_degree", deg.len(), cells.len())?;
        len_is("ho_basis", bas.len(), cells.len())?;
        ascending_below("ho_cell", &cells, nc)?;
        below("ho_nodes", &nodes, np)?;
        for k in 0..cells.len() {
            let n = nodes[off[k] as usize..off[k + 1] as usize].iter().map(|p| PointIx(*p)).collect();
            high_order.push((CellIx(cells[k]), HighOrderCell::new(deg[k], basis_of(bas[k])?, n)));
        }
    }
    let mut face_curves = Vec::new();
    if h.array("fc_face").is_some() {
        let (faces, deg, bas) = (f.u32s("fc_face")?, f.u8s("fc_degree")?, f.u8s("fc_basis")?);
        let off = f.u64s("fc_node_offsets")?;
        let nodes = f.u32s("fc_nodes")?;
        check_csr("fc_node_offsets", &off, faces.len(), nodes.len())?;
        len_is("fc_degree", deg.len(), faces.len())?;
        len_is("fc_basis", bas.len(), faces.len())?;
        ascending_below("fc_face", &faces, nf)?;
        below("fc_nodes", &nodes, np)?;
        for k in 0..faces.len() {
            let n = nodes[off[k] as usize..off[k + 1] as usize].iter().map(|p| PointIx(*p)).collect();
            face_curves.push((FaceIx(faces[k]), deg[k], basis_of(bas[k])?, n));
        }
    }

    let geometry_cache = match &h.geometry_cache {
        None => None,
        Some(g) => {
            let mut c = GeometryCache::new(g.rule.clone(), parse_hash(&g.connectivity_hash)?, parse_hash(&g.coordinate_hash)?);
            let v3 = |name: &str, n: usize| -> Result<Vec<[f64; 3]>> {
                let a = f.f64s(name)?;
                len_is(name, a.len(), 3 * n)?;
                Ok(a.as_chunks::<3>().0.to_vec())
            };
            c.face_area = v3("gc_face_area", nf)?.into_iter().map(|p| sm_core::Vector3::new(p[0], p[1], p[2])).collect();
            c.face_centroid = v3("gc_face_centroid", nf)?.into_iter().map(|p| sm_core::Point3::new(p[0], p[1], p[2])).collect();
            c.face_apex = v3("gc_face_apex", nf)?.into_iter().map(|p| sm_core::Point3::new(p[0], p[1], p[2])).collect();
            c.cell_volume = f.f64s("gc_cell_volume")?.into_owned();
            len_is("gc_cell_volume", c.cell_volume.len(), nc)?;
            c.cell_centroid = v3("gc_cell_centroid", nc)?.into_iter().map(|p| sm_core::Point3::new(p[0], p[1], p[2])).collect();
            Some(c)
        }
    };

    let mut sets = Vec::new();
    for s in &h.sets {
        let w = f.u32s(&s.array)?.into_owned();
        let (members, limit) = match s.members {
            0 => (SetMembers::Vertices(w.into_iter().map(VertexIx).collect()), nv),
            1 => (SetMembers::Faces(w.into_iter().map(FaceIx).collect()), nf),
            2 => (SetMembers::Cells(w.into_iter().map(CellIx).collect()), nc),
            3 => (SetMembers::Sides(w.into_iter().map(SignedFace::from_raw).collect()), nf),
            4 => (SetMembers::ShellSides(w), nc),
            5 => (SetMembers::Points(w.into_iter().map(PointIx).collect()), np),
            c => return bad(format!("unknown set members code {c}")),
        };
        // Sides and shell sides are packed `index << 1 | side` words.
        let packed = matches!(members, SetMembers::Sides(_) | SetMembers::ShellSides(_));
        let words = members.raw();
        if packed { below_packed(&s.array, &words, limit)? } else { below(&s.array, &words, limit)? }
        let mut set = MeshSet::new(parse_id(&s.id)?, s.name.clone(), members);
        set.label = s.label;
        sets.push(set);
    }
    let mut tags = Vec::new();
    for t in &h.tags {
        let target = match t.target.as_str() {
            "vertex" => MeshEntityKind::Vertex,
            "edge" => MeshEntityKind::Edge,
            "face" => MeshEntityKind::Face,
            "cell" => MeshEntityKind::Cell,
            o => return bad(format!("unknown tag target '{o}'")),
        };
        let a = h.array(&t.array).ok_or_else(|| MeshioError::Malformed(format!("missing array '{}'", t.array)))?;
        let values = match a.dtype {
            Dtype::I64 => TagValues::I64(f.i64s(&t.array)?.into_owned()),
            Dtype::F64 => TagValues::F64(f.f64s(&t.array)?.into_owned()),
            d => return bad(format!("tag array has dtype {d:?}")),
        };
        let want = match target {
            MeshEntityKind::Vertex => nv,
            MeshEntityKind::Edge => ne,
            MeshEntityKind::Face => nf,
            _ => nc,
        };
        len_is(&t.array, values.len(), want)?;
        tags.push(TagColumn::new(t.name.clone(), target, values));
    }

    let mesh_id = parse_id(&mi.id)?;
    let mut level_maps = Vec::new();
    for l in &h.level_maps {
        let m = SmshLevelMap {
            fine: parse_id(&l.fine)?,
            coarse: parse_id(&l.coarse)?,
            fine_hash: parse_hash(&l.fine_hash)?,
            coarse_hash: parse_hash(&l.coarse_hash)?,
            coarse_of: f.u32s(&l.coarse_of)?.into_owned(),
            face_map: l.face_map.as_ref().map(|n| f.u32s(n).map(Cow::into_owned)).transpose()?,
        };
        if mesh_id != m.fine && mesh_id != m.coarse {
            return bad(format!("level map {} -> {} does not involve mesh {mesh_id}", m.fine, m.coarse));
        }
        if mesh_id == m.fine {
            len_is(&l.coarse_of, m.coarse_of.len(), nc)?;
            if let (Some(fm), Some(n)) = (&m.face_map, &l.face_map) {
                len_is(n, fm.len(), nf)?;
            }
        }
        if mesh_id == m.coarse {
            below(&l.coarse_of, &m.coarse_of, nc)?;
            if let (Some(fm), Some(n)) = (&m.face_map, &l.face_map) {
                below_or_none(n, fm, nf)?;
            }
        }
        level_maps.push(m);
    }
    let mut extrusions = Vec::new();
    for x in &h.extrusions {
        let e = SmshExtrusion {
            base: parse_id(&x.base)?,
            extruded: parse_id(&x.extruded)?,
            base_hash: parse_hash(&x.base_hash)?,
            extruded_hash: parse_hash(&x.extruded_hash)?,
            z: f.f64s(&x.z)?.into_owned(),
            cell_base: f.u32s(&x.cell_base)?.into_owned(),
            cell_layer: f.u32s(&x.cell_layer)?.into_owned(),
        };
        let n_layers = usize::try_from(x.n_layers).or_else(|_| bad("extrusion n_layers is too large"))?;
        len_is("extrusion z", e.z.len(), n_layers.saturating_add(1))?;
        len_is("extrusion cell_layer", e.cell_layer.len(), e.cell_base.len())?;
        below("extrusion cell_layer", &e.cell_layer, n_layers)?;
        if mesh_id == e.extruded {
            len_is("extrusion cell_base", e.cell_base.len(), nc)?;
        } else if mesh_id == e.base {
            let want = nc.checked_mul(n_layers).map_or_else(|| bad("extrusion size overflows"), Ok)?;
            len_is("extrusion cell_base", e.cell_base.len(), want)?;
            below("extrusion cell_base", &e.cell_base, nc)?;
        } else {
            return bad(format!("extrusion {} -> {} does not involve mesh {mesh_id}", e.base, e.extruded));
        }
        extrusions.push(e);
    }

    Ok(SmshMesh {
        header: h.clone(),
        mesh_id,
        dim,
        x,
        y,
        z,
        vertex_ids,
        face_ids,
        cell_ids,
        face_offsets,
        face_vertices,
        face_left,
        face_right,
        face_kind,
        cell_face_offsets,
        cell_faces,
        cell_shape,
        cell_corner_offsets,
        cell_corners,
        cell_parent,
        cell_active,
        class_entities,
        vertex_class,
        face_class,
        cell_class,
        edges,
        high_order,
        face_curves,
        geometry_cache,
        sets,
        tags,
        level_maps,
        extrusions,
    })
}
