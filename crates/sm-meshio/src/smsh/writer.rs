//! The `.smsh` writer.

use super::format::*;
use crate::crc::crc32;
use crate::error::{MeshioError, Result};
use crate::le::{self, f64_bytes, index_bytes, pack, pack_slice};
use crate::report::LossReport;
use rayon::prelude::*;
use sm_mesh_api::{
    CellIx, CellShape, Classification, ExtrusionMap, FaceIx, LevelMap, MeshDim, MeshEntityKind, MeshRead, NO_CELL, Param, SetMembers, TagValues,
    connectivity_hash, geometry_hash, topology_hash,
};
use std::borrow::Cow;
use std::io::Write;
use std::path::Path;

/// One array queued for writing.
struct Blob<'a> {
    name: String,
    dtype: Dtype,
    shape: Vec<u64>,
    data: Cow<'a, [u8]>,
}

/// Writes one mesh (plus attached metadata) as a `.smsh` container.
///
/// Reads the mesh through [`MeshRead`] only. The output is a pure function of the mesh and the
/// attached metadata: byte-identical across runs and thread counts.
pub struct SmshWriter<'a> {
    mesh: &'a dyn MeshRead,
    provenance: Option<Provenance>,
    decks: Vec<DeckRecord>,
    level_maps: Vec<&'a LevelMap>,
    extrusions: Vec<&'a ExtrusionMap>,
    element_types: Vec<(String, String)>,
    hashes: bool,
}

impl<'a> SmshWriter<'a> {
    /// A writer for `mesh` with no metadata attached.
    pub fn new(mesh: &'a dyn MeshRead) -> Self {
        Self { mesh, provenance: None, decks: Vec::new(), level_maps: Vec::new(), extrusions: Vec::new(), element_types: Vec::new(), hashes: true }
    }

    /// Record the op-timeline hash that produced the mesh.
    pub fn provenance(mut self, timeline_hash: impl Into<String>) -> Self {
        self.provenance = Some(Provenance { timeline_hash: timeline_hash.into() });
        self
    }

    /// Attach a deck transformation. Its `face_set` / `mate_set`, when given, must name sets of
    /// the mesh (checked when writing).
    pub fn deck(mut self, deck: DeckRecord) -> Self {
        self.decks.push(deck);
        self
    }

    /// Attach a fine-to-coarse map; the mesh must be its fine or its coarse mesh.
    pub fn level_map(mut self, map: &'a LevelMap) -> Self {
        self.level_maps.push(map);
        self
    }

    /// Attach an extrusion map; the mesh must be its base or its extruded mesh.
    pub fn extrusion(mut self, map: &'a ExtrusionMap) -> Self {
        self.extrusions.push(map);
        self
    }

    /// Declare the element type of a cell set (`HEX8`, `TET10`, `POLYHEDRON`, ...; see
    /// [`element_type_shape`]). Checked when writing: the set must exist and be a cell set, every
    /// member's shape must equal the type's shape, high-order names need a high-order record with
    /// the right node count on every member, a set takes one record, and a cell in two typed sets
    /// must get the same type. Violations are [`MeshioError::Inconsistent`] or
    /// [`MeshioError::Unsupported`] (unknown spelling, including `NFACED` / `NSIDED`).
    pub fn element_type(mut self, set: impl Into<String>, element_type: impl Into<String>) -> Self {
        self.element_types.push((set.into(), element_type.into()));
        self
    }

    /// What this writer cannot carry: nothing. `.smsh` is lossless for everything `MeshRead`
    /// exposes, so the report is always empty.
    pub fn loss_report(&self) -> LossReport {
        LossReport::new()
    }

    /// Skip the three content hashes (`connectivity`, `topology`, `geometry`) of the mesh API,
    /// which cost a few sequential passes over the mesh. Readers then see no `hashes` entry.
    pub fn without_hashes(mut self) -> Self {
        self.hashes = false;
        self
    }

    /// The complete file as bytes.
    pub fn to_bytes(&self) -> Result<Vec<u8>> {
        let mut v = Vec::new();
        self.write_to(&mut v)?;
        Ok(v)
    }

    /// Write the file to `path` (created or truncated).
    pub fn write_file(&self, path: impl AsRef<Path>) -> Result<()> {
        let f = std::fs::File::create(path)?;
        let mut w = std::io::BufWriter::new(f);
        self.write_to(&mut w)?;
        w.flush()?;
        Ok(())
    }

    /// Write the file to `w`.
    pub fn write_to<W: Write>(&self, w: &mut W) -> Result<()> {
        let (mut header, blobs) = self.build()?;
        let crcs: Vec<u32> = blobs.par_iter().map(|b| crc32(&b.data)).collect();
        let mut off = 0u64;
        for (b, crc) in blobs.iter().zip(crcs) {
            off = align_up(off);
            header.arrays.push(ArrayInfo {
                name: b.name.clone(),
                dtype: b.dtype,
                shape: b.shape.clone(),
                offset: off,
                nbytes: b.data.len() as u64,
                crc32: crc,
            });
            off += b.data.len() as u64;
        }
        let mut json = serde_json::to_vec(&header)?;
        let total = PREAMBLE + json.len() as u64;
        json.resize(json.len() + (align_up(total) - total) as usize, b' ');
        w.write_all(&MAGIC)?;
        w.write_all(&(json.len() as u64).to_le_bytes())?;
        w.write_all(&json)?;
        let zeros = [0u8; ALIGN as usize];
        let mut pos = 0u64;
        for (b, info) in blobs.iter().zip(&header.arrays) {
            w.write_all(&zeros[..(info.offset - pos) as usize])?;
            w.write_all(&b.data)?;
            pos = info.offset + info.nbytes;
        }
        Ok(())
    }

    fn build(&self) -> Result<(Header, Vec<Blob<'a>>)> {
        let m = self.mesh;
        let (nv, nf, nc, ne) =
            (m.count(MeshEntityKind::Vertex), m.count(MeshEntityKind::Face), m.count(MeshEntityKind::Cell), m.count(MeshEntityKind::Edge));
        let co = m.coords();
        let np = co.len();
        let bad = |s: String| MeshioError::Inconsistent(s);
        if co.y.len() != np || co.z.len() != np || np < nv {
            return Err(bad("coordinate columns disagree with the vertex count".into()));
        }
        check_columns(m)?;
        let cols = m.columns();
        let mut b: Vec<Blob<'a>> = Vec::new();
        let mut push = |name: &str, dtype: Dtype, shape: Vec<u64>, data: Cow<'a, [u8]>| {
            b.push(Blob { name: name.to_string(), dtype, shape, data });
        };
        let n1 = |n: usize| vec![n as u64];

        // Coordinates and ids.
        push("x", Dtype::F64, n1(np), f64_bytes(co.x));
        push("y", Dtype::F64, n1(np), f64_bytes(co.y));
        push("z", Dtype::F64, n1(np), f64_bytes(co.z));
        let ids = |k: MeshEntityKind, n: usize| pack(n, |i| m.id(k, i as u32).to_raw());
        push("vertex_ids", Dtype::U64, n1(nv), ids(MeshEntityKind::Vertex, nv).into());
        push("face_ids", Dtype::U64, n1(nf), ids(MeshEntityKind::Face, nf).into());
        push("cell_ids", Dtype::U64, n1(nc), ids(MeshEntityKind::Cell, nc).into());

        // Face-first topology.
        push("face_offsets", Dtype::U64, n1(nf + 1), pack_slice(cols.face_vertex_offsets).into());
        let fv = cols.face_vertices;
        push("face_vertices", Dtype::U32, n1(fv.len()), index_bytes(fv));
        let fl = cols.face_left;
        push("face_left", Dtype::U32, n1(nf), index_bytes(fl));
        push("face_right", Dtype::U32, n1(nf), pack_slice(cols.face_right).into());
        let kinds = pack(nf, |i| m.face_kind(FaceIx::new(i)).code());
        push("face_kind", Dtype::U8, n1(nf), kinds.clone().into());
        push("cell_face_offsets", Dtype::U64, n1(nc + 1), pack_slice(cols.cell_face_offsets).into());
        let cf = cols.cell_faces;
        push("cell_faces", Dtype::U32, n1(cf.len()), index_bytes(cf));

        // Cells: shape tags, template corners, lineage.
        let shapes = pack(nc, |c| m.cell_shape(CellIx::new(c)).code());
        let stats = MeshStats::from_arrays(&shapes, &kinds, cols.cell_face_offsets, cols.face_vertex_offsets);
        push("cell_shape", Dtype::U8, n1(nc), shapes.into());
        let (off, data) = le::csr(nc, |c| m.cell_corners(CellIx::new(c)).unwrap_or(&[]), |v| v.0);
        push("cell_corner_offsets", Dtype::U64, n1(nc + 1), pack_slice(&off).into());
        push("cell_corners", Dtype::U32, n1(data.len() / 4), data.into());
        push("cell_parent", Dtype::U32, n1(nc), pack(nc, |c| m.cell_parent(CellIx::new(c)).map_or(NO_CELL, |p| p.0)).into());
        push("cell_active", Dtype::U8, n1(nc), pack(nc, |c| u8::from(m.cell_active(CellIx::new(c)))).into());

        // Classification.
        let cls = |k: MeshEntityKind, n: usize| -> Vec<Classification> { (0..n).into_par_iter().map(|i| m.classification(k, i as u32)).collect() };
        let vcls = cls(MeshEntityKind::Vertex, nv);
        let fcls = cls(MeshEntityKind::Face, nf);
        let ccls = cls(MeshEntityKind::Cell, nc);
        let edges = m.edges();
        let ecls: &[Classification] = edges.map_or(&[], |t| t.class.as_slice());
        let mut table: Vec<u64> = vcls.iter().chain(&fcls).chain(&ccls).chain(ecls).map(|c| c.entity.to_raw()).collect();
        table.par_sort_unstable();
        table.dedup();
        push("class_entities", Dtype::U64, n1(table.len()), pack_slice(&table).into());
        for (prefix, c) in [("vertex", &vcls[..]), ("face", &fcls[..]), ("cell", &ccls[..])] {
            class_arrays(&mut push, prefix, c, &table)?;
        }

        // Edge table.
        if let Some(t) = edges {
            if t.ids.len() != ne || t.vertices.len() != ne || t.class.len() != ne || t.face_off.len() != nf + 1 {
                return Err(bad("edge table disagrees with the edge count".into()));
            }
            push("edge_ids", Dtype::U64, n1(ne), pack(ne, |i| t.ids[i].to_raw()).into());
            push("edge_vertices", Dtype::U32, vec![ne as u64, 2], index_bytes(&t.vertices));
            class_arrays(&mut push, "edge", ecls, &table)?;
            push("edge_face_offsets", Dtype::U64, n1(nf + 1), pack_slice(&t.face_off).into());
            let fe = &t.face_edges;
            push("edge_face_edge", Dtype::U32, n1(fe.len()), pack(fe.len(), |i| fe[i].edge.0).into());
            push("edge_face_forward", Dtype::U8, n1(fe.len()), pack(fe.len(), |i| u8::from(fe[i].forward)).into());
        }

        // High-order records (sparse).
        let ho: Vec<u32> = (0..nc).into_par_iter().filter(|&c| m.high_order_cell(CellIx::new(c)).is_some()).map(|c| c as u32).collect();
        if !ho.is_empty() {
            let rec = |k: usize| m.high_order_cell(CellIx(ho[k]));
            push("ho_cell", Dtype::U32, n1(ho.len()), pack_slice(&ho).into());
            push("ho_degree", Dtype::U8, n1(ho.len()), pack(ho.len(), |k| rec(k).map_or(0, |h| h.degree)).into());
            push("ho_basis", Dtype::U8, n1(ho.len()), pack(ho.len(), |k| rec(k).map_or(0, |h| h.basis.code())).into());
            let (off, data) = le::csr(ho.len(), |k| rec(k).map_or(&[][..], |h| h.nodes.as_slice()), |p| p.0);
            push("ho_node_offsets", Dtype::U64, n1(ho.len() + 1), pack_slice(&off).into());
            push("ho_nodes", Dtype::U32, n1(data.len() / 4), data.into());
        }
        let fc: Vec<u32> = (0..nf).into_par_iter().filter(|&f| m.face_curve(FaceIx::new(f)).is_some()).map(|f| f as u32).collect();
        if !fc.is_empty() {
            let rec = |k: usize| m.face_curve(FaceIx(fc[k]));
            push("fc_face", Dtype::U32, n1(fc.len()), pack_slice(&fc).into());
            push("fc_degree", Dtype::U8, n1(fc.len()), pack(fc.len(), |k| rec(k).map_or(0, |h| h.degree)).into());
            push("fc_basis", Dtype::U8, n1(fc.len()), pack(fc.len(), |k| rec(k).map_or(0, |h| h.basis.code())).into());
            let (off, data) = le::csr(fc.len(), |k| rec(k).map_or(&[][..], |h| h.nodes), |p| p.0);
            push("fc_node_offsets", Dtype::U64, n1(fc.len() + 1), pack_slice(&off).into());
            push("fc_nodes", Dtype::U32, n1(data.len() / 4), data.into());
        }

        // Cached geometry.
        let mut gc_info = None;
        if let Some(g) = m.geometry_cache() {
            if g.face_area.len() != nf || g.face_centroid.len() != nf || g.face_apex.len() != nf || g.cell_volume.len() != nc || g.cell_centroid.len() != nc {
                return Err(bad("geometry cache disagrees with the entity counts".into()));
            }
            let v3 = |n: usize, f: &(dyn Fn(usize) -> [f64; 3] + Sync)| pack(3 * n, |i| f(i / 3)[i % 3]);
            push("gc_face_area", Dtype::F64, vec![nf as u64, 3], v3(nf, &|i| [g.face_area[i].x, g.face_area[i].y, g.face_area[i].z]).into());
            push("gc_face_centroid", Dtype::F64, vec![nf as u64, 3], v3(nf, &|i| [g.face_centroid[i].x, g.face_centroid[i].y, g.face_centroid[i].z]).into());
            push("gc_face_apex", Dtype::F64, vec![nf as u64, 3], v3(nf, &|i| [g.face_apex[i].x, g.face_apex[i].y, g.face_apex[i].z]).into());
            push("gc_cell_volume", Dtype::F64, n1(nc), f64_bytes(&g.cell_volume));
            push("gc_cell_centroid", Dtype::F64, vec![nc as u64, 3], v3(nc, &|i| [g.cell_centroid[i].x, g.cell_centroid[i].y, g.cell_centroid[i].z]).into());
            gc_info = Some(GeometryCacheInfo { rule: g.rule.clone(), connectivity_hash: hex(g.connectivity_hash), coordinate_hash: hex(g.coordinate_hash) });
        }

        // Sets and tags.
        let mut sets = Vec::new();
        for (i, s) in m.sets().iter().enumerate() {
            let role = match &s.members {
                SetMembers::Vertices(_) => "node_set",
                SetMembers::Faces(_) => "face_set",
                SetMembers::Cells(_) => "cell_set",
                SetMembers::Sides(_) => "side_set",
                SetMembers::ShellSides(_) => "shell_side_set",
                SetMembers::Points(_) => "point_set",
                _ => return Err(MeshioError::Unsupported(format!("set '{}' has a members variant unknown to this format version", s.name))),
            };
            let array = format!("set.{i}");
            let raw = s.members.raw();
            push(&array, Dtype::U32, n1(raw.len()), pack_slice(&raw).into());
            sets.push(SetInfo { id: s.id.to_string(), name: s.name.clone(), label: s.label, members: s.members.code(), role: role.into(), array });
        }
        let mut tags = Vec::new();
        for (i, t) in m.tags().iter().enumerate() {
            let target = match t.target {
                MeshEntityKind::Vertex => "vertex",
                MeshEntityKind::Edge => "edge",
                MeshEntityKind::Face => "face",
                MeshEntityKind::Cell => "cell",
                _ => return Err(MeshioError::Unsupported(format!("tag '{}' targets an entity kind unknown to this format version", t.name))),
            };
            let array = format!("tag.{i}");
            match &t.values {
                TagValues::I64(v) => push(&array, Dtype::I64, n1(v.len()), pack_slice(v).into()),
                TagValues::F64(v) => push(&array, Dtype::F64, n1(v.len()), f64_bytes(v)),
                _ => return Err(MeshioError::Unsupported(format!("tag '{}' has a value type unknown to this format version", t.name))),
            }
            tags.push(TagInfo { name: t.name.clone(), target: target.into(), array });
        }

        // Decks.
        for d in &self.decks {
            if !d.is_finite() {
                return Err(bad(format!("deck '{}' has non-finite numbers", d.name)));
            }
            for s in [&d.face_set, &d.mate_set].into_iter().flatten() {
                if m.set_by_name(s).is_none() {
                    return Err(bad(format!("deck '{}' names unknown set '{s}'", d.name)));
                }
            }
        }

        let element_types = self.check_element_types()?;

        // Hierarchy. A map is checked against the mesh it is attached to (ids, sizes and the
        // connectivity hash it recorded), so a stale map is an error, never a panic.
        let id = m.mesh_id();
        let hash = (!self.level_maps.is_empty() || !self.extrusions.is_empty()).then(|| connectivity_hash(m));
        let stale = |what: String| bad(format!("{what} is stale: it was built for a different mesh state than mesh {id}"));
        let mut level_maps = Vec::new();
        for (i, lm) in self.level_maps.iter().enumerate() {
            if id != lm.fine_id() && id != lm.coarse_id() {
                return Err(bad(format!("level map {i} is not attached to mesh {id}")));
            }
            let (own_hash, own_len) = if id == lm.fine_id() { (lm.fine_hash(), lm.fine_len()) } else { (lm.coarse_hash(), lm.coarse_len()) };
            if hash != Some(own_hash) || own_len != nc {
                return Err(stale(format!("level map {i}")));
            }
            if id == lm.fine_id() && lm.face_map().is_some_and(|f| f.len() != nf) {
                return Err(stale(format!("level map {i}")));
            }
            let name = format!("levelmap.{i}.coarse_of");
            push(&name, Dtype::U32, n1(lm.fine_len()), pack(lm.fine_len(), |c| lm.coarse_of(CellIx::new(c)).0).into());
            let face_map = lm.face_map().map(|f| {
                let n = format!("levelmap.{i}.face_map");
                push(&n, Dtype::U32, n1(f.len()), pack_slice(f).into());
                n
            });
            level_maps.push(LevelMapInfo {
                fine: lm.fine_id().to_string(),
                coarse: lm.coarse_id().to_string(),
                fine_hash: hex(lm.fine_hash()),
                coarse_hash: hex(lm.coarse_hash()),
                coarse_of: name,
                face_map,
            });
        }
        let mut extrusions = Vec::new();
        for (i, x) in self.extrusions.iter().enumerate() {
            let own_hash = if id == x.extruded_id() {
                x.extruded_hash()
            } else if id == x.base_id() {
                x.base_hash()
            } else {
                return Err(bad(format!("extrusion map {i} is not attached to mesh {id}")));
            };
            // The map holds one entry per extruded cell; the hash check keeps it paired with
            // this mesh state.
            let n_ext = x.n_extruded();
            if hash != Some(own_hash) {
                return Err(stale(format!("extrusion map {i}")));
            }
            let (zn, bn, ln) = (format!("extrusion.{i}.z"), format!("extrusion.{i}.cell_base"), format!("extrusion.{i}.cell_layer"));
            push(&zn, Dtype::F64, n1(x.z().len()), f64_bytes(x.z()));
            push(&bn, Dtype::U32, n1(n_ext), pack(n_ext, |c| x.cell_of(CellIx::new(c)).0.0).into());
            push(&ln, Dtype::U32, n1(n_ext), pack(n_ext, |c| x.cell_of(CellIx::new(c)).1).into());
            extrusions.push(ExtrusionInfo {
                base: x.base_id().to_string(),
                extruded: x.extruded_id().to_string(),
                base_hash: hex(x.base_hash()),
                extruded_hash: hex(x.extruded_hash()),
                n_layers: x.n_layers() as u64,
                z: zn,
                cell_base: bn,
                cell_layer: ln,
            });
        }

        let header = Header {
            format: "smsh".into(),
            version: Version { major: VERSION_MAJOR, minor: VERSION_MINOR },
            endian: "little".into(),
            producer: concat!("sm-meshio ", env!("CARGO_PKG_VERSION")).into(),
            mesh: MeshInfo {
                id: id.to_string(),
                dim: match m.dim() {
                    MeshDim::Two => 2,
                    _ => 3,
                },
                geometry_rule: m.geometry_rule().to_string(),
                n_vertices: nv as u64,
                n_points: np as u64,
                n_faces: nf as u64,
                n_cells: nc as u64,
                n_edges: ne as u64,
                stats: Some(stats),
                hashes: self.hashes.then(|| {
                    // Three independent pure passes: run them side by side.
                    let ((c, g), t) = rayon::join(|| rayon::join(|| connectivity_hash(m), || geometry_hash(m)), || topology_hash(m));
                    HashInfo { connectivity: hex(c), topology: hex(t), geometry: hex(g) }
                }),
            },
            arrays: Vec::new(),
            sets,
            tags,
            geometry_cache: gc_info,
            level_maps,
            extrusions,
            decks: self.decks.clone(),
            provenance: self.provenance.clone(),
            element_types,
        };
        Ok((header, b))
    }

    /// Validate the declared element types against the mesh and return the header rows,
    /// ascending by set id.
    fn check_element_types(&self) -> Result<Vec<ElementTypeInfo>> {
        let m = self.mesh;
        let bad = |s: String| MeshioError::Inconsistent(s);
        // (set index, type name, shape) per record, one record per set.
        let mut recs: Vec<(usize, &str, CellShape)> = Vec::new();
        for (set, ty) in &self.element_types {
            let shape = element_type_shape(ty)
                .ok_or_else(|| MeshioError::Unsupported(format!("element type '{ty}' (set '{set}') is not a .smsh element type; use POLYHEDRON / POLYGON, not NFACED / NSIDED")))?;
            let Some(si) = m.sets().iter().position(|s| s.name == *set) else {
                return Err(bad(format!("element type '{ty}' names unknown set '{set}'")));
            };
            if recs.iter().any(|r| r.0 == si) {
                return Err(bad(format!("set '{set}' has more than one element type record")));
            }
            recs.push((si, ty.as_str(), shape));
        }
        recs.sort_by_key(|r| r.0);
        let nc = m.cell_count();
        for (si, ty, shape) in &recs {
            let s = &m.sets()[*si];
            let SetMembers::Cells(cells) = &s.members else {
                return Err(bad(format!("element type '{ty}': set '{}' is not a cell set", s.name)));
            };
            if let Some(c) = cells.iter().find(|c| c.get() >= nc || m.cell_shape(**c) != *shape) {
                return Err(bad(format!("element type '{ty}': cell {} of set '{}' is not a {shape:?}", c.0, s.name)));
            }
            if let Some(n) = element_type_nodes(ty)
                && let Some(c) = cells.iter().find(|c| m.high_order_cell(**c).is_none_or(|h| h.nodes.len() != n))
            {
                return Err(bad(format!("element type '{ty}': cell {} of set '{}' has no high-order record with {n} nodes", c.0, s.name)));
            }
        }
        // A cell in two typed sets must get one type: intersect the sorted member lists.
        for (i, a) in recs.iter().enumerate() {
            for b in &recs[i + 1..] {
                if a.1 == b.1 {
                    continue;
                }
                let (SetMembers::Cells(x), SetMembers::Cells(y)) = (&m.sets()[a.0].members, &m.sets()[b.0].members) else { continue };
                let (mut p, mut q) = (0, 0);
                while p < x.len() && q < y.len() {
                    match x[p].cmp(&y[q]) {
                        core::cmp::Ordering::Less => p += 1,
                        core::cmp::Ordering::Greater => q += 1,
                        core::cmp::Ordering::Equal => {
                            return Err(bad(format!(
                                "cell {} is in set '{}' typed {} and set '{}' typed {}",
                                x[p].0,
                                m.sets()[a.0].name,
                                a.1,
                                m.sets()[b.0].name,
                                b.1
                            )));
                        }
                    }
                }
            }
        }
        Ok(recs.iter().map(|(si, ty, shape)| ElementTypeInfo { set: m.sets()[*si].name.clone(), element_type: (*ty).to_string(), shape: shape.code() }).collect())
    }
}

/// Check that the whole-slice columns of `m` agree with its entity counts, so the writers can
/// index them without risking a panic on a malformed implementation.
pub(crate) fn check_columns(m: &dyn MeshRead) -> Result<()> {
    let (nf, nc) = (m.count(MeshEntityKind::Face), m.count(MeshEntityKind::Cell));
    let cols = m.columns();
    let ok = cols.face_vertex_offsets.len() == nf + 1
        && cols.face_left.len() == nf
        && cols.face_right.len() == nf
        && cols.cell_face_offsets.len() == nc + 1
        && cols.face_vertex_offsets.last().copied().unwrap_or(0) as usize == cols.face_vertices.len()
        && cols.cell_face_offsets.last().copied().unwrap_or(0) as usize == cols.cell_faces.len();
    if ok { Ok(()) } else { Err(MeshioError::Inconsistent("topology columns disagree with the entity counts".into())) }
}

/// Param code and values of a classification, `None` for a variant unknown to this version.
fn param_code(p: Param) -> Option<(u8, [f64; 3])> {
    Some(match p {
        Param::None => (0, [0.0; 3]),
        Param::Curve(u) => (1, [u, 0.0, 0.0]),
        Param::Surface([u, v]) => (2, [u, v, 0.0]),
        Param::Volume([u, v, w]) => (3, [u, v, w]),
        _ => return None,
    })
}

/// Queue `<prefix>_class` (index into the entity table) and, if any parameter is set,
/// `<prefix>_param_kind` / `<prefix>_param`.
fn class_arrays<'a>(
    push: &mut impl FnMut(&str, Dtype, Vec<u64>, Cow<'a, [u8]>),
    prefix: &str,
    cls: &[Classification],
    table: &[u64],
) -> Result<()> {
    let idx: Vec<u32> = cls.par_iter().map(|c| table.binary_search(&c.entity.to_raw()).map_or(u32::MAX, |i| i as u32)).collect();
    let n = cls.len() as u64;
    push(&format!("{prefix}_class"), Dtype::U32, vec![n], pack_slice(&idx).into());
    let codes: Vec<Option<(u8, [f64; 3])>> = cls.par_iter().map(|c| param_code(c.param)).collect();
    if codes.iter().any(Option::is_none) {
        return Err(MeshioError::Unsupported("classification parameter variant unknown to this format version".into()));
    }
    if codes.iter().flatten().any(|c| c.0 != 0) {
        push(&format!("{prefix}_param_kind"), Dtype::U8, vec![n], pack(cls.len(), |i| codes[i].map_or(0, |c| c.0)).into());
        push(&format!("{prefix}_param"), Dtype::F64, vec![n, 3], pack(3 * cls.len(), |i| codes[i / 3].map_or(0.0, |c| c.1[i % 3])).into());
    }
    Ok(())
}
