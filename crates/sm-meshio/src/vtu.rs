//! VTU (VTK XML unstructured grid) export for ParaView.
//!
//! * Template cells map to the VTK cell types (tet 10, hex 12, wedge 13, pyramid 14; our
//!   template node order is VTK's); polyhedra become `VTK_POLYHEDRON` (42) with the `faces` /
//!   `faceoffsets` streams (outward-oriented loops); dimension-2 meshes give triangles, quads
//!   and polygons.
//! * Cell data: `class_entity` (serial of the model entity the cell is classified on, the
//!   "color"), `region` (label of the first labelled cell set containing the cell, `-1` for
//!   none), one `set:<name>` indicator per cell set, cell tags, and `active` when a refined
//!   parent is retained. Point data: `class_dim`, `class_entity`, node-set indicators, vertex
//!   tags.
//! * Binary data is appended (`raw`) or appended base64; both are uncompressed with `UInt64`
//!   block headers. Index arrays use 32 bits when everything fits, 64 bits otherwise.
//!
//! High-order geometry is exported as its linear corners (extra nodes are written as unused
//! points); see `DESIGN.md`.

use crate::error::{MeshioError, Result};
use crate::le::{pack, pack_slice};
use rayon::prelude::*;
use sm_mesh_api::{CellIx, CellShape, MeshEntityKind, MeshRead, SetMembers, TagValues};
use std::io::Write;

/// How the binary arrays are stored.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum VtuEncoding {
    /// `<AppendedData encoding="raw">`: smallest and fastest; the file is not pure text.
    #[default]
    AppendedRaw,
    /// `<AppendedData encoding="base64">`: pure ASCII, 4/3 larger.
    AppendedBase64,
}

/// Export options.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct VtuOptions {
    /// Storage of binary arrays.
    pub encoding: VtuEncoding,
    /// Write one 0/1 `set:<name>` array per cell set and node set.
    pub set_indicators: bool,
    /// Write the mesh's integer and real tag columns on cells and vertices as `tag:<name>`.
    pub tags: bool,
}

impl Default for VtuOptions {
    fn default() -> Self {
        Self { encoding: VtuEncoding::default(), set_indicators: true, tags: true }
    }
}

impl VtuOptions {
    /// Options with the given encoding and the defaults for everything else.
    pub fn with_encoding(encoding: VtuEncoding) -> Self {
        Self { encoding, ..Self::default() }
    }
}

/// One `DataArray`.
struct Arr {
    name: String,
    ty: &'static str,
    ncomp: usize,
    bytes: Vec<u8>,
}

impl Arr {
    fn new(name: impl Into<String>, ty: &'static str, ncomp: usize, bytes: Vec<u8>) -> Self {
        Self { name: name.into(), ty, ncomp, bytes }
    }
}

const VTK_TRIANGLE: u8 = 5;
const VTK_POLYGON: u8 = 7;
const VTK_QUAD: u8 = 9;
const VTK_TETRA: u8 = 10;
const VTK_HEXAHEDRON: u8 = 12;
const VTK_WEDGE: u8 = 13;
const VTK_PYRAMID: u8 = 14;
const VTK_POLYHEDRON: u8 = 42;

const BLOCK: usize = 4096;

/// Connectivity pieces of a block of consecutive cells.
#[derive(Default)]
struct CellBlock {
    types: Vec<u8>,
    conn: Vec<u32>,
    conn_len: Vec<u32>,
    faces: Vec<u32>,
    /// Per cell: length of its face stream, 0 for a non-polyhedron.
    faces_len: Vec<u32>,
}

fn cell_block(m: &dyn MeshRead, lo: usize, hi: usize) -> Result<CellBlock> {
    let cols = m.columns();
    let mut b = CellBlock::default();
    let mut scratch: Vec<u32> = Vec::new();
    for c in lo..hi {
        let ci = CellIx::new(c);
        let shape = m.cell_shape(ci);
        let ty = match shape {
            CellShape::Tet => VTK_TETRA,
            CellShape::Hex => VTK_HEXAHEDRON,
            CellShape::Wedge => VTK_WEDGE,
            CellShape::Pyramid => VTK_PYRAMID,
            CellShape::Polyhedron => VTK_POLYHEDRON,
            CellShape::Tri => VTK_TRIANGLE,
            CellShape::Quad => VTK_QUAD,
            CellShape::Polygon => VTK_POLYGON,
            _ => return Err(MeshioError::Unsupported(format!("cell shape {shape:?} has no VTK mapping"))),
        };
        b.types.push(ty);
        let start = b.conn.len();
        if ty == VTK_POLYHEDRON {
            // Distinct vertices, ascending; then the face stream with outward loops.
            scratch.clear();
            let sfs = m.cell_faces(ci);
            for sf in sfs {
                scratch.extend(m.face_vertices(sf.face()).iter().map(|v| v.0));
            }
            scratch.sort_unstable();
            scratch.dedup();
            b.conn.extend_from_slice(&scratch);
            let fstart = b.faces.len();
            b.faces.push(sfs.len() as u32);
            for sf in sfs {
                let f = sf.face().get();
                let lp = &cols.face_vertices[cols.face_vertex_offsets[f] as usize..cols.face_vertex_offsets[f + 1] as usize];
                b.faces.push(lp.len() as u32);
                if sf.side() == sm_mesh_api::Side::Left {
                    b.faces.extend(lp.iter().map(|v| v.0));
                } else {
                    b.faces.extend(lp.iter().rev().map(|v| v.0));
                }
            }
            b.faces_len.push((b.faces.len() - fstart) as u32);
        } else {
            let corners = m
                .cell_corners(ci)
                .ok_or_else(|| MeshioError::Unsupported(format!("cell {c} of shape {shape:?} has no corner list")))?;
            // Tet, hex, wedge and pyramid share VTK's node order (checked against VTK's signed
            // cell volumes and outward face tables in the tests).
            b.conn.extend(corners.iter().map(|v| v.0));
            b.faces_len.push(0);
        }
        b.conn_len.push((b.conn.len() - start) as u32);
    }
    Ok(b)
}

/// One little-endian index in the output width (`u64::MAX` becomes -1).
#[inline]
fn put_idx(out: &mut Vec<u8>, v: u64, wide: bool) {
    if wide { out.extend_from_slice(&(v as i64).to_le_bytes()) } else { out.extend_from_slice(&(v as i32).to_le_bytes()) }
}

fn idx_arr(name: &str, bytes: Vec<u8>, wide: bool) -> Arr {
    Arr::new(name, if wide { "Int64" } else { "Int32" }, 1, bytes)
}

/// Tag values must have exactly one entry per entity.
fn check_tag_len(name: &str, got: usize, want: usize) -> Result<()> {
    if got == want {
        Ok(())
    } else {
        Err(MeshioError::Inconsistent(format!("tag '{name}' has {got} values for {want} entities")))
    }
}

struct Built {
    np: usize,
    nc: usize,
    points: Arr,
    cells: Vec<Arr>,
    point_data: Vec<Arr>,
    cell_data: Vec<Arr>,
}

fn build_arrays(m: &dyn MeshRead, o: &VtuOptions) -> Result<Built> {
    let (nv, nc) = (m.count(MeshEntityKind::Vertex), m.count(MeshEntityKind::Cell));
    let co = m.coords();
    let np = co.len();
    if co.y.len() != np || co.z.len() != np || np < nv {
        return Err(MeshioError::Inconsistent("coordinate columns disagree with the vertex count".into()));
    }
    crate::smsh::check_columns(m)?;
    let points = Arr::new("Points", "Float64", 3, pack(3 * np, |i| [co.x, co.y, co.z][i % 3][i / 3]));

    let nblocks = nc.div_ceil(BLOCK);
    let results: Vec<Result<CellBlock>> = (0..nblocks).into_par_iter().map(|b| cell_block(m, b * BLOCK, ((b + 1) * BLOCK).min(nc))).collect();
    let mut blocks = Vec::with_capacity(nblocks);
    for r in results {
        blocks.push(r?);
    }
    let conn_total: usize = blocks.iter().map(|b| b.conn.len()).sum();
    let faces_total: usize = blocks.iter().map(|b| b.faces.len()).sum();
    let wide = np >= 1 << 31 || conn_total >= 1 << 31 || faces_total >= 1 << 31;
    let has_poly = faces_total > 0;

    // Pack straight from the block vectors into the output byte buffers.
    let isz = if wide { 8 } else { 4 };
    let mut types = Vec::with_capacity(nc);
    let mut conn: Vec<u8> = Vec::with_capacity(conn_total * isz);
    let mut offsets: Vec<u8> = Vec::with_capacity(nc * isz);
    let mut faces: Vec<u8> = Vec::with_capacity(faces_total * isz);
    let mut faceoffsets: Vec<u8> = Vec::with_capacity(if has_poly { nc * isz } else { 0 });
    let (mut acc, mut facc) = (0u64, 0u64);
    for b in &blocks {
        types.extend_from_slice(&b.types);
        for v in &b.conn {
            put_idx(&mut conn, u64::from(*v), wide);
        }
        for v in &b.faces {
            put_idx(&mut faces, u64::from(*v), wide);
        }
        for (i, l) in b.conn_len.iter().enumerate() {
            acc += u64::from(*l);
            put_idx(&mut offsets, acc, wide);
            if has_poly {
                // -1 (all ones) marks a cell without a face stream.
                if b.types[i] == VTK_POLYHEDRON {
                    facc += u64::from(b.faces_len[i]);
                    put_idx(&mut faceoffsets, facc, wide);
                } else {
                    put_idx(&mut faceoffsets, u64::MAX, wide);
                }
            }
        }
    }
    let mut cells = vec![idx_arr("connectivity", conn, wide), idx_arr("offsets", offsets, wide), Arr::new("types", "UInt8", 1, types)];
    if has_poly {
        cells.push(idx_arr("faces", faces, wide));
        cells.push(idx_arr("faceoffsets", faceoffsets, wide));
    }

    // Point data.
    let mut pd: Vec<Arr> = Vec::new();
    let vcls: Vec<(u8, i64)> = (0..nv)
        .into_par_iter()
        .map(|i| {
            let c = m.classification(MeshEntityKind::Vertex, i as u32);
            (c.dim().unwrap_or(255), c.entity.serial() as i64)
        })
        .collect();
    let vclass = |i: usize| vcls.get(i).copied().unwrap_or((255, -1));
    pd.push(Arr::new("class_dim", "UInt8", 1, pack(np, |i| vclass(i).0)));
    pd.push(Arr::new("class_entity", "Int64", 1, pack(np, |i| vclass(i).1)));
    if o.set_indicators {
        for s in m.sets() {
            if matches!(s.members, SetMembers::Vertices(_) | SetMembers::Points(_)) {
                let mut flag = vec![0u8; np];
                for w in s.members.raw() {
                    if let Some(f) = flag.get_mut(w as usize) {
                        *f = 1;
                    }
                }
                pd.push(Arr::new(format!("set:{}", s.name), "UInt8", 1, flag));
            }
        }
    }
    if o.tags {
        for t in m.tags().iter().filter(|t| t.target == MeshEntityKind::Vertex) {
            // One value per vertex; extra high-order points past `nv` carry the fill value.
            match &t.values {
                TagValues::I64(v) => {
                    check_tag_len(&t.name, v.len(), nv)?;
                    pd.push(Arr::new(format!("tag:{}", t.name), "Int64", 1, pack(np, |i| v.get(i).copied().unwrap_or(0))));
                }
                TagValues::F64(v) => {
                    check_tag_len(&t.name, v.len(), nv)?;
                    pd.push(Arr::new(format!("tag:{}", t.name), "Float64", 1, pack(np, |i| v.get(i).copied().unwrap_or(f64::NAN))));
                }
                _ => return Err(MeshioError::Unsupported(format!("tag '{}' has an unknown value type", t.name))),
            }
        }
    }

    // Cell data.
    let mut cd: Vec<Arr> = Vec::new();
    cd.push(Arr::new("class_entity", "Int64", 1, pack(nc, |c| m.classification(MeshEntityKind::Cell, c as u32).entity.serial() as i64)));
    let mut region = vec![-1i64; nc];
    let mut any_region = false;
    for s in m.sets() {
        if let (SetMembers::Cells(cells), Some(label)) = (&s.members, s.label) {
            any_region = true;
            for c in cells {
                if let Some(r) = region.get_mut(c.get())
                    && *r == -1
                {
                    *r = label;
                }
            }
        }
    }
    if any_region {
        cd.insert(0, Arr::new("region", "Int64", 1, pack(nc, |c| region[c])));
    }
    if o.set_indicators {
        for s in m.sets() {
            if let SetMembers::Cells(cells) = &s.members {
                let mut flag = vec![0u8; nc];
                for c in cells {
                    if let Some(f) = flag.get_mut(c.get()) {
                        *f = 1;
                    }
                }
                cd.push(Arr::new(format!("set:{}", s.name), "UInt8", 1, flag));
            }
        }
    }
    if o.tags {
        for t in m.tags().iter().filter(|t| t.target == MeshEntityKind::Cell) {
            match &t.values {
                TagValues::I64(v) => {
                    check_tag_len(&t.name, v.len(), nc)?;
                    cd.push(Arr::new(format!("tag:{}", t.name), "Int64", 1, pack_slice(v)));
                }
                TagValues::F64(v) => {
                    check_tag_len(&t.name, v.len(), nc)?;
                    cd.push(Arr::new(format!("tag:{}", t.name), "Float64", 1, pack_slice(v)));
                }
                _ => return Err(MeshioError::Unsupported(format!("tag '{}' has an unknown value type", t.name))),
            }
        }
    }
    if (0..nc).any(|c| !m.cell_active(CellIx::new(c))) {
        cd.push(Arr::new("active", "UInt8", 1, pack(nc, |c| u8::from(m.cell_active(CellIx::new(c))))));
    }
    Ok(Built { np, nc, points, cells, point_data: pd, cell_data: cd })
}

/// Standard base64 (RFC 4648, padded), parallel over fixed 3 KiB-multiple chunks.
fn base64(data: &[u8]) -> Vec<u8> {
    const ALPHA: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const CH: usize = 3 * 4096;
    let mut out = vec![0u8; data.len().div_ceil(3) * 4];
    data.par_chunks(CH).zip(out.par_chunks_mut(4 * 4096)).for_each(|(src, dst)| {
        for (s, d) in src.chunks(3).zip(dst.as_chunks_mut::<4>().0.iter_mut()) {
            let n = (u32::from(s[0]) << 16) | (u32::from(*s.get(1).unwrap_or(&0)) << 8) | u32::from(*s.get(2).unwrap_or(&0));
            d[0] = ALPHA[(n >> 18) as usize & 63];
            d[1] = ALPHA[(n >> 12) as usize & 63];
            d[2] = if s.len() > 1 { ALPHA[(n >> 6) as usize & 63] } else { b'=' };
            d[3] = if s.len() > 2 { ALPHA[n as usize & 63] } else { b'=' };
        }
    });
    out
}

fn esc(s: &str) -> String {
    let mut o = String::with_capacity(s.len());
    for ch in s.chars() {
        match ch {
            '&' => o.push_str("&amp;"),
            '<' => o.push_str("&lt;"),
            '>' => o.push_str("&gt;"),
            '"' => o.push_str("&quot;"),
            '\n' | '\r' | '\t' => o.push(' '),
            c if (c as u32) < 0x20 => o.push(' '),
            c => o.push(c),
        }
    }
    o
}

/// Write the mesh as a VTU file.
///
/// Reads the mesh through [`MeshRead`] only. The bytes are a pure function of the mesh and the
/// options (identical across runs and thread counts).
pub fn write_vtu<W: Write>(mesh: &dyn MeshRead, opts: &VtuOptions, w: &mut W) -> Result<()> {
    let b = build_arrays(mesh, opts)?;
    let all: Vec<&Arr> = std::iter::once(&b.points).chain(&b.cells).chain(&b.point_data).chain(&b.cell_data).collect();
    // Encoded appended blocks, in array order.
    let blocks: Vec<Vec<u8>> = all
        .par_iter()
        .map(|a| match opts.encoding {
            VtuEncoding::AppendedRaw => {
                let mut v = Vec::with_capacity(8 + a.bytes.len());
                v.extend_from_slice(&(a.bytes.len() as u64).to_le_bytes());
                v.extend_from_slice(&a.bytes);
                v
            }
            VtuEncoding::AppendedBase64 => {
                // The block header and the payload are encoded separately (VTK convention).
                let mut v = base64(&(a.bytes.len() as u64).to_le_bytes());
                v.extend_from_slice(&base64(&a.bytes));
                v
            }
        })
        .collect();
    let mut offsets = Vec::with_capacity(blocks.len());
    let mut acc = 0usize;
    for bl in &blocks {
        offsets.push(acc);
        acc += bl.len();
    }
    let mut it = offsets.iter();
    let mut tag = |a: &Arr, pad: &str| -> String {
        let off = it.next().copied().unwrap_or(0);
        let comp = if a.ncomp > 1 { format!(" NumberOfComponents=\"{}\"", a.ncomp) } else { String::new() };
        format!("{pad}<DataArray type=\"{}\" Name=\"{}\"{comp} format=\"appended\" offset=\"{off}\"/>\n", a.ty, esc(&a.name))
    };
    let enc = match opts.encoding {
        VtuEncoding::AppendedRaw => "raw",
        VtuEncoding::AppendedBase64 => "base64",
    };
    let mut x = String::new();
    x.push_str("<?xml version=\"1.0\"?>\n<VTKFile type=\"UnstructuredGrid\" version=\"1.0\" byte_order=\"LittleEndian\" header_type=\"UInt64\">\n  <UnstructuredGrid>\n");
    x.push_str(&format!("    <Piece NumberOfPoints=\"{}\" NumberOfCells=\"{}\">\n", b.np, b.nc));
    x.push_str("      <Points>\n");
    x.push_str(&tag(&b.points, "        "));
    x.push_str("      </Points>\n      <Cells>\n");
    for a in &b.cells {
        x.push_str(&tag(a, "        "));
    }
    x.push_str("      </Cells>\n");
    x.push_str("      <PointData>\n");
    for a in &b.point_data {
        x.push_str(&tag(a, "        "));
    }
    x.push_str("      </PointData>\n");
    let first = b.cell_data.first().map_or(String::new(), |a| format!(" Scalars=\"{}\"", esc(&a.name)));
    x.push_str(&format!("      <CellData{first}>\n"));
    for a in &b.cell_data {
        x.push_str(&tag(a, "        "));
    }
    x.push_str("      </CellData>\n    </Piece>\n  </UnstructuredGrid>\n");
    x.push_str(&format!("  <AppendedData encoding=\"{enc}\">\n   _"));
    w.write_all(x.as_bytes())?;
    for bl in &blocks {
        w.write_all(bl)?;
    }
    w.write_all(b"\n  </AppendedData>\n</VTKFile>\n")?;
    Ok(())
}

/// The VTU file as bytes.
pub fn vtu_bytes(mesh: &dyn MeshRead, opts: &VtuOptions) -> Result<Vec<u8>> {
    let mut v = Vec::new();
    write_vtu(mesh, opts, &mut v)?;
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::base64;

    #[test]
    fn base64_vectors() {
        assert_eq!(base64(b""), b"");
        assert_eq!(base64(b"f"), b"Zg==");
        assert_eq!(base64(b"fo"), b"Zm8=");
        assert_eq!(base64(b"foo"), b"Zm9v");
        assert_eq!(base64(b"foobar"), b"Zm9vYmFy");
        let big: Vec<u8> = (0..40_000u32).map(|i| (i * 31 % 251) as u8).collect();
        let e = base64(&big);
        assert_eq!(e.len(), big.len().div_ceil(3) * 4);
    }
}
