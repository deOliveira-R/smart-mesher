//! The Exodus II writer: mesh and options to an [`NcFile`] and bytes.

use crate::error::{ExodusError, Result};
use crate::netcdf::{DimId, NcData, NcFile, NcFormat, NcType, NcValue, VarId, par_pack};
use crate::options::ExodusOptions;
use crate::plan::{BlockPlan, ExodusPlan, plan};
use crate::tables::exodus_element_type;
use rayon::prelude::*;
use sm_mesh_api::{CellIx, CellShape, MeshRead, Side};
use std::io::Write;

const CHUNK: usize = 1 << 14;
const STR_LEN: usize = 32;
const LINE_LEN: usize = 80;
/// `int64_status` = `EX_ALL_INT64_DB` (maps 0x0400, ids 0x0800, bulk 0x1000), as SEACAS writes it.
const INT64_ALL: i32 = 0x1C00;

/// One element block of a [`ExodusReport`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BlockSummary {
    /// Block id.
    pub id: i64,
    /// Block name.
    pub name: String,
    /// Exodus element type (`HEX8`, `NFACED`, ...).
    pub elem_type: String,
    /// Number of elements.
    pub count: usize,
}

/// One side set or node set of a [`ExodusReport`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SetSummary {
    /// Set id.
    pub id: i64,
    /// Set name.
    pub name: String,
    /// Number of entries.
    pub count: usize,
}

/// What [`write_exodus`] wrote.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ExodusReport {
    /// File length in bytes.
    pub bytes: u64,
    /// netCDF flavour written.
    pub format: NcFormat,
    /// Number of nodes.
    pub num_nodes: usize,
    /// Number of elements.
    pub num_elem: usize,
    /// Element blocks, ascending by id.
    pub blocks: Vec<BlockSummary>,
    /// Side sets.
    pub sidesets: Vec<SetSummary>,
    /// Node sets.
    pub nodesets: Vec<SetSummary>,
    /// Side-set entries dropped because their cell is inactive.
    pub skipped_sides: usize,
}

/// Write `mesh` as an Exodus II file to `w`. Output is a pure function of the mesh and `opts`:
/// no clock, no thread-dependent order. Lazy variables are packed one at a time (in parallel
/// inside a variable, into preallocated output), so peak memory is about the largest variable.
pub fn write_exodus(mesh: &dyn MeshRead, opts: &ExodusOptions, w: &mut dyn Write) -> Result<ExodusReport> {
    let p = plan(mesh, opts)?;
    let file = build(mesh, opts, &p)?;
    let bytes = file.write(w)?;
    Ok(ExodusReport {
        bytes,
        format: opts.format,
        num_nodes: p.num_nodes,
        num_elem: p.num_elem,
        blocks: p
            .blocks
            .iter()
            .map(|b| BlockSummary { id: b.id, name: b.name.clone(), elem_type: exodus_element_type(b.shape).to_string(), count: b.cells.len() })
            .collect(),
        sidesets: p.sidesets.iter().map(|s| SetSummary { id: s.id, name: s.name.clone(), count: s.elems.len() }).collect(),
        nodesets: p.nodesets.iter().map(|s| SetSummary { id: s.id, name: s.name.clone(), count: s.nodes.len() }).collect(),
        skipped_sides: p.skipped_sides,
    })
}

/// [`write_exodus`] into a `Vec`.
pub fn exodus_bytes(mesh: &dyn MeshRead, opts: &ExodusOptions) -> Result<Vec<u8>> {
    let mut v = Vec::new();
    write_exodus(mesh, opts, &mut v)?;
    Ok(v)
}

/// The netCDF model of an export whose [`plan`] is `p` (for tests and CDL dumps);
/// [`write_exodus`] writes the same model.
pub fn exodus_nc<'a>(mesh: &'a dyn MeshRead, opts: &'a ExodusOptions, p: &'a ExodusPlan) -> Result<NcFile<'a>> {
    build(mesh, opts, p)
}

fn too_large(what: &str, value: u64, limit: u64) -> ExodusError {
    ExodusError::TooLarge { what: what.to_string(), value, limit }
}

/// Pack `n` ids; `f(i)` is the i-th value. Parallel over fixed chunks into preallocated output.
fn pack_with(n: usize, int64: bool, f: impl Fn(usize) -> i64 + Sync) -> Vec<u8> {
    let w = if int64 { 8 } else { 4 };
    let mut out = vec![0u8; n * w];
    out.par_chunks_mut(w * CHUNK).enumerate().for_each(|(ci, chunk)| {
        for (j, dst) in chunk.chunks_exact_mut(w).enumerate() {
            let v = f(ci * CHUNK + j);
            if int64 {
                dst.copy_from_slice(&v.to_be_bytes());
            } else {
                dst.copy_from_slice(&(v as i32).to_be_bytes());
            }
        }
    });
    out
}

fn ids_bytes(xs: &[i64], int64: bool) -> Vec<u8> {
    pack_with(xs.len(), int64, |i| xs[i])
}

fn rows_bytes(rows: &[&str], width: usize) -> Vec<u8> {
    let mut out = vec![0u8; rows.len() * width];
    for (r, s) in rows.iter().enumerate() {
        let b = s.as_bytes();
        let n = b.len().min(width);
        out[r * width..r * width + n].copy_from_slice(&b[..n]);
    }
    out
}

/// Split a line into pieces of at most `max` bytes at char boundaries.
fn split_line(s: &str, max: usize) -> Vec<String> {
    if s.len() <= max {
        return vec![s.to_string()];
    }
    let mut out = Vec::new();
    let mut cur = String::new();
    for ch in s.chars() {
        if cur.len() + ch.len_utf8() > max {
            out.push(std::mem::take(&mut cur));
        }
        cur.push(ch);
    }
    out.push(cur);
    out
}

struct PolyFaces {
    /// Face copies per cell of the block.
    per_cell: Vec<u32>,
    /// Nodes per face copy, cell by cell.
    per_face: Vec<u32>,
}

fn poly_faces(mesh: &dyn MeshRead, b: &BlockPlan) -> PolyFaces {
    let mut per_cell = Vec::with_capacity(b.cells.len());
    let mut per_face = Vec::new();
    for &c in &b.cells {
        let cf = mesh.cell_faces(CellIx(c));
        per_cell.push(cf.len() as u32);
        per_face.extend(cf.iter().map(|sf| mesh.face_vertices(sf.face()).len() as u32));
    }
    PolyFaces { per_cell, per_face }
}

/// The outward node loop of a face copy, 1-based, appended to `out`.
fn push_outward(mesh: &dyn MeshRead, sf: sm_mesh_api::SignedFace, mut push: impl FnMut(i64)) {
    let l = mesh.face_vertices(sf.face());
    match sf.side() {
        Side::Left => l.iter().for_each(|v| push(i64::from(v.0) + 1)),
        Side::Right => {
            if let Some(first) = l.first() {
                push(i64::from(first.0) + 1);
                l[1..].iter().rev().for_each(|v| push(i64::from(v.0) + 1));
            }
        }
    }
}

fn build<'a>(mesh: &'a dyn MeshRead, opts: &'a ExodusOptions, p: &'a ExodusPlan) -> Result<NcFile<'a>> {
    if opts.int64 && opts.format != NcFormat::Data64 {
        return Err(ExodusError::Unsupported("int64 ids need NcFormat::Data64".into()));
    }
    let int64 = opts.int64;
    let id_ty = if int64 { NcType::Int64 } else { NcType::Int };
    if opts.title.len() > LINE_LEN {
        return Err(too_large("title", opts.title.len() as u64, LINE_LEN as u64));
    }
    for q in &opts.qa {
        if let Some(s) = q.iter().find(|s| s.len() > STR_LEN) {
            return Err(ExodusError::NameTooLong { name: s.clone() });
        }
    }
    let mut info: Vec<String> = opts.info.iter().flat_map(|l| split_line(l, LINE_LEN)).collect();
    if let Some(pv) = &opts.provenance {
        info.extend(split_line(&format!("smart-mesh provenance: {pv}"), LINE_LEN));
    }

    // Per-block derived data.
    let poly: Vec<Option<PolyFaces>> =
        p.blocks.iter().map(|b| (b.shape == CellShape::Polyhedron).then(|| poly_faces(mesh, b))).collect();
    let nsided_nodes: Vec<usize> = p
        .blocks
        .iter()
        .map(|b| if b.shape == CellShape::Polygon { b.cells.iter().map(|&c| mesh.cell_corners(CellIx(c)).map_or(0, <[_]>::len)).sum() } else { 0 })
        .collect();
    let total_faces: usize = poly.iter().flatten().map(|pf| pf.per_face.len()).sum();
    let n_fa_blk = poly.iter().flatten().count();

    if !int64 {
        let max_id = p.blocks.iter().map(|b| b.id).chain(p.sidesets.iter().map(|s| s.id)).chain(p.nodesets.iter().map(|s| s.id)).max().unwrap_or(0);
        let biggest = (p.num_nodes as u64).max(p.num_elem as u64).max(max_id as u64).max(total_faces as u64);
        if biggest > i32::MAX as u64 {
            return Err(too_large("id or count (use int64)", biggest, i32::MAX as u64));
        }
    }

    let len_name = u64::from(p.max_name_length) + 1;
    let mut f = NcFile::new(opts.format);
    let d_time = f.dim("time_step", None);
    let d_str = f.dim("len_string", Some(STR_LEN as u64 + 1));
    let d_line = f.dim("len_line", Some(LINE_LEN as u64 + 1));
    let d_four = f.dim("four", Some(4));
    let d_name = f.dim("len_name", Some(len_name));
    let d_ndim = f.dim("num_dim", Some(u64::from(p.num_dim)));
    let d_nodes = f.dim("num_nodes", Some(p.num_nodes as u64));
    let d_elem = f.dim("num_elem", Some(p.num_elem as u64));
    let d_fablk = (n_fa_blk > 0).then(|| {
        f.dim("num_face", Some(total_faces as u64));
        f.dim("num_fa_blk", Some(n_fa_blk as u64))
    });
    let d_elblk = f.dim("num_el_blk", Some(p.blocks.len() as u64));
    let d_nns = (!p.nodesets.is_empty()).then(|| f.dim("num_node_sets", Some(p.nodesets.len() as u64)));
    let d_nss = (!p.sidesets.is_empty()).then(|| f.dim("num_side_sets", Some(p.sidesets.len() as u64)));
    let d_qa = (!opts.qa.is_empty()).then(|| f.dim("num_qa_rec", Some(opts.qa.len() as u64)));
    let d_info = (!info.is_empty()).then(|| f.dim("num_info", Some(info.len() as u64)));

    f.global_attr("api_version", NcValue::Float(vec![8.11]));
    f.global_attr("version", NcValue::Float(vec![8.11]));
    f.global_attr("floating_point_word_size", NcValue::Int(vec![8]));
    f.global_attr("file_size", NcValue::Int(vec![1]));
    f.global_attr("maximum_name_length", NcValue::Int(vec![p.max_name_length as i32]));
    // libexodus bitmask (recalled, unverified): MAPS 1 | IDS 2 | BULK 4 | INQ 8. Maps, ids and
    // bulk data are all NC_INT64 here, and 0x1C00 (7168) is `EX_ALL_INT64_DB`, the value libexodus writes.
    f.global_attr("int64_status", NcValue::Int(vec![if int64 { INT64_ALL } else { 0 }]));
    f.global_attr("title", NcValue::Text(opts.title.clone()));

    f.var("time_whole", NcType::Double, &[d_time]);

    // Coordinates.
    let co = mesh.coords();
    let nv = p.num_nodes;
    let axes: [(&str, &'a [f64]); 3] = [("coordx", &co.x[..nv]), ("coordy", &co.y[..nv]), ("coordz", &co.z[..nv])];
    for (name, xs) in axes.iter().take(usize::from(p.num_dim)) {
        let v = f.var(name, NcType::Double, &[d_nodes]);
        let xs: &'a [f64] = xs;
        f.set_data(v, NcData::Lazy(Box::new(move || par_pack(xs, f64::to_be_bytes))));
    }
    let cn = f.var("coor_names", NcType::Char, &[d_ndim, d_name]);
    let coor: Vec<&str> = ["x", "y", "z"][..usize::from(p.num_dim)].to_vec();
    let lw = len_name as usize;
    f.set_data(cn, NcData::Bytes(rows_bytes(&coor, lw)));

    // Meta variables of a family of entities.
    let meta = |f: &mut NcFile<'a>, prefix: &str, dim: DimId, ids: &[i64], names: &[&str]| {
        let st = f.var(&format!("{prefix}_status"), id_ty, &[dim]);
        f.set_data(st, NcData::Bytes(ids_bytes(&vec![1; ids.len()], int64)));
        let pr = f.var(&format!("{prefix}_prop1"), id_ty, &[dim]);
        f.var_attr(pr, "name", NcValue::Text("ID".into()));
        f.set_data(pr, NcData::Bytes(ids_bytes(ids, int64)));
        let nm = f.var(&format!("{prefix}_names"), NcType::Char, &[dim, d_name]);
        f.set_data(nm, NcData::Bytes(rows_bytes(names, lw)));
    };
    let ids: Vec<i64> = p.blocks.iter().map(|b| b.id).collect();
    let names: Vec<&str> = p.blocks.iter().map(|b| b.name.as_str()).collect();
    meta(&mut f, "eb", d_elblk, &ids, &names);

    // Blocks.
    let mut fblk = 0usize;
    let mut face_base = 0usize;
    for (bi, b) in p.blocks.iter().enumerate() {
        let k = bi + 1;
        let d_el = f.dim(&format!("num_el_in_blk{k}"), Some(b.cells.len() as u64));
        let cells: &'a [u32] = &b.cells;
        match b.shape {
            CellShape::Polyhedron => {
                let Some(pf) = poly[bi].as_ref() else { continue };
                fblk += 1;
                let nfa = pf.per_face.len();
                let nnod: usize = pf.per_face.iter().map(|x| *x as usize).sum();
                let d_fa = f.dim(&format!("num_fac_per_el{k}"), Some(nfa as u64));
                let v = f.var(&format!("facconn{k}"), id_ty, &[d_fa]);
                f.var_attr(v, "elem_type", NcValue::Text("NFACED".into()));
                let base = face_base;
                f.set_data(v, NcData::Bytes(pack_with(nfa, int64, |i| (base + i) as i64 + 1)));
                let e = f.var(&format!("ebepecnt{k}"), id_ty, &[d_el]);
                f.var_attr(e, "entity_type1", NcValue::Text("FACE".into()));
                f.var_attr(e, "entity_type2", NcValue::Text("ELEM".into()));
                let per_cell: Vec<i64> = pf.per_cell.iter().map(|x| i64::from(*x)).collect();
                f.set_data(e, NcData::Bytes(ids_bytes(&per_cell, int64)));
                // Face block j = fblk.
                let d_ff = f.dim(&format!("num_fa_in_blk{fblk}"), Some(nfa as u64));
                let d_fn = f.dim(&format!("num_nod_per_fa{fblk}"), Some(nnod as u64));
                let fc = f.var(&format!("fbconn{fblk}"), id_ty, &[d_fn]);
                f.var_attr(fc, "elem_type", NcValue::Text("NSIDED".into()));
                f.set_data(
                    fc,
                    NcData::Lazy(Box::new(move || {
                        let mut vals: Vec<i64> = Vec::with_capacity(nnod);
                        for &c in cells {
                            for sf in mesh.cell_faces(CellIx(c)) {
                                push_outward(mesh, *sf, |x| vals.push(x));
                            }
                        }
                        ids_bytes(&vals, int64)
                    })),
                );
                let fe = f.var(&format!("fbepecnt{fblk}"), id_ty, &[d_ff]);
                f.var_attr(fe, "entity_type1", NcValue::Text("NODE".into()));
                f.var_attr(fe, "entity_type2", NcValue::Text("FACE".into()));
                let per_face: Vec<i64> = pf.per_face.iter().map(|x| i64::from(*x)).collect();
                f.set_data(fe, NcData::Bytes(ids_bytes(&per_face, int64)));
                face_base += nfa;
            }
            CellShape::Polygon => {
                let d_n = f.dim(&format!("num_nod_per_el{k}"), Some(nsided_nodes[bi] as u64));
                let v = f.var(&format!("connect{k}"), id_ty, &[d_n]);
                f.var_attr(v, "elem_type", NcValue::Text("NSIDED".into()));
                f.set_data(
                    v,
                    NcData::Lazy(Box::new(move || {
                        let mut vals: Vec<i64> = Vec::new();
                        for &c in cells {
                            if let Some(cs) = mesh.cell_corners(CellIx(c)) {
                                vals.extend(cs.iter().map(|v| i64::from(v.0) + 1));
                            }
                        }
                        ids_bytes(&vals, int64)
                    })),
                );
                let e = f.var(&format!("ebepecnt{k}"), id_ty, &[d_el]);
                f.var_attr(e, "entity_type1", NcValue::Text("NODE".into()));
                f.var_attr(e, "entity_type2", NcValue::Text("ELEM".into()));
                let counts: Vec<i64> = cells.iter().map(|&c| mesh.cell_corners(CellIx(c)).map_or(0, <[_]>::len) as i64).collect();
                f.set_data(e, NcData::Bytes(ids_bytes(&counts, int64)));
            }
            shape => {
                let npe = shape.vertex_count().unwrap_or(0);
                let d_n = f.dim(&format!("num_nod_per_el{k}"), Some(npe as u64));
                let v = f.var(&format!("connect{k}"), id_ty, &[d_el, d_n]);
                f.var_attr(v, "elem_type", NcValue::Text(exodus_element_type(shape).into()));
                f.set_data(v, NcData::Lazy(Box::new(move || template_connectivity(mesh, cells, npe, int64))));
            }
        }
    }
    if let Some(d_fablk) = d_fablk {
        let fids: Vec<i64> = p.blocks.iter().zip(&poly).filter(|(_, pf)| pf.is_some()).map(|(b, _)| b.id).collect();
        let fnames: Vec<&str> = p.blocks.iter().zip(&poly).filter(|(_, pf)| pf.is_some()).map(|(b, _)| b.name.as_str()).collect();
        meta(&mut f, "fa", d_fablk, &fids, &fnames);
    }

    // Node sets.
    if let Some(d_nns) = d_nns {
        let ids: Vec<i64> = p.nodesets.iter().map(|s| s.id).collect();
        let names: Vec<&str> = p.nodesets.iter().map(|s| s.name.as_str()).collect();
        meta(&mut f, "ns", d_nns, &ids, &names);
        for (i, s) in p.nodesets.iter().enumerate() {
            let k = i + 1;
            let d = f.dim(&format!("num_nod_ns{k}"), Some(s.nodes.len() as u64));
            let v = f.var(&format!("node_ns{k}"), id_ty, &[d]);
            let nodes: &'a [u32] = &s.nodes;
            f.set_data(v, NcData::Lazy(Box::new(move || pack_with(nodes.len(), int64, |j| i64::from(nodes[j]) + 1))));
        }
    }
    // Side sets.
    if let Some(d_nss) = d_nss {
        let ids: Vec<i64> = p.sidesets.iter().map(|s| s.id).collect();
        let names: Vec<&str> = p.sidesets.iter().map(|s| s.name.as_str()).collect();
        meta(&mut f, "ss", d_nss, &ids, &names);
        for (i, s) in p.sidesets.iter().enumerate() {
            let k = i + 1;
            let d = f.dim(&format!("num_side_ss{k}"), Some(s.elems.len() as u64));
            let ev = f.var(&format!("elem_ss{k}"), id_ty, &[d]);
            let sv = f.var(&format!("side_ss{k}"), id_ty, &[d]);
            let (elems, sides): (&'a [u32], &'a [u32]) = (&s.elems, &s.sides);
            f.set_data(ev, NcData::Lazy(Box::new(move || pack_with(elems.len(), int64, |j| i64::from(elems[j])))));
            f.set_data(sv, NcData::Lazy(Box::new(move || pack_with(sides.len(), int64, |j| i64::from(sides[j])))));
        }
    }

    // Maps.
    let en: VarId = f.var("elem_num_map", id_ty, &[d_elem]);
    let order: &'a [u32] = &p.elem_order;
    f.set_data(en, NcData::Lazy(Box::new(move || pack_with(order.len(), int64, |j| i64::from(order[j]) + 1))));
    if opts.write_node_num_map {
        let nn = f.var("node_num_map", id_ty, &[d_nodes]);
        f.set_data(nn, NcData::Lazy(Box::new(move || pack_with(nv, int64, |j| j as i64 + 1))));
    }

    // QA and info.
    if let Some(d_qa) = d_qa {
        let v = f.var("qa_records", NcType::Char, &[d_qa, d_four, d_str]);
        let flat: Vec<&str> = opts.qa.iter().flat_map(|r| r.iter().map(String::as_str)).collect();
        f.set_data(v, NcData::Bytes(rows_bytes(&flat, STR_LEN + 1)));
    }
    if let Some(d_info) = d_info {
        let v = f.var("info_records", NcType::Char, &[d_info, d_line]);
        let rows: Vec<&str> = info.iter().map(String::as_str).collect();
        f.set_data(v, NcData::Bytes(rows_bytes(&rows, LINE_LEN + 1)));
    }
    Ok(f)
}

/// `cell_corners + 1` for every cell, one element per chunk, in parallel into preallocated output.
fn template_connectivity(mesh: &dyn MeshRead, cells: &[u32], npe: usize, int64: bool) -> Vec<u8> {
    let w = if int64 { 8 } else { 4 };
    let mut out = vec![0u8; cells.len() * npe * w];
    out.par_chunks_mut(npe * w).zip(cells.par_iter()).with_min_len(256).for_each(|(dst, &c)| {
        if let Some(corners) = mesh.cell_corners(CellIx(c)) {
            for (d, v) in dst.chunks_exact_mut(w).zip(corners) {
                let x = i64::from(v.0) + 1;
                if int64 {
                    d.copy_from_slice(&x.to_be_bytes());
                } else {
                    d.copy_from_slice(&(x as i32).to_be_bytes());
                }
            }
        }
    });
    out
}
