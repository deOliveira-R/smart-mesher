//! An internal decoder of the files this crate writes, for round-trip tests.
//!
//! This is **not** a general Exodus reader: it understands exactly the dimensions and variables
//! [`crate::write_exodus`] emits (no time steps, results, attributes or distribution factors).

use crate::error::{ExodusError, Result};
use crate::netcdf::{NcFile, NcFormat, NcType, NcValue};

/// An element block as stored in the file.
#[derive(Clone, Debug, PartialEq)]
pub struct BlockData {
    /// Block id (`eb_prop1`).
    pub id: i64,
    /// Block name (`eb_names`).
    pub name: String,
    /// `elem_type` attribute (`HEX8`, `NFACED`, ...).
    pub elem_type: String,
    /// Number of elements (`num_el_in_blk<k>`).
    pub num_elem: usize,
    /// `connect<k>`, 1-based, row-major (`num_elem x nodes_per_elem` for templates, the
    /// concatenated node lists for `NSIDED`); empty for `NFACED`.
    pub connectivity: Vec<i64>,
    /// `facconn<k>` (1-based face numbers, `NFACED` only).
    pub face_connectivity: Vec<i64>,
    /// `ebepecnt<k>` (entities per element, polygon / polyhedron blocks only).
    pub entity_counts: Vec<i64>,
}

/// A face block (the `NSIDED` faces referenced by `NFACED` elements).
#[derive(Clone, Debug, PartialEq)]
pub struct FaceBlockData {
    /// Block id (`fa_prop1`).
    pub id: i64,
    /// Block name.
    pub name: String,
    /// Number of faces.
    pub num_faces: usize,
    /// `fbconn<j>`: concatenated 1-based node loops.
    pub connectivity: Vec<i64>,
    /// `fbepecnt<j>`: nodes per face.
    pub entity_counts: Vec<i64>,
}

/// A side set.
#[derive(Clone, Debug, PartialEq)]
pub struct SideSetData {
    /// Set id.
    pub id: i64,
    /// Name.
    pub name: String,
    /// 1-based element numbers.
    pub elems: Vec<i64>,
    /// Exodus sides.
    pub sides: Vec<i64>,
}

/// A node set.
#[derive(Clone, Debug, PartialEq)]
pub struct NodeSetData {
    /// Set id.
    pub id: i64,
    /// Name.
    pub name: String,
    /// 1-based node numbers.
    pub nodes: Vec<i64>,
}

/// Owned decode of an Exodus file written by this crate.
#[derive(Clone, Debug, PartialEq)]
pub struct ExodusData {
    /// netCDF flavour of the file.
    pub format: NcFormat,
    /// `int64_status != 0`.
    pub int64: bool,
    /// Title.
    pub title: String,
    /// `maximum_name_length`.
    pub max_name_length: u32,
    /// `num_dim`.
    pub num_dim: usize,
    /// `num_nodes`.
    pub num_nodes: usize,
    /// `num_elem`.
    pub num_elem: usize,
    /// `coordx|y|z` (as many as `num_dim`).
    pub coords: Vec<Vec<f64>>,
    /// `coor_names`.
    pub coord_names: Vec<String>,
    /// Element blocks in file order.
    pub blocks: Vec<BlockData>,
    /// Face blocks in file order.
    pub face_blocks: Vec<FaceBlockData>,
    /// Side sets in file order.
    pub sidesets: Vec<SideSetData>,
    /// Node sets in file order.
    pub nodesets: Vec<NodeSetData>,
    /// `elem_num_map`.
    pub elem_num_map: Vec<i64>,
    /// `node_num_map`, if present.
    pub node_num_map: Option<Vec<i64>>,
    /// QA records.
    pub qa: Vec<[String; 4]>,
    /// Info records.
    pub info: Vec<String>,
}

fn missing(what: &str) -> ExodusError {
    ExodusError::Malformed(format!("missing {what}"))
}

fn ints(f: &NcFile<'_>, name: &str) -> Result<Vec<i64>> {
    let v = f.var_by_name(name).ok_or_else(|| missing(name))?;
    let b = f.var_bytes(name).ok_or_else(|| missing(name))?;
    match v.ty {
        NcType::Int => Ok(b.as_chunks::<4>().0.iter().map(|c| i64::from(i32::from_be_bytes(*c))).collect()),
        NcType::Int64 => Ok(b.as_chunks::<8>().0.iter().map(|c| i64::from_be_bytes(*c)).collect()),
        _ => Err(ExodusError::Malformed(format!("variable '{name}' is not an integer"))),
    }
}

fn opt_ints(f: &NcFile<'_>, name: &str) -> Result<Vec<i64>> {
    if f.var_by_name(name).is_some() { ints(f, name) } else { Ok(Vec::new()) }
}

fn doubles(f: &NcFile<'_>, name: &str) -> Result<Vec<f64>> {
    let b = f.var_bytes(name).ok_or_else(|| missing(name))?;
    Ok(b.as_chunks::<8>().0.iter().map(|c| f64::from_be_bytes(*c)).collect())
}

fn rows(f: &NcFile<'_>, name: &str, width: usize) -> Result<Vec<String>> {
    let b = f.var_bytes(name).ok_or_else(|| missing(name))?;
    Ok(b.chunks(width.max(1))
        .map(|r| {
            let end = r.iter().rposition(|c| *c != 0).map_or(0, |p| p + 1);
            String::from_utf8_lossy(&r[..end]).into_owned()
        })
        .collect())
}

fn dim(f: &NcFile<'_>, name: &str) -> Result<usize> {
    f.dim_len(name).map(|n| n as usize).ok_or_else(|| missing(name))
}

fn opt_dim(f: &NcFile<'_>, name: &str) -> usize {
    f.dim_len(name).map_or(0, |n| n as usize)
}

fn elem_type(f: &NcFile<'_>, vars: &[String]) -> String {
    vars.iter()
        .filter_map(|v| f.var_by_name(v))
        .flat_map(|v| v.attrs.iter())
        .find_map(|(n, a)| (n == "elem_type").then(|| a.as_text().unwrap_or("").to_string()))
        .unwrap_or_default()
}

/// Decode a file written by [`crate::write_exodus`] (any of the three netCDF formats).
///
/// Errors are `Malformed` / `Unsupported`; it never panics on bad input.
pub fn read_exodus(bytes: &[u8]) -> Result<ExodusData> {
    let f = NcFile::read(bytes)?;
    let text = |n: &str| f.global_attr_by_name(n).and_then(NcValue::as_text).map(str::to_string);
    let int_attr = |n: &str| match f.global_attr_by_name(n) {
        Some(NcValue::Int(v)) => v.first().copied(),
        _ => None,
    };
    let len_name = dim(&f, "len_name")?;
    let num_dim = dim(&f, "num_dim")?;
    let num_nodes = dim(&f, "num_nodes")?;
    let num_elem = dim(&f, "num_elem")?;
    let mut coords = Vec::new();
    for ax in ["coordx", "coordy", "coordz"].iter().take(num_dim) {
        coords.push(doubles(&f, ax)?);
    }
    let n_blk = dim(&f, "num_el_blk")?;
    let ids = ints(&f, "eb_prop1")?;
    let names = rows(&f, "eb_names", len_name)?;
    if ids.len() != n_blk || names.len() != n_blk {
        return Err(ExodusError::Malformed("block metadata length".into()));
    }
    let mut blocks = Vec::new();
    for k in 1..=n_blk {
        let et = elem_type(&f, &[format!("connect{k}"), format!("facconn{k}")]);
        blocks.push(BlockData {
            id: ids[k - 1],
            name: names[k - 1].clone(),
            elem_type: et,
            num_elem: dim(&f, &format!("num_el_in_blk{k}"))?,
            connectivity: opt_ints(&f, &format!("connect{k}"))?,
            face_connectivity: opt_ints(&f, &format!("facconn{k}"))?,
            entity_counts: opt_ints(&f, &format!("ebepecnt{k}"))?,
        });
    }
    let mut face_blocks = Vec::new();
    let n_fa = opt_dim(&f, "num_fa_blk");
    if n_fa > 0 {
        let ids = ints(&f, "fa_prop1")?;
        let names = rows(&f, "fa_names", len_name)?;
        for j in 1..=n_fa {
            face_blocks.push(FaceBlockData {
                id: *ids.get(j - 1).ok_or_else(|| missing("fa_prop1 entry"))?,
                name: names.get(j - 1).cloned().ok_or_else(|| missing("fa_names entry"))?,
                num_faces: dim(&f, &format!("num_fa_in_fblk{j}"))?,
                connectivity: ints(&f, &format!("fbconn{j}"))?,
                entity_counts: ints(&f, &format!("fbepecnt{j}"))?,
            });
        }
    }
    let mut nodesets = Vec::new();
    let n_ns = opt_dim(&f, "num_node_sets");
    if n_ns > 0 {
        let ids = ints(&f, "ns_prop1")?;
        let names = rows(&f, "ns_names", len_name)?;
        for k in 1..=n_ns {
            nodesets.push(NodeSetData {
                id: *ids.get(k - 1).ok_or_else(|| missing("ns_prop1 entry"))?,
                name: names.get(k - 1).cloned().ok_or_else(|| missing("ns_names entry"))?,
                nodes: ints(&f, &format!("node_ns{k}"))?,
            });
        }
    }
    let mut sidesets = Vec::new();
    let n_ss = opt_dim(&f, "num_side_sets");
    if n_ss > 0 {
        let ids = ints(&f, "ss_prop1")?;
        let names = rows(&f, "ss_names", len_name)?;
        for k in 1..=n_ss {
            sidesets.push(SideSetData {
                id: *ids.get(k - 1).ok_or_else(|| missing("ss_prop1 entry"))?,
                name: names.get(k - 1).cloned().ok_or_else(|| missing("ss_names entry"))?,
                elems: ints(&f, &format!("elem_ss{k}"))?,
                sides: ints(&f, &format!("side_ss{k}"))?,
            });
        }
    }
    let mut qa = Vec::new();
    if opt_dim(&f, "num_qa_rec") > 0 {
        let flat = rows(&f, "qa_records", dim(&f, "len_string")?)?;
        for r in flat.as_chunks::<4>().0 {
            qa.push([r[0].clone(), r[1].clone(), r[2].clone(), r[3].clone()]);
        }
    }
    let info = if opt_dim(&f, "num_info") > 0 { rows(&f, "info_records", dim(&f, "len_line")?)? } else { Vec::new() };
    Ok(ExodusData {
        format: f.format(),
        int64: int_attr("int64_status").unwrap_or(0) != 0,
        title: text("title").unwrap_or_default(),
        max_name_length: int_attr("maximum_name_length").unwrap_or(0) as u32,
        num_dim,
        num_nodes,
        num_elem,
        coords,
        coord_names: rows(&f, "coor_names", len_name)?,
        blocks,
        face_blocks,
        sidesets,
        nodesets,
        elem_num_map: ints(&f, "elem_num_map")?,
        node_num_map: if f.var_by_name("node_num_map").is_some() { Some(ints(&f, "node_num_map")?) } else { None },
        qa,
        info,
    })
}
