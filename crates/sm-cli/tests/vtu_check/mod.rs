//! A small independent VTU reader for the milestone (appended raw or base64 data, `UInt64`
//! headers, the types sm-meshio writes), plus the geometric checks of plan §4.6 row 6 done by
//! hand on HEX8 cells. `vtu_check.py` (next to this file) is the stdlib Python twin.
#![allow(dead_code)]

use std::collections::BTreeMap;

/// One decoded data array, widened to `f64` (every integer the milestone sees fits exactly).
#[derive(Clone, Debug, PartialEq)]
pub struct Array {
    pub ty: String,
    pub ncomp: usize,
    pub values: Vec<f64>,
}

/// The decoded file: point and cell counts and every array by `section/name`.
#[derive(Clone, Debug, PartialEq)]
pub struct Vtu {
    pub np: usize,
    pub nc: usize,
    pub arrays: BTreeMap<String, Array>,
}

fn attr(tag: &str, name: &str) -> Option<String> {
    let key = format!(" {name}=\"");
    let i = tag.find(&key)? + key.len();
    let j = tag[i..].find('"')? + i;
    Some(tag[i..j].to_owned())
}

fn b64(s: &[u8]) -> Result<Vec<u8>, String> {
    let val = |c: u8| -> Result<u32, String> {
        Ok(match c {
            b'A'..=b'Z' => u32::from(c - b'A'),
            b'a'..=b'z' => u32::from(c - b'a') + 26,
            b'0'..=b'9' => u32::from(c - b'0') + 52,
            b'+' => 62,
            b'/' => 63,
            _ => return Err(format!("bad base64 byte {c}")),
        })
    };
    let mut out = Vec::new();
    for chunk in s.chunks(4) {
        let pad = chunk.iter().filter(|&&c| c == b'=').count();
        let mut w = 0u32;
        for (k, &c) in chunk.iter().enumerate() {
            w |= if c == b'=' { 0 } else { val(c)? } << (18 - 6 * k);
        }
        let bytes = w.to_be_bytes();
        out.extend_from_slice(&bytes[1..4 - pad]);
    }
    Ok(out)
}

/// Decode `data` (a whole `.vtu` file).
pub fn decode(data: &[u8]) -> Result<Vtu, String> {
    let find = |hay: &[u8], pat: &[u8], from: usize| -> Option<usize> {
        hay[from..].windows(pat.len()).position(|w| w == pat).map(|p| p + from)
    };
    let k = find(data, b"<AppendedData", 0).ok_or("no AppendedData")?;
    let head = std::str::from_utf8(&data[..k]).map_err(|e| e.to_string())?;
    let enc_tag_end = find(data, b">", k).ok_or("unterminated AppendedData")?;
    let enc_tag = std::str::from_utf8(&data[k..enc_tag_end]).map_err(|e| e.to_string())?;
    let enc = attr(enc_tag, "encoding").ok_or("no encoding")?;
    let body_start = find(data, b"_", enc_tag_end).ok_or("no `_` marker")? + 1;
    let body = &data[body_start..];
    let piece_at = head.find("<Piece ").ok_or("no Piece")?;
    let piece = &head[piece_at..head[piece_at..].find('>').ok_or("bad Piece")? + piece_at];
    let num = |n: &str| -> Result<usize, String> {
        attr(piece, n).ok_or(format!("no {n}"))?.parse().map_err(|e| format!("{n}: {e}"))
    };
    let (np, nc) = (num("NumberOfPoints")?, num("NumberOfCells")?);
    let mut arrays = BTreeMap::new();
    let mut section = String::new();
    for raw_tag in head.split('<').skip(1) {
        let tag = raw_tag.split('>').next().unwrap_or("");
        let name = tag.split_whitespace().next().unwrap_or("");
        if matches!(name, "Points" | "Cells" | "PointData" | "CellData") {
            section = name.to_owned();
        }
        if name != "DataArray" {
            continue;
        }
        let tag = format!(" {tag}");
        let ty = attr(&tag, "type").ok_or("DataArray without type")?;
        let aname = attr(&tag, "Name").ok_or("DataArray without Name")?;
        let ncomp = attr(&tag, "NumberOfComponents").map_or(Ok(1), |s| s.parse::<usize>()).map_err(|e| e.to_string())?;
        let off: usize = attr(&tag, "offset").ok_or("no offset")?.parse().map_err(|e| format!("offset: {e}"))?;
        let bytes = match enc.as_str() {
            "raw" => {
                let n = u64::from_le_bytes(body.get(off..off + 8).ok_or("short header")?.try_into().map_err(|_| "header")?) as usize;
                body.get(off + 8..off + 8 + n).ok_or("short array")?.to_vec()
            }
            "base64" => {
                let h = b64(body.get(off..off + 12).ok_or("short b64 header")?)?;
                let n = u64::from_le_bytes(h.get(..8).ok_or("b64 header")?.try_into().map_err(|_| "header")?) as usize;
                let len = n.div_ceil(3) * 4;
                let mut v = b64(body.get(off + 12..off + 12 + len).ok_or("short b64 array")?)?;
                v.truncate(n);
                v
            }
            e => return Err(format!("unknown encoding {e}")),
        };
        let values: Vec<f64> = match ty.as_str() {
            "Float64" => bytes.as_chunks::<8>().0.iter().map(|c| f64::from_le_bytes(*c)).collect(),
            "Int64" => bytes.as_chunks::<8>().0.iter().map(|c| i64::from_le_bytes(*c) as f64).collect(),
            "Int32" => bytes.as_chunks::<4>().0.iter().map(|c| f64::from(i32::from_le_bytes(*c))).collect(),
            "UInt8" => bytes.iter().map(|&b| f64::from(b)).collect(),
            t => return Err(format!("unsupported type {t}")),
        };
        arrays.insert(format!("{section}/{aname}"), Array { ty, ncomp, values });
    }
    Ok(Vtu { np, nc, arrays })
}

/// VTK_HEXAHEDRON.
pub const VTK_HEX: f64 = 12.0;

/// The three edge neighbours of each HEX8 corner (VTK order), right-handed.
const HEX_NBRS: [[usize; 3]; 8] =
    [[1, 3, 4], [2, 0, 5], [3, 1, 6], [0, 2, 7], [7, 5, 0], [4, 6, 1], [5, 7, 2], [6, 4, 3]];

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn det(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> f64 {
    a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
}
fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// Minimum corner scaled Jacobian of a HEX8 (Verdict's definition).
pub fn hex_scaled_jacobian(p: &[[f64; 3]; 8]) -> f64 {
    let mut m = f64::INFINITY;
    for (i, nb) in HEX_NBRS.iter().enumerate() {
        let (a, b, c) = (sub(p[nb[0]], p[i]), sub(p[nb[1]], p[i]), sub(p[nb[2]], p[i]));
        let l = norm(a) * norm(b) * norm(c);
        m = m.min(if l > 0.0 { det(a, b, c) / l } else { f64::NEG_INFINITY });
    }
    m
}

/// Volume of a planar-faced HEX8 (six tetrahedra around the 0-6 diagonal: exact for planar faces).
pub fn hex_volume(p: &[[f64; 3]; 8]) -> f64 {
    const TETS: [[usize; 4]; 6] = [[0, 1, 2, 6], [0, 2, 3, 6], [0, 3, 7, 6], [0, 7, 4, 6], [0, 4, 5, 6], [0, 5, 1, 6]];
    TETS.iter()
        .map(|t| det(sub(p[t[1]], p[t[0]]), sub(p[t[2]], p[t[0]]), sub(p[t[3]], p[t[0]])) / 6.0)
        .sum()
}

/// Facts of a decoded hex mesh, for row 6.
#[derive(Debug)]
pub struct HexFacts {
    pub points: Vec<[f64; 3]>,
    pub cells: Vec<[usize; 8]>,
    pub all_hex8: bool,
    pub total_volume: f64,
    pub min_scaled_jacobian: f64,
    pub max_lattice_dev: f64,
    /// Every point rounds to a distinct lattice point of `{-5..5}^3`.
    pub lattice_bijective: bool,
    /// Every cell has 8 distinct, in-range corners.
    pub cells_valid: bool,
}

/// Points, cells and the hand-computed checks.
pub fn hex_facts(v: &Vtu) -> Result<HexFacts, String> {
    let get = |k: &str| v.arrays.get(k).ok_or(format!("missing array {k}"));
    let pts = get("Points/Points")?;
    let conn = get("Cells/connectivity")?;
    let offs = get("Cells/offsets")?;
    let types = get("Cells/types")?;
    if pts.ncomp != 3 || pts.values.len() != 3 * v.np {
        return Err("Points shape".into());
    }
    let points: Vec<[f64; 3]> = pts.values.as_chunks::<3>().0.to_vec();
    let all_hex8 = types.values.len() == v.nc && types.values.iter().all(|&t| t == VTK_HEX);
    let mut cells = Vec::with_capacity(v.nc);
    let mut start = 0usize;
    let mut cells_valid = offs.values.len() == v.nc;
    for &end in &offs.values {
        let end = end as usize;
        if end < start + 8 || end > conn.values.len() || end - start != 8 {
            cells_valid = false;
            break;
        }
        let mut c = [0usize; 8];
        for (k, x) in conn.values[start..end].iter().enumerate() {
            c[k] = *x as usize;
        }
        let mut s = c;
        s.sort_unstable();
        if s.windows(2).any(|w| w[0] == w[1]) || s[7] >= v.np {
            cells_valid = false;
        }
        cells.push(c);
        start = end;
    }
    let (mut total_volume, mut min_sj) = (0.0, f64::INFINITY);
    if cells_valid {
        for c in &cells {
            let p: [[f64; 3]; 8] = std::array::from_fn(|k| points[c[k]]);
            total_volume += hex_volume(&p);
            min_sj = min_sj.min(hex_scaled_jacobian(&p));
        }
    }
    let mut max_dev: f64 = 0.0;
    let mut seen = std::collections::BTreeSet::new();
    let mut in_range = true;
    for p in &points {
        let r = p.map(f64::round);
        for k in 0..3 {
            max_dev = max_dev.max((p[k] - r[k]).abs());
            in_range &= (-5.0..=5.0).contains(&r[k]);
        }
        seen.insert(r.map(|x| x as i64));
    }
    Ok(HexFacts {
        lattice_bijective: in_range && seen.len() == points.len() && points.len() == 1331,
        points,
        cells,
        all_hex8,
        total_volume,
        min_scaled_jacobian: min_sj,
        max_lattice_dev: max_dev,
        cells_valid,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_cube_has_volume_one_and_jacobian_one() {
        let p = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
        assert!((hex_volume(&p) - 1.0).abs() < 1e-15);
        assert!((hex_scaled_jacobian(&p) - 1.0).abs() < 1e-15);
        let mut q = p;
        q[6] = [2., 2., 2.];
        assert!(hex_scaled_jacobian(&q) < 1.0);
    }

    #[test]
    fn base64_round_trips() {
        assert_eq!(b64(b"aGVsbG8=").unwrap(), b"hello");
        assert_eq!(b64(b"aGVsbG8h").unwrap(), b"hello!");
    }
}
