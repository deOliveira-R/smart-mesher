//! The Fluent `.msh` writer.

use super::{FluentBc, FluentError, FluentOptions, FluentReport, FluentZone};
use crate::error::MeshioError;
use crate::foam::{FoamOptions, PatchOrigin, build_polymesh};
use crate::report::{LossItem, LossReport, LossSeverity};
use rayon::prelude::*;
use sm_mesh_api::MeshRead;
use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;

const NONE: u32 = u32::MAX;
/// Entities formatted per parallel task.
const CHUNK: usize = 8192;
/// Fluent element-type codes (section 12).
const T_TET: u8 = 2;
const T_HEX: u8 = 4;
const T_PYRAMID: u8 = 5;
const T_WEDGE: u8 = 6;
const T_POLY: u8 = 7;
/// Mixed face and cell zones.
const MIXED: u32 = 0;

fn fnv(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bytes {
        h = (h ^ u64::from(*b)).wrapping_mul(0x0100_0000_01b3);
    }
    h
}

fn hex(buf: &mut Vec<u8>, mut v: u64) {
    const D: &[u8; 16] = b"0123456789abcdef";
    let mut tmp = [0u8; 16];
    let mut i = 16;
    loop {
        i -= 1;
        tmp[i] = D[(v & 15) as usize];
        v >>= 4;
        if v == 0 {
            break;
        }
    }
    buf.extend_from_slice(&tmp[i..]);
}

/// Shortest round-trip real, always with a `.` or an exponent that carries a `.`
/// (`1.0e-7`, never `1e-7`), so every reader sees a float.
fn real(buf: &mut Vec<u8>, v: f64) {
    let at = buf.len();
    let _ = write!(buf, "{v:?}");
    if let Some(e) = buf[at..].iter().position(|&c| c == b'e')
        && !buf[at..at + e].contains(&b'.')
    {
        buf.splice(at + e..at + e, *b".0");
    }
}

/// A zone name the lexers of Fluent and OpenFOAM accept: `[a-z][a-z0-9_.-]*`.
fn clean_name(raw: &str) -> String {
    let s: String = raw.chars().map(|c| if c.is_ascii_alphanumeric() || matches!(c, '_' | '.' | '-') { c.to_ascii_lowercase() } else { '_' }).collect();
    match s.chars().next() {
        None => "zone".into(),
        Some(c) if c.is_ascii_lowercase() => s,
        Some(_) => format!("z{s}"),
    }
}

/// Hands out names unique over all zones, recording each change.
struct Names {
    used: BTreeSet<String>,
}

impl Names {
    fn take(&mut self, raw: &str, loss: &mut LossReport) -> String {
        let base = clean_name(raw);
        let mut name = base.clone();
        let mut k = 2;
        while !self.used.insert(name.clone()) {
            name = format!("{base}_{k}");
            k += 1;
        }
        if name != raw {
            loss.push(LossItem::new("fluent.name_changed", LossSeverity::Info, 1, format!("'{raw}' -> '{name}'")));
        }
        name
    }
}

/// Element type of a cell from its face census: tet 4 triangles, hex 6 quads, wedge 2 triangles
/// and 3 quads, pyramid 4 triangles and a quad; anything else, including a hex with a split
/// face, is a polyhedron. For a closed cell these census values force the vertex count.
fn cell_type(tri: u32, quad: u32, other: u32) -> u8 {
    match (tri, quad, other) {
        (4, 0, 0) => T_TET,
        (0, 6, 0) => T_HEX,
        (2, 3, 0) => T_WEDGE,
        (4, 1, 0) => T_PYRAMID,
        _ => T_POLY,
    }
}

/// Write `mesh` as an ASCII Fluent `.msh` to `out`.
///
/// Cells are renumbered by cell zone (ascending source index within a zone); nodes keep the
/// source order minus vertices no face uses. Faces are `[interior][boundary zone 0]...`.
/// Every interior face is written with its stored loop, `c0 = right`, `c1 = left`; every boundary
/// face with the loop reversed, `c0 = left`, `c1 = 0`, so the right-hand normal points toward c0
/// everywhere.
///
/// Errors: everything [`crate::foam::build_polymesh`] refuses (not three-dimensional, inactive
/// cells, non-finite coordinates, unknown sets) as [`FluentError::Mesh`], and
/// [`FluentError::UnknownBoundary`].
pub fn write_fluent<W: Write>(mesh: &dyn MeshRead, opts: &FluentOptions, out: &mut W) -> Result<FluentReport, FluentError> {
    let mut fo = FoamOptions::default();
    fo.patches = opts.patches.clone();
    fo.cell_zones = opts.cell_zones.clone();
    fo.default_patch = "default_faces".into();
    fo.split_small_determinant = false;
    fo.split_warped_faces = false;
    let build = build_polymesh(mesh, &fo)?;
    let mut loss = LossReport::new();
    for i in &build.report.loss.items {
        if i.code == "foam.name_sanitized" {
            continue; // the Fluent rename below reports the final change
        }
        loss.push(LossItem::new(i.code.replacen("foam.", "fluent.", 1), i.severity, i.count, i.detail.clone()));
    }
    let poly = build.mesh;
    let (nc, nf, nv) = (poly.n_cells as usize, poly.n_faces(), poly.points.len());
    let n_int = poly.n_internal();
    if nc == 0 || nf == 0 {
        return Err(MeshioError::Unsupported("a Fluent mesh needs at least one cell".into()).into());
    }

    // ---- boundary types: every key must name a boundary zone ------------------------------------
    let zone_key = |p: &crate::foam::PolyPatch| match &p.origin {
        PatchOrigin::Set(s) => s.clone(),
        _ => p.name.clone(),
    };
    for k in opts.boundary_types.keys() {
        if !poly.patches.iter().any(|p| zone_key(p) == *k || p.name == *k) {
            return Err(FluentError::UnknownBoundary(k.clone()));
        }
    }

    // ---- cell zones: first claim wins, the rest is `fluid` --------------------------------------
    let mut names = Names { used: BTreeSet::new() };
    let mut zone_of = vec![NONE; nc];
    let mut cell_zone_names: Vec<String> = Vec::new();
    for z in &poly.cell_zones {
        let idx = cell_zone_names.len() as u32;
        let mut mine = 0usize;
        let mut lost = 0usize;
        for &c in &z.labels {
            if zone_of[c as usize] == NONE {
                zone_of[c as usize] = idx;
                mine += 1;
            } else {
                lost += 1;
            }
        }
        if lost > 0 {
            loss.push(LossItem::new("fluent.cell_zone_overlap", LossSeverity::Degraded, lost as u64, format!("cell set '{}': cells already in an earlier zone stay there", z.name)));
        }
        if mine > 0 {
            cell_zone_names.push(names.take(&z.name, &mut loss));
        }
    }
    let loose = zone_of.iter().filter(|z| **z == NONE).count();
    if loose > 0 {
        let idx = cell_zone_names.len() as u32;
        cell_zone_names.push(names.take("fluid", &mut loss));
        for z in zone_of.iter_mut().filter(|z| **z == NONE) {
            *z = idx;
        }
        loss.push(LossItem::new("fluent.default_cell_zone", LossSeverity::Info, loose as u64, "cells in no zone go to the cell zone of the last id"));
    }
    let nz = cell_zone_names.len();
    // Counting sort of the cells by zone.
    let mut zone_first = vec![0u64; nz + 1];
    for &z in &zone_of {
        zone_first[z as usize + 1] += 1;
    }
    for i in 0..nz {
        zone_first[i + 1] += zone_first[i];
    }
    let mut fill = zone_first.clone();
    let mut new_of_old = vec![0u32; nc];
    let mut cell_of_fluent = vec![0u32; nc];
    for (c, &z) in zone_of.iter().enumerate() {
        let n = fill[z as usize] as u32;
        fill[z as usize] += 1;
        new_of_old[c] = n;
        cell_of_fluent[n as usize] = c as u32;
    }

    // ---- element types from the face census ---------------------------------------------------
    let mut census = vec![[0u32; 3]; nc];
    for f in 0..nf {
        let k = match poly.face(f).len() {
            3 => 0,
            4 => 1,
            _ => 2,
        };
        census[poly.owner[f] as usize][k] += 1;
        if f < n_int {
            census[poly.neighbour[f] as usize][k] += 1;
        }
    }
    let types: Vec<u8> = (0..nc).map(|n| {
        let c = census[cell_of_fluent[n] as usize];
        cell_type(c[0], c[1], c[2])
    }).collect();
    let mut cells_by_type: BTreeMap<u8, u64> = BTreeMap::new();
    for &t in &types {
        *cells_by_type.entry(t).or_default() += 1;
    }

    // ---- ids: node zone 1, cell zones, then face zones -----------------------------------------
    let mut cell_zones: Vec<FluentZone> = Vec::with_capacity(nz);
    for (i, name) in cell_zone_names.into_iter().enumerate() {
        cell_zones.push(FluentZone { id: 2 + i as u32, name, kind: "fluid", first: zone_first[i] + 1, count: zone_first[i + 1] - zone_first[i] });
    }
    let mut face_zones: Vec<FluentZone> = Vec::with_capacity(poly.patches.len() + 1);
    let mut next_id = 2 + nz as u32;
    let mut bcs: Vec<(u32, &'static str)> = Vec::new(); // bc code and word per face zone
    if n_int > 0 {
        face_zones.push(FluentZone { id: next_id, name: names.take("interior", &mut loss), kind: "interior", first: 1, count: n_int as u64 });
        bcs.push((2, "interior"));
        next_id += 1;
    }
    let mut defaulted: Vec<String> = Vec::new();
    for p in &poly.patches {
        let bc = match opts.boundary_types.get(&zone_key(p)).or_else(|| opts.boundary_types.get(&p.name)) {
            Some(b) => *b,
            None => {
                defaulted.push(p.name.clone());
                FluentBc::Wall
            }
        };
        face_zones.push(FluentZone { id: next_id, name: names.take(&p.name, &mut loss), kind: bc.word(), first: u64::from(p.start_face) + 1, count: u64::from(p.n_faces) });
        bcs.push((bc.code(), bc.word()));
        next_id += 1;
    }
    if !defaulted.is_empty() {
        loss.push(LossItem::new("fluent.boundary_type_default", LossSeverity::Info, defaulted.len() as u64, format!("written as wall: {}", defaulted.join(","))));
    }

    // ---- text ------------------------------------------------------------------------------------
    let mut buf: Vec<u8> = Vec::with_capacity(64 * nf + 48 * nv);
    let _ = write!(buf, "(0 \"smart-mesh Fluent export\")\n(1 \"smart-mesh\")\n(0 \"Dimension:\")\n(2 3)\n");
    let _ = write!(buf, "(0 \"Declarations:\")\n(10 (0 1 {nv:x} 0 3))\n(12 (0 1 {nc:x} 0))\n(13 (0 1 {nf:x} 0))\n");
    let _ = write!(buf, "(0 \"Nodes: zone 1\")\n(10 (1 1 {nv:x} 1 3)(\n");
    let node_text: Vec<Vec<u8>> = poly
        .points
        .par_chunks(CHUNK)
        .map(|chunk| {
            let mut b = Vec::with_capacity(chunk.len() * 40);
            for p in chunk {
                for (i, c) in p.iter().enumerate() {
                    if i > 0 {
                        b.push(b' ');
                    }
                    real(&mut b, *c);
                }
                b.push(b'\n');
            }
            b
        })
        .collect();
    for b in &node_text {
        buf.extend_from_slice(b);
    }
    buf.extend_from_slice(b"))\n");

    // Cells: one section per zone; a zone of one type has no body.
    for z in &cell_zones {
        // 0-based half-open range of the zone.
        let (lo, hi) = (z.first as usize - 1, (z.first + z.count) as usize - 1);
        let t0 = types[lo];
        let _ = writeln!(buf, "(0 \"Cells: zone {} {}\")", z.id, z.name);
        if types[lo..hi].iter().all(|&t| t == t0) {
            let _ = writeln!(buf, "(12 ({:x} {:x} {:x} 1 {:x}))", z.id, z.first, z.first + z.count - 1, t0);
        } else {
            let _ = writeln!(buf, "(12 ({:x} {:x} {:x} 1 {:x})(", z.id, z.first, z.first + z.count - 1, MIXED);
            for line in types[lo..hi].chunks(40) {
                for (i, t) in line.iter().enumerate() {
                    if i > 0 {
                        buf.push(b' ');
                    }
                    hex(&mut buf, u64::from(*t));
                }
                buf.push(b'\n');
            }
            buf.extend_from_slice(b"))\n");
        }
    }

    // Faces: one section per zone, one face per line.
    for (z, &(bc, _)) in face_zones.iter().zip(&bcs) {
        // 0-based half-open range of the zone.
        let (lo, hi) = (z.first as usize - 1, (z.first + z.count) as usize - 1);
        let interior = lo < n_int;
        let (mut tri, mut quad) = (true, true);
        for f in lo..hi {
            let n = poly.face(f).len();
            tri &= n == 3;
            quad &= n == 4;
        }
        let ftype = if tri {
            3
        } else if quad {
            4
        } else {
            MIXED
        };
        let _ = writeln!(buf, "(0 \"Faces: zone {} {}\")", z.id, z.name);
        let _ = writeln!(buf, "(13 ({:x} {:x} {:x} {:x} {:x})(", z.id, z.first, z.first + z.count - 1, bc, ftype);
        let starts: Vec<usize> = (lo..hi).step_by(CHUNK).collect();
        let text: Vec<Vec<u8>> = starts
            .par_iter()
            .map(|&s| {
                let e = (s + CHUNK).min(hi);
                let mut b = Vec::with_capacity((e - s) * 40);
                for f in s..e {
                    let l = poly.face(f);
                    if ftype == MIXED {
                        hex(&mut b, l.len() as u64);
                        b.push(b' ');
                    }
                    let node = |b: &mut Vec<u8>, v: u32| {
                        hex(b, u64::from(v) + 1);
                        b.push(b' ');
                    };
                    if interior {
                        l.iter().for_each(|&v| node(&mut b, v));
                        hex(&mut b, u64::from(new_of_old[poly.neighbour[f] as usize]) + 1);
                        b.push(b' ');
                        hex(&mut b, u64::from(new_of_old[poly.owner[f] as usize]) + 1);
                    } else {
                        l.iter().rev().for_each(|&v| node(&mut b, v));
                        hex(&mut b, u64::from(new_of_old[poly.owner[f] as usize]) + 1);
                        b.extend_from_slice(b" 0");
                    }
                    b.push(b'\n');
                }
                b
            })
            .collect();
        for b in &text {
            buf.extend_from_slice(b);
        }
        buf.extend_from_slice(b"))\n");
    }

    buf.extend_from_slice(b"(0 \"Zones\")\n");
    for z in cell_zones.iter().chain(&face_zones) {
        let _ = writeln!(buf, "(39 ({} {} {})())", z.id, z.kind, z.name);
    }
    out.write_all(&buf)?;
    Ok(FluentReport {
        nodes: nv as u64,
        cells: nc as u64,
        faces: nf as u64,
        interior_faces: n_int as u64,
        cells_by_type,
        cell_zones,
        face_zones,
        cell_of_fluent,
        loss,
        bytes: buf.len() as u64,
        hash: fnv(&buf),
    })
}
