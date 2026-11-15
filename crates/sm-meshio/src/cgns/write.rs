//! The CGNS/HDF5 writer.

use super::{CgnsBc, CgnsError, CgnsLayout, CgnsOptions, CgnsReport, CgnsSection};
use crate::error::MeshioError;
use crate::foam::{FoamError, FoamOptions, PatchOrigin, PolyMesh, build_polymesh};
use crate::h5::{Attr, Dataset, Dtype, Group, write_file};
use crate::report::{LossItem, LossReport, LossSeverity};
use rayon::prelude::*;
use sm_mesh_api::MeshRead;
use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;

const NONE: u32 = u32::MAX;
/// Maximum CGNS node name length.
const NAME_LEN: usize = 32;
/// `ElementType_t` codes (CGNS 4.x).
const E_TRI3: i32 = 5;
const E_QUAD4: i32 = 7;
const E_TETRA4: i32 = 10;
const E_PYRA5: i32 = 12;
const E_PENTA6: i32 = 14;
const E_HEXA8: i32 = 17;
const E_NGON: i32 = 22;
const E_NFACE: i32 = 23;

fn fnv(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bytes {
        h = (h ^ u64::from(*b)).wrapping_mul(0x0100_0000_01b3);
    }
    h
}

fn type_word(t: i32) -> &'static str {
    match t {
        E_TRI3 => "TRI_3",
        E_QUAD4 => "QUAD_4",
        E_TETRA4 => "TETRA_4",
        E_PYRA5 => "PYRA_5",
        E_PENTA6 => "PENTA_6",
        E_HEXA8 => "HEXA_8",
        E_NGON => "NGON_n",
        _ => "NFACE_n",
    }
}

/// A CGNS node name: `[A-Za-z0-9_.+-]`, at most 32 bytes, not `.` or `..`.
fn clean_name(raw: &str) -> String {
    let mut s: String = raw.chars().map(|c| if c.is_ascii_alphanumeric() || matches!(c, '_' | '.' | '+' | '-') { c } else { '_' }).collect();
    s.truncate(NAME_LEN);
    if s.is_empty() || s.chars().all(|c| c == '.') {
        s = "node".into();
    }
    s
}

/// Hands out names unique over the whole file, recording each change.
struct Names {
    used: BTreeSet<String>,
}

impl Names {
    fn take(&mut self, raw: &str, loss: &mut LossReport) -> String {
        let base = clean_name(raw);
        let mut name = base.clone();
        let mut k = 2;
        while !self.used.insert(name.clone()) {
            let suffix = format!("_{k}");
            let mut b = base.clone();
            b.truncate(NAME_LEN - suffix.len());
            name = format!("{b}{suffix}");
            k += 1;
        }
        if name != raw {
            loss.push(LossItem::new("cgns.name_changed", LossSeverity::Info, 1, format!("'{raw}' -> '{name}'")));
        }
        name
    }
}

// ---- node tree helpers ------------------------------------------------------------------------

fn name_attrs(g: &mut Group, name: &str, label: &str, ty: &str) {
    g.put_attr(Attr::string("name", 33, name));
    g.put_attr(Attr::string("label", 33, label));
    g.put_attr(Attr::string("type", 3, ty));
    g.put_attr(Attr::i32s("flags", &[1]));
}

/// A node with data: `ty` is the CGNS type tag (`I4`, `I8`, `R4`, `R8`, `C1`).
fn leaf<'a>(parent: &'a mut Group, name: &str, label: &str, ty: &str, data: Dataset) -> &'a mut Group {
    let g = parent.tracked_group(name);
    name_attrs(g, name, label, ty);
    g.put_dataset(" data", data);
    g
}

/// A node without data (`MT`).
fn empty<'a>(parent: &'a mut Group, name: &str, label: &str) -> &'a mut Group {
    let g = parent.tracked_group(name);
    name_attrs(g, name, label, "MT");
    g
}

fn text(parent: &mut Group, name: &str, label: &str, s: &str) {
    leaf(parent, name, label, "C1", Dataset::raw(Dtype::U8, vec![s.len() as u64], s.as_bytes().to_vec()));
}

/// Integer data of the file's index width: `I4`, or `I8` when some index does not fit.
#[derive(Clone, Copy)]
struct Width(bool);

impl Width {
    fn tag(self) -> &'static str {
        if self.0 { "I8" } else { "I4" }
    }
    fn data(self, dims: Vec<u64>, v: &[u64]) -> Dataset {
        if self.0 {
            Dataset::raw(Dtype::I64, dims, v.iter().flat_map(|x| x.to_le_bytes()).collect())
        } else {
            Dataset::raw(Dtype::I32, dims, v.iter().flat_map(|x| (*x as u32).to_le_bytes()).collect())
        }
    }
    /// Signed values.
    fn signed(self, dims: Vec<u64>, v: &[i64]) -> Dataset {
        if self.0 { Dataset::i64s(dims, v) } else { Dataset::i32s(dims, &v.iter().map(|x| *x as i32).collect::<Vec<_>>()) }
    }
}

fn index_leaf(parent: &mut Group, w: Width, name: &str, label: &str, dims: Vec<u64>, v: &[u64]) {
    leaf(parent, name, label, w.tag(), w.data(dims, v));
}

/// One `Elements_t` node.
struct Section<'a> {
    name: &'a str,
    etype: i32,
    first: u64,
    last: u64,
    conn: Dataset,
    offsets: Option<Dataset>,
}

fn put_section(zone: &mut Group, w: Width, s: Section) {
    let g = zone.tracked_group(s.name);
    name_attrs(g, s.name, "Elements_t", "I4");
    g.put_dataset(" data", Dataset::i32s(vec![2], &[s.etype, 0]));
    index_leaf(g, w, "ElementRange", "IndexRange_t", vec![2], &[s.first, s.last]);
    if let Some(o) = s.offsets {
        leaf(g, "ElementStartOffset", "DataArray_t", w.tag(), o);
    }
    leaf(g, "ElementConnectivity", "DataArray_t", w.tag(), s.conn);
}

// ---- cell vertices for the standard layout ---------------------------------------------------

/// A face loop of at most four vertices.
#[derive(Clone, Copy)]
struct Loop {
    v: [u32; 4],
    n: usize,
}

impl Loop {
    fn s(&self) -> &[u32] {
        &self.v[..self.n]
    }
    fn has(&self, x: u32) -> bool {
        self.s().contains(&x)
    }
}

/// The cell's faces as loops outward from the cell. `None` for a cell with more than six faces
/// or a face over four vertices (a polyhedron).
fn cell_loops(poly: &PolyMesh, faces: &[(u32, bool)]) -> Option<([Loop; 6], usize)> {
    if faces.len() > 6 {
        return None;
    }
    let mut out = [Loop { v: [0; 4], n: 0 }; 6];
    for (i, &(f, owner)) in faces.iter().enumerate() {
        let l = poly.face(f as usize);
        if l.len() > 4 {
            return None;
        }
        let mut lp = Loop { v: [0; 4], n: l.len() };
        for (j, &x) in l.iter().enumerate() {
            lp.v[j] = if owner { x } else { l[l.len() - 1 - j] };
        }
        out[i] = lp;
    }
    Some((out, faces.len()))
}

/// The vertex of `top` joined to `b` by an edge of a face other than `skip_a` and `skip_b`.
fn above(loops: &[Loop], skip: [usize; 2], b: u32, top: &Loop) -> Option<u32> {
    for (i, l) in loops.iter().enumerate() {
        if i == skip[0] || i == skip[1] {
            continue;
        }
        let s = l.s();
        for k in 0..s.len() {
            let (p, q) = (s[k], s[(k + 1) % s.len()]);
            if p == b && top.has(q) {
                return Some(q);
            }
            if q == b && top.has(p) {
                return Some(p);
            }
        }
    }
    None
}

/// Vertices of a standard cell in CGNS order (right-hand rule on the first face points into the
/// cell), plus the element type; `None` if the cell is not a tet, pyramid, wedge or hex.
fn standard_cell(poly: &PolyMesh, faces: &[(u32, bool)]) -> Option<(i32, [u32; 8])> {
    let (loops, n) = cell_loops(poly, faces)?;
    let loops = &loops[..n];
    let tri: Vec<usize> = (0..n).filter(|&i| loops[i].n == 3).collect();
    let quad: Vec<usize> = (0..n).filter(|&i| loops[i].n == 4).collect();
    let mut out = [0u32; 8];
    match (tri.len(), quad.len()) {
        (4, 0) => {
            let a = loops[tri[0]].v;
            let apex = loops[tri[1]].s().iter().copied().find(|x| !loops[tri[0]].has(*x))?;
            out[..4].copy_from_slice(&[a[0], a[2], a[1], apex]);
            Some((E_TETRA4, out))
        }
        (4, 1) => {
            let q = loops[quad[0]].v;
            let apex = loops[tri[0]].s().iter().copied().find(|x| !loops[quad[0]].has(*x))?;
            out[..5].copy_from_slice(&[q[0], q[3], q[2], q[1], apex]);
            Some((E_PYRA5, out))
        }
        (2, 3) => {
            let (t0, t1) = (loops[tri[0]], loops[tri[1]]);
            let base = [t0.v[0], t0.v[2], t0.v[1]];
            out[..3].copy_from_slice(&base);
            for (i, b) in base.iter().enumerate() {
                out[3 + i] = above(loops, [tri[0], tri[1]], *b, &t1)?;
            }
            Some((E_PENTA6, out))
        }
        (0, 6) => {
            let a = loops[0];
            let opp = (1..6).find(|&i| !loops[i].s().iter().any(|x| a.has(*x)))?;
            let base = [a.v[0], a.v[3], a.v[2], a.v[1]];
            out[..4].copy_from_slice(&base);
            for (i, b) in base.iter().enumerate() {
                out[4 + i] = above(loops, [0, opp], *b, &loops[opp])?;
            }
            Some((E_HEXA8, out))
        }
        _ => None,
    }
}

/// Write `mesh` as a CGNS/HDF5 file to `out`.
///
/// Under standard sections cells are renumbered by block then element type (source order within
/// each), the boundary faces follow the cells, and element ids are one index space from 1. Under
/// `Ngon` the faces come first (interior faces, then one run per BC, in [`build_polymesh`]
/// order, each loop outward from its owner) and the cells follow in source order, each as
/// signed face ids: `+f` when the face normal (right-hand rule on its loop) points out of the
/// cell, `-f` when it points in.
///
/// Errors: everything [`build_polymesh`] refuses (not three-dimensional, inactive cells,
/// non-finite coordinates, unknown sets) as [`CgnsError::Mesh`], [`CgnsError::UnknownBoundary`],
/// and I/O errors.
pub fn write_cgns<W: Write>(mesh: &dyn MeshRead, opts: &CgnsOptions, out: &mut W) -> Result<CgnsReport, CgnsError> {
    let fo = FoamOptions {
        patches: opts.patches.clone(),
        cell_zones: opts.cell_zones.clone(),
        default_patch: "default_faces".into(),
        split_small_determinant: false,
        split_warped_faces: false,
        ..FoamOptions::default()
    };
    let build = build_polymesh(mesh, &fo).map_err(|e| match e {
        MeshioError::Foam(FoamError::NotThreeDimensional) => MeshioError::Unsupported("a CGNS export needs a dimension-3 mesh".into()),
        other => other,
    })?;
    let mut loss = LossReport::new();
    for i in &build.report.loss.items {
        loss.push(LossItem::new(i.code.replacen("foam.", "cgns.", 1), i.severity, i.count, i.detail.clone()));
    }
    let poly = build.mesh;
    let (nc, nf, nv) = (poly.n_cells as usize, poly.n_faces(), poly.points.len());
    let n_int = poly.n_internal();
    if nc == 0 || nf == 0 {
        return Err(MeshioError::Unsupported("a CGNS mesh needs at least one cell".into()).into());
    }
    let zone_key = |p: &crate::foam::PolyPatch| match &p.origin {
        PatchOrigin::Set(s) => s.clone(),
        _ => p.name.clone(),
    };
    for k in opts.boundary_types.keys() {
        if !poly.patches.iter().any(|p| zone_key(p) == *k || p.name == *k) {
            return Err(CgnsError::UnknownBoundary(k.clone()));
        }
    }

    // ---- cell -> faces CSR (ascending face id; `true` = the cell owns the face) ----------------
    let mut cf_off = vec![0usize; nc + 1];
    for f in 0..nf {
        cf_off[poly.owner[f] as usize + 1] += 1;
        if f < n_int {
            cf_off[poly.neighbour[f] as usize + 1] += 1;
        }
    }
    for c in 0..nc {
        cf_off[c + 1] += cf_off[c];
    }
    let mut fill = cf_off.clone();
    let mut cell_faces = vec![(0u32, false); cf_off[nc]];
    for f in 0..nf {
        let o = poly.owner[f] as usize;
        cell_faces[fill[o]] = (f as u32, true);
        fill[o] += 1;
        if f < n_int {
            let n = poly.neighbour[f] as usize;
            cell_faces[fill[n]] = (f as u32, false);
            fill[n] += 1;
        }
    }
    let faces_of = |c: usize| &cell_faces[cf_off[c]..cf_off[c + 1]];

    // ---- standard cell vertices (None for any polyhedron) --------------------------------------
    let std_cells: Vec<Option<(i32, [u32; 8])>> = (0..nc).into_par_iter().map(|c| standard_cell(&poly, faces_of(c))).collect();
    let all_standard = std_cells.iter().all(|c| c.is_some());
    let mut cells_by_type: BTreeMap<&'static str, u64> = BTreeMap::new();
    for c in &std_cells {
        *cells_by_type.entry(c.map_or("NFACE_n", |(t, _)| type_word(t))).or_default() += 1;
    }
    let layout = match opts.layout {
        CgnsLayout::Auto if all_standard => CgnsLayout::Auto,
        _ => CgnsLayout::Ngon,
    };
    if opts.layout == CgnsLayout::Auto && layout == CgnsLayout::Ngon {
        let n = std_cells.iter().filter(|c| c.is_none()).count() as u64;
        loss.push(LossItem::new("cgns.layout_ngon", LossSeverity::Info, n, "polyhedral cells present: the zone uses NGON_n and NFACE_n"));
    }

    // ---- names and cell zones ------------------------------------------------------------------
    let mut names = Names { used: BTreeSet::new() };
    let zone_name = names.take("Zone", &mut loss);
    let mut zone_of = vec![NONE; nc];
    let mut blocks: Vec<String> = Vec::new();
    for z in &poly.cell_zones {
        let idx = blocks.len() as u32;
        let (mut mine, mut lost) = (0usize, 0usize);
        for &c in &z.labels {
            if zone_of[c as usize] == NONE {
                zone_of[c as usize] = idx;
                mine += 1;
            } else {
                lost += 1;
            }
        }
        if lost > 0 {
            loss.push(LossItem::new("cgns.cell_zone_overlap", LossSeverity::Degraded, lost as u64, format!("cell set '{}': cells already in an earlier block stay there", z.name)));
        }
        if mine > 0 {
            blocks.push(names.take(&z.name, &mut loss));
        }
    }
    let nz = blocks.len();
    let mut block_cells: Vec<Vec<u32>> = vec![Vec::new(); nz];
    for (c, &z) in zone_of.iter().enumerate() {
        if z != NONE {
            block_cells[z as usize].push(c as u32);
        }
    }

    // ---- boundary conditions -------------------------------------------------------------------
    let mut bc_names: Vec<String> = Vec::new();
    let mut bc_types: Vec<CgnsBc> = Vec::new();
    let mut defaulted: Vec<String> = Vec::new();
    for p in &poly.patches {
        if p.n_faces == 0 {
            continue;
        }
        let bc = match opts.boundary_types.get(&zone_key(p)).or_else(|| opts.boundary_types.get(&p.name)) {
            Some(b) => *b,
            None => {
                defaulted.push(p.name.clone());
                CgnsBc::UserDefined
            }
        };
        bc_names.push(names.take(&p.name, &mut loss));
        bc_types.push(bc);
    }
    if !defaulted.is_empty() {
        loss.push(LossItem::new("cgns.boundary_type_default", LossSeverity::Info, defaulted.len() as u64, format!("written as UserDefined: {}", defaulted.join(","))));
    }
    let live_patches: Vec<&crate::foam::PolyPatch> = poly.patches.iter().filter(|p| p.n_faces > 0).collect();

    // ---- element sections ------------------------------------------------------------------------
    let mut root = Group::tracked();
    root.put_attr(Attr::string("label", 33, "Root Node of HDF5 File"));
    root.put_attr(Attr::string("name", 33, "HDF5 MotherNode"));
    root.put_attr(Attr::string("type", 3, "MT"));
    let mut fmt = b"IEEE_LITTLE_32\0".to_vec();
    root.put_dataset(" format", Dataset::raw(Dtype::U8, vec![fmt.len() as u64], std::mem::take(&mut fmt)));
    let mut ver = b"HDF5 Version 1.10.0".to_vec();
    ver.resize(33, 0);
    root.put_dataset(" hdf5version", Dataset::raw(Dtype::U8, vec![33], ver));
    leaf(&mut root, "CGNSLibraryVersion", "CGNSLibraryVersion_t", "R4", Dataset::f32s(vec![1], &[4.0]));

    // Element ids are one space from 1; sizes decide the index width.
    let conn_len_ngon: usize = poly.face_points.len() + cell_faces.len();
    let wide = (nf + nc + nv) as u64 > i32::MAX as u64 || conn_len_ngon as u64 > i32::MAX as u64 || poly.face_points.len() as u64 > i32::MAX as u64;
    let w = Width(wide);

    let mut sections: Vec<CgnsSection> = Vec::new();
    let mut cell_of_cgns: Vec<u32> = Vec::with_capacity(nc);
    let mut section_nodes: Vec<(String, i32, u64, u64, Dataset, Option<Dataset>)> = Vec::new();
    // Element ids of each BC's faces, and each block's cells (element ids).
    let mut bc_ids: Vec<Vec<u64>> = Vec::new();
    let mut block_ids: Vec<Vec<u64>> = vec![Vec::new(); nz];
    let src = |c: u32| poly.cell_map.get(c as usize).copied().unwrap_or(c);

    if layout == CgnsLayout::Ngon {
        // NGON: every face; NFACE: signed face ids per cell.
        let offs: Vec<u64> = poly.face_offsets.clone();
        let conn: Vec<u64> = poly.face_points.iter().map(|&v| u64::from(v) + 1).collect();
        section_nodes.push(("NGON".into(), E_NGON, 1, nf as u64, w.data(vec![conn.len() as u64], &conn), Some(w.data(vec![offs.len() as u64], &offs))));
        let mut noffs: Vec<u64> = Vec::with_capacity(nc + 1);
        noffs.push(0);
        let mut nconn: Vec<i64> = Vec::with_capacity(cell_faces.len());
        for c in 0..nc {
            for &(f, owner) in faces_of(c) {
                let id = i64::from(f) + 1;
                nconn.push(if owner { id } else { -id });
            }
            noffs.push(nconn.len() as u64);
            cell_of_cgns.push(src(c as u32));
        }
        section_nodes.push(("NFACE".into(), E_NFACE, nf as u64 + 1, (nf + nc) as u64, w.signed(vec![nconn.len() as u64], &nconn), Some(w.data(vec![noffs.len() as u64], &noffs))));
        for p in &live_patches {
            bc_ids.push((u64::from(p.start_face)..u64::from(p.start_face) + u64::from(p.n_faces)).map(|f| f + 1).collect());
        }
        for (z, cells) in block_cells.iter().enumerate() {
            block_ids[z] = cells.iter().map(|&c| nf as u64 + u64::from(c) + 1).collect();
        }
    } else {
        // Standard sections: cells ordered by (block, type), then boundary faces by BC and type.
        let mut order: Vec<u32> = (0..nc as u32).collect();
        order.sort_by_key(|&c| (zone_of[c as usize], std_cells[c as usize].map_or(0, |(t, _)| t)));
        let mut new_of_old = vec![0u32; nc];
        for (n, &c) in order.iter().enumerate() {
            new_of_old[c as usize] = n as u32;
            cell_of_cgns.push(src(c));
        }
        let mut i = 0;
        while i < nc {
            let key = |c: u32| (zone_of[c as usize], std_cells[c as usize].map_or(0, |(t, _)| t));
            let k = key(order[i]);
            let mut j = i;
            while j < nc && key(order[j]) == k {
                j += 1;
            }
            let t = k.1;
            let width = match t {
                E_TETRA4 => 4,
                E_PYRA5 => 5,
                E_PENTA6 => 6,
                _ => 8,
            };
            let conn: Vec<u64> = order[i..j].iter().flat_map(|&c| std_cells[c as usize].map(|(_, v)| v).unwrap_or([0; 8])[..width].iter().map(|&x| u64::from(x) + 1).collect::<Vec<_>>()).collect();
            let word = type_word(t);
            let sec_name = if k.0 == NONE {
                names.take(word, &mut loss)
            } else {
                names.take(&format!("{}_{}", blocks[k.0 as usize], word), &mut loss)
            };
            section_nodes.push((sec_name, t, i as u64 + 1, j as u64, w.data(vec![conn.len() as u64], &conn), None));
            i = j;
        }
        for (z, cells) in block_cells.iter().enumerate() {
            block_ids[z] = cells.iter().map(|&c| u64::from(new_of_old[c as usize]) + 1).collect();
            block_ids[z].sort_unstable();
        }
        // Boundary faces: per BC a TRI_3 run then a QUAD_4 run, loops outward from the owner.
        let mut next = nc as u64 + 1;
        for (pi, p) in live_patches.iter().enumerate() {
            let range = p.start_face as usize..(p.start_face + p.n_faces) as usize;
            let mut ids = Vec::with_capacity(range.len());
            for (len, t) in [(3usize, E_TRI3), (4, E_QUAD4)] {
                let fs: Vec<usize> = range.clone().filter(|&f| poly.face(f).len() == len).collect();
                if fs.is_empty() {
                    continue;
                }
                let conn: Vec<u64> = fs.iter().flat_map(|&f| poly.face(f).iter().map(|&x| u64::from(x) + 1)).collect();
                let sec_name = names.take(&format!("{}_{}", bc_names[pi], type_word(t)), &mut loss);
                let (first, last) = (next, next + fs.len() as u64 - 1);
                ids.extend(first..=last);
                next = last + 1;
                section_nodes.push((sec_name, t, first, last, w.data(vec![conn.len() as u64], &conn), None));
            }
            bc_ids.push(ids);
        }
    }

    // ---- tree --------------------------------------------------------------------------------------
    let mut tree = root;
    fix_base(&mut tree);
    let base = tree.tracked_group("Base");
    let zone = {
        let z = base.tracked_group(&zone_name);
        name_attrs(z, &zone_name, "Zone_t", w.tag());
        z.put_dataset(" data", w.data(vec![3, 1], &[nv as u64, nc as u64, 0]));
        z
    };
    text(zone, "ZoneType", "ZoneType_t", "Unstructured");
    {
        let gc = empty(zone, "GridCoordinates", "GridCoordinates_t");
        for (k, nm) in ["CoordinateX", "CoordinateY", "CoordinateZ"].iter().enumerate() {
            let col: Vec<f64> = poly.points.iter().map(|p| p[k]).collect();
            leaf(gc, nm, "DataArray_t", "R8", Dataset::f64s(vec![nv as u64], &col));
        }
    }
    for (name, etype, first, last, conn, offsets) in section_nodes {
        sections.push(CgnsSection { name: name.clone(), kind: type_word(etype), first, last });
        put_section(zone, w, Section { name: &name, etype, first, last, conn, offsets });
    }
    {
        let zbc = empty(zone, "ZoneBC", "ZoneBC_t");
        for (pi, name) in bc_names.iter().enumerate() {
            let g = zbc.tracked_group(name);
            name_attrs(g, name, "BC_t", "C1");
            g.put_dataset(" data", Dataset::raw(Dtype::U8, vec![15], b"FamilySpecified".to_vec()));
            index_leaf(g, w, "PointList", "IndexArray_t", vec![bc_ids[pi].len() as u64, 1], &bc_ids[pi]);
            text(g, "GridLocation", "GridLocation_t", "FaceCenter");
            text(g, "FamilyName", "FamilyName_t", name);
        }
    }
    for (z, name) in blocks.iter().enumerate() {
        let g = zone.tracked_group(name);
        name_attrs(g, name, "ZoneSubRegion_t", "I4");
        g.put_dataset(" data", Dataset::i32s(vec![1], &[3]));
        text(g, "GridLocation", "GridLocation_t", "CellCenter");
        index_leaf(g, w, "PointList", "IndexArray_t", vec![block_ids[z].len() as u64, 1], &block_ids[z]);
        text(g, "FamilyName", "FamilyName_t", name);
    }
    for (pi, name) in bc_names.iter().enumerate() {
        let fam = empty(base, name, "Family_t");
        text(fam, "FamilyBC", "FamilyBC_t", bc_types[pi].word());
    }
    for name in &blocks {
        empty(base, name, "Family_t");
    }

    let bytes = write_file(tree)?;
    out.write_all(&bytes)?;
    Ok(CgnsReport {
        layout,
        nodes: nv as u64,
        cells: nc as u64,
        faces: sections.iter().filter(|s| matches!(s.kind, "NGON_n" | "TRI_3" | "QUAD_4")).map(|s| s.last - s.first + 1).sum(),
        interior_faces: n_int as u64,
        cells_by_type,
        sections,
        bcs: bc_names,
        blocks,
        cell_of_cgns,
        loss,
        bytes: bytes.len() as u64,
        hash: fnv(&bytes),
    })
}

/// `Base` carries `I4` data `[cell_dim, phys_dim]`.
fn fix_base(root: &mut Group) {
    let b = root.tracked_group("Base");
    b.attrs.clear();
    name_attrs(b, "Base", "CGNSBase_t", "I4");
    b.put_dataset(" data", Dataset::i32s(vec![2], &[3, 3]));
}
