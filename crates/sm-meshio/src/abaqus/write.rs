//! The Abaqus `.inp` writer.

use super::{AbaqusError, AbaqusOptions, AbaqusReport, PolyPolicy, PyramidPolicy};
use crate::report::{LossItem, LossReport, LossSeverity};
use rayon::prelude::*;
use sm_mesh_api::{CellIx, CellShape, FaceIx, MeshDim, MeshRead, SetMembers, Side};
use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;

/// Entities formatted per parallel task.
const CHUNK: usize = 8192;
/// Values per data line of a set (Abaqus allows 16).
const PER_LINE: usize = 16;
/// Abaqus names are at most 80 characters.
const NAME_MAX: usize = 80;

/// An output element type.
struct Ty {
    name: &'static str,
}

/// Output element types, in file order.
const TYPES: [Ty; 4] = [Ty { name: "C3D4" }, Ty { name: "C3D5" }, Ty { name: "C3D6" }, Ty { name: "C3D8" }];

/// Face tables (`Sn` and the node positions of the face), from the Abaqus element library.
type Faces = &'static [(u8, &'static [usize])];
const TET_FACES: Faces = &[(1, &[0, 1, 2]), (2, &[0, 3, 1]), (3, &[1, 3, 2]), (4, &[2, 3, 0])];
const HEX_FACES: Faces = &[(1, &[0, 1, 2, 3]), (2, &[4, 5, 6, 7]), (3, &[0, 1, 5, 4]), (4, &[1, 2, 6, 5]), (5, &[2, 3, 7, 6]), (6, &[3, 0, 4, 7])];
const WEDGE_FACES: Faces = &[(1, &[0, 1, 2]), (2, &[3, 4, 5]), (3, &[0, 1, 4, 3]), (4, &[1, 2, 5, 4]), (5, &[2, 0, 3, 5])];
const PYR_FACES: Faces = &[(1, &[0, 1, 2, 3]), (2, &[0, 1, 4]), (3, &[1, 2, 4]), (4, &[2, 3, 4]), (5, &[3, 0, 4])];
/// A pyramid as a collapsed hexahedron: the side faces are the hex faces S3 to S6 and the top
/// face S2 has zero area.
const PYR_HEX_FACES: Faces = &[(1, &[0, 1, 2, 3]), (3, &[0, 1, 4]), (4, &[1, 2, 4]), (5, &[2, 3, 4]), (6, &[3, 0, 4])];

/// What a record is: the element type index and the face table.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Tet,
    Pyr,
    PyrHex,
    Wedge,
    Hex,
}

impl Kind {
    fn ty(self) -> usize {
        match self {
            Kind::Tet => 0,
            Kind::Pyr => 1,
            Kind::Wedge => 2,
            Kind::Hex | Kind::PyrHex => 3,
        }
    }
    fn faces(self) -> Faces {
        match self {
            Kind::Tet => TET_FACES,
            Kind::Pyr => PYR_FACES,
            Kind::PyrHex => PYR_HEX_FACES,
            Kind::Wedge => WEDGE_FACES,
            Kind::Hex => HEX_FACES,
        }
    }
}

/// Elements in write order: id, kind, nodes (vertex index, 0-based).
#[derive(Default)]
struct Elems {
    id: Vec<u32>,
    kind: Vec<Kind>,
    off: Vec<u32>,
    conn: Vec<u32>,
}

impl Elems {
    fn push(&mut self, id: u32, kind: Kind, nodes: &[u32]) {
        self.id.push(id);
        self.kind.push(kind);
        self.off.push(self.conn.len() as u32);
        self.conn.extend_from_slice(nodes);
    }
    fn nodes(&self, i: usize) -> &[u32] {
        let end = self.off.get(i + 1).map_or(self.conn.len(), |&e| e as usize);
        &self.conn[self.off[i] as usize..end]
    }
}

fn fnv(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bytes {
        h = (h ^ u64::from(*b)).wrapping_mul(0x0100_0000_01b3);
    }
    h
}

/// Shortest round-trip real that always carries a `.` (`1.0e-7`, never `1e-7`).
fn real(buf: &mut Vec<u8>, v: f64) {
    let at = buf.len();
    let _ = write!(buf, "{v:?}");
    if let Some(e) = buf[at..].iter().position(|&c| c == b'e')
        && !buf[at..at + e].contains(&b'.')
    {
        buf.splice(at + e..at + e, *b".0");
    }
}

/// An Abaqus label: `[A-Za-z][A-Za-z0-9_]*`, at most 80 characters (a leading underscore is
/// reserved for Abaqus' own sets, so a name that does not start with a letter gets `s_`).
fn clean_name(raw: &str) -> String {
    let mut s: String = raw.chars().map(|c| if c.is_ascii_alphanumeric() || c == '_' { c } else { '_' }).collect();
    if !s.chars().next().is_some_and(|c| c.is_ascii_alphabetic()) {
        s.insert_str(0, "s_");
    }
    s.truncate(NAME_MAX);
    s
}

/// Hands out names unique over all sets and surfaces. Abaqus labels are case-insensitive, so
/// uniqueness is checked on the upper-cased name.
struct Names {
    used: BTreeSet<String>,
}

impl Names {
    fn take(&mut self, raw: &str, loss: &mut LossReport) -> String {
        let base = clean_name(raw);
        let mut name = base.clone();
        let mut k = 2;
        while !self.used.insert(name.to_ascii_uppercase()) {
            let suffix = format!("_{k}");
            let mut b = base.clone();
            b.truncate(NAME_MAX - suffix.len());
            name = b + &suffix;
            k += 1;
        }
        if name != raw {
            loss.push(LossItem::new("abaqus.name_changed", LossSeverity::Info, 1, format!("'{raw}' -> '{name}'")));
        }
        name
    }
}

fn unsupported<T>(m: impl Into<String>) -> Result<T, AbaqusError> {
    Err(AbaqusError::Unsupported(m.into()))
}

fn centroid(ids: &[u32], pts: &[[f64; 3]]) -> [f64; 3] {
    let mut c = [0.0; 3];
    for &i in ids {
        for (k, ck) in c.iter_mut().enumerate() {
            *ck += pts[i as usize][k] / ids.len() as f64;
        }
    }
    c
}

/// The `Sn` of the face of an element whose nodes are `vs` (as a sorted set), if any.
fn local_face(kind: Kind, nodes: &[u32], vs: &[u32]) -> Option<u8> {
    let mut tmp = [0u32; 4];
    for (s, pos) in kind.faces() {
        if pos.len() != vs.len() {
            continue;
        }
        for (t, &p) in tmp.iter_mut().zip(pos.iter()) {
            *t = nodes[p];
        }
        let f = &mut tmp[..pos.len()];
        f.sort_unstable();
        if f == vs {
            return Some(*s);
        }
    }
    None
}

enum Block {
    ElSet(String, Vec<u32>),
    NSet(String, Vec<u32>),
    Surface(String, Vec<(u32, u8)>),
}

/// Write `mesh` as an Abaqus `.inp` deck to `w`.
///
/// Tets, hexahedra and wedges are `C3D4`, `C3D8` and `C3D6` with the template node order of the
/// mesh API (which is the Abaqus order, positive volume); pyramids follow
/// [`PyramidPolicy`], polyhedra [`PolyPolicy`]. Loss codes: `abaqus.high_order_dropped`,
/// `abaqus.name_changed`, `abaqus.set_dropped`, `abaqus.set_member_not_exported`,
/// `abaqus.poly_dropped`, `abaqus.poly_decomposed`, `abaqus.nodes_added`,
/// `abaqus.pyramid_type_unverified`, `abaqus.pyramid_degenerate_hex`,
/// `abaqus.surface_interior_face`, `abaqus.surface_face_dropped`.
///
/// Errors: dimension other than 3, inactive cells or non-finite coordinates
/// ([`AbaqusError::Unsupported`]); polyhedra or pyramids under the `Error` policies; a
/// non-conforming decomposition.
pub fn write_abaqus(mesh: &dyn MeshRead, o: &AbaqusOptions, w: &mut dyn Write) -> Result<AbaqusReport, AbaqusError> {
    if mesh.dim() != MeshDim::Three {
        return unsupported("an Abaqus export needs a dimension-3 mesh");
    }
    let nv = mesh.vertex_count();
    let nc = mesh.cell_count();
    let co = mesh.coords();
    if co.len() < nv {
        return unsupported("fewer coordinates than vertices");
    }
    if nv >= u32::MAX as usize / 2 || nc >= u32::MAX as usize / 2 {
        return Err(AbaqusError::TooLarge("more than 2^31 nodes or cells"));
    }
    let mut pts: Vec<[f64; 3]> = (0..nv).map(|i| [co.x[i], co.y[i], co.z[i]]).collect();
    if pts.iter().flatten().any(|c| !c.is_finite()) {
        return unsupported("a vertex coordinate is not finite");
    }
    let mut loss = LossReport::new();

    let shape_of = |c: usize| mesh.cell_shape(CellIx::new(c));
    let polys: Vec<usize> = (0..nc).filter(|&c| shape_of(c) == CellShape::Polyhedron).collect();
    if !polys.is_empty() && o.polyhedra == PolyPolicy::Error {
        return Err(AbaqusError::PolyhedraUnsupported { count: polys.len() as u64 });
    }
    let n_pyr = (0..nc).filter(|&c| shape_of(c) == CellShape::Pyramid).count() as u64;
    if n_pyr > 0 && o.pyramids == PyramidPolicy::Error {
        return Err(AbaqusError::PyramidsUnsupported { count: n_pyr });
    }
    // 0 written as is, 1 dropped, 2 decomposed.
    let mut state = vec![0u8; nc];
    for &c in &polys {
        state[c] = if o.polyhedra == PolyPolicy::Drop { 1 } else { 2 };
    }

    // Cells. Element id = cell index + 1.
    let mut el = Elems::default();
    let mut rec: Vec<(u32, u32)> = vec![(0, 0); nc];
    let mut high_order = 0u64;
    for c in 0..nc {
        let ci = CellIx::new(c);
        if !mesh.cell_active(ci) {
            return unsupported("inactive (refined parent) cells: extract the leaf mesh first");
        }
        if state[c] != 0 {
            continue;
        }
        let (kind, n) = match mesh.cell_shape(ci) {
            CellShape::Tet => (Kind::Tet, 4),
            CellShape::Hex => (Kind::Hex, 8),
            CellShape::Wedge => (Kind::Wedge, 6),
            CellShape::Pyramid => (if o.pyramids == PyramidPolicy::C3d5 { Kind::Pyr } else { Kind::PyrHex }, 5),
            s => return unsupported(format!("cell {c} has shape {s:?}")),
        };
        let Some(corners) = mesh.cell_corners(ci).filter(|k| k.len() == n) else {
            return unsupported(format!("cell {c} has no {n} corner vertices"));
        };
        if corners.iter().any(|v| v.get() >= nv) {
            return unsupported(format!("cell {c} indexes a missing vertex"));
        }
        if mesh.high_order_cell(ci).is_some() {
            high_order += 1;
        }
        let mut ids = [0u32; 8];
        for (d, v) in ids.iter_mut().zip(corners) {
            *d = v.0;
        }
        let n = if kind == Kind::PyrHex {
            let apex = ids[4];
            ids[5..8].fill(apex);
            8
        } else {
            n
        };
        rec[c] = (el.id.len() as u32, el.id.len() as u32 + 1);
        el.push(c as u32 + 1, kind, &ids[..n]);
    }
    if high_order > 0 {
        loss.push(LossItem::new("abaqus.high_order_dropped", LossSeverity::Degraded, high_order, "high-order cells are written as their linear corner types"));
    }
    if n_pyr > 0 {
        let (code, sev, detail) = match o.pyramids {
            PyramidPolicy::C3d5 => ("abaqus.pyramid_type_unverified", LossSeverity::Info, "C3D5 is not confirmed by any reader available to the exporter"),
            _ => ("abaqus.pyramid_degenerate_hex", LossSeverity::Degraded, "pyramids are collapsed C3D8 elements (nodes 5-8 at the apex)"),
        };
        loss.push(LossItem::new(code, sev, n_pyr, detail));
    }

    // Decomposition: new nodes, then tetrahedra (ids after the last cell id).
    let mut face_centre: BTreeMap<u32, u32> = BTreeMap::new();
    // (cell, face) -> ids of the tets whose S1 is that face.
    let mut fan: BTreeMap<(u32, u32), Vec<u32>> = BTreeMap::new();
    if !polys.is_empty() && o.polyhedra == PolyPolicy::Decompose {
        for &c in &polys {
            for sf in mesh.cell_faces(CellIx::new(c)) {
                let f = sf.face();
                if mesh.face_vertices(f).len() >= 4
                    && let Some(nb) = mesh.across(*sf)
                    && state.get(nb.get()).is_none_or(|s| *s != 2)
                {
                    return Err(AbaqusError::NonconformingDecomposition { face: f.0 });
                }
            }
        }
        let mut next_id = nc as u32 + 1;
        for &c in &polys {
            let ci = CellIx::new(c);
            let mut vs: Vec<u32> = mesh.cell_faces(ci).iter().flat_map(|sf| mesh.face_vertices(sf.face()).iter().map(|v| v.0)).collect();
            vs.sort_unstable();
            vs.dedup();
            if vs.iter().any(|&v| v as usize >= nv) {
                return unsupported(format!("cell {c} indexes a missing vertex"));
            }
            let cc = pts.len() as u32;
            pts.push(centroid(&vs, &pts));
            let first = el.id.len() as u32;
            for sf in mesh.cell_faces(ci) {
                let f = sf.face();
                let mut loop_: Vec<u32> = mesh.face_vertices(f).iter().map(|v| v.0).collect();
                if loop_.len() < 3 || loop_.iter().any(|&v| v as usize >= nv) {
                    return unsupported(format!("face {} is not a valid polygon", f.0));
                }
                if sf.side() == Side::Right {
                    loop_.reverse();
                }
                let n = loop_.len();
                let ids = fan.entry((c as u32, f.0)).or_default();
                if n == 3 {
                    ids.push(next_id);
                    el.push(next_id, Kind::Tet, &[loop_[0], loop_[2], loop_[1], cc]);
                    next_id += 1;
                    continue;
                }
                let fc = *face_centre.entry(f.0).or_insert_with(|| {
                    let id = pts.len() as u32;
                    let p = centroid(&loop_, &pts);
                    pts.push(p);
                    id
                });
                for k in 0..n {
                    let (a, b) = (loop_[k], loop_[(k + 1) % n]);
                    ids.push(next_id);
                    el.push(next_id, Kind::Tet, &[a, fc, b, cc]);
                    next_id += 1;
                }
            }
            rec[c] = (first, el.id.len() as u32);
        }
        loss.push(LossItem::new("abaqus.poly_decomposed", LossSeverity::Degraded, polys.len() as u64, "polyhedra split into tetrahedra"));
        loss.push(LossItem::new("abaqus.nodes_added", LossSeverity::Info, (pts.len() - nv) as u64, "cell and face centroid nodes"));
    } else if !polys.is_empty() {
        loss.push(LossItem::new("abaqus.poly_dropped", LossSeverity::Dropped, polys.len() as u64, "polyhedral cells are not written"));
    }
    if el.id.is_empty() {
        return unsupported("an Abaqus deck needs at least one written element");
    }

    // Sets.
    let mut names = Names { used: BTreeSet::new() };
    let mut elsets: Vec<Block> = Vec::new();
    let mut nsets: Vec<Block> = Vec::new();
    let mut surfaces: Vec<Block> = Vec::new();
    let (mut interior, mut face_dropped, mut not_exported) = (0u64, 0u64, 0u64);
    let nodes_of_records = |range: &mut dyn Iterator<Item = (u32, u32)>| -> Vec<u32> {
        let mut v: Vec<u32> = range.flat_map(|(a, b)| (a..b).flat_map(|r| el.nodes(r as usize).iter().map(|n| n + 1))).collect();
        v.sort_unstable();
        v.dedup();
        v
    };
    for s in mesh.sets() {
        let vsets = |members: Vec<u32>| -> Vec<u32> {
            let mut v = members;
            v.sort_unstable();
            v.dedup();
            v
        };
        match &s.members {
            SetMembers::Cells(v) => {
                let mut ids: Vec<u32> = Vec::new();
                for c in v.iter().filter(|c| c.get() < nc) {
                    let (a, b) = rec[c.get()];
                    ids.extend((a..b).map(|r| el.id[r as usize]));
                }
                if ids.is_empty() {
                    loss.push(LossItem::new("abaqus.set_dropped", LossSeverity::Dropped, 1, format!("{}: no written element", s.name)));
                    continue;
                }
                let name = names.take(&s.name, &mut loss);
                if o.nsets {
                    let nodes = nodes_of_records(&mut v.iter().filter(|c| c.get() < nc).map(|c| rec[c.get()]));
                    nsets.push(Block::NSet(name.clone(), nodes));
                }
                elsets.push(Block::ElSet(name, vsets(ids)));
            }
            SetMembers::Faces(_) | SetMembers::Sides(_) => {
                // (face, wanted cell) per member.
                let members: Vec<(FaceIx, Option<CellIx>)> = match &s.members {
                    SetMembers::Faces(v) => v
                        .iter()
                        .map(|&f| {
                            if f.get() < mesh.face_count() {
                                if mesh.face_right(f).is_some() {
                                    interior += 1;
                                }
                                (f, Some(mesh.face_left(f)))
                            } else {
                                (f, None)
                            }
                        })
                        .collect(),
                    SetMembers::Sides(v) => v
                        .iter()
                        .map(|x| {
                            let f = x.face();
                            if f.get() >= mesh.face_count() {
                                return (f, None);
                            }
                            (f, if x.side() == Side::Left { Some(mesh.face_left(f)) } else { mesh.face_right(f) })
                        })
                        .collect(),
                    _ => unreachable!("matched above"),
                };
                let mut entries: Vec<(u32, u8)> = Vec::new();
                let mut fnodes: Vec<u32> = Vec::new();
                for (f, cell) in members {
                    let Some(cell) = cell.filter(|c| c.get() < nc) else {
                        face_dropped += 1;
                        continue;
                    };
                    let fv = mesh.face_vertices(f);
                    fnodes.extend(fv.iter().filter(|v| v.get() < nv).map(|v| v.0 + 1));
                    if let Some(&fc) = face_centre.get(&f.0) {
                        fnodes.push(fc + 1);
                    }
                    let c = cell.get();
                    if state[c] == 1 {
                        face_dropped += 1;
                    } else if state[c] == 2 {
                        match fan.get(&(c as u32, f.0)) {
                            Some(ids) => entries.extend(ids.iter().map(|&i| (i, 1u8))),
                            None => face_dropped += 1,
                        }
                    } else {
                        let r = rec[c].0 as usize;
                        let mut vs: Vec<u32> = fv.iter().map(|v| v.0).collect();
                        vs.sort_unstable();
                        match local_face(el.kind[r], el.nodes(r), &vs) {
                            Some(sn) => entries.push((el.id[r], sn)),
                            None => face_dropped += 1,
                        }
                    }
                }
                if entries.is_empty() {
                    loss.push(LossItem::new("abaqus.set_dropped", LossSeverity::Dropped, 1, format!("{}: no written element face", s.name)));
                    continue;
                }
                let name = names.take(&s.name, &mut loss);
                if o.nsets {
                    nsets.push(Block::NSet(name.clone(), vsets(fnodes)));
                }
                surfaces.push(Block::Surface(name, entries));
            }
            SetMembers::Vertices(v) => {
                let ids: Vec<u32> = v.iter().filter(|x| x.get() < nv).map(|x| x.0 + 1).collect();
                not_exported += (v.len() - ids.len()) as u64;
                if ids.is_empty() {
                    loss.push(LossItem::new("abaqus.set_dropped", LossSeverity::Dropped, 1, format!("{}: no node", s.name)));
                    continue;
                }
                nsets.push(Block::NSet(names.take(&s.name, &mut loss), vsets(ids)));
            }
            SetMembers::Points(v) => {
                let ids: Vec<u32> = v.iter().filter(|x| x.get() < nv).map(|x| x.0 + 1).collect();
                not_exported += (v.len() - ids.len()) as u64;
                if ids.is_empty() {
                    loss.push(LossItem::new("abaqus.set_dropped", LossSeverity::Dropped, 1, format!("{}: no vertex node", s.name)));
                    continue;
                }
                nsets.push(Block::NSet(names.take(&s.name, &mut loss), vsets(ids)));
            }
            _ => {
                loss.push(LossItem::new("abaqus.set_dropped", LossSeverity::Dropped, 1, format!("{}: shell-side sets", s.name)));
            }
        }
    }
    if interior > 0 {
        loss.push(LossItem::new("abaqus.surface_interior_face", LossSeverity::Info, interior, "faces of a face set between two cells use the left cell's side"));
    }
    if face_dropped > 0 {
        loss.push(LossItem::new("abaqus.surface_face_dropped", LossSeverity::Degraded, face_dropped, "set faces whose cell is not written or matches no element face"));
    }
    if not_exported > 0 {
        loss.push(LossItem::new("abaqus.set_member_not_exported", LossSeverity::Degraded, not_exported, "node set members that are no vertex"));
    }

    // Text.
    let mut buf: Vec<u8> = Vec::new();
    buf.extend_from_slice(b"*HEADING\nsmart-mesh export\n");
    buf.extend_from_slice(b"*NODE\n");
    let chunks: Vec<Vec<u8>> = pts
        .par_chunks(CHUNK)
        .enumerate()
        .map(|(i, ch)| {
            let mut b = Vec::with_capacity(ch.len() * 64);
            for (k, p) in ch.iter().enumerate() {
                let _ = write!(b, "{}", i * CHUNK + k + 1);
                for x in p {
                    b.extend_from_slice(b", ");
                    real(&mut b, *x);
                }
                b.push(b'\n');
            }
            b
        })
        .collect();
    chunks.iter().for_each(|c| buf.extend_from_slice(c));
    let mut elements_by_type: BTreeMap<String, u64> = BTreeMap::new();
    for (t, ty) in TYPES.iter().enumerate() {
        let idx: Vec<u32> = (0..el.id.len() as u32).filter(|&r| el.kind[r as usize].ty() == t).collect();
        if idx.is_empty() {
            continue;
        }
        elements_by_type.insert(ty.name.to_string(), idx.len() as u64);
        let _ = writeln!(buf, "*ELEMENT, TYPE={}", ty.name);
        let chunks: Vec<Vec<u8>> = idx
            .par_chunks(CHUNK)
            .map(|ch| {
                let mut b = Vec::with_capacity(ch.len() * 48);
                for &r in ch {
                    let _ = write!(b, "{}", el.id[r as usize]);
                    for n in el.nodes(r as usize) {
                        let _ = write!(b, ", {}", n + 1);
                    }
                    b.push(b'\n');
                }
                b
            })
            .collect();
        chunks.iter().for_each(|c| buf.extend_from_slice(c));
    }
    let list = |buf: &mut Vec<u8>, v: &[u32]| {
        for line in v.chunks(PER_LINE) {
            let s: Vec<String> = line.iter().map(u32::to_string).collect();
            buf.extend_from_slice(s.join(", ").as_bytes());
            buf.push(b'\n');
        }
    };
    for b in &elsets {
        if let Block::ElSet(n, v) = b {
            let _ = writeln!(buf, "*ELSET, ELSET={n}");
            list(&mut buf, v);
        }
    }
    for b in &nsets {
        if let Block::NSet(n, v) = b {
            let _ = writeln!(buf, "*NSET, NSET={n}");
            list(&mut buf, v);
        }
    }
    for b in &surfaces {
        if let Block::Surface(n, v) = b {
            let _ = writeln!(buf, "*SURFACE, TYPE=ELEMENT, NAME={n}");
            for (id, s) in v {
                let _ = writeln!(buf, "{id}, S{s}");
            }
        }
    }
    w.write_all(&buf)?;
    Ok(AbaqusReport {
        nodes: pts.len() as u64,
        elements_by_type,
        elsets: elsets.len() as u64,
        nsets: nsets.len() as u64,
        surfaces: surfaces.len() as u64,
        loss,
        bytes: buf.len() as u64,
        hash: fnv(&buf),
    })
}
