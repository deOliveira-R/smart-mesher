//! The msh4 writer.

use super::{GmshError, GmshOptions, GmshReport, PolyPolicy};
use crate::report::{LossItem, LossReport, LossSeverity};
use sm_core::EntityKind;
use sm_mesh_api::{CellIx, CellShape, FaceIx, MeshDim, MeshRead, SetMembers, Side};
use std::collections::{BTreeMap, BTreeSet};
use std::io::Write;

const HINT: &str = "use openfoam, vtu or exodus";

/// Gmsh element type codes.
const T_LINE: u8 = 1;
const T_TRI: u8 = 2;
const T_QUAD: u8 = 3;
const T_TET: u8 = 4;
const T_HEX: u8 = 5;
const T_WEDGE: u8 = 6;
const T_PYRAMID: u8 = 7;
const T_POINT: u8 = 15;

/// Token writer: ASCII (space separated, one record per line) or binary (little endian).
struct Out {
    buf: Vec<u8>,
    binary: bool,
    first: bool,
}

impl Out {
    fn new(binary: bool) -> Self {
        Self { buf: Vec::new(), binary, first: true }
    }
    fn sep(&mut self) {
        if !self.first {
            self.buf.push(b' ');
        }
        self.first = false;
    }
    fn text(&mut self, s: &str) {
        self.buf.extend_from_slice(s.as_bytes());
    }
    fn i32(&mut self, v: i32) {
        if self.binary {
            self.buf.extend_from_slice(&v.to_le_bytes());
        } else {
            self.sep();
            let _ = write!(self.buf, "{v}");
        }
    }
    fn sz(&mut self, v: u64) {
        if self.binary {
            self.buf.extend_from_slice(&v.to_le_bytes());
        } else {
            self.sep();
            let _ = write!(self.buf, "{v}");
        }
    }
    fn f64(&mut self, v: f64) {
        if self.binary {
            self.buf.extend_from_slice(&v.to_le_bytes());
        } else {
            self.sep();
            let _ = write!(self.buf, "{v}");
        }
    }
    fn eol(&mut self) {
        if !self.binary {
            self.buf.push(b'\n');
            self.first = true;
        }
    }
    fn end_section(&mut self, name: &str) {
        if self.binary {
            self.buf.push(b'\n');
        }
        self.text(&format!("$End{name}\n"));
    }
}

fn fnv(bytes: &[u8]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    for b in bytes {
        h = (h ^ u64::from(*b)).wrapping_mul(0x0100_0000_01b3);
    }
    h
}

fn unsupported<T>(m: impl Into<String>) -> Result<T, GmshError> {
    Err(GmshError::Unsupported(m.into()))
}

/// Flat element list: key (dimension, model serial, signature id), Gmsh type, node indices.
#[derive(Default)]
struct Elems {
    dim: Vec<u8>,
    serial: Vec<u64>,
    sig: Vec<u32>,
    etype: Vec<u8>,
    off: Vec<u32>,
    conn: Vec<u32>,
}

impl Elems {
    fn len(&self) -> usize {
        self.dim.len()
    }
    fn push(&mut self, key: (u8, u64, u32), etype: u8, nodes: &[u32]) {
        self.dim.push(key.0);
        self.serial.push(key.1);
        self.sig.push(key.2);
        self.etype.push(etype);
        self.off.push(self.conn.len() as u32);
        self.conn.extend_from_slice(nodes);
    }
    fn nodes(&self, i: usize) -> &[u32] {
        let end = self.off.get(i + 1).map_or(self.conn.len(), |&e| e as usize);
        &self.conn[self.off[i] as usize..end]
    }
}

/// Interned set-membership signatures: sorted physical tag lists.
struct Sigs {
    table: Vec<Vec<u32>>,
    lookup: BTreeMap<Vec<u32>, u32>,
}

impl Sigs {
    fn new() -> Self {
        Self { table: vec![Vec::new()], lookup: BTreeMap::from([(Vec::new(), 0)]) }
    }
    fn intern(&mut self, v: Vec<u32>) -> u32 {
        if let Some(&i) = self.lookup.get(&v) {
            return i;
        }
        let i = self.table.len() as u32;
        self.table.push(v.clone());
        self.lookup.insert(v, i);
        i
    }
    /// Signature id of each of `n` items given the member lists of the groups.
    fn of_items(&mut self, n: usize, groups: &[(u32, Vec<u32>)]) -> Vec<u32> {
        let mut per: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
        for (tag, items) in groups {
            for &i in items {
                if (i as usize) < n {
                    per.entry(i).or_default().push(*tag);
                }
            }
        }
        let mut sig = vec![0u32; n];
        for (i, mut tags) in per {
            tags.sort_unstable();
            tags.dedup();
            sig[i as usize] = self.intern(tags);
        }
        sig
    }
}

struct Group {
    dim: u8,
    tag: u32,
    name: String,
    items: Vec<u32>,
}

fn set_groups(mesh: &dyn MeshRead, loss: &mut LossReport) -> Result<Vec<Group>, GmshError> {
    let nv = mesh.vertex_count();
    let mut raw: Vec<(u8, &sm_mesh_api::MeshSet, Vec<u32>)> = Vec::new();
    let (mut sides, mut shells) = (Vec::new(), Vec::new());
    for s in mesh.sets() {
        if s.name.contains('"') || s.name.contains('\n') {
            return Err(GmshError::BadName(s.name.clone()));
        }
        match &s.members {
            SetMembers::Cells(v) => raw.push((3, s, v.iter().map(|c| c.0).collect())),
            SetMembers::Faces(v) => raw.push((2, s, v.iter().map(|f| f.0).collect())),
            SetMembers::Sides(v) => {
                sides.push(s.name.clone());
                raw.push((2, s, v.iter().map(|x| x.face().0).collect()));
            }
            SetMembers::Vertices(v) => raw.push((0, s, v.iter().map(|x| x.0).collect())),
            SetMembers::Points(v) => raw.push((0, s, v.iter().filter(|p| p.get() < nv).map(|p| p.0).collect())),
            SetMembers::ShellSides(_) => shells.push(s.name.clone()),
            _ => shells.push(s.name.clone()),
        }
    }
    if !sides.is_empty() {
        loss.push(LossItem::new("msh.side_orientation_dropped", LossSeverity::Degraded, sides.len() as u64, sides.join(",")));
    }
    if !shells.is_empty() {
        loss.push(LossItem::new("msh.set_dropped", LossSeverity::Dropped, shells.len() as u64, format!("shell-side sets: {}", shells.join(","))));
    }
    let mut used: [BTreeSet<u32>; 4] = Default::default();
    let mut tags: Vec<Option<u32>> = Vec::new();
    for (dim, s, _) in &raw {
        match s.label {
            Some(l) => {
                let t = u32::try_from(l).ok().filter(|t| *t >= 1 && *t <= i32::MAX as u32);
                match t {
                    Some(t) if used[*dim as usize].insert(t) => tags.push(Some(t)),
                    _ => return Err(GmshError::BadLabel { set: s.name.clone(), label: l }),
                }
            }
            None => tags.push(None),
        }
    }
    let mut next = [1u32; 4];
    let mut out = Vec::with_capacity(raw.len());
    for ((dim, s, items), t) in raw.into_iter().zip(tags) {
        let tag = match t {
            Some(t) => t,
            None => {
                while used[dim as usize].contains(&next[dim as usize]) {
                    next[dim as usize] += 1;
                }
                let t = next[dim as usize];
                used[dim as usize].insert(t);
                t
            }
        };
        out.push(Group { dim, tag, name: s.name.clone(), items });
    }
    out.sort_by_key(|g| (g.dim, g.tag));
    Ok(out)
}

fn class_dim(k: EntityKind) -> Option<u8> {
    match k {
        EntityKind::TopoVertex => Some(0),
        EntityKind::TopoEdge => Some(1),
        EntityKind::TopoFace => Some(2),
        EntityKind::TopoVolume => Some(3),
        _ => None,
    }
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

/// Write `mesh` as Gmsh MSH 4.1.
///
/// Node tags are vertex index + 1 (nodes added by [`PolyPolicy::Decompose`] follow). Each
/// node belongs to the entity of its classification (a split volume gives its nodes to the
/// lowest-tagged piece). Loss codes: `msh.high_order_dropped`, `msh.side_orientation_dropped`,
/// `msh.set_dropped`, `msh.entity_split`, `msh.entity_boundaries_partial`,
/// `msh.poly_dropped`, `msh.poly_decomposed`, `msh.nodes_added`, `msh.polygon_face_dropped`,
/// `msh.faces_unclassified_skipped`, `msh.set_member_not_exported`.
pub fn write_msh4(mesh: &dyn MeshRead, o: &GmshOptions, w: &mut dyn Write) -> Result<GmshReport, GmshError> {
    if mesh.dim() != MeshDim::Three {
        return unsupported("msh4 export needs a dimension-3 mesh");
    }
    let nv = mesh.vertex_count();
    let nc = mesh.cell_count();
    let co = mesh.coords();
    if co.len() < nv {
        return unsupported("fewer coordinates than vertices");
    }
    let mut pts: Vec<[f64; 3]> = (0..nv).map(|i| [co.x[i], co.y[i], co.z[i]]).collect();
    if pts.iter().flatten().any(|c| !c.is_finite()) {
        return unsupported("a vertex coordinate is not finite");
    }
    let mut loss = LossReport::new();
    let groups = set_groups(mesh, &mut loss)?;
    let mut sigs = Sigs::new();
    let by_dim = |d: u8| -> Vec<(u32, Vec<u32>)> { groups.iter().filter(|g| g.dim == d).map(|g| (g.tag, g.items.clone())).collect() };
    let cell_sig = sigs.of_items(nc, &by_dim(3));
    let face_sig = sigs.of_items(mesh.face_count(), &by_dim(2));
    let vert_sig = sigs.of_items(nv, &by_dim(0));

    // Polyhedra policy.
    let polys: Vec<usize> = (0..nc).filter(|&c| mesh.cell_shape(CellIx::new(c)) == CellShape::Polyhedron).collect();
    if !polys.is_empty() && o.polyhedra == PolyPolicy::Error {
        return Err(GmshError::PolyhedraUnsupported { count: polys.len() as u64, hint: HINT });
    }
    // 0 written as is, 1 dropped, 2 decomposed.
    let mut state = vec![0u8; nc];
    for &c in &polys {
        state[c] = if o.polyhedra == PolyPolicy::Drop { 1 } else { 2 };
    }

    // Cells.
    let mut el = Elems::default();
    let mut cell_key: Vec<Option<(u64, u32)>> = vec![None; nc];
    let mut vol_serial: Vec<u64> = Vec::with_capacity(nc);
    let mut high_order = 0u64;
    for c in 0..nc {
        let ci = CellIx::new(c);
        if !mesh.cell_active(ci) {
            return unsupported("inactive (refined parent) cells: extract the leaf mesh first");
        }
        let cl = mesh.cell_class(ci).entity;
        let serial = match cl.kind() {
            EntityKind::TopoVolume => cl.serial(),
            k => return unsupported(format!("cell {c} is classified on {k:?}, not a TopoVolume (map partitions to their TopoVolume first)")),
        };
        vol_serial.push(serial);
        if state[c] == 1 {
            continue;
        }
        cell_key[c] = Some((serial, cell_sig[c]));
        if state[c] == 2 {
            continue;
        }
        let (ty, n) = match mesh.cell_shape(ci) {
            CellShape::Tet => (T_TET, 4),
            CellShape::Hex => (T_HEX, 8),
            CellShape::Wedge => (T_WEDGE, 6),
            CellShape::Pyramid => (T_PYRAMID, 5),
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
        el.push((3, serial, cell_sig[c]), ty, &ids[..n]);
    }
    if high_order > 0 {
        loss.push(LossItem::new("msh.high_order_dropped", LossSeverity::Degraded, high_order, "high-order cells are written as their linear corner types"));
    }

    // Decomposition: new nodes, then tetrahedra.
    let mut extra_owner: Vec<(u64, u32)> = Vec::new();
    let mut face_centre: BTreeMap<u32, u32> = BTreeMap::new();
    if !polys.is_empty() && o.polyhedra == PolyPolicy::Decompose {
        for &c in &polys {
            let ci = CellIx::new(c);
            for sf in mesh.cell_faces(ci) {
                let f = sf.face();
                if mesh.face_vertices(f).len() >= 4
                    && let Some(nb) = mesh.across(*sf)
                    && state[nb.get()] != 2
                {
                    return Err(GmshError::NonconformingDecomposition { face: f.0 });
                }
            }
        }
        for &c in &polys {
            let ci = CellIx::new(c);
            let Some(key) = cell_key[c] else { continue };
            let mut vs: Vec<u32> = mesh.cell_faces(ci).iter().flat_map(|sf| mesh.face_vertices(sf.face()).iter().map(|v| v.0)).collect();
            vs.sort_unstable();
            vs.dedup();
            if vs.iter().any(|&v| v as usize >= nv) {
                return unsupported(format!("cell {c} indexes a missing vertex"));
            }
            let cc = pts.len() as u32;
            pts.push(centroid(&vs, &pts));
            extra_owner.push(key);
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
                if n == 3 {
                    el.push((3, key.0, key.1), T_TET, &[loop_[0], loop_[2], loop_[1], cc]);
                    continue;
                }
                let fc = *face_centre.entry(f.0).or_insert_with(|| {
                    let id = pts.len() as u32;
                    let p = centroid(&loop_, &pts);
                    pts.push(p);
                    extra_owner.push(key);
                    id
                });
                for k in 0..n {
                    let (a, b) = (loop_[k], loop_[(k + 1) % n]);
                    el.push((3, key.0, key.1), T_TET, &[a, fc, b, cc]);
                }
            }
        }
        loss.push(LossItem::new("msh.poly_decomposed", LossSeverity::Degraded, polys.len() as u64, "polyhedra split into tetrahedra"));
        loss.push(LossItem::new("msh.nodes_added", LossSeverity::Info, (pts.len() - nv) as u64, "cell and face centroid nodes"));
    } else if !polys.is_empty() {
        loss.push(LossItem::new("msh.poly_dropped", LossSeverity::Dropped, polys.len() as u64, "polyhedral cells are not written"));
    }

    // Faces: boundary and interface faces on a TopoFace.
    struct FaceLink {
        key: (u64, u32),
        /// Written adjacent cells with the sign of the surface in their boundary.
        cells: [Option<(u32, i32)>; 2],
    }
    let mut links: Vec<FaceLink> = Vec::new();
    let (mut unclassified, mut polygon, mut not_exported) = (0u64, 0u64, 0u64);
    if o.boundary_elements {
        let mut written = vec![false; mesh.face_count()];
        for f in 0..mesh.face_count() {
            let fi = FaceIx::new(f);
            let l = mesh.face_left(fi);
            let r = mesh.face_right(fi);
            let (Some(&lv), rv) = (vol_serial.get(l.get()), r.and_then(|c| vol_serial.get(c.get()).copied())) else {
                return unsupported(format!("face {f} has a cell out of range"));
            };
            if rv == Some(lv) {
                continue;
            }
            let left_written = cell_key[l.get()].is_some();
            let right_written = r.is_some_and(|c| cell_key[c.get()].is_some());
            if !left_written && !right_written {
                continue;
            }
            let fc = mesh.face_class(fi).entity;
            if fc.kind() != EntityKind::TopoFace {
                unclassified += 1;
                continue;
            }
            let vs: Vec<u32> = mesh.face_vertices(fi).iter().map(|v| v.0).collect();
            if vs.iter().any(|&v| v as usize >= nv) {
                return unsupported(format!("face {f} indexes a missing vertex"));
            }
            let key = (2u8, fc.serial(), face_sig[f]);
            match (vs.len(), face_centre.get(&(f as u32)).copied()) {
                (3, _) => el.push(key, T_TRI, &vs),
                (4, None) => el.push(key, T_QUAD, &vs),
                (n, Some(c)) => {
                    // The fan of a decomposed neighbour's face centroid.
                    for k in 0..n {
                        el.push(key, T_TRI, &[vs[k], vs[(k + 1) % n], c]);
                    }
                }
                _ => {
                    polygon += 1;
                    continue;
                }
            }
            written[f] = true;
            let mut cells = [None, None];
            if left_written {
                cells[0] = Some((l.0, 1));
            }
            if let Some(c) = r.filter(|_| right_written) {
                cells[1] = Some((c.0, -1));
            }
            links.push(FaceLink { key: (fc.serial(), face_sig[f]), cells });
        }
        // Face-set members that were not written as elements.
        for g in groups.iter().filter(|g| g.dim == 2) {
            not_exported += g.items.iter().filter(|&&f| !written.get(f as usize).copied().unwrap_or(false)).count() as u64;
        }
    } else {
        for g in groups.iter().filter(|g| g.dim == 2) {
            not_exported += g.items.len() as u64;
        }
    }
    if unclassified > 0 {
        loss.push(LossItem::new("msh.faces_unclassified_skipped", LossSeverity::Dropped, unclassified, "boundary or interface faces not classified on a TopoFace are not written"));
    }
    if polygon > 0 {
        loss.push(LossItem::new("msh.polygon_face_dropped", LossSeverity::Dropped, polygon, "faces with five or more vertices have no msh4 element"));
    }

    // Lines and points.
    if o.lines_and_points {
        if let Some(t) = mesh.edges() {
            for (e, cl) in t.class.iter().enumerate() {
                if cl.entity.kind() == EntityKind::TopoEdge {
                    let [a, b] = t.vertices[e];
                    if a.get() < nv && b.get() < nv {
                        el.push((1, cl.entity.serial(), 0), T_LINE, &[a.0, b.0]);
                    }
                }
            }
        }
        for v in 0..nv {
            let c = mesh.vertex_class(sm_mesh_api::VertexIx::new(v)).entity;
            if c.kind() == EntityKind::TopoVertex {
                el.push((0, c.serial(), vert_sig[v]), T_POINT, &[v as u32]);
            }
        }
    }
    for g in groups.iter().filter(|g| g.dim == 0) {
        let lost = g.items.iter().filter(|&&v| !(o.lines_and_points && mesh.vertex_class(sm_mesh_api::VertexIx::new(v as usize)).entity.kind() == EntityKind::TopoVertex)).count() as u64;
        not_exported += lost;
    }
    if not_exported > 0 {
        loss.push(LossItem::new("msh.set_member_not_exported", LossSeverity::Dropped, not_exported, "face or vertex set members that have no dimension 2 or 0 element in the file"));
    }

    // Node entities: the classification entity of each vertex.
    let mut node_class: Vec<(u8, u64)> = Vec::with_capacity(pts.len());
    for v in 0..nv {
        let c = mesh.vertex_class(sm_mesh_api::VertexIx::new(v)).entity;
        let Some(d) = class_dim(c.kind()) else {
            return unsupported(format!("vertex {v} is classified on {:?} (map partitions to their Topo entity first)", c.kind()));
        };
        node_class.push((d, c.serial()));
    }

    // Entities: distinct (dim, serial, signature), ordered by (dim, serial, tags).
    let mut keys: Vec<(u8, u64, u32)> = (0..el.len()).map(|i| (el.dim[i], el.serial[i], el.sig[i])).collect();
    keys.sort_unstable();
    keys.dedup();
    let have: BTreeSet<(u8, u64)> = keys.iter().map(|k| (k.0, k.1)).collect();
    let mut need: BTreeSet<(u8, u64)> = node_class.iter().copied().collect();
    need.retain(|k| !have.contains(k));
    keys.extend(need.into_iter().map(|(d, s)| (d, s, 0)));
    let table = &sigs.table;
    let ord = |a: &(u8, u64, u32), b: &(u8, u64, u32)| (a.0, a.1, &table[a.2 as usize]).cmp(&(b.0, b.1, &table[b.2 as usize]));
    keys.sort_by(ord);
    keys.dedup();
    let ents = keys;
    let mut tag_of = vec![0u32; ents.len()];
    let mut counts = [0u32; 4];
    for (i, e) in ents.iter().enumerate() {
        counts[e.0 as usize] += 1;
        tag_of[i] = counts[e.0 as usize];
    }
    let find = |k: &(u8, u64, u32)| -> Option<usize> { ents.binary_search_by(|e| ord(e, k)).ok() };
    let distinct_volumes = ents.iter().filter(|e| e.0 == 3).map(|e| e.1).collect::<BTreeSet<_>>().len() as u64;
    if counts[3] as u64 > distinct_volumes {
        loss.push(LossItem::new("msh.entity_split", LossSeverity::Info, counts[3] as u64 - distinct_volumes, "cell sets that hold part of a volume split it into several Gmsh volumes"));
    }
    if counts[0] + counts[1] + counts[2] > 0 {
        loss.push(LossItem::new("msh.entity_boundaries_partial", LossSeverity::Info, (counts[0] + counts[1] + counts[2]) as u64, "only volume to surface boundaries are written; curve and point boundaries are empty"));
    }

    // Node -> entity index.
    let mut node_ent: Vec<u32> = Vec::with_capacity(pts.len());
    for &(d, s) in &node_class {
        let i = ents.partition_point(|e| (e.0, e.1) < (d, s));
        node_ent.push(i as u32);
    }
    // Nodes of a split volume go to its lowest piece; added nodes to their first cell's piece.
    for k in &extra_owner {
        node_ent.push(find(&(3, k.0, k.1)).unwrap_or(0) as u32);
    }
    let nn = node_ent.len();

    // Entity records: bbox from member nodes, physical tags.
    let ne = ents.len();
    let mut lo = vec![[f64::INFINITY; 3]; ne];
    let mut hi = vec![[f64::NEG_INFINITY; 3]; ne];
    let grow = |e: usize, n: usize, lo: &mut Vec<[f64; 3]>, hi: &mut Vec<[f64; 3]>| {
        for k in 0..3 {
            lo[e][k] = lo[e][k].min(pts[n][k]);
            hi[e][k] = hi[e][k].max(pts[n][k]);
        }
    };
    let elem_ent: Vec<u32> = (0..el.len()).map(|i| find(&(el.dim[i], el.serial[i], el.sig[i])).map_or(0, |e| e as u32)).collect();
    for i in 0..el.len() {
        for &n in el.nodes(i) {
            grow(elem_ent[i] as usize, n as usize, &mut lo, &mut hi);
        }
    }
    for (n, &e) in node_ent.iter().enumerate() {
        grow(e as usize, n, &mut lo, &mut hi);
    }
    // Volume -> surface boundaries with signs (+: the normal points out of the volume).
    let mut bound: Vec<BTreeMap<u32, i32>> = vec![BTreeMap::new(); ne];
    for l in &links {
        let Some(se) = find(&(2, l.key.0, l.key.1)) else { continue };
        let stag = tag_of[se] as i32;
        for (cell, sign) in l.cells.iter().flatten() {
            if let Some(k) = cell_key[*cell as usize]
                && let Some(ve) = find(&(3, k.0, k.1))
            {
                bound[ve].entry(stag.unsigned_abs()).or_insert(sign * stag);
            }
        }
    }

    // Emit.
    let mut out = Out::new(o.binary);
    out.text("$MeshFormat\n");
    out.text(if o.binary { "4.1 1 8\n" } else { "4.1 0 8\n" });
    if o.binary {
        out.buf.extend_from_slice(&1i32.to_le_bytes());
        out.buf.push(b'\n');
    }
    out.text("$EndMeshFormat\n");
    if !groups.is_empty() {
        out.text(&format!("$PhysicalNames\n{}\n", groups.len()));
        for g in &groups {
            out.text(&format!("{} {} \"{}\"\n", g.dim, g.tag, g.name));
        }
        out.text("$EndPhysicalNames\n");
    }
    out.text("$Entities\n");
    for c in counts {
        out.sz(c as u64);
    }
    out.eol();
    for (i, e) in ents.iter().enumerate() {
        out.i32(tag_of[i] as i32);
        if e.0 == 0 {
            for k in 0..3 {
                out.f64(if lo[i][k].is_finite() { lo[i][k] } else { 0.0 });
            }
        } else {
            for k in 0..3 {
                out.f64(if lo[i][k].is_finite() { lo[i][k] } else { 0.0 });
            }
            for k in 0..3 {
                out.f64(if hi[i][k].is_finite() { hi[i][k] } else { 0.0 });
            }
        }
        let phys = &sigs.table[e.2 as usize];
        out.sz(phys.len() as u64);
        for t in phys {
            out.i32(*t as i32);
        }
        if e.0 > 0 {
            if e.0 == 3 {
                out.sz(bound[i].len() as u64);
                for v in bound[i].values() {
                    out.i32(*v);
                }
            } else {
                out.sz(0);
            }
        }
        out.eol();
    }
    out.end_section("Entities");

    // Nodes grouped by entity (ascending), ascending node tag inside.
    let mut start = vec![0u32; ne + 1];
    for &e in &node_ent {
        start[e as usize + 1] += 1;
    }
    for i in 0..ne {
        start[i + 1] += start[i];
    }
    let mut fill = start.clone();
    let mut order = vec![0u32; nn];
    for (n, &e) in node_ent.iter().enumerate() {
        order[fill[e as usize] as usize] = n as u32;
        fill[e as usize] += 1;
    }
    let blocks = (0..ne).filter(|&i| start[i + 1] > start[i]).count();
    out.text("$Nodes\n");
    out.sz(blocks as u64);
    out.sz(nn as u64);
    out.sz(u64::from(nn > 0));
    out.sz(nn as u64);
    out.eol();
    for i in 0..ne {
        let (a, b) = (start[i] as usize, start[i + 1] as usize);
        if a == b {
            continue;
        }
        out.i32(i32::from(ents[i].0));
        out.i32(tag_of[i] as i32);
        out.i32(0);
        out.sz((b - a) as u64);
        out.eol();
        for &n in &order[a..b] {
            out.sz(u64::from(n) + 1);
            out.eol();
        }
        for &n in &order[a..b] {
            for k in 0..3 {
                out.f64(pts[n as usize][k]);
            }
            out.eol();
        }
    }
    out.end_section("Nodes");

    // Elements grouped by (dim, entity tag, type).
    let etag: Vec<u32> = elem_ent.iter().map(|&e| tag_of[e as usize]).collect();
    let mut eorder: Vec<u32> = (0..el.len() as u32).collect();
    eorder.sort_unstable_by_key(|&i| (el.dim[i as usize], etag[i as usize], el.etype[i as usize], i));
    let same = |a: u32, b: u32| (el.dim[a as usize], etag[a as usize], el.etype[a as usize]) == (el.dim[b as usize], etag[b as usize], el.etype[b as usize]);
    let mut runs: Vec<(usize, usize)> = Vec::new();
    let mut s = 0;
    while s < eorder.len() {
        let mut e = s + 1;
        while e < eorder.len() && same(eorder[s], eorder[e]) {
            e += 1;
        }
        runs.push((s, e));
        s = e;
    }
    out.text("$Elements\n");
    out.sz(runs.len() as u64);
    out.sz(el.len() as u64);
    out.sz(u64::from(!el.dim.is_empty()));
    out.sz(el.len() as u64);
    out.eol();
    let mut by_type: BTreeMap<u8, u64> = BTreeMap::new();
    let mut next_tag = 1u64;
    for (s, e) in runs {
        let first = eorder[s] as usize;
        out.i32(i32::from(el.dim[first]));
        out.i32(etag[first] as i32);
        out.i32(i32::from(el.etype[first]));
        out.sz((e - s) as u64);
        out.eol();
        *by_type.entry(el.etype[first]).or_default() += (e - s) as u64;
        for &i in &eorder[s..e] {
            out.sz(next_tag);
            next_tag += 1;
            for &n in el.nodes(i as usize) {
                out.sz(u64::from(n) + 1);
            }
            out.eol();
        }
    }
    out.end_section("Elements");

    w.write_all(&out.buf)?;
    Ok(GmshReport {
        nodes: nn as u64,
        elements_by_type: by_type,
        entities: counts.map(u64::from),
        physical_groups: groups.len() as u64,
        loss,
        bytes: out.buf.len() as u64,
        hash: fnv(&out.buf),
    })
}
