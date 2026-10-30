//! The dual as plain data (points, faces, cells), built from the analysis, optionally
//! agglomerated, and committed to a [`MeshBuild`] in one serial pass.

use crate::error::PolyError;
use crate::geom::{self, V3};
use crate::options::FaceMode;
use crate::options::DualReport;
use crate::concave::{self, Split};
use crate::prep::{NONE, Prep, canonical_cycle};
use sm_core::{EntityId, Point3};
use sm_mesh_api::{Classification, FaceIx, GeomTol, MeshBuild, MeshEntityRef, Side, SignedFace, VertexIx};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// Points, faces and cells of the dual. Face loops run outward from the face's first (lower)
/// cell; a cell lists its faces as [`SignedFace`] relative to those loops.
pub(crate) struct PolyData {
    pub pts: Vec<V3>,
    pub pcls: Vec<Classification>,
    /// Face loops in CSR form over point numbers.
    pub f_off: Vec<u32>,
    pub f_pts: Vec<u32>,
    /// Class of a boundary or interface polygon; `None` for edge faces and hub triangles.
    pub f_cls: Vec<Option<EntityId>>,
    /// The cells on the left and right of each face (`NONE` if absent).
    pub f_cells: Vec<[u32; 2]>,
    pub f_alive: Vec<bool>,
    /// Cells in CSR form.
    pub c_off: Vec<u32>,
    pub c_faces: Vec<SignedFace>,
    pub c_region: Vec<u32>,
    pub c_alive: Vec<bool>,
    /// Region ids, for the cell classification.
    pub regions: Vec<EntityId>,
    /// Points below this number are tet centroids (they lie only on edge faces).
    pub n_tet_pts: u32,
}

impl PolyData {
    pub fn n_faces(&self) -> usize {
        self.f_off.len() - 1
    }
    pub fn n_cells(&self) -> usize {
        self.c_off.len() - 1
    }
    pub fn face_loop(&self, f: usize) -> &[u32] {
        &self.f_pts[self.f_off[f] as usize..self.f_off[f + 1] as usize]
    }
    pub fn cell_faces(&self, c: usize) -> &[SignedFace] {
        &self.c_faces[self.c_off[c] as usize..self.c_off[c + 1] as usize]
    }

    fn push_face(&mut self, lp: &[u32], cls: Option<EntityId>) -> u32 {
        let f = self.n_faces() as u32;
        self.f_pts.extend_from_slice(lp);
        self.f_off.push(self.f_pts.len() as u32);
        self.f_cls.push(cls);
        self.f_cells.push([NONE, NONE]);
        self.f_alive.push(true);
        f
    }
}

/// Counters collected while building.
#[derive(Default)]
pub(crate) struct Counts {
    pub boundary_faces: usize,
    pub hub_split_faces: usize,
    pub hub_triangles: usize,
    /// Dual cells split into wedges, and the wedges they became.
    pub split_cells: usize,
    pub wedges: usize,
}

/// Number the used points and record faces and cells in `(region, vertex)` order (the wedges
/// of a split cell in wedge order). Returns the data, the counters and, per output cell, its
/// dual cell (ckey index).
pub(crate) fn build(prep: &Prep<'_>, split: &Split) -> (PolyData, Counts, Vec<u32>) {
    let (nt, nc, ns) = (prep.nt, prep.ckeys.len(), prep.s_edge.len());
    let refined = |s: usize| split.is_split(prep.s_ca[s]) || split.is_split(prep.s_cb[s]);
    let mut used = vec![false; prep.n_virtual()];
    used[..nt].fill(true);
    let mut hub_reg = vec![NONE; prep.ne];
    for s in 0..ns {
        let m = &prep.meta[s];
        let e = prep.s_edge[s];
        if m.open {
            used[prep.p_mid(e) as usize] = true;
        }
        let hub = if refined(s) {
            for &t in prep.ring_of(s) {
                if let Some((en, ex)) = prep.entry_exit(t, e) {
                    used[prep.p_tri(en) as usize] = true;
                    used[prep.p_tri(ex) as usize] = true;
                }
            }
            true
        } else {
            let (path, hub) = slot_path(prep, split, s);
            for &p in &path {
                used[p as usize] = true;
            }
            hub
        };
        if hub && !m.open {
            used[prep.p_hub(e) as usize] = true;
            hub_reg[e as usize] = prep.s_reg[s];
        }
    }
    for &c in split.cells.keys() {
        used[prep.p_vert(prep.ckeys[c as usize] as u32) as usize] = true;
    }
    for &p in &prep.fan_pts {
        used[p as usize] = true;
    }
    let mut num = vec![NONE; used.len()];
    let mut npts = 0u32;
    for (i, &u) in used.iter().enumerate() {
        if u {
            num[i] = npts;
            npts += 1;
        }
    }
    let mut d = PolyData {
        pts: Vec::with_capacity(npts as usize),
        pcls: Vec::with_capacity(npts as usize),
        f_off: vec![0],
        f_pts: Vec::new(),
        f_cls: Vec::new(),
        f_cells: Vec::new(),
        f_alive: Vec::new(),
        c_off: vec![0],
        c_faces: Vec::new(),
        c_region: Vec::with_capacity(nc),
        c_alive: Vec::new(),
        regions: prep.regions.clone(),
        n_tet_pts: nt as u32,
    };
    for (vid, _) in used.iter().enumerate().filter(|(_, u)| **u) {
        d.pts.push(prep.coord(vid as u32));
        d.pcls.push(point_class(prep, vid as u32, &hub_reg));
    }
    let nm = |vid: u32| num[vid as usize];

    let mut counts = Counts::default();
    let mut slot_face: Vec<(u32, u32)> = vec![(NONE, 0); ns];
    let mut c_key: Vec<u32> = Vec::with_capacity(nc);
    // Interface polygons by canonical point cycle -> (face, runs canonical).
    let mut shared: BTreeMap<SmallVec<[u32; 32]>, (u32, bool)> = BTreeMap::new();
    let (mut fan_i, mut pt_i) = (0usize, 0usize);
    let mut lp: SmallVec<[u32; 32]> = SmallVec::new();
    let mut cell_faces: Vec<SignedFace> = Vec::new();
    // Faces between the wedges of the current cell: (face, left wedge, right wedge).
    let mut seps: Vec<(u32, u32, u32)> = Vec::new();
    for cell in 0..nc {
        let slots = prep.slots_of_cell(cell);
        for &s in slots {
            let s = s as usize;
            if prep.s_ca[s] as usize == cell {
                let first = d.n_faces() as u32;
                let count = if refined(s) {
                    emit_kites(prep, s, &nm, &mut d)
                } else {
                    emit_slot(prep, split, s, &nm, &mut d, &mut lp, &mut counts)
                };
                slot_face[s] = (first, count);
            }
        }
        let key = prep.ckeys[cell];
        let (region, v) = ((key >> 32) as u32, key as u32);
        let nw = split.n_wedges(cell as u32);
        seps.clear();
        if split.is_split(cell as u32) {
            counts.split_cells += 1;
            counts.wedges += nw as usize;
            emit_separators(prep, split, cell, v, region, &nm, &mut d, &mut seps);
        }
        let (fan0, pt0) = (fan_i, pt_i);
        for w in 0..nw {
            let out = c_key.len() as u32;
            cell_faces.clear();
            for &s in slots {
                let s = s as usize;
                let lower = prep.s_ca[s] as usize == cell;
                let (first, count) = slot_face[s];
                let side = if lower { Side::Left } else { Side::Right };
                if refined(s) {
                    for (i, &t) in prep.ring_of(s).iter().enumerate() {
                        if split.wedge(cell as u32, t) == w {
                            cell_faces.push(SignedFace::new(FaceIx(first + i as u32), side));
                        }
                    }
                } else {
                    cell_faces.extend((0..count).map(|i| SignedFace::new(FaceIx(first + i), side)));
                }
            }
            for &(f, l, r) in &seps {
                if l == w {
                    cell_faces.push(SignedFace::new(FaceIx(f), Side::Left));
                } else if r == w {
                    cell_faces.push(SignedFace::new(FaceIx(f), Side::Right));
                }
            }
            (fan_i, pt_i) = (fan0, pt0);
            while fan_i < prep.fan_metas.len() && prep.fan_metas[fan_i].cell as usize == cell {
                let fm = prep.fan_metas[fan_i];
                let pts = &prep.fan_pts[pt_i..pt_i + fm.len as usize];
                fan_i += 1;
                pt_i += fm.len as usize;
                // fans of a split vertex are single triangles, so each has one wedge
                let fw = if nw > 1 && fm.tri != NONE { split.wedge(cell as u32, prep.tet_on_face(fm.tri, region)) } else { 0 };
                if fw != w {
                    continue;
                }
                lp.clear();
                lp.extend(pts.iter().map(|&p| nm(p)));
                let sf = if fm.iface {
                    let (key, fwd) = canonical_cycle(&lp);
                    match shared.get(&key) {
                        Some(&(f, first_fwd)) => {
                            SignedFace::new(FaceIx(f), if first_fwd == fwd { Side::Left } else { Side::Right })
                        }
                        None => {
                            let f = d.push_face(&lp, Some(fm.cls));
                            counts.boundary_faces += 1;
                            shared.insert(key, (f, fwd));
                            SignedFace::new(FaceIx(f), Side::Left)
                        }
                    }
                } else {
                    let f = d.push_face(&lp, Some(fm.cls));
                    counts.boundary_faces += 1;
                    SignedFace::new(FaceIx(f), Side::Left)
                };
                cell_faces.push(sf);
            }
            for sf in &cell_faces {
                let slot = &mut d.f_cells[sf.face().get()][usize::from(sf.side() == Side::Right)];
                *slot = out;
            }
            d.c_faces.extend_from_slice(&cell_faces);
            d.c_off.push(d.c_faces.len() as u32);
            d.c_region.push(region);
            c_key.push(cell as u32);
        }
    }
    d.c_alive = vec![true; c_key.len()];
    (d, counts, c_key)
}

/// Record the per-tet kites `(m_e, fc_entry, c_t, fc_exit)` of a refined slot, in ring order;
/// returns how many. They wind like the slot's polygon (about `a -> b`).
fn emit_kites(p: &Prep<'_>, s: usize, nm: &dyn Fn(u32) -> u32, d: &mut PolyData) -> u32 {
    let e = p.s_edge[s];
    let hub = nm(p.edge_mid(e));
    let ring = p.ring_of(s);
    for &t in ring {
        // the ring walk already found both faces of every ring tet
        let (en, ex) = p.entry_exit(t, e).unwrap_or((0, 0));
        d.push_face(&[hub, nm(p.p_tri(en)), nm(t), nm(p.p_tri(ex))], None);
    }
    ring.len() as u32
}

/// Record the quads `(v, m_vw1, fc_f, m_vw2)` between the wedges of split cell `cell`, oriented
/// from the lower wedge to the higher one.
#[allow(clippy::too_many_arguments)]
fn emit_separators(
    p: &Prep<'_>,
    split: &Split,
    cell: usize,
    v: u32,
    region: u32,
    nm: &dyn Fn(u32) -> u32,
    d: &mut PolyData,
    seps: &mut Vec<(u32, u32, u32)>,
) {
    let Some(list) = split.cells.get(&(cell as u32)) else { return };
    for &(t, wt) in list {
        let Some(c) = concave::corner(p, v, region, t) else { continue };
        for j in 0..3 {
            let n = c.nb[j];
            if n == NONE || n < t {
                continue;
            }
            let wn = split.wedge(cell as u32, n);
            if wn == wt {
                continue;
            }
            // the quad winds outward from t
            let q = c.quads[3 + j].map(nm);
            let f = if wt < wn { d.push_face(&q, None) } else { d.push_face(&[q[0], q[3], q[2], q[1]], None) };
            seps.push((f, wt.min(wn), wt.max(wn)));
        }
    }
}

/// Class of virtual point `vid`.
fn point_class(p: &Prep<'_>, vid: u32, hub_reg: &[u32]) -> Classification {
    let src = p.src;
    let (nt, nf, ne, nv) = (p.nt as u32, p.nf as u32, p.ne as u32, p.nv as u32);
    if vid < nt {
        return Classification::on(p.regions[p.treg[vid as usize] as usize]);
    }
    let i = vid - nt;
    if i < nf {
        let fi = FaceIx(i);
        let l = p.treg[src.face_left(fi).get()];
        if src.face_right(fi).is_some_and(|r| p.treg[r.get()] == l) {
            // interior triangle of a region (only used by refined edge faces)
            return Classification::on(p.regions[l as usize]);
        }
        return Classification::on(src.face_class(fi).entity);
    }
    let i = i - nf;
    if i < ne {
        // Every used midpoint belongs to an open path, which fixed its class.
        return Classification::on(p.mid_cls[i as usize].unwrap_or_else(|| p.regions[0]));
    }
    let i = i - ne;
    if i < nv {
        return src.classification(sm_mesh_api::MeshEntityKind::Vertex, i);
    }
    let e = (i - nv) as usize;
    Classification::on(p.regions[hub_reg[e] as usize])
}

/// Record the faces of slot `s` (an edge face, or its hub triangles); returns how many.
/// The open or closed path of an unrefined slot as virtual points (without the closing `m_e`)
/// and whether it is replaced by hub triangles. Next to a split cell the path also passes
/// through the centroid of every *bent* triangle it crosses (an interior triangle with a vertex
/// whose cell in this region is split): the refined faces of the split cell bend the dual edge
/// of that triangle through its centroid, and every face around that edge must follow.
fn slot_path(p: &Prep<'_>, split: &Split, s: usize) -> (SmallVec<[u32; 32]>, bool) {
    let m = p.meta[s];
    let e = p.s_edge[s];
    let region = p.s_reg[s];
    let ring = p.ring_of(s);
    let mut path: SmallVec<[u32; 32]> = SmallVec::new();
    if m.open {
        path.push(p.p_tri(m.f_in));
    }
    let mut bent_any = false;
    for (i, &t) in ring.iter().enumerate() {
        path.push(t);
        if m.open && i + 1 == ring.len() {
            break;
        }
        if !split.cells.is_empty()
            && let Some((_, ex)) = p.entry_exit(t, e)
            && p.src.face_vertices(sm_mesh_api::FaceIx(ex)).iter().any(|u| {
                let c = p.cell_of(region, u.0);
                c != NONE && split.is_split(c)
            })
        {
            path.push(p.p_tri(ex));
            bent_any = true;
        }
    }
    if m.open {
        path.push(p.p_tri(m.f_out));
    }
    let hub = if !bent_any {
        m.hub
    } else if p.mode == FaceMode::Planar {
        let mut pts: SmallVec<[V3; 32]> = path.iter().map(|&x| p.coord(x)).collect();
        if m.open {
            pts.push(p.coord(p.p_mid(e)));
        }
        geom::nonplanarity(&pts) > GeomTol::default().planarity
    } else {
        false
    };
    (path, hub)
}

fn emit_slot(
    p: &Prep<'_>,
    split: &Split,
    s: usize,
    nm: &dyn Fn(u32) -> u32,
    d: &mut PolyData,
    lp: &mut SmallVec<[u32; 32]>,
    counts: &mut Counts,
) -> u32 {
    let m = p.meta[s];
    let e = p.s_edge[s];
    let (vpath, hub) = slot_path(p, split, s);
    let path: SmallVec<[u32; 32]> = vpath.iter().map(|&x| nm(x)).collect();
    if !hub {
        lp.clear();
        lp.extend(path.iter().copied());
        if m.open {
            lp.push(nm(p.p_mid(e)));
        }
        d.push_face(lp, None);
        return 1;
    }
    counts.hub_split_faces += 1;
    let (hub, n) = if m.open { (nm(p.p_mid(e)), path.len() - 1) } else { (nm(p.p_hub(e)), path.len()) };
    for i in 0..n {
        d.push_face(&[hub, path[i], path[(i + 1) % path.len()]], None);
    }
    counts.hub_triangles += n;
    n as u32
}

/// Feed the surviving points, faces and cells to the builder.
pub(crate) fn commit<B: MeshBuild>(d: &PolyData, b: &mut B, want_edges: bool) -> Result<(), PolyError> {
    let (nf, nc) = (d.n_faces(), d.n_cells());
    // Surviving faces (a face of an alive cell) and the points they use.
    let mut face_new = vec![NONE; nf];
    let mut pt_used = vec![false; d.pts.len()];
    let mut n_faces = 0u32;
    for (f, new) in face_new.iter_mut().enumerate() {
        if d.f_alive[f] {
            *new = n_faces;
            n_faces += 1;
            for &p in d.face_loop(f) {
                pt_used[p as usize] = true;
            }
        }
    }
    let n_cells = d.c_alive.iter().filter(|a| **a).count();
    let mut pt_new = vec![NONE; d.pts.len()];
    let mut n_pts = 0u32;
    for (i, u) in pt_used.iter().enumerate() {
        if *u {
            pt_new[i] = n_pts;
            n_pts += 1;
        }
    }
    b.reserve(n_pts as usize, n_cells);
    for (i, _) in pt_used.iter().enumerate().filter(|(_, u)| **u) {
        let p = d.pts[i];
        let v = b.add_vertex(Point3::new(p[0], p[1], p[2]))?;
        b.set_class(MeshEntityRef::Vertex(v), d.pcls[i])?;
    }
    let mut explicit: BTreeMap<(u32, u32), Classification> = BTreeMap::new();
    let mut forced: Option<(u32, u32, Classification)> = None;
    let mut lp: SmallVec<[VertexIx; 32]> = SmallVec::new();
    for f in 0..nf {
        if !d.f_alive[f] {
            continue;
        }
        lp.clear();
        lp.extend(d.face_loop(f).iter().map(|&p| VertexIx(pt_new[p as usize])));
        let fi = b.add_face(&lp)?;
        match d.f_cls[f] {
            Some(cls) => {
                b.set_class(MeshEntityRef::Face(fi), Classification::on(cls))?;
                if want_edges {
                    collect_edge_classes(d, d.face_loop(f), &pt_new, &mut explicit);
                }
            }
            None if want_edges && forced.is_none() => {
                // Segments with a tet centroid end lie on edge faces only (volume interior).
                let l = d.face_loop(f);
                let n = l.len();
                if let Some(i) = (0..n).find(|&i| l[i] < d.n_tet_pts || l[(i + 1) % n] < d.n_tet_pts) {
                    let region = d.regions[d.c_region[d.f_cells[f][0] as usize] as usize];
                    forced = Some((pt_new[l[i] as usize], pt_new[l[(i + 1) % n] as usize], Classification::on(region)));
                }
            }
            None => {}
        }
    }
    let mut sfs: Vec<SignedFace> = Vec::new();
    for c in 0..nc {
        if !d.c_alive[c] {
            continue;
        }
        sfs.clear();
        sfs.extend(d.cell_faces(c).iter().map(|sf| SignedFace::new(FaceIx(face_new[sf.face().get()]), sf.side())));
        let cell = b.add_cell(&sfs, None)?;
        b.set_class(MeshEntityRef::Cell(cell), Classification::on(d.regions[d.c_region[c] as usize]))?;
    }
    if want_edges {
        if explicit.is_empty()
            && let Some((x, y, c)) = forced
        {
            explicit.insert((x.min(y), x.max(y)), c);
        }
        for (&(x, y), &c) in &explicit {
            b.set_class(MeshEntityRef::Edge(VertexIx(x), VertexIx(y)), c)?;
        }
    }
    Ok(())
}

/// Pairs of adjacent loop vertices that both lie on a model vertex or edge: the dual edge takes
/// the class of its dimension-1 end.
fn collect_edge_classes(d: &PolyData, lp: &[u32], pt_new: &[u32], out: &mut BTreeMap<(u32, u32), Classification>) {
    let n = lp.len();
    for i in 0..n {
        let (x, y) = (lp[i] as usize, lp[(i + 1) % n] as usize);
        let (cx, cy) = (d.pcls[x], d.pcls[y]);
        let (dx, dy) = (cx.dim().unwrap_or(9), cy.dim().unwrap_or(9));
        if dx > 1 || dy > 1 {
            continue;
        }
        let c = if dy == 1 {
            cy
        } else if dx == 1 {
            cx
        } else {
            continue;
        };
        let (a, b) = (pt_new[x], pt_new[y]);
        out.entry((a.min(b), a.max(b))).or_insert(Classification::on(c.entity));
    }
}

/// Summary figures of the built data (before agglomeration).
pub(crate) fn fill_report(rep: &mut DualReport, counts: &Counts, prep: &Prep<'_>) {
    rep.edge_faces = prep.s_edge.len();
    rep.boundary_faces = counts.boundary_faces;
    rep.hub_split_faces = counts.hub_split_faces;
    rep.hub_triangles = counts.hub_triangles;
    rep.split_fans = prep.split_fans;
    rep.split_cells = counts.split_cells;
    rep.wedges = counts.wedges;
}
