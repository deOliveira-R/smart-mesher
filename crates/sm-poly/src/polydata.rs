//! The dual as plain data (points, faces, cells), built from the analysis, optionally
//! agglomerated, and committed to a [`MeshBuild`] in one serial pass.

use crate::error::PolyError;
use crate::geom::V3;
use crate::options::DualReport;
use crate::prep::{NONE, Prep, canonical_cycle};
use sm_core::{EntityId, Point3};
use sm_mesh_api::{Classification, FaceIx, MeshBuild, MeshEntityRef, Side, SignedFace, VertexIx};
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
}

/// Number the used points and record faces and cells in `(region, vertex)` order.
pub(crate) fn build(prep: &Prep<'_>) -> (PolyData, Counts) {
    let (nt, nc, ns) = (prep.nt, prep.ckeys.len(), prep.s_edge.len());
    let mut used = vec![false; prep.n_virtual()];
    used[..nt].fill(true);
    let mut hub_reg = vec![NONE; prep.ne];
    for s in 0..ns {
        let m = &prep.meta[s];
        let e = prep.s_edge[s];
        if m.open {
            used[prep.p_tri(m.f_in) as usize] = true;
            used[prep.p_tri(m.f_out) as usize] = true;
            used[prep.p_mid(e) as usize] = true;
        } else if m.hub {
            used[prep.p_hub(e) as usize] = true;
            hub_reg[e as usize] = prep.s_reg[s];
        }
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
        c_alive: vec![true; nc],
        regions: prep.regions.clone(),
        n_tet_pts: nt as u32,
    };
    for (vid, _) in used.iter().enumerate().filter(|(_, u)| **u) {
        d.pts.push(prep.coord(vid as u32));
        d.pcls.push(point_class(prep, vid as u32, &hub_reg));
    }
    let nm = |vid: u32| num[vid as usize];

    // Cell -> slots (edge faces) in ascending slot order.
    let mut cs_off = vec![0u32; nc + 1];
    for s in 0..ns {
        cs_off[prep.s_ca[s] as usize + 1] += 1;
        cs_off[prep.s_cb[s] as usize + 1] += 1;
    }
    for c in 0..nc {
        cs_off[c + 1] += cs_off[c];
    }
    let mut cs = vec![0u32; cs_off[nc] as usize];
    let mut fill = cs_off.clone();
    for s in 0..ns {
        for c in [prep.s_ca[s], prep.s_cb[s]] {
            cs[fill[c as usize] as usize] = s as u32;
            fill[c as usize] += 1;
        }
    }

    let mut counts = Counts::default();
    let mut slot_face: Vec<(u32, u32)> = vec![(NONE, 0); ns];
    // Interface polygons by canonical point cycle -> (face, runs canonical).
    let mut shared: BTreeMap<SmallVec<[u32; 32]>, (u32, bool)> = BTreeMap::new();
    let (mut fan_i, mut pt_i) = (0usize, 0usize);
    let mut lp: SmallVec<[u32; 32]> = SmallVec::new();
    let mut cell_faces: Vec<SignedFace> = Vec::new();
    for cell in 0..nc {
        cell_faces.clear();
        for &s in &cs[cs_off[cell] as usize..cs_off[cell + 1] as usize] {
            let s = s as usize;
            let lower = prep.s_ca[s] as usize == cell;
            if lower {
                let first = d.n_faces() as u32;
                let count = emit_slot(prep, s, &nm, &mut d, &mut lp, &mut counts);
                slot_face[s] = (first, count);
            }
            let (first, count) = slot_face[s];
            let side = if lower { Side::Left } else { Side::Right };
            cell_faces.extend((0..count).map(|i| SignedFace::new(FaceIx(first + i), side)));
        }
        while fan_i < prep.fan_metas.len() && prep.fan_metas[fan_i].cell as usize == cell {
            let fm = prep.fan_metas[fan_i];
            let pts = &prep.fan_pts[pt_i..pt_i + fm.len as usize];
            fan_i += 1;
            pt_i += fm.len as usize;
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
            *slot = cell as u32;
        }
        d.c_faces.extend_from_slice(&cell_faces);
        d.c_off.push(d.c_faces.len() as u32);
        d.c_region.push((prep.ckeys[cell] >> 32) as u32);
    }
    (d, counts)
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
        return Classification::on(src.face_class(FaceIx(i)).entity);
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
fn emit_slot(
    p: &Prep<'_>,
    s: usize,
    nm: &dyn Fn(u32) -> u32,
    d: &mut PolyData,
    lp: &mut SmallVec<[u32; 32]>,
    counts: &mut Counts,
) -> u32 {
    let m = p.meta[s];
    let e = p.s_edge[s];
    let ring = &p.ring[p.s_off[s] as usize..p.s_off[s + 1] as usize];
    let mut path: SmallVec<[u32; 32]> = SmallVec::new();
    if m.open {
        path.push(nm(p.p_tri(m.f_in)));
    }
    path.extend(ring.iter().map(|&t| nm(t)));
    if m.open {
        path.push(nm(p.p_tri(m.f_out)));
    }
    if !m.hub {
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
}
