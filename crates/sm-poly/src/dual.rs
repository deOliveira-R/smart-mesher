//! The barycentric dual: numbering, builder feed and report.

use crate::error::PolyError;
use crate::options::{DualReport, FaceMode, PolyOptions};
use crate::prep::{NONE, Prep};
use crate::resolver::EdgeResolver;
use crate::stats;
use sm_core::Point3;
use sm_mesh_api::{
    Classification, FaceIx, Finished, LoopRelation, MeshBuild, MeshDim, MeshEntityKind, MeshEntityRef,
    MeshError, MeshRead, SerialAlloc, Side, SignedFace, VertexIx,
};
use smallvec::SmallVec;
use std::collections::BTreeMap;

/// Dualize a conformal all-tetrahedron mesh into polyhedral cells.
///
/// One dual cell per (vertex, region) with the dual points and faces described in the crate
/// docs and `DESIGN.md`. `b` must be a fresh dimension-3 builder; `alloc` is unchanged on `Err`.
/// Faces are planar in [`FaceMode::Planar`] (hub triangles and per-triangle quads replace
/// non-planar polygons); [`FaceMode::Keep`] emits one polygon per edge and per fan.
///
/// `edges` maps a boundary edge between two model faces to its `TopoEdge`; a source edge table,
/// when present, is consulted first.
///
/// # Errors
/// [`PolyError::NotTet`], [`PolyError::Dim2`], [`PolyError::UnsupportedFaceKind`],
/// [`PolyError::OpenRing`], [`PolyError::AmbiguousEdge`], [`PolyError::UnsupportedFeature`],
/// [`PolyError::BadClassification`] and [`PolyError::Mesh`] (builder failure).
pub fn dualize<B: MeshBuild>(
    src: &dyn MeshRead,
    opts: &PolyOptions,
    edges: &dyn EdgeResolver,
    mut b: B,
    alloc: &mut SerialAlloc,
) -> Result<(Finished<B::Output>, DualReport), PolyError> {
    if b.dim() != MeshDim::Three || b.vertex_count() != 0 || b.face_count() != 0 || b.cell_count() != 0 {
        return Err(PolyError::Mesh(MeshError::State(
            "dualize needs a fresh dimension-3 builder".into(),
        )));
    }
    let prep = Prep::build(src, opts.faces, edges)?;
    let (nt, nc, ns) = (prep.nt, prep.ckeys.len(), prep.s_edge.len());
    let want_edges = opts.edges.unwrap_or_else(|| src.edges().is_some());

    // Which virtual points are used, and the region of each hub.
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
    b.reserve(npts as usize, nc);
    let mut pcls: Vec<Classification> = Vec::with_capacity(npts as usize);
    for vid in 0..used.len() {
        if !used[vid] {
            continue;
        }
        let cls = point_class(&prep, vid as u32, &hub_reg);
        let p = prep.coord(vid as u32);
        let v = b.add_vertex(Point3::new(p[0], p[1], p[2]))?;
        b.set_class(MeshEntityRef::Vertex(v), cls)?;
        pcls.push(cls);
    }
    let vx = |vid: u32| VertexIx(num[vid as usize]);

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

    let mut slot_face: Vec<(u32, u32)> = vec![(NONE, 0); ns];
    let mut explicit: BTreeMap<(u32, u32), Classification> = BTreeMap::new();
    let mut forced: Option<(u32, u32, Classification)> = None;
    let mut rep = DualReport::default();
    let mut fan_i = 0usize;
    let mut pt_i = 0usize;
    let mut faces: Vec<SignedFace> = Vec::new();
    let mut lp: SmallVec<[VertexIx; 24]> = SmallVec::new();
    for cell in 0..nc {
        faces.clear();
        for &s in &cs[cs_off[cell] as usize..cs_off[cell + 1] as usize] {
            let s = s as usize;
            let lower = prep.s_ca[s] as usize == cell;
            if lower {
                let first = b.face_count() as u32;
                let count = emit_slot(&prep, s, &vx, &mut b, &mut lp, &mut rep, &mut forced, nt as u32)?;
                slot_face[s] = (first, count);
            }
            let (first, count) = slot_face[s];
            let side = if lower { Side::Left } else { Side::Right };
            faces.extend((0..count).map(|i| SignedFace::new(FaceIx(first + i), side)));
        }
        while fan_i < prep.fan_metas.len() && prep.fan_metas[fan_i].cell as usize == cell {
            let fm = prep.fan_metas[fan_i];
            let pts = &prep.fan_pts[pt_i..pt_i + fm.len as usize];
            fan_i += 1;
            pt_i += fm.len as usize;
            lp.clear();
            lp.extend(pts.iter().map(|&p| vx(p)));
            if want_edges {
                collect_edge_classes(&lp, &pcls, &mut explicit);
            }
            let existing = if fm.iface { b.find_face(&lp) } else { None };
            match existing {
                Some((f, rel)) => faces.push(SignedFace::new(
                    f,
                    if rel == LoopRelation::Same { Side::Left } else { Side::Right },
                )),
                None => {
                    let f = b.add_face(&lp)?;
                    b.set_class(MeshEntityRef::Face(f), Classification::on(fm.cls))?;
                    faces.push(SignedFace::new(f, Side::Left));
                    rep.boundary_faces += 1;
                }
            }
        }
        let c = b.add_cell(&faces, None)?;
        let region = prep.regions[(prep.ckeys[cell] >> 32) as usize];
        b.set_class(MeshEntityRef::Cell(c), Classification::on(region))?;
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

    let fin = b.finish(alloc)?;
    rep.cells = nc;
    rep.edge_faces = ns;
    rep.points = npts as usize;
    rep.split_fans = prep.split_fans;
    rep.max_nonplanarity = stats::max_nonplanarity(&fin.mesh);
    let (lo, _hi, sum) = stats::volume_range(&fin.mesh);
    rep.min_cell_volume = lo;
    rep.total_volume = sum;
    debug_assert!(opts.faces == FaceMode::Keep || rep.hub_split_faces == prep.meta.iter().filter(|m| m.hub).count());
    Ok((fin, rep))
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
        return src.classification(MeshEntityKind::Vertex, i);
    }
    let e = (i - nv) as usize;
    Classification::on(p.regions[hub_reg[e] as usize])
}

/// Add the faces of slot `s` (an edge face, or its hub triangles); returns how many.
#[allow(clippy::too_many_arguments)]
fn emit_slot<B: MeshBuild>(
    p: &Prep<'_>,
    s: usize,
    vx: &dyn Fn(u32) -> VertexIx,
    b: &mut B,
    lp: &mut SmallVec<[VertexIx; 24]>,
    rep: &mut DualReport,
    forced: &mut Option<(u32, u32, Classification)>,
    nt: u32,
) -> Result<u32, PolyError> {
    let m = p.meta[s];
    let e = p.s_edge[s];
    let ring = &p.ring[p.s_off[s] as usize..p.s_off[s + 1] as usize];
    let mut path: SmallVec<[VertexIx; 24]> = SmallVec::new();
    if m.open {
        path.push(vx(p.p_tri(m.f_in)));
    }
    path.extend(ring.iter().map(|&t| vx(t)));
    if m.open {
        path.push(vx(p.p_tri(m.f_out)));
    }
    let mut count = 0u32;
    if !m.hub {
        lp.clear();
        lp.extend(path.iter().copied());
        if m.open {
            lp.push(vx(p.p_mid(e)));
        }
        if forced.is_none() {
            let region = p.regions[p.s_reg[s] as usize];
            if let Some(w) = lp.windows(2).find(|w| w[0].0 < nt || w[1].0 < nt) {
                *forced = Some((w[0].0, w[1].0, Classification::on(region)));
            }
        }
        b.add_face(lp)?;
        count = 1;
    } else {
        rep.hub_split_faces += 1;
        let (hub, n) = if m.open {
            (vx(p.p_mid(e)), path.len() - 1)
        } else {
            (vx(p.p_hub(e)), path.len())
        };
        for i in 0..n {
            lp.clear();
            lp.extend([hub, path[i], path[(i + 1) % path.len()]]);
            b.add_face(lp)?;
            count += 1;
        }
        rep.hub_triangles += n;
    }
    Ok(count)
}

/// Pairs of adjacent loop vertices that both lie on a model vertex or edge: the dual edge takes
/// the class of its dimension-1 end.
fn collect_edge_classes(
    lp: &[VertexIx],
    pcls: &[Classification],
    out: &mut BTreeMap<(u32, u32), Classification>,
) {
    let n = lp.len();
    for i in 0..n {
        let (x, y) = (lp[i], lp[(i + 1) % n]);
        let (cx, cy) = (pcls[x.get()], pcls[y.get()]);
        let (dx, dy) = (cx.dim().unwrap_or(9), cy.dim().unwrap_or(9));
        if dx > 1 || dy > 1 {
            continue;
        }
        let c = if dy == 1 { cy } else if dx == 1 { cx } else { continue };
        out.entry((x.0.min(y.0), x.0.max(y.0))).or_insert(Classification::on(c.entity));
    }
}
