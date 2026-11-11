//! Mixed meshes (tets plus hexes, wedges, pyramids, polyhedra): the tet region is dualized, every
//! other cell is kept as a polyhedron, and each transition triangle (a face between a tet and a
//! non-tet cell) is cut into the three dual sub-faces of its vertices.
//!
//! The tets are copied into a tet-only sub-mesh in which the transition triangles are boundary
//! faces, so the all-tet machinery produces the dual cells and, for every transition vertex, one
//! kite `(vertex, mid, centroid, mid)` per triangle. The non-tet cells then take those kites in
//! place of the triangle and gain the edge midpoint as a collinear vertex on every other face
//! that contains a transition edge, so all faces stay shared and conformal.

use crate::error::PolyError;
use crate::polydata::PolyData;
use crate::prep::{NONE, Prep, pair_key};
use crate::resolver::EdgeResolver;
use sm_core::{EntityId, EntityKind};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{
    CellIx, CellShape, Classification, FaceIx, FaceKind, MeshBuild, MeshEntityKind, MeshEntityRef, MeshRead, SerialAlloc, Side, SignedFace,
    VertexIx,
};
use std::collections::{BTreeMap, BTreeSet};

/// The split of a mixed source.
pub(crate) struct Mixed<'a> {
    full: &'a dyn MeshRead,
    /// The tet-only sub-mesh.
    pub sub: sm_mesh::Mesh,
    /// Sub-face of every full face that touches a tet (`NONE` otherwise).
    full_to_sub: Vec<u32>,
    /// Per sub-face: the true class of a transition triangle.
    pub inner: Vec<Option<Classification>>,
    /// Per vertex: on a transition triangle (its fans are emitted as one kite per triangle).
    pub vsplit: Vec<bool>,
}

fn is_tet(m: &dyn MeshRead, c: usize) -> bool {
    m.cell_shape(CellIx::new(c)) == CellShape::Tet
}

/// Split `src` if it holds both tets and other cells; `None` for an all-tet or tet-free mesh
/// (the plain path decides those).
pub(crate) fn split(src: &dyn MeshRead) -> Result<Option<Mixed<'_>>, PolyError> {
    let (nv, nf, nc) = (src.count(MeshEntityKind::Vertex), src.count(MeshEntityKind::Face), src.count(MeshEntityKind::Cell));
    let ntet = (0..nc).filter(|&c| is_tet(src, c)).count();
    if ntet == 0 || ntet == nc || src.dim() == sm_mesh_api::MeshDim::Two {
        return Ok(None);
    }
    if let Some(f) = (0..nf).find(|&f| !matches!(src.face_kind(FaceIx::new(f)), FaceKind::Interior | FaceKind::Boundary)) {
        return Err(PolyError::UnsupportedFaceKind { face: FaceIx::new(f) });
    }
    let co = src.coords();
    let mut b = MeshBuilder::new();
    for v in 0..nv {
        b.add_vertex(sm_core::Point3::new(co.x[v], co.y[v], co.z[v]))?;
        b.set_class(MeshEntityRef::Vertex(VertexIx::new(v)), src.vertex_class(VertexIx::new(v)))?;
    }
    let top = (0..nf)
        .map(|f| src.face_class(FaceIx::new(f)).entity)
        .filter(|e| e.kind() == EntityKind::TopoFace)
        .map(|e| e.serial())
        .max()
        .unwrap_or(0);
    let synthetic = Classification::on(EntityId::new(EntityKind::TopoFace, top + 1).ok_or(PolyError::Mixed("too many model faces"))?);
    // `finish` renumbers faces, so the sub-faces are matched back to the source by vertex set
    let mut prov = vec![NONE; nf];
    let mut trans: BTreeMap<usize, Classification> = BTreeMap::new();
    let mut vsplit = vec![false; nv];
    let mut sub_edges: BTreeSet<u64> = BTreeSet::new();
    #[allow(clippy::needless_range_loop)]
    for f in 0..nf {
        let fi = FaceIx::new(f);
        let l = src.face_left(fi).get();
        let r = src.face_right(fi).map(|r| r.get());
        let (tl, tr) = (is_tet(src, l), r.map(|r| is_tet(src, r)));
        if !tl && tr != Some(true) {
            continue;
        }
        let lp = src.face_vertices(fi);
        let transition = tr.is_some_and(|tr| tr != tl);
        if transition {
            if lp.len() != 3 {
                return Err(PolyError::Mixed("a tet is adjacent to a non-triangular face"));
            }
            for v in lp {
                vsplit[v.get()] = true;
            }
        }
        let sf = b.add_face(lp)?;
        let class = src.face_class(fi);
        // A transition face inside a volume would be an invalid boundary face in the sub-mesh, so
        // it takes one synthetic model face there; `Prep` reads the true class from `inner`.
        let sub_class = if transition && class.dim() != Some(2) { synthetic } else { class };
        b.set_class(MeshEntityRef::Face(sf), sub_class)?;
        prov[f] = sf.0;
        if transition {
            trans.insert(f, class);
        }
        for i in 0..lp.len() {
            sub_edges.insert(pair_key(lp[i].0, lp[(i + 1) % lp.len()].0));
        }
    }
    for c in (0..nc).filter(|&c| is_tet(src, c)) {
        let ci = CellIx::new(c);
        let sfs: Vec<SignedFace> =
            src.cell_faces(ci).iter().map(|sf| SignedFace::new(FaceIx(prov[sf.face().get()]), sf.side())).collect();
        let cell = b.add_cell(&sfs, src.cell_corners(ci))?;
        b.set_class(MeshEntityRef::Cell(cell), src.cell_class(ci))?;
    }
    if let Some(t) = src.edges() {
        for (i, vv) in t.vertices.iter().enumerate() {
            if sub_edges.contains(&pair_key(vv[0].0, vv[1].0)) {
                b.set_class(MeshEntityRef::Edge(vv[0], vv[1]), t.class[i])?;
            }
        }
    }
    let sub = b.finish(&mut SerialAlloc::new())?.mesh;
    let key = |lp: &[VertexIx]| {
        let mut k: Vec<u32> = lp.iter().map(|v| v.0).collect();
        k.sort_unstable();
        k
    };
    let by_verts: BTreeMap<Vec<u32>, usize> =
        (0..nf).filter(|&f| prov[f] != NONE).map(|f| (key(src.face_vertices(FaceIx::new(f))), f)).collect();
    let mut full_to_sub = vec![NONE; nf];
    let mut inner = vec![None; sub.count(MeshEntityKind::Face)];
    for (sf, slot) in inner.iter_mut().enumerate() {
        let f = *by_verts.get(&key(sub.face_vertices(FaceIx::new(sf)))).ok_or(PolyError::Mixed("sub-mesh face lost"))?;
        full_to_sub[f] = sf as u32;
        *slot = trans.get(&f).copied();
    }
    Ok(Some(Mixed { full: src, sub, full_to_sub, inner, vsplit }))
}

impl Mixed<'_> {
    /// Add the non-tet cells to the dual `d` of the sub-mesh; returns how many were added.
    pub(crate) fn stitch(&self, d: &mut PolyData, prep: &Prep<'_>, resolver: &dyn EdgeResolver) -> Result<usize, PolyError> {
        let full = self.full;
        let (nv, nf, nc) =
            (full.count(MeshEntityKind::Vertex), full.count(MeshEntityKind::Face), full.count(MeshEntityKind::Cell));
        let lost = PolyError::Mixed;
        // Point numbers of the transition triangles' centroids and edge midpoints.
        let mut tri_of_pt: BTreeMap<u32, u32> = BTreeMap::new();
        let mut mid_of: BTreeMap<u64, u32> = BTreeMap::new();
        for f in (0..nf).filter(|&f| self.full_to_sub[f] != NONE && self.inner[self.full_to_sub[f] as usize].is_some()) {
            let sf = self.full_to_sub[f];
            let pt = d.vnum[prep.p_tri(sf) as usize];
            if pt == NONE {
                return Err(lost("a transition triangle has no dual point"));
            }
            tri_of_pt.insert(pt, f as u32);
            let lp = full.face_vertices(FaceIx::new(f));
            for i in 0..3 {
                let key = pair_key(lp[i].0, lp[(i + 1) % 3].0);
                let e = prep.ekeys.binary_search(&key).map_err(|_| lost("a transition edge is missing"))?;
                let m = d.vnum[prep.p_mid(e as u32) as usize];
                if m == NONE {
                    return Err(lost("a transition edge has no midpoint"));
                }
                mid_of.insert(key, m);
            }
        }
        // The kites of every transition triangle (faces of the dual with one cell and a class).
        let mut kites: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
        for df in 0..d.n_faces() {
            if d.f_cls[df].is_none() || d.f_cells[df][1] != NONE {
                continue;
            }
            if let Some(&f) = d.face_loop(df).iter().find_map(|p| tri_of_pt.get(p)) {
                kites.entry(f).or_default().push(df as u32);
            }
        }
        if kites.len() != tri_of_pt.len() || kites.values().any(|k| k.len() != 3) {
            return Err(lost("a transition triangle was not cut into three dual sub-faces"));
        }
        // Points of the primal vertices (new ones for vertices only non-tet cells use).
        let mut vpt: Vec<u32> = (0..nv).map(|v| d.vnum[prep.p_vert(v as u32) as usize]).collect();
        let co = full.coords();
        let mut point = |d: &mut PolyData, v: u32| -> u32 {
            if vpt[v as usize] == NONE {
                vpt[v as usize] = d.pts.len() as u32;
                d.pts.push([co.x[v as usize], co.y[v as usize], co.z[v as usize]]);
                d.pcls.push(full.vertex_class(VertexIx(v)));
            }
            vpt[v as usize]
        };
        let base = d.n_cells() as u32;
        let mut cell_new = vec![NONE; nc];
        let mut kept = 0u32;
        for c in (0..nc).filter(|&c| !is_tet(full, c)) {
            cell_new[c] = base + kept;
            kept += 1;
        }
        let edges = full.edges();
        // Model faces meeting at an edge, for edges the source table does not classify: a feature
        // edge between two model faces gets its curve from the resolver (the edge between two
        // model vertices cannot be derived from its end classes).
        let mut meet: BTreeMap<u64, BTreeSet<sm_core::EntityId>> = BTreeMap::new();
        if edges.is_none() {
            for f in 0..nf {
                let fi = FaceIx::new(f);
                let class = full.face_class(fi);
                if class.dim() != Some(2) {
                    continue;
                }
                let lp = full.face_vertices(fi);
                for i in 0..lp.len() {
                    meet.entry(pair_key(lp[i].0, lp[(i + 1) % lp.len()].0)).or_default().insert(class.entity);
                }
            }
        }
        let mut derived: Vec<(u32, u32, Classification)> = Vec::new();
        // Faces between two non-tet cells, or on the boundary of one.
        let mut face_new = vec![NONE; nf];
        for f in (0..nf).filter(|&f| self.full_to_sub[f] == NONE) {
            let fi = FaceIx::new(f);
            let lp = full.face_vertices(fi);
            let n = lp.len();
            let mut out: Vec<u32> = Vec::with_capacity(n + 2);
            for i in 0..n {
                let (a, b) = (lp[i].0, lp[(i + 1) % n].0);
                let pa = point(d, a);
                out.push(pa);
                let mut cls = edges.and_then(|t| t.find(VertexIx(a), VertexIx(b)).map(|e| t.class[e.get()]));
                let mut from_faces = false;
                if cls.is_none()
                    && let Some(ents) = meet.get(&pair_key(a, b))
                    && ents.len() == 2
                    && let (Some(&lo), Some(&hi)) = (ents.first(), ents.last())
                {
                    let ends = [full.vertex_class(VertexIx(a)), full.vertex_class(VertexIx(b))];
                    cls = resolver.resolve(lo, hi, ends).map(Classification::on);
                    from_faces = cls.is_some();
                }
                if let Some(&m) = mid_of.get(&pair_key(a, b)) {
                    out.push(m);
                    if let Some(c) = cls {
                        let pb = point(d, b);
                        let sink = if from_faces { &mut derived } else { &mut d.extra_edges };
                        sink.push((pa, m, c));
                        sink.push((m, pb, c));
                    }
                } else if let Some(c) = cls {
                    let pb = point(d, b);
                    let sink = if from_faces { &mut derived } else { &mut d.extra_edges };
                    sink.push((pa, pb, c));
                }
            }
            let class = full.face_class(fi);
            let cls = class.dim().map(|_| class.entity);
            let nfc = d.push_face(&out, cls);
            let l = cell_new[full.face_left(fi).get()];
            let r = full.face_right(fi).map_or(NONE, |r| cell_new[r.get()]);
            d.f_cells[nfc as usize] = [l, r];
            face_new[f] = nfc;
        }
        d.extra_edges.extend(derived);
        // The cells.
        for c in (0..nc).filter(|&c| !is_tet(full, c)) {
            let ci = CellIx::new(c);
            for sf in full.cell_faces(ci) {
                let f = sf.face().get();
                if self.full_to_sub[f] != NONE {
                    for &df in &kites[&(f as u32)] {
                        d.f_cells[df as usize][1] = cell_new[c];
                        d.c_faces.push(SignedFace::new(FaceIx(df), Side::Right));
                    }
                } else {
                    d.c_faces.push(SignedFace::new(FaceIx(face_new[f]), sf.side()));
                }
            }
            d.c_off.push(d.c_faces.len() as u32);
            let entity = full.cell_class(ci).entity;
            let region = match d.regions.iter().position(|&r| r == entity) {
                Some(r) => r,
                None => {
                    d.regions.push(entity);
                    d.regions.len() - 1
                }
            };
            d.c_region.push(region as u32);
            d.c_alive.push(true);
        }
        Ok(kept as usize)
    }
}
