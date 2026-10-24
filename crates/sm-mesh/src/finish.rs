//! `finish`: orient, canonicalise, validate incidence, assign ids. Every per-entity stage runs
//! in parallel; stage order, tie-breaks and error selection are fixed, so the result and the
//! reported error are independent of the thread count.

use crate::builder::{FLAG_TEMPLATE, MeshBuilder, Validation};
use crate::class::ClassCol;
use crate::store::{IdRange, Mesh};
use crate::util::{CHUNK, ErrSink, chunk_bounds, nil, prefix, range, split_by_offsets};
use rayon::prelude::*;
use sm_core::{EntityId, EntityKind};
use sm_mesh_api::{
    Basis, CellIx, CellShape, FaceIx, FaceKind, Finished, HighOrderCell, LoopRelation,
    MeshDim, MeshEntity, MeshEntityKind, MeshError, NodeRef, PointIx, Result, SerialAlloc, SetMembers,
    Side, SignedFace, TagColumn, TagValues, VertexIx, derive_corners, loop_relation, shape::polygon_cycle,
};
use smallvec::SmallVec;
use std::cmp::Ordering;
use std::collections::BTreeMap;

/// Per-face facts computed from the builder incidence.
#[derive(Clone, Copy, Default)]
struct FaceMeta {
    left: u32,
    right: u32,
    rot: u32,
    reversed: bool,
    kind: FaceKind,
}

/// Sort record: `(left, right)` packed, first final vertex, builder face.
#[derive(Clone, Copy)]
struct Rec {
    key: u64,
    v0: u32,
    f: u32,
}

fn face_meta(b: &MeshBuilder, f: usize) -> Result<FaceMeta> {
    let [cl, cr] = b.face_cells[f];
    let fi = FaceIx::new(f);
    let none = CellIx::NONE;
    let (left, right, reversed) = match (cl != none, cr != none) {
        (false, false) => return Err(MeshError::DanglingFace(fi)),
        (true, true) if cl < cr => (cl, cr, false),
        (true, true) => (cr, cl, true),
        (true, false) => (cl, none, false),
        (false, true) => (cr, none, true),
    };
    let kind = match (right != none, b.fkind.get(&fi.0)) {
        (true, None) => FaceKind::Interior,
        (true, Some(_)) => {
            return Err(MeshError::State(format!(
                "face {fi:?} has two cells and cannot be NonConformal/PartitionCut"
            )));
        }
        (false, k) => k.copied().unwrap_or(FaceKind::Boundary),
    };
    let l = b.loop_of(f);
    let n = l.len();
    let rot = if b.dim == MeshDim::Two {
        0
    } else {
        let p = l.iter().enumerate().min_by_key(|(_, v)| **v).map_or(0, |(p, _)| p);
        if reversed { n - 1 - p } else { p }
    };
    Ok(FaceMeta { left, right, rot: rot as u32, reversed, kind })
}

/// Vertex `i` of the final loop of builder face `f`.
#[inline]
fn final_vertex(l: &[VertexIx], m: &FaceMeta, i: usize) -> VertexIx {
    let n = l.len();
    let mut k = i + m.rot as usize;
    if k >= n {
        k -= n;
    }
    if m.reversed { l[n - 1 - k] } else { l[k] }
}

fn cmp_loops(b: &MeshBuilder, meta: &[FaceMeta], x: u32, y: u32) -> Ordering {
    let (lx, ly) = (b.loop_of(x as usize), b.loop_of(y as usize));
    let (mx, my) = (&meta[x as usize], &meta[y as usize]);
    for i in 0..lx.len().min(ly.len()) {
        let o = final_vertex(lx, mx, i).cmp(&final_vertex(ly, my, i));
        if o != Ordering::Equal {
            return o;
        }
    }
    lx.len().cmp(&ly.len())
}

/// Output of one chunk of cells.
#[derive(Default)]
struct CellChunk {
    /// A cell built from loops whose directed edges do not cancel (not a closed surface).
    open: bool,
    faces: Vec<SignedFace>,
    shapes: Vec<CellShape>,
    corner_len: Vec<u32>,
    corners: Vec<VertexIx>,
}

struct CellCtx<'a> {
    b: &'a MeshBuilder,
    new_of_old: &'a [u32],
    face_off: &'a [u64],
    face_verts: &'a [VertexIx],
    face_left: &'a [CellIx],
}

impl CellCtx<'_> {
    fn chunk(&self, c0: usize, c1: usize) -> std::result::Result<CellChunk, (usize, MeshError)> {
        let b = self.b;
        let mut out = CellChunk::default();
        out.faces.reserve(((b.cell_off[c1] - b.cell_off[c0]) as usize).max(1));
        out.shapes.reserve(c1 - c0);
        out.corner_len.reserve(c1 - c0);
        for c in c0..c1 {
            let cix = CellIx::new(c);
            let mut sfs: SmallVec<[SignedFace; 8]> = b.cell_sf[range(&b.cell_off, c)]
                .iter()
                .map(|sf| {
                    let n = self.new_of_old[sf.face().get()];
                    let side = if self.face_left[n as usize] == cix { Side::Left } else { Side::Right };
                    SignedFace::new(FaceIx(n), side)
                })
                .collect();
            sfs.sort_unstable();
            let given = b.corners_of(c);
            if b.cell_flag[c] & FLAG_TEMPLATE != 0
                && let Some(ov) = given
                && let Some(shape) = CellShape::from_corner_count_in(b.dim, ov.len())
            {
                // Built from the shape template: the corners realise its faces by construction.
                out.faces.extend_from_slice(&sfs);
                out.shapes.push(shape);
                out.corner_len.push(ov.len() as u32);
                out.corners.extend_from_slice(ov);
                continue;
            }
            self.general(&mut out, cix, &sfs, given).map_err(|e| (c, e))?;
        }
        Ok(out)
    }

    /// Shape detection and corner handling for polyhedra, polygons and cells with given corners.
    fn general(
        &self,
        out: &mut CellChunk,
        cix: CellIx,
        sfs: &[SignedFace],
        given: Option<&[VertexIx]>,
    ) -> Result<()> {
        let loops: SmallVec<[SmallVec<[VertexIx; 4]>; 6]> = sfs
            .iter()
            .map(|sf| {
                let l = &self.face_verts[range(self.face_off, sf.face().get())];
                if sf.side() == Side::Left {
                    l.iter().copied().collect()
                } else {
                    l.iter().rev().copied().collect()
                }
            })
            .collect();
        let mut vs: SmallVec<[VertexIx; 32]> = loops.iter().flat_map(|l| l.iter().copied()).collect();
        vs.sort_unstable();
        vs.dedup();
        let sizes: SmallVec<[usize; 6]> = loops.iter().map(SmallVec::len).collect();
        let mut shape = CellShape::detect(vs.len(), &sizes);
        if self.b.dim == MeshDim::Three && !is_closed(&loops) {
            out.open = true;
        }
        let refs: SmallVec<[&[VertexIx]; 6]> = loops.iter().map(SmallVec::as_slice).collect();
        let cs: Option<SmallVec<[VertexIx; 8]>> = if self.b.dim == MeshDim::Two {
            let cyc = polygon_cycle(&refs).ok_or_else(|| {
                MeshError::ShapeMismatch(
                    cix,
                    "the facets of a dimension-2 cell do not form one closed cycle".into(),
                )
            })?;
            match given {
                Some(ov) if loop_relation(ov, &cyc) != Some(LoopRelation::Same) => {
                    return Err(MeshError::ShapeMismatch(
                        cix,
                        "corners are not the cell's vertex cycle in boundary order".into(),
                    ));
                }
                Some(ov) => Some(ov.iter().copied().collect()),
                None => Some(cyc),
            }
        } else {
            match given {
                Some(ov) => {
                    if CellShape::from_corner_count(ov.len()) != Some(shape) {
                        return Err(MeshError::ShapeMismatch(
                            cix,
                            format!("{} corners do not match topology {shape:?}", ov.len()),
                        ));
                    }
                    // Every template face must be one of the cell's outward loops.
                    for lf in shape.local_faces() {
                        let want: SmallVec<[VertexIx; 4]> = lf.iter().map(|&i| ov[i as usize]).collect();
                        if !loops.iter().any(|l| loop_relation(&want, l) == Some(LoopRelation::Same)) {
                            return Err(MeshError::ShapeMismatch(
                                cix,
                                format!("corners do not realise the faces of a {shape:?}"),
                            ));
                        }
                    }
                    Some(ov.iter().copied().collect())
                }
                None if shape != CellShape::Polyhedron => {
                    let d = derive_corners(shape, &refs);
                    if d.is_none() {
                        shape = CellShape::Polyhedron;
                    }
                    d
                }
                None => None,
            }
        };
        out.faces.extend_from_slice(sfs);
        out.shapes.push(shape);
        match cs {
            Some(ov) => {
                out.corner_len.push(ov.len() as u32);
                out.corners.extend_from_slice(&ov);
            }
            None => out.corner_len.push(0),
        }
        Ok(())
    }
}

/// The outward loops of a dimension-3 cell form a closed oriented surface: every directed edge
/// occurs once and its reverse occurs too.
fn is_closed(loops: &[SmallVec<[VertexIx; 4]>]) -> bool {
    let mut dir: SmallVec<[(u32, u32); 32]> = SmallVec::new();
    for l in loops {
        let n = l.len();
        dir.extend((0..n).map(|i| (l[i].0, l[(i + 1) % n].0)));
    }
    dir.sort_unstable();
    dir.windows(2).all(|w| w[0] != w[1]) && dir.iter().all(|&(a, b)| dir.binary_search(&(b, a)).is_ok())
}

fn ids(alloc: &mut SerialAlloc, kind: EntityKind, n: usize) -> Result<IdRange> {
    let first = alloc.take(kind, n)?;
    let first_id = EntityId::new(kind, first).ok_or(MeshError::SerialOverflow)?;
    if n > 0 {
        EntityId::new(kind, first + n as u64 - 1).ok_or(MeshError::SerialOverflow)?;
    }
    Ok(IdRange { first: first_id, n })
}

/// The `validate` rules that builder input can break, taken from the contract's shared
/// functions: classification (kinds, dimension order, reverse index), lineage, and the
/// high-order, face-curve, set and tag records. Everything else `validate` tests (canonical
/// order, orientation, incidence, shape tags, edge table) holds by construction in `finish`.
/// `false` means "suspicious": the caller then runs the full validator, which produces the report.
fn fast_checks(m: &Mesh) -> bool {
    let nf = sm_mesh_api::MeshRead::count(m, MeshEntityKind::Face);
    let loops_ok: Vec<bool> = (0..nf)
        .into_par_iter()
        .map(|f| sm_mesh_api::face_loop_ok(m, FaceIx::new(f)))
        .collect();
    let (a, b) = rayon::join(
        || sm_mesh_api::validate_classification_with(m, &loops_ok).is_empty(),
        || sm_mesh_api::validate_records_with(m, &loops_ok).is_empty(),
    );
    a && b
}

pub(crate) fn finish(b: MeshBuilder, alloc: &mut SerialAlloc) -> Result<Finished<Mesh>> {
    let (nv, nf, nc) = (b.x.len(), b.nfaces(), b.ncells());
    let unset = nil();

    // Classification of vertices and cells is mandatory.
    if let Some(i) = b.vclass.ent.par_iter().position_first(|e| *e == unset) {
        return Err(MeshError::Unclassified(MeshEntity::Vertex(VertexIx::new(i))));
    }
    if let Some(i) = b.cclass.ent.par_iter().position_first(|e| *e == unset) {
        return Err(MeshError::Unclassified(MeshEntity::Cell(CellIx::new(i))));
    }

    // Per-face incidence facts (lowest erroring face is reported).
    let mut meta = vec![FaceMeta::default(); nf];
    let errs = ErrSink::default();
    meta.par_iter_mut().enumerate().for_each(|(f, m)| match face_meta(&b, f) {
        Ok(v) => *m = v,
        Err(e) => errs.push(f, e),
    });
    if let Some(e) = errs.take_first() {
        return Err(e);
    }

    // Canonical order: (left, right-or-none, loop). A strict total order (builder index last).
    let mut recs: Vec<Rec> = (0..nf)
        .into_par_iter()
        .map(|f| {
            let m = &meta[f];
            Rec {
                key: (u64::from(m.left) << 32) | u64::from(m.right),
                v0: final_vertex(b.loop_of(f), m, 0).0,
                f: f as u32,
            }
        })
        .collect();
    recs.par_sort_unstable_by(|x, y| {
        x.key
            .cmp(&y.key)
            .then(x.v0.cmp(&y.v0))
            .then_with(|| cmp_loops(&b, &meta, x.f, y.f))
            .then(x.f.cmp(&y.f))
    });
    if let Some(w) = recs.par_windows(2).position_first(|w| {
        w[0].key == w[1].key && cmp_loops(&b, &meta, w[0].f, w[1].f) == Ordering::Equal
    }) {
        return Err(MeshError::DuplicateFace(FaceIx::new(w)));
    }
    let mut new_of_old = vec![0u32; nf];
    for (n, r) in recs.iter().enumerate() {
        new_of_old[r.f as usize] = n as u32;
    }

    // Face columns.
    let lens: Vec<u32> = recs.par_iter().map(|r| b.loop_of(r.f as usize).len() as u32).collect();
    let face_off = prefix(&lens);
    let mut face_verts = vec![VertexIx(0); b.face_verts.len()];
    {
        let bounds = chunk_bounds(nf, CHUNK);
        let slices = split_by_offsets(&mut face_verts, &face_off, &bounds);
        slices.into_par_iter().zip(bounds.par_windows(2)).for_each(|(s, w)| {
            let base = face_off[w[0]] as usize;
            for n in w[0]..w[1] {
                let r = &recs[n];
                let l = b.loop_of(r.f as usize);
                let m = &meta[r.f as usize];
                let at = face_off[n] as usize - base;
                for i in 0..l.len() {
                    s[at + i] = final_vertex(l, m, i);
                }
            }
        });
    }
    let (face_left, face_right): (Vec<CellIx>, Vec<u32>) =
        recs.par_iter().map(|r| (CellIx((r.key >> 32) as u32), r.key as u32)).unzip();
    let face_kind: Vec<FaceKind> = recs.par_iter().map(|r| meta[r.f as usize].kind).collect();

    // Face classes (derived for unset interior-like faces).
    let mut fent = vec![unset; nf];
    let errs = ErrSink::default();
    fent.par_iter_mut().enumerate().for_each(|(n, slot)| {
        let old = recs[n].f as usize;
        if b.fclass.ent[old] != unset {
            *slot = b.fclass.ent[old];
        } else if face_kind[n] == FaceKind::Boundary {
            errs.push(n, MeshError::Unclassified(MeshEntity::Face(FaceIx::new(n))));
        } else {
            let l = b.cclass.ent[face_left[n].get()];
            if face_right[n] != CellIx::NONE && b.cclass.ent[face_right[n] as usize] != l {
                errs.push(n, MeshError::Unclassified(MeshEntity::Face(FaceIx::new(n))));
            } else {
                *slot = l;
            }
        }
    });
    if let Some(e) = errs.take_first() {
        return Err(e);
    }
    let fclass = ClassCol {
        ent: fent,
        param: b.fclass.param.iter().map(|(old, p)| (new_of_old[*old as usize], *p)).collect(),
    };

    // Cells: shapes, corners (chunked, merged in chunk order; lowest erroring cell reported).
    let cell_bounds = chunk_bounds(nc, CHUNK);
    let ctx = CellCtx { b: &b, new_of_old: &new_of_old, face_off: &face_off, face_verts: &face_verts, face_left: &face_left };
    let chunks: Vec<std::result::Result<CellChunk, (usize, MeshError)>> =
        cell_bounds.par_windows(2).map(|w| ctx.chunk(w[0], w[1])).collect();
    let mut cell_faces = Vec::with_capacity(b.cell_sf.len());
    let mut cell_shape = Vec::with_capacity(nc);
    let mut corner_len: Vec<u32> = Vec::with_capacity(nc);
    let mut corners = Vec::new();
    let mut open_cells = false;
    for ch in chunks {
        let ch = ch.map_err(|(_, e)| e)?;
        open_cells |= ch.open;
        cell_faces.extend_from_slice(&ch.faces);
        cell_shape.extend_from_slice(&ch.shapes);
        corner_len.extend_from_slice(&ch.corner_len);
        corners.extend_from_slice(&ch.corners);
    }
    let corner_off = prefix(&corner_len);

    // Ids.
    let mesh_id = alloc.one(EntityKind::Mesh)?;
    let vids = ids(alloc, EntityKind::MeshVertex, nv)?;
    let fids = ids(alloc, EntityKind::MeshFace, nf)?;
    let cids = ids(alloc, EntityKind::MeshCell, nc)?;

    // Points, high order, face curves.
    let pidx = |n: NodeRef| match n {
        NodeRef::Vertex(v) => PointIx(v.0),
        // Vertex and extra-node counts are capped below u32::MAX by the builder, so this cannot wrap.
        NodeRef::Extra(k) => PointIx(nv as u32 + k),
    };
    let ho: BTreeMap<u32, HighOrderCell> = b
        .ho
        .iter()
        .map(|(c, (degree, basis, nodes))| {
            (*c, HighOrderCell::new(*degree, *basis, nodes.iter().map(|n| pidx(*n)).collect()))
        })
        .collect();
    let mut fcurve: BTreeMap<u32, (u8, Basis, Vec<PointIx>)> = BTreeMap::new();
    for (old, (degree, basis, nodes)) in &b.fcurve {
        let n = b.loop_of(*old as usize).len();
        let nu = new_of_old[*old as usize] as usize;
        let m = &meta[*old as usize];
        // Final loop position j holds the builder-loop position `pos(j)`.
        let pos = |j: usize| -> usize {
            let k = (j + m.rot as usize) % n; // index in the (possibly reversed) builder loop
            if m.reversed { n - 1 - k } else { k }
        };
        let mut out: Vec<PointIx> = Vec::with_capacity(nodes.len());
        for j in 0..n {
            out.push(pidx(nodes[pos(j)]));
        }
        for j in 0..n {
            let (p, q) = (pos(j), pos((j + 1) % n));
            let e = if q == (p + 1) % n { p } else { q };
            out.push(pidx(nodes[n + e]));
        }
        out.extend(nodes[2 * n..].iter().map(|x| pidx(*x)));
        fcurve.insert(nu as u32, (*degree, *basis, out));
    }
    let (mut x, mut y, mut z) = (b.x, b.y, b.z);
    x.extend(b.extra.iter().map(|p| p.x));
    y.extend(b.extra.iter().map(|p| p.y));
    z.extend(b.extra.iter().map(|p| p.z));

    // Sets and tags, in final face numbering.
    let set_ids = ids(alloc, EntityKind::MeshSet, b.sets.len())?;
    let mut sets: Vec<sm_mesh_api::MeshSet> = Vec::new();
    for (k, (name, members)) in b.sets.into_iter().enumerate() {
        let id = set_ids.id(k);
        let oob = |what: &str| MeshError::OutOfRange(format!("set '{name}' {what}"));
        let members = match members {
            SetMembers::Faces(v) => {
                if v.iter().any(|f| f.get() >= nf) {
                    return Err(oob("face"));
                }
                let mut m: Vec<FaceIx> = v.iter().map(|f| FaceIx(new_of_old[f.get()])).collect();
                m.par_sort_unstable();
                m.dedup();
                SetMembers::Faces(m)
            }
            SetMembers::Vertices(mut v) => {
                v.par_sort_unstable();
                v.dedup();
                if v.last().is_some_and(|x| x.get() >= nv) {
                    return Err(oob("vertex"));
                }
                SetMembers::Vertices(v)
            }
            SetMembers::Cells(mut v) => {
                v.par_sort_unstable();
                v.dedup();
                if v.last().is_some_and(|x| x.get() >= nc) {
                    return Err(oob("cell"));
                }
                SetMembers::Cells(v)
            }
            SetMembers::Sides(v) => {
                // Re-side through the cell on that side: `finish` may swap left and right.
                let mut m = Vec::with_capacity(v.len());
                for sf in &v {
                    let pf = sf.face().get();
                    if pf >= nf {
                        return Err(oob("face"));
                    }
                    let c = b.face_cells[pf][usize::from(sf.side() == Side::Right)];
                    if c == CellIx::NONE {
                        return Err(oob("side without a cell"));
                    }
                    let n = new_of_old[pf];
                    let side = if face_left[n as usize] == CellIx(c) { Side::Left } else { Side::Right };
                    m.push(SignedFace::new(FaceIx(n), side));
                }
                m.par_sort_unstable();
                m.dedup();
                SetMembers::Sides(m)
            }
            SetMembers::ShellSides(mut v) => {
                if b.dim != MeshDim::Two {
                    return Err(oob("is a shell side set, valid only in a dimension-2 mesh"));
                }
                v.par_sort_unstable();
                v.dedup();
                if v.last().is_some_and(|x| (x >> 1) as usize >= nc) {
                    return Err(oob("cell"));
                }
                SetMembers::ShellSides(v)
            }
            SetMembers::Points(mut v) => {
                v.par_sort_unstable();
                v.dedup();
                if v.last().is_some_and(|x| x.get() >= nv + b.extra.len()) {
                    return Err(oob("point"));
                }
                SetMembers::Points(v)
            }
            _ => return Err(oob("has an unsupported member kind")),
        };
        let label = b.set_labels.get(&name).copied();
        if label.is_some()
            && sets.iter().any(|s: &sm_mesh_api::MeshSet| s.label == label && s.members.kind() == members.kind())
        {
            return Err(MeshError::DuplicateName(format!("set label {label:?}")));
        }
        let mut s = sm_mesh_api::MeshSet::new(id, name, members);
        s.label = label;
        sets.push(s);
    }
    let mut tags = Vec::new();
    for ((target, name), values) in b.tags {
        let n = match target {
            MeshEntityKind::Vertex => nv,
            MeshEntityKind::Face => nf,
            MeshEntityKind::Cell => nc,
            _ => 0,
        };
        if values.len() != n {
            return Err(MeshError::OutOfRange(format!(
                "tag '{name}' has {} values, expected {n}",
                values.len()
            )));
        }
        let values = if target == MeshEntityKind::Face {
            match values {
                TagValues::I64(v) => TagValues::I64(recs.par_iter().map(|r| v[r.f as usize]).collect()),
                TagValues::F64(v) => TagValues::F64(recs.par_iter().map(|r| v[r.f as usize]).collect()),
                other => other,
            }
        } else {
            values
        };
        tags.push(TagColumn::new(name, target, values));
    }
    tags.sort_by(|a, c| (a.target, &a.name).cmp(&(c.target, &c.name)));

    let mut mesh = Mesh {
        dim: b.dim,
        id: mesh_id,
        vids,
        fids,
        cids,
        x,
        y,
        z,
        face_off,
        face_verts,
        face_left,
        face_right,
        face_kind,
        cell_off: b.cell_off,
        cell_faces,
        cell_shape,
        corner_off,
        corners,
        cell_parent: b.parent,
        cell_active: b.active,
        vclass: b.vclass,
        fclass,
        cclass: b.cclass,
        class_index: BTreeMap::new(),
        class_entities: Vec::new(),
        ho,
        fcurve,
        sets,
        tags,
        edges: None,
        cache: None,
        adj: Default::default(),
    };
    // Edge-slot nodes (dimension 3) need the edge table: derive it even without explicit edge
    // classes (an ambiguous edge fails with `UnclassifiedEdge`; classify it with `set_class`).
    if !b.eclass.is_empty() || mesh.needs_edge_table() {
        let t = crate::edges::derive(&mesh, alloc, &b.eclass)?;
        mesh.edges = Some(t);
    }
    mesh.rebuild_class_index();
    if b.validation == Validation::Full || open_cells || !fast_checks(&mesh) {
        let v = sm_mesh_api::validate(&mesh);
        if !v.is_empty() {
            return Err(MeshError::Invalid(v));
        }
    }
    Ok(Finished::new(mesh, new_of_old.into_iter().map(FaceIx).collect()))
}
