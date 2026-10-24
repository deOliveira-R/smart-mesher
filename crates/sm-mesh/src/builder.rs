//! The incremental builder: `MeshBuild` over flat CSR columns with a lazily built face index.

use crate::class::ClassCol;
use crate::util::{FxBuild, nil, range, set_hash};
use indexmap::IndexMap;
use sm_core::Point3;
use sm_mesh_api::{
    Basis, CellAdded, CellIx, CellShape, Classification, FaceIx, FaceKind, Finished, LoopRelation,
    MAX_FACES, MeshBuild, MeshDim, MeshEntityKind, MeshEntityRef, MeshError, NodeRef, Result,
    SerialAlloc, SetMembers, Side, SignedFace, TagValues, VertexIx, edges::edge_key, face_node_count,
    loop_relation, node_count,
};
use smallvec::SmallVec;
use std::collections::{BTreeMap, HashMap};
use std::sync::{Mutex, MutexGuard, PoisonError};

/// Open-addressing index from vertex-set hash to faces, filled lazily from the face columns.
/// A slot is `tag << 32 | (face + 1)`; equal tags probe in face-creation order.
#[derive(Default)]
struct FaceIndex {
    slots: Vec<u64>,
    upto: usize,
    used: usize,
}

impl FaceIndex {
    fn insert(&mut self, tag: u32, face: u32) {
        if (self.used + 1) * 2 > self.slots.len() {
            self.grow();
        }
        let mask = self.slots.len() - 1;
        let mut i = tag as usize & mask;
        while self.slots[i] != 0 {
            i = (i + 1) & mask;
        }
        self.slots[i] = (u64::from(tag) << 32) | u64::from(face + 1);
        self.used += 1;
    }
    fn grow(&mut self) {
        let cap = (self.slots.len() * 2).max(64);
        let old = std::mem::replace(&mut self.slots, vec![0; cap]);
        let mask = cap - 1;
        for s in old.into_iter().filter(|s| *s != 0) {
            let mut i = (s >> 32) as usize & mask;
            while self.slots[i] != 0 {
                i = (i + 1) & mask;
            }
            self.slots[i] = s;
        }
    }
}

/// Spatial hash for `find_vertex`, built lazily for the tolerance of the last query. Only used
/// for lookups: the answer is the lowest matching index, whatever the table order.
#[derive(Default)]
struct Grid {
    tol: f64,
    upto: usize,
    bins: HashMap<[i64; 3], SmallVec<[u32; 4]>, FxBuild>,
    exact: HashMap<[u64; 3], u32, FxBuild>,
}

fn quant(c: f64, tol: f64) -> i64 {
    (c / tol).floor() as i64
}

fn norm_bits(c: f64) -> u64 {
    (if c == 0.0 { 0.0 } else { c }).to_bits()
}

pub(crate) fn check_class(c: &Classification) -> Result<()> {
    if c.dim().is_none() {
        return Err(MeshError::BadClassification("must name a model or partition entity".into()));
    }
    if !c.param.fits(c.entity.kind()) {
        return Err(MeshError::BadClassification(format!(
            "parameter {:?} does not fit {:?}",
            c.param,
            c.entity.kind()
        )));
    }
    Ok(())
}

/// How much checking `finish` does after building the mesh.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Validation {
    /// Check, in parallel, the rules that the builder input can break (classification kinds and
    /// dimension order, open cells, lineage) and run the full contract validator only when one
    /// fails, to produce its report. Everything else holds by construction. The default.
    #[default]
    Fast,
    /// Always run the full sequential `sm_mesh_api::validate` (what the reference builder does).
    Full,
}

/// Cells built from their shape template: corners and faces agree by construction.
pub(crate) const FLAG_TEMPLATE: u8 = 1;

/// Builder producing a [`crate::Mesh`] (face-first CSR columns, parallel `finish`).
///
/// Same contract as the reference builder (`sm_mesh_api::reference::ReferenceBuilder`): the
/// finished mesh has canonical face order `(left, right, loop)`, the lower cell on the left, loops
/// starting at their smallest vertex, ids from the allocator in dense order, and passes
/// [`sm_mesh_api::validate`]. [`MeshBuilder::new`] builds dimension-3 meshes,
/// [`MeshBuilder::surface`] dimension-2 meshes. For large template meshes,
/// [`MeshBuilder::add_cells_bulk`] builds the face table in parallel.
#[derive(Default)]
pub struct MeshBuilder {
    pub(crate) dim: MeshDim,
    pub(crate) x: Vec<f64>,
    pub(crate) y: Vec<f64>,
    pub(crate) z: Vec<f64>,
    pub(crate) extra: Vec<Point3>,
    pub(crate) vclass: ClassCol,
    pub(crate) vcell: Vec<u32>,
    pub(crate) face_off: Vec<u64>,
    pub(crate) face_verts: Vec<VertexIx>,
    pub(crate) face_sides: Vec<u8>,
    pub(crate) face_cells: Vec<[u32; 2]>,
    pub(crate) fclass: ClassCol,
    pub(crate) fkind: BTreeMap<u32, FaceKind>,
    pub(crate) fcurve: BTreeMap<u32, (u8, Basis, Vec<NodeRef>)>,
    pub(crate) cell_off: Vec<u64>,
    pub(crate) cell_sf: Vec<SignedFace>,
    pub(crate) corner_off: Vec<u64>,
    pub(crate) corner_v: Vec<VertexIx>,
    pub(crate) cell_flag: Vec<u8>,
    pub(crate) cclass: ClassCol,
    pub(crate) parent: Vec<u32>,
    pub(crate) active: Vec<bool>,
    pub(crate) ho: BTreeMap<u32, (u8, Basis, Vec<NodeRef>)>,
    pub(crate) eclass: BTreeMap<[VertexIx; 2], Classification>,
    pub(crate) sets: IndexMap<String, SetMembers>,
    pub(crate) set_labels: BTreeMap<String, i64>,
    pub(crate) tags: IndexMap<(MeshEntityKind, String), TagValues>,
    pub(crate) validation: Validation,
    face_index: Mutex<FaceIndex>,
    grid: Mutex<Grid>,
}

impl MeshBuilder {
    /// Start an empty dimension-3 mesh. The mesh id is drawn from the allocator in `finish`.
    pub fn new() -> Self {
        Self::with_dim(MeshDim::Three)
    }

    /// Start an empty dimension-2 (surface) mesh.
    pub fn surface() -> Self {
        Self::with_dim(MeshDim::Two)
    }

    /// Start an empty mesh of dimension `dim`.
    pub fn with_dim(dim: MeshDim) -> Self {
        Self { dim, face_off: vec![0], cell_off: vec![0], corner_off: vec![0], ..Self::default() }
    }

    /// Choose how thoroughly `finish` validates (default [`Validation::Fast`]).
    pub fn with_validation(mut self, v: Validation) -> Self {
        self.validation = v;
        self
    }

    pub(crate) fn nfaces(&self) -> usize {
        self.face_off.len().saturating_sub(1)
    }

    pub(crate) fn ncells(&self) -> usize {
        self.cell_off.len().saturating_sub(1)
    }

    pub(crate) fn loop_of(&self, f: usize) -> &[VertexIx] {
        &self.face_verts[range(&self.face_off, f)]
    }

    pub(crate) fn corners_of(&self, c: usize) -> Option<&[VertexIx]> {
        let s = &self.corner_v[range(&self.corner_off, c)];
        (!s.is_empty()).then_some(s)
    }

    /// Forget the lazily built face index (after the face table was replaced wholesale).
    pub(crate) fn reset_face_index(&self) {
        *self.face_index.lock().unwrap_or_else(PoisonError::into_inner) = FaceIndex::default();
    }

    fn tag_of(l: &[VertexIx]) -> u32 {
        (set_hash(l.iter().map(|v| v.0)) >> 32) as u32
    }

    /// The face index, brought up to the current face count. Uncontended in practice (builders
    /// are driven from one thread); the lock only keeps the builder `Sync` for `finish`.
    fn synced_index(&self) -> MutexGuard<'_, FaceIndex> {
        let mut ix = self.face_index.lock().unwrap_or_else(PoisonError::into_inner);
        let nf = self.nfaces();
        for f in ix.upto..nf {
            let tag = Self::tag_of(self.loop_of(f));
            ix.insert(tag, f as u32);
        }
        ix.upto = nf;
        ix
    }

    /// Faces with the same vertex set as `l`, in creation order, with their loop relation.
    fn candidates<R>(
        &self,
        l: &[VertexIx],
        mut visit: impl FnMut(FaceIx, LoopRelation) -> Option<R>,
    ) -> Option<R> {
        let ix = self.synced_index();
        if ix.slots.is_empty() {
            return None;
        }
        let tag = Self::tag_of(l);
        let mask = ix.slots.len() - 1;
        let mut i = tag as usize & mask;
        loop {
            let s = ix.slots[i];
            if s == 0 {
                return None;
            }
            if (s >> 32) as u32 == tag {
                let f = (s as u32 - 1) as usize;
                if let Some(rel) = loop_relation(l, self.loop_of(f))
                    && let Some(r) = visit(FaceIx::new(f), rel)
                {
                    return Some(r);
                }
            }
            i = (i + 1) & mask;
        }
    }

    /// Shape of a builder cell computed from its current loops (cold path).
    fn shape_of(&self, c: usize) -> CellShape {
        let mut vs: Vec<VertexIx> = Vec::new();
        let mut sizes = Vec::new();
        for sf in &self.cell_sf[range(&self.cell_off, c)] {
            let l = self.loop_of(sf.face().get());
            sizes.push(l.len());
            vs.extend_from_slice(l);
        }
        vs.sort_unstable();
        vs.dedup();
        CellShape::detect(vs.len(), &sizes)
    }

    fn check_loop(&self, l: &[VertexIx]) -> Result<()> {
        let distinct = if l.len() <= 8 {
            (0..l.len()).all(|i| (i + 1..l.len()).all(|j| l[i] != l[j]))
        } else {
            let mut k: SmallVec<[u32; 16]> = l.iter().map(|v| v.0).collect();
            k.sort_unstable();
            k.windows(2).all(|w| w[0] != w[1])
        };
        if !self.dim.facet_len_ok(l.len()) || !distinct {
            return Err(MeshError::InvalidLoop(match self.dim {
                MeshDim::Three => "needs at least 3 distinct vertices".into(),
                MeshDim::Two => "a dimension-2 facet needs exactly 2 distinct vertices".into(),
            }));
        }
        if let Some(v) = l.iter().find(|v| v.get() >= self.x.len()) {
            return Err(MeshError::UnknownVertex(*v));
        }
        Ok(())
    }

    /// `add_cell` with an explicit template flag.
    fn add_cell_flagged(&mut self, faces: &[SignedFace], corners: Option<&[VertexIx]>, flag: u8) -> Result<CellIx> {
        // Phase 1: plan (no mutation).
        let c = CellIx::new(self.ncells());
        if self.ncells() >= u32::MAX as usize - 1 {
            return Err(MeshError::TooLarge);
        }
        if faces.len() < self.dim.min_cell_facets() {
            return Err(MeshError::State(format!(
                "a cell needs at least {} faces",
                self.dim.min_cell_facets()
            )));
        }
        let mut seen: SmallVec<[u32; 8]> = faces.iter().map(|sf| sf.face().0).collect();
        seen.sort_unstable();
        if seen.windows(2).any(|w| w[0] == w[1]) {
            return Err(MeshError::RepeatedCellFace(c));
        }
        for sf in faces {
            let f = sf.face();
            if f.get() >= self.nfaces() {
                return Err(MeshError::OutOfRange(format!("face {f:?}")));
            }
            let bit = if sf.side() == Side::Left { 1 } else { 2 };
            match self.face_sides[f.get()] {
                3 => return Err(MeshError::NonManifoldFace(f)),
                s if s & bit != 0 => return Err(MeshError::InconsistentOrientation(f)),
                _ => {}
            }
        }
        if let Some(ov) = corners {
            let mut s: SmallVec<[u32; 8]> = ov.iter().map(|v| v.0).collect();
            s.sort_unstable();
            if CellShape::from_corner_count_in(self.dim, ov.len()).is_none()
                || s.windows(2).any(|w| w[0] == w[1])
                || ov.iter().any(|v| v.get() >= self.x.len())
            {
                return Err(MeshError::State(
                    "corners are not distinct vertices of a template shape".into(),
                ));
            }
        }
        // Phase 2: commit.
        for sf in faces {
            let f = sf.face().get();
            self.face_sides[f] |= if sf.side() == Side::Left { 1 } else { 2 };
            self.face_cells[f][usize::from(sf.side() == Side::Right)] = c.0;
            for v in &self.face_verts[range(&self.face_off, f)] {
                if self.vcell[v.get()] == CellIx::NONE {
                    self.vcell[v.get()] = c.0;
                }
            }
        }
        self.cell_sf.extend_from_slice(faces);
        self.cell_off.push(self.cell_sf.len() as u64);
        if let Some(ov) = corners {
            self.corner_v.extend_from_slice(ov);
        }
        self.corner_off.push(self.corner_v.len() as u64);
        self.cell_flag.push(flag);
        self.cclass.ent.push(nil());
        self.parent.push(CellIx::NONE);
        self.active.push(true);
        Ok(c)
    }

    /// `add_polyhedron_with_corners` with an explicit template flag.
    fn add_poly_flagged(
        &mut self,
        outward: &[&[VertexIx]],
        corners: Option<&[VertexIx]>,
        flag: u8,
    ) -> Result<CellAdded> {
        let mark = self.nfaces();
        let mut faces: SmallVec<[SignedFace; 6]> = SmallVec::new();
        let mut reused: SmallVec<[bool; 6]> = SmallVec::new();
        for l in outward {
            match self.find_face(l) {
                Some((f, rel)) => {
                    faces.push(SignedFace::new(
                        f,
                        if rel == LoopRelation::Same { Side::Left } else { Side::Right },
                    ));
                    reused.push(true);
                }
                None => match self.add_face(l) {
                    Ok(f) => {
                        faces.push(SignedFace::new(f, Side::Left));
                        reused.push(false);
                    }
                    Err(e) => {
                        let _ = self.truncate_faces(mark);
                        return Err(e);
                    }
                },
            }
        }
        match self.add_cell_flagged(&faces, corners, flag) {
            Ok(cell) => Ok(CellAdded::new(cell, faces, reused)),
            Err(e) => {
                let _ = self.truncate_faces(mark);
                Err(e)
            }
        }
    }
}

impl MeshBuild for MeshBuilder {
    type Output = crate::Mesh;

    fn dim(&self) -> MeshDim {
        self.dim
    }
    fn add_vertex(&mut self, p: Point3) -> Result<VertexIx> {
        if self.x.len() + self.extra.len() >= u32::MAX as usize - 1 {
            return Err(MeshError::TooLarge);
        }
        self.x.push(p.x);
        self.y.push(p.y);
        self.z.push(p.z);
        self.vclass.ent.push(nil());
        self.vcell.push(CellIx::NONE);
        Ok(VertexIx::new(self.x.len() - 1))
    }
    fn add_extra_node(&mut self, p: Point3) -> Result<NodeRef> {
        if self.x.len() + self.extra.len() >= u32::MAX as usize - 1 {
            return Err(MeshError::TooLarge);
        }
        self.extra.push(p);
        Ok(NodeRef::Extra(self.extra.len() as u32 - 1))
    }
    fn vertex_count(&self) -> usize {
        self.x.len()
    }
    fn face_count(&self) -> usize {
        self.nfaces()
    }
    fn cell_count(&self) -> usize {
        self.ncells()
    }
    fn vertex_point(&self, v: VertexIx) -> Point3 {
        Point3::new(self.x[v.get()], self.y[v.get()], self.z[v.get()])
    }
    fn find_vertex(&self, p: Point3, tol: f64) -> Option<VertexIx> {
        let mut g = self.grid.lock().unwrap_or_else(PoisonError::into_inner);
        let exact = !(tol > 0.0 && tol.is_finite());
        let tol = if exact { 0.0 } else { tol };
        if g.tol != tol {
            *g = Grid { tol, ..Grid::default() };
        }
        let from = g.upto;
        for i in from..self.x.len() {
            let (qx, qy, qz) = (self.x[i], self.y[i], self.z[i]);
            if exact {
                g.exact.entry([norm_bits(qx), norm_bits(qy), norm_bits(qz)]).or_insert(i as u32);
            } else {
                g.bins
                    .entry([quant(qx, tol), quant(qy, tol), quant(qz, tol)])
                    .or_default()
                    .push(i as u32);
            }
        }
        g.upto = self.x.len();
        if exact {
            return g
                .exact
                .get(&[norm_bits(p.x), norm_bits(p.y), norm_bits(p.z)])
                .map(|&i| VertexIx(i));
        }
        let (kx, ky, kz) = (quant(p.x, tol), quant(p.y, tol), quant(p.z, tol));
        let mut best: Option<u32> = None;
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(bin) = g.bins.get(&[
                        kx.saturating_add(dx),
                        ky.saturating_add(dy),
                        kz.saturating_add(dz),
                    ]) else {
                        continue;
                    };
                    for &i in bin {
                        let i = i as usize;
                        if (self.x[i] - p.x).abs() <= tol
                            && (self.y[i] - p.y).abs() <= tol
                            && (self.z[i] - p.z).abs() <= tol
                            && best.is_none_or(|b| (i as u32) < b)
                        {
                            best = Some(i as u32);
                        }
                    }
                }
            }
        }
        best.map(VertexIx)
    }

    fn add_face(&mut self, l: &[VertexIx]) -> Result<FaceIx> {
        self.check_loop(l)?;
        if self.nfaces() >= MAX_FACES {
            return Err(MeshError::TooLarge);
        }
        let f = self.nfaces();
        self.face_verts.extend_from_slice(l);
        self.face_off.push(self.face_verts.len() as u64);
        self.fclass.ent.push(nil());
        self.face_sides.push(0);
        self.face_cells.push([CellIx::NONE; 2]);
        Ok(FaceIx::new(f))
    }
    fn find_face(&self, l: &[VertexIx]) -> Option<(FaceIx, LoopRelation)> {
        self.candidates(l, |f, rel| Some((f, rel)))
    }
    fn find_face_where(
        &self,
        l: &[VertexIx],
        accept: &mut dyn FnMut(FaceIx, LoopRelation) -> bool,
    ) -> Option<(FaceIx, LoopRelation)> {
        self.candidates(l, |f, rel| accept(f, rel).then_some((f, rel)))
    }
    fn face_loop(&self, f: FaceIx) -> &[VertexIx] {
        self.loop_of(f.get())
    }
    fn face_degree(&self, f: FaceIx) -> usize {
        self.face_sides[f.get()].count_ones() as usize
    }
    fn truncate_faces(&mut self, len: usize) -> Result<()> {
        if len >= self.nfaces() {
            return Ok(());
        }
        if self.face_sides[len..].iter().any(|&s| s != 0) {
            return Err(MeshError::State("cannot drop faces that have cells".into()));
        }
        {
            let mut ix = self.face_index.lock().unwrap_or_else(PoisonError::into_inner);
            if ix.upto > len {
                *ix = FaceIndex::default();
            }
        }
        self.face_verts.truncate(self.face_off[len] as usize);
        self.face_off.truncate(len + 1);
        self.fclass.ent.truncate(len);
        self.fclass.param.retain(|&f, _| (f as usize) < len);
        self.face_sides.truncate(len);
        self.face_cells.truncate(len);
        self.fkind.retain(|&f, _| (f as usize) < len);
        self.fcurve.retain(|&f, _| (f as usize) < len);
        Ok(())
    }

    fn add_cell(&mut self, faces: &[SignedFace], corners: Option<&[VertexIx]>) -> Result<CellIx> {
        self.add_cell_flagged(faces, corners, 0)
    }
    fn cell_faces(&self, c: CellIx) -> &[SignedFace] {
        &self.cell_sf[range(&self.cell_off, c.get())]
    }
    fn face_cell(&self, f: FaceIx, side: Side) -> Option<CellIx> {
        self.face_cells.get(f.get()).and_then(|c| {
            let x = c[usize::from(side == Side::Right)];
            (x != CellIx::NONE).then_some(CellIx(x))
        })
    }
    fn vertex_cell(&self, v: VertexIx) -> Option<CellIx> {
        self.vcell.get(v.get()).and_then(|&c| (c != CellIx::NONE).then_some(CellIx(c)))
    }
    fn vertex_class(&self, v: VertexIx) -> Option<Classification> {
        (v.get() < self.vclass.len() && self.vclass.ent[v.get()] != nil())
            .then(|| self.vclass.get(v.get()))
    }
    fn face_class(&self, f: FaceIx) -> Option<Classification> {
        (f.get() < self.fclass.len() && self.fclass.ent[f.get()] != nil())
            .then(|| self.fclass.get(f.get()))
    }
    fn cell_class(&self, c: CellIx) -> Option<Classification> {
        (c.get() < self.cclass.len() && self.cclass.ent[c.get()] != nil())
            .then(|| self.cclass.get(c.get()))
    }

    fn set_class(&mut self, e: MeshEntityRef, c: Classification) -> Result<()> {
        check_class(&c)?;
        let oob = || MeshError::OutOfRange(format!("{e:?}"));
        match e {
            MeshEntityRef::Vertex(v) => {
                (v.get() < self.vclass.len()).then_some(()).ok_or_else(oob)?;
                self.vclass.set(v.get(), c);
            }
            MeshEntityRef::Face(f) => {
                (f.get() < self.fclass.len()).then_some(()).ok_or_else(oob)?;
                self.fclass.set(f.get(), c);
            }
            MeshEntityRef::Cell(cell) => {
                if !self.dim.accepts_cell(c.entity.kind()) {
                    return Err(MeshError::BadClassification(format!(
                        "cells of a {:?} mesh cannot be classified on {:?}",
                        self.dim,
                        c.entity.kind()
                    )));
                }
                (cell.get() < self.cclass.len()).then_some(()).ok_or_else(oob)?;
                self.cclass.set(cell.get(), c);
            }
            MeshEntityRef::Edge(..) if self.dim == MeshDim::Two => {
                return Err(MeshError::Unsupported(
                    "a dimension-2 mesh has no edge table: its facets are the edges, classify the facet".into(),
                ));
            }
            MeshEntityRef::Edge(a, b) => {
                if a == b || a.get() >= self.x.len() || b.get() >= self.x.len() {
                    return Err(oob());
                }
                self.eclass.insert(edge_key(a, b), c);
            }
            _ => return Err(MeshError::Unsupported("unknown entity reference".into())),
        }
        Ok(())
    }
    fn set_face_kind(&mut self, f: FaceIx, kind: FaceKind) -> Result<()> {
        if f.get() >= self.nfaces() {
            return Err(MeshError::OutOfRange(format!("{f:?}")));
        }
        if kind == FaceKind::Interior {
            return Err(MeshError::State(
                "Interior is derived from having two cells and cannot be set".into(),
            ));
        }
        if kind == FaceKind::Boundary {
            self.fkind.remove(&f.0); // back to the derived default
        } else {
            self.fkind.insert(f.0, kind);
        }
        Ok(())
    }
    fn set_cell_parent(&mut self, c: CellIx, parent: CellIx) -> Result<()> {
        if c.get() >= self.ncells() || parent >= c {
            return Err(MeshError::OutOfRange(
                "parent must exist and have a lower index than its child".into(),
            ));
        }
        self.parent[c.get()] = parent.0;
        Ok(())
    }
    fn set_cell_active(&mut self, c: CellIx, active: bool) -> Result<()> {
        *self
            .active
            .get_mut(c.get())
            .ok_or_else(|| MeshError::OutOfRange(format!("{c:?}")))? = active;
        Ok(())
    }
    fn set_high_order(&mut self, c: CellIx, degree: u8, basis: Basis, nodes: &[NodeRef]) -> Result<()> {
        if c.get() >= self.ncells() {
            return Err(MeshError::OutOfRange(format!("{c:?}")));
        }
        let shape = self.shape_of(c.get());
        if shape == CellShape::Polyhedron {
            return Err(MeshError::Unsupported(
                "polyhedra cannot carry a high-order record; curve their faces instead".into(),
            ));
        }
        let Some(corners) = self.corners_of(c.get()) else {
            return Err(MeshError::Unsupported(
                "high order needs template corners (use add_cell_from_vertices)".into(),
            ));
        };
        if degree < 2 {
            return Err(MeshError::Unsupported("a high-order record needs degree >= 2".into()));
        }
        let want = node_count(shape, degree, basis).ok_or_else(|| {
            MeshError::Unsupported(format!("high order {shape:?} degree {degree} {basis:?}"))
        })?;
        if nodes.len() != want {
            return Err(MeshError::OutOfRange(format!(
                "{shape:?} degree {degree} {basis:?} needs {want} nodes, got {}",
                nodes.len()
            )));
        }
        if nodes.iter().zip(corners).any(|(n, v)| *n != NodeRef::Vertex(*v)) {
            return Err(MeshError::OutOfRange("the first nodes must be the cell corners".into()));
        }
        if nodes.iter().any(|n| match *n {
            NodeRef::Vertex(v) => v.get() >= self.x.len(),
            NodeRef::Extra(k) => k as usize >= self.extra.len(),
        }) {
            return Err(MeshError::OutOfRange("node does not exist".into()));
        }
        self.ho.insert(c.0, (degree, basis, nodes.to_vec()));
        Ok(())
    }
    fn set_face_curve(&mut self, f: FaceIx, degree: u8, basis: Basis, nodes: &[NodeRef]) -> Result<()> {
        if f.get() >= self.nfaces() {
            return Err(MeshError::OutOfRange(format!("{f:?}")));
        }
        let l = self.loop_of(f.get());
        let want = face_node_count(l.len(), degree, basis).ok_or_else(|| {
            MeshError::Unsupported(format!(
                "face curve with {} vertices, degree {degree}, {basis:?}",
                l.len()
            ))
        })?;
        if nodes.len() != want {
            return Err(MeshError::OutOfRange(format!("expected {want} nodes, got {}", nodes.len())));
        }
        if nodes.iter().zip(l).any(|(n, v)| *n != NodeRef::Vertex(*v)) {
            return Err(MeshError::OutOfRange(
                "the first nodes must be the face loop vertices in loop order".into(),
            ));
        }
        if nodes.iter().any(|n| matches!(*n, NodeRef::Extra(k) if k as usize >= self.extra.len())) {
            return Err(MeshError::OutOfRange("node does not exist".into()));
        }
        self.fcurve.insert(f.0, (degree, basis, nodes.to_vec()));
        Ok(())
    }
    fn add_set(&mut self, name: &str, members: SetMembers) -> Result<()> {
        if self.sets.contains_key(name) {
            return Err(MeshError::DuplicateName(name.into()));
        }
        self.sets.insert(name.into(), members);
        Ok(())
    }
    fn set_label(&mut self, name: &str, label: Option<i64>) -> Result<()> {
        if !self.sets.contains_key(name) {
            return Err(MeshError::State(format!("no set '{name}'")));
        }
        match label {
            Some(l) => self.set_labels.insert(name.into(), l),
            None => self.set_labels.remove(name),
        };
        Ok(())
    }
    fn add_tag(&mut self, name: &str, target: MeshEntityKind, values: TagValues) -> Result<()> {
        if target == MeshEntityKind::Edge {
            return Err(MeshError::State(
                "edge tags belong on the finished mesh (MeshEdit::set_tag)".into(),
            ));
        }
        let key = (target, name.to_string());
        if self.tags.contains_key(&key) {
            return Err(MeshError::DuplicateName(name.into()));
        }
        self.tags.insert(key, values);
        Ok(())
    }

    fn finish(self, alloc: &mut SerialAlloc) -> Result<Finished<crate::Mesh>> {
        // Run against a copy so a failed call draws no ids.
        let mut a = alloc.clone();
        let fin = crate::finish::finish(self, &mut a)?;
        *alloc = a;
        Ok(fin)
    }

    fn add_polyhedron_with_corners(
        &mut self,
        outward: &[&[VertexIx]],
        corners: Option<&[VertexIx]>,
    ) -> Result<CellAdded> {
        self.add_poly_flagged(outward, corners, 0)
    }

    fn add_cell_from_vertices(&mut self, shape: CellShape, verts: &[VertexIx]) -> Result<CellAdded> {
        if shape.vertex_count() != Some(verts.len()) {
            return Err(MeshError::State(format!(
                "{shape:?} needs {:?} vertices, got {}",
                shape.vertex_count(),
                verts.len()
            )));
        }
        if shape.dim() != self.dim {
            return Err(MeshError::State(format!(
                "{shape:?} cells do not belong in a {:?} mesh",
                self.dim
            )));
        }
        let lf = shape.local_faces();
        let mut buf = [[VertexIx(0); 4]; 6];
        let mut refs: SmallVec<[&[VertexIx]; 6]> = SmallVec::new();
        // Loops live in `buf`; `refs` borrows it immutably after filling.
        for (k, l) in lf.iter().enumerate() {
            for (j, &i) in l.iter().enumerate() {
                buf[k][j] = verts[i as usize];
            }
        }
        for (k, l) in lf.iter().enumerate() {
            refs.push(&buf[k][..l.len()]);
        }
        self.add_poly_flagged(&refs, Some(verts), FLAG_TEMPLATE)
    }
}
