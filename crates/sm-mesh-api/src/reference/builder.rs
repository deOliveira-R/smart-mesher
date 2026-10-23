//! Reference builder: collects vertices, faces and signed cells, then `finish` normalises.

use super::ReferenceMesh;
use crate::build::{Finished, MeshBuild, SerialAlloc};
use crate::classify::Classification;
use crate::edges::{EdgeTable, edge_key};
use crate::error::{MeshError, Result};
use crate::highorder::{Basis, HighOrderCell, NodeRef, face_node_count, node_count};
use crate::index::{
    CellIx, FaceIx, FaceKind, MAX_FACES, MeshDim, MeshEntity, MeshEntityKind, MeshEntityRef,
    PointIx, Side, SignedFace, VertexIx,
};
use crate::sets::{MeshSet, SetMembers, TagColumn, TagValues};
use crate::shape::{CellShape, LoopRelation, derive_corners, loop_relation, polygon_cycle};
use indexmap::IndexMap;
use sm_core::{EntityId, EntityKind, Point3};
use smallvec::SmallVec;
use std::cell::RefCell;
use std::collections::BTreeMap;

/// Columns handed from the builder to [`ReferenceMesh::from_parts`].
pub(crate) struct Parts {
    pub dim: MeshDim,
    pub id: EntityId,
    pub vertex_ids: Vec<EntityId>,
    pub face_ids: Vec<EntityId>,
    pub cell_ids: Vec<EntityId>,
    pub x: Vec<f64>,
    pub y: Vec<f64>,
    pub z: Vec<f64>,
    pub n_vertices: usize,
    pub face_off: Vec<u64>,
    pub face_verts: Vec<VertexIx>,
    pub face_left: Vec<CellIx>,
    pub face_right: Vec<u32>,
    pub face_kind: Vec<FaceKind>,
    pub cell_off: Vec<u64>,
    pub cell_faces: Vec<SignedFace>,
    pub cell_shape: Vec<CellShape>,
    pub corner_off: Vec<u64>,
    pub corners: Vec<VertexIx>,
    pub cell_parent: Vec<u32>,
    pub cell_active: Vec<bool>,
    pub vclass: Vec<Classification>,
    pub fclass: Vec<Classification>,
    pub cclass: Vec<Classification>,
    pub ho: BTreeMap<u32, HighOrderCell>,
    pub fcurve: BTreeMap<u32, (u8, Basis, Vec<PointIx>)>,
    pub sets: Vec<MeshSet>,
    pub tags: Vec<TagColumn>,
}

struct BCell {
    faces: SmallVec<[SignedFace; 6]>,
    corners: Option<SmallVec<[VertexIx; 8]>>,
}

type FaceKey = SmallVec<[u32; 8]>;

/// Spatial hash for `find_vertex`, built lazily for the tolerance of the last query.
#[derive(Default)]
struct Grid {
    tol: f64,
    upto: usize,
    bins: BTreeMap<[i64; 3], SmallVec<[u32; 4]>>,
    exact: BTreeMap<[u64; 3], u32>,
}

fn quant(c: f64, tol: f64) -> i64 {
    (c / tol).floor() as i64
}

fn norm_bits(c: f64) -> u64 {
    (if c == 0.0 { 0.0 } else { c }).to_bits()
}

/// Builder producing a [`ReferenceMesh`].
///
/// `finish` guarantees: every face has one or two cells; the lower cell is `left` and sees the
/// loop normal as outward (loops are reversed where needed); faces are sorted by
/// `(left, right, loop)` with loops starting at their smallest vertex; ids are assigned from the
/// allocator in dense order; template-shaped cells have corner lists; the result passes
/// [`crate::validate::validate`].
///
/// [`ReferenceBuilder::new`] builds dimension-3 meshes, [`ReferenceBuilder::surface`]
/// dimension-2 meshes (polygon cells, two-vertex facets; facet loops are directed and never
/// rotated, the lower cell traverses its facet forward).
#[derive(Default)]
pub struct ReferenceBuilder {
    dim: MeshDim,
    pts: Vec<Point3>,
    extra: Vec<Point3>,
    vclass: Vec<Option<Classification>>,
    vcell: Vec<u32>,
    face_off: Vec<u64>,
    face_verts: Vec<VertexIx>,
    face_key: BTreeMap<FaceKey, SmallVec<[u32; 1]>>,
    face_sides: Vec<u8>,
    face_deg: Vec<usize>,
    face_cells: Vec<[u32; 2]>,
    fclass: Vec<Option<Classification>>,
    fkind: BTreeMap<u32, FaceKind>,
    fcurve: BTreeMap<u32, (u8, Basis, Vec<NodeRef>)>,
    cells: Vec<BCell>,
    cclass: Vec<Option<Classification>>,
    parent: Vec<u32>,
    active: Vec<bool>,
    ho: BTreeMap<u32, (u8, Basis, Vec<NodeRef>)>,
    eclass: BTreeMap<[VertexIx; 2], Classification>,
    sets: IndexMap<String, SetMembers>,
    set_labels: BTreeMap<String, i64>,
    tags: IndexMap<(MeshEntityKind, String), TagValues>,
    grid: RefCell<Grid>,
}

impl ReferenceBuilder {
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
        Self {
            dim,
            face_off: vec![0],
            ..Self::default()
        }
    }

    fn nfaces(&self) -> usize {
        self.face_off.len().saturating_sub(1)
    }

    fn loop_of(&self, f: usize) -> &[VertexIx] {
        &self.face_verts[self.face_off[f] as usize..self.face_off[f + 1] as usize]
    }

    fn key_of(l: &[VertexIx]) -> FaceKey {
        let mut k: FaceKey = l.iter().map(|v| v.0).collect();
        k.sort_unstable();
        k
    }

    /// Shape of a builder cell computed from its current loops (cold path).
    fn shape_of(&self, c: &BCell) -> CellShape {
        let mut vs: Vec<VertexIx> = Vec::new();
        let mut sizes = Vec::new();
        for sf in &c.faces {
            let l = self.loop_of(sf.face().get());
            sizes.push(l.len());
            vs.extend_from_slice(l);
        }
        vs.sort_unstable();
        vs.dedup();
        CellShape::detect(vs.len(), &sizes)
    }
}

fn check_class(c: &Classification) -> Result<()> {
    if c.dim().is_none() {
        return Err(MeshError::BadClassification(
            "must name a model or partition entity".into(),
        ));
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

impl MeshBuild for ReferenceBuilder {
    type Output = ReferenceMesh;

    fn dim(&self) -> MeshDim {
        self.dim
    }
    fn add_vertex(&mut self, p: Point3) -> Result<VertexIx> {
        if self.pts.len() + self.extra.len() >= u32::MAX as usize - 1 {
            return Err(MeshError::TooLarge);
        }
        self.pts.push(p);
        self.vclass.push(None);
        self.vcell.push(CellIx::NONE);
        Ok(VertexIx::new(self.pts.len() - 1))
    }
    fn add_extra_node(&mut self, p: Point3) -> Result<NodeRef> {
        if self.pts.len() + self.extra.len() >= u32::MAX as usize - 1 {
            return Err(MeshError::TooLarge);
        }
        self.extra.push(p);
        Ok(NodeRef::Extra(self.extra.len() as u32 - 1))
    }
    fn vertex_count(&self) -> usize {
        self.pts.len()
    }
    fn face_count(&self) -> usize {
        self.nfaces()
    }
    fn cell_count(&self) -> usize {
        self.cells.len()
    }
    fn vertex_point(&self, v: VertexIx) -> Point3 {
        self.pts[v.get()]
    }
    fn find_vertex(&self, p: Point3, tol: f64) -> Option<VertexIx> {
        let mut g = self.grid.borrow_mut();
        let exact = !(tol > 0.0 && tol.is_finite());
        let tol = if exact { 0.0 } else { tol };
        if g.tol != tol {
            *g = Grid {
                tol,
                ..Grid::default()
            };
        }
        let from = g.upto;
        for i in from..self.pts.len() {
            let q = self.pts[i];
            if exact {
                g.exact
                    .entry([norm_bits(q.x), norm_bits(q.y), norm_bits(q.z)])
                    .or_insert(i as u32);
            } else {
                g.bins
                    .entry([quant(q.x, tol), quant(q.y, tol), quant(q.z, tol)])
                    .or_default()
                    .push(i as u32);
            }
        }
        g.upto = self.pts.len();
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
                        let q = self.pts[i as usize];
                        if (q.x - p.x).abs() <= tol
                            && (q.y - p.y).abs() <= tol
                            && (q.z - p.z).abs() <= tol
                            && best.is_none_or(|b| i < b)
                        {
                            best = Some(i);
                        }
                    }
                }
            }
        }
        best.map(VertexIx)
    }

    fn add_face(&mut self, l: &[VertexIx]) -> Result<FaceIx> {
        let key = Self::key_of(l);
        if !self.dim.facet_len_ok(l.len()) || key.windows(2).any(|w| w[0] == w[1]) {
            return Err(MeshError::InvalidLoop(match self.dim {
                MeshDim::Three => "needs at least 3 distinct vertices".into(),
                MeshDim::Two => "a dimension-2 facet needs exactly 2 distinct vertices".into(),
            }));
        }
        if let Some(v) = l.iter().find(|v| v.get() >= self.pts.len()) {
            return Err(MeshError::UnknownVertex(*v));
        }
        if self.nfaces() >= MAX_FACES {
            return Err(MeshError::TooLarge);
        }
        let f = self.nfaces();
        self.face_verts.extend_from_slice(l);
        self.face_off.push(self.face_verts.len() as u64);
        self.fclass.push(None);
        self.face_sides.push(0);
        self.face_deg.push(0);
        self.face_cells.push([CellIx::NONE; 2]);
        self.face_key.entry(key).or_default().push(f as u32);
        Ok(FaceIx::new(f))
    }
    fn find_face(&self, l: &[VertexIx]) -> Option<(FaceIx, LoopRelation)> {
        self.face_key
            .get(&Self::key_of(l))?
            .iter()
            .find_map(|&f| loop_relation(l, self.loop_of(f as usize)).map(|r| (FaceIx(f), r)))
    }
    fn find_face_where(
        &self,
        l: &[VertexIx],
        accept: &mut dyn FnMut(FaceIx, LoopRelation) -> bool,
    ) -> Option<(FaceIx, LoopRelation)> {
        self.face_key.get(&Self::key_of(l))?.iter().find_map(|&f| {
            let r = loop_relation(l, self.loop_of(f as usize))?;
            accept(FaceIx(f), r).then_some((FaceIx(f), r))
        })
    }
    fn face_loop(&self, f: FaceIx) -> &[VertexIx] {
        self.loop_of(f.get())
    }
    fn face_degree(&self, f: FaceIx) -> usize {
        self.face_deg[f.get()]
    }
    fn truncate_faces(&mut self, len: usize) -> Result<()> {
        if len >= self.nfaces() {
            return Ok(());
        }
        if self.face_deg[len..].iter().any(|&d| d > 0) {
            return Err(MeshError::State("cannot drop faces that have cells".into()));
        }
        for f in len..self.nfaces() {
            let key = Self::key_of(self.loop_of(f));
            if let Some(v) = self.face_key.get_mut(&key) {
                v.retain(|x| *x as usize != f);
                if v.is_empty() {
                    self.face_key.remove(&key);
                }
            }
        }
        self.face_verts.truncate(self.face_off[len] as usize);
        self.face_off.truncate(len + 1);
        self.fclass.truncate(len);
        self.face_sides.truncate(len);
        self.face_deg.truncate(len);
        self.face_cells.truncate(len);
        self.fkind.retain(|&f, _| (f as usize) < len);
        self.fcurve.retain(|&f, _| (f as usize) < len);
        Ok(())
    }

    fn add_cell(&mut self, faces: &[SignedFace], corners: Option<&[VertexIx]>) -> Result<CellIx> {
        // Phase 1: plan (no mutation).
        let c = CellIx::new(self.cells.len());
        if self.cells.len() >= u32::MAX as usize - 1 {
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
                || ov.iter().any(|v| v.get() >= self.pts.len())
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
            self.face_deg[f] += 1;
            self.face_cells[f][usize::from(sf.side() == Side::Right)] = c.0;
            for v in
                self.face_verts[self.face_off[f] as usize..self.face_off[f + 1] as usize].iter()
            {
                if self.vcell[v.get()] == CellIx::NONE {
                    self.vcell[v.get()] = c.0;
                }
            }
        }
        self.cells.push(BCell {
            faces: faces.iter().copied().collect(),
            corners: corners.map(|o| o.iter().copied().collect()),
        });
        self.cclass.push(None);
        self.parent.push(CellIx::NONE);
        self.active.push(true);
        Ok(c)
    }
    fn cell_faces(&self, c: CellIx) -> &[SignedFace] {
        &self.cells[c.get()].faces
    }
    fn face_cell(&self, f: FaceIx, side: Side) -> Option<CellIx> {
        self.face_cells.get(f.get()).and_then(|c| {
            let x = c[usize::from(side == Side::Right)];
            (x != CellIx::NONE).then_some(CellIx(x))
        })
    }
    fn vertex_cell(&self, v: VertexIx) -> Option<CellIx> {
        self.vcell
            .get(v.get())
            .and_then(|&c| (c != CellIx::NONE).then_some(CellIx(c)))
    }
    fn vertex_class(&self, v: VertexIx) -> Option<Classification> {
        self.vclass.get(v.get()).copied().flatten()
    }
    fn face_class(&self, f: FaceIx) -> Option<Classification> {
        self.fclass.get(f.get()).copied().flatten()
    }
    fn cell_class(&self, c: CellIx) -> Option<Classification> {
        self.cclass.get(c.get()).copied().flatten()
    }

    fn set_class(&mut self, e: MeshEntityRef, c: Classification) -> Result<()> {
        check_class(&c)?;
        let oob = || MeshError::OutOfRange(format!("{e:?}"));
        match e {
            MeshEntityRef::Vertex(v) => *self.vclass.get_mut(v.get()).ok_or_else(oob)? = Some(c),
            MeshEntityRef::Face(f) => *self.fclass.get_mut(f.get()).ok_or_else(oob)? = Some(c),
            MeshEntityRef::Cell(cell) => {
                if !self.dim.accepts_cell(c.entity.kind()) {
                    return Err(MeshError::BadClassification(format!(
                        "cells of a {:?} mesh cannot be classified on {:?}",
                        self.dim,
                        c.entity.kind()
                    )));
                }
                *self.cclass.get_mut(cell.get()).ok_or_else(oob)? = Some(c);
            }
            MeshEntityRef::Edge(..) if self.dim == MeshDim::Two => {
                return Err(MeshError::Unsupported(
                    "a dimension-2 mesh has no edge table: its facets are the edges, classify the facet".into(),
                ));
            }
            MeshEntityRef::Edge(a, b) => {
                if a == b || a.get() >= self.pts.len() || b.get() >= self.pts.len() {
                    return Err(oob());
                }
                self.eclass.insert(edge_key(a, b), c);
            }
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
        if c.get() >= self.cells.len() || parent >= c {
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
    fn set_high_order(
        &mut self,
        c: CellIx,
        degree: u8,
        basis: Basis,
        nodes: &[NodeRef],
    ) -> Result<()> {
        let cell = self
            .cells
            .get(c.get())
            .ok_or_else(|| MeshError::OutOfRange(format!("{c:?}")))?;
        let shape = self.shape_of(cell);
        if shape == CellShape::Polyhedron {
            return Err(MeshError::Unsupported(
                "polyhedra cannot carry a high-order record; curve their faces instead".into(),
            ));
        }
        let Some(corners) = &cell.corners else {
            return Err(MeshError::Unsupported(
                "high order needs template corners (use add_cell_from_vertices)".into(),
            ));
        };
        if degree < 2 {
            return Err(MeshError::Unsupported(
                "a high-order record needs degree >= 2".into(),
            ));
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
        if nodes
            .iter()
            .zip(corners)
            .any(|(n, v)| *n != NodeRef::Vertex(*v))
        {
            return Err(MeshError::OutOfRange(
                "the first nodes must be the cell corners".into(),
            ));
        }
        if nodes.iter().any(|n| match *n {
            NodeRef::Vertex(v) => v.get() >= self.pts.len(),
            NodeRef::Extra(k) => k as usize >= self.extra.len(),
        }) {
            return Err(MeshError::OutOfRange("node does not exist".into()));
        }
        self.ho.insert(c.0, (degree, basis, nodes.to_vec()));
        Ok(())
    }
    fn set_face_curve(
        &mut self,
        f: FaceIx,
        degree: u8,
        basis: Basis,
        nodes: &[NodeRef],
    ) -> Result<()> {
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
            return Err(MeshError::OutOfRange(format!(
                "expected {want} nodes, got {}",
                nodes.len()
            )));
        }
        if nodes.iter().zip(l).any(|(n, v)| *n != NodeRef::Vertex(*v)) {
            return Err(MeshError::OutOfRange(
                "the first nodes must be the face loop vertices in loop order".into(),
            ));
        }
        if nodes
            .iter()
            .any(|n| matches!(*n, NodeRef::Extra(k) if k as usize >= self.extra.len()))
        {
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

    fn finish(self, alloc: &mut SerialAlloc) -> Result<Finished<ReferenceMesh>> {
        // Run against a copy so a failed call draws no ids.
        let mut a = alloc.clone();
        let fin = self.finish_inner(&mut a)?;
        *alloc = a;
        Ok(fin)
    }
}

impl ReferenceBuilder {
    fn finish_inner(self, alloc: &mut SerialAlloc) -> Result<Finished<ReferenceMesh>> {
        let (nv, nf, nc) = (self.pts.len(), self.nfaces(), self.cells.len());
        let vclass: Vec<Classification> = self
            .vclass
            .iter()
            .enumerate()
            .map(|(i, c)| {
                c.ok_or(MeshError::Unclassified(MeshEntity::Vertex(VertexIx::new(
                    i,
                ))))
            })
            .collect::<Result<_>>()?;
        let cclass: Vec<Classification> = self
            .cclass
            .iter()
            .enumerate()
            .map(|(i, c)| c.ok_or(MeshError::Unclassified(MeshEntity::Cell(CellIx::new(i)))))
            .collect::<Result<_>>()?;

        // Incidences, ascending by cell.
        let mut inc: Vec<SmallVec<[(CellIx, Side); 2]>> = vec![SmallVec::new(); nf];
        for (ci, c) in self.cells.iter().enumerate() {
            for sf in &c.faces {
                inc[sf.face().get()].push((CellIx::new(ci), sf.side()));
            }
        }
        struct Fin {
            old: usize,
            lp: SmallVec<[VertexIx; 8]>,
            left: CellIx,
            right: u32,
            kind: FaceKind,
            reversed: bool,
            rot: usize,
        }
        let mut fin: Vec<Fin> = Vec::with_capacity(nf);
        for (f, i) in inc.iter().enumerate() {
            let fi = FaceIx::new(f);
            match i.len() {
                0 => return Err(MeshError::DanglingFace(fi)),
                1 | 2 => {}
                _ => return Err(MeshError::NonManifoldFace(fi)),
            }
            if i.len() == 2 && i[0].1 == i[1].1 {
                return Err(MeshError::InconsistentOrientation(fi));
            }
            let kind = match (i.len(), self.fkind.get(&fi.0)) {
                (2, None) => FaceKind::Interior,
                (2, Some(_)) => {
                    return Err(MeshError::State(format!(
                        "face {fi:?} has two cells and cannot be NonConformal/PartitionCut"
                    )));
                }
                (_, k) => k.copied().unwrap_or(FaceKind::Boundary),
            };
            let mut lp: SmallVec<[VertexIx; 8]> = self.loop_of(f).iter().copied().collect();
            let reversed = i[0].1 == Side::Right;
            if reversed {
                lp.reverse();
            }
            // A two-vertex facet is directed: rotating it would reverse it.
            let rot = if self.dim == MeshDim::Two {
                0
            } else {
                lp.iter()
                    .enumerate()
                    .min_by_key(|(_, v)| **v)
                    .map_or(0, |(p, _)| p)
            };
            lp.rotate_left(rot);
            fin.push(Fin {
                old: f,
                lp,
                left: i[0].0,
                right: i.get(1).map_or(CellIx::NONE, |x| x.0.0),
                kind,
                reversed,
                rot,
            });
        }
        fin.sort_by(|a, b| (a.left, a.right, &a.lp).cmp(&(b.left, b.right, &b.lp)));
        if let Some(w) = fin
            .windows(2)
            .position(|w| w[0].left == w[1].left && w[0].right == w[1].right && w[0].lp == w[1].lp)
        {
            return Err(MeshError::DuplicateFace(FaceIx::new(w)));
        }
        let mut new_of_old = vec![0u32; nf];
        for (n, f) in fin.iter().enumerate() {
            new_of_old[f.old] = n as u32;
        }

        // Face columns and classification (derived for unset interior-like faces).
        let mut face_off = vec![0u64];
        let mut face_verts = Vec::new();
        let (mut face_left, mut face_right, mut face_kind, mut fclass) = (
            Vec::with_capacity(nf),
            Vec::with_capacity(nf),
            Vec::with_capacity(nf),
            Vec::with_capacity(nf),
        );
        for (n, f) in fin.iter().enumerate() {
            face_verts.extend_from_slice(&f.lp);
            face_off.push(face_verts.len() as u64);
            face_left.push(f.left);
            face_right.push(f.right);
            face_kind.push(f.kind);
            let class = match self.fclass[f.old] {
                Some(c) => c,
                None if f.kind == FaceKind::Boundary => {
                    return Err(MeshError::Unclassified(MeshEntity::Face(FaceIx::new(n))));
                }
                None => {
                    let l = cclass[f.left.get()];
                    if f.right != CellIx::NONE && cclass[f.right as usize].entity != l.entity {
                        return Err(MeshError::Unclassified(MeshEntity::Face(FaceIx::new(n))));
                    }
                    Classification::on(l.entity)
                }
            };
            fclass.push(class);
        }

        // Cells: CSR, shapes, corners.
        let (mut cell_off, mut cell_faces) = (vec![0u64], Vec::new());
        let (mut corner_off, mut corners) = (vec![0u64], Vec::new());
        let mut cell_shape = Vec::with_capacity(nc);
        for (ci, c) in self.cells.iter().enumerate() {
            let cix = CellIx::new(ci);
            let mut sfs: SmallVec<[SignedFace; 6]> = c
                .faces
                .iter()
                .map(|sf| {
                    let n = new_of_old[sf.face().get()];
                    SignedFace::new(
                        FaceIx(n),
                        if face_left[n as usize] == cix {
                            Side::Left
                        } else {
                            Side::Right
                        },
                    )
                })
                .collect();
            sfs.sort_by_key(|sf| sf.face());
            let loops: SmallVec<[SmallVec<[VertexIx; 4]>; 6]> = sfs
                .iter()
                .map(|sf| {
                    let l = &face_verts[face_off[sf.face().get()] as usize
                        ..face_off[sf.face().get() + 1] as usize];
                    if sf.side() == Side::Left {
                        l.iter().copied().collect()
                    } else {
                        l.iter().rev().copied().collect()
                    }
                })
                .collect();
            let mut vs: SmallVec<[VertexIx; 16]> =
                loops.iter().flat_map(|l| l.iter().copied()).collect();
            vs.sort_unstable();
            vs.dedup();
            let sizes: SmallVec<[usize; 6]> = loops.iter().map(SmallVec::len).collect();
            let mut shape = CellShape::detect(vs.len(), &sizes);
            let cs: Option<SmallVec<[VertexIx; 8]>> = match &c.corners {
                _ if self.dim == MeshDim::Two => {
                    let refs: SmallVec<[&[VertexIx]; 6]> =
                        loops.iter().map(SmallVec::as_slice).collect();
                    let cyc = polygon_cycle(&refs).ok_or_else(|| {
                        MeshError::ShapeMismatch(
                            cix,
                            "the facets of a dimension-2 cell do not form one closed cycle".into(),
                        )
                    })?;
                    match &c.corners {
                        Some(ov) if loop_relation(ov, &cyc) != Some(LoopRelation::Same) => {
                            return Err(MeshError::ShapeMismatch(
                                cix,
                                "corners are not the cell's vertex cycle in boundary order".into(),
                            ));
                        }
                        Some(ov) => Some(ov.clone()),
                        None => Some(cyc),
                    }
                }
                Some(ov) => {
                    if CellShape::from_corner_count(ov.len()) != Some(shape) {
                        return Err(MeshError::ShapeMismatch(
                            cix,
                            format!("{} corners do not match topology {shape:?}", ov.len()),
                        ));
                    }
                    // Every template face must be one of the cell's outward loops.
                    for lf in shape.local_faces() {
                        let want: SmallVec<[VertexIx; 4]> =
                            lf.iter().map(|&i| ov[i as usize]).collect();
                        if !loops
                            .iter()
                            .any(|l| loop_relation(&want, l) == Some(LoopRelation::Same))
                        {
                            return Err(MeshError::ShapeMismatch(
                                cix,
                                format!("corners do not realise the faces of a {shape:?}"),
                            ));
                        }
                    }
                    Some(ov.clone())
                }
                None if shape != CellShape::Polyhedron => {
                    let refs: SmallVec<[&[VertexIx]; 6]> =
                        loops.iter().map(SmallVec::as_slice).collect();
                    let d = derive_corners(shape, &refs);
                    if d.is_none() {
                        shape = CellShape::Polyhedron;
                    }
                    d
                }
                None => None,
            };
            if let Some(ov) = &cs {
                corners.extend_from_slice(ov);
            }
            corner_off.push(corners.len() as u64);
            cell_faces.extend_from_slice(&sfs);
            cell_off.push(cell_faces.len() as u64);
            cell_shape.push(shape);
        }

        // Ids.
        let mesh_id = alloc.one(EntityKind::Mesh)?;
        let ids = |alloc: &mut SerialAlloc, kind: EntityKind, n: usize| -> Result<Vec<EntityId>> {
            let first = alloc.take(kind, n)?;
            (0..n as u64)
                .map(|i| EntityId::new(kind, first + i).ok_or(MeshError::SerialOverflow))
                .collect()
        };
        let vertex_ids = ids(alloc, EntityKind::MeshVertex, nv)?;
        let face_ids = ids(alloc, EntityKind::MeshFace, nf)?;
        let cell_ids = ids(alloc, EntityKind::MeshCell, nc)?;

        // Points, high order, face curves.
        let pidx = |n: NodeRef| match n {
            NodeRef::Vertex(v) => PointIx(v.0),
            // Vertex and extra-node counts are capped below u32::MAX by the builder, so this cannot wrap.
            NodeRef::Extra(k) => PointIx(nv as u32 + k),
        };
        let ho: BTreeMap<u32, HighOrderCell> = self
            .ho
            .iter()
            .map(|(c, (degree, basis, nodes))| {
                (
                    *c,
                    HighOrderCell {
                        degree: *degree,
                        basis: *basis,
                        nodes: nodes.iter().map(|n| pidx(*n)).collect(),
                    },
                )
            })
            .collect();
        let mut fcurve = BTreeMap::new();
        for (old, (degree, basis, nodes)) in &self.fcurve {
            let n = self.loop_of(*old as usize).len();
            let nu = new_of_old[*old as usize] as usize;
            let f = &fin[nu];
            // Final loop position j holds the builder-loop position `pos(j)`.
            let pos = |j: usize| -> usize {
                let k = (j + f.rot) % n; // index in the (possibly reversed) builder loop
                if f.reversed { n - 1 - k } else { k }
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
        let (mut x, mut y, mut z) = (Vec::new(), Vec::new(), Vec::new());
        for p in self.pts.iter().chain(self.extra.iter()) {
            x.push(p.x);
            y.push(p.y);
            z.push(p.z);
        }

        // Sets and tags, in final face numbering.
        let set_ids = ids(alloc, EntityKind::MeshSet, self.sets.len())?;
        let mut sets = Vec::new();
        for ((name, members), id) in self.sets.into_iter().zip(set_ids) {
            let oob = |what: &str| MeshError::OutOfRange(format!("set '{name}' {what}"));
            let members = match members {
                SetMembers::Faces(v) => {
                    if v.iter().any(|f| f.get() >= nf) {
                        return Err(oob("face"));
                    }
                    let mut m: Vec<FaceIx> =
                        v.iter().map(|f| FaceIx(new_of_old[f.get()])).collect();
                    m.sort_unstable();
                    m.dedup();
                    SetMembers::Faces(m)
                }
                SetMembers::Vertices(mut v) => {
                    v.sort_unstable();
                    v.dedup();
                    if v.last().is_some_and(|x| x.get() >= nv) {
                        return Err(oob("vertex"));
                    }
                    SetMembers::Vertices(v)
                }
                SetMembers::Cells(mut v) => {
                    v.sort_unstable();
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
                        let c = self.face_cells[pf][usize::from(sf.side() == Side::Right)];
                        if c == CellIx::NONE {
                            return Err(oob("side without a cell"));
                        }
                        let n = new_of_old[pf];
                        let side = if face_left[n as usize] == CellIx(c) { Side::Left } else { Side::Right };
                        m.push(SignedFace::new(FaceIx(n), side));
                    }
                    m.sort_unstable();
                    m.dedup();
                    SetMembers::Sides(m)
                }
                SetMembers::ShellSides(mut v) => {
                    if self.dim != MeshDim::Two {
                        return Err(oob("is a shell side set, valid only in a dimension-2 mesh"));
                    }
                    v.sort_unstable();
                    v.dedup();
                    if v.last().is_some_and(|x| (x >> 1) as usize >= nc) {
                        return Err(oob("cell"));
                    }
                    SetMembers::ShellSides(v)
                }
                SetMembers::Points(mut v) => {
                    v.sort_unstable();
                    v.dedup();
                    if v.last().is_some_and(|x| x.get() >= nv + self.extra.len()) {
                        return Err(oob("point"));
                    }
                    SetMembers::Points(v)
                }
            };
            let label = self.set_labels.get(&name).copied();
            if label.is_some() && sets.iter().any(|s: &MeshSet| s.label == label && s.members.kind() == members.kind()) {
                return Err(MeshError::DuplicateName(format!("set label {label:?}")));
            }
            sets.push(MeshSet { id, name, label, members });
        }
        let mut tags = Vec::new();
        for ((target, name), values) in self.tags {
            let n = match target {
                MeshEntityKind::Vertex => nv,
                MeshEntityKind::Face => nf,
                MeshEntityKind::Cell => nc,
                MeshEntityKind::Edge => 0,
            };
            if values.len() != n {
                return Err(MeshError::OutOfRange(format!(
                    "tag '{name}' has {} values, expected {n}",
                    values.len()
                )));
            }
            let values = if target == MeshEntityKind::Face {
                match values {
                    TagValues::I64(v) => TagValues::I64(fin.iter().map(|f| v[f.old]).collect()),
                    TagValues::F64(v) => TagValues::F64(fin.iter().map(|f| v[f.old]).collect()),
                }
            } else {
                values
            };
            tags.push(TagColumn {
                name,
                target,
                values,
            });
        }
        tags.sort_by(|a, b| (a.target, &a.name).cmp(&(b.target, &b.name)));

        let mut mesh = ReferenceMesh::from_parts(Parts {
            dim: self.dim,
            id: mesh_id,
            vertex_ids,
            face_ids,
            cell_ids,
            x,
            y,
            z,
            n_vertices: nv,
            face_off,
            face_verts,
            face_left,
            face_right,
            face_kind,
            cell_off,
            cell_faces,
            cell_shape,
            corner_off,
            corners,
            cell_parent: self.parent,
            cell_active: self.active,
            vclass,
            fclass,
            cclass,
            ho,
            fcurve,
            sets,
            tags,
        });
        // Edge-slot nodes (dimension 3) need the edge table: derive it even without explicit
        // edge classes (an ambiguous edge fails with `UnclassifiedEdge`; classify it with
        // `set_class(Edge(..))`).
        if !self.eclass.is_empty() || mesh.needs_edge_table() {
            let t = EdgeTable::derive(&mesh, alloc, &self.eclass)?;
            mesh.attach_edges(t);
        }
        let v = crate::validate::validate(&mesh);
        if !v.is_empty() {
            return Err(MeshError::Invalid(v));
        }
        Ok(Finished {
            mesh,
            face_remap: new_of_old.into_iter().map(FaceIx).collect(),
        })
    }
}
