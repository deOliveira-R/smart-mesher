//! In-memory reference implementation of the mesh contract.
//!
//! Plain `Vec` columns in CSR layout; no cleverness. It exists to pin down semantics for the
//! contract tests and to be the oracle other implementations are compared against.

mod builder;
mod catalog;

pub use builder::ReferenceBuilder;
pub use catalog::ReferenceCatalog;

use crate::build::{MeshEdit, SerialAlloc};
use crate::classify::{ClassSet, Classification};
use crate::edges::EdgeTable;
use crate::error::{MeshError, Result};
use crate::highorder::HighOrderCell;
use crate::index::{CellFace, CellIx, EdgeIx, FaceIx, MeshEntity, MeshEntityKind, VertexIx};
use crate::read::{Coords, MeshRead};
use crate::sets::{MeshSet, SetMembers, TagColumn};
use crate::shape::CellShape;
use sm_core::{EntityId, EntityKind, Point3};
use std::collections::BTreeMap;

/// A finished, canonical, in-memory mesh. Create with [`ReferenceBuilder`].
#[derive(Clone, Debug, PartialEq)]
pub struct ReferenceMesh {
    id: EntityId,
    vertex_ids: Vec<EntityId>,
    face_ids: Vec<EntityId>,
    cell_ids: Vec<EntityId>,
    x: Vec<f64>,
    y: Vec<f64>,
    z: Vec<f64>,
    n_vertices: usize,
    face_off: Vec<u32>,
    face_verts: Vec<VertexIx>,
    face_left: Vec<CellIx>,
    face_right: Vec<u32>,
    cell_off: Vec<u32>,
    cell_faces: Vec<CellFace>,
    cell_shape: Vec<CellShape>,
    ordered_off: Vec<u32>,
    ordered: Vec<VertexIx>,
    vclass: Vec<Classification>,
    fclass: Vec<Classification>,
    cclass: Vec<Classification>,
    class_index: BTreeMap<EntityId, ClassSet>,
    ho: BTreeMap<u32, HighOrderCell>,
    sets: Vec<MeshSet>,
    tags: Vec<TagColumn>,
    edges: Option<EdgeTable>,
}

fn sorted_insert<T: Ord + Copy>(v: &mut Vec<T>, x: T) {
    if let Err(p) = v.binary_search(&x) {
        v.insert(p, x);
    }
}
fn sorted_remove<T: Ord + Copy>(v: &mut Vec<T>, x: T) {
    if let Ok(p) = v.binary_search(&x) {
        v.remove(p);
    }
}

impl MeshRead for ReferenceMesh {
    fn mesh_id(&self) -> EntityId {
        self.id
    }
    fn vertex_count(&self) -> usize {
        self.n_vertices
    }
    fn face_count(&self) -> usize {
        self.face_ids.len()
    }
    fn cell_count(&self) -> usize {
        self.cell_ids.len()
    }
    fn vertex_id(&self, v: VertexIx) -> EntityId {
        self.vertex_ids[v.get()]
    }
    fn face_id(&self, f: FaceIx) -> EntityId {
        self.face_ids[f.get()]
    }
    fn cell_id(&self, c: CellIx) -> EntityId {
        self.cell_ids[c.get()]
    }
    fn vertex_index(&self, id: EntityId) -> Option<VertexIx> {
        self.vertex_ids.binary_search(&id).ok().map(VertexIx::new)
    }
    fn face_index(&self, id: EntityId) -> Option<FaceIx> {
        self.face_ids.binary_search(&id).ok().map(FaceIx::new)
    }
    fn cell_index(&self, id: EntityId) -> Option<CellIx> {
        self.cell_ids.binary_search(&id).ok().map(CellIx::new)
    }
    fn coords(&self) -> Coords<'_> {
        Coords { x: &self.x, y: &self.y, z: &self.z }
    }
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx] {
        &self.face_verts[self.face_off[f.get()] as usize..self.face_off[f.get() + 1] as usize]
    }
    fn face_left(&self, f: FaceIx) -> CellIx {
        self.face_left[f.get()]
    }
    fn face_right(&self, f: FaceIx) -> Option<CellIx> {
        let r = self.face_right[f.get()];
        (r != CellIx::NONE).then_some(CellIx(r))
    }
    fn cell_faces(&self, c: CellIx) -> &[CellFace] {
        &self.cell_faces[self.cell_off[c.get()] as usize..self.cell_off[c.get() + 1] as usize]
    }
    fn cell_shape(&self, c: CellIx) -> CellShape {
        self.cell_shape[c.get()]
    }
    fn cell_ordered_vertices(&self, c: CellIx) -> Option<&[VertexIx]> {
        let s = &self.ordered[self.ordered_off[c.get()] as usize..self.ordered_off[c.get() + 1] as usize];
        (!s.is_empty()).then_some(s)
    }
    fn high_order_cell(&self, c: CellIx) -> Option<&HighOrderCell> {
        self.ho.get(&c.0)
    }
    fn vertex_class(&self, v: VertexIx) -> Classification {
        self.vclass[v.get()]
    }
    fn face_class(&self, f: FaceIx) -> Classification {
        self.fclass[f.get()]
    }
    fn cell_class(&self, c: CellIx) -> Classification {
        self.cclass[c.get()]
    }
    fn classified_on(&self, e: EntityId) -> Option<&ClassSet> {
        self.class_index.get(&e)
    }
    fn classified_entities(&self) -> Vec<EntityId> {
        self.class_index.keys().copied().collect()
    }
    fn sets(&self) -> &[MeshSet] {
        &self.sets
    }
    fn tags(&self) -> &[TagColumn] {
        &self.tags
    }
    fn edges(&self) -> Option<&EdgeTable> {
        self.edges.as_ref()
    }
}

impl ReferenceMesh {
    fn reindex(&mut self, e: MeshEntity, old: Classification, new: Classification) {
        if old.entity == new.entity {
            return;
        }
        if let Some(s) = self.class_index.get_mut(&old.entity) {
            match e {
                MeshEntity::Vertex(v) => sorted_remove(&mut s.vertices, v),
                MeshEntity::Face(f) => sorted_remove(&mut s.faces, f),
                MeshEntity::Cell(c) => sorted_remove(&mut s.cells, c),
                MeshEntity::Edge(_) => {}
            }
            if s.vertices.is_empty() && s.faces.is_empty() && s.cells.is_empty() {
                self.class_index.remove(&old.entity);
            }
        }
        let s = self.class_index.entry(new.entity).or_default();
        match e {
            MeshEntity::Vertex(v) => sorted_insert(&mut s.vertices, v),
            MeshEntity::Face(f) => sorted_insert(&mut s.faces, f),
            MeshEntity::Cell(c) => sorted_insert(&mut s.cells, c),
            MeshEntity::Edge(_) => {}
        }
    }
}

impl MeshEdit for ReferenceMesh {
    fn set_point(&mut self, i: usize, p: Point3) -> Result<()> {
        if i >= self.x.len() {
            return Err(MeshError::OutOfRange(format!("point {i}")));
        }
        (self.x[i], self.y[i], self.z[i]) = (p.x, p.y, p.z);
        Ok(())
    }
    fn set_class(&mut self, e: MeshEntity, c: Classification) -> Result<()> {
        if c.dim().is_none() {
            return Err(MeshError::State("classification must name a model or partition entity".into()));
        }
        let oob = || MeshError::OutOfRange(format!("{e:?}"));
        match e {
            MeshEntity::Vertex(v) => {
                let old = *self.vclass.get(v.get()).ok_or_else(oob)?;
                self.vclass[v.get()] = c;
                self.reindex(e, old, c);
            }
            MeshEntity::Face(f) => {
                let old = *self.fclass.get(f.get()).ok_or_else(oob)?;
                self.fclass[f.get()] = c;
                self.reindex(e, old, c);
            }
            MeshEntity::Cell(cell) => {
                let old = *self.cclass.get(cell.get()).ok_or_else(oob)?;
                self.cclass[cell.get()] = c;
                self.reindex(e, old, c);
            }
            MeshEntity::Edge(EdgeIx(i)) => {
                let t = self.edges.as_mut().ok_or_else(|| MeshError::State("no edge table".into()))?;
                *t.class.get_mut(i as usize).ok_or_else(oob)? = Some(c);
            }
        }
        Ok(())
    }
    fn set_high_order(&mut self, c: CellIx, h: HighOrderCell) -> Result<()> {
        if c.get() >= self.cell_count() || h.degree < 2 || h.nodes.iter().any(|&n| n as usize >= self.x.len()) {
            return Err(MeshError::OutOfRange("high-order record".into()));
        }
        self.ho.insert(c.0, h);
        Ok(())
    }
    fn clear_high_order(&mut self, c: CellIx) -> Result<()> {
        self.ho.remove(&c.0);
        Ok(())
    }
    fn add_set(&mut self, name: &str, members: SetMembers, alloc: &mut SerialAlloc) -> Result<EntityId> {
        if self.sets.iter().any(|s| s.name == name) {
            return Err(MeshError::DuplicateName(name.into()));
        }
        let (n, len) = match &members {
            SetMembers::Vertices(v) => (self.vertex_count(), v.iter().map(|x| x.get()).collect::<Vec<_>>()),
            SetMembers::Faces(v) => (self.face_count(), v.iter().map(|x| x.get()).collect()),
            SetMembers::Cells(v) => (self.cell_count(), v.iter().map(|x| x.get()).collect()),
        };
        if len.iter().any(|&i| i >= n) || len.windows(2).any(|w| w[0] >= w[1]) {
            return Err(MeshError::OutOfRange(format!("set '{name}' members must be ascending, unique and in range")));
        }
        let id = alloc.one(EntityKind::Group)?;
        self.sets.push(MeshSet { id, name: name.into(), members });
        self.sets.sort_by_key(|s| s.id);
        Ok(id)
    }
    fn set_tag(&mut self, name: &str, target: MeshEntityKind, values: Vec<i64>) -> Result<()> {
        let n = match target {
            MeshEntityKind::Vertex => self.vertex_count(),
            MeshEntityKind::Edge => self.edges.as_ref().map_or(0, EdgeTable::len),
            MeshEntityKind::Face => self.face_count(),
            MeshEntityKind::Cell => self.cell_count(),
        };
        if values.len() != n {
            return Err(MeshError::OutOfRange(format!("tag '{name}' has {} values, expected {n}", values.len())));
        }
        let col = TagColumn { name: name.into(), target, values };
        match self.tags.iter_mut().find(|t| t.name == name) {
            Some(t) => *t = col,
            None => self.tags.push(col),
        }
        Ok(())
    }
    fn ensure_edges(&mut self, alloc: &mut SerialAlloc) -> Result<()> {
        if self.edges.is_none() {
            self.edges = Some(EdgeTable::derive(self, alloc)?);
        }
        Ok(())
    }
    fn drop_edges(&mut self) {
        self.edges = None;
    }
}
