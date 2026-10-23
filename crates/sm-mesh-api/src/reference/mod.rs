//! In-memory reference implementation of the mesh contract.
//!
//! Plain `Vec` columns in CSR layout; no cleverness. It exists to pin down semantics for the
//! contract tests and to be the oracle other implementations are compared against.

mod blocks;
mod builder;
mod catalog;

pub use blocks::{hex_block, hex_block_with_edges};
pub use builder::ReferenceBuilder;
pub use catalog::ReferenceCatalog;

use crate::build::{MeshEdit, SerialAlloc};
use crate::classify::{ClassSet, Classification};
use crate::edges::{EdgeTable, edge_key};
use crate::error::{MeshError, Result};
use crate::highorder::{Basis, FaceCurve, HighOrderCell, face_node_count, node_count};
use crate::index::{CellIx, FaceIx, FaceKind, MeshEntity, MeshEntityKind, PointIx, VertexIx};
use crate::read::{Coords, GeometryCache, MeshRead, TopologyColumns};
use crate::sets::{MeshSet, SetMembers, TagColumn, TagValues};
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
    face_off: Vec<u64>,
    face_verts: Vec<VertexIx>,
    face_left: Vec<CellIx>,
    face_right: Vec<u32>,
    face_kind: Vec<FaceKind>,
    cell_off: Vec<u64>,
    cell_faces: Vec<crate::index::SignedFace>,
    cell_shape: Vec<CellShape>,
    corner_off: Vec<u64>,
    corners: Vec<VertexIx>,
    cell_parent: Vec<u32>,
    cell_active: Vec<bool>,
    vclass: Vec<Classification>,
    fclass: Vec<Classification>,
    cclass: Vec<Classification>,
    class_index: BTreeMap<EntityId, ClassSet>,
    class_entities: Vec<EntityId>,
    ho: BTreeMap<u32, HighOrderCell>,
    fcurve: BTreeMap<u32, (u8, Basis, Vec<PointIx>)>,
    sets: Vec<MeshSet>,
    tags: Vec<TagColumn>,
    edges: Option<EdgeTable>,
    cache: Option<GeometryCache>,
}

fn range(off: &[u64], i: usize) -> core::ops::Range<usize> {
    off[i] as usize..off[i + 1] as usize
}

impl MeshRead for ReferenceMesh {
    fn mesh_id(&self) -> EntityId {
        self.id
    }
    fn count(&self, k: MeshEntityKind) -> usize {
        match k {
            MeshEntityKind::Vertex => self.n_vertices,
            MeshEntityKind::Edge => self.edges.as_ref().map_or(0, EdgeTable::len),
            MeshEntityKind::Face => self.face_ids.len(),
            MeshEntityKind::Cell => self.cell_ids.len(),
        }
    }
    fn id(&self, k: MeshEntityKind, ix: u32) -> EntityId {
        match k {
            MeshEntityKind::Vertex => self.vertex_ids[ix as usize],
            MeshEntityKind::Edge => self.edges.as_ref().map_or(&[][..], |t| &t.ids)[ix as usize],
            MeshEntityKind::Face => self.face_ids[ix as usize],
            MeshEntityKind::Cell => self.cell_ids[ix as usize],
        }
    }
    fn index_of(&self, id: EntityId) -> Option<u32> {
        let ids: &[EntityId] = match id.kind() {
            EntityKind::MeshVertex => &self.vertex_ids,
            EntityKind::MeshEdge => self.edges.as_ref().map_or(&[][..], |t| &t.ids),
            EntityKind::MeshFace => &self.face_ids,
            EntityKind::MeshCell => &self.cell_ids,
            _ => return None,
        };
        ids.binary_search(&id).ok().map(|i| i as u32)
    }
    fn coords(&self) -> Coords<'_> {
        Coords {
            x: &self.x,
            y: &self.y,
            z: &self.z,
        }
    }
    fn columns(&self) -> TopologyColumns<'_> {
        TopologyColumns {
            face_vertex_offsets: &self.face_off,
            face_vertices: &self.face_verts,
            face_left: &self.face_left,
            face_right: &self.face_right,
            cell_face_offsets: &self.cell_off,
            cell_faces: &self.cell_faces,
        }
    }
    fn face_vertices(&self, f: FaceIx) -> &[VertexIx] {
        &self.face_verts[range(&self.face_off, f.get())]
    }
    fn face_left(&self, f: FaceIx) -> CellIx {
        self.face_left[f.get()]
    }
    fn face_right(&self, f: FaceIx) -> Option<CellIx> {
        let r = self.face_right[f.get()];
        (r != CellIx::NONE).then_some(CellIx(r))
    }
    fn face_kind(&self, f: FaceIx) -> FaceKind {
        self.face_kind[f.get()]
    }
    fn cell_faces(&self, c: CellIx) -> &[crate::index::SignedFace] {
        &self.cell_faces[range(&self.cell_off, c.get())]
    }
    fn cell_shape(&self, c: CellIx) -> CellShape {
        self.cell_shape[c.get()]
    }
    fn cell_corners(&self, c: CellIx) -> Option<&[VertexIx]> {
        let s = &self.corners[range(&self.corner_off, c.get())];
        (!s.is_empty()).then_some(s)
    }
    fn high_order_cell(&self, c: CellIx) -> Option<&HighOrderCell> {
        self.ho.get(&c.0)
    }
    fn face_curve(&self, f: FaceIx) -> Option<FaceCurve<'_>> {
        self.fcurve
            .get(&f.0)
            .map(|(degree, basis, nodes)| FaceCurve {
                degree: *degree,
                basis: *basis,
                nodes,
            })
    }
    fn cell_parent(&self, c: CellIx) -> Option<CellIx> {
        let p = self.cell_parent[c.get()];
        (p != CellIx::NONE).then_some(CellIx(p))
    }
    fn cell_active(&self, c: CellIx) -> bool {
        self.cell_active[c.get()]
    }
    fn classification(&self, k: MeshEntityKind, ix: u32) -> Classification {
        match k {
            MeshEntityKind::Vertex => self.vclass[ix as usize],
            MeshEntityKind::Edge => self.edges.as_ref().map_or(&[][..], |t| &t.class)[ix as usize],
            MeshEntityKind::Face => self.fclass[ix as usize],
            MeshEntityKind::Cell => self.cclass[ix as usize],
        }
    }
    fn classified_on(&self, model: EntityId) -> Option<&ClassSet> {
        self.class_index.get(&model)
    }
    fn classified_entities(&self) -> &[EntityId] {
        &self.class_entities
    }
    fn edges(&self) -> Option<&EdgeTable> {
        self.edges.as_ref()
    }
    fn sets(&self) -> &[MeshSet] {
        &self.sets
    }
    fn tags(&self) -> &[TagColumn] {
        &self.tags
    }
    fn geometry_cache(&self) -> Option<&GeometryCache> {
        self.cache.as_ref()
    }
}

impl ReferenceMesh {
    /// Assemble a mesh from finished columns (used by the builder).
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn from_parts(p: builder::Parts) -> Self {
        let mut m = ReferenceMesh {
            id: p.id,
            vertex_ids: p.vertex_ids,
            face_ids: p.face_ids,
            cell_ids: p.cell_ids,
            x: p.x,
            y: p.y,
            z: p.z,
            n_vertices: p.n_vertices,
            face_off: p.face_off,
            face_verts: p.face_verts,
            face_left: p.face_left,
            face_right: p.face_right,
            face_kind: p.face_kind,
            cell_off: p.cell_off,
            cell_faces: p.cell_faces,
            cell_shape: p.cell_shape,
            corner_off: p.corner_off,
            corners: p.corners,
            cell_parent: p.cell_parent,
            cell_active: p.cell_active,
            vclass: p.vclass,
            fclass: p.fclass,
            cclass: p.cclass,
            class_index: BTreeMap::new(),
            class_entities: Vec::new(),
            ho: p.ho,
            fcurve: p.fcurve,
            sets: p.sets,
            tags: p.tags,
            edges: None,
            cache: None,
        };
        m.rebuild_class_index();
        m
    }

    pub(crate) fn attach_edges(&mut self, t: EdgeTable) {
        self.edges = Some(t);
        self.rebuild_class_index();
    }

    fn rebuild_class_index(&mut self) {
        let mut idx: BTreeMap<EntityId, ClassSet> = BTreeMap::new();
        for (i, c) in self.vclass.iter().enumerate() {
            idx.entry(c.entity)
                .or_default()
                .vertices
                .push(VertexIx::new(i));
        }
        if let Some(t) = &self.edges {
            for (i, c) in t.class.iter().enumerate() {
                idx.entry(c.entity)
                    .or_default()
                    .edges
                    .push(crate::index::EdgeIx::new(i));
            }
        }
        for (i, c) in self.fclass.iter().enumerate() {
            idx.entry(c.entity).or_default().faces.push(FaceIx::new(i));
        }
        for (i, c) in self.cclass.iter().enumerate() {
            idx.entry(c.entity).or_default().cells.push(CellIx::new(i));
        }
        self.class_entities = idx.keys().copied().collect();
        self.class_index = idx;
    }

    /// Everything `set_class` checks, without changing anything.
    fn check_set_class(&self, e: MeshEntity, c: &Classification) -> Result<()> {
        Self::check_class(c)?;
        let oob = || MeshError::OutOfRange(format!("{e:?}"));
        match e {
            MeshEntity::Vertex(v) => {
                (v.get() < self.vclass.len())
                    .then_some(())
                    .ok_or_else(oob)?;
            }
            MeshEntity::Face(f) => {
                let kind = *self.face_kind.get(f.get()).ok_or_else(oob)?;
                let k = c.entity.kind();
                let ok = match kind {
                    FaceKind::Boundary => k == EntityKind::TopoFace,
                    _ => {
                        k == EntityKind::TopoFace
                            || matches!(k, EntityKind::TopoVolume | EntityKind::PartitionCell)
                    }
                };
                if !ok {
                    return Err(MeshError::BadClassification(format!(
                        "a {kind:?} face cannot be classified on {k:?}"
                    )));
                }
            }
            MeshEntity::Cell(cell) => {
                if cell.get() >= self.cclass.len() {
                    return Err(oob());
                }
                if !matches!(
                    c.entity.kind(),
                    EntityKind::TopoVolume | EntityKind::PartitionCell
                ) {
                    return Err(MeshError::BadClassification(
                        "cells need a volume or partition cell".into(),
                    ));
                }
            }
            MeshEntity::Edge(i) => {
                let t = self
                    .edges
                    .as_ref()
                    .ok_or_else(|| MeshError::State("no edge table".into()))?;
                (i.get() < t.class.len()).then_some(()).ok_or_else(oob)?;
            }
        }
        Ok(())
    }

    /// Store a class already accepted by `check_set_class`.
    fn apply_class(&mut self, e: MeshEntity, c: Classification) {
        match e {
            MeshEntity::Vertex(v) => self.vclass[v.get()] = c,
            MeshEntity::Face(f) => self.fclass[f.get()] = c,
            MeshEntity::Cell(cell) => self.cclass[cell.get()] = c,
            MeshEntity::Edge(i) => {
                if let Some(t) = self.edges.as_mut() {
                    t.class[i.get()] = c;
                }
            }
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

    fn cell_corner_count(&self, c: CellIx) -> Result<&[VertexIx]> {
        self.cell_corners(c)
            .ok_or_else(|| MeshError::Unsupported("high order needs a template-shaped cell".into()))
    }
}

impl MeshEdit for ReferenceMesh {
    fn set_point(&mut self, i: PointIx, p: Point3) -> Result<()> {
        if i.get() >= self.x.len() {
            return Err(MeshError::OutOfRange(format!("point {i:?}")));
        }
        (self.x[i.get()], self.y[i.get()], self.z[i.get()]) = (p.x, p.y, p.z);
        self.cache = None;
        Ok(())
    }
    fn add_extra_node(&mut self, p: Point3) -> Result<PointIx> {
        if self.x.len() >= u32::MAX as usize - 1 {
            return Err(MeshError::TooLarge);
        }
        self.x.push(p.x);
        self.y.push(p.y);
        self.z.push(p.z);
        self.cache = None;
        Ok(PointIx::new(self.x.len() - 1))
    }
    fn set_class(&mut self, e: MeshEntity, c: Classification) -> Result<()> {
        self.check_set_class(e, &c)?;
        self.apply_class(e, c);
        self.rebuild_class_index();
        Ok(())
    }
    fn set_classes(&mut self, items: &[(MeshEntity, Classification)]) -> Result<()> {
        for (e, c) in items {
            self.check_set_class(*e, c)?;
        }
        for (e, c) in items {
            self.apply_class(*e, *c);
        }
        self.rebuild_class_index();
        Ok(())
    }
    fn set_face_kind(&mut self, f: FaceIx, kind: FaceKind) -> Result<()> {
        let cur = *self
            .face_kind
            .get(f.get())
            .ok_or_else(|| MeshError::OutOfRange(format!("{f:?}")))?;
        if cur == FaceKind::Interior || kind == FaceKind::Interior {
            return Err(MeshError::State(
                "Interior is derived from having two cells and cannot be set".into(),
            ));
        }
        if kind == FaceKind::Boundary && self.fclass[f.get()].entity.kind() != EntityKind::TopoFace
        {
            return Err(MeshError::BadClassification(
                "a Boundary face must be classified on a TopoFace".into(),
            ));
        }
        self.face_kind[f.get()] = kind;
        Ok(())
    }
    fn set_high_order(&mut self, c: CellIx, h: Option<HighOrderCell>) -> Result<()> {
        if c.get() >= self.cell_ids.len() {
            return Err(MeshError::OutOfRange(format!("{c:?}")));
        }
        let Some(h) = h else {
            self.ho.remove(&c.0);
            return Ok(());
        };
        let corners = self.cell_corner_count(c)?;
        let want = node_count(self.cell_shape[c.get()], h.degree, h.basis)
            .filter(|_| h.degree >= 2)
            .ok_or_else(|| {
                MeshError::Unsupported(format!(
                    "high order {:?} degree {} {:?}",
                    self.cell_shape[c.get()],
                    h.degree,
                    h.basis
                ))
            })?;
        if h.nodes.len() != want {
            return Err(MeshError::OutOfRange(format!(
                "expected {want} nodes, got {}",
                h.nodes.len()
            )));
        }
        if h.nodes.iter().any(|n| n.get() >= self.x.len())
            || h.nodes.iter().zip(corners).any(|(n, v)| n.get() != v.get())
        {
            return Err(MeshError::OutOfRange(
                "nodes must exist and start with the cell corners".into(),
            ));
        }
        self.ho.insert(c.0, h);
        Ok(())
    }
    fn set_face_curve(&mut self, f: FaceIx, c: Option<(u8, Basis, Vec<PointIx>)>) -> Result<()> {
        if f.get() >= self.face_ids.len() {
            return Err(MeshError::OutOfRange(format!("{f:?}")));
        }
        let Some((degree, basis, nodes)) = c else {
            self.fcurve.remove(&f.0);
            return Ok(());
        };
        let l = &self.face_verts[range(&self.face_off, f.get())];
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
        if nodes.iter().any(|n| n.get() >= self.x.len())
            || nodes.iter().zip(l).any(|(n, v)| n.get() != v.get())
        {
            return Err(MeshError::OutOfRange(
                "nodes must exist and start with the face loop vertices".into(),
            ));
        }
        self.fcurve.insert(f.0, (degree, basis, nodes));
        Ok(())
    }
    fn add_set(
        &mut self,
        name: &str,
        members: SetMembers,
        alloc: &mut SerialAlloc,
    ) -> Result<EntityId> {
        if self.sets.iter().any(|s| s.name == name) {
            return Err(MeshError::DuplicateName(name.into()));
        }
        let n = self.count(members.kind());
        let raw = members.raw();
        if raw.last().is_some_and(|&i| i as usize >= n) || raw.windows(2).any(|w| w[0] >= w[1]) {
            return Err(MeshError::OutOfRange(format!(
                "set '{name}' members must be ascending, unique and in range"
            )));
        }
        let id = alloc.one(EntityKind::Group)?;
        self.sets.push(MeshSet {
            id,
            name: name.into(),
            members,
        });
        self.sets.sort_by_key(|s| s.id);
        Ok(id)
    }
    fn set_tag(&mut self, name: &str, target: MeshEntityKind, values: TagValues) -> Result<()> {
        if target == MeshEntityKind::Edge && self.edges.is_none() {
            return Err(MeshError::State("no edge table for an edge tag".into()));
        }
        let n = self.count(target);
        if values.len() != n {
            return Err(MeshError::OutOfRange(format!(
                "tag '{name}' has {} values, expected {n}",
                values.len()
            )));
        }
        let col = TagColumn {
            name: name.into(),
            target,
            values,
        };
        match self
            .tags
            .binary_search_by(|t| (t.target, &t.name).cmp(&(target, &col.name)))
        {
            Ok(i) => self.tags[i] = col,
            Err(i) => self.tags.insert(i, col),
        }
        Ok(())
    }
    fn ensure_edges(
        &mut self,
        alloc: &mut SerialAlloc,
        explicit: &[(VertexIx, VertexIx, Classification)],
    ) -> Result<()> {
        if self.edges.is_some() {
            return Ok(());
        }
        let mut map = BTreeMap::new();
        for (a, b, c) in explicit {
            Self::check_class(c)?;
            map.insert(edge_key(*a, *b), *c);
        }
        let t = EdgeTable::derive(self, alloc, &map)?;
        self.attach_edges(t);
        Ok(())
    }
    fn drop_edges(&mut self) {
        self.edges = None;
        self.tags.retain(|t| t.target != MeshEntityKind::Edge);
        self.rebuild_class_index();
    }
    fn set_geometry_cache(&mut self, g: Option<GeometryCache>) -> Result<()> {
        if let Some(g) = &g {
            let (nf, nc) = (self.face_ids.len(), self.cell_ids.len());
            let ok = g.rule == self.geometry_rule()
                && g.face_area.len() == nf
                && g.face_centroid.len() == nf
                && g.face_apex.len() == nf
                && g.cell_volume.len() == nc
                && g.cell_centroid.len() == nc
                && g.connectivity_hash == crate::validate::connectivity_hash(self)
                && g.coordinate_hash == crate::validate::coordinate_hash(self);
            if !ok {
                return Err(MeshError::State(
                    "geometry cache does not match this mesh (rule, sizes, connectivity or coordinates)"
                        .into(),
                ));
            }
        }
        self.cache = g;
        Ok(())
    }
}
