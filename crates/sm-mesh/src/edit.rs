//! `MeshEdit` for the finished mesh: topology-preserving edits.

use crate::builder::check_class;
use crate::store::Mesh;
use crate::util::range;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::{
    Basis, CellIx, Classification, FaceIx, FaceKind, GeometryCache, HighOrderCell, MeshDim, MeshEdit,
    MeshEntity, MeshEntityKind, MeshError, MeshRead, PointIx, Result, SerialAlloc, SetMembers, Side,
    TagColumn, TagValues, VertexIx, edges::edge_key, face_node_count, node_count,
};
use std::collections::BTreeMap;

impl Mesh {
    fn require_edges_for_high_order(&self) -> Result<()> {
        if self.dim == MeshDim::Three && self.edges.is_none() {
            return Err(MeshError::State(
                "a high-order dimension-3 mesh needs an edge table: call ensure_edges first".into(),
            ));
        }
        Ok(())
    }

    /// Everything `set_class` checks, without changing anything.
    fn check_set_class(&self, e: MeshEntity, c: &Classification) -> Result<()> {
        check_class(c)?;
        let oob = || MeshError::OutOfRange(format!("{e:?}"));
        match e {
            MeshEntity::Vertex(v) => {
                (v.get() < self.vclass.len()).then_some(()).ok_or_else(oob)?;
            }
            MeshEntity::Face(f) => {
                let kind = *self.face_kind.get(f.get()).ok_or_else(oob)?;
                let k = c.entity.kind();
                if !self.dim.accepts_facet(kind, k) {
                    return Err(MeshError::BadClassification(format!(
                        "a {kind:?} face cannot be classified on {k:?}"
                    )));
                }
            }
            MeshEntity::Cell(cell) => {
                if cell.get() >= self.cclass.len() {
                    return Err(oob());
                }
                if !self.dim.accepts_cell(c.entity.kind()) {
                    return Err(MeshError::BadClassification(format!(
                        "cells of a {:?} mesh cannot be classified on {:?}",
                        self.dim,
                        c.entity.kind()
                    )));
                }
            }
            MeshEntity::Edge(i) => {
                let t = self.edges.as_ref().ok_or_else(|| MeshError::State("no edge table".into()))?;
                (i.get() < t.class.len()).then_some(()).ok_or_else(oob)?;
            }
            _ => return Err(MeshError::Unsupported("unknown entity".into())),
        }
        Ok(())
    }

    /// Store a class already accepted by `check_set_class`.
    fn apply_class(&mut self, e: MeshEntity, c: Classification) {
        match e {
            MeshEntity::Vertex(v) => self.vclass.set(v.get(), c),
            MeshEntity::Face(f) => self.fclass.set(f.get(), c),
            MeshEntity::Cell(cell) => self.cclass.set(cell.get(), c),
            MeshEntity::Edge(i) => {
                if let Some(t) = self.edges.as_mut() {
                    t.class[i.get()] = c;
                }
            }
            _ => {}
        }
    }

    /// Attach an edge table (derived elsewhere) and refresh everything that depends on it.
    pub(crate) fn attach_edges(&mut self, t: sm_mesh_api::EdgeTable) {
        self.edges = Some(t);
        self.reset_edge_caches();
        self.rebuild_class_index();
    }
}

impl MeshEdit for Mesh {
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
        if kind == FaceKind::Boundary
            && !self.dim.accepts_facet(FaceKind::Boundary, self.fclass.ent[f.get()].kind())
        {
            return Err(MeshError::BadClassification(
                "a Boundary face must be classified on a face entity (TopoFace / PartitionFace; edge entities in a dimension-2 mesh)".into(),
            ));
        }
        self.face_kind[f.get()] = kind;
        // `face_kind` is part of the connectivity hash.
        self.cache = None;
        Ok(())
    }
    fn set_high_order(&mut self, c: CellIx, h: Option<HighOrderCell>) -> Result<()> {
        if c.get() >= self.cids.n {
            return Err(MeshError::OutOfRange(format!("{c:?}")));
        }
        let Some(h) = h else {
            self.ho.remove(&c.0);
            return Ok(());
        };
        self.require_edges_for_high_order()?;
        let corners = self
            .cell_corners(c)
            .ok_or_else(|| MeshError::Unsupported("high order needs a template-shaped cell".into()))?;
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
            return Err(MeshError::OutOfRange(format!("expected {want} nodes, got {}", h.nodes.len())));
        }
        if h.nodes.iter().any(|n| n.get() >= self.x.len())
            || h.nodes.iter().zip(corners).any(|(n, v)| n.get() != v.get())
        {
            return Err(MeshError::OutOfRange("nodes must exist and start with the cell corners".into()));
        }
        self.ho.insert(c.0, h);
        Ok(())
    }
    fn set_face_curve(&mut self, f: FaceIx, c: Option<(u8, Basis, Vec<PointIx>)>) -> Result<()> {
        if f.get() >= self.fids.n {
            return Err(MeshError::OutOfRange(format!("{f:?}")));
        }
        let Some((degree, basis, nodes)) = c else {
            self.fcurve.remove(&f.0);
            return Ok(());
        };
        self.require_edges_for_high_order()?;
        let l = &self.face_verts[range(&self.face_off, f.get())];
        let want = face_node_count(l.len(), degree, basis).ok_or_else(|| {
            MeshError::Unsupported(format!(
                "face curve with {} vertices, degree {degree}, {basis:?}",
                l.len()
            ))
        })?;
        if nodes.len() != want {
            return Err(MeshError::OutOfRange(format!("expected {want} nodes, got {}", nodes.len())));
        }
        if nodes.iter().any(|n| n.get() >= self.x.len()) || nodes.iter().zip(l).any(|(n, v)| n.get() != v.get()) {
            return Err(MeshError::OutOfRange(
                "nodes must exist and start with the face loop vertices".into(),
            ));
        }
        self.fcurve.insert(f.0, (degree, basis, nodes));
        Ok(())
    }
    fn add_set(&mut self, name: &str, members: SetMembers, alloc: &mut SerialAlloc) -> Result<EntityId> {
        if self.sets.iter().any(|s| s.name == name) {
            return Err(MeshError::DuplicateName(name.into()));
        }
        let n = match &members {
            SetMembers::Points(_) => self.x.len(),
            m => self.count(m.kind()),
        };
        if matches!(members, SetMembers::ShellSides(_)) && self.dim != MeshDim::Two {
            return Err(MeshError::Unsupported(format!(
                "set '{name}': shell side sets belong to dimension-2 meshes"
            )));
        }
        let raw = members.raw();
        if members.max_index().is_some_and(|i| i as usize >= n) || raw.windows(2).any(|w| w[0] >= w[1]) {
            return Err(MeshError::OutOfRange(format!(
                "set '{name}' members must be ascending, unique and in range"
            )));
        }
        if let SetMembers::Sides(v) = &members
            && v.iter().any(|sf| sf.side() == Side::Right && self.face_right(sf.face()).is_none())
        {
            return Err(MeshError::OutOfRange(format!("set '{name}' names the right side of a one-cell face")));
        }
        let id = alloc.one(EntityKind::MeshSet)?;
        self.sets.push(sm_mesh_api::MeshSet::new(id, name, members));
        self.sets.sort_by_key(|s| s.id);
        Ok(id)
    }
    fn set_label(&mut self, name: &str, label: Option<i64>) -> Result<()> {
        let i = self
            .sets
            .iter()
            .position(|s| s.name == name)
            .ok_or_else(|| MeshError::State(format!("no set '{name}'")))?;
        let kind = self.sets[i].members.kind();
        if label.is_some()
            && self.sets.iter().enumerate().any(|(j, s)| j != i && s.label == label && s.members.kind() == kind)
        {
            return Err(MeshError::DuplicateName(format!("set label {label:?}")));
        }
        self.sets[i].label = label;
        Ok(())
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
        let col = TagColumn::new(name, target, values);
        match self.tags.binary_search_by(|t| (t.target, &t.name).cmp(&(target, &col.name))) {
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
        if self.dim == MeshDim::Two {
            return Err(MeshError::Unsupported(
                "a dimension-2 mesh has no edge table: its facets are the edges".into(),
            ));
        }
        let mut map = BTreeMap::new();
        for (a, b, c) in explicit {
            check_class(c)?;
            map.insert(edge_key(*a, *b), *c);
        }
        let t = crate::edges::derive(self, alloc, &map)?;
        self.attach_edges(t);
        Ok(())
    }
    fn drop_edges(&mut self) -> Result<()> {
        if self.needs_edge_table() {
            return Err(MeshError::State(
                "a high-order dimension-3 mesh keeps its edge table (drop the high-order records first)".into(),
            ));
        }
        self.edges = None;
        self.tags.retain(|t| t.target != MeshEntityKind::Edge);
        self.reset_edge_caches();
        self.rebuild_class_index();
        Ok(())
    }
    fn set_geometry_cache(&mut self, g: Option<GeometryCache>) -> Result<()> {
        if let Some(g) = &g {
            let (nf, nc) = (self.fids.n, self.cids.n);
            let ok = g.rule == self.geometry_rule()
                && g.face_area.len() == nf
                && g.face_centroid.len() == nf
                && g.face_apex.len() == nf
                && g.cell_volume.len() == nc
                && g.cell_centroid.len() == nc
                && g.connectivity_hash == sm_mesh_api::connectivity_hash(self)
                && g.coordinate_hash == sm_mesh_api::coordinate_hash(self);
            if !ok {
                return Err(MeshError::State(
                    "geometry cache does not match this mesh (rule, sizes, connectivity or coordinates)".into(),
                ));
            }
        }
        self.cache = g;
        Ok(())
    }
}
