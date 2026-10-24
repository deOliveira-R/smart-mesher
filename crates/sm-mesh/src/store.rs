//! The finished mesh store.

use crate::adjacency::Caches;
use crate::class::{ClassCol, build_index};
use crate::util::range;
use sm_core::{EntityId, EntityKind};
use sm_mesh_api::{
    Basis, CellIx, CellShape, ClassSet, Classification, Coords, EdgeTable, FaceCurve, FaceIx,
    FaceKind, GeometryCache, HighOrderCell, MeshDim, MeshEntityKind, MeshRead, MeshSet, PointIx,
    SignedFace, TagColumn, TopologyColumns, VertexIx,
};
use std::collections::BTreeMap;

/// First id and count of one entity kind: ids are consecutive serials, so the id column is
/// implicit (no per-entity storage) and `index_of` is `O(1)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct IdRange {
    pub(crate) first: EntityId,
    pub(crate) n: usize,
}

impl IdRange {
    pub(crate) fn id(&self, ix: usize) -> EntityId {
        EntityId::from_raw(self.first.to_raw() + ix as u64).unwrap_or(self.first)
    }
    pub(crate) fn index_of(&self, id: EntityId) -> Option<u32> {
        let d = id.serial().checked_sub(self.first.serial())?;
        (id.kind() == self.first.kind() && (d as usize) < self.n && self.n > 0).then_some(d as u32)
    }
}

/// A finished, canonical face-first mesh in structure-of-arrays / CSR layout.
///
/// Created by [`sm_mesh_api::MeshBuild::finish`] on a [`crate::MeshBuilder`]; implements [`MeshRead`] and
/// [`sm_mesh_api::MeshEdit`]. All topology lives in flat columns (see [`crate::RawColumns`]);
/// stable ids are implicit consecutive ranges; classification is a dense `u64` column per entity
/// kind plus a sparse parameter map; adjacency tables are built on demand and cached.
#[derive(Clone, Debug, PartialEq)]
pub struct Mesh {
    pub(crate) dim: MeshDim,
    pub(crate) id: EntityId,
    pub(crate) vids: IdRange,
    pub(crate) fids: IdRange,
    pub(crate) cids: IdRange,
    pub(crate) x: Vec<f64>,
    pub(crate) y: Vec<f64>,
    pub(crate) z: Vec<f64>,
    pub(crate) face_off: Vec<u64>,
    pub(crate) face_verts: Vec<VertexIx>,
    pub(crate) face_left: Vec<CellIx>,
    pub(crate) face_right: Vec<u32>,
    pub(crate) face_kind: Vec<FaceKind>,
    pub(crate) cell_off: Vec<u64>,
    pub(crate) cell_faces: Vec<SignedFace>,
    pub(crate) cell_shape: Vec<CellShape>,
    pub(crate) corner_off: Vec<u64>,
    pub(crate) corners: Vec<VertexIx>,
    pub(crate) cell_parent: Vec<u32>,
    pub(crate) cell_active: Vec<bool>,
    pub(crate) vclass: ClassCol,
    pub(crate) fclass: ClassCol,
    pub(crate) cclass: ClassCol,
    pub(crate) class_index: BTreeMap<EntityId, ClassSet>,
    pub(crate) class_entities: Vec<EntityId>,
    pub(crate) ho: BTreeMap<u32, HighOrderCell>,
    pub(crate) fcurve: BTreeMap<u32, (u8, Basis, Vec<PointIx>)>,
    pub(crate) sets: Vec<MeshSet>,
    pub(crate) tags: Vec<TagColumn>,
    pub(crate) edges: Option<EdgeTable>,
    pub(crate) cache: Option<GeometryCache>,
    pub(crate) adj: Caches,
}

impl Mesh {
    /// Number of vertices.
    pub fn n_vertices(&self) -> usize {
        self.vids.n
    }
    /// Number of faces.
    pub fn n_faces(&self) -> usize {
        self.fids.n
    }
    /// Number of cells.
    pub fn n_cells(&self) -> usize {
        self.cids.n
    }

    /// Heap bytes held by the topology, coordinate and classification columns, the edge table and
    /// the reverse class index (lengths, not capacities; the lazily built adjacency tables, sets,
    /// tags and high-order records are not counted). A density figure for reports and budgets.
    pub fn memory_bytes(&self) -> usize {
        use std::mem::size_of_val as sz;
        let class = |c: &ClassCol| sz(&c.ent[..]) + c.param.len() * 40;
        let edges = self.edges.as_ref().map_or(0, |t| {
            sz(&t.ids[..]) + sz(&t.vertices[..]) + sz(&t.face_off[..]) + sz(&t.face_edges[..]) + sz(&t.class[..])
        });
        let index: usize = self
            .class_index
            .values()
            .map(|c| 4 * (c.vertices.len() + c.edges.len() + c.faces.len() + c.cells.len()))
            .sum();
        sz(&self.x[..]) * 3
            + sz(&self.face_off[..])
            + sz(&self.face_verts[..])
            + sz(&self.face_left[..])
            + sz(&self.face_right[..])
            + sz(&self.face_kind[..])
            + sz(&self.cell_off[..])
            + sz(&self.cell_faces[..])
            + sz(&self.cell_shape[..])
            + sz(&self.corner_off[..])
            + sz(&self.corners[..])
            + sz(&self.cell_parent[..])
            + sz(&self.cell_active[..])
            + class(&self.vclass)
            + class(&self.fclass)
            + class(&self.cclass)
            + edges
            + index
    }

    /// Recompute the reverse classification index (after classification or edge-table edits).
    pub(crate) fn rebuild_class_index(&mut self) {
        let (idx, ents) = build_index(&self.vclass, &self.fclass, &self.cclass, self.edges.as_ref());
        self.class_index = idx;
        self.class_entities = ents;
    }

    /// True for a dimension-3 mesh with high-order cells or face curves (edge-slot nodes).
    pub(crate) fn needs_edge_table(&self) -> bool {
        self.dim == MeshDim::Three && (!self.ho.is_empty() || !self.fcurve.is_empty())
    }
}

impl MeshRead for Mesh {
    fn mesh_id(&self) -> EntityId {
        self.id
    }
    fn dim(&self) -> MeshDim {
        self.dim
    }
    fn count(&self, k: MeshEntityKind) -> usize {
        match k {
            MeshEntityKind::Vertex => self.vids.n,
            MeshEntityKind::Edge => self.edges.as_ref().map_or(0, EdgeTable::len),
            MeshEntityKind::Face => self.fids.n,
            MeshEntityKind::Cell => self.cids.n,
            _ => 0,
        }
    }
    fn id(&self, k: MeshEntityKind, ix: u32) -> EntityId {
        match k {
            MeshEntityKind::Vertex => self.vids.id(ix as usize),
            MeshEntityKind::Face => self.fids.id(ix as usize),
            MeshEntityKind::Cell => self.cids.id(ix as usize),
            _ => self.edges.as_ref().map_or(self.id, |t| t.ids[ix as usize]),
        }
    }
    fn index_of(&self, id: EntityId) -> Option<u32> {
        match id.kind() {
            EntityKind::MeshVertex => self.vids.index_of(id),
            EntityKind::MeshEdge => {
                let ids = &self.edges.as_ref()?.ids;
                ids.binary_search(&id).ok().map(|i| i as u32)
            }
            EntityKind::MeshFace => self.fids.index_of(id),
            EntityKind::MeshCell => self.cids.index_of(id),
            _ => None,
        }
    }
    fn coords(&self) -> Coords<'_> {
        Coords::new(&self.x, &self.y, &self.z)
    }
    fn columns(&self) -> TopologyColumns<'_> {
        TopologyColumns::new(
            &self.face_off,
            &self.face_verts,
            &self.face_left,
            &self.face_right,
            &self.cell_off,
            &self.cell_faces,
        )
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
    fn cell_faces(&self, c: CellIx) -> &[SignedFace] {
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
        self.fcurve.get(&f.0).map(|(degree, basis, nodes)| FaceCurve {
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
            MeshEntityKind::Vertex => self.vclass.get(ix as usize),
            MeshEntityKind::Face => self.fclass.get(ix as usize),
            MeshEntityKind::Cell => self.cclass.get(ix as usize),
            _ => self.edges.as_ref().map_or(Classification::on(self.id), |t| t.class[ix as usize]),
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
