//! Several meshes over one model: fine/coarse levels with inter-mesh maps, and the instanced
//! view with a deterministic `flatten`.

use std::collections::BTreeMap;
use std::ops::Range;

use indexmap::IndexMap;
use sm_core::{EntityId, EntityKind, Point3, Transform};

use crate::build::MeshBuilder;
use crate::classify::Classification;
use crate::error::{MeshError, MeshResult};
use crate::index::{CellIx, Dim, VertexIx};
use crate::query::MeshRead;
use crate::schema::TagDef;
use crate::shape::CellShape;
use crate::table::Csr;

/// Map from the cells of a fine mesh to the cells of a coarse mesh (CMFD, multigrid).
#[derive(Clone, Debug, PartialEq)]
pub struct LevelMap {
    fine: EntityId,
    coarse: EntityId,
    fine_to_coarse: Vec<CellIx>,
    coarse_to_fine: Csr<CellIx>,
}

impl LevelMap {
    /// Build a map; the inverse CSR is derived by counting sort so each coarse cell lists its
    /// fine cells ascending. Errors if a coarse index is out of range or a coarse cell is empty.
    pub fn new(fine: EntityId, coarse: EntityId, fine_to_coarse: Vec<CellIx>, num_coarse: usize) -> MeshResult<Self> {
        if fine.kind() != EntityKind::Mesh || coarse.kind() != EntityKind::Mesh {
            return Err(MeshError::WrongKind(if fine.kind() != EntityKind::Mesh { fine } else { coarse }));
        }
        let mut rows: Vec<Vec<CellIx>> = vec![Vec::new(); num_coarse];
        for (i, c) in fine_to_coarse.iter().enumerate() {
            rows.get_mut(c.index())
                .ok_or(MeshError::OutOfRange { dim: Dim::Cell, index: c.0, len: num_coarse })?
                .push(CellIx(i as u32));
        }
        if let Some(e) = rows.iter().position(Vec::is_empty) {
            return Err(MeshError::LevelMap(format!("coarse cell {e} has no fine cells")));
        }
        Ok(Self { fine, coarse, fine_to_coarse, coarse_to_fine: Csr::from_rows(rows) })
    }
    /// Id of the fine mesh.
    pub fn fine(&self) -> EntityId {
        self.fine
    }
    /// Id of the coarse mesh.
    pub fn coarse(&self) -> EntityId {
        self.coarse
    }
    /// Coarse cell containing fine cell `f`.
    pub fn coarse_of(&self, f: CellIx) -> CellIx {
        self.fine_to_coarse[f.index()]
    }
    /// Fine cells of coarse cell `c`, ascending.
    pub fn fines_of(&self, c: CellIx) -> &[CellIx] {
        self.coarse_to_fine.row(c.index())
    }
    /// Number of fine cells.
    pub fn num_fine(&self) -> usize {
        self.fine_to_coarse.len()
    }
    /// Number of coarse cells.
    pub fn num_coarse(&self) -> usize {
        self.coarse_to_fine.len()
    }
    /// Sum a fine-cell field into coarse cells (fixed ascending order, deterministic).
    pub fn restrict_sum(&self, fine_values: &[f64]) -> MeshResult<Vec<f64>> {
        if fine_values.len() != self.num_fine() {
            return Err(MeshError::LevelMap("field length differs from fine cell count".into()));
        }
        Ok(self.coarse_to_fine.rows().map(|r| r.iter().map(|f| fine_values[f.index()]).sum()).collect())
    }
    /// Copy a coarse-cell field down to the fine cells.
    pub fn prolong_copy(&self, coarse_values: &[f64]) -> MeshResult<Vec<f64>> {
        if coarse_values.len() != self.num_coarse() {
            return Err(MeshError::LevelMap("field length differs from coarse cell count".into()));
        }
        Ok(self.fine_to_coarse.iter().map(|c| coarse_values[c.index()]).collect())
    }
    /// Compose `self` (fine to mid) with `next` (mid to coarse).
    pub fn compose(&self, next: &LevelMap) -> MeshResult<LevelMap> {
        if self.coarse != next.fine {
            return Err(MeshError::LevelMap("maps do not chain".into()));
        }
        let f2c = self.fine_to_coarse.iter().map(|&m| next.coarse_of(m)).collect();
        LevelMap::new(self.fine, next.coarse, f2c, next.num_coarse())
    }
}

/// Meshes of one model, finest first, with maps between consecutive or arbitrary levels.
pub struct Hierarchy<M> {
    levels: IndexMap<EntityId, M>,
    maps: Vec<LevelMap>,
}

impl<M: MeshRead> Default for Hierarchy<M> {
    fn default() -> Self {
        Self { levels: IndexMap::new(), maps: Vec::new() }
    }
}

impl<M: MeshRead> Hierarchy<M> {
    /// An empty hierarchy.
    pub fn new() -> Self {
        Self::default()
    }
    /// Add a level. The first level added is the finest; ids must be unique.
    pub fn add_level(&mut self, mesh: M) -> MeshResult<()> {
        let id = mesh.mesh_id();
        if self.levels.contains_key(&id) {
            return Err(MeshError::DuplicateName(format!("{id:?}")));
        }
        self.levels.insert(id, mesh);
        Ok(())
    }
    /// Add a map; both meshes must exist and the map sizes must match their cell counts.
    pub fn add_map(&mut self, map: LevelMap) -> MeshResult<()> {
        let (f, c) = (self.level(map.fine()), self.level(map.coarse()));
        match (f, c) {
            (Some(f), Some(c)) if f.len(Dim::Cell) == map.num_fine() && c.len(Dim::Cell) == map.num_coarse() => {
                self.maps.push(map);
                Ok(())
            }
            _ => Err(MeshError::LevelMap("map does not match its meshes".into())),
        }
    }
    /// A level by mesh id.
    pub fn level(&self, id: EntityId) -> Option<&M> {
        self.levels.get(&id)
    }
    /// Mesh ids finest first.
    pub fn level_ids(&self) -> Vec<EntityId> {
        self.levels.keys().copied().collect()
    }
    /// The map from `fine` to `coarse`, if registered.
    pub fn map(&self, fine: EntityId, coarse: EntityId) -> Option<&LevelMap> {
        self.maps.iter().find(|m| m.fine() == fine && m.coarse() == coarse)
    }
}

/// One placement of a prototype mesh.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Instance {
    /// Instance id (kind `Instance`); instances flatten in ascending id order.
    pub id: EntityId,
    /// Prototype mesh id.
    pub prototype: EntityId,
    /// Placement; may be improper (mirror).
    pub transform: Transform,
}

/// Prototypes plus an instance table (the instanced view).
pub struct InstancedMesh<M> {
    prototypes: IndexMap<EntityId, M>,
    instances: BTreeMap<EntityId, Instance>,
}

impl<M: MeshRead> Default for InstancedMesh<M> {
    fn default() -> Self {
        Self { prototypes: IndexMap::new(), instances: BTreeMap::new() }
    }
}

/// Options for [`InstancedMesh::flatten_into`].
#[derive(Clone, Copy, Debug, Default)]
pub struct FlattenOptions {
    /// Merge vertices whose transformed coordinates are bit-identical, so coincident faces
    /// between instances (e.g. on a mirror plane) become shared interior faces.
    pub weld_exact: bool,
}

/// Where one instance landed in the flat mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct InstanceRange {
    /// The instance.
    pub instance: EntityId,
    /// Its cells in the flat mesh.
    pub cells: Range<u32>,
    /// Prototype vertex index to flat vertex index.
    pub vertex_map: Vec<VertexIx>,
}

/// Result details of a flatten: the instance-path to id-range map and a loss report.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FlattenMap {
    /// One range per instance, ascending by instance id.
    pub ranges: Vec<InstanceRange>,
    /// What was not carried over (differentiator F: every lowering reports its loss).
    pub loss: Vec<String>,
}

impl<M: MeshRead> InstancedMesh<M> {
    /// An empty instanced view.
    pub fn new() -> Self {
        Self::default()
    }
    /// Register a prototype mesh (keyed by its mesh id).
    pub fn add_prototype(&mut self, mesh: M) -> MeshResult<()> {
        let id = mesh.mesh_id();
        if self.prototypes.contains_key(&id) {
            return Err(MeshError::DuplicateName(format!("{id:?}")));
        }
        self.prototypes.insert(id, mesh);
        Ok(())
    }
    /// Place a prototype.
    pub fn add_instance(&mut self, inst: Instance) -> MeshResult<()> {
        if inst.id.kind() != EntityKind::Instance {
            return Err(MeshError::WrongKind(inst.id));
        }
        if !self.prototypes.contains_key(&inst.prototype) {
            return Err(MeshError::UnknownName(format!("{:?}", inst.prototype)));
        }
        if self.instances.insert(inst.id, inst).is_some() {
            return Err(MeshError::DuplicateName(format!("{:?}", inst.id)));
        }
        Ok(())
    }
    /// Instances in canonical (ascending id) order.
    pub fn instances(&self) -> impl Iterator<Item = &Instance> {
        self.instances.values()
    }
    /// Prototype by id.
    pub fn prototype(&self, id: EntityId) -> Option<&M> {
        self.prototypes.get(&id)
    }

    /// Deterministically flatten into one mesh by replaying every instance, in ascending
    /// instance id, through `builder`. Improper transforms flip face loops (and permute template
    /// corners) so the orientation rule holds on the mapped geometry.
    ///
    /// Carried over: shape/degree, cell and vertex classification, edge/face classification
    /// (last instance wins on welded faces), cell tags. Dropped (and reported): sets, vertex
    /// tags, edge/face tags, curved faces.
    pub fn flatten_into<B: MeshBuilder>(&self, mut builder: B, opts: FlattenOptions) -> MeshResult<(B::Mesh, FlattenMap)> {
        let mut map = FlattenMap::default();
        let mut weld: BTreeMap<[u64; 3], VertexIx> = BTreeMap::new();
        let mut defined: Vec<TagDef> = Vec::new();
        let mut next_cell = 0u32;
        let mut loss_notes: Vec<String> = Vec::new();

        for inst in self.instances.values() {
            let Some(proto) = self.prototypes.get(&inst.prototype) else {
                return Err(MeshError::UnknownName(format!("{:?}", inst.prototype)));
            };
            let improper = inst.transform.is_improper();
            note_losses(proto, &mut loss_notes);

            let nv = proto.len(Dim::Vertex);
            let mut vmap = Vec::with_capacity(nv);
            for i in 0..nv as u32 {
                let p = inst.transform.apply_point(proto.point(VertexIx(i)));
                let key = [(p.x + 0.0).to_bits(), (p.y + 0.0).to_bits(), (p.z + 0.0).to_bits()];
                let existing = if opts.weld_exact { weld.get(&key).copied() } else { None };
                let v = match existing {
                    Some(v) => v,
                    None => {
                        let v = builder.add_vertex(Point3::new(p.x, p.y, p.z));
                        if opts.weld_exact {
                            weld.insert(key, v);
                        }
                        if let Some(c) = proto.classification(Dim::Vertex, i) {
                            builder.classify_vertex(v, c)?;
                        }
                        v
                    }
                };
                vmap.push(v);
            }

            for def in proto.schema().tags.iter().filter(|t| t.dim == Dim::Cell) {
                match defined.iter().find(|d| d.name == def.name) {
                    Some(d) if d != def => return Err(MeshError::TagMismatch(def.name.clone())),
                    Some(_) => {}
                    None => {
                        builder.define_tag(def.clone())?;
                        defined.push(def.clone());
                    }
                }
            }

            let first_cell = next_cell;
            for ci in 0..proto.len(Dim::Cell) as u32 {
                let c = CellIx(ci);
                let spec = proto.cell_spec(c);
                let new = match spec.shape {
                    CellShape::Polyhedron => {
                        let loops: Vec<Vec<VertexIx>> = proto
                            .cell_faces(c)
                            .iter()
                            .map(|sf| {
                                let l = proto.face_vertices(sf.face());
                                let flip = (sf.sign() == crate::index::Sign::Minus) != improper;
                                let it: Vec<VertexIx> = if flip {
                                    l.iter().rev().map(|v| vmap[v.index()]).collect()
                                } else {
                                    l.iter().map(|v| vmap[v.index()]).collect()
                                };
                                it
                            })
                            .collect();
                        let refs: Vec<&[VertexIx]> = loops.iter().map(Vec::as_slice).collect();
                        builder.add_polyhedron(&refs)?
                    }
                    shape => {
                        let nodes = proto.cell_nodes(c);
                        let mut mapped: Vec<VertexIx> = nodes.iter().map(|v| vmap[v.index()]).collect();
                        if improper {
                            if spec.degree > 1 {
                                return Err(MeshError::Unsupported("improper transform of a high-order cell"));
                            }
                            let sigma = shape.mirror_permutation().ok_or(MeshError::Unsupported("no mirror permutation"))?;
                            mapped = sigma.iter().map(|&s| mapped[s as usize]).collect();
                        }
                        if spec.degree > 1 { builder.add_high_order_cell(spec, &mapped)? } else { builder.add_cell(shape, &mapped)? }
                    }
                };
                next_cell = new.0 + 1;
                if let Some(Classification { entity, .. }) = proto.classification(Dim::Cell, ci) {
                    builder.classify_cell(new, entity)?;
                }
                for d in &defined {
                    if let Some(row) = proto.tag_row(Dim::Cell, &d.name, ci) {
                        builder.set_tag(Dim::Cell, &d.name, new.0, row)?;
                    }
                }
            }

            for f in 0..proto.len(Dim::Face) as u32 {
                if let Some(c) = proto.classification(Dim::Face, f) {
                    let l: Vec<VertexIx> = proto.face_vertices(crate::index::FaceIx(f)).iter().map(|v| vmap[v.index()]).collect();
                    builder.classify_face(&l, c)?;
                }
            }
            for e in 0..proto.len(Dim::Edge) as u32 {
                if let Some(c) = proto.classification(Dim::Edge, e) {
                    let [a, b] = proto.edge_vertices(crate::index::EdgeIx(e));
                    builder.classify_edge(vmap[a.index()], vmap[b.index()], c)?;
                }
            }
            map.ranges.push(InstanceRange { instance: inst.id, cells: first_cell..next_cell, vertex_map: vmap });
        }
        loss_notes.sort();
        loss_notes.dedup();
        map.loss = loss_notes;
        Ok((builder.finish()?, map))
    }
}

fn note_losses<M: MeshRead>(proto: &M, notes: &mut Vec<String>) {
    let s = proto.schema();
    if !s.sets.is_empty() {
        notes.push("sets dropped".into());
    }
    if s.tags.iter().any(|t| t.dim != Dim::Cell) {
        notes.push("vertex/edge/face tags dropped".into());
    }
    if (0..proto.len(Dim::Face) as u32).any(|f| proto.face_curve(crate::index::FaceIx(f)).is_some()) {
        notes.push("curved faces dropped".into());
    }
}
