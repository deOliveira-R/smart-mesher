//! Several meshes over one model: levels with fine->coarse maps, instanced views, flatten and
//! mirror-unfold.
//!
//! A *catalog* holds the meshes of one model. Level 0 is the finest. A coarse mesh is an
//! independent mesh over the same model (its cells are classified on the same volumes); the
//! fine->coarse [`CellMap`] says which coarse cell contains each fine cell (CMFD, multigrid).
//! An [`InstancedView`] stores one prototype mesh plus placements; [`flatten`] expands it
//! deterministically into a single mesh and reports which cells came from which instance.

use crate::build::{MeshBuild, SerialAlloc};
use crate::classify::Classification;
use crate::error::{MeshError, Result};
use crate::highorder::NodeRef;
use crate::index::{CellIx, FaceIx, MeshEntity, Side, VertexIx};
use crate::read::MeshRead;
use sm_core::{EntityId, Transform};
use std::ops::Range;

/// Total map from fine cells to coarse cells, with its inverse as CSR.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellMap {
    coarse_of: Vec<CellIx>,
    off: Vec<u32>,
    fines: Vec<CellIx>,
}

impl CellMap {
    /// Build from the coarse parent of every fine cell. Every coarse cell must have a child.
    pub fn new(coarse_count: usize, coarse_of: Vec<CellIx>) -> Result<Self> {
        let mut counts = vec![0u32; coarse_count + 1];
        for &c in &coarse_of {
            if c.get() >= coarse_count {
                return Err(MeshError::Hierarchy(format!("coarse cell {c:?} out of range")));
            }
            counts[c.get() + 1] += 1;
        }
        if counts[1..].contains(&0) {
            return Err(MeshError::Hierarchy("a coarse cell has no fine children".into()));
        }
        for i in 0..coarse_count {
            counts[i + 1] += counts[i];
        }
        let mut cursor = counts.clone();
        let mut fines = vec![CellIx(0); coarse_of.len()];
        for (f, &c) in coarse_of.iter().enumerate() {
            fines[cursor[c.get()] as usize] = CellIx::new(f);
            cursor[c.get()] += 1;
        }
        Ok(Self { coarse_of, off: counts, fines })
    }
    /// Number of fine cells.
    pub fn fine_len(&self) -> usize {
        self.coarse_of.len()
    }
    /// Number of coarse cells.
    pub fn coarse_len(&self) -> usize {
        self.off.len() - 1
    }
    /// Coarse parent of a fine cell.
    pub fn coarse_of(&self, fine: CellIx) -> CellIx {
        self.coarse_of[fine.get()]
    }
    /// Fine children of a coarse cell, ascending.
    pub fn fines_of(&self, coarse: CellIx) -> &[CellIx] {
        &self.fines[self.off[coarse.get()] as usize..self.off[coarse.get() + 1] as usize]
    }
}

/// Read access to the meshes of one model.
pub trait MeshCatalog {
    /// Ids of all meshes, ascending.
    fn mesh_ids(&self) -> Vec<EntityId>;
    /// A mesh by id.
    fn mesh(&self, id: EntityId) -> Option<&dyn MeshRead>;
    /// Level of a mesh (0 = finest).
    fn level(&self, id: EntityId) -> Option<u32>;
    /// The coarse mesh linked above `fine`, with the fine->coarse map.
    fn coarse_of(&self, fine: EntityId) -> Option<(EntityId, &CellMap)>;
    /// Instanced views stored in the catalog.
    fn views(&self) -> &[InstancedView];
}

/// Write access to a catalog.
pub trait CatalogEdit: MeshCatalog {
    /// Insert a mesh at a level. Ids must be new.
    fn insert_mesh(&mut self, mesh: Box<dyn MeshRead>, level: u32) -> Result<()>;
    /// Link `coarse` above `fine` (levels must be consecutive; map and classification must agree).
    fn link_levels(&mut self, fine: EntityId, coarse: EntityId, map: CellMap) -> Result<()>;
    /// Add an instanced view; every prototype must already be in the catalog.
    fn insert_view(&mut self, view: InstancedView) -> Result<()>;
}

/// One placement of a prototype mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct Instance {
    /// Stable id (`EntityKind::Instance`); flatten order is ascending id.
    pub id: EntityId,
    /// Prototype mesh id (`EntityKind::Mesh`).
    pub prototype: EntityId,
    /// Placement; may be improper (mirrored copy).
    pub transform: Transform,
}

/// An instanced view: prototypes live in the catalog, the view only lists placements.
#[derive(Clone, Debug, PartialEq)]
pub struct InstancedView {
    /// Stable id of the view.
    pub id: EntityId,
    /// Placements.
    pub instances: Vec<Instance>,
}

/// Which flattened cells came from which instance.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FlattenMap {
    /// `(instance, cell range)` in flatten order (ascending instance id, ascending cell index).
    pub ranges: Vec<(EntityId, Range<u32>)>,
}

impl FlattenMap {
    /// The instance and prototype-local cell index of a flattened cell.
    pub fn locate(&self, c: CellIx) -> Option<(EntityId, CellIx)> {
        let i = self.ranges.partition_point(|(_, r)| r.end <= c.0);
        let (id, r) = self.ranges.get(i)?;
        (r.start <= c.0).then(|| (*id, CellIx(c.0 - r.start)))
    }
}

/// Options for [`append_transformed`].
#[derive(Default)]
pub struct AppendOptions<'a> {
    /// Merge vertices within this distance with vertices already in the builder; faces with the
    /// same vertex cycle are always shared. `None` keeps copies disjoint.
    pub weld_tol: Option<f64>,
    /// Maps source classification to the classification in the target model (e.g. mirror image
    /// of a face in the unfolded model). Identity when `None`.
    pub remap_class: Option<&'a dyn Fn(Classification) -> Classification>,
}

/// Where source entities landed in the builder.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AppendMap {
    /// Builder vertex for each source vertex.
    pub vertex: Vec<VertexIx>,
    /// Builder (provisional) face for each source face.
    pub face: Vec<FaceIx>,
    /// Builder cell for each source cell.
    pub cell: Vec<CellIx>,
}

/// Append a transformed copy of `src` to a builder.
///
/// Orientation is rebuilt from outward loops, so an **improper** transform (mirror, `det < 0`)
/// reverses every face loop and permutes fast-path vertex orders
/// ([`crate::CellShape::reflect_permutation`]); the builder's `finish` then re-applies the
/// left/right rule. High-order cells are only supported for proper transforms.
pub fn append_transformed<B: MeshBuild>(b: &mut B, src: &dyn MeshRead, t: &Transform, o: &AppendOptions<'_>) -> Result<AppendMap> {
    let remap = |c: Classification| o.remap_class.map_or(c, |f| f(c));
    let improper = t.is_improper();
    let (nv, co) = (src.vertex_count(), src.coords());
    let mut vmap = Vec::with_capacity(nv);
    for i in 0..nv {
        let p = t.apply_point(co.get(i));
        if let Some(v) = o.weld_tol.and_then(|tol| b.find_vertex(p, tol)) {
            vmap.push(v);
        } else {
            let v = b.add_vertex(p)?;
            b.set_class(MeshEntity::Vertex(v), remap(src.vertex_class(VertexIx::new(i))))?;
            vmap.push(v);
        }
    }
    let mut extra = Vec::new();
    for i in nv..co.len() {
        extra.push(b.add_extra_node(t.apply_point(co.get(i)))?);
    }
    let mut fmap = vec![FaceIx(FaceIx::NONE); src.face_count()];
    let mut cmap = Vec::with_capacity(src.cell_count());
    for ci in 0..src.cell_count() {
        let c = CellIx::new(ci);
        let cfs = src.cell_faces(c);
        let loops: Vec<Vec<VertexIx>> = cfs
            .iter()
            .map(|cf| {
                let mut l: Vec<VertexIx> = src.face_vertices(cf.face).iter().map(|v| vmap[v.get()]).collect();
                if (cf.side == Side::Right) != improper {
                    l.reverse();
                }
                l
            })
            .collect();
        let ordered: Option<Vec<VertexIx>> = src.cell_ordered_vertices(c).map(|ov| {
            let mapped: Vec<VertexIx> = ov.iter().map(|v| vmap[v.get()]).collect();
            match (improper, src.cell_shape(c).reflect_permutation()) {
                (true, Some(p)) => p.iter().map(|&k| mapped[k as usize]).collect(),
                _ => mapped,
            }
        });
        let before = b.face_count();
        let nc = b.add_polyhedron_ordered(&loops, ordered.as_deref())?;
        for (cf, l) in cfs.iter().zip(&loops) {
            let (f, _) = b.find_face(l).ok_or_else(|| MeshError::State("face vanished after add".into()))?;
            fmap[cf.face.get()] = f;
            if f.get() >= before {
                b.set_class(MeshEntity::Face(f), remap(src.face_class(cf.face)))?;
            }
        }
        b.set_class(MeshEntity::Cell(nc), remap(src.cell_class(c)))?;
        if let Some(h) = src.high_order_cell(c) {
            if improper {
                return Err(MeshError::State("high-order cells cannot be mirrored".into()));
            }
            let nodes: Vec<NodeRef> = h
                .nodes
                .iter()
                .map(|&n| if (n as usize) < nv { NodeRef::Vertex(vmap[n as usize]) } else { extra[n as usize - nv] })
                .collect();
            b.set_high_order(nc, h.degree, h.basis, &nodes)?;
        }
        cmap.push(nc);
    }
    Ok(AppendMap { vertex: vmap, face: fmap, cell: cmap })
}

/// Expand an instanced view into one mesh. Instances are placed in ascending id order, so the
/// result is deterministic. With `weld_tol`, abutting instances share vertices and faces
/// (a conforming flatten); otherwise they stay disjoint and meet through interface records.
/// Sets and tags are not carried (use the returned [`FlattenMap`] to re-derive them).
pub fn flatten<B: MeshBuild>(
    view: &InstancedView,
    catalog: &dyn MeshCatalog,
    mut b: B,
    weld_tol: Option<f64>,
    alloc: &mut SerialAlloc,
) -> Result<(B::Output, FlattenMap)> {
    let mut insts: Vec<&Instance> = view.instances.iter().collect();
    insts.sort_by_key(|i| i.id);
    let mut ranges = Vec::with_capacity(insts.len());
    let opts = AppendOptions { weld_tol, remap_class: None };
    for inst in insts {
        let proto = catalog.mesh(inst.prototype).ok_or(MeshError::UnknownMesh(inst.prototype))?;
        let start = b.cell_count() as u32;
        append_transformed(&mut b, proto, &inst.transform, &opts)?;
        ranges.push((inst.id, start..b.cell_count() as u32));
    }
    Ok((b.finish(alloc)?, FlattenMap { ranges }))
}

/// Unfold a reduced mesh by a deck transformation `t` (typically a mirror): the result holds the
/// original plus its image, welded where they touch, with orientation re-derived.
pub fn unfold<B: MeshBuild>(
    src: &dyn MeshRead,
    t: &Transform,
    weld_tol: f64,
    remap_class: Option<&dyn Fn(Classification) -> Classification>,
    mut b: B,
    alloc: &mut SerialAlloc,
) -> Result<B::Output> {
    append_transformed(&mut b, src, &Transform::IDENTITY, &AppendOptions::default())?;
    append_transformed(&mut b, src, t, &AppendOptions { weld_tol: Some(weld_tol), remap_class })?;
    b.finish(alloc)
}
