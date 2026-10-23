//! Instanced view (prototype meshes + placed instances) and deterministic `flatten`.
//!
//! Flattening replays each instance, in order, through the target [`MeshEdit`]: every cell is
//! re-emitted with outward-wound loops, so the orientation rule (lower id = left) is
//! re-applied by the builder. For an improper transform (mirror, `det < 0`) the transformed
//! loops would wind inward; flatten reverses them, which is exactly the face-loop flip of
//! docs/design/ir.md §5.1.

use crate::error::MeshError;
use crate::hierarchy::MeshHandle;
use crate::point::{Point, Stratum};
use crate::traits::{CellSpec, FaceLoop, MeshEdit, MeshView};
use indexmap::IndexMap;
use sm_core::{EntityId, EntityKind, Transform};
use smallvec::SmallVec;
use std::collections::{BTreeMap, BTreeSet};
use std::ops::Range;

/// A placed copy of a prototype mesh.
#[derive(Clone, Debug)]
pub struct Instance {
    /// Id of the instance (kind `Instance`).
    pub id: EntityId,
    /// Id of the prototype mesh (kind `Mesh`).
    pub prototype: EntityId,
    /// Placement; may be improper.
    pub transform: Transform,
    /// If `Some(tol)`, vertices the transform leaves fixed (`|T p - p| <= tol`, e.g. on a
    /// mirror plane or rotation axis) are shared between all instances of the prototype, so
    /// the unfolded copies are stitched along a deck boundary. `None` keeps copies disjoint.
    pub weld_tol: Option<f64>,
}

/// Where a flattened cell came from.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellOrigin {
    /// The instance.
    pub instance: EntityId,
    /// The cell in the prototype mesh.
    pub proto_cell: Point,
}

/// Instance-path -> id-range map produced by flatten.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct FlattenMap {
    /// Origin of each flattened cell, indexed by flattened cell index.
    pub cell_origin: Vec<CellOrigin>,
    /// Contiguous flattened cell-index range of each instance, in instance order.
    pub instance_cells: Vec<(EntityId, Range<u32>)>,
}

/// Prototypes plus instances (one level; nest by flattening a prototype first).
#[derive(Default)]
pub struct InstancedView {
    prototypes: IndexMap<EntityId, MeshHandle>,
    instances: Vec<Instance>,
}

impl InstancedView {
    /// An empty view.
    pub fn new() -> Self {
        Self::default()
    }

    /// Register a prototype under its `mesh_id`.
    pub fn add_prototype(&mut self, mesh: MeshHandle) -> EntityId {
        let id = mesh.mesh_id();
        self.prototypes.insert(id, mesh);
        id
    }

    /// Add an instance. Order of addition is the order of flattening.
    pub fn add_instance(&mut self, inst: Instance) -> Result<(), MeshError> {
        if inst.id.kind() != EntityKind::Instance {
            return Err(MeshError::InvalidInstance("id is not of kind Instance".into()));
        }
        if !self.prototypes.contains_key(&inst.prototype) {
            return Err(MeshError::UnknownMesh(inst.prototype));
        }
        self.instances.push(inst);
        Ok(())
    }

    /// Instances in flatten order.
    pub fn instances(&self) -> &[Instance] {
        &self.instances
    }

    /// Deterministically flatten into one mesh built with `builder`.
    ///
    /// Classification and labels of the first copy of each entity are carried over (a
    /// welded face shared by two copies keeps the first copy's data). High-order cells are
    /// not supported yet.
    pub fn flatten<B: MeshEdit>(&self, mut builder: B) -> Result<(B::Output, FlattenMap), MeshError> {
        let mut map = FlattenMap::default();
        let mut fixed: BTreeMap<(EntityId, u32), Point> = BTreeMap::new();
        let mut seen: BTreeSet<Point> = BTreeSet::new();
        let mut n_cells = 0u32;
        for inst in &self.instances {
            let proto: &dyn MeshView = self.prototypes[&inst.prototype].as_ref();
            let improper = inst.transform.is_improper();
            let labels: Vec<(String, BTreeMap<Point, i32>)> = proto
                .label_names()
                .into_iter()
                .map(|n| {
                    let e = proto.label_entries(&n).into_iter().collect();
                    (n, e)
                })
                .collect();
            let apply_labels = |b: &mut B, src: Point, dst: Point| -> Result<(), MeshError> {
                for (name, e) in &labels {
                    if let Some(&v) = e.get(&src) {
                        b.set_label(name, dst, v)?;
                    }
                }
                Ok(())
            };
            let nv = proto.count(Stratum::Vertex);
            let mut vmap = Vec::with_capacity(nv as usize);
            for v in 0..nv {
                let x = proto.vertex_coord(v);
                let y = inst.transform.apply_point(x);
                let pv = Point::vertex(v);
                let is_fixed = inst.weld_tol.is_some_and(|tol| (y - x).norm() <= tol);
                let np = match fixed.get(&(inst.prototype, v)) {
                    Some(&p) if is_fixed => p,
                    _ => {
                        let p = builder.add_vertex(y, proto.classification(pv));
                        if is_fixed {
                            fixed.insert((inst.prototype, v), p);
                        }
                        p
                    }
                };
                if seen.insert(np) {
                    apply_labels(&mut builder, pv, np)?;
                }
                vmap.push(np);
            }
            let first = n_cells;
            for c in Point::range(Stratum::Cell, proto.count(Stratum::Cell)) {
                if proto.cell_geometry(c).degree != 1 {
                    return Err(MeshError::Unsupported("flatten of high-order cells"));
                }
                let cone = proto.cone(c);
                let mut faces: SmallVec<[FaceLoop; 6]> = SmallVec::new();
                for (f, o) in cone.iter() {
                    // Outward loop of the prototype; an improper map winds it inward, so reverse.
                    let o = if improper { o.flip() } else { o };
                    let l = proto.outward_loop(f, o);
                    faces.push(l.iter().map(|&v| vmap[v as usize]).collect());
                }
                let spec = CellSpec { shape: proto.cell_shape(c), faces, class: proto.classification(c) };
                let added = builder.add_cell(&spec)?;
                for ((f, _), &nf) in cone.iter().zip(added.faces.iter()) {
                    if seen.insert(nf) {
                        if let Some(cl) = proto.classification(f) {
                            builder.classify(nf, cl)?;
                        }
                        apply_labels(&mut builder, f, nf)?;
                    }
                }
                seen.insert(added.cell);
                apply_labels(&mut builder, c, added.cell)?;
                map.cell_origin.push(CellOrigin { instance: inst.id, proto_cell: c });
                n_cells += 1;
            }
            map.instance_cells.push((inst.id, first..n_cells));
        }
        Ok((builder.finish()?, map))
    }
}
