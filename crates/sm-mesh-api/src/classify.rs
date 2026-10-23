//! Classification: every mesh entity lies on exactly one model or partition entity.
//!
//! The model entity is an [`EntityId`] (`TopoVertex/Edge/Face/Volume` or `PartitionCell`).
//! Boundary sets, materials and deck-boundary lowering are all derived through this table, so a
//! boundary face's "boundary set" is the classification of the face, never a second source of truth.

use crate::index::{CellIx, FaceIx, VertexIx};
use sm_core::{EntityId, EntityKind};

/// Parametric position on the classified model entity (when it has a parametrisation).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub enum Param {
    /// No parametric information (volumes, faces, or not computed).
    #[default]
    None,
    /// Parameter on a curve.
    Curve(f64),
    /// Parameters on a surface.
    Surface([f64; 2]),
}

/// The model/partition entity a mesh entity lies on, with optional parametric coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Classification {
    /// Model (`Topo*`) or partition (`PartitionCell`) entity.
    pub entity: EntityId,
    /// Parametric coordinates on `entity`.
    pub param: Param,
}

impl Classification {
    /// Classification without parametric coordinates.
    pub const fn on(entity: EntityId) -> Self {
        Self { entity, param: Param::None }
    }

    /// Topological dimension of the classified entity (0 vertex, 1 edge, 2 face, 3 volume or
    /// partition cell), or `None` if the id is not a model/partition entity.
    pub const fn dim(&self) -> Option<u8> {
        match self.entity.kind() {
            EntityKind::TopoVertex => Some(0),
            EntityKind::TopoEdge => Some(1),
            EntityKind::TopoFace => Some(2),
            EntityKind::TopoVolume | EntityKind::PartitionCell => Some(3),
            _ => None,
        }
    }
}

/// Reverse classification index: all mesh entities on one model/partition entity, ascending.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ClassSet {
    /// Vertices classified on the entity.
    pub vertices: Vec<VertexIx>,
    /// Faces classified on the entity.
    pub faces: Vec<FaceIx>,
    /// Cells classified on the entity.
    pub cells: Vec<CellIx>,
}
