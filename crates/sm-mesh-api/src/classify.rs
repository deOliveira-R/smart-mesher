//! Classification: every mesh entity lies on exactly one model or partition entity.
//!
//! The model entity is an [`EntityId`]: a model entity (`TopoVertex/Edge/Face/Volume`) or a
//! decomposition entity (`PartitionVertex/Edge/Face/Cell`, the virtual cells of the topology
//! overlay and the pieces of the model's decomposition).
//! Boundary sets, materials and deck-boundary lowering are all derived through this table, so a
//! boundary face's "boundary set" is the classification of the face, never a second source of truth.
//!
//! **Dimension order.** Writing `dim(x)` for the dimension of the entity `x` is classified on
//! (vertex 0, edge 1, face 2, volume 3, with each `Partition*` kind at the dimension of its
//! rank, see [`entity_dim`]), a valid mesh satisfies
//! `dim(sub-entity) <= dim(entity)` for every vertex of an edge, face or cell and every edge of
//! a face. `validate` reports violations.

use crate::index::{CellIx, EdgeIx, FaceIx, VertexIx};
use sm_core::{EntityId, EntityKind};

/// Dimension of a model or decomposition entity kind: `TopoVertex` / `PartitionVertex` 0,
/// `TopoEdge` / `PartitionEdge` 1, `TopoFace` / `PartitionFace` 2, `TopoVolume` /
/// `PartitionCell` 3; `None` for every other kind (mesh entities cannot be classified on them).
pub const fn entity_dim(k: EntityKind) -> Option<u8> {
    match k {
        EntityKind::TopoVertex | EntityKind::PartitionVertex => Some(0),
        EntityKind::TopoEdge | EntityKind::PartitionEdge => Some(1),
        EntityKind::TopoFace | EntityKind::PartitionFace => Some(2),
        EntityKind::TopoVolume | EntityKind::PartitionCell => Some(3),
        _ => None,
    }
}

/// Parametric position on the classified model entity (when it has a parametrisation). The
/// variant must match the kind of the classified entity (see [`Param::fits`]).
#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Param {
    /// No parametric information (always allowed).
    #[default]
    None,
    /// Parameter on a `TopoEdge`.
    Curve(f64),
    /// Parameters on a `TopoFace`.
    Surface([f64; 2]),
    /// Parameters inside a `TopoVolume`.
    Volume([f64; 3]),
}

impl Param {
    /// True if this parameter kind can describe a point on an entity of kind `k`.
    ///
    /// Virtual overlay cells take the parameters of their carrier: a `PartitionEdge` has a curve
    /// carrier (`Curve`), a `PartitionFace` a surface carrier (`Surface`). `PartitionVertex` and
    /// `PartitionCell` have no parametrization and take only `None`.
    pub const fn fits(&self, k: EntityKind) -> bool {
        matches!(
            (self, k),
            (Param::None, _)
                | (Param::Curve(_), EntityKind::TopoEdge | EntityKind::PartitionEdge)
                | (Param::Surface(_), EntityKind::TopoFace | EntityKind::PartitionFace)
                | (Param::Volume(_), EntityKind::TopoVolume)
        )
    }
}

/// The model/partition entity a mesh entity lies on, with optional parametric coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Classification {
    /// Model (`Topo*`) or decomposition (`Partition*`) entity.
    pub entity: EntityId,
    /// Parametric coordinates on `entity`.
    pub param: Param,
}

impl Classification {
    /// Classification without parametric coordinates.
    pub const fn on(entity: EntityId) -> Self {
        Self {
            entity,
            param: Param::None,
        }
    }

    /// Dimension of the classified entity ([`entity_dim`]: 0 vertex, 1 edge, 2 face, 3 volume,
    /// `Partition*` kinds at their rank), or `None` if the id is not a model/partition entity.
    pub const fn dim(&self) -> Option<u8> {
        entity_dim(self.entity.kind())
    }

    /// True if the entity is a model/partition entity and the parameter fits it.
    pub const fn is_valid(&self) -> bool {
        self.dim().is_some() && self.param.fits(self.entity.kind())
    }

    /// Bit pattern of the parameter, for hashing: `(tag, bits of up to three values)`.
    pub fn param_bits(&self) -> [u64; 4] {
        match self.param {
            Param::None => [0, 0, 0, 0],
            Param::Curve(u) => [1, u.to_bits(), 0, 0],
            Param::Surface([u, v]) => [2, u.to_bits(), v.to_bits(), 0],
            Param::Volume([u, v, w]) => [3, u.to_bits(), v.to_bits(), w.to_bits()],
        }
    }
}

/// Reverse classification index: all mesh entities on one model/partition entity, ascending.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ClassSet {
    /// Vertices classified on the entity.
    pub vertices: Vec<VertexIx>,
    /// Edges classified on the entity (empty when the mesh has no edge table).
    pub edges: Vec<EdgeIx>,
    /// Faces classified on the entity.
    pub faces: Vec<FaceIx>,
    /// Cells classified on the entity.
    pub cells: Vec<CellIx>,
}

impl ClassSet {
    /// True if nothing is classified on the entity.
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
            && self.edges.is_empty()
            && self.faces.is_empty()
            && self.cells.is_empty()
    }
}
