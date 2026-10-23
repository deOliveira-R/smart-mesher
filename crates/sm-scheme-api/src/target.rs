//! What a scheme meshes ([`Target`]) and how generated nodes are named ([`NodeKey`]).

use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_model_api::PieceRef;

/// Dimension of a scheme (curve, surface, volume).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum SchemeDim {
    /// Curve schemes (equal, bias, dualbias, curvature, stretch, pinpoint, copy).
    Curve,
    /// Surface schemes (map, submap, pave, tri*, circle, hole, polyhedron, copy, mirror).
    Surface,
    /// Volume schemes (map, submap, sweep, tetmesh, copy, polyhedron).
    Volume,
}

impl SchemeDim {
    /// Topological dimension (1, 2, 3).
    pub const fn rank(self) -> u8 {
        match self {
            SchemeDim::Curve => 1,
            SchemeDim::Surface => 2,
            SchemeDim::Volume => 3,
        }
    }
}

/// Identifier of an atomic meshing group (multisweep, merged volumes meshed together, THex).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct GroupId(pub u32);

/// One unit of meshing work.
///
/// Curves and surfaces are cells of the [`sm_topo_api::TopologyView`] being meshed (real
/// `Topo*` cells or virtual `Partition*` cells of a decomposition overlay). Volumes are
/// **pieces** of the decomposition layer: a `TopoVolume` id stands for
/// [`PieceRef::Whole`], a `PartitionCell` id for [`PieceRef::Cell`]. A group is meshed by one
/// scheme invocation and committed all-or-nothing.
///
/// Ordering (derived) is curves, surfaces, volumes, groups, then by id: this is also the
/// generation order of independent work and the commit order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Target {
    /// An edge of the topology view.
    Curve(EntityId),
    /// A face of the topology view.
    Surface(EntityId),
    /// A piece (`TopoVolume` = whole volume, `PartitionCell` = decomposition cell).
    Volume(EntityId),
    /// An atomic group of volumes (see [`crate::MeshGroup`]).
    Group(GroupId),
}

impl Target {
    /// The scheme dimension of the target (a group is a volume target).
    pub const fn dim(&self) -> SchemeDim {
        match self {
            Target::Curve(_) => SchemeDim::Curve,
            Target::Surface(_) => SchemeDim::Surface,
            Target::Volume(_) | Target::Group(_) => SchemeDim::Volume,
        }
    }

    /// The entity id, if the target is a single entity.
    pub const fn entity(&self) -> Option<EntityId> {
        match self {
            Target::Curve(e) | Target::Surface(e) | Target::Volume(e) => Some(*e),
            Target::Group(_) => None,
        }
    }

    /// The decomposition piece of a volume target.
    pub fn piece(&self) -> Option<PieceRef> {
        match self {
            Target::Volume(e) => piece_of(*e),
            _ => None,
        }
    }

    /// The volume target of a piece.
    pub const fn of_piece(p: PieceRef) -> Target {
        match p {
            PieceRef::Whole(v) | PieceRef::Cell(v) => Target::Volume(v),
        }
    }
}

/// The piece named by a volume-target id (`TopoVolume` = whole, `PartitionCell` = cell).
pub fn piece_of(id: EntityId) -> Option<PieceRef> {
    match id.kind() {
        EntityKind::TopoVolume => Some(PieceRef::Whole(id)),
        EntityKind::PartitionCell => Some(PieceRef::Cell(id)),
        _ => None,
    }
}

/// Canonical name of a generated mesh node, independent of generation order.
///
/// `owner` is the lowest-dimensional entity the node lies on; `index` numbers the nodes the
/// owner's generator created:
///
/// | owner | index |
/// |---|---|
/// | topology vertex | always 0 (one node per vertex, created by the driver) |
/// | edge with `n` intervals | `1..n` in the **edge's own direction** (start vertex = 0) |
/// | face | generator-defined, `0..` (a copy keeps the source's index) |
/// | volume / piece | generator-defined, `0..` |
///
/// Because keys are canonical, the edge nodes a face generator needs are known without asking
/// the curve scheme ([`crate::GenCtx::edge_nodes`]), and the committer can number mesh
/// vertices by sorting keys, so the result does not depend on which thread generated what.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct NodeKey {
    /// The entity the node lies on.
    pub owner: EntityId,
    /// Index within the owner (see the table above).
    pub index: u32,
}

impl NodeKey {
    /// The node of a topology vertex.
    pub const fn vertex(v: EntityId) -> Self {
        Self { owner: v, index: 0 }
    }
    /// Node `index` owned by `owner`.
    pub const fn new(owner: EntityId, index: u32) -> Self {
        Self { owner, index }
    }
}
