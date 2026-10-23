//! What gets meshed ([`Target`]) and how mesh nodes are named ([`NodeKey`]).
//!
//! A node is named by the entity that **owns** it plus a local index. Nodes on a model edge are
//! owned by the edge, so two faces sharing that edge refer to the same nodes and are conformal by
//! construction; no welding step exists. The owner also gives the classification for free.

use sm_core::{EntityId, EntityKind};
use sm_mesh_api::Classification;
use sm_model_api::{Keep, PieceFace, PieceRef};
use sm_topo_api::Dim;

/// An entity a mesh job can read or write.
///
/// The total order (`Ord`) is the determinism backbone: job ids, footprints, schedules and commit
/// order all derive from sorting targets.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Target {
    /// A topology cell: `TopoVertex/Edge/Face/Volume`, or a virtual `PartitionVertex/Edge/Face`
    /// of a decomposition overlay (an `sm_topo_api::Overlay` is itself a `TopologyView`).
    Topo(EntityId),
    /// A decomposition piece (`sm_model_api::PieceRef`), meshed as a volume.
    Piece(PieceRef),
    /// A face of a piece that has no topology id (part of a model face, or a virtual cut face).
    /// Use [`Target::piece_face`] to build it: cut faces are normalized to one key per cut.
    PieceFace(PieceFace),
}

impl Target {
    /// Normalizing constructor for piece faces: a whole model face becomes `Topo(face)`, and the
    /// two sides of a cut become one key (`Keep::Negative`), because both pieces share one mesh
    /// of the cut face.
    pub fn piece_face(pf: PieceFace) -> Self {
        match pf {
            PieceFace::Whole(f) => Target::Topo(f),
            PieceFace::Cut { cut, .. } => Target::PieceFace(PieceFace::Cut { cut, side: Keep::Negative }),
            other => Target::PieceFace(other),
        }
    }

    /// Topological dimension (0 vertex .. 3 volume), or `None` for an id that is no cell.
    pub fn dim(&self) -> Option<u8> {
        match self {
            Target::Topo(id) => match Dim::of_kind(id.kind())? {
                Dim::Vertex => Some(0),
                Dim::Edge => Some(1),
                Dim::Face => Some(2),
                Dim::Volume => Some(3),
                Dim::Body => None,
            },
            Target::Piece(_) => Some(3),
            Target::PieceFace(_) => Some(2),
        }
    }

    /// The topology id, for `Topo` targets.
    pub fn topo(&self) -> Option<EntityId> {
        match self {
            Target::Topo(id) => Some(*id),
            _ => None,
        }
    }

    /// The entity that mesh entities owned by this target are classified on, as accepted by
    /// `sm_mesh_api::Classification` (`Topo*` or `PartitionCell`).
    ///
    /// `None` for virtual overlay cells (`PartitionVertex/Edge/Face`, which the frozen mesh
    /// classification does not accept yet) and for cut faces (they lie inside a volume: the writer
    /// classifies them on the cell's entity).
    pub fn class_entity(&self) -> Option<EntityId> {
        match self {
            Target::Topo(id) => matches!(id.kind(), EntityKind::TopoVertex | EntityKind::TopoEdge | EntityKind::TopoFace | EntityKind::TopoVolume).then_some(*id),
            Target::Piece(PieceRef::Whole(v)) => Some(*v),
            Target::Piece(PieceRef::Cell(c)) => Some(*c),
            Target::PieceFace(pf) => pf.base(),
        }
    }
}

impl From<EntityId> for Target {
    fn from(id: EntityId) -> Self {
        Target::Topo(id)
    }
}

/// Stable name of a mesh node: owner plus local index.
///
/// Conventions every scheme follows, so independently written patches agree:
/// - a model vertex has exactly one node, `{ owner: vertex, local: 0 }`, seeded by the driver;
/// - the interior nodes of an edge mesh with `n` intervals are `local = 1 .. n-1` in the edge's
///   own direction (start vertex to end vertex), whatever face reads them;
/// - face and volume interior nodes are numbered freely by the job that writes the owner.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct NodeKey {
    /// The entity the node lies on (its classification).
    pub owner: Target,
    /// Index local to the owner.
    pub local: u32,
}

impl NodeKey {
    /// The node of a model vertex.
    pub const fn vertex(v: EntityId) -> Self {
        Self { owner: Target::Topo(v), local: 0 }
    }

    /// Interior node `i` (1-based) of an edge, counted in the edge's direction.
    pub const fn edge(e: EntityId, i: u32) -> Self {
        Self { owner: Target::Topo(e), local: i }
    }

    /// Interior node `i` of a face or volume target.
    pub const fn interior(owner: Target, i: u32) -> Self {
        Self { owner, local: i }
    }

    /// Classification of this node, or `None` when the owner has no classifiable entity
    /// (see [`Target::class_entity`]).
    pub fn classification(&self, param: sm_mesh_api::Param) -> Option<Classification> {
        let entity = self.owner.class_entity()?;
        let param = if param.fits(entity.kind()) { param } else { sm_mesh_api::Param::None };
        Some(Classification { entity, param })
    }
}
