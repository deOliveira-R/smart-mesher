//! [`ShapeKey`]: the one name for "something the driver meshes".
//!
//! Schemes mesh topology cells (`TopoEdge/Face/Volume`, or the virtual `Partition*` cells of a
//! `sm_topo_api::Overlay`) **and** decomposition pieces (`sm_model_api::PieceRef`, with their
//! `PieceFace`s). One ordered key type covers all of them, so plans, requirements, interval
//! variables and intent all use the same name and sort the same way everywhere.

use core::fmt;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_model_api::{PieceFace, PieceRef};
use sm_topo_api::Dim;

/// An entity the driver can mesh. `Ord` is the deterministic processing order.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum ShapeKey {
    /// A cell known to the `TopologyView` (real `Topo*` or virtual `Partition*` kinds).
    Cell(EntityId),
    /// A decomposition piece (rank 3).
    Piece(PieceRef),
    /// A face of a decomposition piece (rank 2).
    PieceFace(PieceFace),
}

impl ShapeKey {
    /// Key of a topology cell.
    pub const fn cell(id: EntityId) -> Self {
        ShapeKey::Cell(id)
    }

    /// Rank of the keyed entity, `None` for an id that is not a cell kind.
    pub fn dim(&self) -> Option<Dim> {
        match self {
            ShapeKey::Cell(id) => Dim::of_kind(id.kind()),
            ShapeKey::Piece(_) => Some(Dim::Volume),
            ShapeKey::PieceFace(_) => Some(Dim::Face),
        }
    }

    /// The canonical spelling: an undecomposed piece `Piece(Whole(v))` is the volume `Cell(v)`
    /// and a whole piece face `PieceFace(Whole(f))` is the face `Cell(f)`. The driver
    /// canonicalizes every key it receives, so one entity never gets two meshes.
    pub fn canonical(self) -> Self {
        match self {
            ShapeKey::Piece(PieceRef::Whole(v)) => ShapeKey::Cell(v),
            ShapeKey::PieceFace(PieceFace::Whole(f)) => ShapeKey::Cell(f),
            k => k,
        }
    }

    /// The topology cell id, if this key names one directly.
    pub fn as_cell(&self) -> Option<EntityId> {
        match self {
            ShapeKey::Cell(id) => Some(*id),
            _ => None,
        }
    }

    /// True for real topology kinds (`TopoVertex..TopoVolume`).
    pub fn is_real_cell(&self) -> bool {
        matches!(self, ShapeKey::Cell(id) if matches!(id.kind(), EntityKind::TopoVertex | EntityKind::TopoEdge | EntityKind::TopoFace | EntityKind::TopoVolume))
    }
}

impl From<EntityId> for ShapeKey {
    fn from(id: EntityId) -> Self {
        ShapeKey::Cell(id)
    }
}

impl From<PieceRef> for ShapeKey {
    fn from(p: PieceRef) -> Self {
        ShapeKey::Piece(p).canonical()
    }
}

impl fmt::Display for ShapeKey {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ShapeKey::Cell(id) => write!(f, "{id}"),
            ShapeKey::Piece(PieceRef::Whole(v)) => write!(f, "piece(whole {v})"),
            ShapeKey::Piece(PieceRef::Cell(c)) => write!(f, "piece({c})"),
            ShapeKey::PieceFace(pf) => write!(f, "piece-face({pf:?})"),
        }
    }
}
