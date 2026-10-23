//! Errors of topology edits.

use crate::types::Dim;
use sm_core::EntityId;

/// Failure of a topology edit. Edits are atomic: on error the complex is unchanged.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum TopoError {
    /// The id does not exist in this complex.
    #[error("no such cell {0:?}")]
    NoSuchCell(EntityId),
    /// The cell exists but has been deleted, split or merged away.
    #[error("cell {0:?} is not alive")]
    Dead(EntityId),
    /// The id has the wrong dimension for this operation.
    #[error("cell {id:?} is not a {expected:?}")]
    WrongDim {
        /// Offending id.
        id: EntityId,
        /// Required dimension.
        expected: Dim,
    },
    /// A loop's coedges do not chain head-to-tail.
    #[error("loop {ring} is not closed at coedge {at}")]
    OpenLoop {
        /// Loop index.
        ring: usize,
        /// Coedge position where the chain breaks.
        at: usize,
    },
    /// A face needs at least one non-empty loop.
    #[error("empty loop")]
    EmptyLoop,
    /// The face side requested is already occupied by another volume.
    #[error("side of face {face:?} already occupied")]
    SideOccupied {
        /// The face.
        face: EntityId,
    },
    /// The cell is still used by a parent and cannot be deleted.
    #[error("cell {0:?} is still referenced")]
    StillReferenced(EntityId),
    /// A chord edge does not connect two vertices of the face's loop.
    #[error("chord does not lie on the face loop")]
    ChordNotOnLoop,
    /// Operation not available for this configuration (documented per operator).
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
    /// A parameter is outside the allowed range.
    #[error("invalid parameter: {0}")]
    InvalidParameter(&'static str),
    /// The cells to merge are not geometrically or topologically coincident.
    #[error("cells are not coincident: {0}")]
    NotCoincident(&'static str),
    /// The incidence would violate the relation's dimension rules.
    #[error("invalid incidence {parent:?} -> {child:?}")]
    InvalidIncidence {
        /// Parent.
        parent: EntityId,
        /// Child.
        child: EntityId,
    },
    /// The id space of a kind is exhausted.
    #[error("id space exhausted")]
    IdExhausted,
}
