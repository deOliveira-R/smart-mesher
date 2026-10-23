//! Errors of topology edits and geometry queries.

use crate::types::Dim;
use crate::validate::TopoIssue;
use sm_core::EntityId;
use sm_geom_api::GeomError;

/// Failure of a topology edit. Edits are atomic: on error the complex is unchanged (the failing
/// operator is rolled back to its starting state, and `end_op` aborts the whole operation).
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum TopoError {
    /// An edit was attempted outside `begin_op` .. `end_op`.
    #[error("no operation is open (call begin_op first)")]
    NoOpenOp,
    /// `begin_op` was called while an operation is open.
    #[error("an operation is already open")]
    OpAlreadyOpen,
    /// `begin_op` was given an id that does not exceed the previous operation's.
    #[error("operation ids must increase (previous {previous}, got {got})")]
    OpNotIncreasing {
        /// Previous operation id.
        previous: u64,
        /// Offending id.
        got: u64,
    },
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
    /// A chord edge does not connect vertices of the face's outer loop.
    #[error("cut does not lie on the face loop")]
    ChordNotOnLoop,
    /// A baffle (`Side::Both`) or embedded sheet was put in an `Outer` or `Void` shell.
    #[error("baffles belong in an Internal shell")]
    BaffleInClosedShell,
    /// The cut faces do not separate the volume into exactly two parts.
    #[error("cut does not separate the volume into two parts ({0} found)")]
    CutDoesNotSeparate(usize),
    /// Operation not available for this configuration (documented per operator).
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
    /// A parameter is outside the allowed range.
    #[error("invalid parameter: {0}")]
    InvalidParameter(&'static str),
    /// A merge was asked to keep the higher id; the lowest id always survives.
    #[error("the lowest id survives a merge: keep {keep:?} must be below drop {drop:?}")]
    KeepNotLowest {
        /// The requested survivor.
        keep: EntityId,
        /// The requested absorbed cell.
        drop: EntityId,
    },
    /// The cells to merge are not geometrically or topologically coincident.
    #[error("cells are not coincident: {0}")]
    NotCoincident(&'static str),
    /// The merge or edit would raise a tolerance above `linear_max`.
    #[error("tolerance {wanted:e} exceeds linear_max {max:e}")]
    ToleranceTooLarge {
        /// The tolerance that was asked for.
        wanted: f64,
        /// The cap.
        max: f64,
    },
    /// The incidence would violate the relation's dimension rules.
    #[error("invalid incidence {parent:?} -> {child:?}")]
    InvalidIncidence {
        /// Parent.
        parent: EntityId,
        /// Child.
        child: EntityId,
    },
    /// A shape operation failed.
    #[error("geometry: {0}")]
    Geom(#[from] GeomError),
    /// `end_op` found the operation's result invalid and rolled it back.
    #[error("operation left the complex invalid ({} issue(s)), rolled back", .0.len())]
    OpInvalid(Vec<TopoIssue>),
    /// The overlay was built over a host revision that is no longer current.
    #[error("overlay is stale: built on host revision {built}, host is at {host}")]
    StaleOverlay {
        /// Revision the overlay was built on.
        built: u64,
        /// Current host revision.
        host: u64,
    },
    /// The id space of a kind is exhausted.
    #[error("id space exhausted")]
    IdExhausted,
}

/// Failure of a geometric query on a view.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum QueryError {
    /// This implementor or this surface kind does not provide the query.
    #[error("unsupported query: {0}")]
    Unsupported(&'static str),
    /// The id is unknown or not alive.
    #[error("no such cell {0:?}")]
    NoSuchCell(EntityId),
    /// The cell has the wrong rank for this query.
    #[error("cell {0:?} has the wrong rank")]
    WrongDim(EntityId),
    /// The topology is incomplete for this query (missing carrier, open loop).
    #[error("incomplete topology: {0}")]
    Incomplete(&'static str),
    /// A shape operation failed.
    #[error("geometry: {0}")]
    Geom(#[from] GeomError),
}
