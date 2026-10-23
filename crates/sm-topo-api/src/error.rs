//! Errors of topology edits and validation findings.

use crate::ids::{FaceUse, OpId};
use sm_core::{EntityId, EntityKind};
use sm_geom_api::GeomError;
use thiserror::Error;

/// Error from a topology edit.
#[derive(Clone, Debug, PartialEq, Error)]
#[non_exhaustive]
pub enum TopoError {
    /// No such entity.
    #[error("entity {0:?} not found")]
    NotFound(EntityId),
    /// The entity exists but is retired (split, merged or deleted).
    #[error("entity {0:?} is not active")]
    NotActive(EntityId),
    /// An edit was attempted outside `begin_op`/`end_op`.
    #[error("no operation is open")]
    NoOpenOp,
    /// `begin_op` while another op is open, or with a non-increasing op id.
    #[error("operation {0:?} cannot begin (another is open or the id is not increasing)")]
    BadOp(OpId),
    /// The 56-bit serial space of a kind is exhausted.
    #[error("id space exhausted for {0:?}")]
    IdSpaceExhausted(EntityKind),
    /// A loop is not a closed chain, or refers to missing or retired edges.
    #[error("invalid loop: {0}")]
    InvalidLoop(String),
    /// The shape handle does not resolve in the shape store.
    #[error("shape handle {0:?} does not resolve")]
    UnknownShape(EntityId),
    /// A face use is already owned by a shell.
    #[error("face use {0:?} is already bound")]
    SideOccupied(FaceUse),
    /// A face use is not bound where the edit expects.
    #[error("face use {0:?} is not bound")]
    NotBound(FaceUse),
    /// Two entities do not coincide within the operation tolerance, so they cannot merge.
    #[error("entities do not coincide: {0}")]
    NotCoincident(String),
    /// The entity is still referenced and cannot be removed.
    #[error("entity {0:?} is still in use")]
    InUse(EntityId),
    /// The request is outside what this implementation supports.
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
    /// Geometry query failed.
    #[error(transparent)]
    Geometry(#[from] GeomError),
}

/// Category of a validation finding.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum ViolationCode {
    /// Reference to a missing or retired entity.
    DanglingReference,
    /// Loop is not a closed chain.
    LoopNotClosed,
    /// Face use owner/shell/sense bookkeeping is inconsistent.
    FaceUseMismatch,
    /// A shell is not closed (an edge is not traversed equally often in both directions).
    ShellNotClosed,
    /// Radial ring is missing entries, has extras, or neighbours disagree on the volume between them.
    RadialRing,
    /// Edge geometry disagrees with its vertices beyond the model tolerance.
    EdgeGeometry,
    /// Body/volume membership is inconsistent.
    Membership,
    /// Coincidence record is inconsistent with the entities.
    Coincidence,
}

/// One validation finding.
#[derive(Clone, Debug, PartialEq)]
pub struct Violation {
    /// Category.
    pub code: ViolationCode,
    /// The entity concerned, when there is one.
    pub entity: Option<EntityId>,
    /// Human-readable detail.
    pub message: String,
}
