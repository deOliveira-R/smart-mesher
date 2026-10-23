//! Errors of the edit trait.

use crate::ids::OpId;
use sm_core::EntityId;

/// Errors from topology edits. Edits are atomic: on error the complex is unchanged.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum TopoError {
    /// An edit was attempted outside `begin_op`/`end_op`.
    #[error("no active operation")]
    NoActiveOp,
    /// `begin_op` while another operation is open.
    #[error("operation {0:?} is still open")]
    OpAlreadyActive(OpId),
    /// The id does not exist or was retired.
    #[error("{0} not found or retired")]
    NotFound(EntityId),
    /// A malformed argument (message names it).
    #[error("invalid argument: {0}")]
    Invalid(&'static str),
    /// A loop's coedges do not chain end-to-start.
    #[error("loop is not closed at coedge {at}")]
    LoopNotClosed {
        /// Position of the first coedge whose start does not match the previous end.
        at: usize,
    },
    /// Merge precondition: the entities do not coincide topologically (e.g. different vertices).
    #[error("entities {0} and {1} are not topologically coincident")]
    NotCoincident(EntityId, EntityId),
    /// A tolerance would exceed `TolerancePolicy::linear_max` or shrink.
    #[error("tolerance {0} outside the policy or not monotone")]
    Tolerance(f64),
    /// A face may have at most two uses, with opposite senses.
    #[error("face {0} would have an invalid set of uses")]
    FaceUses(EntityId),
    /// The entity is still referenced and cannot be deleted.
    #[error("{0} is still in use")]
    InUse(EntityId),
    /// The operation is not supported for this configuration (message names it).
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
    /// The 56-bit id space is exhausted.
    #[error("id space exhausted")]
    IdSpace,
}
