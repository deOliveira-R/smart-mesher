//! The typed errors of the library functions.

use sm_core::{EntityId, Point3};
use sm_geom_api::GeomError;
use sm_op_api::OpError;
use sm_topo_api::{QueryError, TopoError};

/// Why a boolean, imprint, merge or webcut failed.
///
/// Every variant names the entities involved and, where one exists, the way out (raise `tol`,
/// perturb the tool, intersect fewer bodies). A library function that returns `Err` may have
/// left the (open) operation half-edited: the caller aborts the operation, which is what
/// `sm_layer_api::topo_txn` does for the ops.
#[derive(Debug, Clone, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum BooleanError {
    /// The result has no volume (for example the intersection of disjoint bodies).
    #[error("the result is empty: the operands share no volume; check that the bodies overlap, or use a different rule")]
    EmptyResult,
    /// The inputs are valid but outside what this kernel handles.
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
    /// An input volume is not a manifold solid (baffle `Both` uses, fins, free faces).
    #[error("{0} is not a manifold solid (baffle, fin or free face): boolean operations need closed shells; delete or detach the sheet first")]
    NonManifoldInput(EntityId),
    /// The operands touch along a line or at points only (tangent contact), which the arrangement
    /// cannot split robustly.
    #[error("tangent contact at {entity}: {detail}; perturb the tool by more than the model tolerance (move or resize it so the contact becomes a proper overlap or a clean gap)")]
    TangentContact {
        /// The face (or, for point contact between whole operands, the volume) where it was found.
        entity: EntityId,
        /// What was found.
        detail: &'static str,
    },
    /// Two surfaces could not be intersected.
    #[error("intersecting the surfaces of faces {a} and {b} failed: {source}; perturb the tool or intersect fewer bodies")]
    Ssi {
        /// First face.
        a: EntityId,
        /// Second face.
        b: EntityId,
        /// The geometry error.
        source: GeomError,
    },
    /// Closing a geometric gap needs a tolerance above the model maximum.
    #[error("{entity} needs a tolerance of {needed:e}, above the model maximum {max:e}; perturb the tool or use a coarser model tolerance")]
    Tolerance {
        /// The cell that cannot be closed.
        entity: EntityId,
        /// The tolerance that would be needed.
        needed: f64,
        /// `ModelTolerance::linear_max`.
        max: f64,
    },
    /// A classification ray hit an edge or vertex in all eight directions.
    #[error("cannot classify face {face} at ({:.6e}, {:.6e}, {:.6e}): every probe ray is ambiguous; perturb the tool slightly", .point.x, .point.y, .point.z)]
    Ambiguous {
        /// The face being classified.
        face: EntityId,
        /// The probe point on it.
        point: Point3,
    },
    /// The assembled volume failed its final check.
    #[error("the {rule} result volume {volume} is invalid (open shell, non-positive volume or failed validation); the operation was rolled back: perturb the tool or intersect fewer bodies; operands that are nearly tangent (clearance below about 1e-2 model units on a curved surface) are a known cause, so open the clearance or close it to a clean overlap")]
    ResultInvalid {
        /// The offending volume.
        volume: EntityId,
        /// The operation that produced it.
        rule: &'static str,
    },
    /// The inputs are not acceptable (reported as `Invalid` by the ops).
    #[error("{0}")]
    Invalid(String),
    /// A topology edit was refused.
    #[error("topology edit refused: {0}")]
    Topo(#[from] TopoError),
    /// A topology query failed.
    #[error("topology query failed: {0}")]
    Query(#[from] QueryError),
    /// A geometry evaluation failed.
    #[error("geometry: {0}")]
    Geom(#[from] GeomError),
}

impl From<BooleanError> for OpError {
    fn from(e: BooleanError) -> Self {
        match e {
            BooleanError::Invalid(m) => OpError::Invalid(m),
            BooleanError::Unsupported(m) => OpError::Unsupported(m.to_owned()),
            e @ BooleanError::TangentContact { .. } => OpError::Unsupported(e.to_string()),
            other => OpError::Failed(other.to_string()),
        }
    }
}
