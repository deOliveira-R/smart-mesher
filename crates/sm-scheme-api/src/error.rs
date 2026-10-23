//! Typed errors. Scheme failures are structured ([`SchemeError::kind`]) so the driver can decide
//! to fall back; driver errors carry the full list of attempts for diagnostics.

use crate::intent::SchemeId;
use crate::interval::IntervalError;
use crate::key::ShapeKey;
use serde::{Deserialize, Serialize};
use sm_mesh_api::MeshError;

/// Why a scheme failed. Everything except `Internal` lets the driver try the next candidate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum FailureKind {
    /// The scheme cannot mesh this kind of entity (wrong topology, missing roles).
    NotApplicable,
    /// The scheme declined during planning (e.g. no 4 corners for map).
    Rejected,
    /// The solved intervals or the boundary mesh violate what the scheme needs.
    Constraint,
    /// A geometry evaluation failed.
    Geometry,
    /// The result is below the quality threshold or inverted.
    Quality,
    /// A feature the scheme does not implement yet.
    Unsupported,
    /// A bug or broken invariant; never retried.
    Internal,
}

/// Error returned by a scheme phase or a shape query.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum SchemeError {
    /// The scheme does not apply to the entity.
    #[error("not applicable: {0}")]
    NotApplicable(String),
    /// A structured failure.
    #[error("{kind:?}: {message}")]
    Failed {
        /// The failure class.
        kind: FailureKind,
        /// Human-readable detail.
        message: String,
    },
    /// A shape (topology / geometry) query failed.
    #[error("shape query on {key}: {message}")]
    Shape {
        /// The entity queried.
        key: ShapeKey,
        /// Detail.
        message: String,
    },
    /// Parameters or intent are malformed.
    #[error("bad parameters: {0}")]
    BadParams(String),
    /// Not implemented for this input.
    #[error("unsupported: {0}")]
    Unsupported(String),
}

impl SchemeError {
    /// A structured failure.
    pub fn failed(kind: FailureKind, message: impl Into<String>) -> Self {
        SchemeError::Failed { kind, message: message.into() }
    }

    /// A shape query failure.
    pub fn shape(key: ShapeKey, message: impl Into<String>) -> Self {
        SchemeError::Shape { key, message: message.into() }
    }

    /// The failure class.
    pub fn kind(&self) -> FailureKind {
        match self {
            SchemeError::NotApplicable(_) => FailureKind::NotApplicable,
            SchemeError::Failed { kind, .. } => *kind,
            SchemeError::Shape { .. } => FailureKind::Geometry,
            SchemeError::BadParams(_) => FailureKind::Rejected,
            SchemeError::Unsupported(_) => FailureKind::Unsupported,
        }
    }

    /// True if the driver may try the next scheme in the chain.
    pub fn allows_fallback(&self) -> bool {
        self.kind() != FailureKind::Internal
    }
}

/// Which phase an attempt failed in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Phase {
    /// The scheme's capabilities violate a requirement imposed on the entity.
    Requirements,
    /// `applicability` returned score 0.
    Applicability,
    /// `plan` failed.
    Plan,
    /// `apply` failed (the driver re-planned without this scheme).
    Apply,
}

/// One failed attempt of one scheme on one entity (kept for diagnostics and the mesh report).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Attempt {
    /// The entity.
    pub target: ShapeKey,
    /// The scheme tried.
    pub scheme: SchemeId,
    /// Where it failed.
    pub phase: Phase,
    /// Failure class.
    pub kind: FailureKind,
    /// Detail.
    pub message: String,
}

/// Errors of the driver.
#[derive(Debug, thiserror::Error)]
pub enum DriveError {
    /// Every candidate scheme failed (or none was registered) for an entity.
    #[error("no scheme could mesh {target} ({} attempt(s))", attempts.len())]
    NoScheme {
        /// The entity.
        target: ShapeKey,
        /// What was tried.
        attempts: Vec<Attempt>,
    },
    /// Two hard settings or requirements contradict each other.
    #[error("conflict on {target}: {message}")]
    Conflict {
        /// The entity.
        target: ShapeKey,
        /// Detail naming both sides.
        message: String,
    },
    /// The interval problem has no solution.
    #[error(transparent)]
    Intervals(#[from] IntervalError),
    /// A scheme failed in `apply` and no fallback was possible.
    #[error("{scheme} failed on {target}: {error}")]
    Failed {
        /// The entity.
        target: ShapeKey,
        /// The scheme.
        scheme: SchemeId,
        /// The failure.
        error: SchemeError,
        /// Earlier attempts.
        attempts: Vec<Attempt>,
    },
    /// Same-dimension ordering constraints (`Plan::after`, copy sources) form a cycle.
    #[error("ordering cycle through {0:?}")]
    Cycle(Vec<ShapeKey>),
    /// A shape query failed outside any scheme.
    #[error(transparent)]
    Shape(#[from] SchemeError),
    /// A scheme returned a patch that breaks the patch contract.
    #[error("bad patch from {scheme} on {target}: {message}")]
    BadPatch {
        /// The entity.
        target: ShapeKey,
        /// The scheme.
        scheme: SchemeId,
        /// Detail.
        message: String,
    },
    /// Writing the mesh failed.
    #[error(transparent)]
    Mesh(#[from] MeshError),
    /// Fallback re-planning hit `DriverOptions::max_replans`.
    #[error("too many fallback re-plans ({} attempt(s))", attempts.len())]
    TooManyReplans {
        /// Every attempt so far.
        attempts: Vec<Attempt>,
    },
    /// Registry misuse (duplicate or unknown scheme).
    #[error("registry: {0}")]
    Registry(String),
}
