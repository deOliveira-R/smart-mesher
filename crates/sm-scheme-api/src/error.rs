//! Typed errors. [`GenerateFailure`] is the structured failure that lets the driver fall back.

use crate::intent::SchemeName;
use crate::problem::ConstraintId;
use crate::target::Target;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;

/// Why generation failed, and what the driver may do about it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FailureKind {
    /// The scheme could not mesh this input (bad quality, inverted element, unsupported
    /// topology). The driver rejects `(target, scheme)`, re-plans and tries the next scheme of
    /// the fallback chain (or the next auto candidate).
    Fallback,
    /// The plan itself is inconsistent with what the scheme needs (a missing boundary patch, a
    /// side count mismatch). This is a bug in a scheme or the solver: no fallback.
    Fatal,
}

/// A structured generation failure.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, thiserror::Error)]
#[error("{scheme} failed on {target:?} ({kind:?}): {reason}")]
pub struct GenerateFailure {
    /// The target being generated.
    pub target: Target,
    /// The scheme that failed.
    pub scheme: SchemeName,
    /// Whether the driver may fall back.
    pub kind: FailureKind,
    /// Human-readable reason.
    pub reason: String,
}

impl GenerateFailure {
    /// A failure the driver may recover from by falling back.
    pub fn fallback(target: Target, scheme: &str, reason: impl Into<String>) -> Self {
        Self { target, scheme: SchemeName::new(scheme), kind: FailureKind::Fallback, reason: reason.into() }
    }
    /// A failure that aborts meshing.
    pub fn fatal(target: Target, scheme: &str, reason: impl Into<String>) -> Self {
        Self { target, scheme: SchemeName::new(scheme), kind: FailureKind::Fatal, reason: reason.into() }
    }
}

/// The constraint system has no solution. Lists the constraints and entities involved so the
/// caller (or a UI) can show the conflict, and so the driver can fall back.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, thiserror::Error)]
#[error("infeasible: {reason} (constraints {constraints:?}, entities {entities:?})")]
pub struct Infeasible {
    /// What could not be satisfied.
    pub reason: String,
    /// The constraints involved (ascending).
    pub constraints: Vec<ConstraintId>,
    /// The entities involved (ascending).
    pub entities: Vec<EntityId>,
}

/// Errors of planning, generation and commit.
#[derive(Debug, thiserror::Error)]
pub enum SchemeError {
    /// No scheme of this name is registered for the target's dimension.
    #[error("unknown scheme `{name}` for {target:?}")]
    UnknownScheme {
        /// The name.
        name: SchemeName,
        /// The target.
        target: Target,
    },
    /// Scheme parameters could not be read.
    #[error("bad parameters for `{scheme}`: {reason}")]
    BadParams {
        /// The scheme.
        scheme: SchemeName,
        /// What was wrong.
        reason: String,
    },
    /// A hard scheme request is not applicable to its target.
    #[error("`{scheme}` is not applicable to {target:?}: {reason}")]
    NotApplicable {
        /// The target.
        target: Target,
        /// The scheme.
        scheme: SchemeName,
        /// Why not.
        reason: String,
    },
    /// Nothing in the chain (intent, fallbacks, impositions, auto candidates) applies.
    #[error("no applicable scheme for {target:?}; tried {tried:?}")]
    NoApplicableScheme {
        /// The target.
        target: Target,
        /// `(scheme, reason)` for every candidate, in the order tried.
        tried: Vec<(SchemeName, String)>,
    },
    /// A parent scheme imposes a scheme set on a child whose hard intent is outside it, or two
    /// parents impose disjoint sets.
    #[error("{child:?}: {imposed_by:?} requires one of {required:?}, but {conflict}")]
    ImpositionConflict {
        /// The constrained child.
        child: Target,
        /// The imposing parent.
        imposed_by: Target,
        /// The allowed schemes.
        required: Vec<SchemeName>,
        /// The conflicting request (a hard intent or another imposition).
        conflict: String,
    },
    /// Two copy constraints (deck, sweep target, user copy) claim the same target.
    #[error("{target:?} is the target of two copy constraints ({first:?}, {second:?})")]
    CopyConflict {
        /// The target.
        target: Target,
        /// The first constraint.
        first: ConstraintId,
        /// The second one.
        second: ConstraintId,
    },
    /// A copy correspondence could not be built (entities do not map under the transform).
    #[error("copy {source_target:?} -> {target:?}: {reason}")]
    Correspondence {
        /// The source.
        source_target: Target,
        /// The target.
        target: Target,
        /// What did not map.
        reason: String,
    },
    /// A boundary layer was requested on a deck face (ir.md §5.1).
    #[error("boundary layer requested on deck face {face} (boundary {boundary})")]
    BoundaryLayerOnDeck {
        /// The face.
        face: EntityId,
        /// Its deck boundary record.
        boundary: EntityId,
    },
    /// The interval or corner system has no solution.
    #[error(transparent)]
    Infeasible(#[from] Infeasible),
    /// A scheme failed to generate its patch.
    #[error(transparent)]
    Generate(#[from] GenerateFailure),
    /// A patch broke the patch contract (wrong node owners, missing nodes, duplicate keys).
    #[error("invalid patch for {target:?}: {reason}")]
    InvalidPatch {
        /// The target.
        target: Target,
        /// What was wrong.
        reason: String,
    },
    /// A node owner cannot be classified onto a model or piece entity.
    #[error("cannot classify nodes owned by {owner}")]
    Unclassifiable {
        /// The owner.
        owner: EntityId,
    },
    /// The input topology or model is unusable (missing carrier, unknown piece, ...).
    #[error("input: {0}")]
    Input(String),
    /// Re-planning after fallbacks did not converge.
    #[error("gave up after {0} re-plans")]
    ReplanLimit(usize),
    /// A topology query failed.
    #[error(transparent)]
    Topology(#[from] sm_topo_api::QueryError),
    /// A geometry evaluation failed.
    #[error(transparent)]
    Geometry(#[from] sm_geom_api::GeomError),
    /// The mesh builder rejected the commit.
    #[error(transparent)]
    Mesh(#[from] sm_mesh_api::MeshError),
}

impl SchemeError {
    /// The fallback-eligible generation failure inside this error, if any.
    pub fn as_fallback(&self) -> Option<&GenerateFailure> {
        match self {
            SchemeError::Generate(g) if g.kind == FailureKind::Fallback => Some(g),
            _ => None,
        }
    }
}

/// Result alias.
pub type Result<T, E = SchemeError> = core::result::Result<T, E>;
