//! Typed errors, the attempt history and the structured failures that let the driver re-plan.

use crate::intent::SchemeName;
use crate::problem::{ConstraintId, Need};
use crate::target::{Job, Target};
use serde::{Deserialize, Serialize};
use sm_core::EntityId;

/// Why a scheme attempt failed or was passed over, and what the driver may do about it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[non_exhaustive]
pub enum FailureKind {
    /// The scheme does not apply to this input (reported by `applicability`).
    NotApplicable,
    /// The scheme was passed over: overridden by an imposition, outscored, or rejected by an
    /// earlier failure.
    Rejected,
    /// The scheme's constraints cannot be met with the others (interval parity, copy
    /// mismatch, corner sums).
    Constraint,
    /// The geometry defeated the scheme (self-intersecting loop, failed projection).
    Geometry,
    /// The mesh the scheme produced is not good enough (inverted element, bad quality).
    Quality,
    /// The input needs a feature the scheme does not implement.
    Unsupported,
    /// The plan or the scheme is inconsistent (a bug). No fallback.
    Internal,
}

impl FailureKind {
    /// True if the driver may reject the scheme and re-plan. Only `Internal` is fatal.
    pub const fn allows_fallback(self) -> bool {
        !matches!(self, FailureKind::Internal)
    }
}

/// The driver phase an [`Attempt`] belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Phase {
    /// Scheme selection (applicability, impositions, auto scoring).
    Resolve,
    /// Requirements (impositions, roles, claims, copies).
    Requirements,
    /// Corner picking.
    Corners,
    /// Interval assembly and solve.
    Intervals,
    /// Footprints and waves.
    Schedule,
    /// Scheme generation.
    Generate,
    /// Standalone and group transforms.
    Transform,
    /// Post-generation copy verification.
    Verify,
    /// Commit into the mesh builder.
    Commit,
}

/// One scheme candidate that was evaluated and did not win, or that failed.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct Attempt {
    /// The target.
    pub target: Target,
    /// The candidate scheme.
    pub scheme: SchemeName,
    /// Where it was rejected or failed.
    pub phase: Phase,
    /// Why.
    pub kind: FailureKind,
    /// Human-readable detail.
    pub message: String,
}

impl Attempt {
    /// An attempt of `scheme` on `target` that was rejected or failed in `phase`.
    pub fn new(target: Target, scheme: SchemeName, phase: Phase, kind: FailureKind, message: impl Into<String>) -> Self {
        Self { target, scheme, phase, kind, message: message.into() }
    }
}

/// A structured generation failure.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, thiserror::Error)]
#[error("{scheme} failed on {target:?} ({kind:?}): {reason}")]
#[non_exhaustive]
pub struct GenerateFailure {
    /// The target being generated.
    pub target: Target,
    /// The scheme that failed.
    pub scheme: SchemeName,
    /// What kind of failure. Everything but `Internal` lets the driver fall back.
    pub kind: FailureKind,
    /// Human-readable reason.
    pub reason: String,
}

impl GenerateFailure {
    /// A failure of `kind`.
    pub fn new(target: Target, scheme: &str, kind: FailureKind, reason: impl Into<String>) -> Self {
        Self {
            target,
            scheme: SchemeName::new(scheme),
            kind,
            reason: reason.into(),
        }
    }
    /// A quality failure: the driver may recover by falling back.
    pub fn fallback(target: Target, scheme: &str, reason: impl Into<String>) -> Self {
        Self::new(target, scheme, FailureKind::Quality, reason)
    }
    /// A failure that aborts meshing (`Internal`).
    pub fn fatal(target: Target, scheme: &str, reason: impl Into<String>) -> Self {
        Self::new(target, scheme, FailureKind::Internal, reason)
    }
}

/// The constraint system has no solution. Lists the constraints and entities involved so the
/// caller (or a UI) can show the conflict, and so the driver can re-plan.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, thiserror::Error)]
#[error("infeasible: {reason} (constraints {constraints:?}, entities {entities:?})")]
#[non_exhaustive]
pub struct Infeasible {
    /// What could not be satisfied.
    pub reason: String,
    /// The constraints involved (ascending).
    pub constraints: Vec<ConstraintId>,
    /// The entities involved (ascending).
    pub entities: Vec<EntityId>,
    /// The `(target, scheme)` the driver would reject to re-plan: among the involved
    /// scheme-origin constraints whose selection is not a hard intent, the lowest-dimension
    /// one, ties to the later contribution. Solvers leave it `None`; the driver fills it.
    #[serde(default)]
    pub blame: Option<(Target, SchemeName)>,
}

impl Infeasible {
    /// An infeasibility over `constraints` and `entities` (sorted and deduplicated here); the
    /// driver fills [`blame`](Self::blame).
    pub fn new(reason: impl Into<String>, mut constraints: Vec<ConstraintId>, mut entities: Vec<EntityId>) -> Self {
        constraints.sort_unstable();
        constraints.dedup();
        entities.sort_unstable();
        entities.dedup();
        Self { reason: reason.into(), constraints, entities, blame: None }
    }
}

/// Errors of planning, generation and commit.
#[derive(Debug, thiserror::Error)]
#[non_exhaustive]
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
        /// Every candidate evaluated, in the order tried.
        tried: Vec<Attempt>,
        /// The imposing parents that can be re-planned (soft or auto choices, never `copy`).
        /// [`SchemeError::blame`] returns the first.
        imposed_by: Vec<(Target, SchemeName)>,
    },
    /// A parent imposes something on a child that its hard intent (or another imposition)
    /// contradicts.
    #[error("{child:?}: {imposed_by:?} requires {required:?}, but {conflict}")]
    ImpositionConflict {
        /// The constrained child.
        child: Target,
        /// The imposing parent.
        imposed_by: Target,
        /// What was required.
        required: Need,
        /// The conflicting request (a hard intent or another imposition).
        conflict: String,
    },
    /// A scheme contributed a requirement about something that is not its strict sub-entity,
    /// or about something already resolved.
    #[error("{by:?}: bad requirement on {child:?}: {reason}")]
    BadRequirement {
        /// The requesting target.
        by: Target,
        /// The entity the requirement is about.
        child: Target,
        /// What is wrong.
        reason: String,
    },
    /// Two copy constraints of one equivalence class disagree (different transforms or vertex
    /// correspondences).
    #[error("copy conflict on {target:?} ({first:?}, {second:?}): {reason}")]
    CopyConflict {
        /// The target where the disagreement shows.
        target: Target,
        /// The first constraint.
        first: ConstraintId,
        /// The second one.
        second: ConstraintId,
        /// What disagrees.
        reason: String,
    },
    /// A claimed or externally written copy target does not equal the image of its source.
    #[error("copy constraint {constraint:?} violated: {reason}")]
    CopyMismatch {
        /// The (last) constraint on the path from the class root to the member.
        constraint: ConstraintId,
        /// What differs.
        reason: String,
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
    /// A boundary layer was requested on a deck face (ir.md section 5.1).
    #[error("boundary layer requested on deck face {face} (boundary {boundary})")]
    BoundaryLayerOnDeck {
        /// The face.
        face: EntityId,
        /// Its deck boundary record.
        boundary: EntityId,
    },
    /// The jobs cannot be scheduled: a dependency cycle, two writers of one target, or a read
    /// with no writer.
    #[error("schedule: {reason} (jobs {jobs:?}, constraints {constraints:?})")]
    Schedule {
        /// The jobs involved.
        jobs: Vec<Job>,
        /// The constraints involved (claims, copies, precedences).
        constraints: Vec<ConstraintId>,
        /// What is wrong.
        reason: String,
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
    /// The input topology, model or intent is unusable.
    #[error("input: {0}")]
    Input(String),
    /// Re-planning after fallbacks did not converge.
    #[error("gave up after {} attempts", attempts.len())]
    ReplanLimit {
        /// Everything tried.
        attempts: Vec<Attempt>,
    },
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
    /// The `(target, scheme)` the driver should reject to re-plan after this error, if the
    /// error is something a different scheme choice could avoid.
    pub fn blame(&self) -> Option<(Target, SchemeName)> {
        match self {
            SchemeError::Infeasible(i) => i.blame.clone(),
            SchemeError::Generate(g) if g.kind.allows_fallback() => {
                Some((g.target, g.scheme.clone()))
            }
            SchemeError::NoApplicableScheme { imposed_by, .. } => imposed_by.first().cloned(),
            _ => None,
        }
    }
}

/// Result alias.
pub type Result<T, E = SchemeError> = core::result::Result<T, E>;
