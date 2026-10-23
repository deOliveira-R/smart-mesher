//! What a scheme says *before* it meshes: applicability, child requirements, claims, ordering
//! and interval constraints. Plans are plain serde data, so a whole meshing plan can be
//! previewed, diffed and stored.

use crate::caps::{Caps, Role};
use crate::intent::{Firmness, Params, SchemeId};
use crate::interval::IntervalConstraint;
use crate::key::ShapeKey;
use crate::vertex::CornerAssignment;
use serde::{Deserialize, Serialize};
use sm_core::{Transform, Vector3};
use sm_topo_api::Sense;

/// Result of `applicability`: a score in `[0, 1]` (0 = cannot mesh) and a reason for the user.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Applicability {
    /// 0 = not applicable; higher is preferred by auto selection.
    pub score: f64,
    /// Why (shown in auto-selection reports).
    pub reason: String,
}

impl Applicability {
    /// Applicable with `score` (clamped to `[0, 1]`).
    pub fn yes(score: f64, reason: impl Into<String>) -> Self {
        Applicability { score: if score.is_finite() { score.clamp(0.0, 1.0) } else { 0.0 }, reason: reason.into() }
    }
    /// Not applicable.
    pub fn no(reason: impl Into<String>) -> Self {
        Applicability { score: 0.0, reason: reason.into() }
    }
    /// True if the score is positive.
    pub fn is_applicable(&self) -> bool {
        self.score > 0.0
    }
}

/// A requirement a plan (or the driver, for decks) imposes on another entity.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Requirement {
    /// The entity's scheme must provide these capabilities (map volume: `QUAD | STRUCTURED` on
    /// every face; sweep: the same on linking faces).
    Caps {
        /// Required bits.
        need: Caps,
        /// `Hard` filters candidates; `Soft` only lowers the score of auto candidates.
        firmness: Firmness,
    },
    /// The entity's scheme must not have these capabilities (deck faces: `BOUNDARY_LAYER`).
    Forbid {
        /// Forbidden bits.
        caps: Caps,
    },
    /// The entity's scheme must be one of these (Cubit-style naming, when a property is not
    /// enough).
    Scheme {
        /// Allowed schemes.
        allowed: Vec<SchemeId>,
        /// Firmness.
        firmness: Firmness,
    },
    /// The entity plays this role for the imposing plan.
    Role(Role),
    /// The entity's mesh must be the image of `source`'s mesh under `transform` (deck pair,
    /// copy mesh). The driver meshes it with the copy scheme of its dimension, after `source`.
    CopyOf {
        /// The entity to copy from (same dimension).
        source: ShapeKey,
        /// Source -> this entity.
        transform: Transform,
        /// Relative direction (curves: `Reversed` = the image runs against this curve).
        sense: Sense,
    },
    /// Nodes on the entity stay on this plane (mirror face; a smoothing constraint).
    OnPlane {
        /// Unit normal.
        normal: Vector3,
        /// Offset (`n . p = offset`).
        offset: f64,
    },
}

/// A requirement on `target`, with who imposed it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ChildRequirement {
    /// The constrained entity.
    pub target: ShapeKey,
    /// What is required.
    pub req: Requirement,
    /// The entity whose plan imposed it (for a deck: the B face itself).
    pub by: ShapeKey,
}

/// A scheme's plan for one entity or one atomic group.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Plan {
    /// The scheme that made the plan.
    pub scheme: SchemeId,
    /// The entities meshed by this plan (one, or several for an atomic group), ascending.
    pub targets: Vec<ShapeKey>,
    /// Lower-dimensional entities this plan meshes itself (a sweep's target face). The driver
    /// does not schedule them separately; the volume patch must contain them.
    pub claims: Vec<ShapeKey>,
    /// Same-dimension entities whose mesh must be committed before this plan is applied.
    pub after: Vec<ShapeKey>,
    /// Requirements on children (lower dimension).
    pub requires: Vec<ChildRequirement>,
    /// Interval constraints.
    pub intervals: Vec<IntervalConstraint>,
    /// Vertex types this plan relies on (reported back; soft unless the user set them).
    pub vertex_types: Vec<CornerAssignment>,
    /// Scheme-private data computed in `plan` and read in `apply` (logical sides, roles).
    pub data: Params,
}

impl Plan {
    /// A plan for one target with nothing else.
    pub fn single(scheme: &str, target: ShapeKey) -> Self {
        Plan {
            scheme: SchemeId::new(scheme),
            targets: vec![target],
            claims: Vec::new(),
            after: Vec::new(),
            requires: Vec::new(),
            intervals: Vec::new(),
            vertex_types: Vec::new(),
            data: Params::new(),
        }
    }

    /// Builder: require `req` on `target`, imposed by this plan's first target.
    pub fn require(&mut self, target: ShapeKey, req: Requirement) {
        let by = self.targets.first().copied().unwrap_or(target);
        self.requires.push(ChildRequirement { target, req, by });
    }
}
