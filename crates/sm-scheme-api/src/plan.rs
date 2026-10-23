//! The solved [`Plan`]: the problem plus its solution (intervals, corner layouts, copy
//! classes, footprints, generation waves). Generation is a pure function of the plan and the
//! inputs.

use crate::driver::Rejections;
use crate::error::Attempt;
use crate::intent::{SchemeChoice, SizingSpec};
use crate::problem::{ConstraintId, Lazy, MeshingProblem, Role, TargetPlan};
use crate::target::{Job, Target, TransformId};
use crate::vertex::FaceLayout;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Transform};
use std::collections::BTreeMap;

/// How a copy-class member gets its mesh.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CopyMode {
    /// The driver generates it as the image of the class root.
    Generate,
    /// Another job writes it (it is claimed by a volume). After generation the driver checks
    /// that it equals the image of the root and fails with `CopyMismatch` otherwise.
    Verify,
}

/// One non-root member of a copy class.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CopyMember {
    /// The member.
    pub target: Target,
    /// Placement of the member relative to the root (composed along the class tree).
    pub from_root: Transform,
    /// Vertex correspondence `(root vertex, member vertex)`, sorted by root vertex (composed
    /// along the class tree).
    pub vertex_map: Vec<(EntityId, EntityId)>,
    /// Generate or verify.
    pub mode: CopyMode,
    /// The copy constraints on the tree path from the root to this member.
    pub constraints: Vec<ConstraintId>,
}

/// A set of faces (or of curves) that carry the same mesh up to placement. Built from the
/// copy constraints, which are edges of an equivalence relation: one root is meshed by a
/// scheme (preference: a sweep source, then a face already written by a scheme or a parent,
/// then a hard intent, then the lowest id) and every other member derives from it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CopyClass {
    /// The member meshed by a scheme (or written by its claimer).
    pub root: Target,
    /// All other members, ascending by target.
    pub members: Vec<CopyMember>,
}

/// What a job reads and writes. Derived by the driver, never written by a scheme:
/// - writes = the job's own target plus the children it claims;
/// - reads = its boundary children it does not claim, its copy-class root, its `Precedes`
///   predecessors, and (for a transform) its targets.
///
/// A job runs after the writer of everything it reads.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Footprint {
    /// The job.
    pub job: Job,
    /// Targets read (ascending).
    pub reads: Vec<Target>,
    /// Targets written (ascending).
    pub writes: Vec<Target>,
}

/// A standalone transform as scheduled.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TransformPlan {
    /// The id.
    pub id: TransformId,
    /// The patch holders it rewrites (volume or group targets, ascending).
    pub targets: Vec<Target>,
    /// The transform and its parameters.
    pub transform: SchemeChoice,
}

#[derive(Debug, Default)]
pub(crate) struct PlanIndex {
    member_of: BTreeMap<Target, (usize, Option<usize>)>,
}

/// A solved meshing problem. Serializable and comparable, so it can be cached, diffed and
/// checked for determinism. A stored plan reproduces its mesh exactly.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Plan {
    /// The assembled problem (with every constraint's origin, for diagnostics).
    pub problem: MeshingProblem,
    /// Solved interval count per edge, ascending by edge.
    pub intervals: Vec<(EntityId, u32)>,
    /// Solved corner layout per face with a vertex rule, ascending by face.
    pub layouts: Vec<FaceLayout>,
    /// Copy classes, ascending by root.
    pub copies: Vec<CopyClass>,
    /// Standalone transforms (and group conversions), by id.
    pub transforms: Vec<TransformPlan>,
    /// Footprints of every job, in job order.
    pub footprints: Vec<Footprint>,
    /// Generation waves of independent jobs, in order; sorted inside a wave.
    pub waves: Vec<Vec<Job>>,
    /// The `(target, scheme)` pairs rejected after failures, which this plan avoids.
    pub rejections: Rejections,
    /// Every candidate that was evaluated and did not win (applicability, impositions,
    /// scoring, earlier rejections), in evaluation order.
    pub attempts: Vec<Attempt>,
    /// The sizing of the intent this plan came from, so `generate` can rebuild the size field.
    #[serde(default)]
    pub sizing: Option<SizingSpec>,
    #[serde(skip)]
    pub(crate) index: Lazy<PlanIndex>,
}

impl Plan {
    /// Solved interval count of `edge`.
    pub fn intervals(&self, edge: EntityId) -> Option<u32> {
        self.intervals
            .binary_search_by(|p| p.0.cmp(&edge))
            .ok()
            .map(|i| self.intervals[i].1)
    }

    /// Corner layout of `face`.
    pub fn layout(&self, face: EntityId) -> Option<&FaceLayout> {
        self.layouts
            .binary_search_by(|l| l.face.cmp(&face))
            .ok()
            .map(|i| &self.layouts[i])
    }

    /// The plan row of `target`.
    pub fn target(&self, t: Target) -> Option<&TargetPlan> {
        self.problem.target(t)
    }

    /// The class and member row of `t` (`None` for roots and for targets in no class).
    pub fn copy_of(&self, t: Target) -> Option<(&CopyClass, &CopyMember)> {
        let ix = self.index.0.get_or_init(|| {
            let mut m = BTreeMap::new();
            for (c, class) in self.copies.iter().enumerate() {
                m.insert(class.root, (c, None));
                for (k, mem) in class.members.iter().enumerate() {
                    m.insert(mem.target, (c, Some(k)));
                }
            }
            PlanIndex { member_of: m }
        });
        match ix.member_of.get(&t)? {
            (c, Some(k)) => Some((&self.copies[*c], &self.copies[*c].members[*k])),
            _ => None,
        }
    }

    /// The class `t` belongs to, as root or member.
    pub fn class_of(&self, t: Target) -> Option<&CopyClass> {
        self.copy_of(t);
        let ix = self.index.0.get()?;
        ix.member_of.get(&t).map(|(c, _)| &self.copies[*c])
    }

    /// Roles of `t`.
    pub fn roles_of(&self, t: Target) -> impl Iterator<Item = Role> + '_ {
        self.problem.roles_of(t)
    }

    /// The parent that writes `child`, if it is claimed.
    pub fn claimed_by(&self, child: Target) -> Option<Target> {
        self.problem.claim_of(child).map(|c| c.item.by)
    }
}
