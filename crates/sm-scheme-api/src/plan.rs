//! The solved [`Plan`]: the problem plus its solution (intervals, corner layouts, generation
//! waves). Generation is a pure function of the plan and the inputs.

use crate::problem::{CopyConstraint, MeshingProblem, Record, Role, TargetPlan};
use crate::target::Target;
use crate::vertex::FaceLayout;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;

/// A solved meshing problem. Serializable and comparable, so it can be cached, diffed and
/// checked for determinism.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Plan {
    /// The assembled problem (with every constraint's origin, for diagnostics).
    pub problem: MeshingProblem,
    /// Solved interval count per edge, ascending by edge.
    pub intervals: Vec<(EntityId, u32)>,
    /// Solved corner layout per face with a vertex rule, ascending by face.
    pub layouts: Vec<FaceLayout>,
    /// Generation waves. Targets inside a wave are independent (they may run concurrently);
    /// waves run in order: curves, surfaces, volumes, groups, each split further by copy and
    /// precedence depth.
    pub waves: Vec<Vec<Target>>,
}

impl Plan {
    /// Solved interval count of `edge`.
    pub fn intervals(&self, edge: EntityId) -> Option<u32> {
        self.intervals.binary_search_by(|p| p.0.cmp(&edge)).ok().map(|i| self.intervals[i].1)
    }

    /// Corner layout of `face`.
    pub fn layout(&self, face: EntityId) -> Option<&FaceLayout> {
        self.layouts.binary_search_by(|l| l.face.cmp(&face)).ok().map(|i| &self.layouts[i])
    }

    /// The plan row of `target`.
    pub fn target(&self, t: Target) -> Option<&TargetPlan> {
        self.problem.target(t)
    }

    /// The copy constraint whose target is `t`.
    pub fn copy_into(&self, t: Target) -> Option<&Record<CopyConstraint>> {
        self.problem.copy_into(t)
    }

    /// Roles of `t`.
    pub fn roles_of(&self, t: Target) -> impl Iterator<Item = Role> + '_ {
        self.problem.roles_of(t)
    }
}
