//! The scheme contract: one trait for every dimension, with a plan phase (before intervals are
//! known) and an execute phase (pure function of committed meshes -> patch).

use crate::error::SchemeFailure;
use crate::intent::{Firmness, GroupKind, MeshIntent, Params, SchemeName, SchemeSpec, VertexTypeSetting};
use crate::interval::{IntervalConstraint, IntervalSolution};
use crate::job::{Footprint, MeshJob, SweepRole};
use crate::ledger::Ledger;
use crate::patch::MeshPatch;
use crate::target::Target;
use sm_core::{Point3, Transform};
use sm_model_api::{Model, Plane};
use sm_topo_api::{CornerKey, TopologyView};
use std::collections::BTreeMap;
use std::sync::Arc;

/// Resolved vertex types per vertex use.
pub type VertexTypes = BTreeMap<CornerKey, VertexTypeSetting>;

/// Which kind of target a scheme meshes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SchemeDim {
    /// Edges: node distribution along a curve.
    Curve,
    /// Faces.
    Surface,
    /// Volumes and pieces (one or a group).
    Volume,
    /// Rewrites committed meshes (THex, QTri, HTet, boundary layers).
    Transform,
}

impl SchemeDim {
    /// The scheme dimension that meshes a target of topological dimension `d`.
    pub const fn for_dim(d: u8) -> Option<Self> {
        match d {
            1 => Some(SchemeDim::Curve),
            2 => Some(SchemeDim::Surface),
            3 => Some(SchemeDim::Volume),
            _ => None,
        }
    }
}

/// Whether a scheme fits a request, for auto selection and diagnostics.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Applicability {
    /// Fits. Higher scores win auto selection (Cubit's preference order map > submap > pave
    /// becomes scores); ties break by scheme name.
    Applicable {
        /// Preference score.
        score: u32,
        /// Why (e.g. "4 corners, opposite sides matchable").
        reason: String,
    },
    /// Does not fit.
    NotApplicable {
        /// Why.
        reason: String,
    },
}

impl Applicability {
    /// The score, if applicable.
    pub fn score(&self) -> Option<u32> {
        match self {
            Applicability::Applicable { score, .. } => Some(*score),
            Applicability::NotApplicable { .. } => None,
        }
    }
    /// The reason text.
    pub fn reason(&self) -> &str {
        match self {
            Applicability::Applicable { reason, .. } | Applicability::NotApplicable { reason } => reason,
        }
    }
}

/// What a parent job (or the model) requires of a target it does not write.
#[derive(Clone, Debug, PartialEq)]
pub enum Requirement {
    /// The target must use this scheme (map volume -> map faces and curves).
    Scheme(SchemeSpec),
    /// The target plays this role in a sweep (a `Source` must be meshed before the sweep runs).
    Role(SweepRole),
    /// No boundary layer may be grown on the target (every deck face).
    NoBoundaryLayer,
    /// Nodes on the target must stay on this plane (mirror deck faces; a smoothing constraint).
    OnPlane(Plane),
    /// The target's mesh must be the image of `source`'s under `transform` (translation /
    /// rotation deck side B). Met by a copy job, or verified after commit when another job (a
    /// sweep) writes the target.
    CopyOf {
        /// Side-A target.
        source: Target,
        /// A -> B transform.
        transform: Transform,
    },
}

/// A requirement with its firmness, on one target.
#[derive(Clone, Debug, PartialEq)]
pub struct Imposition {
    /// Target the requirement is on.
    pub target: Target,
    /// The requirement.
    pub requirement: Requirement,
    /// Firmness (a parent scheme imposes `Soft`, the model's deck rules `Hard`).
    pub firmness: Firmness,
}

/// Element size field (implemented in `sm-sizing`).
pub trait SizeField: Send + Sync {
    /// Isotropic target size at `p`.
    fn size(&self, p: Point3) -> f64;
    /// Optional anisotropic metric (symmetric 3x3, row-major) at `p`.
    fn metric(&self, _p: Point3) -> Option<[[f64; 3]; 3]> {
        None
    }
}

/// Read-only inputs for planning.
#[derive(Clone, Copy)]
pub struct PlanCtx<'a> {
    /// Topology (an `Overlay` exposes virtual partition faces/edges the same way).
    pub topo: &'a dyn TopologyView,
    /// Model semantics and decomposition.
    pub model: &'a Model,
    /// The intent.
    pub intent: &'a MeshIntent,
    /// Optional size field.
    pub size: Option<&'a dyn SizeField>,
}

/// What a scheme is asked to plan: one job over a set of targets.
#[derive(Clone, Copy, Debug)]
pub struct JobRequest<'a> {
    /// Targets, ascending (one volume, a group, one face, one edge).
    pub targets: &'a [Target],
    /// Scheme parameters.
    pub params: &'a Params,
    /// Requirements other jobs or the model placed on these targets.
    pub imposed: &'a [Imposition],
    /// Group kind for multi-target jobs.
    pub group: Option<GroupKind>,
}

/// A scheme's plan for one job, produced before any meshing.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct JobPlan {
    /// Read/write sets. Writes must include every request target, plus whatever the scheme meshes
    /// itself (a sweep writes its target and linking faces).
    pub footprint: Footprint,
    /// Interval constraints for the global solve.
    pub constraints: Vec<IntervalConstraint>,
    /// Vertex types the scheme picked (usually `Soft`).
    pub vertex_types: Vec<(CornerKey, VertexTypeSetting)>,
    /// Requirements on targets the job reads (map volume -> map faces; sweep -> source role).
    pub impositions: Vec<Imposition>,
    /// Roles assigned to targets (kept on the job).
    pub roles: BTreeMap<Target, SweepRole>,
}

/// Read-only inputs for execution.
#[derive(Clone, Copy)]
pub struct ExecCtx<'a> {
    /// Topology.
    pub topo: &'a dyn TopologyView,
    /// Model.
    pub model: &'a Model,
    /// Meshes committed by earlier waves (everything in the job's read set is there).
    pub ledger: &'a Ledger,
    /// Solved interval counts.
    pub intervals: &'a IntervalSolution,
    /// Resolved vertex types.
    pub vertex_types: &'a VertexTypes,
    /// Optional size field.
    pub size: Option<&'a dyn SizeField>,
}

/// A meshing scheme of any dimension.
///
/// Contract:
/// - `applicability` and `plan` are cheap and side-effect free; `plan` may fail with a
///   [`SchemeFailure`] whose `retry` says whether another candidate should be tried.
/// - `execute` is a pure function of `(ctx, job)`: it reads only targets in
///   `job.footprint.reads` (plus model vertices) and returns a patch writing exactly
///   `job.footprint.writes`. It must be deterministic and must not depend on thread count; it may
///   use data parallelism internally if the result is bit-identical.
/// - Nodes follow the [`crate::NodeKey`] conventions, so separately meshed entities are conformal.
pub trait Scheme: Send + Sync {
    /// Registered name.
    fn name(&self) -> &str;
    /// What it meshes.
    fn dim(&self) -> SchemeDim;
    /// Whether it fits, with a score and a reason.
    fn applicability(&self, cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Applicability;
    /// Plan the job: footprint, constraints, vertex types, impositions on children.
    fn plan(&self, cx: &PlanCtx<'_>, req: &JobRequest<'_>) -> Result<JobPlan, SchemeFailure>;
    /// Produce the patch.
    fn execute(&self, cx: &ExecCtx<'_>, job: &MeshJob) -> Result<MeshPatch, SchemeFailure>;
}

/// Scheme registry: unique names across all dimensions, iterated in name order.
#[derive(Clone, Default)]
pub struct SchemeRegistry {
    schemes: BTreeMap<SchemeName, Arc<dyn Scheme>>,
}

impl SchemeRegistry {
    /// Empty registry.
    pub fn new() -> Self {
        Self::default()
    }
    /// Register a scheme; returns `false` (and keeps the old one) if the name is taken.
    pub fn register(&mut self, s: Arc<dyn Scheme>) -> bool {
        let name = SchemeName::new(s.name());
        if self.schemes.contains_key(&name) {
            return false;
        }
        self.schemes.insert(name, s);
        true
    }
    /// Look up a scheme.
    pub fn get(&self, name: &SchemeName) -> Option<&Arc<dyn Scheme>> {
        self.schemes.get(name)
    }
    /// Registered names of one dimension, ascending.
    pub fn names_of(&self, dim: SchemeDim) -> Vec<SchemeName> {
        self.schemes.iter().filter(|(_, s)| s.dim() == dim).map(|(n, _)| n.clone()).collect()
    }
}
