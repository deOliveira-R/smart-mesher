//! The scheme traits: explicit `applicability -> plan -> apply` phases.
//!
//! - `applicability` and `plan` run top-down (volumes, then faces, then curves) on a
//!   [`PlanCtx`]: shape queries plus the requirements parents imposed on the entity. They never
//!   see a mesh.
//! - `apply` runs bottom-up (curves, faces, volumes) on an input that holds the committed
//!   boundary mesh and the solved intervals, and returns a local patch. It must be a pure
//!   function of its input: the driver may run it on any thread.

use crate::caps::Caps;
use crate::error::SchemeError;
use crate::intent::Params;
use crate::interval::IntervalSolution;
use crate::key::ShapeKey;
use crate::mesh::CommittedMesh;
use crate::patch::{CurvePatch, NodeId, SurfacePatch, TransformPatch, VolumePatch};
use crate::plan::{Applicability, ChildRequirement, Plan};
use crate::shape::{FaceUseRef, ShapeView};
use crate::size::SizeField;
use crate::vertex::{CornerAssignment, CornerPicker};
use sm_topo_api::{Dim, Sense};

/// Planning context for one entity (or one atomic group).
#[derive(Clone, Copy)]
pub struct PlanCtx<'a> {
    /// Shape queries.
    pub shape: &'a dyn ShapeView,
    /// The entities to plan (one, or the members of an atomic group), ascending.
    pub targets: &'a [ShapeKey],
    /// The scheme's parameters from the intent.
    pub params: &'a Params,
    /// Requirements imposed on the targets by parents or decks.
    pub imposed: &'a [ChildRequirement],
    /// The shared corner picker.
    pub corners: &'a dyn CornerPicker,
    /// Vertex types set in the intent for the target face(s).
    pub vertex_types: &'a [CornerAssignment],
}

impl PlanCtx<'_> {
    /// The single target (first of a group).
    pub fn target(&self) -> ShapeKey {
        self.targets[0]
    }
}

/// `apply` input of a curve scheme.
#[derive(Clone, Copy)]
pub struct CurveInput<'a> {
    /// Shape queries.
    pub shape: &'a dyn ShapeView,
    /// Everything committed so far (copy sources).
    pub mesh: &'a CommittedMesh,
    /// The curve.
    pub curve: ShapeKey,
    /// Solved interval count (`>= 1`).
    pub intervals: u32,
    /// Arc length.
    pub length: f64,
    /// True if the curve starts and ends at the same vertex.
    pub closed: bool,
    /// The plan.
    pub plan: &'a Plan,
    /// Requirements imposed on the curve.
    pub imposed: &'a [ChildRequirement],
    /// Optional size field.
    pub size: Option<&'a dyn SizeField>,
}

/// One curve use of a discretized boundary loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BoundaryUse {
    /// The curve.
    pub curve: ShapeKey,
    /// Traversal sense relative to the curve.
    pub sense: Sense,
    /// Index into the loop's `nodes` of the node where this use starts.
    pub first: u32,
    /// Number of segments (the curve's interval count).
    pub segments: u32,
}

/// A discretized face loop: committed nodes in loop order (each node once).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoundaryLoop {
    /// Loop index (0 = outer).
    pub ring: u16,
    /// Nodes in loop order, cyclic.
    pub nodes: Vec<NodeId>,
    /// The curve uses, in loop order.
    pub uses: Vec<BoundaryUse>,
}

/// `apply` input of a surface scheme.
#[derive(Clone, Copy)]
pub struct SurfaceInput<'a> {
    /// Shape queries.
    pub shape: &'a dyn ShapeView,
    /// Everything committed so far.
    pub mesh: &'a CommittedMesh,
    /// The face.
    pub face: ShapeKey,
    /// Discretized loops, outer first.
    pub loops: &'a [BoundaryLoop],
    /// The plan.
    pub plan: &'a Plan,
    /// Requirements imposed on the face.
    pub imposed: &'a [ChildRequirement],
    /// Solved intervals.
    pub intervals: &'a IntervalSolution,
    /// Optional size field.
    pub size: Option<&'a dyn SizeField>,
}

/// `apply` input of a volume scheme (or atomic group).
#[derive(Clone, Copy)]
pub struct VolumeInput<'a> {
    /// Shape queries.
    pub shape: &'a dyn ShapeView,
    /// Everything committed so far (all unclaimed faces of the targets are meshed).
    pub mesh: &'a CommittedMesh,
    /// The targets, ascending.
    pub targets: &'a [ShapeKey],
    /// Face uses per target (same order as `targets`).
    pub faces: &'a [Vec<FaceUseRef>],
    /// The plan.
    pub plan: &'a Plan,
    /// Requirements imposed on the targets.
    pub imposed: &'a [ChildRequirement],
    /// Solved intervals.
    pub intervals: &'a IntervalSolution,
    /// Optional size field.
    pub size: Option<&'a dyn SizeField>,
}

/// Phases shared by all schemes.
pub trait SchemeCore: Send + Sync {
    /// Stable name (the registry key and the name stored in intent).
    fn id(&self) -> &str;
    /// Dimension of the entities it meshes.
    fn dim(&self) -> Dim;
    /// What its output is ([`Caps::INHERIT`] for copy schemes).
    fn caps(&self) -> Caps;
    /// Can it mesh `cx.targets`, and how well? Cheap; no mesh is available.
    fn applicability(&self, cx: &PlanCtx<'_>) -> Applicability;
    /// Requirements on children, claims, ordering and interval constraints.
    fn plan(&self, cx: &PlanCtx<'_>) -> Result<Plan, SchemeError>;
}

/// A curve scheme (equal, bias, dualbias, curvature, stretch, pinpoint, copy).
pub trait CurveScheme: SchemeCore {
    /// Interior node fractions for the solved interval count.
    fn apply(&self, input: &CurveInput<'_>) -> Result<CurvePatch, SchemeError>;
}

/// A surface scheme (map, submap, pave, tri*, circle, hole, polyhedron, copy, mirror).
pub trait SurfaceScheme: SchemeCore {
    /// Interior nodes and elements for the discretized boundary.
    fn apply(&self, input: &SurfaceInput<'_>) -> Result<SurfacePatch, SchemeError>;
}

/// A volume scheme (map, submap, sweep, tetmesh, copy, polyhedron). With several targets it is
/// an atomic group scheme (multisweep, merged volumes).
pub trait VolumeScheme: SchemeCore {
    /// Cells (and claimed faces) for the meshed boundary.
    fn apply(&self, input: &VolumeInput<'_>) -> Result<VolumePatch, SchemeError>;
}

/// A conversion over a set of already meshed volumes (THex, QTri, HTet), committed all at once.
pub trait MeshTransform: Send + Sync {
    /// Stable name.
    fn id(&self) -> &str;
    /// Capabilities of the result.
    fn caps(&self) -> Caps;
    /// Can it convert `cx.targets` (all of them, together)?
    fn applicability(&self, cx: &PlanCtx<'_>) -> Applicability;
    /// Replacement cells for all targets plus every surface mesh it changes.
    fn apply(&self, shape: &dyn ShapeView, mesh: &CommittedMesh, targets: &[ShapeKey]) -> Result<TransformPatch, SchemeError>;
}
