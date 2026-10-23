//! Contract between the meshing driver and every meshing scheme (curve, surface, volume and
//! transform schemes), plus the sizing seam.
//!
//! Approach: **multi-entity transactions**. The unit of work is a [`MeshJob`] over a *set* of
//! targets (one volume, a multisweep group, a merged-volume cluster, a deck-paired face). A job
//! declares a read/write [`Footprint`]; jobs with disjoint footprints run concurrently; each
//! produces a [`MeshPatch`] that the [`Ledger`] commits atomically, in job-id order. Copy and
//! mirror are first-class job kinds ([`JobKind::Copy`]).
//!
//! | Module | Content |
//! |---|---|
//! | [`intent`] | serializable scheme choices, parameters, intervals, vertex types, groups |
//! | [`target`] | [`Target`] (what is meshed) and [`NodeKey`] (owner-keyed node names) |
//! | [`scheme`] | the [`Scheme`] trait, plan/exec contexts, impositions, applicability, registry |
//! | [`job`] | [`MeshJob`], [`Footprint`], [`JobKind`], [`CopySpec`] |
//! | [`patch`] | [`MeshPatch`] (columnar delta) and [`PatchBuilder`] |
//! | [`ledger`] | committed meshes, commit rules, the writer into `sm_mesh_api::MeshBuild` |
//! | [`interval`] | [`IntervalConstraint`], the [`IntervalSolver`] seam and a reference solver |
//! | [`vertex`] | loop corners, logical sides, edge meshes in loop direction |
//! | [`copy`] | entity correspondence, copy execution, deck verification |
//! | [`driver`] | the reference driver: build, solve, schedule, execute, fallback loop |
//!
//! See `DESIGN.md` for the rationale.

pub mod copy;
pub mod driver;
pub mod error;
pub mod intent;
pub mod interval;
pub mod job;
pub mod ledger;
pub mod patch;
pub mod scheme;
#[cfg(feature = "serde")]
pub(crate) mod serde_pairs;
pub mod target;
pub mod vertex;

pub use driver::{Executor, MeshProblem, ProblemInput, RunOutput, Schedule, SequentialExecutor, run};
pub use error::{Attempt, DriverError, FailureKind, PatchError, Retry, SchemeFailure};
pub use intent::{Firmness, GroupIntent, GroupKind, IntervalSpec, MeshIntent, ParamValue, Params, SchemeChoice, SchemeIntent, SchemeName, SchemeSpec, TargetIntent, TransformIntent, VertexType, VertexTypeSetting};
pub use interval::{ConstraintKind, IntervalConstraint, IntervalError, IntervalSolution, IntervalSolver, Origin, ReferenceIntervalSolver, Relation, Term};
pub use job::{CopyReason, CopySpec, EntityMap, Footprint, JobId, JobKind, MeshJob, SweepRole};
pub use ledger::{CellMesh, FaceMesh, Ledger, NodeRec, WriteMap};
pub use patch::{MeshPatch, Part, PatchBuilder, PatchNode};
pub use scheme::{Applicability, ExecCtx, Imposition, JobPlan, JobRequest, PlanCtx, Requirement, Scheme, SchemeDim, SchemeRegistry, SizeField, VertexTypes};
pub use target::{NodeKey, Target};
