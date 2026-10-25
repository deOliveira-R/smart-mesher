//! Contract between the meshing driver and every meshing scheme (curve, surface, volume) and
//! the sizing layer. Approach: **constraint system first**, with capability impositions, claims
//! and footprint scheduling.
//!
//! 1. Intent ([`MeshIntent`], serde data that lives in an Op and in git) says what was asked.
//! 2. Schemes are mostly declarative. In planning they only *contribute* constraints to one
//!    global [`MeshingProblem`]: child impositions over capabilities ([`Caps`], [`Need`]),
//!    claims (a parent writes a child itself), roles, corner rules and vertex types
//!    ([`VertexRule`], [`VertexType`] per [`sm_topo_api::CornerKey`]), copy equalities
//!    ([`CopyConstraint`], also generated from the model's deck pairs) and linear interval
//!    rows ([`IntervalConstraint`]). The driver adds sizing goals and deck constraints.
//! 3. Copy constraints form equivalence classes ([`CopyClass`]); each class is meshed from one
//!    root and the other members derive from it, or are verified when a parent writes them.
//! 4. The problem is solved globally ([`CornerPicker`], then [`IntervalSolver`]) into a
//!    serializable [`Plan`] before any mesh exists; footprints derived from the plan are
//!    levelled into waves ([`Footprint`], [`Job`]).
//! 5. Generation is a pure function of the plan: each scheme returns a [`Patch`] whose nodes
//!    have canonical [`NodeKey`]s. Jobs of one wave run concurrently ([`Executor`]); the
//!    driver merges and commits patches in a fixed order into an `sm_mesh_api::MeshBuild`, so
//!    1 thread and N threads give bit-identical meshes.
//! 6. A failure is structured ([`FailureKind`], [`Attempt`]); the driver rejects the blamed
//!    `(target, scheme)` and re-plans.
//!
//! Volumes are pieces of the decomposition layer (`sm_model_api::PieceRef`), named by
//! `TopoVolume` / `PartitionCell` ids; see [`Target`]. See `DESIGN.md`.

pub mod caps;
mod classes;
pub mod copy;
pub mod driver;
pub mod error;
pub mod exec;
pub mod firm;
pub mod geom;
pub mod intent;
pub mod patch;
pub mod plan;
mod planner;
pub mod problem;
mod schedule;
pub mod scheme;
pub mod sizing;
pub mod solve;
/// Stub scheme implementors and fixtures for tests (feature `stubs`).
#[cfg(feature = "stubs")]
#[doc(hidden)]
pub mod stubs;
pub mod target;
pub mod vertex;

pub use caps::Caps;
/// Re-export of [`sm_mesh_api::Param`], so scheme crates need no direct `sm-mesh-api` edge for it.
pub use sm_mesh_api::Param;
pub use driver::{Driver, DriverOptions, Meshed, Rejections};
pub use error::{Attempt, FailureKind, GenerateFailure, Infeasible, Phase, Result, SchemeError};
pub use exec::{Executor, ScopedThreads, Sequential};
pub use firm::{Firm, Firmness};
pub use intent::{
    AutoPolicy, BoundaryLayerIntent, CornerIntent, Defaults, EntityIntent, IntervalRequest,
    MeshGroup, MeshIntent, Params, SchemeChoice, SchemeName, SizingSpec, TransformIntent,
};
pub use patch::{ElemShape, Elements, NodeGrid, Patch, PatchBuilder, PatchNode, PatchSet};
pub use plan::{CopyClass, CopyMember, CopyMode, Footprint, Plan, TransformPlan};
pub use problem::{
    Claim, ConstraintId, CopyConstraint, CopyReason, Imposition, IntervalConstraint, IntervalSink,
    MeshConstraint, MeshingProblem, Need, Origin, Record, Rel, Requirements, Role, Selection,
    TargetPlan,
};
pub use scheme::{
    Applicability, BoundaryLoop, BoundaryUse, DecompositionFaces, GenCtx, IntervalCtx, MeshInput,
    MeshTransform, PieceResolver, PlanCtx, Scheme, SchemeRegistry, WholeFaces,
};
pub use sizing::{ConstantSize, SizeField, SymMat3};
pub use solve::{CornerPicker, IntervalSolver, ReferenceCornerPicker, ReferenceIntervalSolver};
pub use target::{GroupId, Job, NodeKey, SchemeDim, Target, TransformId, piece_of};
pub use vertex::{CornerAssignment, FaceLayout, LogicalSide, SideDir, VertexRule, VertexType};
