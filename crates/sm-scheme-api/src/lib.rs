//! Contract between the meshing driver and every meshing scheme (curve, surface, volume) and
//! the sizing layer. Approach: **constraint system first**.
//!
//! 1. Intent ([`MeshIntent`], serde data that lives in an Op and in git) says what was asked.
//! 2. Schemes are mostly declarative. In planning they only *contribute* constraints to one
//!    global [`MeshingProblem`]: child-scheme impositions and roles ([`Requirements`]), corner
//!    rules and vertex types ([`VertexRule`], [`VertexType`] per [`sm_topo_api::CornerKey`]),
//!    copy equalities ([`CopyConstraint`], also generated from the model's deck pairs) and
//!    interval equations ([`IntervalConstraint`]: hard/soft counts, side sums, parity, sweep
//!    layers). The driver adds sizing goals and deck constraints.
//! 3. The problem is solved globally ([`CornerPicker`], then [`IntervalSolver`]) into a
//!    serializable [`Plan`] before any mesh exists.
//! 4. Generation is a pure function of the plan: each scheme returns a [`Patch`] whose nodes
//!    have canonical [`NodeKey`]s. Targets of one wave run concurrently ([`Executor`]); the
//!    driver merges and commits patches in a fixed order into an `sm_mesh_api::MeshBuild`, so
//!    1 thread and N threads give bit-identical meshes.
//! 5. A generation failure is structured ([`GenerateFailure`]); fallback-eligible ones make
//!    the driver reject `(target, scheme)` and re-plan.
//!
//! Volumes are pieces of the decomposition layer (`sm_model_api::PieceRef`), named by
//! `TopoVolume` / `PartitionCell` ids; see [`Target`]. See `DESIGN.md`.

pub mod copy;
pub mod driver;
pub mod error;
pub mod exec;
pub mod firm;
pub mod geom;
pub mod intent;
pub mod patch;
pub mod plan;
pub mod problem;
pub mod scheme;
pub mod sizing;
pub mod solve;
pub mod target;
pub mod vertex;

pub use driver::{Driver, DriverOptions, Meshed, Rejections};
pub use error::{FailureKind, GenerateFailure, Infeasible, Result, SchemeError};
pub use exec::{Executor, ScopedThreads, Sequential};
pub use firm::{Firm, Firmness};
pub use intent::{AutoPolicy, BoundaryLayerIntent, CornerIntent, Defaults, EntityIntent, IntervalRequest, MeshGroup, MeshIntent, Params, SchemeChoice, SchemeName, SizingSpec};
pub use patch::{ElemShape, Elements, NodeGrid, Patch, PatchNode, PatchSet};
pub use plan::Plan;
pub use problem::{ConstraintId, CopyConstraint, CopyReason, Imposition, IntervalConstraint, IntervalSink, MeshConstraint, MeshingProblem, Origin, Record, Requirements, Role, Selection, TargetPlan};
pub use scheme::{Applicability, GenCtx, IntervalCtx, MeshInput, MeshTransform, PieceResolver, PlanCtx, Scheme, SchemeRegistry, WholeFaces};
pub use sizing::{ConstantSize, SizeField, SymMat3};
pub use solve::{CornerPicker, IntervalSolver, ReferenceCornerPicker, ReferenceIntervalSolver};
pub use target::{GroupId, NodeKey, SchemeDim, Target, piece_of};
pub use vertex::{CornerAssignment, FaceLayout, LogicalSide, VertexRule, VertexType};
