//! `sm-scheme-api`: the contract between the meshing driver and every meshing scheme.
//!
//! Approach: a **plan/apply pipeline with capabilities**.
//!
//! - Every scheme is a trait object with explicit phases ([`SchemeCore`]):
//!   `applicability -> Applicability { score, reason }`, `plan -> Plan` (requirements on
//!   children, claims, ordering, interval constraints, vertex types), and a dimension-specific
//!   `apply` ([`CurveScheme`], [`SurfaceScheme`], [`VolumeScheme`]) that turns the committed
//!   boundary mesh into a local patch. [`MeshTransform`] converts sets of meshed volumes
//!   atomically.
//! - Requirements name **capabilities** ([`Caps`]) rather than scheme names where possible, so a
//!   deck copy of a mapped face satisfies a sweep that wants structured linking faces.
//! - The [`SchemeRegistry`] maps names to code and holds fallback chains and auto lists.
//! - The reference [`Driver`] plans top-down (3, 2, 1), solves all interval constraints once
//!   ([`IntervalSolver`]), applies bottom-up (1, 2, 3) with independent jobs on an
//!   [`Executor`] and a fixed commit order, re-plans on fallback, and [`emit`]s into any
//!   `sm_mesh_api::MeshBuild` with classification.
//! - Intent ([`MeshIntent`], [`SchemeSpec`], [`Params`]) and plans are serde data, so they can
//!   live in an op and in git.
//! - Shape access goes through [`ShapeView`] (implemented by [`TopoShape`] over `sm-topo-api` +
//!   `sm-geom-api` + `sm-model-api`), keyed by [`ShapeKey`], which also names decomposition
//!   pieces.
//!
//! See `DESIGN.md` for positioning and the brief-to-type map.

pub mod builtin;
pub mod caps;
pub mod driver;
pub mod error;
pub mod intent;
pub mod interval;
pub mod key;
pub mod mesh;
pub mod patch;
pub mod plan;
pub mod registry;
pub mod scheme;
pub mod shape;
pub mod size;
pub mod vertex;

pub use builtin::{CURVE_COPY, CurveCopy, SURFACE_COPY, SurfaceCopy};
pub use caps::{Caps, Role};
pub use driver::{Driver, DriverOptions, EmitMap, Executor, Job, MeshOutcome, MeshPlan, Sequential, Threads, emit, grid_of};
pub use error::{Attempt, DriveError, FailureKind, Phase, SchemeError};
pub use intent::{EntityIntent, Firmness, GroupIntent, MeshIntent, ParamValue, Params, SchemeChoice, SchemeId, SchemeSpec, SizeSpec};
pub use interval::{IntervalConstraint, IntervalError, IntervalProblem, IntervalSolution, IntervalSolver, Relation, SimpleSolver, SourcedConstraint, Term};
pub use key::ShapeKey;
pub use mesh::{CommittedCells, CommittedCurve, CommittedMesh, CommittedSurface, NodeOrigin};
pub use patch::{CellList, ClaimedSurface, CurvePatch, FaceList, Grid, NewNode, NodeId, PatchRef, SurfacePatch, TransformPatch, VolumePatch};
pub use plan::{Applicability, ChildRequirement, Plan, Requirement};
pub use registry::{Candidate, Origin, SchemeImpl, SchemeRegistry};
pub use scheme::{BoundaryLoop, BoundaryUse, CurveInput, CurveScheme, MeshTransform, PlanCtx, SchemeCore, SurfaceInput, SurfaceScheme, VolumeInput, VolumeScheme};
pub use shape::{CurveUse, DeckLink, FaceUseRef, LoopUse, ShapeView, SweepHint, TopoShape};
pub use size::{ConstantSize, SizeField, SymMat3};
pub use vertex::{AnglePicker, CornerAssignment, CornerPicker, VertexType, interior_angle};
