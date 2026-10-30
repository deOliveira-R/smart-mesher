//! Request, outcome, report and error types of the smoother.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_mesh_api::{CellIx, Classification, MeshError, MeshRead, VertexIx};
use sm_model_api::Model;
use sm_topo_api::TopologyView;

/// What a smoothing run reads: the mesh, the topology its classification refers to, and the
/// model tables (deck and interface boundaries).
#[derive(Clone, Copy)]
pub struct SmoothInput<'a> {
    /// The mesh to smooth.
    pub mesh: &'a dyn MeshRead,
    /// The topology that carries the curves and surfaces of the classification.
    pub topo: &'a dyn TopologyView,
    /// The model (boundary rows decide which faces are pinned).
    pub model: &'a Model,
}

/// Which nodes a run may move.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NodeSelect {
    /// The whole mesh. With `fix_boundary: true` only volume-classified nodes move.
    All,
    /// The nodes classified on these `TopoVolume` / `TopoFace` / `TopoEdge` entities. A listed
    /// face or edge selects its own nodes explicitly, whatever `fix_boundary` says.
    Entities(Vec<EntityId>),
    /// Explicit vertices (still subject to the constraint table and the pins).
    Vertices(Vec<VertexIx>),
}

/// Node-update rule.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
#[non_exhaustive]
pub enum Method {
    /// `x <- x + relax (mean(neighbours) - x)`, then projected onto the node's geometry.
    Laplacian {
        /// Relaxation factor in `(0, 2)`.
        #[serde(default = "one")]
        relax: f64,
    },
    /// Laplacian, accepted only if the minimum incident `Shape` does not decrease.
    SmartLaplacian {
        /// Relaxation factor in `(0, 2)`.
        #[serde(default = "one")]
        relax: f64,
    },
    /// `x <- sum(A_c centroid_c) / sum(A_c)` over incident cells (faces or edges for boundary nodes).
    CentroidArea {
        /// Relaxation factor in `(0, 2)`.
        #[serde(default = "one")]
        relax: f64,
    },
    /// Winslow smoothing of a structured-interior all-quad surface mesh: a node with exactly four
    /// incident quads moves to the discrete Winslow (elliptic grid generation) update; every
    /// other node is pinned (`winslow_unstructured`). Affine invariant.
    Winslow {
        /// Relaxation factor in `(0, 2)`.
        #[serde(default = "one")]
        relax: f64,
    },
    /// Node-local minimisation of `sum_c beta_c^p` by finite-difference steepest descent
    /// with Armijo backtracking.
    ///
    /// A node whose incident cell is degenerate or inverted has an infinite objective, so
    /// `Optimize` never moves it (untangling is deferred to W3b); the guard-checked
    /// Laplacian-type methods may still move such nodes.
    Optimize {
        /// The cell quality objective.
        #[serde(default)]
        objective: Objective,
        /// Exponent in `[1, 32]`.
        #[serde(default = "four")]
        p: f64,
    },
}

fn one() -> f64 {
    1.0
}
fn four() -> f64 {
    4.0
}

impl Default for Method {
    fn default() -> Self {
        Method::Laplacian { relax: 1.0 }
    }
}

/// Cell quality an [`Method::Optimize`] run minimises.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
#[non_exhaustive]
pub enum Objective {
    /// Knupp mean ratio, `beta = 1 / Metric::Shape`.
    #[default]
    MeanRatio,
    /// Condition number, `beta = Metric::Condition` (1 is ideal).
    Condition,
}

/// Sweep order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Schedule {
    /// Candidates from the old positions, applied together, then guard repair.
    Jacobi,
    /// Greedy node colouring; colours in order, each colour computed in parallel.
    #[default]
    Colored,
}

/// A smoothing request.
#[derive(Clone, Debug, PartialEq)]
pub struct SmoothRequest {
    /// Nodes that may move.
    pub nodes: NodeSelect,
    /// Update rule.
    pub method: Method,
    /// Sweep limit (at least 1).
    pub iterations: u32,
    /// Stop when `max_v |dx_v| / l_v < tolerance`.
    pub tolerance: f64,
    /// Sweep order.
    pub schedule: Schedule,
    /// With `false`, face and edge nodes of the selected volumes move too, when every meshed
    /// volume above them is selected.
    pub fix_boundary: bool,
    /// Guard floor on `ScaledJacobian` in `[0, 1)`.
    pub guard_floor: f64,
}

impl Default for SmoothRequest {
    fn default() -> Self {
        Self {
            nodes: NodeSelect::All,
            method: Method::default(),
            iterations: 10,
            tolerance: 1e-6,
            schedule: Schedule::Colored,
            fix_boundary: true,
            guard_floor: 1e-3,
        }
    }
}

/// Aggregate quality of a mesh.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct QualityStats {
    /// Smallest `Shape` over the cells that define it (0 if none does).
    pub min_shape: f64,
    /// Mean `Shape` over the same cells (0 if none).
    pub mean_shape: f64,
    /// Largest `Condition` (0 if undefined everywhere).
    pub max_condition: f64,
    /// Inverted cells (`sm_quality::validity`).
    pub n_inverted: usize,
    /// Degenerate cells.
    pub n_degenerate: usize,
}

/// What a run did.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct SmoothReport {
    /// Sweeps executed.
    pub iterations_run: u32,
    /// The movement criterion was met before the sweep limit.
    pub converged: bool,
    /// `max_v |dx_v| / l_v` of the last sweep.
    pub max_move_last: f64,
    /// Nodes that were free to move.
    pub movable: usize,
    /// Selected nodes that were frozen, by reason (`vertex`, `deck`, `interface`,
    /// `polyhedral`, `unprojectable`, `unsupported_shape`, `winslow_unstructured`, `isolated`).
    pub pinned: BTreeMap<String, usize>,
    /// Entities behind the pins: faces for `deck` and `interface`, the classifying entities
    /// for the other reasons (at most 20 each).
    pub pinned_entities: BTreeMap<String, Vec<EntityId>>,
    /// Node updates applied (position changed), summed over sweeps.
    pub accepted: u64,
    /// Updates refused by the guard (including Jacobi repair reverts).
    pub rejected_guard: u64,
    /// Updates refused because the projection failed or left the curve range or trimmed face.
    pub rejected_geometry: u64,
    /// Quality before.
    pub before: QualityStats,
    /// Quality after.
    pub after: QualityStats,
    /// The (at most 10) cells with the smallest `Shape` after, worst first.
    pub worst_after: Vec<CellIx>,
    /// Up to 10 inverted cells of the input.
    pub inverted_before: Vec<CellIx>,
}

/// The result of [`crate::smooth`]: new positions and the classification parameters to store.
#[derive(Clone, Debug, PartialEq)]
pub struct SmoothOutcome {
    /// New position of every vertex (unmoved vertices keep their input bits).
    pub points: Vec<[f64; 3]>,
    /// Refreshed classification (`Param::Curve` / `Param::Surface`) of the moved edge and face nodes.
    pub classes: Vec<(VertexIx, Classification)>,
    /// The report.
    pub report: SmoothReport,
}

/// Why a smoothing run failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
#[non_exhaustive]
pub enum SmoothError {
    /// The mesh or request is outside what the smoother handles.
    #[error("unsupported: {0}")]
    Unsupported(&'static str),
    /// A parameter is out of range.
    #[error("bad parameters: {0}")]
    BadParams(String),
    /// A listed entity does not exist or has no nodes in the mesh.
    #[error("unknown entity {0:?}")]
    UnknownEntity(EntityId),
    /// A vertex of the input has a NaN or infinite coordinate.
    #[error("vertex {0} has a non-finite coordinate")]
    NonFiniteInput(usize),
    /// The selection leaves no movable node.
    #[error("nothing to smooth: no selected node is free to move")]
    NothingToSmooth,
    /// A mesh edit failed.
    #[error(transparent)]
    Mesh(#[from] MeshError),
}
