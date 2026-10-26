//! Options and report of [`dualize`](crate::dualize).

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// How dual faces are emitted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
#[non_exhaustive]
pub enum FaceMode {
    /// Every emitted face is planar within the default `GeomTol`: a non-planar edge face is
    /// replaced by hub triangles and a non-planar boundary fan by one quad per triangle. The
    /// surface the two cells of a face share is the same on both sides, so volume is conserved
    /// exactly. This is the only mode the `mesh.to_poly` op uses.
    #[default]
    Planar,
    /// One polygon per primal edge and per boundary fan, planar or not (geometry follows the
    /// `smartmesh.fan_vertex_mean.v1` rule). For tests and cross-checks; not reachable from the
    /// op.
    Keep,
}

/// Options of [`dualize`](crate::dualize).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[non_exhaustive]
pub struct PolyOptions {
    /// Face emission mode (default [`FaceMode::Planar`]).
    #[serde(default)]
    pub faces: FaceMode,
    /// Whether the output carries an edge table; `None` = it does if the source has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub edges: Option<bool>,
    /// Merge small cells into a neighbour (default off; library only, not reachable from the op).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub agglomerate: Option<Agglomerate>,
}

impl PolyOptions {
    /// The defaults: [`FaceMode::Planar`], edge table iff the source has one.
    pub fn new() -> Self {
        Self::default()
    }
    /// Set the face mode.
    pub fn with_faces(mut self, faces: FaceMode) -> Self {
        self.faces = faces;
        self
    }
    /// Force the edge table on or off (`None` = follow the source).
    pub fn with_edges(mut self, edges: Option<bool>) -> Self {
        self.edges = edges;
        self
    }
    /// Turn agglomeration on (`Some`) or off (`None`, the default).
    pub fn with_agglomerate(mut self, agglomerate: Option<Agglomerate>) -> Self {
        self.agglomerate = agglomerate;
        self
    }
}

/// Greedy agglomeration of small cells (S2): cells whose volume is below `min_volume_ratio`
/// times the median volume are merged with the same-class neighbour that shares the most face
/// area, provided the shared faces form one edge-connected disc patch and the merged cell stays
/// near-convex (`max_{f,v} (x_v - apex_f) . n_f / V^(1/3) <= max_concavity`) and valid. Merging
/// only removes faces, so every face stays planar; classes never mix and interface faces
/// survive. Rounds repeat (each cell merges at most once per round) until nothing merges or
/// `max_rounds` is reached; `max_rounds = 0` is the identity.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
#[non_exhaustive]
pub struct Agglomerate {
    /// Cells below this fraction of the median cell volume are merge candidates.
    pub min_volume_ratio: f64,
    /// Largest accepted concavity of a merged cell (default 0.1).
    #[serde(default = "default_concavity")]
    pub max_concavity: f64,
    /// Largest number of rounds (default 4).
    #[serde(default = "default_rounds")]
    pub max_rounds: usize,
}

fn default_concavity() -> f64 {
    0.1
}

fn default_rounds() -> usize {
    4
}

impl Agglomerate {
    /// Agglomeration with the given volume ratio, concavity 0.1 and 4 rounds.
    pub fn new(min_volume_ratio: f64) -> Self {
        Self { min_volume_ratio, max_concavity: default_concavity(), max_rounds: default_rounds() }
    }
    /// Set the concavity limit.
    pub fn with_max_concavity(mut self, max_concavity: f64) -> Self {
        self.max_concavity = max_concavity;
        self
    }
    /// Set the round limit.
    pub fn with_max_rounds(mut self, max_rounds: usize) -> Self {
        self.max_rounds = max_rounds;
        self
    }
}

/// What [`dualize`](crate::dualize) did. All measures are taken from the finished mesh.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[non_exhaustive]
pub struct DualReport {
    /// Cells of the output: one per vertex per region touching it, minus the merges.
    pub cells: usize,
    /// Cell pairs merged by agglomeration (0 unless [`PolyOptions::agglomerate`] is set):
    /// `cells` equals the unmerged count minus `merges`.
    pub merges: usize,
    /// Primal edges of all regions, one dual face each (before hub splitting; this and the two
    /// face counts below are taken before agglomeration).
    pub edge_faces: usize,
    /// Boundary polygons emitted as faces (fans, or their per-triangle quads in `Planar`);
    /// an interface polygon shared by two cells counts once.
    pub boundary_faces: usize,
    /// Edge faces replaced by hub triangles (always 0 in `Keep`).
    pub hub_split_faces: usize,
    /// Hub triangles emitted for them.
    pub hub_triangles: usize,
    /// Boundary fans replaced by one quad per triangle (always 0 in `Keep`).
    pub split_fans: usize,
    /// Mesh vertices of the dual.
    pub points: usize,
    /// Largest planarity measure (`GeomTol::planarity` metric) over the emitted faces.
    pub max_nonplanarity: f64,
    /// Smallest cell volume.
    pub min_cell_volume: f64,
    /// Sum of cell volumes.
    pub total_volume: f64,
}
