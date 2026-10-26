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
}

/// What [`dualize`](crate::dualize) did. All measures are taken from the finished mesh.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[non_exhaustive]
pub struct DualReport {
    /// Dual cells (one per vertex per region touching it).
    pub cells: usize,
    /// Primal edges of all regions, one dual face each (before hub splitting).
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
