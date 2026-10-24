//! The `mesh.info` query.

use std::collections::BTreeMap;

use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::MeshLayer;
use sm_mesh_api::{CellIx, MeshEntityKind, MeshRead, cell_volume, geometry_hash_quantized, topology_hash};
use sm_op_api::{OpError, QueryCtx, State, TypedQuery};

/// `mesh.info`: counts, hashes and total measure of one mesh.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshInfoQuery {
    /// The mesh.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
}

/// The reply of [`MeshInfoQuery`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct MeshInfo {
    /// The mesh.
    pub mesh: EntityId,
    /// The model entities it was generated for.
    pub selection: Vec<EntityId>,
    /// The selection changed or was deleted after meshing.
    pub stale: bool,
    /// Number of vertices.
    pub vertices: usize,
    /// Number of mesh edges (0 unless the edge table exists).
    pub edges: usize,
    /// Number of faces.
    pub faces: usize,
    /// Number of cells.
    pub cells: usize,
    /// Cells by shape (`"Hex"`, `"Tet"`, ...).
    pub cell_shapes: BTreeMap<String, usize>,
    /// Exact integer hash of the connectivity, 16 lower-case hex digits.
    pub topology_hash: String,
    /// Quantized geometry hash (the entry's quanta), 16 hex digits.
    pub geometry_hash: String,
    /// Total volume of the cells (area for a surface mesh), summed in a fixed order.
    pub volume: f64,
    /// Candidates the final plan passed over or that failed and caused a re-plan.
    pub attempts: usize,
}

/// Cells per summation chunk: the chunking is fixed, so the sum does not depend on thread count.
const CHUNK: usize = 4096;

/// Sum of cell volumes: fixed-size chunks summed in parallel, then added in chunk order.
fn total_volume(m: &dyn MeshRead) -> f64 {
    let n = m.count(MeshEntityKind::Cell);
    let parts: Vec<f64> = (0..n.div_ceil(CHUNK))
        .into_par_iter()
        .map(|c| {
            (c * CHUNK..((c + 1) * CHUNK).min(n))
                .map(|i| cell_volume(m, CellIx(i as u32)))
                .sum::<f64>()
        })
        .collect();
    parts.into_iter().sum()
}

impl TypedQuery for MeshInfoQuery {
    const NAME: &'static str = "mesh.info";
    type Output = MeshInfo;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<MeshInfo, OpError> {
        let layer = state
            .layer::<MeshLayer>()
            .ok_or_else(|| OpError::MissingLayer("meshes".into()))?;
        let e = layer
            .get(self.mesh)
            .ok_or(OpError::UnknownEntity(self.mesh))?;
        let m = e.mesh();
        let mut shapes: BTreeMap<String, usize> = BTreeMap::new();
        for i in 0..m.count(MeshEntityKind::Cell) {
            *shapes
                .entry(format!("{:?}", m.cell_shape(CellIx(i as u32))))
                .or_default() += 1;
        }
        Ok(MeshInfo {
            mesh: self.mesh,
            selection: e.selection.clone(),
            stale: e.stale,
            vertices: m.count(MeshEntityKind::Vertex),
            edges: m.count(MeshEntityKind::Edge),
            faces: m.count(MeshEntityKind::Face),
            cells: m.count(MeshEntityKind::Cell),
            cell_shapes: shapes,
            topology_hash: format!("{:016x}", topology_hash(m)),
            geometry_hash: format!("{:016x}", geometry_hash_quantized(m, &e.quanta)),
            volume: total_volume(m),
            attempts: e.plan.as_ref().map_or(0, |p| p.attempts.len()),
        })
    }
}
