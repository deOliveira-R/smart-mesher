//! `mesh.volume_check`: per-region mesh volume against the model volume.

use std::collections::BTreeMap;

use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh_api::{CellIx, MeshEntityKind, MeshRead, cell_volume_centroid};
use sm_op_api::{OpError, QueryCtx, State, TypedQuery};

use crate::common::{fin, resolve_mesh, schema_version};

/// Cells per parallel chunk; the per-cell values are collected in cell order and summed
/// sequentially, so the result does not depend on the thread count.
const CHUNK: usize = 4096;

/// Volume of one region (all active cells classified on one entity).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct RegionVolume {
    /// The classification entity of the cells (a `TopoVolume` for a model region).
    pub region: EntityId,
    /// Active cells classified on it.
    pub cells: usize,
    /// Sum of the cell volumes (compensated sum in cell order).
    pub mesh_volume: Option<f64>,
    /// The model volume (`TopologyView::volume_moments`); `null` if it cannot be computed.
    pub model_volume: Option<f64>,
    /// Why `model_volume` is `null`, when it is.
    pub model_note: Option<String>,
    /// `model_volume - mesh_volume`: positive when the mesh is smaller than the model (chord
    /// error of a round region). `null` when the model volume is unknown.
    pub deficit: Option<f64>,
    /// `deficit / model_volume`; `null` when undefined.
    pub relative_deficit: Option<f64>,
}

/// Reply of `mesh.volume_check`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct VolumeReport {
    /// Wire version, 1.
    pub schema_version: u32,
    /// The mesh.
    pub mesh: EntityId,
    /// One row per classification entity of the active cells, ordered by entity id.
    pub regions: Vec<RegionVolume>,
    /// Number of active cells counted.
    pub cells: usize,
    /// Total mesh volume (sum of the region mesh volumes in region order).
    pub total_mesh_volume: Option<f64>,
    /// Total model volume over the regions whose model volume is known.
    pub total_model_volume: Option<f64>,
    /// `total_model_volume - total_mesh_volume` over the regions whose model volume is known.
    pub total_deficit: Option<f64>,
}

/// Neumaier compensated sum, in the order given.
fn neumaier<I: IntoIterator<Item = f64>>(it: I) -> f64 {
    let (mut s, mut c) = (0.0f64, 0.0f64);
    for x in it {
        let t = s + x;
        c += if s.abs() >= x.abs() { (s - t) + x } else { (x - t) + s };
        s = t;
    }
    s + c
}

/// Per-region mesh volume against `model_volume(region)`. Pure and deterministic: bit-identical
/// for any thread count. Inactive cells (refined parents) are skipped.
pub fn volume_report(m: &dyn MeshRead, model_volume: &dyn Fn(EntityId) -> Result<f64, String>) -> VolumeReport {
    let n = m.count(MeshEntityKind::Cell);
    let vols: Vec<f64> = (0..n)
        .into_par_iter()
        .with_min_len(CHUNK)
        .map(|c| {
            let c = CellIx::new(c);
            if m.cell_active(c) { cell_volume_centroid(m, c).0 } else { 0.0 }
        })
        .collect();
    let mut by: BTreeMap<EntityId, Vec<f64>> = BTreeMap::new();
    let mut counted = 0usize;
    for (c, &v) in vols.iter().enumerate() {
        if !m.cell_active(CellIx::new(c)) {
            continue;
        }
        counted += 1;
        by.entry(m.classification(MeshEntityKind::Cell, c as u32).entity).or_default().push(v);
    }
    let mut regions = Vec::with_capacity(by.len());
    let (mut tm, mut tmod, mut tdef, mut any_model) = (Vec::new(), Vec::new(), Vec::new(), false);
    for (region, vs) in by {
        let mesh_v = neumaier(vs.iter().copied());
        let (model_v, note) = if region.kind() == EntityKind::TopoVolume {
            match model_volume(region) {
                Ok(v) => (Some(v), None),
                Err(e) => (None, Some(e)),
            }
        } else {
            (None, Some(format!("{region} is not a TopoVolume")))
        };
        let deficit = model_v.map(|v| v - mesh_v);
        if let Some(v) = model_v {
            any_model = true;
            tmod.push(v);
            tdef.push(v - mesh_v);
        }
        tm.push(mesh_v);
        regions.push(RegionVolume {
            region,
            cells: vs.len(),
            mesh_volume: fin(mesh_v),
            model_volume: model_v.and_then(fin),
            model_note: note,
            deficit: deficit.and_then(fin),
            relative_deficit: model_v.filter(|v| *v != 0.0).and_then(|v| fin((v - mesh_v) / v)),
        });
    }
    VolumeReport {
        schema_version: schema_version(),
        mesh: m.mesh_id(),
        regions,
        cells: counted,
        total_mesh_volume: fin(neumaier(tm)),
        total_model_volume: any_model.then(|| neumaier(tmod)).and_then(fin),
        total_deficit: any_model.then(|| neumaier(tdef)).and_then(fin),
    }
}

/// `mesh.volume_check`: the volume of every region of a mesh against the model volume it is
/// classified on.
///
/// Works for tet, hex, polyhedral and mixed meshes (cell volume is the fan polyhedron of its
/// faces). The deficit is `model - mesh`: for a round region meshed with `n` equal intervals it
/// is the chord error, `pi r^2 H - 0.5 n r^2 sin(2 pi / n) H` for a cylinder. Read-only.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshVolumeCheck {
    /// The mesh; defaults to the only mesh of the state.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: Option<EntityId>,
}

impl TypedQuery for MeshVolumeCheck {
    const NAME: &'static str = "mesh.volume_check";
    type Output = VolumeReport;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<VolumeReport, OpError> {
        let layer = state.layer::<MeshLayer>().ok_or_else(|| OpError::MissingLayer("meshes".into()))?;
        let model = state.layer::<ModelLayer>().ok_or_else(|| OpError::MissingLayer("model".into()))?;
        let id = resolve_mesh(layer, self.mesh)?;
        let entry = layer.get(id).ok_or(OpError::UnknownEntity(id))?;
        let topo = model.topo();
        let mv = |v: EntityId| topo.volume_moments(v).map(|m| m.volume).map_err(|e| e.to_string());
        Ok(volume_report(entry.mesh(), &mv))
    }
}
