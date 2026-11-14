//! The `mesh.decompose_plan` query: the cuts that make a volume sweepable, from `sm-decomp`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_decomp::{CutStep, DecompError, plan_cuts, render_script};
use sm_layer_api::ModelLayer;
use sm_op_api::{OpError, QueryCtx, State, TypedQuery};
use sm_topo_api::{Dim, TopologyView};

/// `mesh.decompose_plan`: the webcut steps that make round-cavity volumes sweepable. Read-only; nothing is
/// cut and no id is minted. Apply the steps in order with `model.webcut_cylinder` / `model.webcut_plane`,
/// then `model.imprint` and `model.merge`.
///
/// A step target is `{id}` (the planned volume) or `{step, role}` (the `positive` or `negative` output of
/// an earlier step).
#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshDecomposePlanQuery {
    /// Volumes to plan (default: every live volume, ascending).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub volumes: Option<Vec<EntityId>>,
}

/// What the planner found for one volume.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum DecomposeStatus {
    /// Nothing to cut (no cavity wall, or it already runs through the volume).
    NoCuts,
    /// `steps` lists the cuts.
    Cuts,
    /// The geometry is outside what the planner handles; see `reason`. Nothing is suggested.
    NoDecomposition,
}

/// The plan for one volume.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct VolumeCuts {
    /// The volume.
    pub volume: EntityId,
    /// Outcome.
    pub status: DecomposeStatus,
    /// The ordered cuts (empty unless `status` is `cuts`).
    pub steps: Vec<CutStep>,
    /// The same steps as command-language lines (empty unless `status` is `cuts`).
    pub script: String,
    /// Why no decomposition was found (`no_decomposition` only).
    pub reason: Option<String>,
}

/// Reply of `mesh.decompose_plan`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct DecomposePlanReport {
    /// One row per requested volume, ascending by id.
    pub volumes: Vec<VolumeCuts>,
}

/// Plan one volume of `topo`.
pub(crate) fn volume_cuts(
    topo: &dyn TopologyView,
    volume: EntityId,
) -> Result<VolumeCuts, OpError> {
    let row = |status, steps: Vec<CutStep>, reason| VolumeCuts {
        volume,
        status,
        script: render_script(&steps, &volume.to_string(), "cut"),
        steps,
        reason,
    };
    match plan_cuts(topo, volume) {
        Ok(steps) if steps.is_empty() => Ok(row(DecomposeStatus::NoCuts, steps, None)),
        Ok(steps) => Ok(row(DecomposeStatus::Cuts, steps, None)),
        Err(DecompError::NoDecomposition { reason, .. }) => Ok(row(
            DecomposeStatus::NoDecomposition,
            Vec::new(),
            Some(reason),
        )),
        Err(DecompError::NotAVolume(v)) => {
            Err(OpError::Invalid(format!("{v} is not a live volume")))
        }
    }
}

impl TypedQuery for MeshDecomposePlanQuery {
    const NAME: &'static str = "mesh.decompose_plan";
    type Output = DecomposePlanReport;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<DecomposePlanReport, OpError> {
        let model = state
            .layer::<ModelLayer>()
            .ok_or_else(|| OpError::MissingLayer("model".into()))?;
        let topo = model.topo();
        let mut ids: Vec<EntityId> = match &self.volumes {
            Some(v) if v.is_empty() => return Err(OpError::Invalid("`volumes` is empty".into())),
            Some(v) => v.clone(),
            None => topo.cells(Dim::Volume).to_vec(),
        };
        ids.sort_unstable();
        ids.dedup();
        let volumes = ids
            .into_iter()
            .map(|v| volume_cuts(topo, v))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(DecomposePlanReport { volumes })
    }
}
