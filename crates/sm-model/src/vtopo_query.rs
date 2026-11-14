//! `model.virtual_info`: read the layer's virtual-topology record.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::ModelLayer;
use sm_op_api::{OpError, QueryCtx, State, TypedQuery};
use sm_topo_api::{VirtualOp, VirtualOverlay};

/// `model.virtual_info {id?}`: the overlay's recorded virtual operations, in order, with what
/// each created and used. With `id`, only the operations that created or use that cell, and
/// `members` (the host cells a live virtual `id` stands for). A model with no overlay reports
/// `has_overlay: false` and no operations.
#[derive(Debug, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct VirtualInfo {
    /// Restrict to the operations that created or use this cell.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVertex", "TopoEdge", "TopoFace", "TopoVolume", "PartitionVertex", "PartitionEdge", "PartitionFace"]))]
    pub id: Option<EntityId>,
}

/// One recorded virtual operation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct VirtualOpInfo {
    /// Position in the record (0-based).
    pub index: usize,
    /// `composite_faces`, `composite_edges`, `partition_edge`, `partition_face`,
    /// `partition_volume` or `collapse_edge`.
    pub op: String,
    /// Virtual ids the operation minted.
    pub created: Vec<EntityId>,
    /// Cells it consumed (hidden from the view, or split).
    pub inputs: Vec<EntityId>,
    /// Collapse gap (`collapse_edge` only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gap: Option<f64>,
    /// Curve parameter (`partition_edge` only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub param: Option<f64>,
}

/// Result of `model.virtual_info`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct VirtualInfoOut {
    /// Whether the layer holds an overlay.
    pub has_overlay: bool,
    /// The overlay was built on an older host revision than the layer's (it would be rebased
    /// by the next host edit).
    pub stale: bool,
    /// The recorded operations (filtered by `id`).
    pub ops: Vec<VirtualOpInfo>,
    /// Host cells a live virtual `id` stands for (empty without `id`, or for a real cell).
    pub members: Vec<EntityId>,
    /// Next id serial per rank: vertex, edge, face (ids already minted are never reused).
    pub next_serial: [u64; 3],
}

fn describe(index: usize, op: &VirtualOp) -> VirtualOpInfo {
    let (name, inputs, gap, param): (&str, Vec<EntityId>, Option<f64>, Option<f64>) = match op {
        VirtualOp::Composite { faces, .. } => ("composite_faces", faces.clone(), None, None),
        VirtualOp::CompositeEdges { edges, .. } => ("composite_edges", edges.clone(), None, None),
        VirtualOp::PartitionEdge { edge, param, .. } => ("partition_edge", vec![*edge], None, Some(*param)),
        VirtualOp::PartitionFace { face, cut, .. } => ("partition_face", vec![*face, cut.from, cut.to], None, None),
        VirtualOp::PartitionVolume { volume, .. } => ("partition_volume", vec![*volume], None, None),
        VirtualOp::CollapseEdge { edge, into_vertex, gap, repointed } => {
            let mut v = vec![*edge, *into_vertex];
            v.extend(repointed.iter().map(|&(o, _)| o));
            ("collapse_edge", v, Some(*gap), None)
        }
        _ => ("unknown", Vec::new(), None, None),
    };
    VirtualOpInfo { index, op: name.to_owned(), created: op.created().to_vec(), inputs, gap, param }
}

impl TypedQuery for VirtualInfo {
    const NAME: &'static str = "model.virtual_info";
    type Output = VirtualInfoOut;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<VirtualInfoOut, OpError> {
        let layer = state.layer::<ModelLayer>().ok_or_else(|| OpError::MissingLayer(<ModelLayer as sm_op_api::LayerType>::NAME.to_owned()))?;
        let Some(ov) = layer.overlay() else {
            return Ok(VirtualInfoOut { has_overlay: false, stale: false, ops: Vec::new(), members: Vec::new(), next_serial: [1; 3] });
        };
        let ops = ov
            .ops()
            .iter()
            .enumerate()
            .map(|(i, op)| describe(i, op))
            .filter(|o| self.id.is_none_or(|id| o.created.contains(&id) || o.inputs.contains(&id)))
            .collect();
        let members = self.id.map(|id| ov.resolve(id).to_vec()).unwrap_or_default();
        Ok(VirtualInfoOut { has_overlay: true, stale: ov.is_stale_against(layer.host()), ops, members, next_serial: ov.data().next_serial })
    }
}
