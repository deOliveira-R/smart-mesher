//! `model.delete`: remove bodies and everything only they use.

use std::collections::BTreeSet;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{LayerResultExt, commit_topology, topo_txn};
use sm_op_api::{ApplyCtx, OpError, TypedOp};

/// `model.delete`: delete bodies, their volumes, and every face, edge and vertex that no
/// surviving cell uses (`kill_cascade`). All retired ids are retired in the entity table, the
/// model tables drop the deleted volumes and faces, and `meshes` entries on a deleted volume
/// become `stale` (they are not deleted: `mesh.delete` retires meshes).
#[derive(Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Delete {
    /// The bodies to delete.
    #[schemars(extend("x-entity-kind" = ["TopoBody"]))]
    pub bodies: Vec<EntityId>,
}

impl TypedOp for Delete {
    const NAME: &'static str = "model.delete";
    const NO_OUTPUTS: bool = true;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        if self.bodies.is_empty() {
            return Err(OpError::Invalid("`bodies` is empty".into()));
        }
        let mut bodies: BTreeSet<EntityId> = BTreeSet::new();
        for &b in &self.bodies {
            ctx.resolve_kind(b, EntityKind::TopoBody)?;
            bodies.insert(b);
        }
        let ((), txn) = topo_txn(ctx, |topo, _model| {
            for &b in &bodies {
                topo.kill_cascade(b).or_op()?;
            }
            Ok(())
        })?;
        commit_topology(ctx, &txn, &[])
    }
}
