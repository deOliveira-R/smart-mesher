//! `mesh.unflag_cells`: remove a cell set made by `mesh.flag_cells`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::{LayerError, MeshLayer};
use sm_mesh_api::SetMembers;
use sm_op_api::{ApplyCtx, OpError, TypedOp};
use std::sync::Arc;

use crate::common::resolve_mesh;

/// `mesh.unflag_cells`: remove the cell set `set` from a mesh.
///
/// The inverse of `mesh.flag_cells`: the mesh content (topology and geometry) and its set list
/// afterwards equal those before the set was added. Allocator state (next serial, retired set
/// id) does not roll back, so the state hash is not restored. Only cell sets can be removed. An
/// unknown name is an error that lists the sets of the mesh.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshUnflagCells {
    /// The mesh; defaults to the only mesh of the state.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: Option<EntityId>,
    /// Name of the cell set to remove.
    pub set: String,
}

impl TypedOp for MeshUnflagCells {
    const NAME: &'static str = "mesh.unflag_cells";

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let (id, mut host, alloc, set_id) = {
            let layer = ctx.layer::<MeshLayer>()?;
            let id = resolve_mesh(layer, self.mesh)?;
            let entry = layer.get(id).ok_or(OpError::UnknownEntity(id))?;
            let m = entry.mesh();
            let Some(set) = m.set_by_name(&self.set) else {
                let names: Vec<&str> = m.sets().iter().map(|s| s.name.as_str()).collect();
                return Err(OpError::Invalid(format!("{id} has no set named `{}`; its sets: [{}]", self.set, names.join(", "))));
            };
            if !matches!(set.members, SetMembers::Cells(_)) {
                return Err(OpError::Invalid(format!("set `{}` of {id} is not a cell set", self.set)));
            }
            (id, entry.mesh.clone_host(), layer.alloc().clone(), set.id)
        };
        host.remove_set(set_id).map_err(LayerError::from)?;
        if !ctx.state().contains(set_id) {
            return Err(OpError::UnknownEntity(set_id));
        }
        let host: Arc<dyn sm_layer_api::MeshHost> = Arc::from(host);
        ctx.layer::<MeshLayer>()?.check_replace(id, &*host, &alloc)?;
        ctx.retire(set_id)?;
        ctx.layer_mut::<MeshLayer>()?.replace_mesh(id, host, alloc)?;
        Ok(())
    }
}
