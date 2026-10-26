//! Helpers shared by the queries and the op.

use sm_core::EntityId;
use sm_layer_api::MeshLayer;
use sm_op_api::OpError;

/// Wire version of every reply of this crate.
pub const SCHEMA_VERSION: u32 = 1;

pub(crate) fn schema_version() -> u32 {
    SCHEMA_VERSION
}

/// `Some(x)` for a finite `x`, `None` (JSON `null`) otherwise: no NaN or inf goes on the wire.
pub(crate) fn fin(x: f64) -> Option<f64> {
    x.is_finite().then_some(x)
}

/// The mesh a request names, or the only mesh of the layer when `mesh` is omitted.
///
/// A pure function of the state, so ops may use it without breaking replay. With no id and zero
/// or several meshes the error lists the candidates.
pub(crate) fn resolve_mesh(layer: &MeshLayer, mesh: Option<EntityId>) -> Result<EntityId, OpError> {
    match mesh {
        Some(id) => layer.get(id).map(|_| id).ok_or(OpError::UnknownEntity(id)),
        None => {
            let ids: Vec<EntityId> = layer.meshes().map(|(id, _)| id).collect();
            match ids.as_slice() {
                [only] => Ok(*only),
                [] => Err(OpError::Invalid("`mesh` is required and the state has no mesh".into())),
                many => {
                    let list: Vec<String> = many.iter().map(ToString::to_string).collect();
                    Err(OpError::Invalid(format!(
                        "`mesh` is required: the state has {} meshes: {}",
                        many.len(),
                        list.join(", ")
                    )))
                }
            }
        }
    }
}
