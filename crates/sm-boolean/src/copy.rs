//! Deep copies of volumes for `keep = true`.

use sm_core::EntityId;
use sm_topo_api::TopologyEdit;

use crate::error::BooleanError;

/// Copy `volume` with all its cells (vertices, edges, faces, shells); the copies share the curve
/// and surface handles with the originals.
pub(crate) fn copy_volume<E: TopologyEdit + ?Sized>(topo: &mut E, volume: EntityId) -> Result<EntityId, BooleanError> {
    let _ = (topo, volume);
    Err(BooleanError::Unsupported("keep is not implemented yet"))
}
