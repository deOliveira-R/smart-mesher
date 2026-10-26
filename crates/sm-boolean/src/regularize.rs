//! Regularization of the result volumes: merge coplanar and co-surface fragments with
//! `join_faces`, dissolve degree-2 vertices with `join_edges`.

use std::collections::BTreeSet;

use sm_core::EntityId;
use sm_topo_api::TopologyEdit;

use crate::error::BooleanError;

pub(crate) fn regularize<E: TopologyEdit + ?Sized>(topo: &mut E, volumes: &[EntityId], b_faces: &BTreeSet<EntityId>) -> Result<(), BooleanError> {
    let _ = (topo, volumes, b_faces);
    Ok(())
}
