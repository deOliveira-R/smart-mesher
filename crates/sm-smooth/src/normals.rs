//! The carrier-oriented face normal of 2-D cells.

use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_mesh_api::FaceNormals;
use sm_topo_api::{Carrier, Sense, TopologyView};

/// Surface normal of a `TopoFace` / `PartitionFace` at the point nearest `near`, times the
/// carrier sense (the face normal of `sm-topo-api`). `None` without a surface carrier.
pub(crate) struct CarrierNormals<'a> {
    pub topo: &'a dyn TopologyView,
}

impl FaceNormals for CarrierNormals<'_> {
    fn normal(&self, entity: EntityId, near: Point3) -> Option<Vector3> {
        if !matches!(entity.kind(), EntityKind::TopoFace | EntityKind::PartitionFace) {
            return None;
        }
        let Carrier::Surface(sc) = self.topo.carrier(entity) else { return None };
        let s = self.topo.shapes().surface(sc.surface)?;
        let uv = s.project(near, None).ok()?.uv;
        let n = s.normal(uv).ok()?;
        Some(match sc.sense {
            Sense::Forward => n,
            Sense::Reversed => Vector3::new(-n.x, -n.y, -n.z),
        })
    }
}
