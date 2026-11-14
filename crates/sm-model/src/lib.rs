//! Primitives, transforms, copy and topology queries (W2 task T1, W3b-PRIM).
//!
//! [`ModelPlugin`] registers ops and queries only; the `model` layer itself comes from
//! `sm_layer_api::CoreLayers`. See `DESIGN.md`.

mod copy;
mod delete;
mod prim;
mod prim2;
mod queries;
mod region;
mod sphere_part;
mod vtopo;
mod vtopo_query;
mod xform;
mod xform2;

pub use copy::{CopyXform, ModelCopy};
pub use delete::Delete;
pub use prim::{CreateBrick, CreateCylinder, CreateSphere};
pub use prim2::{CreateFrustum, CreatePrism, CreatePyramid, CreateTorus};
pub use queries::{BBox, Measurement, ModelList, ModelMeasure, TopoRelated};
pub use region::{FrameRow, HexOrientSpec, LatticeDefine, LatticeTag, LayoutSpec, Outside, RegionClear, RegionTable, RegionTags, TagRow};
pub use sphere_part::{CreateSpherePart, HalfSpace, Octants};
pub use vtopo::{CollapseEdge, CompositeEdges, CompositeFaces, PartitionEdge, PartitionFace, RemoveVirtual, SplitRealized, UnsplitRealized};
pub use vtopo_query::{VirtualInfo, VirtualInfoOut, VirtualOpInfo};
pub use xform::{Move, Reflect, Rotate, Scale, XformError};
pub use xform2::{Anchor, AxisName, AxisSpec, MoveTarget, MoveTo, PointSpec, RotateAbout};

use sm_op_api::{Plugin, Registry, RegistryError};

/// The model plugin (ops and queries of the `model` and `topo` namespaces).
#[derive(Clone, Copy, Debug, Default)]
pub struct ModelPlugin;

impl Plugin for ModelPlugin {
    fn id(&self) -> &str {
        "model"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register::<CreateBrick>()?;
        registry.register::<CreateCylinder>()?;
        registry.register::<CreateSphere>()?;
        registry.register::<Move>()?;
        registry.register::<Rotate>()?;
        registry.register::<Scale>()?;
        registry.register::<Reflect>()?;
        registry.register::<CreateFrustum>()?;
        registry.register::<CreatePrism>()?;
        registry.register::<CreatePyramid>()?;
        registry.register::<CreateTorus>()?;
        registry.register::<CreateSpherePart>()?;
        registry.register::<MoveTo>()?;
        registry.register::<RotateAbout>()?;
        registry.register::<ModelCopy>()?;
        registry.register::<Delete>()?;
        registry.register::<LatticeDefine>()?;
        registry.register::<LatticeTag>()?;
        registry.register::<RegionClear>()?;
        registry.register::<CompositeFaces>()?;
        registry.register::<CompositeEdges>()?;
        registry.register::<PartitionEdge>()?;
        registry.register::<PartitionFace>()?;
        registry.register::<CollapseEdge>()?;
        registry.register::<RemoveVirtual>()?;
        registry.register::<SplitRealized>()?;
        registry.register::<UnsplitRealized>()?;
        registry.register_query::<VirtualInfo>()?;
        registry.register_query::<TopoRelated>()?;
        registry.register_query::<ModelList>()?;
        registry.register_query::<ModelMeasure>()?;
        registry.register_query::<RegionTags>()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plugin_installs_and_registers_the_model_surface() {
        let mut reg = Registry::new();
        reg.install(&ModelPlugin).expect("install");
        assert_eq!(ModelPlugin.id(), "model");
        let mut ops: Vec<&str> = reg.op_names().into_iter().filter(|n| !n.starts_with("core.")).collect();
        ops.sort_unstable();
        assert_eq!(
            ops,
            [
                "model.collapse_edge",
                "model.composite_edges",
                "model.composite_faces",
                "model.copy",
                "model.create_brick",
                "model.create_cylinder",
                "model.create_frustum",
                "model.create_prism",
                "model.create_pyramid",
                "model.create_sphere",
                "model.create_sphere_part",
                "model.create_torus",
                "model.delete",
                "model.lattice_define",
                "model.lattice_tag",
                "model.move",
                "model.move_to",
                "model.partition_edge",
                "model.partition_face",
                "model.reflect",
                "model.region_clear",
                "model.remove_virtual",
                "model.rotate",
                "model.rotate_about",
                "model.scale",
                "model.split_realized",
                "model.unsplit_realized"
            ]
        );
        let mut qs = reg.query_names();
        qs.sort_unstable();
        assert_eq!(qs, ["model.list", "model.measure", "model.region_tags", "model.virtual_info", "topo.related"]);
    }

    #[test]
    fn the_plugin_registers_no_layers() {
        let mut reg = Registry::new();
        reg.install(&ModelPlugin).expect("install");
        assert!(reg.layer_roles().is_empty(), "layers belong to sm_layer_api::CoreLayers");
    }
}
