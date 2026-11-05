//! Primitives, transforms, copy and topology queries (W2 task T1, W3b-PRIM).
//!
//! [`ModelPlugin`] registers ops and queries only; the `model` layer itself comes from
//! `sm_layer_api::CoreLayers`. See `DESIGN.md`.

mod copy;
mod delete;
mod prim;
mod prim2;
mod queries;
mod sphere_part;
mod xform;
mod xform2;

pub use copy::{CopyXform, ModelCopy};
pub use delete::Delete;
pub use prim::{CreateBrick, CreateCylinder, CreateSphere};
pub use prim2::{CreateFrustum, CreatePrism, CreatePyramid, CreateTorus};
pub use queries::{BBox, Measurement, ModelList, ModelMeasure, TopoRelated};
pub use sphere_part::{CreateSpherePart, HalfSpace, Octants};
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
        registry.register_query::<TopoRelated>()?;
        registry.register_query::<ModelList>()?;
        registry.register_query::<ModelMeasure>()?;
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
                "model.move",
                "model.move_to",
                "model.reflect",
                "model.rotate",
                "model.rotate_about",
                "model.scale"
            ]
        );
        let mut qs = reg.query_names();
        qs.sort_unstable();
        assert_eq!(qs, ["model.list", "model.measure", "topo.related"]);
    }

    #[test]
    fn the_plugin_registers_no_layers() {
        let mut reg = Registry::new();
        reg.install(&ModelPlugin).expect("install");
        assert!(reg.layer_roles().is_empty(), "layers belong to sm_layer_api::CoreLayers");
    }
}
