//! The mesh-driver node (`mesh.run`) and `mesh.delete`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshEntry, MeshLayer, ModelLayer, insert_mesh, retire_mesh};
use sm_mesh::MeshBuilder;
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_scheme_api::{Driver, DriverOptions, MeshInput, SchemeError, WholeFaces};
use sm_topo_api::Dim;

use crate::config::MesherConfig;
use crate::exec::RayonExec;
use crate::intent_layer::IntentLayer;
use crate::intent_ops::live_targets;

/// `mesh.run`: mesh whole volumes with the driver and store the result as one mesh.
///
/// Runs the driver (plan, solve, schedule, generate, commit) on the volumes in `targets`, with the
/// recorded intent and the schemes, solver and picker of the injected [`MesherConfig`]. The
/// result is one `Mesh` entity; its vertex, face and cell ids are minted by the layer's
/// allocator and adopted in one range per kind. Output role `mesh` holds the `Mesh` id.
///
/// Atomic: any failure (infeasible hard intervals, no applicable scheme, a failed generation
/// after the allowed re-plans) fails the op and leaves the state untouched. A volume that is
/// already meshed, or that shares a face with a meshed volume not in `targets`, is refused:
/// `mesh.delete` first, or mesh the neighbours together.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshRun {
    /// The volumes to mesh.
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub targets: Vec<EntityId>,
}

const RUN_OUTPUTS: &[OutputRole] = &[OutputRole::new(
    "mesh",
    &[EntityKind::Mesh],
    Cardinality::One,
    "the new mesh",
)];

fn failed(e: SchemeError) -> OpError {
    match e {
        SchemeError::Input(_) | SchemeError::BadParams { .. } | SchemeError::UnknownScheme { .. } => {
            OpError::Invalid(e.to_string())
        }
        other => OpError::Failed(other.to_string()),
    }
}

impl TypedOp for MeshRun {
    const NAME: &'static str = "mesh.run";
    const OUTPUTS: &'static [OutputRole] = RUN_OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let targets = live_targets(ctx, &self.targets, &[EntityKind::TopoVolume])?;
        // Borrows of the ctx are exclusive: take Arc-cheap clones of what we read, then write.
        let cfg = ctx.service::<MesherConfig>()?.clone();
        let model = ctx.layer::<ModelLayer>()?.clone();
        let intent = ctx.layer::<IntentLayer>()?.intent().clone();
        let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();
        {
            let meshes = ctx.layer::<MeshLayer>()?;
            let view = model.topo();
            for &t in &targets {
                if let Some(m) = meshes.mesh_of(t) {
                    return Err(OpError::Invalid(format!(
                        "{t} is already meshed ({m}); mesh.delete first"
                    )));
                }
                for &f in view.volume_faces(t) {
                    for inc in view.coboundary(f) {
                        let other = inc.cell;
                        if other != t
                            && view.dim_of(other) == Some(Dim::Volume)
                            && targets.binary_search(&other).is_err()
                            && let Some(m) = meshes.mesh_of(other)
                        {
                            return Err(OpError::Invalid(format!(
                                "{t} shares {f} with {other}, already meshed ({m}); \
                                 mesh.delete first or mesh them together"
                            )));
                        }
                    }
                }
            }
        }
        ctx.step(0, 3, "planning")?;

        let field = match (cfg.size_fields(), &intent.defaults.sizing) {
            (Some(build), Some(spec)) => build(spec).map_err(failed)?,
            _ => None,
        };
        let mut driver = Driver::new(cfg.registry(), cfg.solver(), cfg.picker());
        driver.size = field.as_deref();
        driver.options = DriverOptions::new();
        let input = MeshInput::new(model.topo(), model.model(), &WholeFaces, &targets);
        // Work on a copy of the allocator; the layer only advances on success.
        let mut alloc = alloc0.clone();
        let out = driver
            .mesh(&input, &intent, &RayonExec, MeshBuilder::new(), &mut alloc)
            .map_err(failed)?;
        ctx.step(2, 3, "committing")?;

        let entry = MeshEntry::new(
            Box::new(out.finished.mesh),
            targets,
            Some(out.plan),
            model.hash_quanta(),
        );
        insert_mesh(ctx, entry, &alloc0, alloc, "mesh")
    }
}

/// `mesh.delete`: delete meshes and retire every id they own.
///
/// Name the meshes (`meshes`) or the volumes they were generated for (`targets`); exactly one of
/// the two. Every `Mesh*` id of each mesh is retired, so the live entity counts return to what
/// they were before `mesh.run`. Fails, changing nothing, if a named mesh does not exist or a
/// target has no mesh.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshDelete {
    /// The meshes to delete.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub meshes: Option<Vec<EntityId>>,
    /// Delete the meshes of these volumes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume"]))]
    pub targets: Option<Vec<EntityId>>,
}

impl TypedOp for MeshDelete {
    const NAME: &'static str = "mesh.delete";

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let mut meshes: Vec<EntityId> = match (&self.meshes, &self.targets) {
            (Some(ms), None) => {
                for &m in ms {
                    if m.kind() != EntityKind::Mesh {
                        return Err(OpError::Invalid(format!("{m} is not a Mesh")));
                    }
                }
                ms.clone()
            }
            (None, Some(ts)) => {
                let layer = ctx.layer::<MeshLayer>()?;
                let mut v = Vec::with_capacity(ts.len());
                for &t in ts {
                    v.push(layer.mesh_of(t).ok_or_else(|| {
                        OpError::Invalid(format!("{t} has no mesh"))
                    })?);
                }
                v
            }
            _ => {
                return Err(OpError::Invalid(
                    "give exactly one of `meshes` and `targets`".into(),
                ));
            }
        };
        if meshes.is_empty() {
            return Err(OpError::Invalid("nothing to delete".into()));
        }
        meshes.sort_unstable();
        meshes.dedup();
        for m in meshes {
            retire_mesh(ctx, m)?;
        }
        Ok(())
    }
}
