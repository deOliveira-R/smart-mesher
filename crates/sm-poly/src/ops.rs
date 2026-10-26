//! Op `mesh.to_poly` and query `mesh.poly_info`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshEntry, MeshLayer, ModelLayer, insert_mesh, retire_mesh};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{MeshDim, SerialAlloc};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, QueryCtx, State, TypedOp, TypedQuery};

use crate::error::PolyError;
use crate::options::PolyOptions;
use crate::resolver::TopologyResolver;
use crate::stats::{PolyStats, poly_stats};

/// `mesh.to_poly`: convert a whole all-tet mesh into a new polyhedral mesh.
///
/// The source mesh entry (`mesh`) must be a dimension-3 mesh of tetrahedra with `Interior` and
/// `Boundary` faces only. The result is a NEW mesh entry (output role `mesh`): one cell per
/// (vertex, volume), every face planar, boundary faces classified on the source's model faces.
/// With the default `keep_source` the source stays, and the volume then has two mesh entries,
/// so exports must name the mesh. `keep_source: false` retires the source's `Mesh*` ids.
///
/// Atomic: any refusal (not 3D, a non-tet cell, an already polyhedral mesh, a non-conformal
/// face, an unsupported feature) fails with `OpError::Invalid` and changes nothing.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ToPoly {
    /// The tet mesh to convert.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Keep the source mesh entry (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep_source: Option<bool>,
}

const OUTPUTS: &[OutputRole] = &[OutputRole::new(
    "mesh",
    &[EntityKind::Mesh],
    Cardinality::One,
    "the new polyhedral mesh",
)];

fn refuse(e: PolyError) -> OpError {
    match e {
        PolyError::Mesh(m) => OpError::Failed(m.to_string()),
        PolyError::NotTet { cell } => OpError::Invalid(format!(
            "cell {} is not a tetrahedron; mesh.to_poly converts all-tet meshes only (a polyhedral or mixed mesh cannot be converted)",
            cell.0
        )),
        other => OpError::Invalid(other.to_string()),
    }
}

impl TypedOp for ToPoly {
    const NAME: &'static str = "mesh.to_poly";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let source = ctx.resolve_kind(self.mesh, EntityKind::Mesh)?;
        // Cheap clones (Arc) of what we read, so the context is free for the writes below.
        let (host, selection, plan, quanta, alloc0) = {
            let layer = ctx.layer::<MeshLayer>()?;
            let e = layer.get(source).ok_or(OpError::UnknownEntity(source))?;
            (e.mesh.clone(), e.selection.clone(), e.plan.clone(), e.quanta, layer.alloc().clone())
        };
        let model = ctx.layer::<ModelLayer>()?.clone();
        let src = host.as_read();
        if src.dim() != MeshDim::Three {
            return Err(OpError::Invalid(format!("{source} is not a 3D mesh")));
        }
        ctx.step(0, 2, "dualizing")?;
        let mut alloc: SerialAlloc = alloc0.clone();
        let (fin, _report) = crate::dualize(
            src,
            &PolyOptions::new(),
            &TopologyResolver(model.topo()),
            MeshBuilder::new(),
            &mut alloc,
        )
        .map_err(refuse)?;
        ctx.step(1, 2, "inserting")?;
        let mut entry = MeshEntry::new(Box::new(fin.mesh), selection, None, quanta);
        entry.plan = plan;
        insert_mesh(ctx, entry, &alloc0, alloc, "mesh")?;
        if self.keep_source == Some(false) {
            retire_mesh(ctx, source)?;
        }
        Ok(())
    }
}

/// `mesh.poly_info`: statistics of a mesh (faces per cell, vertices per face, planarity,
/// volumes, boundary area). Equal to [`poly_stats`].
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PolyInfo {
    /// The mesh.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
}

impl TypedQuery for PolyInfo {
    const NAME: &'static str = "mesh.poly_info";
    type Output = PolyStats;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<PolyStats, OpError> {
        let layer = state
            .layer::<MeshLayer>()
            .ok_or_else(|| OpError::MissingLayer("meshes".into()))?;
        let e = layer.get(self.mesh).ok_or(OpError::UnknownEntity(self.mesh))?;
        Ok(poly_stats(e.mesh()))
    }
}
