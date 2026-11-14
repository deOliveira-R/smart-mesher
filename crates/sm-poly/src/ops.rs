//! Op `mesh.to_poly` and query `mesh.poly_info`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshEntry, MeshLayer, ModelLayer, insert_mesh, retire_mesh};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{MeshDim, MeshRead, SerialAlloc};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, QueryCtx, State, TypedOp, TypedQuery};

use crate::error::PolyError;
use crate::options::{DualPoints, PolyOptions};
use crate::resolver::TopologyResolver;
use crate::stats::{PolyStats, poly_stats};

/// `mesh.to_poly`: convert a whole tet mesh into a new polyhedral mesh.
///
/// The source mesh entry (`mesh`) must be a dimension-3 mesh of tetrahedra with `Interior` and
/// `Boundary` faces only. A mixed mesh (tets plus hexes, wedges or pyramids) is accepted too: the
/// non-tet cells are kept as polyhedra and the faces between them and the tets are cut into dual
/// sub-faces; a mesh with no tet is refused. The result is a NEW mesh entry (output role `mesh`): one cell per
/// (vertex, volume), every face planar, boundary faces classified on the source's model faces.
/// With the default `keep_source` the source stays, and the volume then has two mesh entries,
/// so exports must name the mesh. `keep_source: false` retires the source's `Mesh*` ids.
///
/// The delta records the succession `source -> [source, new]` (`source -> [new]` with
/// `keep_source: false`), so layers and front ends can tell the new mesh is derived from the
/// source. The entry's `selection`, `plan`, `quanta` and `stale` are copied from the source.
///
/// Atomic: any refusal (not 3D, a mesh with no tet, an already polyhedral mesh, a non-conformal
/// face, an unsupported mesh topology) fails with `OpError::Invalid`, and a mesh with an
/// unsupported feature (a seam or dangling feature edge) with `OpError::Unsupported`; a cell that
/// cannot be repaired to a valid star-shaped polyhedron with `OpError::Failed`; the state
/// is unchanged either way.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ToPoly {
    /// The tet mesh to convert.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Keep the source mesh entry (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep_source: Option<bool>,
    /// Dual-point rule: `"barycentric"` (default, valid on any tet mesh) or `"circumcentric"`
    /// (planar convex orthogonal cells with one face per neighbour pair; strict, refused with
    /// `OpError::Failed` unless the mesh is locally Delaunay with inside circumcentres).
    /// Omitted from the parameters when barycentric.
    #[serde(default, skip_serializing_if = "DualPoints::is_default")]
    pub dual: DualPoints,
}

const OUTPUTS: &[OutputRole] = &[OutputRole::new(
    "mesh",
    &[EntityKind::Mesh],
    Cardinality::One,
    "the new polyhedral mesh",
)];

/// The op error of a [`PolyError`], the same mapping for every op that dualizes (`mesh.to_poly`,
/// the mesher's dual strategies): construction failures are `Failed`, an unsupported feature is
/// `Unsupported`, anything else is a bad input (`Invalid`).
pub fn poly_error_to_op(e: PolyError) -> OpError {
    match e {
        PolyError::Mesh(m) => OpError::Failed(m.to_string()),
        e @ (PolyError::UnrepairedCell { .. } | PolyError::InvalidAgglomeration { .. } | PolyError::NotOrthogonalDual { .. } | PolyError::InvalidOutput { .. }) => OpError::Failed(e.to_string()),
        PolyError::UnsupportedFeature { vertex } => OpError::Unsupported(format!(
            "vertex {} has exactly one feature edge in a region (a seam or a dangling curve); not supported in this version",
            vertex.0
        )),
        PolyError::NotTet { cell } => OpError::Invalid(format!(
            "cell {} is not a tetrahedron and the mesh has no tetrahedron; mesh.to_poly needs at least one (a polyhedral or all-hex mesh cannot be converted)",
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
        let (host, selection, plan, quanta, stale, alloc0) = {
            let layer = ctx.layer::<MeshLayer>()?;
            let e = layer.get(source).ok_or(OpError::UnknownEntity(source))?;
            (e.mesh.clone(), e.selection.clone(), e.plan.clone(), e.quanta, e.stale, layer.alloc().clone())
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
            &PolyOptions::new().with_all_polyhedra(true).with_dual(self.dual),
            &TopologyResolver(model.topo()),
            MeshBuilder::new(),
            &mut alloc,
        )
        .map_err(poly_error_to_op)?;
        ctx.step(1, 2, "inserting")?;
        let new = fin.mesh.mesh_id();
        let mut entry = MeshEntry::new(Box::new(fin.mesh), selection, None, quanta);
        entry.plan = plan;
        // Scheme notes (`entry.notes`) are located on the source cells: not inherited.
        entry.stale = stale;
        entry.derived_from = Some(source);
        insert_mesh(ctx, entry, &alloc0, alloc, "mesh")?;
        // The new mesh is a derived copy of the source (`A -> [A, A']`), or its replacement.
        if self.keep_source == Some(false) {
            ctx.record_successors(source, &[new])?;
            retire_mesh(ctx, source)?;
        } else {
            ctx.record_successors(source, &[source, new])?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use sm_core::EntityKind;

    #[test]
    fn unrepaired_and_agglomeration_failures_are_failed_not_invalid() {
        let region = EntityId::new(EntityKind::TopoVolume, 1).unwrap();
        let a = PolyError::UnrepairedCell { vertex: sm_mesh_api::VertexIx(3), region, at: [1.0; 3], count: 2 };
        let b = PolyError::InvalidAgglomeration { region, at: [1.0; 3], count: 1 };
        assert!(matches!(poly_error_to_op(a), OpError::Failed(m) if m.contains("not star-shaped")));
        assert!(matches!(poly_error_to_op(b), OpError::Failed(m) if m.contains("agglomerated cell")));
        assert!(matches!(poly_error_to_op(PolyError::Dim2), OpError::Invalid(_)));
    }
}
