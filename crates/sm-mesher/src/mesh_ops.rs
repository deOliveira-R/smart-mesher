//! The mesh-driver node (`mesh.run`) and `mesh.delete`.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshEntry, MeshLayer, ModelLayer, insert_mesh, retire_mesh};
use sm_poly::{DualPoints, PolyError, PolyOptions, TopologyResolver};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{MeshEntityKind, MeshRead, SerialAlloc};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_scheme_api::{
    Driver, DriverOptions, MeshInput, MeshIntent, Plan, SchemeError, SizeField, WholeFaces,
};
use sm_topo_api::{Dim, TopologyView};

use crate::config::MesherConfig;
use crate::exec::RayonExec;
use crate::intent_layer::IntentLayer;
use crate::intent_ops::live_targets;

/// `mesh.run`: mesh whole volumes, or faces only, with the driver and store the result as one
/// mesh.
///
/// `targets` are `TopoVolume` ids **or** `TopoFace` ids, never both (a mix is refused: run it
/// twice). Runs the driver (plan, solve, schedule, generate, commit) on them, with the recorded
/// intent and the schemes, solver and picker of the injected [`MesherConfig`]. A volume run
/// makes a 3D mesh; a face run (Cubit `mesh surface`) makes a surface mesh of dimension 2 whose
/// entry selection is the faces. The result is one `Mesh` entity; its vertex, face and cell ids
/// are minted by the layer's allocator and adopted in one range per kind. Output role `mesh`
/// holds the `Mesh` id.
///
/// Atomic: any failure (infeasible hard intervals, no applicable scheme, a failed generation
/// after the allowed re-plans) fails the op and leaves the state untouched. Guards: a volume
/// that is already meshed, that shares a face with a meshed volume not in `targets`, or that
/// has a face in a surface mesh is refused; a face that is already meshed (or bounds a meshed
/// volume), or that shares an edge with a face meshed in another entry, is refused. Each
/// refusal names the ids and the fix (`mesh.delete` first, or mesh them together).
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshRun {
    /// The volumes, or the faces, to mesh (not both).
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace"]))]
    pub targets: Vec<EntityId>,
}

const RUN_OUTPUTS: &[OutputRole] = &[OutputRole::new(
    "mesh",
    &[EntityKind::Mesh],
    Cardinality::One,
    "the new mesh",
)];

pub(crate) fn failed(e: SchemeError) -> OpError {
    match e {
        SchemeError::Input(_)
        | SchemeError::BadParams { .. }
        | SchemeError::OverBudget { .. }
        | SchemeError::UnknownScheme { .. } => OpError::Invalid(e.to_string()),
        other => OpError::Failed(other.to_string()),
    }
}

/// Targets of one kind: `true` for faces. The caller has checked that every id is a volume or a
/// face; a mix is refused.
pub(crate) fn faces_mode(targets: &[EntityId]) -> Result<bool, OpError> {
    let faces = targets
        .iter()
        .filter(|t| t.kind() == EntityKind::TopoFace)
        .count();
    if faces != 0 && faces != targets.len() {
        return Err(OpError::Invalid(
            "mesh.run takes volumes or faces, not both; run it twice".into(),
        ));
    }
    Ok(faces != 0)
}

/// Every reason `targets` (sorted, one kind) cannot be meshed now, in target order. `mesh.run`
/// fails with the first one; `mesh.plan` reports them all as `blockers`.
pub(crate) fn run_guards(
    meshes: &MeshLayer,
    view: &dyn TopologyView,
    targets: &[EntityId],
    faces: bool,
) -> Vec<String> {
    let mut out = Vec::new();
    for &t in targets {
        if faces {
            face_guards(meshes, view, targets, t, &mut out);
        } else {
            volume_guards(meshes, view, targets, t, &mut out);
        }
    }
    out
}

fn volume_guards(
    meshes: &MeshLayer,
    view: &dyn TopologyView,
    targets: &[EntityId],
    t: EntityId,
    out: &mut Vec<String>,
) {
    // INT-4: several entries on one volume (a source plus a derived mesh) are ambiguous;
    // refuse and name them all.
    let ids: Vec<EntityId> = meshes
        .meshes()
        .filter(|(_, e)| e.selection.contains(&t))
        .map(|(m, _)| m)
        .collect();
    if ids.len() > 1 {
        let list: Vec<String> = ids
            .iter()
            .map(|m| match meshes.get(*m).and_then(|e| e.derived_from) {
                Some(src) => format!("{m}, derived from {src}"),
                None => m.to_string(),
            })
            .collect();
        out.push(format!(
            "{t} already has {} meshes ({}); mesh.delete the ones you do not want first",
            ids.len(),
            list.join(", ")
        ));
    } else if let Some(m) = meshes.mesh_of(t) {
        out.push(format!("{t} is already meshed ({m}); mesh.delete first"));
    }
    for &f in view.volume_faces(t) {
        for inc in view.coboundary(f) {
            let other = inc.cell;
            if other != t
                && view.dim_of(other) == Some(Dim::Volume)
                && targets.binary_search(&other).is_err()
                && let Some(m) = meshes.mesh_of(other)
            {
                out.push(format!(
                    "{t} shares {f} with {other}, already meshed ({m}); \
                     mesh.delete first or mesh them together"
                ));
            }
        }
        // A face of this volume that a surface mesh owns would be meshed twice.
        if let Some(m) = meshes.mesh_of(f) {
            out.push(format!(
                "{t} has a face in a surface mesh: {f} is in {m}; mesh.delete first"
            ));
        }
    }
}

/// The mesh that covers face `g`: an entry that selects it, or one that selects a volume
/// bounded by it.
fn face_mesh(meshes: &MeshLayer, view: &dyn TopologyView, g: EntityId) -> Option<EntityId> {
    meshes.mesh_of(g).or_else(|| {
        view.coboundary(g)
            .iter()
            .filter(|i| view.dim_of(i.cell) == Some(Dim::Volume))
            .find_map(|i| meshes.mesh_of(i.cell))
    })
}

fn face_guards(
    meshes: &MeshLayer,
    view: &dyn TopologyView,
    targets: &[EntityId],
    f: EntityId,
    out: &mut Vec<String>,
) {
    if let Some(m) = meshes.mesh_of(f) {
        out.push(format!("{f} is already meshed ({m}); mesh.delete first"));
    }
    for inc in view.coboundary(f) {
        if view.dim_of(inc.cell) == Some(Dim::Volume)
            && let Some(m) = meshes.mesh_of(inc.cell)
        {
            out.push(format!(
                "{f} bounds {}, already meshed ({m}); mesh.delete first",
                inc.cell
            ));
        }
    }
    // Q13: a neighbour across an edge that is meshed elsewhere would leave two unrelated
    // node sets on the shared edge.
    let mut seen = std::collections::BTreeSet::new();
    let mut edges: Vec<EntityId> = view.face_edges(f).to_vec();
    edges.sort_unstable();
    edges.dedup();
    for e in edges {
        for inc in view.coboundary(e) {
            let g = inc.cell;
            if g == f
                || view.dim_of(g) != Some(Dim::Face)
                || targets.binary_search(&g).is_ok()
                || !seen.insert(g)
            {
                continue;
            }
            if let Some(m) = face_mesh(meshes, view, g) {
                out.push(format!(
                    "{f} shares {e} with {g}, meshed in {m}; mesh them together in one \
                     mesh.run, or mesh.delete {m} first"
                ));
            }
        }
    }
}

/// Rough cell count of a face run: per face, the square of the geometric mean of the solved
/// counts of its edges (exact for a structured quad face). Edges not in the plan are skipped; a
/// face with none falls back to the mean over the whole plan. Like the volume estimate, it only
/// guards against runaway requests.
pub(crate) fn estimate_face_cells(plan: &Plan, view: &dyn TopologyView, faces: &[EntityId]) -> f64 {
    let geo = |counts: &mut dyn Iterator<Item = u32>| -> Option<f64> {
        let (mut sum, mut n) = (0.0_f64, 0usize);
        for c in counts {
            sum += f64::from(c.max(1)).ln();
            n += 1;
        }
        (n > 0).then(|| (sum / n as f64).exp().powi(2))
    };
    let mut total = 0.0;
    for &f in faces {
        let mut edges: Vec<EntityId> = view.face_edges(f).to_vec();
        edges.sort_unstable();
        edges.dedup();
        let est = geo(&mut edges.iter().filter_map(|&e| plan.intervals(e)))
            .or_else(|| geo(&mut plan.intervals.iter().map(|p| p.1)));
        total += est.unwrap_or(0.0);
    }
    total
}

/// Volumes whose interval request cannot reach a face run: the planner puts a volume's request
/// on the edges of the volumes in the run, so for faces only it is ignored. Returns the volumes
/// bounding a target face that has no request of its own (on it or on its edges) but whose
/// volume has one.
pub(crate) fn ignored_volume_requests(
    view: &dyn TopologyView,
    intent: &MeshIntent,
    faces: &[EntityId],
) -> Vec<EntityId> {
    let has = |id: &EntityId| {
        intent
            .entities
            .get(id)
            .is_some_and(|e| e.intervals.is_some())
    };
    let mut out = Vec::new();
    for &f in faces {
        if has(&f) || view.face_edges(f).iter().any(has) {
            continue;
        }
        for inc in view.coboundary(f) {
            if view.dim_of(inc.cell) == Some(Dim::Volume) && has(&inc.cell) {
                out.push(inc.cell);
            }
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// The size field of the intent's default sizing, if any.
pub(crate) fn size_field(
    cfg: &MesherConfig,
    intent: &MeshIntent,
) -> Result<Option<Box<dyn SizeField>>, OpError> {
    match (cfg.size_fields(), &intent.defaults.sizing) {
        (Some(build), Some(spec)) => build(spec).map_err(failed),
        _ => Ok(None),
    }
}

/// The driver `mesh.run` and `mesh.plan` share (replan budget `16 + 2 * n_targets`). A face run checks its own budget (the driver's
/// estimate counts volumes), so the driver's is off for it.
pub(crate) fn make_driver<'a>(
    cfg: &'a MesherConfig,
    field: Option<&'a dyn SizeField>,
    faces: bool,
    n_targets: usize,
) -> Driver<'a> {
    let mut driver = Driver::new(cfg.registry(), cfg.solver(), cfg.picker());
    driver.size = field;
    driver.options = DriverOptions::new()
        .with_max_cells(cfg.max_cells())
        .with_max_replans(crate::auto::replan_budget(n_targets))
        .with_cascade(cfg.cascade());
    if faces {
        driver.options.max_cells = None;
    }
    driver
}

impl TypedOp for MeshRun {
    const NAME: &'static str = "mesh.run";
    const IMPL_VERSION: u32 = 2;
    const OUTPUTS: &'static [OutputRole] = RUN_OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let targets = live_targets(
            ctx,
            &self.targets,
            &[EntityKind::TopoVolume, EntityKind::TopoFace],
        )?;
        let faces = faces_mode(&targets)?;
        // Borrows of the ctx are exclusive: take Arc-cheap clones of what we read, then write.
        let cfg = ctx.service::<MesherConfig>()?.clone();
        let model = ctx.layer::<ModelLayer>()?.clone();
        let mut intent = ctx.layer::<IntentLayer>()?.intent().clone();
        // The `polyhedron` scheme: tets from `tetmesh`, then the dual (volumes only).
        let mut dual = None;
        if !faces && let Some((lowered, d)) = crate::polyscheme::lower(&intent, &targets)? {
            intent = lowered;
            dual = Some(d);
        }
        let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();
        {
            let meshes = ctx.layer::<MeshLayer>()?;
            if let Some(first) = run_guards(meshes, model.topo(), &targets, faces)
                .into_iter()
                .next()
            {
                return Err(OpError::Invalid(first));
            }
        }
        ctx.step(0, 3, "planning")?;
        if faces {
            let ignored = ignored_volume_requests(model.topo(), &intent, &targets);
            if !ignored.is_empty() {
                ctx.warn(
                    "mesh.volume_request_ignored",
                    "mesh.run on faces: the size or interval count set on the volume does not \
                     apply to a face-only run; set it on the faces or their edges",
                    &ignored,
                );
            }
        }

        let field = size_field(&cfg, &intent)?;
        let driver = make_driver(&cfg, field.as_deref(), faces, targets.len());
        let input = MeshInput::new(model.topo(), model.model(), &WholeFaces, &targets);
        // Plan once; the driver enforces the cell budget on this plan and every re-plan
        // (volumes), or we do for the first plan (faces).
        let plan = driver.plan(&input, &intent).map_err(failed)?;
        if faces {
            let cells = estimate_face_cells(&plan, model.topo(), &targets);
            if cells > cfg.max_cells() as f64 {
                return Err(failed(SchemeError::OverBudget {
                    cells,
                    max: cfg.max_cells(),
                }));
            }
        }
        let builder = if faces {
            MeshBuilder::surface()
        } else {
            MeshBuilder::new()
        };
        // Work on a copy of the allocator; the layer only advances on success.
        let mut alloc = alloc0.clone();
        let out = driver
            .mesh_from_plan(&input, &intent, plan, &RayonExec, builder, &mut alloc)
            .map_err(failed)?;
        ctx.step(2, 3, "committing")?;

        // INT-7: the schemes' located diagnostics ride on the (unhashed) entry, not the plan:
        // the plan reproduces its mesh exactly and a derived mesh does not inherit the notes.
        let notes = out.patches.notes();
        let mut entry = MeshEntry::new(
            Box::new(out.finished.mesh),
            targets,
            Some(out.plan),
            model.hash_quanta(),
        );
        entry.notes = notes;
        match dual {
            None => insert_mesh(ctx, entry, &alloc0, alloc, "mesh"),
            Some(dual) => {
                let source = entry.mesh().mesh_id();
                insert_scratch(ctx, entry, &alloc0, alloc)?;
                dualize_in_place(ctx, &model, source, dual)
            }
        }
    }
}

/// [`insert_mesh`] without an output role: the tet mesh of a `polyhedron` run is adopted (so
/// the ids and the allocator advance exactly as for a stand-alone `mesh.run`) and retired again
/// by [`dualize_in_place`]; the delta's `mesh` role holds the polyhedral mesh only.
fn insert_scratch(
    ctx: &mut ApplyCtx<'_>,
    entry: MeshEntry,
    before: &SerialAlloc,
    after: SerialAlloc,
) -> Result<(), OpError> {
    let id = entry.mesh().mesh_id();
    ctx.layer::<MeshLayer>()?.check_insert(id, &entry, &after)?;
    for (kind, next) in after.state() {
        let first = before.next_serial(kind);
        if next <= first {
            continue;
        }
        if kind == EntityKind::Mesh {
            let ids: Vec<EntityId> = (first..next)
                .filter_map(|s| EntityId::new(kind, s))
                .collect();
            ctx.adopt(&ids)?;
        } else {
            ctx.adopt_range(EntityId::new(kind, first).ok_or(OpError::IdExhausted)?, next - first)?;
        }
    }
    ctx.layer_mut::<MeshLayer>()?.insert(id, entry, after)?;
    Ok(())
}

/// The second half of a `polyhedron` run: what `mesh.to_poly(keep_source: false)` does to the
/// tet mesh `source` just inserted (same allocator use, same entry fields, same succession), so
/// the layer ends byte-identical to `tetmesh` run, then that op.
fn dualize_in_place(
    ctx: &mut ApplyCtx<'_>,
    model: &ModelLayer,
    source: EntityId,
    dual: DualPoints,
) -> Result<(), OpError> {
    let (host, selection, plan, quanta, stale, alloc0) = {
        let layer = ctx.layer::<MeshLayer>()?;
        let e = layer.get(source).ok_or(OpError::UnknownEntity(source))?;
        (
            e.mesh.clone(),
            e.selection.clone(),
            e.plan.clone(),
            e.quanta,
            e.stale,
            layer.alloc().clone(),
        )
    };
    ctx.step(2, 4, "dualizing")?;
    let mut alloc = alloc0.clone();
    let (fin, _report) = sm_poly::dualize(
        host.as_read(),
        &PolyOptions::new().with_all_polyhedra(true).with_dual(dual),
        &TopologyResolver(model.topo()),
        MeshBuilder::new(),
        &mut alloc,
    )
    .map_err(|e| match e {
        PolyError::NotOrthogonalDual { .. }
        | PolyError::UnrepairedCell { .. }
        | PolyError::InvalidOutput { .. }
        | PolyError::Mesh(_) => OpError::Failed(e.to_string()),
        other => OpError::Invalid(other.to_string()),
    })?;
    let new = fin.mesh.mesh_id();
    let mut entry = MeshEntry::new(Box::new(fin.mesh), selection, None, quanta);
    entry.plan = plan;
    entry.stale = stale;
    entry.derived_from = Some(source);
    insert_mesh(ctx, entry, &alloc0, alloc, "mesh")?;
    ctx.record_successors(source, &[new])?;
    retire_mesh(ctx, source)
}

/// `mesh.delete`: delete meshes and retire every id they own.
///
/// Name the meshes (`meshes`) or the volumes or faces they were generated for (`targets`);
/// exactly one of the two. Every `Mesh*` id of each mesh is retired, so the live entity counts return to what
/// they were before `mesh.run`. Fails, changing nothing, if a named mesh does not exist or a
/// target has no mesh.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshDelete {
    /// The meshes to delete.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub meshes: Option<Vec<EntityId>>,
    /// Delete the meshes of these volumes or faces.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace"]))]
    pub targets: Option<Vec<EntityId>>,
}

impl TypedOp for MeshDelete {
    const NAME: &'static str = "mesh.delete";
    const NO_OUTPUTS: bool = true;

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
                    v.push(
                        layer
                            .mesh_of(t)
                            .ok_or_else(|| OpError::Invalid(format!("{t} has no mesh")))?,
                    );
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
            retire_mesh_bulk(ctx, m)?;
        }
        Ok(())
    }
}

/// Same contract as `sm_layer_api::retire_mesh` (retires the `Mesh` id and every vertex,
/// edge, face, cell and set id of the mesh), but retires consecutive id runs with
/// [`ApplyCtx::retire_range`]. State hash and `Delta::removed` are identical to the per-id path.
fn retire_mesh_bulk(ctx: &mut ApplyCtx<'_>, mesh: EntityId) -> Result<(), OpError> {
    let ids = {
        let entry = ctx
            .layer::<MeshLayer>()?
            .get(mesh)
            .ok_or(OpError::UnknownEntity(mesh))?;
        let m: &dyn MeshRead = entry.mesh();
        let mut ids = vec![mesh];
        for kind in [
            MeshEntityKind::Vertex,
            MeshEntityKind::Edge,
            MeshEntityKind::Face,
            MeshEntityKind::Cell,
        ] {
            ids.extend((0..m.count(kind)).map(|i| m.id(kind, i as u32)));
        }
        ids.extend(m.sets().iter().map(|s| s.id));
        ids.sort_unstable();
        ids.dedup();
        ids
    };
    // Maximal consecutive runs of one kind.
    let mut runs: Vec<(EntityId, u64)> = Vec::new();
    for &id in &ids {
        match runs.last_mut() {
            Some((first, n)) if first.kind() == id.kind() && first.serial() + *n == id.serial() => {
                *n += 1;
            }
            _ => runs.push((id, 1)),
        }
    }
    // A dead id fails the op, and the engine then discards the whole scratch state.
    ctx.layer_mut::<MeshLayer>()?.remove(mesh);
    for (first, n) in runs {
        ctx.retire_range(first, n)?;
    }
    Ok(())
}
