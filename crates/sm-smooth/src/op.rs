//! The `mesh.smooth` op, the `mesh.smooth_preview` query and the plugin.

use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{LayerResultExt, MeshLayer, ModelLayer};
use sm_mesh_api::CellIx;
use sm_op_api::{ApplyCtx, OpError, Plugin, QueryCtx, Registry, RegistryError, State, TypedOp, TypedQuery};

use crate::smooth::{apply_outcome, smooth};
use crate::types::{Method, NodeSelect, Schedule, SmoothError, SmoothInput, SmoothReport, SmoothRequest};

/// The `smooth` plugin: registers `mesh.smooth` and `mesh.smooth_preview`. It registers no
/// layer; install `CoreLayers` first. It writes only the `meshes` layer.
#[derive(Clone, Copy, Debug, Default)]
pub struct SmoothPlugin;

impl Plugin for SmoothPlugin {
    fn id(&self) -> &str {
        "smooth"
    }

    fn register(&self, registry: &mut Registry) -> Result<(), RegistryError> {
        registry.register::<MeshSmooth>()?;
        registry.register_query::<MeshSmoothPreview>()?;
        Ok(())
    }
}

/// `mesh.smooth`: smooth the nodes of a mesh in place (the mesh keeps its ids; no id is minted
/// or retired). Boundary nodes stay on their curves and surfaces, vertices never move, and the
/// guard refuses any move that makes an incident cell worse than `min(its ScaledJacobian,
/// guard_floor)` or turns a positive volume non-positive.
///
/// Diagnostics: `smooth.report` (the [`SmoothReport`] as JSON), and warnings
/// `smooth.inverted_input`, `smooth.no_improvement`, `smooth.not_converged`,
/// `smooth.pinned_deck`, `smooth.pinned_interface`, `smooth.pinned_polyhedral`.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshSmooth {
    /// The mesh.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Restrict to the nodes of these volumes, faces or edges (default: the whole mesh).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace", "TopoEdge"]))]
    pub entities: Option<Vec<EntityId>>,
    /// Update rule, e.g. `{"kind":"laplacian","relax":1.0}` or
    /// `{"kind":"optimize","objective":"mean_ratio","p":4}`.
    pub method: Method,
    /// Sweeps, 1 to 10000 (default 10).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(range(min = 1, max = 10000))]
    pub iterations: Option<u32>,
    /// Stop when the largest relative node move falls below this (default 1e-6).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tolerance: Option<f64>,
    /// `"colored"` (default) or `"jacobi"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub schedule: Option<Schedule>,
    /// Keep face and edge nodes fixed unless listed (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fix_boundary: Option<bool>,
    /// Guard floor on the scaled Jacobian, in [0, 1) (default 1e-3).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub guard_floor: Option<f64>,
}

/// `mesh.smooth_preview`: the [`SmoothReport`] `mesh.smooth` would produce, without changing
/// the state. Same arguments as [`MeshSmooth`].
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshSmoothPreview {
    /// The mesh.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Restrict to the nodes of these volumes, faces or edges (default: the whole mesh).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace", "TopoEdge"]))]
    pub entities: Option<Vec<EntityId>>,
    /// Update rule.
    pub method: Method,
    /// Sweeps, 1 to 10000 (default 10).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(range(min = 1, max = 10000))]
    pub iterations: Option<u32>,
    /// Stop when the largest relative node move falls below this (default 1e-6).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tolerance: Option<f64>,
    /// `"colored"` (default) or `"jacobi"`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub schedule: Option<Schedule>,
    /// Keep face and edge nodes fixed unless listed (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fix_boundary: Option<bool>,
    /// Guard floor on the scaled Jacobian, in [0, 1) (default 1e-3).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub guard_floor: Option<f64>,
}

fn request(
    entities: &Option<Vec<EntityId>>,
    method: Method,
    iterations: Option<u32>,
    tolerance: Option<f64>,
    schedule: Option<Schedule>,
    fix_boundary: Option<bool>,
    guard_floor: Option<f64>,
) -> Result<SmoothRequest, OpError> {
    let d = SmoothRequest::default();
    let iterations = iterations.unwrap_or(d.iterations);
    if !(1..=10_000).contains(&iterations) {
        return Err(OpError::Invalid(format!("iterations {iterations} must be in 1..=10000")));
    }
    let nodes = match entities {
        None => NodeSelect::All,
        Some(v) if v.is_empty() => return Err(OpError::Invalid("entities is empty; omit it to smooth the whole mesh".into())),
        Some(v) => NodeSelect::Entities(v.clone()),
    };
    Ok(SmoothRequest {
        nodes,
        method,
        iterations,
        tolerance: tolerance.unwrap_or(d.tolerance),
        schedule: schedule.unwrap_or(d.schedule),
        fix_boundary: fix_boundary.unwrap_or(d.fix_boundary),
        guard_floor: guard_floor.unwrap_or(d.guard_floor),
    })
}

impl MeshSmooth {
    fn request(&self) -> Result<SmoothRequest, OpError> {
        request(&self.entities, self.method, self.iterations, self.tolerance, self.schedule, self.fix_boundary, self.guard_floor)
    }
}

impl MeshSmoothPreview {
    fn request(&self) -> Result<SmoothRequest, OpError> {
        request(&self.entities, self.method, self.iterations, self.tolerance, self.schedule, self.fix_boundary, self.guard_floor)
    }
}

fn map_err(e: SmoothError) -> OpError {
    match e {
        SmoothError::UnknownEntity(id) => OpError::UnknownEntity(id),
        SmoothError::Unsupported(s) => OpError::Unsupported(format!("mesh.smooth: {s}")),
        other => OpError::Invalid(other.to_string()),
    }
}

impl TypedOp for MeshSmooth {
    const NAME: &'static str = "mesh.smooth";
    const NO_OUTPUTS: bool = true;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        ctx.resolve_kind(self.mesh, EntityKind::Mesh)?;
        let req = self.request()?;
        // Everything that can fail runs before the O(mesh) host copy.
        let (outcome, host, alloc, inverted_ids) = {
            let meshes = ctx.layer::<MeshLayer>()?;
            let entry = meshes.get(self.mesh).ok_or(OpError::UnknownEntity(self.mesh))?;
            let model = ctx.layer::<ModelLayer>()?;
            let input = SmoothInput { mesh: entry.mesh(), topo: model.topo(), model: model.model() };
            let outcome = smooth(&input, &req).map_err(map_err)?;
            let ids: Vec<EntityId> =
                outcome.report.inverted_before.iter().map(|c: &CellIx| entry.mesh().cell_id(*c)).collect();
            (outcome, entry.mesh.clone_host(), meshes.alloc().clone(), ids)
        };
        let mut host = host;
        apply_outcome(&mut *host, &outcome).map_err(map_err)?;
        ctx.layer_mut::<MeshLayer>()?.replace_mesh(self.mesh, Arc::from(host), alloc).or_op()?;
        diagnostics(ctx, self.mesh, &outcome.report, &inverted_ids);
        Ok(())
    }
}

fn diagnostics(ctx: &mut ApplyCtx<'_>, mesh: EntityId, r: &SmoothReport, inverted_ids: &[EntityId]) {
    let json = serde_json::to_string(r).unwrap_or_default();
    ctx.info("smooth.report", json, &[mesh]);
    if r.before.n_inverted > 0 {
        let mut ents = vec![mesh];
        ents.extend_from_slice(inverted_ids);
        ctx.warn(
            "smooth.inverted_input",
            format!(
                "{} input cell(s) were inverted. Smoothing never makes a cell worse but does not untangle; \
                 untangling is deferred, so remesh with different intervals or scheme first",
                r.before.n_inverted
            ),
            &ents,
        );
    }
    if r.after.min_shape <= r.before.min_shape + 1e-12 * r.before.min_shape.abs().max(1.0) {
        ctx.warn(
            "smooth.no_improvement",
            format!(
                "the minimum Shape did not improve ({:.6} -> {:.6}); change the intervals or scheme, \
                 or smooth with fix_boundary false or method optimize",
                r.before.min_shape, r.after.min_shape
            ),
            &[mesh],
        );
    }
    if !r.converged {
        ctx.warn(
            "smooth.not_converged",
            format!(
                "the largest relative move was {:.3e} after {} sweep(s); raise iterations or tolerance",
                r.max_move_last, r.iterations_run
            ),
            &[mesh],
        );
    }
    let list = |k: &str| r.pinned_entities.get(k).cloned().unwrap_or_default();
    for (key, code, what) in [
        ("deck", "smooth.pinned_deck", "nodes on translation or rotation deck faces were left in place (tied smoothing is not available)"),
        ("interface", "smooth.pinned_interface", "nodes on explicit interface faces were left in place"),
        ("polyhedral", "smooth.pinned_polyhedral", "nodes of polyhedral cells were left in place; smooth the tets before mesh.to_poly"),
    ] {
        if let Some(n) = r.pinned.get(key) {
            let mut ents = vec![mesh];
            ents.extend(list(key));
            let code: &'static str = code;
            ctx.warn(code, format!("{n} {what}"), &ents);
        }
    }
}

impl TypedQuery for MeshSmoothPreview {
    const NAME: &'static str = "mesh.smooth_preview";
    type Output = SmoothReport;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<SmoothReport, OpError> {
        let meshes = state.layer::<MeshLayer>().ok_or_else(|| OpError::MissingLayer("meshes".into()))?;
        let model = state.layer::<ModelLayer>().ok_or_else(|| OpError::MissingLayer("model".into()))?;
        let entry = meshes.get(self.mesh).ok_or(OpError::UnknownEntity(self.mesh))?;
        let req = self.request()?;
        let input = SmoothInput { mesh: entry.mesh(), topo: model.topo(), model: model.model() };
        smooth(&input, &req).map(|o| o.report).map_err(map_err)
    }
}
