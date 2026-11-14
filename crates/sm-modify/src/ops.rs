//! Op `mesh.refine`: uniform refinement into a new, linked mesh.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshEntry, MeshLayer, ModelLayer, insert_mesh, retire_mesh};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{CatalogEdit, LevelMap, MeshDim, MeshRead, SerialAlloc};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_quality::{Metric, Options, compute_many};

use crate::refine::{RefineError, refine};

/// A level warns (`refine.quality`) when its minimum scaled Jacobian is below this factor times the
/// parent's. Measured on the default-mesher pin (boundary-hugging corner cells): 0.54, 0.51, 0.50
/// per level, so 0.6 catches that and stays quiet for the 0.7 of a tet octahedron cut.
pub const QUALITY_DROP_FACTOR: f64 = 0.6;

/// Most levels one `mesh.refine` call may add (each is 8 times the cells).
pub const MAX_LEVELS: u32 = 6;

/// `mesh.refine`: refine a mesh uniformly into new, finer, independent mesh entries.
///
/// Every cell is split by its uniform rule (tet 1 to 8, hex 1 to 8, wedge 1 to 8, pyramid 1 to 6
/// pyramids plus 4 tets, tri 1 to 4, quad 1 to 4). Classification, sets, blocks and cell tags
/// carry over: a set's members become their children (face sets: the child faces on the
/// member faces). New nodes classified on a model edge or face are projected onto it unless that
/// would invert an incident cell (`project: false` keeps straight midpoints).
///
/// Each level is a NEW mesh entry (output role `mesh`, coarsest first) linked to the previous
/// level (the source for the first) by a fine-to-coarse `LevelMap` with a face map, so a CMFD or
/// multigrid code gets the pair directly. With the default `keep_source` the source stays and the
/// first level links to it; `keep_source: false` retires the source and its links. The report
/// of each level (counts, projection counters, volumes and minimum scaled Jacobian of parent and
/// child) is the `refine.report` diagnostic.
///
/// Projection moves new nodes onto the circle but does not make the fine polygon volume equal to
/// the exact one: run `mesh.preserve_volume` (VOLFIT) again after each level. A level whose
/// minimum scaled Jacobian drops below 0.6 times its parent's raises the `refine.quality`
/// warning (boundary-hugging corner cells flatten as the circle polygon refines; node placement
/// cannot repair that).
///
/// Refused with `OpError::Invalid`: polyhedral cells (refine the tet source and dualize again),
/// inactive or non-conformal cells; high-order meshes with `OpError::Unsupported`. Atomic: the
/// state is unchanged on any refusal.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Refine {
    /// The mesh to refine.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// How many times to refine (default 1, at most 6); each level is its own mesh entry.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub levels: Option<u32>,
    /// Keep the source mesh entry (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep_source: Option<bool>,
    /// Project new boundary nodes onto the model geometry (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub project: Option<bool>,
}

const OUTPUTS: &[OutputRole] = &[OutputRole::new(
    "mesh",
    &[EntityKind::Mesh],
    Cardinality::Many,
    "the refined meshes, coarsest first",
)];

fn refuse(e: RefineError) -> OpError {
    match e {
        RefineError::HighOrder => OpError::Unsupported(e.to_string()),
        RefineError::UnsupportedCell { .. }
        | RefineError::Inactive(_)
        | RefineError::NonConformal(_)
        | RefineError::Empty => OpError::Invalid(e.to_string()),
        e => OpError::Failed(e.to_string()),
    }
}

/// `(min scaled Jacobian, total volume)` of a mesh, `None` where the metrics fail.
fn quality(m: &dyn MeshRead) -> (Option<f64>, Option<f64>) {
    match compute_many(m, &[Metric::ScaledJacobian, Metric::Volume], &Options::default()) {
        Ok(t) => {
            let sj = t.columns[0].iter().copied().filter(|x| x.is_finite()).fold(f64::INFINITY, f64::min);
            let vol: f64 = t.columns[1].iter().copied().filter(|x| x.is_finite()).sum();
            (sj.is_finite().then_some(sj), Some(vol))
        }
        Err(_) => (None, None),
    }
}

impl TypedOp for Refine {
    const NAME: &'static str = "mesh.refine";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let source = ctx.resolve_kind(self.mesh, EntityKind::Mesh)?;
        let levels = self.levels.unwrap_or(1);
        if !(1..=MAX_LEVELS).contains(&levels) {
            return Err(OpError::Invalid(format!("levels must be 1..={MAX_LEVELS}, got {levels}")));
        }
        let keep = self.keep_source != Some(false);
        let (mut cur, selection, plan, quanta, stale) = {
            let layer = ctx.layer::<MeshLayer>()?;
            let e = layer.get(source).ok_or(OpError::UnknownEntity(source))?;
            (e.mesh.clone(), e.selection.clone(), e.plan.clone(), e.quanta, e.stale)
        };
        let model = ctx.layer::<ModelLayer>()?.clone();
        let topo = Some(model.topo());
        let project = self.project != Some(false);
        let mut prev = source;
        for level in 1..=levels {
            ctx.step(u64::from(level - 1), u64::from(levels), "refining")?;
            let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();
            let mut alloc: SerialAlloc = alloc0.clone();
            let src = cur.as_read();
            let builder = if src.dim() == MeshDim::Two { MeshBuilder::surface() } else { MeshBuilder::new() };
            let r = refine(src, topo, project, builder, &mut alloc).map_err(refuse)?;
            let map = LevelMap::new(&r.finished.mesh, src, r.coarse_of, Some(r.face_map))
                .map_err(|e| OpError::Failed(e.to_string()))?;
            let (sj0, vol0) = quality(src);
            let (sj1, vol1) = quality(&r.finished.mesh);
            let new = r.finished.mesh.mesh_id();
            let mut entry = MeshEntry::new(Box::new(r.finished.mesh), selection.clone(), None, quanta);
            entry.plan = plan.clone();
            entry.stale = stale;
            entry.derived_from = Some(prev);
            insert_mesh(ctx, entry, &alloc0, alloc, "mesh")?;
            if keep || level > 1 {
                ctx.layer_mut::<MeshLayer>()?.link(map).map_err(|e| OpError::Failed(e.to_string()))?;
            }
            let s = r.stats;
            let report = json!({
                "level": level, "mesh": new.to_string(), "parent": prev.to_string(),
                "parent_cells": s.parents, "cells": s.children, "vertices": s.vertices,
                "projected": s.projected, "projection_failed": s.projection_failed,
                "projection_rejected": s.projection_rejected,
                "volume_parent": vol0, "volume": vol1,
                "min_scaled_jacobian_parent": sj0, "min_scaled_jacobian": sj1,
            });
            ctx.info("refine.report", report.to_string(), &[new]);
            if s.projection_failed + s.projection_rejected > 0 {
                ctx.warn(
                    "refine.projection",
                    format!(
                        "level {level}: {} new boundary node(s) kept their straight position ({} where the projection failed, {} where it would have inverted a cell)",
                        s.projection_failed + s.projection_rejected,
                        s.projection_failed,
                        s.projection_rejected
                    ),
                    &[new],
                );
            }
            if let (Some(a), Some(b)) = (sj0, sj1)
                && a > 0.0
                && b < QUALITY_DROP_FACTOR * a
            {
                ctx.warn(
                    "refine.quality",
                    format!(
                        "level {level}: the minimum scaled Jacobian fell from {a:.4} to {b:.4} (below {QUALITY_DROP_FACTOR} of the parent's); \
                         cells hugging a curved boundary flatten as the boundary polygon refines, and smoothing cannot repair that"
                    ),
                    &[new],
                );
            }
            for (code, msg) in r.notes {
                ctx.warn(code, msg, &[source]);
            }
            if level == 1 && !keep {
                ctx.record_successors(source, &[new])?;
            } else {
                ctx.record_successors(prev, &[prev, new])?;
            }
            cur = ctx.layer::<MeshLayer>()?.get(new).ok_or(OpError::UnknownEntity(new))?.mesh.clone();
            prev = new;
        }
        if !keep {
            retire_mesh(ctx, source)?;
        }
        Ok(())
    }
}
