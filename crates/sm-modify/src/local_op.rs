//! Op `mesh.refine_local`: local refinement with polyhedral hanging faces.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshEntry, MeshLayer, ModelLayer, insert_mesh, retire_mesh};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{CatalogEdit, CellIx, CellShape, LevelMap, MeshDim, MeshEntityKind, MeshRead, SerialAlloc, SetMembers, TagValues};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_quality::{Metric, Options, compute_many};

use crate::local::{LocalError, LocalOptions, MAX_LOCAL_LEVELS, refine_local};
use crate::ops::quality;

/// `mesh.refine_local`: split the selected hexes of a volume mesh into 8 (or 64, 512 with
/// `levels`) children and keep the mesh conformal without transition elements: a neighbour that
/// meets a refined cell gains the split sub-faces of the shared face and the hanging edge nodes
/// on its other faces, and becomes a polyhedron (the OpenFOAM `hexRef8` idea, native here because
/// the cell is a polyhedron). The 3x3x3 block with its centre hex refined gives 8 children, 6 face
/// neighbours with 9 faces each (4 of their side faces become 5-vertex polygons), 12 edge
/// neighbours with 6 faces (2 of them 5-vertex), 8 untouched corner hexes and `64 + 19` vertices.
///
/// Selection: `cell_set` names a cell set (a block or a `mesh.flag_cells` group), or `tag` names
/// a cell tag column and `threshold` selects the cells whose value is at least it (exactly one of
/// the two ways). Selected cells are refined `levels` times (default 1, at most 3) in this one
/// call; the result is one new mesh entry linked to the source by a `LevelMap` with a face map.
/// With `balance` (default true) the level jump across any face is at most 1: the neighbours of a
/// 2-level cell are raised to level 1. With `balance: false` a jump above 1 is refused, naming the
/// face. Only hexes are refined; any other cell that the selection or the balance would refine is
/// refused. Unrefined cells of any shape that touch a split face or edge become polyhedra.
/// Balance is per call: a second call cannot select polyhedra, so it can leave a jump above 1.
///
/// Classification, sets, blocks and cell tags carry over as in `mesh.refine` (a vertex set gains
/// a new node when it lies on a boundary face of the mesh and all the source vertices it spans are
/// members). New nodes on a model edge
/// or face are projected unless that would invert a hex (`project: false` keeps them straight; then
/// every face stays planar and the volume is exact). The `refine_local.report` diagnostic carries
/// counts (hexes, polyhedra, balance-raised cells), volumes, the minimum scaled Jacobian and the
/// maximum edge ratio over the hexes (and, separately, over the polyhedra, whose hanging half
/// edges count) of source and result.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RefineLocal {
    /// The volume mesh to refine.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Refine the cells of this cell set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cell_set: Option<String>,
    /// Refine the cells whose value of this cell tag column is at least `threshold`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tag: Option<String>,
    /// The tag threshold (with `tag`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub threshold: Option<f64>,
    /// How many times to split a selected cell (default 1, at most 3).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub levels: Option<u32>,
    /// Raise neighbours so no face has a level jump above 1 (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub balance: Option<bool>,
    /// Keep the source mesh entry (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep_source: Option<bool>,
    /// Project new boundary nodes onto the model geometry (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub project: Option<bool>,
}

fn refuse(e: LocalError) -> OpError {
    match e {
        LocalError::HighOrder => OpError::Unsupported(e.to_string()),
        LocalError::Mesh(_) | LocalError::Inconsistent(_) | LocalError::EdgeClass(_) | LocalError::TooLarge => OpError::Failed(e.to_string()),
        e => OpError::Invalid(e.to_string()),
    }
}

fn select(src: &dyn MeshRead, op: &RefineLocal) -> Result<Vec<bool>, OpError> {
    let mut mask = vec![false; src.cell_count()];
    if op.cell_set.is_some() && op.threshold.is_some() {
        return Err(OpError::Invalid("`threshold` goes with `tag`; it cannot be combined with `cell_set`".into()));
    }
    match (&op.cell_set, &op.tag) {
        (Some(name), None) => {
            let set = src.sets().iter().find(|s| s.name == *name).ok_or_else(|| {
                let names: Vec<&str> = src.sets().iter().filter(|s| matches!(s.members, SetMembers::Cells(_))).map(|s| s.name.as_str()).collect();
                OpError::Invalid(format!("the mesh has no set `{name}` (cell sets: {names:?})"))
            })?;
            let SetMembers::Cells(cells) = &set.members else {
                return Err(OpError::Invalid(format!("set `{name}` is not a cell set")));
            };
            for c in cells {
                mask[c.get()] = true;
            }
        }
        (None, Some(name)) => {
            let th = op.threshold.ok_or_else(|| OpError::Invalid("`tag` needs a `threshold`".into()))?;
            let t = src
                .tags()
                .iter()
                .find(|t| t.name == *name && t.target == MeshEntityKind::Cell)
                .ok_or_else(|| OpError::Invalid(format!("the mesh has no cell tag `{name}`")))?;
            match &t.values {
                TagValues::F64(v) => v.iter().zip(&mut mask).for_each(|(x, m)| *m = *x >= th),
                TagValues::I64(v) => v.iter().zip(&mut mask).for_each(|(x, m)| *m = *x as f64 >= th),
                _ => return Err(OpError::Invalid(format!("tag `{name}` is neither integer nor real"))),
            }
        }
        _ => return Err(OpError::Invalid("give exactly one of `cell_set` or `tag` (with `threshold`)".into())),
    }
    Ok(mask)
}

/// Largest finite edge ratio over the hexes and over the polyhedra of a mesh (a polyhedron's
/// hanging half edges count as edges, so its ratio is larger by design), `None` where it fails.
fn max_edge_ratio(m: &dyn MeshRead) -> (Option<f64>, Option<f64>) {
    let Ok(t) = compute_many(m, &[Metric::EdgeRatio], &Options::default()) else { return (None, None) };
    let mut out = [f64::NEG_INFINITY; 2];
    for (c, x) in t.columns[0].iter().copied().enumerate() {
        if x.is_finite() {
            let k = usize::from(m.cell_shape(CellIx::new(c)) == CellShape::Polyhedron);
            out[k] = out[k].max(x);
        }
    }
    (out[0].is_finite().then_some(out[0]), out[1].is_finite().then_some(out[1]))
}

impl TypedOp for RefineLocal {
    const NAME: &'static str = "mesh.refine_local";
    const OUTPUTS: &'static [OutputRole] = &[OutputRole::new("mesh", &[EntityKind::Mesh], Cardinality::Many, "the locally refined mesh")];

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let levels = self.levels.unwrap_or(1);
        if !(1..=MAX_LOCAL_LEVELS).contains(&levels) {
            return Err(OpError::Invalid(format!("levels must be 1..={MAX_LOCAL_LEVELS}, got {levels}")));
        }
        let keep = self.keep_source != Some(false);
        let source = ctx.resolve_kind(self.mesh, EntityKind::Mesh)?;
        let (cur, selection, plan, quanta, stale) = {
            let layer = ctx.layer::<MeshLayer>()?;
            let e = layer.get(source).ok_or(OpError::UnknownEntity(source))?;
            (e.mesh.clone(), e.selection.clone(), e.plan.clone(), e.quanta, e.stale)
        };
        let model = ctx.layer::<ModelLayer>()?.clone();
        let src = cur.as_read();
        if src.dim() != MeshDim::Three {
            return Err(OpError::Invalid("mesh.refine_local needs a volume mesh".into()));
        }
        let mask = select(src, self)?;
        let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();
        let mut alloc: SerialAlloc = alloc0.clone();
        let opts = LocalOptions { selected: &mask, levels, balance: self.balance != Some(false), project: self.project != Some(false) };
        let r = refine_local(src, Some(model.topo()), &opts, MeshBuilder::new(), &mut alloc).map_err(refuse)?;
        let map = LevelMap::new(&r.finished.mesh, src, r.coarse_of.clone(), Some(r.face_map.clone())).map_err(|e| OpError::Failed(e.to_string()))?;
        let (sj0, vol0) = quality(src);
        let (sj1, vol1) = quality(&r.finished.mesh);
        let ((er0, _), (er1, er1p)) = (max_edge_ratio(src), max_edge_ratio(&r.finished.mesh));
        let new = r.finished.mesh.mesh_id();
        let s = r.stats;
        let notes = r.notes.clone();
        let mut entry = MeshEntry::new(Box::new(r.finished.mesh), selection, None, quanta);
        entry.plan = plan;
        entry.stale = stale;
        entry.derived_from = Some(source);
        insert_mesh(ctx, entry, &alloc0, alloc, "mesh")?;
        if keep {
            ctx.layer_mut::<MeshLayer>()?.link(map).map_err(|e| OpError::Failed(e.to_string()))?;
        }
        let report = json!({
            "mesh": new.to_string(), "parent": source.to_string(), "levels": levels,
            "parent_cells": s.parents, "cells": s.children, "vertices": s.vertices,
            "refined": s.refined, "balanced": s.balanced, "hexes": s.hexes, "polyhedra": s.polyhedra,
            "projected": s.projected, "projection_failed": s.projection_failed, "projection_rejected": s.projection_rejected,
            "volume_parent": vol0, "volume": vol1,
            "min_scaled_jacobian_parent": sj0, "min_scaled_jacobian": sj1,
            "max_edge_ratio_parent": er0, "max_edge_ratio": er1, "max_edge_ratio_polyhedra": er1p,
        });
        ctx.info("refine_local.report", report.to_string(), &[new]);
        if s.projection_failed + s.projection_rejected > 0 {
            ctx.warn(
                "refine_local.projection",
                format!(
                    "{} new boundary node(s) kept their straight position ({} where the projection failed, {} where it would have inverted a cell)",
                    s.projection_failed + s.projection_rejected,
                    s.projection_failed,
                    s.projection_rejected
                ),
                &[new],
            );
        }
        for (code, msg) in notes {
            if code == "refine.set_ambiguous" {
                ctx.info(code, msg, &[source]);
            } else {
                ctx.warn(code, msg, &[source]);
            }
        }
        if keep {
            ctx.record_successors(source, &[source, new])?;
        } else {
            ctx.record_successors(source, &[new])?;
            retire_mesh(ctx, source)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sm_mesh_api::reference::hex_block_with_edges;
    use sm_mesh_api::MeshEdit;

    #[test]
    fn tag_threshold_selects_the_cells_at_or_above_it_for_real_and_integer_columns() {
        let mut m = hex_block_with_edges([2, 2, 1], [1.0; 3], &mut SerialAlloc::new()).expect("block");
        m.set_tag("w", MeshEntityKind::Cell, TagValues::F64(vec![0.5, 1.0, 1.5, 2.0])).expect("tag");
        m.set_tag("k", MeshEntityKind::Cell, TagValues::I64(vec![0, 3, 1, 3])).expect("tag");
        let op = |tag: &str, th: f64| RefineLocal {
            mesh: EntityId::new(EntityKind::Mesh, 1).expect("id"),
            cell_set: None,
            tag: Some(tag.into()),
            threshold: Some(th),
            levels: None,
            balance: None,
            keep_source: None,
            project: None,
        };
        assert_eq!(select(&m, &op("w", 1.0)).expect("w"), vec![false, true, true, true]);
        assert_eq!(select(&m, &op("k", 3.0)).expect("k"), vec![false, true, false, true]);
        assert!(select(&m, &op("nope", 1.0)).is_err());
    }
}
