//! Ops `mesh.htet` and `mesh.qtri`: hex, wedge and pyramid to tetrahedra with polyhedral closure,
//! and quads to triangles, into new linked meshes.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshEntry, MeshLayer, ModelLayer, insert_mesh, retire_mesh};
use sm_mesh::MeshBuilder;
use sm_mesh_api::{CatalogEdit, LevelMap, MeshDim, MeshRead, SerialAlloc, SetMembers};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};

use crate::deck;
use crate::htet::{Closure, Converted, HtetError, HtetOptions, QtriMode, htet, qtri};
use crate::ops::quality;

/// What `mesh.htet` does with an unselected cell that shares a quad with a selected one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum ClosureMode {
    /// The neighbour gains the two triangles of the cut quad and becomes a polyhedron.
    Polyhedral,
    /// Refuse unless the selection is closed (no unselected cell touches a selected one by a quad).
    Whole,
}

/// How `mesh.qtri` splits a quad.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum QtriSplit {
    /// Two triangles along the diagonal through the lowest-ranked corner.
    Two,
    /// Four triangles around a new centroid vertex.
    Four,
}

/// `mesh.htet`: split hexes, wedges and pyramids into tetrahedra, conformally and without any
/// propagation: every quad is cut along the diagonal through its lowest-ranked corner, so
/// neighbouring cells agree by construction (Dompierre, Labbe, Vallet, Camarero 1999). A hex gives
/// 5 or 6 tets, a wedge 3, a pyramid 2; vertices keep their indices and no point is added.
///
/// Deck pairs (translation and rotation decks of the model) rank the two partner vertices alike,
/// so a paired face is cut identically on both sides whatever the vertex numbering; the pairs are
/// re-verified on the result (`htet.deck_mismatch` warning when a partial selection breaks one).
///
/// `cell_set` names a cell set of the mesh (a block, or a `mesh.flag_cells` group) to convert;
/// without it every hex, wedge and pyramid is converted. With `closure: "polyhedral"` (the
/// default) an unselected neighbour of a cut quad gains the two triangles and becomes a polyhedron
/// (no transition pyramids); `"whole"` refuses with the neighbour's index unless the selection is
/// closed. Tets pass through; polyhedra cannot be converted and keep their shape.
///
/// The result is a new mesh entry linked to the source by a `LevelMap` with a face map.
/// Classification, sets, blocks and cell tags carry over as in `mesh.refine`. A cell whose cut
/// would give a tet of zero or negative volume (a folded face) is refused with `htet.inverted_tet`
/// naming it. The `htet.report` diagnostic carries counts, volumes and the minimum scaled Jacobian
/// of source and result (a cube cut in tets has a poor scaled Jacobian: smooth afterwards).
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Htet {
    /// The mesh to convert.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// Convert only the cells of this cell set (default: all hexes, wedges and pyramids).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cell_set: Option<String>,
    /// What to do with unselected neighbours: `polyhedral` (default) or `whole`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub closure: Option<ClosureMode>,
    /// Keep the source mesh entry (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep_source: Option<bool>,
}

/// `mesh.qtri`: split the quads of a surface mesh into triangles, either 2 per quad along the
/// diagonal through the lowest-indexed corner (`mode: "two"`, default; deck pairs are not considered on surface meshes) or
/// 4 per quad around a new centroid vertex (`mode: "four"`, each triangle has A/4 of a
/// parallelogram). Triangles pass through. `cell_set` names the quads to split (default all).
///
/// The result is a new mesh entry linked to the source by a `LevelMap` with a face map;
/// classification, sets and tags carry over as in `mesh.refine` (the centroid is classified on the
/// cell). The `qtri.report` diagnostic carries counts, areas and the minimum scaled Jacobian.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Qtri {
    /// The surface mesh to convert.
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: EntityId,
    /// `two` (default) or `four`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mode: Option<QtriSplit>,
    /// Convert only the quads of this cell set (default: all quads).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cell_set: Option<String>,
    /// Keep the source mesh entry (default true).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub keep_source: Option<bool>,
}

const OUTPUTS: &[OutputRole] = &[OutputRole::new("mesh", &[EntityKind::Mesh], Cardinality::Many, "the converted mesh")];

fn refuse(e: HtetError) -> OpError {
    match e {
        HtetError::HighOrder => OpError::Unsupported(e.to_string()),
        HtetError::Mesh(_) | HtetError::Inconsistent(_) => OpError::Failed(e.to_string()),
        e => OpError::Invalid(e.to_string()),
    }
}

/// Mask of the cells of the named set.
fn select(src: &dyn MeshRead, name: Option<&str>) -> Result<Option<Vec<bool>>, OpError> {
    let Some(name) = name else { return Ok(None) };
    let set = src.sets().iter().find(|s| s.name == name).ok_or_else(|| {
        let names: Vec<&str> = src.sets().iter().filter(|s| matches!(s.members, SetMembers::Cells(_))).map(|s| s.name.as_str()).collect();
        OpError::Invalid(format!("the mesh has no set `{name}` (cell sets: {names:?})"))
    })?;
    let SetMembers::Cells(cells) = &set.members else {
        return Err(OpError::Invalid(format!("set `{name}` is not a cell set")));
    };
    let mut mask = vec![false; src.cell_count()];
    for c in cells {
        mask[c.get()] = true;
    }
    Ok(Some(mask))
}

struct Plan<'a> {
    mesh: EntityId,
    cell_set: Option<&'a str>,
    keep: bool,
    htet: Option<Closure>,
    qtri: QtriMode,
    report: &'static str,
}

fn drive(ctx: &mut ApplyCtx<'_>, p: Plan<'_>) -> Result<(), OpError> {
    let source = ctx.resolve_kind(p.mesh, EntityKind::Mesh)?;
    let (cur, selection, plan, quanta, stale) = {
        let layer = ctx.layer::<MeshLayer>()?;
        let e = layer.get(source).ok_or(OpError::UnknownEntity(source))?;
        (e.mesh.clone(), e.selection.clone(), e.plan.clone(), e.quanta, e.stale)
    };
    let model: ModelLayer = ctx.layer::<ModelLayer>()?.clone();
    let src = cur.as_read();
    let want = if p.htet.is_some() { MeshDim::Three } else { MeshDim::Two };
    if src.dim() != want {
        let e = if p.htet.is_some() { "mesh.htet needs a volume mesh (use mesh.qtri for surface meshes)" } else { "mesh.qtri needs a surface mesh (use mesh.htet for volumes)" };
        return Err(OpError::Invalid(e.into()));
    }
    let mask = select(src, p.cell_set)?;
    // Decks are read on volume meshes only (the deck pass is tested in 3D).
    let decks = if p.htet.is_some() { deck::decks(&model, src) } else { deck::none(src) };
    let alloc0 = ctx.layer::<MeshLayer>()?.alloc().clone();
    let mut alloc: SerialAlloc = alloc0.clone();
    let builder = if src.dim() == MeshDim::Two { MeshBuilder::surface() } else { MeshBuilder::new() };
    let r: Converted<sm_mesh::Mesh> = match p.htet {
        Some(closure) => htet(src, &HtetOptions { select: mask.as_deref(), root: Some(&decks.root), closure }, builder, &mut alloc),
        None => qtri(src, p.qtri, mask.as_deref(), Some(&decks.root), builder, &mut alloc),
    }
    .map_err(refuse)?;
    let map = LevelMap::new(&r.finished.mesh, src, r.coarse_of.clone(), Some(r.face_map.clone())).map_err(|e| OpError::Failed(e.to_string()))?;
    let (sj0, vol0) = quality(src);
    let (sj1, vol1) = quality(&r.finished.mesh);
    let bad_decks = deck::verify(&model, &r.finished.mesh, &decks);
    let new = r.finished.mesh.mesh_id();
    let s = r.stats;
    let notes = r.notes.clone();
    let mut entry = MeshEntry::new(Box::new(r.finished.mesh), selection, None, quanta);
    entry.plan = plan;
    entry.stale = stale;
    entry.derived_from = Some(source);
    insert_mesh(ctx, entry, &alloc0, alloc, "mesh")?;
    if p.keep {
        ctx.layer_mut::<MeshLayer>()?.link(map).map_err(|e| OpError::Failed(e.to_string()))?;
    }
    let report = json!({
        "mesh": new.to_string(), "parent": source.to_string(),
        "parent_cells": s.parents, "cells": s.children, "cut_cells": s.cut_cells,
        "hex_5_tets": s.hex5, "hex_6_tets": s.hex6, "cut_quads": s.cut_quads, "polyhedra": s.polyhedra,
        "deck_pairs": decks.pairs.len(), "deck_mismatch": bad_decks.iter().map(|(_, n)| n).sum::<usize>(),
        "volume_parent": vol0, "volume": vol1,
        "min_scaled_jacobian_parent": sj0, "min_scaled_jacobian": sj1,
    });
    ctx.info(p.report, report.to_string(), &[new]);
    for (name, n) in &bad_decks {
        let (code, what) = if p.htet.is_some() { ("htet.deck_mismatch", "htet") } else { ("qtri.deck_mismatch", "qtri") };
        ctx.warn(
            code,
            format!("deck `{name}`: {n} face(s) of one side are not the image of a face of the other after {what} (a selection that cuts one side of a pair only)"),
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
    if p.keep {
        ctx.record_successors(source, &[source, new])?;
    } else {
        ctx.record_successors(source, &[new])?;
        retire_mesh(ctx, source)?;
    }
    Ok(())
}

impl TypedOp for Htet {
    const NAME: &'static str = "mesh.htet";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let closure = if self.closure == Some(ClosureMode::Whole) { Closure::Whole } else { Closure::Polyhedral };
        drive(
            ctx,
            Plan {
                mesh: self.mesh,
                cell_set: self.cell_set.as_deref(),
                keep: self.keep_source != Some(false),
                htet: Some(closure),
                qtri: QtriMode::Two,
                report: "htet.report",
            },
        )
    }
}

impl TypedOp for Qtri {
    const NAME: &'static str = "mesh.qtri";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let mode = if self.mode == Some(QtriSplit::Four) { QtriMode::Four } else { QtriMode::Two };
        drive(
            ctx,
            Plan { mesh: self.mesh, cell_set: self.cell_set.as_deref(), keep: self.keep_source != Some(false), htet: None, qtri: mode, report: "qtri.report" },
        )
    }
}
