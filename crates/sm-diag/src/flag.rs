//! `mesh.flag_cells`: store the offending cells of a mesh as a named cell set.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshLayer, add_mesh_set};
use sm_mesh_api::{CellIx, MeshRead, SetMembers};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_quality::{CellStatus, FvEntity, Measure, Options, Profile, Reduce, face_to_cell, validity};

use crate::analysis::{G_FAIL, G_UNDEF, G_WARN, compute_columns, grade_ranks, resolve};
use crate::common::resolve_mesh;
use crate::error::DiagError;
use crate::quality::{ProfileName, parse_metrics};

/// Which grade flags a cell.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum Level {
    /// Only cells graded `fail`.
    #[default]
    Fail,
    /// Cells graded `warn` or `fail`.
    Warn,
}

/// Cells of `m` to flag, ascending: any selected metric grades at or above `level` (face metrics
/// reach their cells through [`face_to_cell`]), or the cell is invalid when `include_invalid`.
/// Metrics without a rule in the profile are not graded and flag nothing; metrics no cell
/// supports are skipped.
pub fn flagged_cells(
    m: &dyn MeshRead,
    profile: Profile,
    metrics: Option<&[Measure]>,
    level: Level,
    include_invalid: bool,
) -> Result<Vec<CellIx>, DiagError> {
    let min_rank = if level == Level::Warn { G_WARN } else { G_FAIL };
    let items: Vec<_> = resolve(profile, metrics, &BTreeMap::new())?.into_iter().filter(|(_, r)| r.is_some()).collect();
    let measures: Vec<Measure> = items.iter().map(|(x, _)| *x).collect();
    let (cols, _skipped) = compute_columns(m, &measures)?;
    let mut flagged = vec![false; m.cell_count()];
    for col in &cols {
        let Some(rule) = items.iter().find(|(x, _)| *x == col.measure).and_then(|(_, r)| r.as_ref()) else { continue };
        let ranks = grade_ranks(m, col, rule);
        match col.entity {
            FvEntity::Cell => {
                for (f, &r) in flagged.iter_mut().zip(&ranks) {
                    *f |= r != G_UNDEF && r >= min_rank;
                }
            }
            _ => {
                let as_f64: Vec<f64> = ranks.iter().map(|&r| if r == G_UNDEF { f64::NAN } else { f64::from(r) }).collect();
                for (f, &r) in flagged.iter_mut().zip(&face_to_cell(m, &as_f64, Reduce::Max)) {
                    *f |= r.is_finite() && r >= f64::from(min_rank);
                }
            }
        }
    }
    if include_invalid {
        let v = validity(m, &Options::default())?;
        for (f, s) in flagged.iter_mut().zip(&v.status) {
            *f |= matches!(s, CellStatus::Inverted | CellStatus::Degenerate);
        }
    }
    Ok(flagged.iter().enumerate().filter(|(_, f)| **f).map(|(i, _)| CellIx::new(i)).collect())
}

fn default_true() -> bool {
    true
}

const OUTPUTS: &[OutputRole] = &[OutputRole::new("set", &[EntityKind::MeshSet], Cardinality::Optional, "the new cell set; absent when nothing was flagged")];

/// `mesh.flag_cells`: store the cells that fail (or warn) a quality profile as a cell set.
///
/// A cell is flagged if any selected metric grades at or above `level`, or if it is invalid
/// (`include_invalid`). The set is a `MeshSet` of cells named `name`, stored in the mesh so
/// later ops can target it by id. An empty selection succeeds with a `diag.nothing_to_flag`
/// warning and no set. A name already used in the mesh is an error.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshFlagCells {
    /// The mesh; defaults to the only mesh of the state.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: Option<EntityId>,
    /// Name of the set: 1 to 64 characters of `[A-Za-z0-9_.-]`, unused in the mesh.
    #[schemars(length(min = 1, max = 64), regex(pattern = r"^[A-Za-z0-9_.-]+$"))]
    pub name: String,
    /// `generic` (default), `openfoam`, `fluent` or `transport`.
    #[serde(default)]
    pub profile: ProfileName,
    /// Metric names to grade instead of the profile's rules.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub metrics: Option<Vec<String>>,
    /// `fail` (default) flags failing cells, `warn` also warning cells.
    #[serde(default)]
    pub level: Level,
    /// Also flag inverted and degenerate cells (default true).
    #[serde(default = "default_true")]
    pub include_invalid: bool,
}

fn check_name(name: &str) -> Result<(), OpError> {
    if name.is_empty() || name.len() > 64 || !name.bytes().all(|b| b.is_ascii_alphanumeric() || matches!(b, b'_' | b'.' | b'-')) {
        return Err(OpError::Invalid(format!("set name `{name}` must be 1 to 64 characters of [A-Za-z0-9_.-]")));
    }
    Ok(())
}

impl TypedOp for MeshFlagCells {
    const NAME: &'static str = "mesh.flag_cells";
    const OUTPUTS: &'static [OutputRole] = OUTPUTS;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        check_name(&self.name)?;
        let (id, host) = {
            let layer = ctx.layer::<MeshLayer>()?;
            let id = resolve_mesh(layer, self.mesh)?;
            let entry = layer.get(id).ok_or(OpError::UnknownEntity(id))?;
            (id, entry.mesh.clone())
        };
        let m = (*host).as_read();
        if m.set_by_name(&self.name).is_some() {
            return Err(OpError::Invalid(format!("{id} already has a set named `{}`", self.name)));
        }
        let metrics = parse_metrics(&self.metrics)?;
        let cells = flagged_cells(m, self.profile.into(), metrics.as_deref(), self.level, self.include_invalid)?;
        if cells.is_empty() {
            ctx.warn("diag.nothing_to_flag", format!("no cell of {id} is graded {:?} or invalid: no set `{}` was created", self.level, self.name), &[id]);
            return Ok(());
        }
        add_mesh_set(ctx, id, &self.name, SetMembers::Cells(cells), "set")?;
        Ok(())
    }
}
