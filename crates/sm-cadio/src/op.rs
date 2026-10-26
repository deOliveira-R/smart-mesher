//! `model.import_step`: the content-addressed STEP import op.

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{ModelLayer, TopoHost, commit_topology, topo_txn};
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, TypedOp};
use sm_topo_api::{Severity, TopologyView, check_cells, validate_geometry};

use crate::decode::read_step;
use crate::inputs::StepInputs;
use crate::model::{LengthUnit, NoteSeverity, ReadOptions};
use crate::realize::{RealizeOptions, realize};
use crate::source::{Source, load};

/// Output roles of the import (all `Many`, in STEP-id order).
const IMPORT_OUTPUTS: &[OutputRole] = &[
    OutputRole::new("bodies", &[EntityKind::TopoBody], Cardinality::Many, "One body per imported solid."),
    OutputRole::new("volumes", &[EntityKind::TopoVolume], Cardinality::Many, "One volume per imported solid (none for sheets)."),
    OutputRole::new("faces", &[EntityKind::TopoFace], Cardinality::Many, "The faces of the imported solids."),
    OutputRole::new("edges", &[EntityKind::TopoEdge], Cardinality::Many, "The edges (including pole edges) of the imported solids."),
    OutputRole::new("vertices", &[EntityKind::TopoVertex], Cardinality::Many, "The vertices of the imported solids."),
];

/// How many validator issues a failed `check` lists.
const MAX_LISTED_ISSUES: usize = 20;

/// `model.import_step`: read the B-rep solids of a STEP file into the model.
///
/// The op never takes a path. `source` is `{"sha256": "<64 hex>"}` (bytes from the injected
/// [`StepInputs`] service, re-hashed on read) or `{"inline": "<base64>"}` (at most 8 MiB, carried
/// in the op so replay and fork are self-contained). Both forms of one file give the same state.
///
/// - `name`: a label for the file used in diagnostics (not stored in the state).
/// - `to_unit`: convert coordinates to this unit; absent keeps the file's unit (reported in a
///   `step.units` info). The op never changes the model tolerance.
/// - `assume_unit`: the unit to use when the file declares none (default `mm`, with a warning).
/// - `weld_tol`: vertices closer than this (target units) are welded; default the model's linear
///   tolerance, at most its `linear_max`.
/// - `solids`: 0-based ordinals, in ascending STEP id, of the solids to import (default all).
/// - `sheets` (default `false`): also import `SHELL_BASED_SURFACE_MODEL`s, one body of free
///   faces each, after the solids (`solids` ordinals select among solids only). They produce no
///   volume.
/// - `check` (default `true`): run `sm_topo_api::check_cells` and `validate_geometry` on the new
///   cells; an `Error` issue fails the op (`Invalid`, first 20 issues listed) and leaves the
///   state unchanged, warnings become `step.validate` diagnostics.
///
/// Diagnostic codes: `step.units`, `step.inverted_solid`, `step.welded`, `step.sheets_skipped`,
/// `step.unsupported_entity`, `step.validate`, `step.edge_gap`, `step.edge_sense`,
/// `step.attributes`, `step.assembly_transforms_ignored`, `step.assembly_flattened`,
/// `step.split_closed_face`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ImportStep {
    /// The file: `{sha256}` or `{inline}`.
    pub source: Source,
    /// A label for the file in diagnostics.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    /// Convert to this unit (`um`, `mm`, `cm`, `m`, `in`, `ft`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub to_unit: Option<LengthUnit>,
    /// The unit to assume when the file declares none (default `mm`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub assume_unit: Option<LengthUnit>,
    /// Vertex weld tolerance in target units (default the model linear tolerance).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub weld_tol: Option<f64>,
    /// Ordinals of the solids to import (default all).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub solids: Option<Vec<u32>>,
    /// Import sheets (not supported yet).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sheets: Option<bool>,
    /// Validate the imported cells (default `true`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub check: Option<bool>,
}

fn emit(ctx: &mut ApplyCtx<'_>, n: &crate::model::Note, label: &str, ents: &[EntityId]) {
    let msg = if label.is_empty() { n.message.clone() } else { format!("{label}: {}", n.message) };
    match n.severity {
        NoteSeverity::Info => ctx.info(n.code.as_str(), msg, ents),
        NoteSeverity::Warning => ctx.warn(n.code.as_str(), msg, ents),
    }
}

impl TypedOp for ImportStep {
    const NAME: &'static str = "model.import_step";
    const OUTPUTS: &'static [OutputRole] = IMPORT_OUTPUTS;
    /// Bump when the reader changes the cells it creates for the same bytes.
    const IMPL_VERSION: u32 = 2;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let tol = ctx.layer::<ModelLayer>()?.tolerance();
        let weld_tol = match self.weld_tol {
            None => tol.linear,
            Some(w) if w.is_finite() && w > 0.0 && w <= tol.linear_max => w,
            Some(w) => return Err(OpError::Invalid(format!("weld_tol {w} must be finite, positive and at most the model's linear_max {}", tol.linear_max))),
        };
        let bytes = load(&self.source, || ctx.service::<StepInputs>())?;
        let opts = ReadOptions { to_unit: self.to_unit, assume_unit: Some(self.assume_unit.unwrap_or(LengthUnit::Mm)), weld_tol, solids: self.solids.clone(), sheets: self.sheets.unwrap_or(false), flatten_assemblies: true };
        let model = read_step(&bytes, &opts)?;
        if model.solids.is_empty() {
            return Err(OpError::Invalid("step.no_solids: the file contains no solid (MANIFOLD_SOLID_BREP, BREP_WITH_VOIDS, FACETED_BREP) to import".to_owned()));
        }
        let label = self.name.clone().unwrap_or_default();
        for n in &model.notes {
            emit(ctx, n, &label, &[]);
        }
        for s in &model.solids {
            if !s.units.declared {
                ctx.warn("step.units", format!("{label}solid #{}: the file declares no length unit; assuming {}", s.id, self.assume_unit.unwrap_or(LengthUnit::Mm).symbol()), &[]);
            }
        }
        let check = self.check.unwrap_or(true);
        let ((realized, warnings), txn) = topo_txn(ctx, |topo: &mut dyn TopoHost, _model| {
            let realized = realize(&model, topo, &RealizeOptions::default())?;
            let mut warnings = Vec::new();
            if check {
                let mut new_cells: Vec<EntityId> = realized.vertices.iter().chain(&realized.edges).chain(&realized.faces).chain(&realized.volumes).chain(&realized.bodies).copied().collect();
                new_cells.sort_unstable();
                let view: &dyn TopologyView = topo.as_view();
                let mut issues = check_cells(view, &new_cells);
                issues.extend(validate_geometry(view).into_iter().filter(|i| new_cells.binary_search(&i.cell).is_ok()));
                let errors: Vec<_> = issues.iter().filter(|i| i.severity == Severity::Error).collect();
                if !errors.is_empty() {
                    let listed: Vec<String> = errors.iter().take(MAX_LISTED_ISSUES).map(|i| format!("{} ({:?}): {}", i.cell, i.rule, i.message)).collect();
                    return Err(OpError::Invalid(format!(
                        "step.validate: the imported cells have {} validation error(s){}: {}",
                        errors.len(),
                        if errors.len() > MAX_LISTED_ISSUES { format!(" (first {MAX_LISTED_ISSUES} listed)") } else { String::new() },
                        listed.join("; ")
                    )));
                }
                warnings = issues.into_iter().filter(|i| i.severity == Severity::Warning).map(|i| (i.cell, format!("{:?}: {}", i.rule, i.message))).collect::<Vec<_>>();
            }
            Ok((realized, warnings))
        })?;
        commit_topology(
            ctx,
            &txn,
            &[("bodies", &realized.bodies), ("volumes", &realized.volumes), ("faces", &realized.faces), ("edges", &realized.edges), ("vertices", &realized.vertices)],
        )?;
        for inv in &realized.inverted {
            let ix = model.solids.iter().position(|s| s.id == inv.solid);
            let body: Vec<EntityId> = ix.and_then(|i| realized.bodies.get(i).copied()).into_iter().collect();
            ctx.warn("step.inverted_solid", format!("{label}solid #{} had inward-pointing normals in shell(s) {:?}; orientation flipped", inv.solid, inv.shells), &body);
        }
        for (cell, msg) in warnings {
            ctx.warn("step.validate", msg, &[cell]);
        }
        Ok(())
    }
}
