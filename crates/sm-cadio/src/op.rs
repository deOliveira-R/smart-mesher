//! `model.import_step`: the content-addressed STEP import op.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{ModelLayer, TopoHost, commit_topology, model_txn, topo_txn};
use sm_model_api::Domain;
use sm_op_api::{ApplyCtx, Cardinality, OpError, OutputRole, QueryCtx, State, TypedOp, TypedQuery};
use sm_topo_api::{Severity, TopologyView, check_cells, validate_geometry};

use crate::attrs::read_attributes;
use crate::decode::read_step;
use crate::inputs::StepInputs;
use crate::model::{BrepModel, LengthUnit, NoteCode, NoteSeverity, ReadOptions, SolidKind};
use crate::part21;
use crate::realize::{RealizeOptions, Realized, realize};
use crate::source::{Source, load};
use crate::write::{StepSchema, WriteOptions, WriteReport, export_step};

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
/// - `attributes` (default `false`): also import STEP names and colors into the model tables
///   (see below). Absent or `false`, the op is exactly as before (same state, same hash).
///
/// **Attributes.** One model group per distinct solid name (the solid's `name`, else its
/// `PRODUCT` name), holding the volumes of the solids of that name (the body for a sheet).
/// One volume color (`Domain::Volume`) per distinct color name (`COLOUR_RGB` name, else
/// `rgb_rrggbb`; a name that exists as a volume color is reused), applied to the volumes of the
/// solids that carry it. Colors come only from a `STYLED_ITEM` attached to the solid itself;
/// face-only colors are ignored and counted (`step.attr_ignored`). Name clashes with existing
/// groups, colors of another domain and one name with two RGB values are skipped with a
/// `step.attr_skipped` warning. Reported by `step.attr_groups` and `step.attr_colors`.
///
/// Diagnostic codes: `step.units`, `step.inverted_solid`, `step.welded`, `step.sheets_skipped`,
/// `step.unsupported_entity`, `step.validate`, `step.edge_gap`, `step.edge_sense`,
/// `step.attributes`, `step.assembly_transforms_ignored`, `step.assembly_flattened`,
/// `step.split_closed_face`, and with `attributes`: `step.attr_groups`, `step.attr_colors`,
/// `step.attr_ignored`, `step.attr_skipped`.
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
    /// Import STEP names as groups and solid colors as volume colors (default `false`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub attributes: Option<bool>,
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
        let check = self.check.unwrap_or(true);
        // A closed NURBS face the splitter could not handle has wrong area/volume integrals that
        // both validators miss: refuse it under `check`.
        if check && let Some(n) = model.notes.iter().find(|n| n.code == NoteCode::SplitClosedFace && n.severity == NoteSeverity::Warning) {
            return Err(OpError::Invalid(format!("step.split_closed_face: {} (pass check: false to import it anyway)", n.message)));
        }
        let attrs = self.attributes.unwrap_or(false);
        for n in &model.notes {
            if attrs && n.code == NoteCode::Attributes {
                let mut n = n.clone();
                n.message = format!("{} (colors on solids and names were imported, see step.attr_*)", n.message);
                emit(ctx, &n, &label, &[]);
                continue;
            }
            emit(ctx, n, &label, &[]);
        }
        for s in &model.solids {
            if !s.units.declared {
                ctx.warn("step.units", format!("{label}solid #{}: the file declares no length unit; assuming {}", s.id, self.assume_unit.unwrap_or(LengthUnit::Mm).symbol()), &[]);
            }
        }
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
        if attrs {
            import_attributes(ctx, &bytes, &model, &realized, &label)?;
        }
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

/// Names to groups and solid colors to volume colors, in one model-only transaction.
fn import_attributes(ctx: &mut ApplyCtx<'_>, bytes: &[u8], model: &BrepModel, realized: &Realized, label: &str) -> Result<(), OpError> {
    let file = part21::parse(bytes)?;
    let solids: Vec<(u64, String)> = model.solids.iter().map(|s| (s.id, s.name.clone())).collect();
    let attrs = read_attributes(&file, &solids);
    // The cell standing for each solid: its volume, or its body for a sheet.
    let mut members: Vec<(EntityId, bool)> = Vec::with_capacity(model.solids.len());
    let mut next_volume = 0usize;
    for (k, s) in model.solids.iter().enumerate() {
        if s.kind == SolidKind::Sheet {
            members.push((realized.bodies[k], false));
        } else {
            members.push((realized.volumes[next_volume], true));
            next_volume += 1;
        }
    }
    let mut groups: BTreeMap<String, Vec<EntityId>> = BTreeMap::new();
    let mut colors: BTreeMap<String, ([f64; 3], Vec<EntityId>)> = BTreeMap::new();
    let mut skipped: Vec<String> = Vec::new();
    for (s, (cell, is_volume)) in model.solids.iter().zip(&members) {
        if let Some(n) = attrs.names.get(&s.id) {
            groups.entry(n.clone()).or_default().push(*cell);
        }
        if let Some(c) = attrs.colors.get(&s.id) {
            if !*is_volume {
                skipped.push(format!("solid #{}: a sheet has no volume to color", s.id));
                continue;
            }
            let name = c.model_name();
            let slot = colors.entry(name.clone()).or_insert_with(|| (c.rgb, Vec::new()));
            if slot.0 != c.rgb {
                skipped.push(format!("solid #{}: color name {name:?} is used with two RGB values; the first is kept", s.id));
            }
            slot.1.push(*cell);
        }
    }
    let invalid = |e: sm_model_api::ModelError| OpError::Invalid(format!("step.attributes: {e}"));
    let (made_groups, made_colors) = model_txn(
        ctx,
        |m, _topo| {
            let mut made_groups = Vec::new();
            for (name, cells) in &groups {
                if m.group_by_name(name).is_some() {
                    skipped.push(format!("group {name:?} already exists; not created"));
                    continue;
                }
                m.add_group(name, cells).map_err(invalid)?;
                made_groups.push((name.clone(), cells.len()));
            }
            let mut made_colors = Vec::new();
            for (name, (_, vols)) in &colors {
                let id = match m.color_by_name(name) {
                    Some(id) if m.color(id).is_some_and(|c| c.domain == Domain::Volume) => id,
                    Some(_) => {
                        skipped.push(format!("color {name:?} exists in another domain; not applied"));
                        continue;
                    }
                    None => {
                        let label = m.colors().filter(|c| c.domain == Domain::Volume).map(|c| c.label).max().map_or(1, |l| l.saturating_add(1));
                        m.add_color(label, name, Domain::Volume).map_err(invalid)?
                    }
                };
                m.color_volumes(id, vols).map_err(invalid)?;
                made_colors.push((name.clone(), vols.len()));
            }
            Ok((made_groups, made_colors))
        },
        |_, _| Vec::new(),
    )?;
    let pre = if label.is_empty() { String::new() } else { format!("{label}: ") };
    let list = |v: &[(String, usize)]| v.iter().map(|(n, c)| format!("{n} ({c})")).collect::<Vec<_>>().join(", ");
    ctx.info("step.attr_groups", format!("{pre}{} group(s) from STEP names: {}", made_groups.len(), list(&made_groups)), &[]);
    ctx.info("step.attr_colors", format!("{pre}{} volume color(s) from solid-attached STEP colors: {}", made_colors.len(), list(&made_colors)), &[]);
    if attrs.face_only > 0 || attrs.other > 0 {
        ctx.info("step.attr_ignored", format!("{pre}ignored {} face-only color(s) and {} other styled item(s); only colors attached to a solid are imported", attrs.face_only, attrs.other), &[]);
    }
    for m in skipped {
        ctx.warn("step.attr_skipped", format!("{pre}{m}"), &[]);
    }
    Ok(())
}

/// STEP application protocol of `model.export`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum ExportSchema {
    /// AP214 (default).
    #[default]
    Ap214,
    /// AP242.
    Ap242,
}

/// `model.export {bodies?, schema?, path?}`: the model's bodies as STEP text.
///
/// A read-only command, not a timeline op: it adds no timeline node and leaves the state hash
/// unchanged, so replay never writes a file. The reply carries the bytes (base64) and a
/// [`WriteReport`] (entities written, and what STEP cannot hold, located by cell). Output is
/// byte-identical for equal input: fixed header time stamp, entities numbered in model-id
/// order.
///
/// - `bodies`: bodies (standing for their volumes) or volumes to write; default every body
///   (every volume when the model has no bodies). An empty list, a dead id or an id of another
///   kind is refused, naming the entity.
/// - `schema`: `ap214` (default) or `ap242`.
/// - `path`: accepted so the input schema is stable; this library ignores it. The server host
///   writes the bytes to a session path outside history.
///
/// Errors: `unknown-entity`, `wrong-kind`, or `invalid-params` with a `step.*` code
/// (`step.unsupported` names the cell whose curve or surface kind STEP export cannot hold).
#[derive(Clone, Debug, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExportStepQuery {
    /// Bodies or volumes to write (default all).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoBody", "TopoVolume"]))]
    pub bodies: Option<Vec<EntityId>>,
    /// STEP application protocol (default `ap214`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub schema: Option<ExportSchema>,
    /// Session path to write to (honoured by the server host, ignored here).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
}

/// Reply of `model.export`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ExportStepReply {
    /// The STEP file, base64 (standard alphabet, padded).
    pub step: String,
    /// What was written and what was lost.
    pub report: WriteReport,
}

impl TypedQuery for ExportStepQuery {
    const NAME: &'static str = "model.export";
    type Output = ExportStepReply;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<ExportStepReply, OpError> {
        use base64::Engine as _;
        let layer = state.layer::<ModelLayer>().ok_or_else(|| OpError::MissingLayer(<ModelLayer as sm_op_api::LayerType>::NAME.to_owned()))?;
        if let Some(list) = &self.bodies {
            for &id in list {
                if layer.topo().is_alive(id) && !matches!(id.kind(), EntityKind::TopoBody | EntityKind::TopoVolume) {
                    return Err(OpError::WrongKind { id, expected: EntityKind::TopoBody });
                }
            }
        }
        let schema = match self.schema.unwrap_or_default() {
            ExportSchema::Ap214 => StepSchema::Ap214,
            ExportSchema::Ap242 => StepSchema::Ap242,
        };
        let opts = WriteOptions { schema, ..WriteOptions::default() };
        let mut buf = Vec::new();
        let report = export_step(state, self.bodies.as_deref(), &opts, &mut buf)?;
        Ok(ExportStepReply { step: base64::engine::general_purpose::STANDARD.encode(&buf), report })
    }
}
