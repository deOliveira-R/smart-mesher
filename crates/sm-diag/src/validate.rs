//! `mesh.validate`: structural, geometric and classification findings, located and ranked.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::MeshLayer;
use sm_mesh_api::{GeomTol, MeshEntity, MeshRead, validate, validate_geometry};
use sm_op_api::{OpError, QueryCtx, State, TypedQuery};
use sm_quality::{CellStatus, Options, validity};

use crate::common::{resolve_mesh, schema_version};
use crate::locate::{EntityRef, locate, parts};

/// Largest accepted `limit`.
pub const MAX_LIMIT: usize = 10_000;

/// Severity of a violation. Errors make a mesh invalid, warnings do not.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    /// The mesh is invalid.
    Error,
    /// Worth a look; the mesh stays valid.
    Warning,
}

/// A finding before location data is attached.
#[derive(Clone, Debug, PartialEq, Eq)]
#[non_exhaustive]
pub struct Finding {
    /// Stable code (`loop.invalid`, `quality.inverted`, `diag.class_retired`, ...).
    pub code: String,
    /// Severity.
    pub severity: Severity,
    /// The offending mesh entity, if one can be named.
    pub entity: Option<MeshEntity>,
    /// Human-readable detail.
    pub detail: String,
}

impl Finding {
    /// A finding.
    pub fn new(code: impl Into<String>, severity: Severity, entity: Option<MeshEntity>, detail: impl Into<String>) -> Self {
        Self { code: code.into(), severity, entity, detail: detail.into() }
    }
}

/// Options of [`validate_mesh`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ValidateOptions {
    /// Run the geometric checks (needs a structurally sound mesh).
    pub geometry: bool,
    /// Largest distance of a face vertex from the face's best plane, relative to `sqrt(area)`.
    pub planarity: f64,
    /// Cells with volume at most this are reported.
    pub min_volume: f64,
    /// Number of violations returned in full (all are counted).
    pub limit: usize,
}

impl Default for ValidateOptions {
    fn default() -> Self {
        Self { geometry: true, planarity: 1e-6, min_volume: 0.0, limit: 100 }
    }
}

/// One violation with its location.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Violation {
    /// Stable code.
    pub code: String,
    /// `error` or `warning`.
    pub severity: Severity,
    /// The offending entity, `null` for a finding about the whole mesh.
    pub entity: Option<EntityRef>,
    /// Human-readable detail.
    pub detail: String,
    /// The classification of the entity itself.
    pub owner: Option<EntityId>,
    /// Model entities of dimension below the owner's, from the entity's vertex classifications
    /// (sorted, at most 8).
    pub near: Vec<EntityId>,
    /// Vertex point, edge midpoint, face centroid or cell centroid; `null` for an out-of-range index.
    pub location: Option<[f64; 3]>,
}

/// Number of violations owned by one model entity.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct OwnerCount {
    /// The owning classification entity.
    pub owner: EntityId,
    /// Violations of entities classified on it.
    pub count: usize,
}

/// Reply of `mesh.validate`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct ValidateReport {
    /// Wire version, 1.
    pub schema_version: u32,
    /// The mesh.
    pub mesh: EntityId,
    /// True if there is no error-severity violation.
    pub valid: bool,
    /// Whether the geometric checks ran.
    pub geometry_checked: bool,
    /// Why they did not, when `geometry_checked` is false.
    pub geometry_skipped_reason: Option<String>,
    /// All violations found, including those cut by `limit`.
    pub total: usize,
    /// Violations per code; sums to `total`.
    pub by_code: BTreeMap<String, usize>,
    /// The ten classification entities owning most violations, by count then id.
    pub by_owner: Vec<OwnerCount>,
    /// The violations sorted by severity, code, entity kind, index and detail, cut to `limit`.
    pub violations: Vec<Violation>,
    /// True if `violations` was cut.
    pub truncated: bool,
}

/// Why the geometric checks did or did not run.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GeometryStatus {
    /// They ran.
    Checked,
    /// They were skipped, for this reason.
    Skipped(String),
}

fn is_structural(code: &str) -> bool {
    code == "ids.order"
        || code == "index.range"
        || ["loop.", "face.", "cell.", "shape."].iter().any(|p| code.starts_with(p))
}

fn kind_rank(e: Option<MeshEntity>) -> (u8, u32) {
    match e {
        None => (0, 0),
        Some(e) => {
            let (k, ix) = parts(e);
            (1 + k as u8, ix)
        }
    }
}

/// Sort `findings`, locate them and assemble the report. The order of the input does not matter:
/// findings are ordered by severity, code, entity kind, index and detail.
pub fn build_report(m: &dyn MeshRead, mut findings: Vec<Finding>, o: &ValidateOptions, geometry: GeometryStatus) -> ValidateReport {
    findings.sort_by(|a, b| {
        a.severity
            .cmp(&b.severity)
            .then_with(|| a.code.cmp(&b.code))
            .then_with(|| kind_rank(a.entity).cmp(&kind_rank(b.entity)))
            .then_with(|| a.detail.cmp(&b.detail))
    });
    let total = findings.len();
    let mut by_code: BTreeMap<String, usize> = BTreeMap::new();
    let mut owners: BTreeMap<EntityId, usize> = BTreeMap::new();
    for f in &findings {
        *by_code.entry(f.code.clone()).or_default() += 1;
        if let Some((k, ix)) = f.entity.map(parts)
            && let Some(o) = crate::locate::owner(m, k, ix)
        {
            *owners.entry(o).or_default() += 1;
        }
    }
    let mut by_owner: Vec<OwnerCount> = owners.into_iter().map(|(owner, count)| OwnerCount { owner, count }).collect();
    by_owner.sort_by(|a, b| b.count.cmp(&a.count).then(a.owner.cmp(&b.owner)));
    by_owner.truncate(10);
    let violations: Vec<Violation> = findings
        .iter()
        .take(o.limit)
        .map(|f| {
            let (entity, owner, near, location) = match f.entity.map(parts) {
                Some((k, ix)) => {
                    let l = locate(m, k, ix);
                    (Some(l.entity), l.owner, l.near, l.location)
                }
                None => (None, None, Vec::new(), None),
            };
            Violation { code: f.code.clone(), severity: f.severity, entity, detail: f.detail.clone(), owner, near, location }
        })
        .collect();
    let (geometry_checked, geometry_skipped_reason) = match geometry {
        GeometryStatus::Checked => (true, None),
        GeometryStatus::Skipped(r) => (false, Some(r)),
    };
    ValidateReport {
        schema_version: schema_version(),
        mesh: m.mesh_id(),
        valid: !findings.iter().any(|f| f.severity == Severity::Error),
        geometry_checked,
        geometry_skipped_reason,
        total,
        by_code,
        by_owner,
        truncated: total > violations.len(),
        violations,
    }
}

/// Collect every finding of `m` (the sources of `mesh.validate`, in order), plus `mesh.stale`
/// when `stale`.
fn collect(m: &dyn MeshRead, o: &ValidateOptions, live: Option<&dyn Fn(EntityId) -> bool>, stale: bool) -> (Vec<Finding>, GeometryStatus) {
    let mut out: Vec<Finding> = validate(m).into_iter().map(|v| Finding::new(v.code.as_str(), Severity::Error, v.entity, v.detail)).collect();
    let structural = out.iter().find(|f| is_structural(&f.code)).map(|f| f.code.clone());
    let status = if !o.geometry {
        GeometryStatus::Skipped("geometry checks disabled by the request".into())
    } else if let Some(code) = structural {
        GeometryStatus::Skipped(format!("structural violation `{code}` found: geometry needs a sound topology"))
    } else {
        let mut tol = GeomTol::default();
        tol.planarity = o.planarity;
        tol.min_volume = o.min_volume;
        out.extend(validate_geometry(m, &tol).into_iter().map(|v| Finding::new(v.code.as_str(), Severity::Error, v.entity, v.detail)));
        match validity(m, &Options::default()) {
            Ok(v) => {
                for (c, s) in v.status.iter().enumerate() {
                    let (code, what) = match s {
                        CellStatus::Inverted => ("quality.inverted", "a corner Jacobian or the volume is negative"),
                        CellStatus::Degenerate => ("quality.degenerate", "the cell is collapsed (zero Jacobian or non-finite coordinates)"),
                        _ => continue,
                    };
                    out.push(Finding::new(code, Severity::Error, Some(MeshEntity::Cell(sm_mesh_api::CellIx::new(c))), what));
                }
            }
            Err(e) => out.push(Finding::new("diag.validity_unavailable", Severity::Warning, None, format!("cell validity (inverted and degenerate cells) could not be computed: {e}"))),
        }
        GeometryStatus::Checked
    };
    if stale {
        out.push(Finding::new("mesh.stale", Severity::Warning, None, "the model entities this mesh was generated for changed or were deleted after meshing"));
    }
    if let Some(live) = live {
        for &id in m.classified_entities() {
            if live(id) {
                continue;
            }
            let Some(set) = m.classified_on(id) else { continue };
            let n = set.vertices.len() + set.edges.len() + set.faces.len() + set.cells.len();
            let first = set
                .cells
                .first()
                .map(|c| MeshEntity::Cell(*c))
                .or_else(|| set.faces.first().map(|f| MeshEntity::Face(*f)))
                .or_else(|| set.edges.first().map(|e| MeshEntity::Edge(*e)))
                .or_else(|| set.vertices.first().map(|v| MeshEntity::Vertex(*v)));
            out.push(Finding::new("diag.class_retired", Severity::Error, first, format!("{id} classifies {n} mesh entities but is no longer live")));
        }
    }
    (out, status)
}

/// Validate `m`: [`sm_mesh_api::validate()`](sm_mesh_api::validate()); then, if no structural code was found, geometry
/// ([`sm_mesh_api::validate_geometry`] and [`sm_quality::validity`] as `quality.inverted` /
/// `quality.degenerate`; if that cannot run, a `diag.validity_unavailable` warning says so); then `diag.class_retired` for every classification entity `live`
/// rejects. `live` is `None` when there is no state to ask. Pure and deterministic.
pub fn validate_mesh(m: &dyn MeshRead, o: &ValidateOptions, live: Option<&dyn Fn(EntityId) -> bool>) -> ValidateReport {
    validate_with(m, o, live, false)
}

pub(crate) fn validate_with(m: &dyn MeshRead, o: &ValidateOptions, live: Option<&dyn Fn(EntityId) -> bool>, stale: bool) -> ValidateReport {
    let (findings, status) = collect(m, o, live, stale);
    build_report(m, findings, o, status)
}

fn default_true() -> bool {
    true
}
fn default_planarity() -> f64 {
    1e-6
}
fn default_limit() -> usize {
    100
}

/// `mesh.validate`: check a mesh and locate what is wrong.
///
/// Runs `sm_mesh_api::validate`, then (if the topology is sound) the geometry checks and the
/// corner-Jacobian validity, then flags classification entities that are no longer live and a
/// stale mesh entry. Every violation carries its entity, its owner (the entity's own
/// classification), the lower-dimensional model entities near it and a point.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshValidate {
    /// The mesh; defaults to the only mesh of the state.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: Option<EntityId>,
    /// Run the geometric checks (default true).
    #[serde(default = "default_true")]
    pub geometry: bool,
    /// Planarity tolerance relative to `sqrt(area)` (default 1e-6).
    #[serde(default = "default_planarity")]
    pub planarity: f64,
    /// Cells with volume at most this are reported (default 0).
    #[serde(default)]
    pub min_volume: f64,
    /// Number of violations returned in full, 0 to 10000 (default 100).
    #[serde(default = "default_limit")]
    pub limit: usize,
}

impl TypedQuery for MeshValidate {
    const NAME: &'static str = "mesh.validate";
    type Output = ValidateReport;

    fn run(&self, state: &State, _cx: &QueryCtx<'_>) -> Result<ValidateReport, OpError> {
        if self.limit > MAX_LIMIT {
            return Err(OpError::Invalid(format!("limit {} exceeds {MAX_LIMIT}", self.limit)));
        }
        if !self.planarity.is_finite() || self.planarity < 0.0 || !self.min_volume.is_finite() {
            return Err(OpError::Invalid("planarity must be finite and >= 0, min_volume finite".into()));
        }
        let layer = state.layer::<MeshLayer>().ok_or_else(|| OpError::MissingLayer("meshes".into()))?;
        let id = resolve_mesh(layer, self.mesh)?;
        let entry = layer.get(id).ok_or(OpError::UnknownEntity(id))?;
        let o = ValidateOptions { geometry: self.geometry, planarity: self.planarity, min_volume: self.min_volume, limit: self.limit };
        let live = |e: EntityId| state.contains(e);
        Ok(validate_with(entry.mesh(), &o, Some(&live), entry.stale))
    }
}

