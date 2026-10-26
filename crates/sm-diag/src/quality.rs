//! `mesh.quality`: graded metric reports with worst entities, hot-spot clusters and suggestions.

use std::collections::BTreeMap;

use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::MeshLayer;
use sm_mesh_api::{CellIx, FaceIx, MeshDim, MeshEntityKind, MeshRead, entity_dim};
use sm_op_api::{OpError, QueryCtx, State, TypedQuery};
use sm_quality::{CHUNK, CellStatus, DBL_MAX, FvEntity, Measure, Options, Profile, Rule, Sense, histogram_auto, summarize, validity};

use crate::OpNames;
use crate::analysis::{G_FAIL, G_WARN, MetricColumn, SkippedMetric, ThresholdOverride, compute_columns, grade_ranks, resolve, sense_of};
use crate::common::{fin, resolve_mesh, schema_version};
use crate::error::DiagError;
use crate::locate::{EntityRef, entity_ref, location, near, owner};
use crate::suggest::{InvalidOwner, MetricFacts, Suggestion, suggest};

/// Largest accepted `worst_n`.
pub const MAX_WORST: usize = 100;
/// Number of clusters kept per metric.
pub const MAX_CLUSTERS: usize = 10;

/// Name of a quality profile on the wire.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum ProfileName {
    /// General-purpose limits.
    #[default]
    Generic,
    /// OpenFOAM `checkMesh` limits.
    Openfoam,
    /// Fluent orthogonal quality.
    Fluent,
    /// Planar convex faces for ray tracing and sweeps.
    Transport,
}

impl From<ProfileName> for Profile {
    fn from(p: ProfileName) -> Self {
        match p {
            ProfileName::Generic => Profile::Generic,
            ProfileName::Openfoam => Profile::OpenFoam,
            ProfileName::Fluent => Profile::Fluent,
            ProfileName::Transport => Profile::Transport,
        }
    }
}

/// Overall verdict of a quality report.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum Verdict {
    /// No failing or warning entity, no invalid cell.
    Pass,
    /// Some entity warns, none fails.
    Warn,
    /// Some entity fails or a cell is invalid.
    Fail,
}

/// Options of [`quality_report`].
#[derive(Clone, Debug, PartialEq)]
pub struct QualityOptions {
    /// The profile whose rules are graded.
    pub profile: Profile,
    /// Metrics to report instead of the profile's rules (graded only where a rule exists).
    pub metrics: Option<Vec<Measure>>,
    /// Limit overrides by metric wire name.
    pub thresholds: BTreeMap<String, ThresholdOverride>,
    /// Worst entities per metric, 0 to 100.
    pub worst_n: usize,
    /// Histogram bins, 2 to 200.
    pub bins: usize,
    /// False when no op registry was available: every suggestion with an op is then unavailable
    /// and says so.
    pub registry_provided: bool,
}

impl Default for QualityOptions {
    fn default() -> Self {
        Self { profile: Profile::Generic, metrics: None, thresholds: BTreeMap::new(), worst_n: 10, bins: 20, registry_provided: true }
    }
}

/// Summary statistics of the finite, non-sentinel values; `null` when there are none.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct SummaryOut {
    /// Smallest value.
    pub min: Option<f64>,
    /// Largest value.
    pub max: Option<f64>,
    /// Mean.
    pub mean: Option<f64>,
    /// Population standard deviation.
    pub std_dev: Option<f64>,
}

/// Limits actually applied; `null` where there is none (or it is infinite).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct ThresholdOut {
    /// Warning limit.
    pub warn: Option<f64>,
    /// Failure limit.
    pub fail: Option<f64>,
    /// Failure limit of boundary faces.
    pub fail_boundary: Option<f64>,
}

/// Histogram of the finite, non-sentinel values.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct HistogramOut {
    /// Lower edge of the first bin.
    pub lo: f64,
    /// Upper edge of the last bin.
    pub hi: f64,
    /// Counts per bin.
    pub counts: Vec<u64>,
    /// Values below `lo`.
    pub below: u64,
    /// Values above `hi`.
    pub above: u64,
}

/// One of the worst entities of a metric.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct WorstEntry {
    /// The entity.
    pub entity: EntityRef,
    /// Its value (the 1e30 sentinel is a number).
    pub value: f64,
    /// `ok`, `warn` or `fail`; `null` if the metric is not graded.
    pub grade: Option<String>,
    /// Face centroid or cell centroid.
    pub location: Option<[f64; 3]>,
    /// The entity's own classification.
    pub owner: Option<EntityId>,
    /// Lower-dimensional model entities around it.
    pub near: Vec<EntityId>,
    /// The cells of a face; absent for cell metrics.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cells: Option<Vec<EntityId>>,
}

/// A model entity where bad entities concentrate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Cluster {
    /// The model entity (dimension at most 2).
    pub model: EntityId,
    /// Bad entities (grade warn or fail) touching it.
    pub n_bad: usize,
    /// Entities with a defined value touching it.
    pub n_total: usize,
    /// `n_bad / n_total`.
    pub fraction: f64,
    /// `fraction` over the mesh-wide bad fraction.
    pub enrichment: f64,
}

/// Report of one metric.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct MetricReport {
    /// Wire name of the metric.
    pub metric: String,
    /// `face` or `cell`.
    pub entity: String,
    /// `deg` or empty.
    pub unit: String,
    /// `lower_is_worse` or `higher_is_worse` (informational for ungraded metrics).
    pub sense: String,
    /// Entities of this kind in the mesh: `counts + below + above + undefined + n_sentinel`.
    pub count: usize,
    /// Entities where the metric is undefined (NaN, boundary face for an internal-only metric).
    pub undefined: usize,
    /// Entities holding the 1e30 sentinel (excluded from summary and histogram).
    pub n_sentinel: usize,
    /// Statistics without the sentinel.
    pub summary: SummaryOut,
    /// The limits applied; all `null` for an ungraded metric.
    pub threshold: ThresholdOut,
    /// Entities graded `warn` (not counting `fail`).
    pub n_warn: usize,
    /// Entities graded `fail`.
    pub n_fail: usize,
    /// Histogram without the sentinel.
    pub histogram: HistogramOut,
    /// The worst entities, worst first.
    pub worst: Vec<WorstEntry>,
    /// Hot spots by model entity, at most ten.
    pub clusters: Vec<Cluster>,
}

/// Cell validity counts.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct ValiditySummary {
    /// Valid cells.
    pub n_valid: usize,
    /// Degenerate cells.
    pub n_degenerate: usize,
    /// Inverted cells.
    pub n_inverted: usize,
}

/// Reply of `mesh.quality`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct QualityReport {
    /// Wire version, 1.
    pub schema_version: u32,
    /// The mesh.
    pub mesh: EntityId,
    /// The profile graded.
    pub profile: String,
    /// Number of cells.
    pub cells: usize,
    /// Mesh dimension, 2 or 3.
    pub dim: u8,
    /// `pass`, `warn` or `fail`.
    pub verdict: Verdict,
    /// Cell validity.
    pub validity: ValiditySummary,
    /// One report per computed metric.
    pub metrics: Vec<MetricReport>,
    /// Metrics no cell supports.
    pub skipped: Vec<SkippedMetric>,
    /// Suggested actions.
    pub suggestions: Vec<Suggestion>,
}

fn sense_name(s: Sense) -> &'static str {
    match s {
        Sense::LowerIsWorse => "lower_is_worse",
        _ => "higher_is_worse",
    }
}

/// True for the Verdict "large" sentinel (either sign).
fn is_sentinel(x: f64) -> bool {
    x.is_finite() && x.abs() >= DBL_MAX
}

/// `a` is a worse value than `b` (ties are the caller's business).
fn worse(sense: Sense, a: f64, b: f64) -> std::cmp::Ordering {
    match sense {
        Sense::LowerIsWorse => a.total_cmp(&b),
        _ => b.total_cmp(&a),
    }
}

/// The `k` worst finite entries of `values`, worst first, ties by ascending index. Per-chunk
/// lists are merged in chunk order, so the result is independent of scheduling.
fn worst_indices(values: &[f64], sense: Sense, k: usize) -> Vec<usize> {
    if k == 0 {
        return Vec::new();
    }
    let cmp = |a: &(f64, usize), b: &(f64, usize)| worse(sense, a.0, b.0).then(a.1.cmp(&b.1));
    let parts: Vec<Vec<(f64, usize)>> = values
        .par_chunks(CHUNK)
        .enumerate()
        .map(|(ci, chunk)| {
            let mut top: Vec<(f64, usize)> = Vec::with_capacity(k + 1);
            for (i, &v) in chunk.iter().enumerate() {
                if !v.is_finite() {
                    continue;
                }
                let cand = (v, ci * CHUNK + i);
                if top.len() == k && cmp(&cand, &top[k - 1]).is_ge() {
                    continue;
                }
                let at = top.partition_point(|t| cmp(t, &cand).is_lt());
                top.insert(at, cand);
                top.truncate(k);
            }
            top
        })
        .collect();
    let mut all: Vec<(f64, usize)> = parts.into_iter().flatten().collect();
    all.sort_by(cmp);
    all.truncate(k);
    all.into_iter().map(|t| t.1).collect()
}

/// Vertex-class model indices of every vertex (dimension <= 2), for the cluster pass.
struct Touch {
    models: Vec<EntityId>,
    vclass: Vec<u32>,
}

const NO_MODEL: u32 = u32::MAX;

impl Touch {
    fn new(m: &dyn MeshRead) -> Self {
        let nv = m.count(MeshEntityKind::Vertex);
        let classes: Vec<Option<EntityId>> = (0..nv)
            .map(|v| {
                let e = m.classification(MeshEntityKind::Vertex, v as u32).entity;
                entity_dim(e.kind()).filter(|d| *d <= 2).map(|_| e)
            })
            .collect();
        let mut models: Vec<EntityId> = classes.iter().flatten().copied().collect();
        models.sort_unstable();
        models.dedup();
        let vclass = classes.iter().map(|c| c.and_then(|e| models.binary_search(&e).ok()).map_or(NO_MODEL, |i| i as u32)).collect();
        Self { models, vclass }
    }

    /// Model indices touched by entity `ix` (sorted, distinct) into `out`.
    fn of(&self, m: &dyn MeshRead, entity: FvEntity, ix: usize, out: &mut Vec<u32>) {
        out.clear();
        let mut add = |vs: &[sm_mesh_api::VertexIx]| {
            for v in vs {
                let c = self.vclass[v.get()];
                if c != NO_MODEL {
                    out.push(c);
                }
            }
        };
        match entity {
            FvEntity::Face => add(m.face_vertices(FaceIx::new(ix))),
            _ => {
                let c = CellIx::new(ix);
                match m.cell_corners(c) {
                    Some(corners) => add(corners),
                    None => {
                        for sf in m.cell_faces(c) {
                            add(m.face_vertices(sf.face()));
                        }
                    }
                }
            }
        }
        out.sort_unstable();
        out.dedup();
    }
}

/// Clusters of bad entities by model entity: `bad[i]` marks bad entities, `defined[i]` those with
/// a value. Sorted by fraction (descending), bad count (descending), id; the top ten.
fn clusters(m: &dyn MeshRead, touch: &Touch, entity: FvEntity, ranks: &[u8], values: &[f64]) -> Vec<Cluster> {
    let mut counts: Vec<(usize, usize)> = vec![(0, 0); touch.models.len()];
    let (mut bad_all, mut defined_all) = (0usize, 0usize);
    let mut scratch: Vec<u32> = Vec::new();
    for (i, &v) in values.iter().enumerate() {
        if !v.is_finite() {
            continue;
        }
        let bad = ranks[i] == G_WARN || ranks[i] == G_FAIL;
        defined_all += 1;
        bad_all += usize::from(bad);
        touch.of(m, entity, i, &mut scratch);
        for &c in &scratch {
            counts[c as usize].1 += 1;
            counts[c as usize].0 += usize::from(bad);
        }
    }
    if bad_all == 0 {
        return Vec::new();
    }
    let overall = bad_all as f64 / defined_all as f64;
    let mut out: Vec<(usize, usize, EntityId)> =
        counts.iter().enumerate().filter(|(_, c)| c.0 > 0).map(|(i, c)| (c.0, c.1, touch.models[i])).collect();
    // fraction descending by exact cross-multiplication, then bad count descending, then id
    out.sort_by(|a, b| (b.0 * a.1).cmp(&(a.0 * b.1)).then(b.0.cmp(&a.0)).then(a.2.cmp(&b.2)));
    out.truncate(MAX_CLUSTERS);
    out.into_iter()
        .map(|(n_bad, n_total, model)| {
            let fraction = n_bad as f64 / n_total as f64;
            Cluster { model, n_bad, n_total, fraction, enrichment: fraction / overall }
        })
        .collect()
}

struct Analyzed {
    report: MetricReport,
    n_bad: usize,
}

fn threshold_out(rule: Option<&Rule>) -> ThresholdOut {
    match rule {
        Some(r) => ThresholdOut { warn: r.warn.and_then(fin), fail: fin(r.fail), fail_boundary: r.fail_boundary.and_then(fin) },
        None => ThresholdOut { warn: None, fail: None, fail_boundary: None },
    }
}

fn analyze(m: &dyn MeshRead, col: &MetricColumn, rule: Option<&Rule>, o: &QualityOptions, touch: &mut Option<Touch>) -> Analyzed {
    let n = col.values.len();
    let sense = sense_of(col.measure, rule);
    let counts: Vec<[usize; 2]> = col
        .values
        .par_chunks(CHUNK)
        .map(|c| {
            let mut a = [0usize; 2];
            for &v in c {
                if !v.is_finite() {
                    a[0] += 1;
                } else if is_sentinel(v) {
                    a[1] += 1;
                }
            }
            a
        })
        .collect();
    let (undefined, n_sentinel) = counts.iter().fold((0, 0), |s, a| (s.0 + a[0], s.1 + a[1]));
    let clean: Vec<f64> = col.values.par_iter().map(|&v| if v.is_finite() && !is_sentinel(v) { v } else { f64::NAN }).collect();
    let s = summarize(&clean);
    let summary = if s.count == 0 {
        SummaryOut { min: None, max: None, mean: None, std_dev: None }
    } else {
        SummaryOut { min: fin(s.min), max: fin(s.max), mean: fin(s.mean), std_dev: fin(s.std_dev) }
    };
    let histogram = match histogram_auto(&clean, o.bins) {
        Ok(h) => HistogramOut { lo: h.lo, hi: h.hi, counts: h.counts, below: h.below, above: h.above },
        Err(_) => HistogramOut { lo: 0.0, hi: 1.0, counts: vec![0; o.bins], below: 0, above: 0 },
    };
    let ranks = rule.map(|r| grade_ranks(m, col, r));
    let (mut n_warn, mut n_fail) = (0, 0);
    if let Some(r) = &ranks {
        for &g in r {
            n_warn += usize::from(g == G_WARN);
            n_fail += usize::from(g == G_FAIL);
        }
    }
    let n_bad = n_warn + n_fail;
    let face = col.entity == FvEntity::Face;
    let kind = if face { MeshEntityKind::Face } else { MeshEntityKind::Cell };
    let worst: Vec<WorstEntry> = worst_indices(&col.values, sense, o.worst_n)
        .into_iter()
        .map(|i| {
            let ow = owner(m, kind, i as u32);
            let grade = rule.map(|r| r.grade(col.values[i], face && m.face_right(FaceIx::new(i)).is_none()).name().to_owned());
            let cells = face.then(|| {
                let (l, r) = m.face_cells(FaceIx::new(i));
                std::iter::once(l).chain(r).map(|c| m.cell_id(c)).collect()
            });
            WorstEntry {
                entity: entity_ref(m, kind, i as u32),
                value: col.values[i],
                grade,
                location: location(m, kind, i as u32),
                owner: ow,
                near: near(m, kind, i as u32, ow),
                cells,
            }
        })
        .collect();
    let clusters = match &ranks {
        Some(r) if n_bad > 0 => {
            let t = touch.get_or_insert_with(|| Touch::new(m));
            clusters(m, t, col.entity, r, &col.values)
        }
        _ => Vec::new(),
    };
    let unit = match col.measure {
        Measure::Fv(f) => f.unit(),
        other => other.unit(),
    };
    Analyzed {
        report: MetricReport {
            metric: col.measure.name().into(),
            entity: col.entity.name().into(),
            unit: unit.into(),
            sense: sense_name(sense).into(),
            count: n,
            undefined,
            n_sentinel,
            summary,
            threshold: threshold_out(rule),
            n_warn,
            n_fail,
            histogram,
            worst,
            clusters,
        },
        n_bad,
    }
}

/// Compute the quality report of `m`.
///
/// `ops` answers "is this op registered" for the suggestion table; set
/// [`QualityOptions::registry_provided`] to false when there is no registry to ask. Metrics no
/// cell supports go to `skipped`, never to an error. Assumes a structurally valid mesh (every
/// finished mesh is); run `mesh.validate` for a suspect one.
pub fn quality_report(m: &dyn MeshRead, o: &QualityOptions, ops: &dyn Fn(&str) -> bool) -> Result<QualityReport, DiagError> {
    if o.worst_n > MAX_WORST {
        return Err(DiagError::BadArgument(format!("worst_n {} exceeds {MAX_WORST}", o.worst_n)));
    }
    if !(2..=200).contains(&o.bins) {
        return Err(DiagError::BadArgument(format!("bins {} is outside 2..=200", o.bins)));
    }
    let items = resolve(o.profile, o.metrics.as_deref(), &o.thresholds)?;
    let measures: Vec<Measure> = items.iter().map(|(m, _)| *m).collect();
    let (cols, skipped) = compute_columns(m, &measures)?;
    let v = validity(m, &Options::default())?;
    let validity_out = ValiditySummary { n_valid: v.n_valid, n_degenerate: v.n_degenerate, n_inverted: v.n_inverted };
    let mut touch: Option<Touch> = None;
    let mut metrics = Vec::with_capacity(cols.len());
    let mut bad_counts = Vec::with_capacity(cols.len());
    for col in &cols {
        let rule = items.iter().find(|(x, _)| *x == col.measure).and_then(|(_, r)| r.as_ref());
        let a = analyze(m, col, rule, o, &mut touch);
        bad_counts.push(a.n_bad);
        metrics.push(a.report);
    }
    let facts: Vec<MetricFacts<'_>> = cols
        .iter()
        .zip(&metrics)
        .zip(&bad_counts)
        .map(|((c, r), &n_bad)| MetricFacts { measure: c.measure, n_fail: r.n_fail, n_bad, clusters: &r.clusters })
        .collect();
    // Invalid cells grouped by their owner, in ascending owner order.
    let mut by_owner: BTreeMap<EntityId, (usize, usize)> = BTreeMap::new();
    for (c, s) in v.status.iter().enumerate() {
        let slot = match s {
            CellStatus::Inverted => 0,
            CellStatus::Degenerate => 1,
            _ => continue,
        };
        let e = &mut by_owner.entry(m.classification(MeshEntityKind::Cell, c as u32).entity).or_default();
        if slot == 0 {
            e.0 += 1;
        } else {
            e.1 += 1;
        }
    }
    let invalid: Vec<InvalidOwner> = by_owner.into_iter().map(|(owner, (n_inverted, n_degenerate))| InvalidOwner { owner, n_inverted, n_degenerate }).collect();
    let suggestions = suggest(m, o.profile, &facts, &invalid, ops, o.registry_provided);
    let verdict = if v.n_inverted + v.n_degenerate > 0 || metrics.iter().any(|r| r.n_fail > 0) {
        Verdict::Fail
    } else if metrics.iter().any(|r| r.n_warn > 0) {
        Verdict::Warn
    } else {
        Verdict::Pass
    };
    Ok(QualityReport {
        schema_version: schema_version(),
        mesh: m.mesh_id(),
        profile: o.profile.name().into(),
        cells: m.cell_count(),
        dim: if m.dim() == MeshDim::Two { 2 } else { 3 },
        verdict,
        validity: validity_out,
        metrics,
        skipped,
        suggestions,
    })
}

fn default_worst() -> usize {
    10
}
fn default_bins() -> usize {
    20
}

/// `mesh.quality`: grade a mesh against a profile and say where and why it is bad.
///
/// Per metric: counts, summary, histogram, the worst entities with locations, the model entities
/// where bad entities cluster (with an enrichment ratio over the mesh average) and, from a fixed
/// rule table, suggested actions that name a registered op.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshQuality {
    /// The mesh; defaults to the only mesh of the state.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: Option<EntityId>,
    /// `generic` (default), `openfoam`, `fluent` or `transport`.
    #[serde(default)]
    pub profile: ProfileName,
    /// Metric names to report instead of the profile's rules.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub metrics: Option<Vec<String>>,
    /// Limit overrides by metric name: `{"non_orthogonality": {"fail": 40}}`.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub thresholds: BTreeMap<String, ThresholdOverride>,
    /// Worst entities per metric, 0 to 100 (default 10).
    #[serde(default = "default_worst")]
    pub worst_n: usize,
    /// Histogram bins, 2 to 200 (default 20).
    #[serde(default = "default_bins")]
    pub bins: usize,
}

/// Parse metric names into measures.
pub(crate) fn parse_metrics(names: &Option<Vec<String>>) -> Result<Option<Vec<Measure>>, DiagError> {
    names
        .as_ref()
        .map(|v| v.iter().map(|n| Measure::from_name(n).ok_or_else(|| DiagError::BadArgument(format!("unknown metric `{n}`")))).collect())
        .transpose()
}

impl TypedQuery for MeshQuality {
    const NAME: &'static str = "mesh.quality";
    type Output = QualityReport;

    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<QualityReport, OpError> {
        let layer = state.layer::<MeshLayer>().ok_or_else(|| OpError::MissingLayer("meshes".into()))?;
        let id = resolve_mesh(layer, self.mesh)?;
        let entry = layer.get(id).ok_or(OpError::UnknownEntity(id))?;
        let o = QualityOptions {
            profile: self.profile.into(),
            metrics: parse_metrics(&self.metrics)?,
            thresholds: self.thresholds.clone(),
            worst_n: self.worst_n,
            bins: self.bins,
            registry_provided: cx.service::<OpNames>().is_ok(),
        };
        let names = cx.service::<OpNames>().ok();
        let ops = |op: &str| names.is_some_and(|n| n.0.contains(op));
        Ok(quality_report(entry.mesh(), &o, &ops)?)
    }
}

