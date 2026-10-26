//! Metric columns and grading shared by `mesh.quality` and `mesh.flag_cells`.

use std::collections::BTreeMap;

use rayon::prelude::*;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_mesh_api::{FaceIx, MeshDim, MeshRead};
use sm_quality::{
    CHUNK, FvColumn, FvEntity, FvMetric, Grade, Measure, Options, Profile, QualityError, Rule, Sense, compute, fv_compute, fv_geometry, rules,
};

use crate::error::DiagError;

/// Per-metric override of the limits of a rule.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ThresholdOverride {
    /// New warning limit.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub warn: Option<f64>,
    /// New failure limit.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fail: Option<f64>,
}

/// A metric no cell supports, and why.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct SkippedMetric {
    /// Wire name of the metric.
    pub metric: String,
    /// Why it was not computed.
    pub reason: String,
}

/// One computed column of a measure.
pub(crate) struct MetricColumn {
    pub measure: Measure,
    /// Face- or cell-indexed.
    pub entity: FvEntity,
    pub values: Vec<f64>,
}

pub(crate) const G_OK: u8 = 0;
pub(crate) const G_WARN: u8 = 1;
pub(crate) const G_FAIL: u8 = 2;
pub(crate) const G_UNDEF: u8 = 3;

pub(crate) fn rank_of(g: Grade) -> u8 {
    match g {
        Grade::Ok => G_OK,
        Grade::Warn => G_WARN,
        Grade::Fail => G_FAIL,
        _ => G_UNDEF,
    }
}

fn limit_ok(x: f64) -> bool {
    x.is_finite()
}

/// The rule of `measure` in any profile (the profile's own first), used for the sense and the
/// boundary limit of a measure the chosen profile does not grade.
fn any_rule(profile: Profile, measure: Measure) -> Option<Rule> {
    std::iter::once(profile).chain(Profile::ALL.iter().copied()).find_map(|p| rules(p).iter().find(|r| r.measure == measure).copied())
}

/// Measures to compute with their effective rule (`None` = computed but not graded).
///
/// `metrics` of `None` means the rules of `profile`. A requested metric is graded only if a rule
/// exists: the profile's, else one from the thresholds override (which borrows sense and
/// boundary limit from the first rule of that metric in any profile). Overrides must name a
/// metric of the set.
pub(crate) fn resolve(
    profile: Profile,
    metrics: Option<&[Measure]>,
    thresholds: &BTreeMap<String, ThresholdOverride>,
) -> Result<Vec<(Measure, Option<Rule>)>, DiagError> {
    let mut set: Vec<(Measure, Option<Rule>)> = match metrics {
        None => rules(profile).iter().map(|r| (r.measure, Some(*r))).collect(),
        Some(ms) => {
            let mut out: Vec<(Measure, Option<Rule>)> = Vec::new();
            for &m in ms {
                if !out.iter().any(|(x, _)| *x == m) {
                    out.push((m, rules(profile).iter().find(|r| r.measure == m).copied()));
                }
            }
            out
        }
    };
    for (name, t) in thresholds {
        let measure = Measure::from_name(name).ok_or_else(|| DiagError::BadArgument(format!("thresholds: unknown metric `{name}`")))?;
        let Some(slot) = set.iter_mut().find(|(m, _)| *m == measure) else {
            return Err(DiagError::BadArgument(format!("thresholds: metric `{name}` is not in the metric set")));
        };
        for v in [t.warn, t.fail].into_iter().flatten() {
            if !limit_ok(v) {
                return Err(DiagError::BadArgument(format!("thresholds: `{name}` limits must be finite")));
            }
        }
        let mut rule = match slot.1 {
            Some(r) => r,
            None => {
                let mut r = any_rule(profile, measure)
                    .ok_or_else(|| DiagError::BadArgument(format!("thresholds: no rule or sense known for metric `{name}`")))?;
                r.warn = None;
                r
            }
        };
        if let Some(w) = t.warn {
            rule.warn = Some(w);
        }
        if let Some(f) = t.fail {
            rule.fail = f;
        }
        slot.1 = Some(rule);
    }
    Ok(set)
}

/// Informational sense of an ungraded measure (graded ones use their rule).
pub(crate) fn sense_of(measure: Measure, rule: Option<&Rule>) -> Sense {
    use sm_quality::Metric as M;
    if let Some(r) = rule {
        return r.sense;
    }
    if let Some(r) = Profile::ALL.iter().find_map(|p| rules(*p).iter().find(|r| r.measure == measure)) {
        return r.sense;
    }
    match measure {
        Measure::Verdict(
            M::Volume | M::Jacobian | M::ScaledJacobian | M::Shape | M::ShapeAndSize | M::RelativeSizeSquared | M::Shear | M::ShearAndSize | M::Warpage | M::Stretch
            | M::Diagonal | M::Distortion | M::CollapseRatio | M::MinAngle | M::Sphericity | M::Starness,
        ) => Sense::LowerIsWorse,
        _ => Sense::HigherIsWorse,
    }
}

/// Compute the columns of `measures` (request order); unsupported ones go to the second list.
pub(crate) fn compute_columns(m: &dyn MeshRead, measures: &[Measure]) -> Result<(Vec<MetricColumn>, Vec<SkippedMetric>), DiagError> {
    let dim3 = m.dim() != MeshDim::Two;
    let fv: Vec<FvMetric> = measures.iter().filter_map(|x| if let Measure::Fv(f) = x { Some(*f) } else { None }).collect();
    let mut fv_cols: Vec<FvColumn> = if dim3 && !fv.is_empty() {
        let g = fv_geometry(m)?;
        fv_compute(m, &g, &fv)?
    } else {
        Vec::new()
    };
    let opts = Options::default();
    let (mut cols, mut skipped) = (Vec::new(), Vec::new());
    for &measure in measures {
        match measure {
            Measure::Verdict(metric) => match compute(m, metric, &opts) {
                Ok(values) => cols.push(MetricColumn { measure, entity: FvEntity::Cell, values }),
                Err(QualityError::Unsupported { .. }) => skipped.push(SkippedMetric {
                    metric: measure.name().into(),
                    reason: "not defined for any cell shape present in the mesh".into(),
                }),
                Err(e) => return Err(e.into()),
            },
            Measure::Fv(f) => {
                if !dim3 {
                    skipped.push(SkippedMetric { metric: measure.name().into(), reason: "finite-volume metrics need a dimension-3 mesh".into() });
                } else if let Some(i) = fv_cols.iter().position(|c| c.metric == f) {
                    let c = fv_cols.swap_remove(i);
                    cols.push(MetricColumn { measure, entity: c.entity, values: c.values });
                }
            }
            _ => skipped.push(SkippedMetric { metric: measure.name().into(), reason: "unknown measure kind".into() }),
        }
    }
    Ok((cols, skipped))
}

/// Grade rank of every entity of `col` (`G_*`); boundary faces use the boundary limit.
pub(crate) fn grade_ranks(m: &dyn MeshRead, col: &MetricColumn, rule: &Rule) -> Vec<u8> {
    let mut out = vec![G_UNDEF; col.values.len()];
    let face = col.entity == FvEntity::Face;
    out.par_chunks_mut(CHUNK).zip(col.values.par_chunks(CHUNK)).enumerate().for_each(|(ci, (o, v))| {
        for (i, (slot, &x)) in o.iter_mut().zip(v).enumerate() {
            let boundary = face && m.face_right(FaceIx::new(ci * CHUNK + i)).is_none();
            *slot = rank_of(rule.grade(x, boundary));
        }
    });
    out
}
