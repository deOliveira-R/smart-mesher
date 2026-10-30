//! The fixed suggestion table of `mesh.quality`.
//!
//! Rules (documented in `DESIGN.md`, deterministic, no op list is hard-coded: availability is
//! asked from the registry through the `ops` predicate):
//!
//! | Trigger | Suggestion |
//! |---|---|
//! | cluster on a `TopoEdge`, fraction >= 0.5 and enrichment >= 2 | `adjust_intervals` (op `mesh.intervals`, params `{current_count}`) |
//! | cluster on a `TopoFace`, same thresholds | `change_scheme` (op `mesh.scheme`); for `face_flatness`, `face_warp`, `face_concavity` on the transport profile `refine_surface` (op `mesh.size`) instead |
//! | bad entities of a metric and no cluster with enrichment >= 1.5 | `smooth` (op `mesh.smooth`) |
//! | bad entities of a metric and none of the rules above produced a suggestion for it (a hot spot below the cluster thresholds or on a vertex) | `inspect` (op `mesh.flag_cells`): store the bad cells as a set to look at |
//! | inverted or degenerate cells | `fix_geometry_or_scheme` per owning volume, no op |

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use sm_core::{EntityId, EntityKind};
use sm_mesh_api::MeshRead;
use sm_quality::{FvMetric, Measure, Profile};

use crate::quality::Cluster;
use crate::validate::Severity;

/// Op named by `adjust_intervals`.
pub const OP_INTERVALS: &str = "mesh.intervals";
/// Op named by `change_scheme`.
pub const OP_SCHEME: &str = "mesh.scheme";
/// Op named by `refine_surface`.
pub const OP_SIZE: &str = "mesh.size";
/// Op named by `inspect`.
pub const OP_FLAG: &str = "mesh.flag_cells";
/// Op named by `smooth`.
pub const OP_SMOOTH: &str = "mesh.smooth";

/// Cluster fraction needed for an `adjust_intervals` / `change_scheme` suggestion.
pub const MIN_FRACTION: f64 = 0.5;
/// Cluster enrichment needed for an `adjust_intervals` / `change_scheme` suggestion.
pub const MIN_ENRICHMENT: f64 = 2.0;
/// Enrichment below which a metric's bad entities count as spread over the whole mesh.
pub const SPREAD_ENRICHMENT: f64 = 1.5;

/// A rule-based suggested action.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Suggestion {
    /// Stable id: `kind:target:metric`.
    pub id: String,
    /// `adjust_intervals`, `change_scheme`, `refine_surface`, `smooth`, `inspect` or `fix_geometry_or_scheme`.
    pub kind: String,
    /// `error` if the metric fails somewhere, else `warning`.
    pub severity: Severity,
    /// The metric that triggered it (`validity` for invalid cells).
    pub metric: String,
    /// Entities the action applies to.
    pub targets: Vec<EntityId>,
    /// Name of the op that carries it out, if any.
    pub op: Option<String>,
    /// Ready-made op arguments, if any (the op's own schema decides the rest).
    pub args: Option<Value>,
    /// Numbers behind the suggestion.
    pub params: Value,
    /// Why, in a sentence.
    pub reason: String,
    /// True if `op` is registered.
    pub available: bool,
}

/// What the table needs to know about one graded metric.
pub(crate) struct MetricFacts<'a> {
    pub measure: Measure,
    pub n_fail: usize,
    pub n_bad: usize,
    pub clusters: &'a [Cluster],
}

/// Invalid cells of one owner.
pub(crate) struct InvalidOwner {
    pub owner: EntityId,
    pub n_inverted: usize,
    pub n_degenerate: usize,
}

struct Availability<'a> {
    ops: &'a dyn Fn(&str) -> bool,
    provided: bool,
}

impl Availability<'_> {
    /// `(available, reason suffix)` for `op`.
    fn of(&self, op: &str) -> (bool, String) {
        if !self.provided {
            (false, "; op registry not provided".into())
        } else if (self.ops)(op) {
            (true, String::new())
        } else {
            (false, format!("; op `{op}` is not registered"))
        }
    }
}

pub(crate) fn suggest(
    m: &dyn MeshRead,
    profile: Profile,
    facts: &[MetricFacts<'_>],
    invalid: &[InvalidOwner],
    ops: &dyn Fn(&str) -> bool,
    registry_provided: bool,
) -> Vec<Suggestion> {
    let av = Availability { ops, provided: registry_provided };
    let mut out: Vec<Suggestion> = Vec::new();
    let mesh = m.mesh_id();
    for f in facts {
        let name = f.measure.name();
        let severity = if f.n_fail > 0 { Severity::Error } else { Severity::Warning };
        let transport_surface = profile == Profile::Transport
            && matches!(f.measure, Measure::Fv(FvMetric::FaceFlatness | FvMetric::FaceWarp | FvMetric::FaceConcavity));
        let first = out.len();
        let strong = |c: &&Cluster| c.fraction >= MIN_FRACTION && c.enrichment >= MIN_ENRICHMENT;
        for c in f.clusters.iter().filter(strong) {
            let pct = format!("{}/{} touching entities are bad, enrichment {:.1}", c.n_bad, c.n_total, c.enrichment);
            match c.model.kind() {
                EntityKind::TopoEdge => {
                    let current = m.classified_on(c.model).map_or(0, |s| s.vertices.len()) as u64 + 1;
                    let (available, why) = av.of(OP_INTERVALS);
                    out.push(Suggestion {
                        id: format!("adjust_intervals:{}:{name}", c.model),
                        kind: "adjust_intervals".into(),
                        severity,
                        metric: name.into(),
                        targets: vec![c.model],
                        op: Some(OP_INTERVALS.into()),
                        args: None,
                        params: json!({ "current_count": current }),
                        reason: format!("{name}: bad entities cluster on {} ({pct}); change its interval count (now {current}){why}", c.model),
                        available,
                    });
                }
                EntityKind::TopoFace if transport_surface => {
                    let (available, why) = av.of(OP_SIZE);
                    out.push(Suggestion {
                        id: format!("refine_surface:{}:{name}", c.model),
                        kind: "refine_surface".into(),
                        severity,
                        metric: name.into(),
                        targets: vec![c.model],
                        op: Some(OP_SIZE.into()),
                        args: None,
                        params: json!({ "n_bad": c.n_bad, "n_total": c.n_total }),
                        reason: format!("{name}: bad entities cluster on {} ({pct}); a finer size on this surface flattens its faces{why}", c.model),
                        available,
                    });
                }
                EntityKind::TopoFace => {
                    let (available, why) = av.of(OP_SCHEME);
                    out.push(Suggestion {
                        id: format!("change_scheme:{}:{name}", c.model),
                        kind: "change_scheme".into(),
                        severity,
                        metric: name.into(),
                        targets: vec![c.model],
                        op: Some(OP_SCHEME.into()),
                        args: None,
                        params: json!({ "n_bad": c.n_bad, "n_total": c.n_total }),
                        reason: format!("{name}: bad entities cluster on {} ({pct}); try another scheme on it{why}", c.model),
                        available,
                    });
                }
                _ => {}
            }
        }
        if f.n_bad > 0 && !f.clusters.iter().any(|c| c.enrichment >= SPREAD_ENRICHMENT) {
            let (available, why) = av.of(OP_SMOOTH);
            out.push(Suggestion {
                id: format!("smooth:{mesh}:{name}"),
                kind: "smooth".into(),
                severity,
                metric: name.into(),
                targets: vec![mesh],
                op: Some(OP_SMOOTH.into()),
                args: None,
                params: json!({ "n_bad": f.n_bad }),
                reason: format!("{name}: {} bad entities are spread over the mesh, no hot spot; smoothing may help{why}", f.n_bad),
                available,
            });
        }
        // Gap-filler: bad entities that no rule above turned into an action.
        if f.n_bad > 0 && out.len() == first {
            let (available, why) = av.of(OP_FLAG);
            out.push(Suggestion {
                id: format!("inspect:{mesh}:{name}"),
                kind: "inspect".into(),
                severity,
                metric: name.into(),
                targets: vec![mesh],
                op: Some(OP_FLAG.into()),
                args: None,
                params: json!({ "n_bad": f.n_bad }),
                reason: format!("{name}: {} bad entities form a hot spot too small or too specific for a rule; flag them as a set and inspect{why}", f.n_bad),
                available,
            });
        }
    }
    for i in invalid {
        out.push(Suggestion {
            id: format!("fix_geometry_or_scheme:{}:validity", i.owner),
            kind: "fix_geometry_or_scheme".into(),
            severity: Severity::Error,
            metric: "validity".into(),
            targets: vec![i.owner],
            op: None,
            args: None,
            params: json!({ "n_inverted": i.n_inverted, "n_degenerate": i.n_degenerate }),
            reason: format!("{} inverted and {} degenerate cells on {}; fix the geometry or pick another scheme", i.n_inverted, i.n_degenerate, i.owner),
            available: false,
        });
    }
    out.sort_by(|a, b| a.severity.cmp(&b.severity).then_with(|| a.id.cmp(&b.id)));
    out
}
