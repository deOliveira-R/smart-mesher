//! The auto-selection census: what `mesh.auto` would change, per volume, against what
//! `mesh.run` plans today.
//!
//! SERVER-HOST runs it through `standard()` with the real schemes (plan acceptance B7): every
//! case whose outcome differs is listed, none is hidden. The census only reads the state and
//! meshes into scratch; nothing is stored.

use std::fmt::Write as _;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_op_api::{OpError, State};
use sm_scheme_api::{Attempt, Firmness, Plan, SchemeChoice, SchemeError, Selection, Target};
use sm_topo_api::Dim;

use crate::auto::{AutoInput, DEFAULT_STRATEGY, scratch_plan};
use crate::config::MesherConfig;
use crate::intent_layer::IntentLayer;

/// What one side of the comparison planned for a volume.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct CensusOutcome {
    /// The scheme, when the run succeeds.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scheme: Option<String>,
    /// How it was chosen (`Intent`, `Auto`, `Imposed`, ...), when the run succeeds.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub selection: Option<String>,
    /// Why the run fails, when it does.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// One volume of the census.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct CensusRow {
    /// The volume.
    pub volume: EntityId,
    /// Today: `mesh.run` with the recorded intent and the registry's auto lists.
    pub before: CensusOutcome,
    /// With the strategy's assignments added.
    pub after: CensusOutcome,
    /// Faces of the volume whose scheme differs: `(face, before, after)`.
    pub faces_changed: Vec<(EntityId, String, String)>,
    /// `before` and `after` differ in scheme or in error.
    pub changed: bool,
}

/// The census of one strategy over a state.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct CensusReport {
    /// The strategy.
    pub strategy: String,
    /// One row per volume, ascending.
    pub rows: Vec<CensusRow>,
}

impl CensusReport {
    /// Rows whose outcome changed.
    pub fn changed(&self) -> impl Iterator<Item = &CensusRow> {
        self.rows.iter().filter(|r| r.changed)
    }

    /// A Markdown table of every changed case, with the totals.
    pub fn to_markdown(&self) -> String {
        let n = self.changed().count();
        let mut s = format!(
            "# Auto census, strategy `{}`\n\n{} volumes, {} changed.\n\n",
            self.strategy,
            self.rows.len(),
            n
        );
        if n > 0 {
            s.push_str("| volume | before | after | faces changed |\n|---|---|---|---|\n");
        }
        let show = |o: &CensusOutcome| match (&o.scheme, &o.error) {
            (Some(x), _) => x.clone(),
            (None, Some(e)) => format!("error: {}", e.replace('|', "/")),
            _ => "?".into(),
        };
        for r in self.changed() {
            let _ = writeln!(
                s,
                "| {} | {} | {} | {} |",
                r.volume,
                show(&r.before),
                show(&r.after),
                r.faces_changed.len()
            );
        }
        s
    }
}

fn outcome(
    plan: Result<(Plan, Vec<Attempt>), SchemeError>,
    t: Target,
) -> (CensusOutcome, Option<Plan>) {
    match plan {
        Ok((p, _)) => {
            let row = p.problem.targets.iter().find(|r| r.target == t);
            let o = CensusOutcome {
                scheme: row.map(|r| r.scheme.as_str().to_owned()),
                selection: row.map(|r| selection_name(&r.selection)),
                error: None,
            };
            (o, Some(p))
        }
        Err(e) => (
            CensusOutcome {
                scheme: None,
                selection: None,
                error: Some(e.to_string()),
            },
            None,
        ),
    }
}

fn selection_name(s: &Selection) -> String {
    match s {
        Selection::Intent { .. } => "Intent",
        Selection::Registry { .. } => "Registry",
        Selection::PieceShape => "PieceShape",
        Selection::Imposed { .. } => "Imposed",
        Selection::Auto { .. } => "Auto",
        Selection::Copy { .. } => "Copy",
        Selection::Claimed { .. } => "Claimed",
        _ => "Other",
    }
    .to_owned()
}

/// Run the census of `strategy` (default `"default"`) over `volumes` (default every unmeshed
/// volume), one volume at a time so that a failure stays with its volume.
///
/// "Before" meshes the volume into scratch with the recorded intent as it is; "after" meshes it
/// again with the strategy's assignments written as hard schemes, i.e. what `mesh.auto` followed
/// by `mesh.run` would plan.
pub fn census(
    state: &State,
    cfg: &MesherConfig,
    strategy: Option<&str>,
    volumes: Option<&[EntityId]>,
) -> Result<CensusReport, OpError> {
    let name = strategy.unwrap_or(DEFAULT_STRATEGY);
    let strat = cfg
        .strategies()
        .get(name)
        .ok_or_else(|| OpError::Invalid(format!("unknown strategy `{name}`")))?
        .clone();
    let model = state
        .layer::<ModelLayer>()
        .ok_or_else(|| OpError::MissingLayer("model".into()))?;
    let intent = state
        .layer::<IntentLayer>()
        .ok_or_else(|| OpError::MissingLayer("intent".into()))?
        .intent();
    let meshes = state
        .layer::<MeshLayer>()
        .ok_or_else(|| OpError::MissingLayer("meshes".into()))?;
    let mut vols: Vec<EntityId> = match volumes {
        Some(v) => v.to_vec(),
        None => model
            .topo()
            .cells(Dim::Volume)
            .iter()
            .copied()
            .filter(|&v| meshes.mesh_of(v).is_none())
            .collect(),
    };
    vols.sort_unstable();
    vols.dedup();
    let mut rows = Vec::with_capacity(vols.len());
    for v in vols {
        let one = [v];
        let alloc = meshes.alloc().clone();
        let t = Target::Volume(v);
        let (before, before_plan) = outcome(
            scratch_plan(cfg, model, intent, &one, false, &alloc, None),
            t,
        );
        let input = AutoInput::new(cfg, model, intent, &one, false, alloc.clone());
        let (after, faces_changed) = match strat.choose(&input) {
            Err(e) => (
                CensusOutcome {
                    scheme: None,
                    selection: None,
                    error: Some(e.to_string()),
                },
                Vec::new(),
            ),
            Ok(asg) => {
                // The intent `mesh.auto` would store, then the plan `mesh.run` would make.
                let mut i2 = intent.clone();
                for a in &asg {
                    let c = match &a.params {
                        Some(p) => {
                            SchemeChoice::with_params(&a.scheme, p.0.clone(), Firmness::Hard)
                        }
                        None => SchemeChoice::new(&a.scheme, Firmness::Hard),
                    };
                    i2.entity_mut(a.target).scheme = Some(c);
                }
                let (o, p2) = outcome(scratch_plan(cfg, model, &i2, &one, false, &alloc, None), t);
                let mut diff = Vec::new();
                if let (Some(b), Some(a)) = (&before_plan, &p2) {
                    for r in &a.problem.targets {
                        if let Target::Surface(f) = r.target
                            && let Some(old) =
                                b.problem.targets.iter().find(|x| x.target == r.target)
                            && old.scheme != r.scheme
                        {
                            diff.push((
                                f,
                                old.scheme.as_str().to_owned(),
                                r.scheme.as_str().to_owned(),
                            ));
                        }
                    }
                }
                (o, diff)
            }
        };
        let changed = before.scheme != after.scheme || before.error != after.error;
        rows.push(CensusRow {
            volume: v,
            before,
            after,
            faces_changed,
            changed,
        });
    }
    Ok(CensusReport {
        strategy: name.to_owned(),
        rows,
    })
}
