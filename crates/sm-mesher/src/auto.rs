//! Pluggable automatic scheme selection: [`AutoStrategy`], [`DefaultStrategy`], the
//! `mesh.auto` op and the `mesh.auto_strategies` query.
//!
//! `mesh.auto` runs a named strategy **once**, at apply, and stores its answer as explicit
//! assignments (`mesh.auto {strategy, assignments}`) through the canonical-record hook. Replay
//! applies the stored assignments and never calls a strategy, so changing or removing a strategy
//! cannot move the hash of a recorded timeline.

use std::collections::BTreeMap;
use std::sync::Arc;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, EntityKind};
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh::MeshBuilder;
use sm_mesh_api::SerialAlloc;
use sm_op_api::{ApplyCtx, OpError, OpRecord, QueryCtx, State, TypedOp, TypedQuery};
use sm_scheme_api::{
    Attempt, AutoPolicy, Driver, FailureKind, Firmness, MeshInput, MeshIntent, Params, Phase, Plan,
    Rejections, SchemeChoice, SchemeDim, SchemeError, SchemeName, Selection, Target, WholeFaces,
};
use sm_topo_api::{Dim, TopologyView};

use crate::config::MesherConfig;
use crate::exec::RayonExec;
use crate::intent_layer::IntentLayer;
use crate::intent_ops::{finish, live_targets, store_scheme, warn_kept};
use crate::mesh_ops::{estimate_face_cells, faces_mode, failed, make_driver, size_field};

/// Name of the strategy `mesh.auto` uses when none is given.
pub const DEFAULT_STRATEGY: &str = "default";

/// Replan budget for a run over `n_targets` targets: `16 + 2 * n_targets`.
///
/// Each target that has to fall back costs one replan (the driver reports the first failed
/// job per replan), so a fixed 16 would make a 40-volume model fail.
pub const fn replan_budget(n_targets: usize) -> usize {
    16 + 2 * n_targets
}

/// One explicit scheme choice for one target, as stored in a `mesh.auto` record.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SchemeAssignment {
    /// The face or volume.
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace"]))]
    pub target: EntityId,
    /// Registered scheme name.
    pub scheme: String,
    /// Scheme parameters.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub params: Option<Params>,
    /// Set by a strategy that placed a tet mesher after passing over earlier candidates: why
    /// each was passed over. Never stored; it only feeds the `mesh.auto_tet` warning.
    #[serde(skip)]
    #[schemars(skip)]
    pub tet_reasons: Option<Vec<String>>,
}

impl SchemeAssignment {
    /// An assignment with no parameters.
    pub fn new(target: EntityId, scheme: &str) -> Self {
        Self {
            target,
            scheme: scheme.to_owned(),
            params: None,
            tet_reasons: None,
        }
    }
}

/// What a strategy sees: the model, the recorded intent, the targets and the mesher service.
pub struct AutoInput<'a> {
    cfg: &'a MesherConfig,
    model: &'a ModelLayer,
    intent: &'a MeshIntent,
    targets: &'a [EntityId],
    faces: bool,
    alloc: SerialAlloc,
}

impl<'a> AutoInput<'a> {
    pub(crate) fn new(
        cfg: &'a MesherConfig,
        model: &'a ModelLayer,
        intent: &'a MeshIntent,
        targets: &'a [EntityId],
        faces: bool,
        alloc: SerialAlloc,
    ) -> Self {
        Self {
            cfg,
            model,
            intent,
            targets,
            faces,
            alloc,
        }
    }
}

impl AutoInput<'_> {
    /// The mesher service (scheme registry, solver, picker, hooks).
    pub fn config(&self) -> &MesherConfig {
        self.cfg
    }
    /// The topology.
    pub fn topo(&self) -> &dyn TopologyView {
        self.model.topo()
    }
    /// The recorded intent (schemes already chosen win over any strategy).
    pub fn intent(&self) -> &MeshIntent {
        self.intent
    }
    /// The volumes, or the faces, to assign (sorted, de-duplicated, one kind).
    pub fn targets(&self) -> &[EntityId] {
        self.targets
    }
    /// The targets are faces.
    pub fn is_faces(&self) -> bool {
        self.faces
    }
    /// [`replan_budget`] for these targets.
    pub fn replan_budget(&self) -> usize {
        replan_budget(self.targets.len())
    }
}

/// A policy that picks the schemes of targets that have none.
///
/// A strategy is consulted only by `mesh.auto` at apply time; its answer is stored, so it may
/// change between releases without moving the hash of an existing timeline. It must be
/// deterministic in its input.
pub trait AutoStrategy: Send + Sync {
    /// Registry name (`mesh.auto {strategy}`).
    fn name(&self) -> &'static str;
    /// One line for `mesh.auto_strategies`.
    fn description(&self) -> &'static str {
        ""
    }
    /// One assignment per target (face or volume) that had no intent scheme.
    fn choose(&self, input: &AutoInput<'_>) -> Result<Vec<SchemeAssignment>, SchemeError>;
}

/// Candidate lists of a list-driven strategy: the list is the candidate set and its order the
/// tie-break between equal scores.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AutoLists {
    /// Curve candidates.
    pub curve: Vec<String>,
    /// Surface candidates.
    pub surface: Vec<String>,
    /// Volume candidates.
    pub volume: Vec<String>,
}

impl AutoLists {
    /// The standard lists: surface `[map, submap, pave, trimesh]`, volume `[map, sweep,
    /// tetmesh]`, curve `[equal]`.
    pub fn standard() -> Self {
        let v = |a: &[&str]| a.iter().map(|s| (*s).to_owned()).collect();
        Self {
            curve: v(&["equal"]),
            surface: v(&["map", "submap", "pave", "trimesh"]),
            volume: v(&["map", "sweep", "tetmesh"]),
        }
    }
}

/// Plan with the replan loop applied at plan time. Returns the plan and the number of
/// rejections tried (they share `max_replans` with generation-time ones).
///
/// `Driver::plan` fails with `NoApplicableScheme` when the imposition of a neighbour cannot be
/// met (a tri face next to a `map` volume that imposes quad). The error lists the imposing
/// parents that can be re-planned, in sorted order; each is tried in turn: its rejection is
/// kept if the replan gets past this target, and dropped if the same target fails again (or
/// the rejected scheme's own target runs out of candidates), so the culprit need not be the
/// first entry. Every try counts against the budget. With a
/// budget of 0 the planner's own error is returned unchanged.
pub(crate) fn plan_with_blame(
    driver: &Driver<'_>,
    input: &MeshInput<'_>,
    intent: &MeshIntent,
) -> Result<(Plan, usize), SchemeError> {
    let budget = driver.options.max_replans;
    let mut rej = Rejections::default();
    let mut used = 0usize;
    let mut history: Vec<Attempt> = Vec::new();
    loop {
        let err = match driver.plan_with(input, intent, &rej) {
            Ok(plan) => return Ok((plan, used)),
            Err(e) => e,
        };
        let SchemeError::NoApplicableScheme {
            target, imposed_by, ..
        } = &err
        else {
            return Err(err);
        };
        let cands: Vec<&(Target, SchemeName)> = imposed_by
            .iter()
            .filter(|(t, s)| s.as_str() != SchemeName::COPY && !rej.contains(*t, s))
            .collect();
        let mut next: Option<Rejections> = None;
        for (i, (t, s)) in cands.iter().map(|c| (&c.0, &c.1)).enumerate() {
            if used >= budget {
                return Err(if used == 0 {
                    err
                } else {
                    SchemeError::ReplanLimit { attempts: history }
                });
            }
            used += 1;
            let a = Attempt::new(
                *t,
                s.clone(),
                Phase::Resolve,
                FailureKind::NotApplicable,
                err.to_string(),
            );
            let mut trial = rej.clone();
            trial.insert(a.clone());
            match driver.plan_with(input, intent, &trial) {
                Ok(plan) => return Ok((plan, used)),
                Err(SchemeError::NoApplicableScheme { target: t2, .. })
                    if (t2 == *target || t2 == *t) && i + 1 < cands.len() =>
                {
                    history.push(a);
                }
                Err(_) => {
                    history.push(a);
                    next = Some(trial);
                    break;
                }
            }
        }
        match next {
            Some(n) => rej = n,
            None => return Err(err),
        }
    }
}

/// Plan and mesh `targets` into a scratch mesh (nothing is stored) and return the final plan and
/// every attempt passed over or rejected on the way. This is what finds fallbacks that only
/// show at generation time.
pub(crate) fn scratch_plan(
    cfg: &MesherConfig,
    model: &ModelLayer,
    intent: &MeshIntent,
    targets: &[EntityId],
    faces: bool,
    alloc: &SerialAlloc,
    fixed_budget: Option<usize>,
) -> Result<(Plan, Vec<Attempt>), SchemeError> {
    let field = size_field(cfg, intent).map_err(|e| SchemeError::Input(e.to_string()))?;
    let mut driver = make_driver(cfg, field.as_deref(), faces, targets.len());
    if let Some(n) = fixed_budget {
        driver.options.max_replans = n;
    }
    let mi = MeshInput::new(model.topo(), model.model(), &WholeFaces, targets);
    let (plan, used) = plan_with_blame(&driver, &mi, intent)?;
    driver.options.max_replans = driver.options.max_replans.saturating_sub(used);
    if faces {
        let cells = estimate_face_cells(&plan, model.topo(), targets);
        if cells > cfg.max_cells() as f64 {
            return Err(SchemeError::OverBudget {
                cells,
                max: cfg.max_cells(),
            });
        }
    }
    let builder = if faces {
        MeshBuilder::surface()
    } else {
        MeshBuilder::new()
    };
    let mut alloc = alloc.clone();
    let meshed = driver.mesh_from_plan(&mi, intent, plan, &RayonExec, builder, &mut alloc)?;
    Ok((meshed.plan, meshed.attempts))
}

/// The `"default"` strategy: scores the candidate lists with the planner and falls back through
/// its replan loop.
///
/// It plans and meshes into a scratch mesh with the lists installed as the auto policy: the
/// planner scores every candidate (ties go to the earlier list entry), a candidate that fails to
/// apply or generate is rejected and the problem re-planned, up to [`replan_budget`]. The
/// winners are read from the final plan. Only rows chosen by auto-selection
/// (`Selection::Auto`) become assignments: targets with an intent scheme, faces that a parent
/// imposes or claims, copies and curves are left alone. Names not registered are dropped from
/// the lists.
#[derive(Debug, Clone)]
pub struct DefaultStrategy {
    lists: AutoLists,
    tet: &'static str,
    fixed_budget: Option<usize>,
}

impl Default for DefaultStrategy {
    fn default() -> Self {
        Self::with_lists(AutoLists::standard())
    }
}

impl DefaultStrategy {
    /// A strategy over `lists`; a volume given the scheme `tetmesh` is reported as a tet
    /// fallback.
    pub fn with_lists(lists: AutoLists) -> Self {
        Self {
            lists,
            tet: "tetmesh",
            fixed_budget: None,
        }
    }
    /// Name the scheme that counts as the tet fallback (for `mesh.auto_tet`).
    #[must_use]
    pub fn with_tet_scheme(mut self, name: &'static str) -> Self {
        self.tet = name;
        self
    }
    /// Use a fixed replan budget instead of [`replan_budget`] (negative controls).
    #[must_use]
    pub fn with_fixed_budget(mut self, n: usize) -> Self {
        self.fixed_budget = Some(n);
        self
    }
    /// The candidate lists.
    pub fn lists(&self) -> &AutoLists {
        &self.lists
    }

    fn policy(&self, cfg: &MesherConfig) -> AutoPolicy {
        let reg = cfg.registry();
        let keep = |dim: SchemeDim, names: &[String]| -> Vec<SchemeName> {
            names
                .iter()
                .filter(|n| reg.get(dim, n).is_some())
                .map(|n| SchemeName::new(n))
                .collect()
        };
        AutoPolicy::new(
            keep(SchemeDim::Curve, &self.lists.curve),
            keep(SchemeDim::Surface, &self.lists.surface),
            keep(SchemeDim::Volume, &self.lists.volume),
        )
    }
}

impl AutoStrategy for DefaultStrategy {
    fn name(&self) -> &'static str {
        DEFAULT_STRATEGY
    }

    fn description(&self) -> &'static str {
        "planner scores over surface [map, submap, pave, trimesh] and volume [map, sweep, \
         tetmesh]; list order breaks ties; failed candidates fall back through the replan loop"
    }

    fn choose(&self, input: &AutoInput<'_>) -> Result<Vec<SchemeAssignment>, SchemeError> {
        let cfg = input.cfg;
        let mut intent = input.intent.clone();
        intent.defaults.auto = self.policy(cfg);
        let (plan, attempts) = scratch_plan(
            cfg,
            input.model,
            &intent,
            input.targets,
            input.faces,
            &input.alloc,
            self.fixed_budget,
        )?;
        let mut out = Vec::new();
        for row in &plan.problem.targets {
            if !matches!(row.selection, Selection::Auto { .. }) {
                continue;
            }
            let (id, want) = match row.target {
                Target::Surface(f) => (f, EntityKind::TopoFace),
                Target::Volume(v) => (v, EntityKind::TopoVolume),
                _ => continue,
            };
            if id.kind() != want {
                continue;
            }
            let mut a = SchemeAssignment::new(id, row.scheme.as_str());
            if !row.params.is_empty() {
                a.params = Some(row.params.clone());
            }
            if want == EntityKind::TopoVolume && row.scheme.as_str() == self.tet {
                let mut why: Vec<String> = Vec::new();
                for at in &attempts {
                    if at.target == row.target && at.scheme != row.scheme {
                        let s = format!("{}: {}", at.scheme, at.message);
                        if !why.contains(&s) {
                            why.push(s);
                        }
                    }
                }
                a.tet_reasons = Some(why);
            }
            out.push(a);
        }
        out.sort_by_key(|a| a.target);
        Ok(out)
    }
}

/// The strategies a [`MesherConfig`] offers, by name.
#[derive(Clone)]
pub struct StrategyRegistry {
    by_name: BTreeMap<&'static str, Arc<dyn AutoStrategy>>,
}

impl Default for StrategyRegistry {
    /// Holds [`DefaultStrategy`].
    fn default() -> Self {
        let mut r = Self {
            by_name: BTreeMap::new(),
        };
        r.insert(Arc::new(DefaultStrategy::default()));
        r
    }
}

impl StrategyRegistry {
    /// Register `s`, replacing any strategy of the same name.
    pub fn insert(&mut self, s: Arc<dyn AutoStrategy>) {
        self.by_name.insert(s.name(), s);
    }
    /// The strategy called `name`.
    pub fn get(&self, name: &str) -> Option<&Arc<dyn AutoStrategy>> {
        self.by_name.get(name)
    }
    /// Names and descriptions, ascending by name.
    pub fn list(&self) -> Vec<(&'static str, &'static str)> {
        self.by_name
            .iter()
            .map(|(n, s)| (*n, s.description()))
            .collect()
    }
}

/// `mesh.auto`: let a strategy choose schemes for targets that have none, and store its choices.
///
/// Give `strategy` (default `"default"`) and optionally `targets` (volumes or faces, not both;
/// default every unmeshed volume). The strategy runs now and the op is stored as
/// `{strategy, assignments: [{target, scheme, params}]}`; the effect equals one `mesh.scheme`
/// per assignment. A record that already holds `assignments` (replay, or a hand-written one)
/// applies them as given and calls no strategy. Targets that already have an intent scheme keep
/// it. Emits `mesh.auto_tet` for each volume given a tet mesher, with the reasons the earlier
/// candidates were passed over.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshAuto {
    /// Strategy name (default `"default"`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub strategy: Option<String>,
    /// Volumes or faces to assign (default: every unmeshed volume). Not allowed with
    /// `assignments`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace"]))]
    pub targets: Option<Vec<EntityId>>,
    /// Explicit assignments to apply without consulting a strategy.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub assignments: Option<Vec<SchemeAssignment>>,
}

fn dim_of(id: EntityId) -> SchemeDim {
    if id.kind() == EntityKind::TopoFace {
        SchemeDim::Surface
    } else {
        SchemeDim::Volume
    }
}

impl TypedOp for MeshAuto {
    const NAME: &'static str = "mesh.auto";
    const NO_OUTPUTS: bool = true;

    fn apply(&self, ctx: &mut ApplyCtx<'_>) -> Result<(), OpError> {
        let cfg = ctx.service::<MesherConfig>()?.clone();
        let strategy_name = self.strategy.as_deref().unwrap_or(DEFAULT_STRATEGY);
        let replay = self.assignments.is_some();
        let mut assignments = match &self.assignments {
            Some(a) => {
                if self.targets.is_some() {
                    return Err(OpError::Invalid(
                        "give `targets` or `assignments`, not both".into(),
                    ));
                }
                a.clone()
            }
            None => {
                let strategy = cfg
                    .strategies()
                    .get(strategy_name)
                    .cloned()
                    .ok_or_else(|| {
                        let names: Vec<_> = cfg
                            .strategies()
                            .list()
                            .into_iter()
                            .map(|(n, _)| n)
                            .collect();
                        OpError::Invalid(format!(
                            "unknown strategy `{strategy_name}`; known: {}",
                            names.join(", ")
                        ))
                    })?;
                let model = ctx.layer::<ModelLayer>()?.clone();
                let intent = ctx.layer::<IntentLayer>()?.intent().clone();
                let alloc = ctx.layer::<MeshLayer>()?.alloc().clone();
                let targets = match &self.targets {
                    Some(t) => {
                        live_targets(ctx, t, &[EntityKind::TopoVolume, EntityKind::TopoFace])?
                    }
                    None => {
                        let meshes = ctx.layer::<MeshLayer>()?;
                        let mut v: Vec<EntityId> = model
                            .topo()
                            .cells(Dim::Volume)
                            .iter()
                            .copied()
                            .filter(|&v| meshes.mesh_of(v).is_none())
                            .collect();
                        v.sort_unstable();
                        v
                    }
                };
                if targets.is_empty() {
                    return Err(OpError::Invalid("no unmeshed volumes to assign".into()));
                }
                let faces = faces_mode(&targets)?;
                let input = AutoInput::new(&cfg, &model, &intent, &targets, faces, alloc);
                strategy.choose(&input).map_err(failed)?
            }
        };
        assignments.sort_by_key(|a| a.target);
        if assignments.windows(2).any(|w| w[0].target == w[1].target) {
            return Err(OpError::Invalid("two assignments for one target".into()));
        }
        // Validate everything before writing anything.
        let kinds = [EntityKind::TopoVolume, EntityKind::TopoFace];
        let mut items = Vec::with_capacity(assignments.len());
        for a in &assignments {
            let t = live_targets(ctx, &[a.target], &kinds)?[0];
            let dim = dim_of(t);
            if a.scheme != SchemeName::COPY && cfg.registry().get(dim, &a.scheme).is_none() {
                return Err(OpError::Invalid(format!(
                    "no {dim:?} scheme named `{}` (target {t})",
                    a.scheme
                )));
            }
            let choice = match &a.params {
                Some(p) if !p.is_empty() => {
                    p.check().map_err(|e| OpError::Invalid(e.to_string()))?;
                    SchemeChoice::with_params(&a.scheme, p.0.clone(), Firmness::Hard)
                }
                _ => SchemeChoice::new(&a.scheme, Firmness::Hard),
            };
            items.push((t, choice));
        }
        let layer = ctx.layer_mut::<IntentLayer>()?;
        let mut kept = Vec::new();
        for (t, choice) in &items {
            if !store_scheme(layer, *t, choice, &[], Firmness::Hard) {
                kept.push(*t);
            }
        }
        finish(layer)?;
        warn_kept(ctx, Self::NAME, &kept);
        for a in &assignments {
            if let Some(why) = &a.tet_reasons {
                let mut msg = format!(
                    "{} was given a tet mesh by strategy `{strategy_name}`",
                    a.target
                );
                if why.is_empty() {
                    msg.push_str(" (no earlier candidate applied)");
                } else {
                    msg.push_str(&format!("; passed over: {}", why.join("; ")));
                }
                msg.push_str(&format!(
                    ". To keep hexes: `mesh.scheme {} sweep` or use a strategy without tets",
                    a.target
                ));
                ctx.warn("mesh.auto_tet", msg, &[a.target]);
            }
        }
        if !replay {
            let stored = MeshAuto {
                strategy: Some(strategy_name.to_owned()),
                targets: None,
                assignments: Some(assignments),
            };
            let rec = OpRecord::of(&stored).map_err(|e| OpError::Invalid(e.to_string()))?;
            ctx.set_canonical_record(rec);
        }
        Ok(())
    }
}

/// `mesh.auto_strategies`: the registered strategies.
#[derive(Debug, Clone, Default, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshAutoStrategiesQuery {}

/// One row of [`StrategiesInfo`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct StrategyInfo {
    /// Name for `mesh.auto {strategy}`.
    pub name: String,
    /// One line.
    pub description: String,
}

/// The reply of [`MeshAutoStrategiesQuery`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct StrategiesInfo {
    /// Strategies, ascending by name.
    pub strategies: Vec<StrategyInfo>,
}

impl TypedQuery for MeshAutoStrategiesQuery {
    const NAME: &'static str = "mesh.auto_strategies";
    type Output = StrategiesInfo;

    fn run(&self, _state: &State, cx: &QueryCtx<'_>) -> Result<StrategiesInfo, OpError> {
        let cfg = cx.service::<MesherConfig>()?;
        Ok(StrategiesInfo {
            strategies: cfg
                .strategies()
                .list()
                .into_iter()
                .map(|(n, d)| StrategyInfo {
                    name: n.to_owned(),
                    description: d.to_owned(),
                })
                .collect(),
        })
    }
}
