//! The `mesh.plan` query: explain what ran (or what would run) and why.

use std::collections::BTreeMap;

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use sm_core::EntityId;
use sm_layer_api::{MeshLayer, ModelLayer};
use sm_mesh_api::{CellIx, CellShape, MeshDim, MeshRead};
use sm_op_api::{OpError, QueryCtx, State, TypedQuery};
use sm_scheme_api::{
    Caps, Driver, Firmness, IntervalConstraint, MeshInput, Origin, Plan, Record, SchemeError,
    Selection, Target, TargetPlan, WholeFaces,
};
use sm_topo_api::TopologyView;

use crate::config::MesherConfig;
use crate::intent_layer::IntentLayer;
use crate::mesh_ops::{estimate_face_cells, faces_mode, make_driver, run_guards, size_field};

/// `mesh.plan`: explain a mesh run. Read-only; no id is minted and nothing is generated.
///
/// Give exactly one of `mesh` and `targets`.
/// * `mesh`: what **ran**, from the plan stored with the mesh (`status: ran`).
/// * `targets` (volumes or faces, not both): what **will run**: the same plan `mesh.run` would
///   make on this state (`status: will_run`). A planning failure comes back as
///   `status: infeasible` with the error text and its blame, not as a query error, and the
///   reasons `mesh.run` would refuse the targets are listed in `blockers`.
#[derive(Debug, Clone, Serialize, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshPlanQuery {
    /// Explain this mesh (what ran).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["Mesh"]))]
    pub mesh: Option<EntityId>,
    /// Explain a run of these volumes or faces (what will run).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    #[schemars(extend("x-entity-kind" = ["TopoVolume", "TopoFace"]))]
    pub targets: Option<Vec<EntityId>>,
    /// List the candidates that lost, per entity (default true).
    #[serde(default = "yes")]
    pub attempts: bool,
}

fn yes() -> bool {
    true
}

/// Whether the plan ran, will run, or cannot be made.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum PlanStatus {
    /// The report describes an existing mesh.
    Ran,
    /// The report describes the plan `mesh.run` would use now.
    WillRun,
    /// Planning failed; see [`PlanReport::failure`].
    Infeasible,
}

/// Dimension of the mesh the plan makes: 2 (faces only) or 3.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlanDim {
    /// A surface mesh.
    Two,
    /// A volume mesh.
    Three,
}

impl Serialize for PlanDim {
    fn serialize<S: serde::Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_u8(match self {
            PlanDim::Two => 2,
            PlanDim::Three => 3,
        })
    }
}

impl<'de> Deserialize<'de> for PlanDim {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        match u8::deserialize(d)? {
            2 => Ok(PlanDim::Two),
            3 => Ok(PlanDim::Three),
            n => Err(serde::de::Error::custom(format!("dim {n} is not 2 or 3"))),
        }
    }
}

impl JsonSchema for PlanDim {
    fn schema_name() -> std::borrow::Cow<'static, str> {
        "PlanDim".into()
    }
    fn json_schema(_: &mut schemars::SchemaGenerator) -> schemars::Schema {
        schemars::json_schema!({"type": "integer", "enum": [2, 3]})
    }
}

/// Why a planning attempt failed (`status: infeasible`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct PlanFailure {
    /// The full error text.
    pub message: String,
    /// For an infeasible interval system: what could not be satisfied.
    pub reason: Option<String>,
    /// Ids of the interval constraints involved.
    pub constraints: Vec<u32>,
    /// Entities involved, ascending.
    pub entities: Vec<EntityId>,
    /// The `target (scheme)` the driver would blame and re-plan around, if any.
    pub blame: Option<String>,
}

/// A candidate scheme that lost or failed for an entity.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Alternative {
    /// The candidate scheme.
    pub scheme: String,
    /// Where it was rejected (`Resolve`, `Intervals`, `Generate`, ...).
    pub phase: String,
    /// The kind of failure (`NotApplicable`, `Rejected`, `Constraint`, ...).
    pub kind: String,
    /// Detail.
    pub message: String,
}

/// Cells per shape (a `ran` report) or the element type the caps give (a `will_run` report).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(untagged)]
pub enum ElementsInfo {
    /// Cells classified on the entity by shape: `{"Quad": 6}`.
    Counts(BTreeMap<String, usize>),
    /// From the scheme's capabilities: `"QUAD4"`, `"TRI3"`, `"HEX8"`, ..., or `"mixed"`.
    Kind(String),
}

/// The scheme that meshes one entity and how it was chosen.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct EntityPlan {
    /// The target (`TopoFace#5`, or `Group#n`).
    pub target: String,
    /// `curve`, `surface`, `volume` or `group`.
    pub kind: String,
    /// The scheme (`copy` for copy-class members).
    pub scheme: String,
    /// `intent#rank`, `registry#rank`, `piece_shape`, `imposed`, `auto`, `copy` or `claimed`.
    pub how: String,
    /// The winning score, when the selection has one.
    pub score: Option<u32>,
    /// The winner's applicability text, verbatim.
    pub reason: String,
    /// The scheme parameters.
    pub params: serde_json::Value,
    /// Capabilities of the result (`tri`, `quad`, `structured`, ...).
    pub caps: Vec<String>,
    /// The parent scheme that writes this entity, if it is claimed.
    pub claimed_by: Option<String>,
    /// The root of the copy class this entity is a member of.
    pub copy_of: Option<String>,
    /// Cells per shape (ran) or the element type (will_run); `None` for curves and groups
    /// where it does not apply.
    pub elements: Option<ElementsInfo>,
    /// Number of cells classified on the entity (ran only).
    pub cells: Option<usize>,
    /// Candidates that lost, in evaluation order: failures that forced a re-plan first, then
    /// the candidates the final plan passed over. Empty when `attempts` is false.
    pub alternatives: Vec<Alternative>,
}

/// The source that decided an edge's interval count.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum WhyKind {
    /// A hard count (or size) the user asked for.
    HardIntent,
    /// A soft count (or size) the user asked for.
    SoftIntent,
    /// A scheme's own requirement or preference (names the scheme and target).
    Scheme,
    /// A deck boundary (matching or periodic faces).
    Deck,
    /// A size goal: `mesh.size`, or the default sizing (carries the goal).
    SizingGoal,
    /// No request reached the edge: the built-in default goal.
    Default,
    /// Solved from linear rows (opposite sides, sweep layers, ...); lists their ids.
    Derived,
    /// A decomposition piece shape hint.
    Piece,
}

/// One interval request on an edge.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct IntervalRequestRow {
    /// Constraint id within the plan's problem.
    pub id: u32,
    /// Who contributed it (`intent`, `sizing`, `scheme map on TopoFace#5`, ...).
    pub origin: String,
    /// `fixed`, `goal`, `range`, `linear` or `even_sum`.
    pub kind: String,
    /// Firmness of a fixed count.
    pub firmness: Option<String>,
    /// The requested count or goal.
    pub value: Option<f64>,
    /// A short text of the row.
    pub detail: String,
    /// The solved count is what this row asks for (or inside its bounds).
    pub agrees: bool,
    /// This row decided the count.
    pub winner: bool,
}

/// The solved interval count of one edge and why it is that.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct IntervalWhy {
    /// The edge.
    pub edge: EntityId,
    /// Solved count.
    pub count: u32,
    /// The deciding source.
    pub why: WhyKind,
    /// The goal, for `sizing_goal`.
    pub goal: Option<f64>,
    /// The scheme, for `scheme`.
    pub scheme: Option<String>,
    /// The target the scheme was planning, for `scheme`.
    pub target: Option<String>,
    /// The linear rows the count was solved from, for `derived`.
    pub constraints: Vec<u32>,
    /// Every request that reaches the edge. When two sources disagree both are listed and
    /// the winner is marked.
    pub requests: Vec<IntervalRequestRow>,
}

/// A located scheme diagnostic (mirror of `SchemeNote`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct PlanNote {
    /// Where the finding is.
    pub position: [f64; 3],
    /// Model entities near it.
    pub entities: Vec<EntityId>,
    /// The measure out of bounds.
    pub metric: String,
    /// Measured value.
    pub value: f64,
    /// The bound it violates.
    pub threshold: f64,
    /// What would probably cure it.
    pub suggestion: String,
}

/// The reply of [`MeshPlanQuery`]. A pure function of `(state, request)`: byte-identical for
/// every thread count.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct PlanReport {
    /// `ran`, `will_run` or `infeasible`.
    pub status: PlanStatus,
    /// The mesh (`ran` only).
    pub mesh: Option<EntityId>,
    /// The mesh's selection changed or was deleted after meshing (`ran` only).
    pub stale: bool,
    /// 2 for a face run, 3 for a volume run.
    pub dim: PlanDim,
    /// Volumes or faces meshed (or to mesh).
    pub selection: Vec<EntityId>,
    /// Rough cell count (`will_run` only; see the budget in DESIGN.md).
    pub estimated_cells: Option<u64>,
    /// Number of generation waves.
    pub waves: usize,
    /// 16 hex digits of a stable hash of the serialized plan.
    pub plan_hash: Option<String>,
    /// Reasons `mesh.run` would refuse these targets now (the same texts as its errors).
    pub blockers: Vec<String>,
    /// Why planning failed (`infeasible` only).
    pub failure: Option<PlanFailure>,
    /// One row per planned entity: curves, surfaces, volumes, groups, ascending.
    pub entities: Vec<EntityPlan>,
    /// One row per edge, ascending.
    pub intervals: Vec<IntervalWhy>,
    /// Located scheme diagnostics of the mesh (`ran` only).
    pub notes: Vec<PlanNote>,
}

fn target_str(t: Target) -> String {
    match t.entity() {
        Some(e) => e.to_string(),
        None => match t {
            Target::Group(g) => format!("Group#{}", g.0),
            _ => "?".into(),
        },
    }
}

fn kind_str(t: Target) -> &'static str {
    match t {
        Target::Curve(_) => "curve",
        Target::Surface(_) => "surface",
        Target::Volume(_) => "volume",
        _ => "group",
    }
}

fn caps_names(c: Caps) -> Vec<String> {
    [
        (Caps::TRI, "tri"),
        (Caps::QUAD, "quad"),
        (Caps::STRUCTURED, "structured"),
        (Caps::TET, "tet"),
        (Caps::HEX, "hex"),
        (Caps::WEDGE, "wedge"),
        (Caps::PYRAMID, "pyramid"),
        (Caps::POLY, "poly"),
        (Caps::BOUNDARY_LAYER, "boundary_layer"),
        (Caps::MESHES_CHILDREN, "meshes_children"),
        (Caps::BLOCK_STRUCTURED, "block_structured"),
        (Caps::INHERIT, "inherit"),
    ]
    .into_iter()
    .filter(|(b, _)| c.contains(*b))
    .map(|(_, n)| n.to_owned())
    .collect()
}

/// Element type a scheme's caps promise for a surface or volume target.
fn caps_element(c: Caps, kind: &str) -> Option<ElementsInfo> {
    let names: &[(Caps, &str)] = match kind {
        "surface" => &[(Caps::TRI, "TRI3"), (Caps::QUAD, "QUAD4")],
        "volume" | "group" => &[
            (Caps::TET, "TET4"),
            (Caps::HEX, "HEX8"),
            (Caps::WEDGE, "WEDGE6"),
            (Caps::PYRAMID, "PYRAMID5"),
        ],
        _ => return None,
    };
    let hit: Vec<&str> = names
        .iter()
        .filter(|(b, _)| c.contains(*b))
        .map(|(_, n)| *n)
        .collect();
    match hit.as_slice() {
        [] if c.contains(Caps::POLY) => Some(ElementsInfo::Kind("mixed".into())),
        [] => None,
        [one] if !c.contains(Caps::POLY) => Some(ElementsInfo::Kind((*one).into())),
        _ => Some(ElementsInfo::Kind("mixed".into())),
    }
}

fn how_of(s: &Selection) -> (String, Option<u32>) {
    match s {
        Selection::Intent { rank } => (format!("intent#{rank}"), None),
        Selection::Registry { rank } => (format!("registry#{rank}"), None),
        Selection::PieceShape => ("piece_shape".into(), None),
        Selection::Imposed { score } => ("imposed".into(), Some(*score)),
        Selection::Auto { score } => ("auto".into(), Some(*score)),
        Selection::Copy { .. } => ("copy".into(), None),
        Selection::Claimed { .. } => ("claimed".into(), None),
        _ => ("other".into(), None),
    }
}

fn origin_str(o: &Origin) -> String {
    match o {
        Origin::Intent => "intent".into(),
        Origin::Sizing => "sizing".into(),
        Origin::Scheme { scheme, target } => {
            format!("scheme {} on {}", scheme.as_str(), target_str(*target))
        }
        Origin::Preference { scheme, target } => {
            format!(
                "preference of {} on {}",
                scheme.as_str(),
                target_str(*target)
            )
        }
        Origin::Deck { boundary } => format!("deck {boundary}"),
        Origin::Piece { piece } => format!("piece {piece}"),
        Origin::Driver => "driver".into(),
        _ => "other".into(),
    }
}

/// Stable 64-bit FNV-1a, 16 hex digits.
fn fnv_hex(bytes: &[u8]) -> String {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    format!("{h:016x}")
}

fn alternatives_of(plan: &Plan, t: Target) -> Vec<Alternative> {
    let mut rows: Vec<&sm_scheme_api::Attempt> = Vec::new();
    // Failures that forced a re-plan happened first.
    for a in plan.rejections.attempts().chain(plan.attempts.iter()) {
        if a.target == t && !rows.contains(&a) {
            rows.push(a);
        }
    }
    rows.into_iter()
        .map(|a| Alternative {
            scheme: a.scheme.as_str().to_owned(),
            phase: format!("{:?}", a.phase),
            kind: format!("{:?}", a.kind),
            message: a.message.clone(),
        })
        .collect()
}

fn classified_elements(m: &dyn MeshRead, t: Target) -> (Option<ElementsInfo>, Option<usize>) {
    let Some(e) = t.entity() else {
        return (None, None);
    };
    let Some(cs) = m.classified_on(e) else {
        return (None, None);
    };
    match t {
        Target::Curve(_) => {
            let n = if !cs.edges.is_empty() {
                cs.edges.len()
            } else if m.dim() == MeshDim::Two {
                cs.faces.len()
            } else {
                return (None, None);
            };
            (
                Some(ElementsInfo::Counts(BTreeMap::from([(
                    "Edge".to_owned(),
                    n,
                )]))),
                Some(n),
            )
        }
        _ => {
            let mut by: BTreeMap<CellShape, usize> = BTreeMap::new();
            for &c in &cs.cells {
                *by.entry(m.cell_shape(CellIx(c.0))).or_default() += 1;
            }
            let counts = by.into_iter().map(|(k, v)| (format!("{k:?}"), v)).collect();
            (Some(ElementsInfo::Counts(counts)), Some(cs.cells.len()))
        }
    }
}

fn entity_row(plan: &Plan, tp: &TargetPlan, mesh: Option<&dyn MeshRead>) -> EntityPlan {
    let t = tp.target;
    let (how, score) = how_of(&tp.selection);
    let kind = kind_str(t);
    let (elements, cells) = match mesh {
        Some(m) => classified_elements(m, t),
        None => (caps_element(tp.caps, kind), None),
    };
    EntityPlan {
        target: target_str(t),
        kind: kind.into(),
        scheme: tp.scheme.as_str().to_owned(),
        how,
        score,
        reason: tp.reason.clone(),
        params: tp.params.0.clone(),
        caps: caps_names(tp.caps),
        claimed_by: plan.claimed_by(t).map(target_str),
        copy_of: plan.copy_of(t).map(|(c, _)| target_str(c.root)),
        elements,
        cells,
        alternatives: alternatives_of(plan, t),
    }
}

/// Lower is stronger.
fn rank(origin: &Origin, c: &IntervalConstraint, sized: bool) -> u8 {
    match (origin, c) {
        (
            Origin::Intent,
            IntervalConstraint::Fixed {
                firmness: Firmness::Hard,
                ..
            },
        ) => 0,
        (Origin::Deck { .. }, _) => 1,
        (Origin::Scheme { .. } | Origin::Preference { .. } | Origin::Piece { .. }, _) => 2,
        (Origin::Intent, IntervalConstraint::Fixed { .. }) => 3,
        (Origin::Intent, _) => 4,
        (Origin::Sizing, _) if sized => 5,
        _ => 6,
    }
}

fn row_of(
    r: &Record<IntervalConstraint>,
    count: u32,
    edge: EntityId,
) -> (IntervalRequestRow, Option<f64>) {
    let c = f64::from(count);
    let (kind, firmness, value, detail, agrees) = match &r.item {
        IntervalConstraint::Fixed {
            count: n, firmness, ..
        } => (
            "fixed",
            Some(format!("{firmness:?}").to_lowercase()),
            Some(f64::from(*n)),
            format!("count {n}"),
            *n == count,
        ),
        IntervalConstraint::Goal { goal, .. } => (
            "goal",
            None,
            Some(*goal),
            format!("goal {goal}"),
            (c - goal).abs() < 1.0,
        ),
        IntervalConstraint::Range { min, max, .. } => (
            "range",
            None,
            None,
            match max {
                Some(m) => format!("{min}..={m}"),
                None => format!("{min}.."),
            },
            count >= *min && max.is_none_or(|m| count <= m),
        ),
        IntervalConstraint::Linear { rel, rhs, .. } => (
            "linear",
            None,
            None,
            format!("row over {} edges, {rel:?} {rhs}", r.item.edges().len()),
            true,
        ),
        IntervalConstraint::EvenSum { edges } => (
            "even_sum",
            None,
            None,
            format!("even sum over {} edges", edges.len()),
            true,
        ),
        _ => ("other", None, None, String::new(), true),
    };
    let _ = edge;
    (
        IntervalRequestRow {
            id: r.id.0,
            origin: origin_str(&r.origin),
            kind: kind.into(),
            firmness,
            value,
            detail,
            agrees,
            winner: false,
        },
        value,
    )
}

fn interval_why(plan: &Plan, edge: EntityId, count: u32) -> IntervalWhy {
    let sized = plan.sizing.is_some();
    let mentions: Vec<&Record<IntervalConstraint>> = plan
        .problem
        .intervals
        .iter()
        .filter(|r| r.item.edges().binary_search(&edge).is_ok())
        .collect();
    let mut rows: Vec<IntervalRequestRow> =
        mentions.iter().map(|r| row_of(r, count, edge).0).collect();
    // The strongest direct request that agrees with the solved count decides it.
    let winner = mentions
        .iter()
        .enumerate()
        .filter(|(i, r)| {
            rows[*i].agrees
                && matches!(
                    r.item,
                    IntervalConstraint::Fixed { .. } | IntervalConstraint::Goal { .. }
                )
        })
        .min_by_key(|(_, r)| (rank(&r.origin, &r.item, sized), r.id))
        .map(|(i, _)| i);
    let linear: Vec<u32> = mentions
        .iter()
        .filter(|r| {
            matches!(
                r.item,
                IntervalConstraint::Linear { .. } | IntervalConstraint::EvenSum { .. }
            )
        })
        .map(|r| r.id.0)
        .collect();
    let (why, goal, scheme, target, constraints) = match winner {
        Some(i) => {
            rows[i].winner = true;
            let r = mentions[i];
            let goal = match r.item {
                IntervalConstraint::Goal { goal, .. } => Some(goal),
                _ => None,
            };
            let (why, sch, tgt) = match (&r.origin, &r.item) {
                (
                    Origin::Intent,
                    IntervalConstraint::Fixed {
                        firmness: Firmness::Hard,
                        ..
                    },
                ) => (WhyKind::HardIntent, None, None),
                (Origin::Intent, IntervalConstraint::Fixed { .. }) => {
                    (WhyKind::SoftIntent, None, None)
                }
                (Origin::Intent, _) => (WhyKind::SizingGoal, None, None),
                (Origin::Sizing, _) if sized => (WhyKind::SizingGoal, None, None),
                (Origin::Sizing, _) => (WhyKind::Default, None, None),
                (Origin::Deck { .. }, _) => (WhyKind::Deck, None, None),
                (Origin::Scheme { scheme, target }, _)
                | (Origin::Preference { scheme, target }, _) => (
                    WhyKind::Scheme,
                    Some(scheme.as_str().to_owned()),
                    Some(target_str(*target)),
                ),
                (Origin::Piece { .. }, _) => (WhyKind::Piece, None, None),
                _ => (WhyKind::Default, None, None),
            };
            (why, goal, sch, tgt, Vec::new())
        }
        None if !linear.is_empty() => {
            for r in &mut rows {
                if r.kind == "linear" || r.kind == "even_sum" {
                    r.winner = true;
                }
            }
            (WhyKind::Derived, None, None, None, linear)
        }
        None => (WhyKind::Default, None, None, None, Vec::new()),
    };
    IntervalWhy {
        edge,
        count,
        why,
        goal,
        scheme,
        target,
        constraints,
        requests: rows,
    }
}

/// Explain `plan`: what each entity gets and why, and where every interval count came from.
///
/// With `mesh`, the report is `ran` and counts the cells classified on each entity; without it,
/// it is `will_run` with the element type the caps promise and an estimated cell count. Pure:
/// the callers (`mesh.plan`, diagnostics, the CLI) fill what only they know (`mesh`, `stale`,
/// `blockers`, `notes`, the selection of a `ran` report).
pub fn explain_plan(
    plan: &Plan,
    view: &dyn TopologyView,
    mesh: Option<&dyn MeshRead>,
) -> PlanReport {
    let targets = &plan.problem.targets;
    let three = targets
        .iter()
        .any(|t| matches!(t.target, Target::Volume(_) | Target::Group(_)));
    let ids = |f: fn(&Target) -> bool| -> Vec<EntityId> {
        targets
            .iter()
            .filter(|t| f(&t.target))
            .filter_map(|t| t.target.entity())
            .collect()
    };
    let selection = if three {
        ids(|t| matches!(t, Target::Volume(_)))
    } else {
        ids(|t| matches!(t, Target::Surface(_)))
    };
    let estimated = mesh.is_none().then(|| {
        let n = if three {
            Driver::estimate_cells(plan, view, &selection)
        } else {
            estimate_face_cells(plan, view, &selection)
        };
        n.round().min(u64::MAX as f64) as u64
    });
    PlanReport {
        status: if mesh.is_some() {
            PlanStatus::Ran
        } else {
            PlanStatus::WillRun
        },
        mesh: None,
        stale: false,
        dim: if three { PlanDim::Three } else { PlanDim::Two },
        selection,
        estimated_cells: estimated,
        waves: plan.waves.len(),
        plan_hash: serde_json::to_vec(plan).ok().map(|b| fnv_hex(&b)),
        blockers: Vec::new(),
        failure: None,
        entities: targets
            .iter()
            .map(|tp| entity_row(plan, tp, mesh))
            .collect(),
        intervals: plan
            .intervals
            .iter()
            .map(|&(e, n)| interval_why(plan, e, n))
            .collect(),
        notes: Vec::new(),
    }
}

fn infeasible(e: &SchemeError, faces: bool, selection: Vec<EntityId>) -> PlanReport {
    let (reason, constraints, entities) = match e {
        SchemeError::Infeasible(i) => (
            Some(i.reason.clone()),
            i.constraints.iter().map(|c| c.0).collect(),
            i.entities.clone(),
        ),
        _ => (None, Vec::new(), Vec::new()),
    };
    PlanReport {
        status: PlanStatus::Infeasible,
        mesh: None,
        stale: false,
        dim: if faces { PlanDim::Two } else { PlanDim::Three },
        selection,
        estimated_cells: None,
        waves: 0,
        plan_hash: None,
        blockers: Vec::new(),
        failure: Some(PlanFailure {
            message: e.to_string(),
            reason,
            constraints,
            entities,
            blame: e
                .blame()
                .map(|(t, s)| format!("{} (scheme {})", target_str(t), s.as_str())),
        }),
        entities: Vec::new(),
        intervals: Vec::new(),
        notes: Vec::new(),
    }
}

fn strip(mut r: PlanReport, attempts: bool) -> PlanReport {
    if !attempts {
        for e in &mut r.entities {
            e.alternatives.clear();
        }
    }
    r
}

impl TypedQuery for MeshPlanQuery {
    const NAME: &'static str = "mesh.plan";
    type Output = PlanReport;

    fn run(&self, state: &State, cx: &QueryCtx<'_>) -> Result<PlanReport, OpError> {
        let meshes = state
            .layer::<MeshLayer>()
            .ok_or_else(|| OpError::MissingLayer("meshes".into()))?;
        let model = state
            .layer::<ModelLayer>()
            .ok_or_else(|| OpError::MissingLayer("model".into()))?;
        match (&self.mesh, &self.targets) {
            (Some(m), None) => {
                let m = *m;
                let e = meshes.get(m).ok_or(OpError::UnknownEntity(m))?;
                let plan = e
                    .plan
                    .as_deref()
                    .ok_or_else(|| OpError::Invalid("mesh has no plan".into()))?;
                let mut r = explain_plan(plan, model.topo(), Some(e.mesh()));
                r.mesh = Some(m);
                r.stale = e.stale;
                r.selection = e.selection.clone();
                r.notes = e
                    .notes
                    .iter()
                    .map(|n| PlanNote {
                        position: [n.position.x, n.position.y, n.position.z],
                        entities: n.entities.clone(),
                        metric: n.metric.clone(),
                        value: n.value,
                        threshold: n.threshold,
                        suggestion: n.suggestion.clone(),
                    })
                    .collect();
                Ok(strip(r, self.attempts))
            }
            (None, Some(ts)) => {
                if ts.is_empty() {
                    return Err(OpError::Invalid("`targets` is empty".into()));
                }
                for &id in ts {
                    if !state.contains(id) {
                        return Err(OpError::UnknownEntity(id));
                    }
                    if !matches!(
                        id.kind(),
                        sm_core::EntityKind::TopoVolume | sm_core::EntityKind::TopoFace
                    ) {
                        return Err(OpError::Invalid(format!(
                            "{id} is a {:?}; expected a TopoVolume or TopoFace",
                            id.kind()
                        )));
                    }
                }
                let mut targets = ts.clone();
                targets.sort_unstable();
                targets.dedup();
                let faces = faces_mode(&targets)?;
                let cfg = cx.service::<MesherConfig>()?;
                let intent = state
                    .layer::<IntentLayer>()
                    .ok_or_else(|| OpError::MissingLayer("intent".into()))?
                    .intent();
                // `polyhedron` plans as the `tetmesh` it runs.
                let lowered = if faces {
                    None
                } else {
                    crate::polyscheme::lower(intent, &targets)?.map(|(i, _)| i)
                };
                let intent = lowered.as_ref().unwrap_or(intent);
                let field = size_field(cfg, intent)?;
                let driver = make_driver(cfg, field.as_deref(), faces, targets.len());
                let input = MeshInput::new(model.topo(), model.model(), &WholeFaces, &targets);
                let mut blockers = run_guards(meshes, model.topo(), &targets, faces);
                let mut r = match driver.plan(&input, intent) {
                    Ok(plan) => {
                        let cells = if faces {
                            estimate_face_cells(&plan, model.topo(), &targets)
                        } else {
                            Driver::estimate_cells(&plan, model.topo(), &targets)
                        };
                        if cells > cfg.max_cells() as f64 {
                            blockers.push(
                                SchemeError::OverBudget {
                                    cells,
                                    max: cfg.max_cells(),
                                }
                                .to_string(),
                            );
                        }
                        explain_plan(&plan, model.topo(), None)
                    }
                    Err(e) => infeasible(&e, faces, targets.clone()),
                };
                r.selection = targets;
                r.blockers = blockers;
                Ok(strip(r, self.attempts))
            }
            _ => Err(OpError::Invalid(
                "give exactly one of `mesh` and `targets`".into(),
            )),
        }
    }
}
