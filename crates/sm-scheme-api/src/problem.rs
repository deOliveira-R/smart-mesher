//! The global [`MeshingProblem`]: every constraint contributed by intent, schemes, the model
//! (decks) and the driver, each with its origin, before anything is solved or generated.

use crate::caps::Caps;
use crate::error::{Result, SchemeError};
use crate::firm::{Firm, Firmness};
use crate::intent::{BoundaryLayerIntent, Params, SchemeName};
use crate::target::{GroupId, Target};
use crate::vertex::{VertexRule, VertexType};
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Transform};
use sm_topo_api::CornerKey;
use std::collections::{BTreeMap, BTreeSet};
use std::sync::OnceLock;

/// Identifier of a constraint within one problem (dense, in contribution order).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ConstraintId(pub u32);

/// Who contributed a constraint.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Origin {
    /// The user's intent.
    Intent,
    /// The sizing layer (interval goals from sizes).
    Sizing,
    /// A scheme, while planning `target`.
    Scheme {
        /// The scheme.
        scheme: SchemeName,
        /// The target it was planning.
        target: Target,
    },
    /// A deck boundary of the model (`sm-model-api`).
    Deck {
        /// The boundary record.
        boundary: EntityId,
    },
    /// A decomposition piece shape hint (`PieceShape`).
    Piece {
        /// The piece.
        piece: EntityId,
    },
    /// The driver (copy closure, defaults).
    Driver,
}

/// Relation of a [`IntervalConstraint::Linear`] row to its right-hand side.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
pub enum Rel {
    /// `sum == rhs`.
    Eq,
    /// `sum <= rhs`.
    Le,
    /// `sum >= rhs`.
    Ge,
}

/// An interval constraint over curve interval counts (the IIA system).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schemars", derive(schemars::JsonSchema))]
#[non_exhaustive]
pub enum IntervalConstraint {
    /// The count of `edge` is `count` (hard: never changed; soft/default: a preferred value).
    Fixed {
        /// The edge.
        edge: EntityId,
        /// The count.
        count: u32,
        /// Firmness.
        firmness: Firmness,
    },
    /// A real-valued goal (from sizing: `length / size`); the solver stays close to it.
    Goal {
        /// The edge.
        edge: EntityId,
        /// The goal (> 0).
        goal: f64,
    },
    /// Bounds on the count of `edge`.
    Range {
        /// The edge.
        edge: EntityId,
        /// Minimum (>= 1).
        min: u32,
        /// Maximum, if any.
        max: Option<u32>,
    },
    /// `sum(coefficient * count(edge)) rel rhs`. Opposite map sides, submap direction sums,
    /// sweep layer sums and many-to-one path sums are all rows of this form. An edge may be
    /// listed more than once; coefficients add.
    Linear {
        /// `(edge, coefficient)` terms.
        terms: Vec<(EntityId, i32)>,
        /// Relation to `rhs`.
        rel: Rel,
        /// Right-hand side.
        rhs: i64,
    },
    /// `sum(edges)` is even (quad meshing of a loop: pave, qtri base).
    EvenSum {
        /// The edges (with multiplicity).
        edges: Vec<EntityId>,
    },
}

impl IntervalConstraint {
    /// `sum(lhs) == sum(rhs)` as a [`IntervalConstraint::Linear`] row (an edge on both sides
    /// cancels; terms are sorted).
    pub fn sum_equal(lhs: &[EntityId], rhs: &[EntityId]) -> Self {
        let mut acc: BTreeMap<EntityId, i32> = BTreeMap::new();
        for e in lhs {
            *acc.entry(*e).or_insert(0) += 1;
        }
        for e in rhs {
            *acc.entry(*e).or_insert(0) -= 1;
        }
        IntervalConstraint::Linear {
            terms: acc.into_iter().filter(|x| x.1 != 0).collect(),
            rel: Rel::Eq,
            rhs: 0,
        }
    }

    /// Canonical form: `Linear` terms sorted by edge with repeated edges merged (coefficients
    /// add; zero terms dropped), `EvenSum` edges sorted (multiplicity kept).
    pub fn canonicalize(&mut self) {
        match self {
            IntervalConstraint::Linear { terms, .. } => {
                let mut acc: BTreeMap<EntityId, i64> = BTreeMap::new();
                for (e, c) in terms.iter() {
                    *acc.entry(*e).or_insert(0) += i64::from(*c);
                }
                *terms = acc.into_iter().filter(|x| x.1 != 0).map(|(e, c)| (e, c.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32)).collect();
            }
            IntervalConstraint::EvenSum { edges } => edges.sort_unstable(),
            _ => {}
        }
    }

    /// Check a user row (W0f gaps 2, `MeshIntent::intervals`): every edge is a `TopoEdge` or
    /// `PartitionEdge`; `Fixed` counts and `Range` minimums are at least 1 and a maximum is not
    /// below its minimum; a `Goal` is finite and positive; a `Linear` row has at least one term,
    /// no zero coefficient and a right-hand side within `±2^53`; an `EvenSum` lists an edge.
    pub fn check(&self) -> Result<()> {
        let bad = |m: String| Err(SchemeError::Input(format!("interval row: {m}")));
        if let Some(e) = self.edges().into_iter().find(|e| !matches!(e.kind(), sm_core::EntityKind::TopoEdge | sm_core::EntityKind::PartitionEdge)) {
            return bad(format!("{e} is not an edge"));
        }
        match self {
            IntervalConstraint::Fixed { count: 0, .. } => bad("a fixed count must be >= 1".into()),
            IntervalConstraint::Goal { goal, .. } if !(goal.is_finite() && *goal > 0.0) => bad("a goal must be positive and finite".into()),
            IntervalConstraint::Range { min, max, .. } if *min == 0 || max.is_some_and(|m| m < *min) => bad("a range needs 1 <= min <= max".into()),
            IntervalConstraint::Linear { terms, rhs, .. } => {
                if terms.is_empty() {
                    bad("a linear row needs a term".into())
                } else if terms.iter().any(|t| t.1 == 0) {
                    bad("a linear row has a zero coefficient".into())
                } else if rhs.unsigned_abs() > 1 << 53 {
                    bad("the right-hand side is out of range".into())
                } else {
                    Ok(())
                }
            }
            IntervalConstraint::EvenSum { edges } if edges.is_empty() => bad("an even-sum row needs an edge".into()),
            _ => Ok(()),
        }
    }

    /// The edges the constraint mentions, sorted and unique.
    pub fn edges(&self) -> Vec<EntityId> {
        let mut v: Vec<EntityId> = match self {
            IntervalConstraint::Fixed { edge, .. }
            | IntervalConstraint::Goal { edge, .. }
            | IntervalConstraint::Range { edge, .. } => vec![*edge],
            IntervalConstraint::Linear { terms, .. } => terms.iter().map(|t| t.0).collect(),
            IntervalConstraint::EvenSum { edges } => edges.clone(),
        };
        v.sort_unstable();
        v.dedup();
        v
    }
}

/// What a parent requires of a child entity's scheme.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Need {
    /// The child's scheme is one of these names (sorted). For the rare "exactly this
    /// algorithm" case; prefer [`Need::Caps`].
    Schemes(Vec<SchemeName>),
    /// The child's scheme has all of these capabilities (`copy` inherits through its source).
    Caps(Caps),
    /// The child's scheme has none of these capabilities.
    Forbid(Caps),
}

/// A requirement imposed on a lower-dimensional entity by a parent scheme (map volume forces
/// quad/structured surfaces; sweep forces the same on linking faces) or by the model (decks
/// forbid boundary layers).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Imposition {
    /// The constrained entity.
    pub child: Target,
    /// What is required.
    pub need: Need,
    /// A `Hard` imposition makes a hard intent that contradicts it an
    /// `ImpositionConflict`; a `Soft` or `Default` one just overrides soft intents too.
    pub firmness: Firmness,
    /// The imposing parent.
    pub by: Target,
}

/// A parent declares that it writes a lower-dimensional child itself (the target of a
/// many-to-one sweep, the linking faces when a sweep wants them). A claimed child is not
/// resolved or generated on its own.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Claim {
    /// The claimed child.
    pub child: Target,
    /// The claiming parent.
    pub by: Target,
}

/// The role an entity plays for a parent scheme.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Role {
    /// Source face of a sweep (any surface scheme; meshed first).
    SweepSource {
        /// The sweeping piece.
        volume: Target,
    },
    /// Target face of a sweep (meshed by copying the source, or claimed).
    SweepTarget {
        /// The sweeping piece.
        volume: Target,
    },
    /// Linking face of a sweep (map/submap; one direction is the sweep direction).
    SweepLinking {
        /// The sweeping piece.
        volume: Target,
    },
}

/// Why a copy constraint exists.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum CopyReason {
    /// A translation/rotation deck pair: the B face must carry exactly `g(mesh of A)`.
    Deck {
        /// The deck boundary record.
        boundary: EntityId,
    },
    /// The target face of a sweep.
    SweepTarget {
        /// The sweeping piece.
        volume: Target,
    },
    /// A user copy / mirror scheme.
    User,
    /// Implied by a copy of a parent entity (edges of copied faces).
    Closure {
        /// The copied parent constraint.
        parent: ConstraintId,
    },
}

/// The mesh of `target` is the image of the mesh of `source` under `transform`, with nodes
/// mapped through the vertex correspondence (edges follow from vertices). Copy constraints are
/// **edges of an equivalence relation**: the driver groups them into classes and meshes each
/// class from one root, so the direction written here is only the direction of `transform`.
///
/// `#[non_exhaustive]` (W0g r3): build it with [`CopyConstraint::new`] (+
/// [`CopyConstraint::with_vertex_map`]). It is persisted in `Plan.problem.copies`, so any field
/// added later is `#[serde(default)]`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub struct CopyConstraint {
    /// Source entity.
    pub source: Target,
    /// Target entity.
    pub target: Target,
    /// Placement of the copy (may be improper: mirror).
    pub transform: Transform,
    /// Vertex correspondence `(source vertex, target vertex)`, sorted by source. Empty: derive
    /// it geometrically from `transform` (within the model tolerance).
    pub vertex_map: Vec<(EntityId, EntityId)>,
    /// Why.
    pub reason: CopyReason,
}

impl CopyConstraint {
    /// A copy of `source` onto `target` under `transform`, with an empty vertex map (derived
    /// geometrically from `transform`).
    pub fn new(source: Target, target: Target, transform: Transform, reason: CopyReason) -> Self {
        Self { source, target, transform, vertex_map: Vec::new(), reason }
    }

    /// With an explicit vertex correspondence `(source vertex, target vertex)`; it is sorted by
    /// source here.
    #[must_use]
    pub fn with_vertex_map(mut self, mut vertex_map: Vec<(EntityId, EntityId)>) -> Self {
        vertex_map.sort_unstable();
        self.vertex_map = vertex_map;
        self
    }
}

/// Other (non-interval) constraints on the generated mesh. `#[non_exhaustive]` (W0f) so later
/// constraints (curve boundary layers, ...) are not breaking.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum MeshConstraint {
    /// Nodes of the face must stay on the plane `n . x = offset` (mirror deck faces; a
    /// smoothing constraint).
    PlaneLock {
        /// The face.
        face: EntityId,
        /// Unit normal.
        normal: sm_core::Vector3,
        /// Offset.
        offset: f64,
    },
    /// `before` must be generated before `after` (beyond the dimension order).
    Precedes {
        /// First.
        before: Target,
        /// Second.
        after: Target,
    },
    /// The members are generated and committed together, all-or-nothing.
    Atomic {
        /// The group.
        group: GroupId,
        /// The member targets (sorted).
        members: Vec<Target>,
    },
    /// A boundary layer grows from `face` into `volume`. Only a volume scheme with
    /// [`Caps::BOUNDARY_LAYER`] may honour it; deck faces refuse it.
    BoundaryLayer {
        /// The face.
        face: EntityId,
        /// The volume (or group) the layer grows into.
        volume: Target,
        /// The layer specification.
        spec: BoundaryLayerIntent,
    },
}

/// One contributed item with its id and origin.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Record<T> {
    /// Dense id (contribution order, which the driver makes deterministic).
    pub id: ConstraintId,
    /// Who contributed it.
    pub origin: Origin,
    /// The constraint.
    pub item: T,
}

/// How a target's scheme was chosen. `#[non_exhaustive]` (W0f) so later selection kinds (a
/// seeded selection, ...) are not breaking.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum Selection {
    /// Requested in the intent (`rank` is the position in `[scheme, fallbacks...]`).
    Intent {
        /// Position in the chain.
        rank: u32,
    },
    /// A registry fallback of the last intent choice.
    Registry {
        /// Position in the fallback list.
        rank: u32,
    },
    /// From a decomposition piece shape hint.
    PieceShape,
    /// Best-scoring scheme among those that satisfy the impositions.
    Imposed {
        /// The winning score.
        score: u32,
    },
    /// Automatic selection with its applicability score.
    Auto {
        /// The winning score.
        score: u32,
    },
    /// The target is a member of a copy class (deck, sweep target, user copy).
    Copy {
        /// Index into [`crate::Plan::copies`].
        class: u32,
    },
    /// The target is written by a parent scheme.
    Claimed {
        /// The claiming parent.
        by: Target,
    },
}

/// The resolved scheme of one target.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TargetPlan {
    /// The target.
    pub target: Target,
    /// The scheme name ([`SchemeName::COPY`] for copies; the claimer's scheme for claimed
    /// targets).
    pub scheme: SchemeName,
    /// Its parameters.
    pub params: Params,
    /// Scheme-private data stored in the requirements phase ([`Requirements::set_data`]) and
    /// read back in the interval and generation phases.
    pub data: Params,
    /// What the target's mesh provides (`copy` resolved through its class root).
    pub caps: Caps,
    /// How it was chosen.
    pub selection: Selection,
    /// Why the winner applies (its [`crate::Applicability`] reason).
    pub reason: String,
}

/// A lazily built lookup table that never takes part in equality or serialization.
#[derive(Debug, Default)]
pub(crate) struct Lazy<T>(pub(crate) OnceLock<T>);

impl<T> Clone for Lazy<T> {
    fn clone(&self) -> Self {
        Lazy(OnceLock::new())
    }
}

impl<T> PartialEq for Lazy<T> {
    fn eq(&self, _: &Self) -> bool {
        true
    }
}

#[derive(Debug, Default)]
pub(crate) struct ProblemIndex {
    copy_into: BTreeMap<Target, usize>,
    roles: BTreeMap<Target, Vec<usize>>,
    claims: BTreeMap<Target, usize>,
    targets: Vec<Target>,
}

/// Every contribution, by kind. Each list is in contribution order (= id order within the
/// list); ids are unique across lists.
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
pub struct MeshingProblem {
    /// Resolved scheme per target, sorted by target.
    pub targets: Vec<TargetPlan>,
    /// Topology vertices of the meshed region (one node each), ascending.
    pub vertices: Vec<EntityId>,
    /// Child impositions.
    pub impositions: Vec<Record<Imposition>>,
    /// Claims.
    pub claims: Vec<Record<Claim>>,
    /// Roles.
    pub roles: Vec<Record<(Target, Role)>>,
    /// Copy constraints.
    pub copies: Vec<Record<CopyConstraint>>,
    /// Corner rules per face.
    pub vertex_rules: Vec<Record<(EntityId, VertexRule)>>,
    /// Vertex-type requests per corner.
    pub vertex_types: Vec<Record<(CornerKey, Firm<VertexType>)>>,
    /// Interval constraints.
    pub intervals: Vec<Record<IntervalConstraint>>,
    /// Other constraints.
    pub constraints: Vec<Record<MeshConstraint>>,
    #[serde(default)]
    next: u32,
    #[serde(skip)]
    pub(crate) index: Lazy<ProblemIndex>,
}

impl MeshingProblem {
    fn next_id(&mut self) -> ConstraintId {
        let id = ConstraintId(self.next);
        self.next += 1;
        self.index = Lazy::default();
        id
    }

    fn idx(&self) -> &ProblemIndex {
        self.index.0.get_or_init(|| {
            let mut ix = ProblemIndex::default();
            for (i, c) in self.copies.iter().enumerate() {
                ix.copy_into.entry(c.item.target).or_insert(i);
            }
            for (i, r) in self.roles.iter().enumerate() {
                ix.roles.entry(r.item.0).or_default().push(i);
            }
            for (i, c) in self.claims.iter().enumerate() {
                ix.claims.entry(c.item.child).or_insert(i);
            }
            ix.targets = self.targets.iter().map(|t| t.target).collect();
            ix
        })
    }

    /// The plan row of `target` (the `targets` table is sorted).
    pub fn target(&self, t: Target) -> Option<&TargetPlan> {
        self.targets
            .binary_search_by(|p| p.target.cmp(&t))
            .ok()
            .map(|i| &self.targets[i])
    }

    /// The first copy constraint whose target is `t`.
    pub fn copy_into(&self, t: Target) -> Option<&Record<CopyConstraint>> {
        self.idx().copy_into.get(&t).map(|&i| &self.copies[i])
    }

    /// The roles of `t`, in contribution order.
    pub fn roles_of(&self, t: Target) -> impl Iterator<Item = Role> + '_ {
        self.idx()
            .roles
            .get(&t)
            .into_iter()
            .flatten()
            .map(move |&i| self.roles[i].item.1)
    }

    /// The first claim on `child`.
    pub fn claim_of(&self, child: Target) -> Option<&Record<Claim>> {
        self.idx().claims.get(&child).map(|&i| &self.claims[i])
    }

    /// Who contributed constraint `id`, whatever its kind.
    pub fn record_origin(&self, id: ConstraintId) -> Option<&Origin> {
        fn find<T>(v: &[Record<T>], id: ConstraintId) -> Option<&Origin> {
            v.binary_search_by(|r| r.id.cmp(&id))
                .ok()
                .map(|i| &v[i].origin)
        }
        find(&self.impositions, id)
            .or_else(|| find(&self.claims, id))
            .or_else(|| find(&self.roles, id))
            .or_else(|| find(&self.copies, id))
            .or_else(|| find(&self.vertex_rules, id))
            .or_else(|| find(&self.vertex_types, id))
            .or_else(|| find(&self.intervals, id))
            .or_else(|| find(&self.constraints, id))
    }

    /// The target ids of the plan table, sorted.
    pub fn planned_targets(&self) -> &[Target] {
        &self.idx().targets
    }

    pub(crate) fn push_interval(&mut self, origin: Origin, c: IntervalConstraint) -> ConstraintId {
        let id = self.next_id();
        self.intervals.push(Record {
            id,
            origin,
            item: c,
        });
        id
    }

    pub(crate) fn push_copy(&mut self, origin: Origin, c: CopyConstraint) -> ConstraintId {
        let id = self.next_id();
        self.copies.push(Record {
            id,
            origin,
            item: c,
        });
        id
    }

    pub(crate) fn push_constraint(&mut self, origin: Origin, c: MeshConstraint) -> ConstraintId {
        let id = self.next_id();
        self.constraints.push(Record {
            id,
            origin,
            item: c,
        });
        id
    }

    pub(crate) fn push_imposition(&mut self, origin: Origin, c: Imposition) -> ConstraintId {
        let id = self.next_id();
        self.impositions.push(Record {
            id,
            origin,
            item: c,
        });
        id
    }

    pub(crate) fn push_claim(&mut self, origin: Origin, c: Claim) -> ConstraintId {
        let id = self.next_id();
        self.claims.push(Record {
            id,
            origin,
            item: c,
        });
        id
    }

    pub(crate) fn push_role(&mut self, origin: Origin, t: Target, r: Role) -> ConstraintId {
        let id = self.next_id();
        self.roles.push(Record {
            id,
            origin,
            item: (t, r),
        });
        id
    }

    pub(crate) fn push_vertex_rule(
        &mut self,
        origin: Origin,
        face: EntityId,
        r: VertexRule,
    ) -> ConstraintId {
        let id = self.next_id();
        self.vertex_rules.push(Record {
            id,
            origin,
            item: (face, r),
        });
        id
    }

    pub(crate) fn push_vertex_type(
        &mut self,
        origin: Origin,
        c: CornerKey,
        t: Firm<VertexType>,
    ) -> ConstraintId {
        let id = self.next_id();
        self.vertex_types.push(Record {
            id,
            origin,
            item: (c, t),
        });
        id
    }

    pub(crate) fn invalidate(&mut self) {
        self.index = Lazy::default();
    }
}

/// What a requirement may be about: the strict sub-entities of the requesting target and the
/// targets that are already resolved. Built by the driver.
pub(crate) struct Scope<'a> {
    pub(crate) children: BTreeSet<Target>,
    pub(crate) resolved: &'a BTreeMap<Target, TargetPlan>,
    /// Is the target part of the meshed region?
    pub(crate) in_region: &'a dyn Fn(Target) -> bool,
    /// `within(parent, child)`: is `child` a strict sub-entity of `parent`?
    pub(crate) within: &'a dyn Fn(Target, Target) -> bool,
}

/// The id returned for a requirement that was refused (the error surfaces when the scheme's
/// `requirements` returns).
pub const REFUSED: ConstraintId = ConstraintId(u32::MAX);

/// The writer a scheme gets in the requirements phase. Every item is recorded with the
/// scheme and target as its origin, and every child must be a strict sub-entity of the
/// planned target (or of a group member) that is not resolved yet; otherwise the call is
/// refused with [`REFUSED`] and the scheme's `requirements` fails with
/// [`SchemeError::BadRequirement`].
pub struct Requirements<'a> {
    pub(crate) problem: &'a mut MeshingProblem,
    pub(crate) origin: Origin,
    pub(crate) target: Target,
    pub(crate) scope: &'a Scope<'a>,
    pub(crate) error: Option<SchemeError>,
    pub(crate) data: Option<Params>,
}

impl<'a> Requirements<'a> {
    pub(crate) fn new(
        problem: &'a mut MeshingProblem,
        scheme: &SchemeName,
        target: Target,
        scope: &'a Scope<'a>,
    ) -> Self {
        Self {
            problem,
            origin: Origin::Scheme {
                scheme: scheme.clone(),
                target,
            },
            target,
            scope,
            error: None,
            data: None,
        }
    }

    /// Finish: the first refused requirement as an error, else the stored private data.
    pub(crate) fn finish(self) -> Result<Option<Params>> {
        match self.error {
            Some(e) => Err(e),
            None => Ok(self.data),
        }
    }

    fn fail(&mut self, child: Target, reason: &str) {
        if self.error.is_none() {
            self.error = Some(SchemeError::BadRequirement {
                by: self.target,
                child,
                reason: reason.to_owned(),
            });
        }
    }

    fn child_ok(&mut self, child: Target, unresolved: bool) -> bool {
        if self.error.is_some() {
            return false;
        }
        if !self.scope.children.contains(&child) {
            self.fail(
                child,
                "not a strict sub-entity of the requesting target inside the meshed region",
            );
            return false;
        }
        if unresolved && self.scope.resolved.contains_key(&child) {
            self.fail(child, "already resolved");
            return false;
        }
        true
    }

    fn face_ok(&mut self, face: EntityId) -> bool {
        let t = Target::Surface(face);
        if t == self.target {
            return self.error.is_none();
        }
        self.child_ok(t, true)
    }

    /// Require that the scheme of `child` is one of `allowed` (hard). Prefer
    /// [`Requirements::require_caps`].
    pub fn impose(&mut self, child: Target, allowed: &[&str]) -> ConstraintId {
        if !self.child_ok(child, true) {
            return REFUSED;
        }
        let mut names: Vec<SchemeName> = allowed.iter().map(|s| SchemeName::new(s)).collect();
        names.sort();
        names.dedup();
        let by = self.target;
        self.problem.push_imposition(
            self.origin.clone(),
            Imposition {
                child,
                need: Need::Schemes(names),
                firmness: Firmness::Hard,
                by,
            },
        )
    }

    /// Require that the scheme of `child` provides all of `caps` (`copy` inherits through its
    /// class root).
    pub fn require_caps(&mut self, child: Target, caps: Caps, firmness: Firmness) -> ConstraintId {
        if !self.child_ok(child, true) {
            return REFUSED;
        }
        let by = self.target;
        self.problem.push_imposition(
            self.origin.clone(),
            Imposition {
                child,
                need: Need::Caps(caps),
                firmness,
                by,
            },
        )
    }

    /// Forbid every capability in `caps` on `child` (hard).
    pub fn forbid(&mut self, child: Target, caps: Caps) -> ConstraintId {
        if !self.child_ok(child, true) {
            return REFUSED;
        }
        let by = self.target;
        self.problem.push_imposition(
            self.origin.clone(),
            Imposition {
                child,
                need: Need::Forbid(caps),
                firmness: Firmness::Hard,
                by,
            },
        )
    }

    /// Declare that this target writes `child` itself (many-to-one sweep target, linking
    /// faces). The child gets no scheme of its own.
    pub fn claim(&mut self, child: Target) -> ConstraintId {
        if !self.child_ok(child, true) {
            return REFUSED;
        }
        let by = self.target;
        self.problem
            .push_claim(self.origin.clone(), Claim { child, by })
    }

    /// Give `child` a role (sweep source/target/linking).
    pub fn role(&mut self, child: Target, role: Role) -> ConstraintId {
        if !self.child_ok(child, false) {
            return REFUSED;
        }
        self.problem.push_role(self.origin.clone(), child, role)
    }

    /// Relate `target` to `source` as copies (an edge of a copy class). Both must be strict
    /// sub-entities of the requesting target, and `target` must not be resolved yet.
    pub fn copy(&mut self, c: CopyConstraint) -> ConstraintId {
        if !(self.child_ok(c.source, false) && self.child_ok(c.target, true)) {
            return REFUSED;
        }
        self.problem.push_copy(self.origin.clone(), c)
    }

    /// Require a corner rule on `face` (map: 4 ends; submap: sum 4; ...). `face` is the
    /// planned face itself or a face of the planned volume.
    pub fn vertex_rule(&mut self, face: EntityId, rule: VertexRule) -> ConstraintId {
        if !self.face_ok(face) {
            return REFUSED;
        }
        self.problem
            .push_vertex_rule(self.origin.clone(), face, rule)
    }

    /// Request a vertex type on one corner.
    pub fn vertex_type(&mut self, corner: CornerKey, t: Firm<VertexType>) -> ConstraintId {
        if !self.face_ok(corner.face) {
            return REFUSED;
        }
        self.problem
            .push_vertex_type(self.origin.clone(), corner, t)
    }

    /// Add another mesh constraint (precedence, plane lock, boundary layer). Faces and targets
    /// named by it must be sub-entities of the requesting target; atomic groups are the
    /// driver's.
    pub fn constraint(&mut self, c: MeshConstraint) -> ConstraintId {
        let ok = match &c {
            MeshConstraint::PlaneLock { face, .. } | MeshConstraint::BoundaryLayer { face, .. } => {
                self.face_ok(*face)
            }
            MeshConstraint::Precedes { before, after } => {
                let (b, a) = (*before, *after);
                self.child_ok(b, false) && self.child_ok(a, false)
            }
            MeshConstraint::Atomic { .. } => {
                let t = self.target;
                self.fail(t, "atomic groups come from the intent");
                false
            }
        };
        if !ok {
            return REFUSED;
        }
        self.problem.push_constraint(self.origin.clone(), c)
    }

    /// Declare that this target's generation reads the patch of `other`, a target that is not
    /// one of its sub-entities (a curve scheme reading a sibling curve's patch: curve `copy` by
    /// percent or auto, PARITY C). Records a scheme-origin
    /// `MeshConstraint::Precedes { before: other, after: self }`, so `other` is generated in an
    /// earlier wave and `GenCtx::patches.get(other)` is guaranteed to be present when this
    /// target generates.
    ///
    /// Refused (with [`REFUSED`], the error surfacing when `requirements` returns) if `other`
    /// is not a target of the meshed region, is the requester itself, one of its ancestors
    /// (a target the requester is a sub-entity of), or one of the children it claimed (it
    /// writes them). A dependency cycle through such a read is found when the plan is
    /// scheduled and reported as [`crate::Infeasible`] blaming the reading `(target, scheme)`,
    /// so the driver rejects that scheme and re-plans with a fallback.
    pub fn reads(&mut self, other: Target) -> ConstraintId {
        if self.error.is_some() {
            return REFUSED;
        }
        let me = self.target;
        let why = if !(self.scope.in_region)(other) {
            Some("not a target of the meshed region")
        } else if other == me {
            Some("a target cannot read itself")
        } else if (self.scope.within)(other, me) {
            Some("an ancestor is generated after its sub-entities")
        } else if self.problem.claims.iter().any(|c| c.item.by == me && c.item.child == other) {
            Some("a claimed child is written by the requester itself")
        } else {
            None
        };
        if let Some(reason) = why {
            self.fail(other, reason);
            return REFUSED;
        }
        self.problem.push_constraint(self.origin.clone(), MeshConstraint::Precedes { before: other, after: me })
    }

    /// Store scheme-private data for this target. It is kept in
    /// [`TargetPlan::data`] and read back by the interval and generation phases, so a scheme
    /// does not recompute its analysis in every phase.
    pub fn set_data(&mut self, data: Params) {
        self.data = Some(data);
    }
}

/// The writer a scheme gets in the interval phase (after corners and roles are solved).
pub struct IntervalSink<'a> {
    pub(crate) problem: &'a mut MeshingProblem,
    pub(crate) origin: Origin,
}

impl<'a> IntervalSink<'a> {
    pub(crate) fn new(
        problem: &'a mut MeshingProblem,
        scheme: &SchemeName,
        target: Target,
    ) -> Self {
        Self {
            problem,
            origin: Origin::Scheme {
                scheme: scheme.clone(),
                target,
            },
        }
    }

    /// `sum(lhs) == sum(rhs)`.
    pub fn equal(&mut self, lhs: &[EntityId], rhs: &[EntityId]) -> ConstraintId {
        self.problem
            .push_interval(self.origin.clone(), IntervalConstraint::sum_equal(lhs, rhs))
    }

    /// `sum(coefficient * count) rel rhs`.
    pub fn linear(&mut self, terms: &[(EntityId, i32)], rel: Rel, rhs: i64) -> ConstraintId {
        self.problem.push_interval(
            self.origin.clone(),
            IntervalConstraint::Linear {
                terms: terms.to_vec(),
                rel,
                rhs,
            },
        )
    }

    /// `sum(edges)` is even.
    pub fn even(&mut self, edges: &[EntityId]) -> ConstraintId {
        self.problem.push_interval(
            self.origin.clone(),
            IntervalConstraint::EvenSum {
                edges: edges.to_vec(),
            },
        )
    }

    /// Fix a count.
    pub fn fix(&mut self, edge: EntityId, count: u32, firmness: Firmness) -> ConstraintId {
        self.problem.push_interval(
            self.origin.clone(),
            IntervalConstraint::Fixed {
                edge,
                count,
                firmness,
            },
        )
    }

    /// State a real-valued goal for the count of `edge` (for example `length / size`), without
    /// rounding it to an integer first. Emits [`IntervalConstraint::Goal`].
    ///
    /// A goal is a preference at the lowest firmness: the solver averages every goal on an
    /// equal-count class (the per-edge sizing goal, `Default` counts and goals from schemes)
    /// and rounds the mean once, so a goal never overrides anything. A soft count on the class
    /// replaces the mean, and a hard count replaces both; ranges and linear rows still bind
    /// the rounded value. Prefer this to `fix(.., Firmness::Soft)` when a count is derived from
    /// sizes, because a soft count can override the user's own soft count. A goal that is not
    /// finite and positive is ignored by the solver.
    pub fn goal(&mut self, edge: EntityId, goal: f64) -> ConstraintId {
        self.problem
            .push_interval(self.origin.clone(), IntervalConstraint::Goal { edge, goal })
    }

    /// Bound a count.
    pub fn range(&mut self, edge: EntityId, min: u32, max: Option<u32>) -> ConstraintId {
        self.problem.push_interval(
            self.origin.clone(),
            IntervalConstraint::Range { edge, min, max },
        )
    }
}
