//! The global [`MeshingProblem`]: every constraint contributed by intent, schemes, the model
//! (decks) and the driver, each with its origin, before anything is solved or generated.

use crate::firm::{Firm, Firmness};
use crate::intent::{Params, SchemeName};
use crate::target::{GroupId, Target};
use crate::vertex::{VertexRule, VertexType};
use serde::{Deserialize, Serialize};
use sm_core::{EntityId, Transform};
use sm_topo_api::CornerKey;

/// Identifier of a constraint within one problem (dense, in contribution order).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct ConstraintId(pub u32);

/// Who contributed a constraint.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
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

/// An interval constraint over curve interval counts (the IIA system).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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
    /// `sum(lhs) == sum(rhs)`; an edge listed twice counts twice (opposite map sides, submap
    /// sums, sweep layer counts, copy pairs).
    SumEqual {
        /// Left side.
        lhs: Vec<EntityId>,
        /// Right side.
        rhs: Vec<EntityId>,
    },
    /// `sum(edges)` is even (quad meshing of a loop: pave, qtri base).
    EvenSum {
        /// The edges (with multiplicity).
        edges: Vec<EntityId>,
    },
}

/// A scheme set imposed on a lower-dimensional entity by a parent scheme (map volume forces map
/// surfaces; sweep forces map/submap on linking faces).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Imposition {
    /// The constrained entity.
    pub child: Target,
    /// Allowed schemes (sorted). The child's own request must be one of them, else it is
    /// overridden (soft/default) or an error (hard).
    pub allowed: Vec<SchemeName>,
    /// The imposing parent.
    pub by: Target,
}

/// The role an entity plays for a parent scheme.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum Role {
    /// Source face of a sweep (any surface scheme; meshed first).
    SweepSource {
        /// The sweeping piece.
        volume: Target,
    },
    /// Target face of a sweep (meshed by copying the source).
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
    /// Implied by a copy of a parent entity (edges of a copied face).
    Closure {
        /// The copied parent constraint.
        parent: ConstraintId,
    },
}

/// The mesh of `target` is the image of the mesh of `source` under `transform`, with nodes
/// mapped through the vertex correspondence (edges follow from vertices). Implies equal
/// intervals on corresponding edges and generation of `target` after `source`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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

/// Other (non-interval) constraints on the generated mesh.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum MeshConstraint {
    /// No boundary layer may grow from this face (every deck face).
    NoBoundaryLayer {
        /// The face.
        face: EntityId,
    },
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

/// How a target's scheme was chosen.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Selection {
    /// Requested in the intent (or as a fallback listed there: `rank` is the chain position).
    Intent {
        /// Position in `[scheme, fallbacks...]`.
        rank: u32,
    },
    /// From a decomposition piece shape hint.
    PieceShape,
    /// Chosen among the schemes allowed by impositions.
    Imposed,
    /// Automatic selection with its applicability score.
    Auto {
        /// The winning score.
        score: u32,
    },
    /// The target is a copy (deck, sweep target, user copy).
    Copy {
        /// The copy constraint.
        constraint: ConstraintId,
    },
}

/// The resolved scheme of one target.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TargetPlan {
    /// The target.
    pub target: Target,
    /// The scheme name ([`SchemeName::COPY`] for copies).
    pub scheme: SchemeName,
    /// Its parameters.
    pub params: Params,
    /// How it was chosen.
    pub selection: Selection,
    /// Candidates rejected on the way, with reasons (applicability or earlier fallbacks).
    pub rejected: Vec<(SchemeName, String)>,
}

/// Every contribution, by kind. Each list is in contribution order (= id order within the
/// list); ids are unique across lists.
#[derive(Clone, Debug, PartialEq, Default, Serialize, Deserialize)]
pub struct MeshingProblem {
    /// Resolved scheme per target, sorted by target.
    pub targets: Vec<TargetPlan>,
    /// Topology vertices of the meshed region (one node each), ascending.
    pub vertices: Vec<EntityId>,
    /// Child scheme impositions.
    pub impositions: Vec<Record<Imposition>>,
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
}

impl MeshingProblem {
    fn next_id(&mut self) -> ConstraintId {
        let id = ConstraintId(self.next);
        self.next += 1;
        id
    }

    /// The plan row of `target`.
    pub fn target(&self, t: Target) -> Option<&TargetPlan> {
        self.targets.binary_search_by(|p| p.target.cmp(&t)).ok().map(|i| &self.targets[i])
    }

    /// The copy constraint whose target is `t`.
    pub fn copy_into(&self, t: Target) -> Option<&Record<CopyConstraint>> {
        self.copies.iter().find(|c| c.item.target == t)
    }

    /// The roles of `t`.
    pub fn roles_of(&self, t: Target) -> impl Iterator<Item = Role> + '_ {
        self.roles.iter().filter(move |r| r.item.0 == t).map(|r| r.item.1)
    }

    /// Add an interval constraint.
    pub fn push_interval(&mut self, origin: Origin, c: IntervalConstraint) -> ConstraintId {
        let id = self.next_id();
        self.intervals.push(Record { id, origin, item: c });
        id
    }

    /// Add a copy constraint.
    pub fn push_copy(&mut self, origin: Origin, c: CopyConstraint) -> ConstraintId {
        let id = self.next_id();
        self.copies.push(Record { id, origin, item: c });
        id
    }

    /// Add a mesh constraint.
    pub fn push_constraint(&mut self, origin: Origin, c: MeshConstraint) -> ConstraintId {
        let id = self.next_id();
        self.constraints.push(Record { id, origin, item: c });
        id
    }

    /// Add an imposition.
    pub fn push_imposition(&mut self, origin: Origin, c: Imposition) -> ConstraintId {
        let id = self.next_id();
        self.impositions.push(Record { id, origin, item: c });
        id
    }

    /// Add a role.
    pub fn push_role(&mut self, origin: Origin, t: Target, r: Role) -> ConstraintId {
        let id = self.next_id();
        self.roles.push(Record { id, origin, item: (t, r) });
        id
    }

    /// Add a corner rule.
    pub fn push_vertex_rule(&mut self, origin: Origin, face: EntityId, r: VertexRule) -> ConstraintId {
        let id = self.next_id();
        self.vertex_rules.push(Record { id, origin, item: (face, r) });
        id
    }

    /// Add a vertex-type request.
    pub fn push_vertex_type(&mut self, origin: Origin, c: CornerKey, t: Firm<VertexType>) -> ConstraintId {
        let id = self.next_id();
        self.vertex_types.push(Record { id, origin, item: (c, t) });
        id
    }
}

/// The writer a scheme gets in the requirements phase. Every item is recorded with the
/// scheme and target as its origin.
pub struct Requirements<'a> {
    pub(crate) problem: &'a mut MeshingProblem,
    pub(crate) origin: Origin,
    pub(crate) target: Target,
}

impl<'a> Requirements<'a> {
    /// A writer that records into `problem` on behalf of `scheme` planning `target`.
    pub fn new(problem: &'a mut MeshingProblem, scheme: &SchemeName, target: Target) -> Self {
        Self { problem, origin: Origin::Scheme { scheme: scheme.clone(), target }, target }
    }

    /// Restrict the schemes of a lower-dimensional `child` (e.g. map/submap on linking faces).
    pub fn impose(&mut self, child: Target, allowed: &[&str]) -> ConstraintId {
        let mut allowed: Vec<SchemeName> = allowed.iter().map(|s| SchemeName::new(s)).collect();
        allowed.sort();
        allowed.dedup();
        let by = self.target;
        self.problem.push_imposition(self.origin.clone(), Imposition { child, allowed, by })
    }

    /// Give `child` a role (sweep source/target/linking).
    pub fn role(&mut self, child: Target, role: Role) -> ConstraintId {
        self.problem.push_role(self.origin.clone(), child, role)
    }

    /// Require that `target` be meshed as a copy of `source`.
    pub fn copy(&mut self, c: CopyConstraint) -> ConstraintId {
        self.problem.push_copy(self.origin.clone(), c)
    }

    /// Require a corner rule on `face` (map: 4 ends; submap: sum 4; ...).
    pub fn vertex_rule(&mut self, face: EntityId, rule: VertexRule) -> ConstraintId {
        self.problem.push_vertex_rule(self.origin.clone(), face, rule)
    }

    /// Request a vertex type on one corner.
    pub fn vertex_type(&mut self, corner: CornerKey, t: Firm<VertexType>) -> ConstraintId {
        self.problem.push_vertex_type(self.origin.clone(), corner, t)
    }

    /// Add another mesh constraint (precedence, plane lock, atomic group).
    pub fn constraint(&mut self, c: MeshConstraint) -> ConstraintId {
        self.problem.push_constraint(self.origin.clone(), c)
    }
}

/// The writer a scheme gets in the interval phase (after corners and roles are solved).
pub struct IntervalSink<'a> {
    pub(crate) problem: &'a mut MeshingProblem,
    pub(crate) origin: Origin,
}

impl<'a> IntervalSink<'a> {
    /// A writer that records into `problem` on behalf of `scheme` planning `target`.
    pub fn new(problem: &'a mut MeshingProblem, scheme: &SchemeName, target: Target) -> Self {
        Self { problem, origin: Origin::Scheme { scheme: scheme.clone(), target } }
    }

    /// `sum(lhs) == sum(rhs)`.
    pub fn equal(&mut self, lhs: &[EntityId], rhs: &[EntityId]) -> ConstraintId {
        let c = IntervalConstraint::SumEqual { lhs: lhs.to_vec(), rhs: rhs.to_vec() };
        self.problem.push_interval(self.origin.clone(), c)
    }

    /// `sum(edges)` is even.
    pub fn even(&mut self, edges: &[EntityId]) -> ConstraintId {
        self.problem.push_interval(self.origin.clone(), IntervalConstraint::EvenSum { edges: edges.to_vec() })
    }

    /// Fix a count.
    pub fn fix(&mut self, edge: EntityId, count: u32, firmness: Firmness) -> ConstraintId {
        self.problem.push_interval(self.origin.clone(), IntervalConstraint::Fixed { edge, count, firmness })
    }

    /// Bound a count.
    pub fn range(&mut self, edge: EntityId, min: u32, max: Option<u32>) -> ConstraintId {
        self.problem.push_interval(self.origin.clone(), IntervalConstraint::Range { edge, min, max })
    }
}
