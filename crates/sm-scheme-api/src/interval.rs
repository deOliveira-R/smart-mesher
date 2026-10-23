//! Interval constraints and the solver seam (Cubit's interval assignment, "IIA").
//!
//! Every scheme states, **before meshing**, what its edges' interval counts must satisfy (map:
//! opposite logical sides equal; pave: even boundary sum; sweep: equal layer counts along every
//! source -> target path; deck pairs: mapped edges equal). The driver adds the user's counts and
//! sizes as goals and hands everything to one global [`IntervalSolver`] (implemented in
//! `sm-sizing`). The variables are edges (`TopoEdge` or overlay `PartitionEdge`).

use crate::intent::{Firmness, SchemeName};
use crate::target::Target;
use smallvec::SmallVec;
use sm_core::EntityId;
use std::collections::{BTreeMap, BTreeSet};

/// One term `coeff * intervals(edge)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Term {
    /// The edge (interval variable).
    pub edge: EntityId,
    /// Integer coefficient.
    pub coeff: i32,
}

/// Comparison of a linear constraint.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Relation {
    /// `sum == rhs`.
    Eq,
    /// `sum <= rhs`.
    Le,
    /// `sum >= rhs`.
    Ge,
}

/// The mathematical content of a constraint.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ConstraintKind {
    /// `sum(terms) rel rhs`. Side sums of map/submap, sweep layer equalities, deck pairs.
    Linear {
        /// Terms (an edge may appear in several).
        terms: SmallVec<[Term; 4]>,
        /// Relation.
        rel: Relation,
        /// Right-hand side.
        rhs: i64,
    },
    /// `sum(intervals(edges))` is even (paving, all-quad meshes of a loop).
    Even {
        /// The edges of the loop(s).
        edges: Vec<EntityId>,
    },
    /// A desired count from a size or a user count; the solver stays as close as it can.
    Goal {
        /// The edge.
        edge: EntityId,
        /// Desired (real-valued) count, at least 1.
        count: f64,
    },
}

/// Who asked for a constraint (for error reports and for dropping a failed scheme's constraints).
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Origin {
    /// User intervals or sizes on a target.
    User(Target),
    /// The default size.
    DefaultSize,
    /// A scheme planning a job over `target`.
    Scheme {
        /// Scheme name.
        scheme: SchemeName,
        /// First written target of the job.
        target: Target,
    },
    /// A deck pairing: edge `a` on side A maps to edge `b` on side B.
    Deck {
        /// Side-A edge.
        a: EntityId,
        /// Side-B edge.
        b: EntityId,
    },
}

/// A constraint with its firmness and origin.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IntervalConstraint {
    /// Content.
    pub kind: ConstraintKind,
    /// `Hard` linear/parity constraints must hold; goals of higher firmness win.
    pub firmness: Firmness,
    /// Who asked.
    pub origin: Origin,
}

impl IntervalConstraint {
    /// `sum(a) == sum(b)` (each side one coefficient per edge), the map/sweep side rule.
    pub fn sums_equal(a: &[EntityId], b: &[EntityId], firmness: Firmness, origin: Origin) -> Self {
        let mut terms: SmallVec<[Term; 4]> = a.iter().map(|&edge| Term { edge, coeff: 1 }).collect();
        terms.extend(b.iter().map(|&edge| Term { edge, coeff: -1 }));
        Self { kind: ConstraintKind::Linear { terms, rel: Relation::Eq, rhs: 0 }, firmness, origin }
    }

    /// A goal count.
    pub fn goal(edge: EntityId, count: f64, firmness: Firmness, origin: Origin) -> Self {
        Self { kind: ConstraintKind::Goal { edge, count }, firmness, origin }
    }

    /// The edges this constraint mentions, ascending and unique.
    pub fn edges(&self) -> Vec<EntityId> {
        let mut v: Vec<EntityId> = match &self.kind {
            ConstraintKind::Linear { terms, .. } => terms.iter().map(|t| t.edge).collect(),
            ConstraintKind::Even { edges } => edges.clone(),
            ConstraintKind::Goal { edge, .. } => vec![*edge],
        };
        v.sort_unstable();
        v.dedup();
        v
    }

    /// True if `counts` satisfies this constraint (goals always do). Missing edges count as 0.
    pub fn holds(&self, counts: &IntervalSolution) -> bool {
        let n = |e: &EntityId| i64::from(counts.get(*e).unwrap_or(0));
        match &self.kind {
            ConstraintKind::Linear { terms, rel, rhs } => {
                let s: i64 = terms.iter().map(|t| i64::from(t.coeff) * n(&t.edge)).sum();
                match rel {
                    Relation::Eq => s == *rhs,
                    Relation::Le => s <= *rhs,
                    Relation::Ge => s >= *rhs,
                }
            }
            ConstraintKind::Even { edges } => edges.iter().map(n).sum::<i64>() % 2 == 0,
            ConstraintKind::Goal { .. } => true,
        }
    }
}

/// Solved interval counts per edge.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct IntervalSolution {
    /// Count per edge (every count >= 1).
    pub counts: BTreeMap<EntityId, u32>,
}

impl IntervalSolution {
    /// The count of an edge.
    pub fn get(&self, edge: EntityId) -> Option<u32> {
        self.counts.get(&edge).copied()
    }
}

/// Why the interval solve failed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum IntervalError {
    /// Two hard goals in one equality class disagree.
    #[error("hard interval counts disagree on edges {0:?}")]
    HardConflict(Vec<EntityId>),
    /// Constraints left unsatisfied (indices into the input slice).
    #[error("{} interval constraint(s) unsatisfied", .0.len())]
    Unsatisfied(Vec<usize>),
}

/// The global interval solver (IIA). Implemented in `sm-sizing`.
///
/// Contract: deterministic (same inputs, same counts, any thread count); every variable gets a
/// count >= 1; every `Hard` linear/parity constraint holds or the call fails; goals are met as
/// closely as the solver's objective allows, higher firmness first.
pub trait IntervalSolver: Send + Sync {
    /// Solve for `vars` (ascending, unique) under `constraints`.
    fn solve(&self, vars: &[EntityId], constraints: &[IntervalConstraint]) -> Result<IntervalSolution, IntervalError>;
}

/// Reference solver: equality classes plus rounding, then a parity repair and a full check.
///
/// It handles what structured schemes on simple models produce (pairwise side equalities, goals,
/// parity on loops with a free class) and reports anything else as `Unsatisfied`. Steps:
/// 1. union-find over two-term `a - b == 0` constraints;
/// 2. per class: the hard goal (all hard goals must agree), else the rounded mean of the
///    highest-firmness goals, else 1;
/// 3. each failing `Even` constraint bumps the lowest class that occurs an odd number of times in
///    it and has no hard goal;
/// 4. every constraint is checked.
#[derive(Clone, Copy, Debug, Default)]
pub struct ReferenceIntervalSolver;

fn find(p: &mut [usize], mut i: usize) -> usize {
    while p[i] != i {
        p[i] = p[p[i]];
        i = p[i];
    }
    i
}

impl IntervalSolver for ReferenceIntervalSolver {
    fn solve(&self, vars: &[EntityId], constraints: &[IntervalConstraint]) -> Result<IntervalSolution, IntervalError> {
        let ix = |e: EntityId| vars.binary_search(&e).ok();
        let mut parent: Vec<usize> = (0..vars.len()).collect();
        for c in constraints {
            if let ConstraintKind::Linear { terms, rel: Relation::Eq, rhs: 0 } = &c.kind
                && let [a, b] = terms.as_slice()
                && a.coeff == -b.coeff
                && a.coeff.abs() == 1
                && let (Some(i), Some(j)) = (ix(a.edge), ix(b.edge))
            {
                let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                if ri != rj {
                    // Lower index becomes the root: deterministic.
                    let (lo, hi) = (ri.min(rj), ri.max(rj));
                    parent[hi] = lo;
                }
            }
        }
        let roots: Vec<usize> = (0..vars.len()).map(|i| find(&mut parent, i)).collect();
        // Goals per class: (firmness, count).
        let mut goals: BTreeMap<usize, Vec<(Firmness, f64, EntityId)>> = BTreeMap::new();
        for c in constraints {
            if let ConstraintKind::Goal { edge, count } = c.kind
                && let Some(i) = ix(edge)
            {
                goals.entry(roots[i]).or_default().push((c.firmness, count.max(1.0), edge));
            }
        }
        let mut value: Vec<u32> = vec![1; vars.len()];
        let mut hard: BTreeSet<usize> = BTreeSet::new();
        for (&r, g) in &goals {
            let top = g.iter().map(|x| x.0).max().unwrap_or_default();
            let sel: Vec<&(Firmness, f64, EntityId)> = g.iter().filter(|x| x.0 == top).collect();
            let v = if top == Firmness::Hard {
                let first = sel[0].1.round();
                if sel.iter().any(|x| x.1.round() != first) {
                    return Err(IntervalError::HardConflict(sel.iter().map(|x| x.2).collect()));
                }
                hard.insert(r);
                first
            } else {
                (sel.iter().map(|x| x.1).sum::<f64>() / sel.len() as f64).round()
            };
            value[r] = v.clamp(1.0, u32::MAX as f64) as u32;
        }
        let assign = |value: &[u32]| IntervalSolution { counts: vars.iter().enumerate().map(|(i, &e)| (e, value[roots[i]])).collect() };
        let mut sol = assign(&value);
        for c in constraints {
            if let ConstraintKind::Even { edges } = &c.kind
                && !c.holds(&sol)
            {
                let mut mult: BTreeMap<usize, u32> = BTreeMap::new();
                for e in edges {
                    if let Some(i) = ix(*e) {
                        *mult.entry(roots[i]).or_default() += 1;
                    }
                }
                if let Some((&r, _)) = mult.iter().find(|(r, m)| *m % 2 == 1 && !hard.contains(r)) {
                    value[r] += 1;
                    sol = assign(&value);
                }
            }
        }
        let bad: Vec<usize> = constraints.iter().enumerate().filter(|(_, c)| !c.holds(&sol)).map(|(i, _)| i).collect();
        if bad.is_empty() { Ok(sol) } else { Err(IntervalError::Unsatisfied(bad)) }
    }
}
