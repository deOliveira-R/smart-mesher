//! Interval constraints and the global interval-matching contract.
//!
//! Every curve is one integer variable (its interval count). Schemes emit constraints in
//! `plan` (map: opposite logical sides have equal sums; pave: even boundary sum; sweep: equal
//! layer counts along every linking path), the driver adds sizes and deck pairings, and one
//! [`IntervalSolver`] resolves them for the whole model before any curve is meshed (Cubit's
//! interval assignment, "IIA"). The production solver lives in `sm-sizing`; [`SimpleSolver`] is a
//! small deterministic reference used by tests.

use crate::intent::{Firmness, SchemeId};
use crate::key::ShapeKey;
use serde::{Deserialize, Serialize};

/// `coef * intervals(curve)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct Term {
    /// The curve.
    pub curve: ShapeKey,
    /// Integer coefficient.
    pub coef: i32,
}

/// Relation of a linear constraint `sum(terms) REL rhs`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Relation {
    /// Equal.
    Eq,
    /// Less than or equal.
    Le,
    /// Greater than or equal.
    Ge,
}

/// One constraint on interval counts.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum IntervalConstraint {
    /// A count on one curve: `Hard` fixes it, `Soft` is a strong goal, `Default` a weak goal.
    Count {
        /// The curve.
        curve: ShapeKey,
        /// The count.
        n: u32,
        /// Firmness.
        firmness: Firmness,
    },
    /// A real-valued goal (curve length / target size).
    Goal {
        /// The curve.
        curve: ShapeKey,
        /// Desired count (not rounded).
        goal: f64,
    },
    /// `sum(coef * n) REL rhs`, e.g. `side0 - side2 = 0` for map.
    Linear {
        /// The terms.
        terms: Vec<Term>,
        /// The relation.
        rel: Relation,
        /// Right-hand side.
        rhs: i64,
    },
    /// The sum over the curves (with multiplicity) is even (quad meshing of a loop, pave).
    Even {
        /// The curves.
        curves: Vec<ShapeKey>,
    },
    /// Lower bound on one curve.
    Min {
        /// The curve.
        curve: ShapeKey,
        /// Minimum count.
        n: u32,
    },
}

impl IntervalConstraint {
    /// `n(a) = n(b)`.
    pub fn equal(a: ShapeKey, b: ShapeKey) -> Self {
        IntervalConstraint::Linear { terms: vec![Term { curve: a, coef: 1 }, Term { curve: b, coef: -1 }], rel: Relation::Eq, rhs: 0 }
    }

    /// `sum n(a) = sum n(b)` (opposite logical sides of a mapped face).
    pub fn sides_equal(a: &[ShapeKey], b: &[ShapeKey]) -> Self {
        let terms = a.iter().map(|&curve| Term { curve, coef: 1 }).chain(b.iter().map(|&curve| Term { curve, coef: -1 })).collect();
        IntervalConstraint::Linear { terms, rel: Relation::Eq, rhs: 0 }
    }

    /// The curves this constraint mentions.
    pub fn curves(&self) -> Vec<ShapeKey> {
        match self {
            IntervalConstraint::Count { curve, .. } | IntervalConstraint::Goal { curve, .. } | IntervalConstraint::Min { curve, .. } => vec![*curve],
            IntervalConstraint::Linear { terms, .. } => terms.iter().map(|t| t.curve).collect(),
            IntervalConstraint::Even { curves } => curves.clone(),
        }
    }

    /// True if `sol` satisfies the constraint (goals and soft counts always do).
    pub fn is_satisfied(&self, sol: &IntervalSolution) -> bool {
        let n = |c: &ShapeKey| i64::from(sol.get(*c).unwrap_or(0));
        match self {
            IntervalConstraint::Count { curve, n: k, firmness: Firmness::Hard } => n(curve) == i64::from(*k),
            IntervalConstraint::Count { .. } | IntervalConstraint::Goal { .. } => true,
            IntervalConstraint::Min { curve, n: k } => n(curve) >= i64::from(*k),
            IntervalConstraint::Linear { terms, rel, rhs } => {
                let s: i64 = terms.iter().map(|t| i64::from(t.coef) * n(&t.curve)).sum();
                match rel {
                    Relation::Eq => s == *rhs,
                    Relation::Le => s <= *rhs,
                    Relation::Ge => s >= *rhs,
                }
            }
            IntervalConstraint::Even { curves } => curves.iter().map(n).sum::<i64>() % 2 == 0,
        }
    }
}

/// A constraint with where it came from (for conflict reports).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SourcedConstraint {
    /// The constraint.
    pub constraint: IntervalConstraint,
    /// The entity whose plan (or intent / deck) produced it.
    pub by: ShapeKey,
    /// The scheme, if a scheme produced it.
    pub scheme: Option<SchemeId>,
    /// Short reason, e.g. `"map: opposite sides"`, `"deck pair"`, `"size"`.
    pub why: String,
}

/// The whole interval problem.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct IntervalProblem {
    /// Every curve variable, ascending, unique.
    pub curves: Vec<ShapeKey>,
    /// All constraints, in deterministic (driver) order.
    pub constraints: Vec<SourcedConstraint>,
}

/// Interval counts per curve.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct IntervalSolution {
    counts: Vec<(ShapeKey, u32)>,
}

impl IntervalSolution {
    /// From `(curve, count)` pairs (sorted here; a later duplicate wins).
    pub fn from_pairs(mut pairs: Vec<(ShapeKey, u32)>) -> Self {
        pairs.sort_by_key(|p| p.0);
        let mut counts: Vec<(ShapeKey, u32)> = Vec::with_capacity(pairs.len());
        for p in pairs {
            match counts.last_mut() {
                Some(last) if last.0 == p.0 => *last = p,
                _ => counts.push(p),
            }
        }
        IntervalSolution { counts }
    }
    /// The count of `curve`.
    pub fn get(&self, curve: ShapeKey) -> Option<u32> {
        self.counts.binary_search_by(|(k, _)| k.cmp(&curve)).ok().map(|i| self.counts[i].1)
    }
    /// All counts, ascending by curve.
    pub fn iter(&self) -> impl Iterator<Item = (ShapeKey, u32)> + '_ {
        self.counts.iter().copied()
    }
}

/// Interval solve failures.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum IntervalError {
    /// Two hard counts disagree on curves forced equal.
    #[error("hard interval conflict on {curve}: {a} vs {b}")]
    HardConflict {
        /// A curve of the conflicting class.
        curve: ShapeKey,
        /// One hard count.
        a: u32,
        /// The other.
        b: u32,
    },
    /// A constraint names a curve that is not a variable.
    #[error("constraint names unknown curve {0}")]
    UnknownCurve(ShapeKey),
    /// No assignment satisfies the hard constraints.
    #[error("interval problem infeasible: {}", violated.join("; "))]
    Infeasible {
        /// Descriptions of the constraints still violated.
        violated: Vec<String>,
    },
}

/// Global interval solver (Cubit IIA). Must be deterministic: same problem, same solution.
pub trait IntervalSolver: Send + Sync {
    /// Solve for every curve in `p.curves`.
    fn solve(&self, p: &IntervalProblem) -> Result<IntervalSolution, IntervalError>;
}

/// Reference solver: union-find over pairwise equalities, weighted rounding of goals, then a
/// bounded greedy repair that only *raises* counts (sums and parity). Not optimal; enough for
/// mapped/swept test models.
#[derive(Clone, Copy, Debug, Default)]
pub struct SimpleSolver;

struct Classes {
    parent: Vec<usize>,
}

impl Classes {
    fn find(&mut self, mut i: usize) -> usize {
        while self.parent[i] != i {
            self.parent[i] = self.parent[self.parent[i]];
            i = self.parent[i];
        }
        i
    }
    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
        self.parent[hi] = lo;
    }
}

impl IntervalSolver for SimpleSolver {
    fn solve(&self, p: &IntervalProblem) -> Result<IntervalSolution, IntervalError> {
        let n = p.curves.len();
        let ix = |c: &ShapeKey| p.curves.binary_search(c).map_err(|_| IntervalError::UnknownCurve(*c));
        let mut uf = Classes { parent: (0..n).collect() };
        for sc in &p.constraints {
            for c in sc.constraint.curves() {
                ix(&c)?;
            }
            if let IntervalConstraint::Linear { terms, rel: Relation::Eq, rhs: 0 } = &sc.constraint
                && let [a, b] = terms.as_slice()
                && a.coef == -b.coef
                && a.coef != 0
            {
                uf.union(ix(&a.curve)?, ix(&b.curve)?);
            }
        }
        let root: Vec<usize> = (0..n).map(|i| uf.find(i)).collect();
        let mut hard: Vec<Option<(u32, ShapeKey)>> = vec![None; n];
        let mut min = vec![1u32; n];
        let mut goal = vec![(0.0f64, 0.0f64); n];
        for sc in &p.constraints {
            match &sc.constraint {
                IntervalConstraint::Count { curve, n: k, firmness } => {
                    let r = root[ix(curve)?];
                    match firmness {
                        Firmness::Hard => match hard[r] {
                            Some((h, _)) if h != *k => return Err(IntervalError::HardConflict { curve: *curve, a: h, b: *k }),
                            _ => hard[r] = Some((*k, *curve)),
                        },
                        Firmness::Soft => {
                            goal[r].0 += 10.0 * f64::from(*k);
                            goal[r].1 += 10.0;
                        }
                        Firmness::Default => {
                            goal[r].0 += f64::from(*k);
                            goal[r].1 += 1.0;
                        }
                    }
                }
                IntervalConstraint::Goal { curve, goal: g } if g.is_finite() => {
                    let r = root[ix(curve)?];
                    goal[r].0 += g;
                    goal[r].1 += 1.0;
                }
                IntervalConstraint::Min { curve, n: k } => {
                    let r = root[ix(curve)?];
                    min[r] = min[r].max(*k);
                }
                _ => {}
            }
        }
        let mut val = vec![1u32; n];
        for r in 0..n {
            if root[r] != r {
                continue;
            }
            val[r] = match hard[r] {
                Some((h, _)) => h,
                None if goal[r].1 > 0.0 => ((goal[r].0 / goal[r].1).round().max(1.0)).min(f64::from(u32::MAX / 4)) as u32,
                None => 1,
            }
            .max(min[r]);
        }
        let is_hard = |r: usize| hard[r].is_some();
        // Greedy repair, bounded.
        let max_rounds = 16 + 4 * p.constraints.len();
        for _ in 0..max_rounds {
            let mut changed = false;
            for sc in &p.constraints {
                match &sc.constraint {
                    IntervalConstraint::Linear { terms, rel, rhs } => {
                        let mut net: Vec<(usize, i64)> = Vec::new();
                        for t in terms {
                            let r = root[ix(&t.curve)?];
                            match net.iter_mut().find(|e| e.0 == r) {
                                Some(e) => e.1 += i64::from(t.coef),
                                None => net.push((r, i64::from(t.coef))),
                            }
                        }
                        let s: i64 = net.iter().map(|&(r, c)| c * i64::from(val[r])).sum();
                        let res = s - rhs;
                        let need_up = match rel {
                            Relation::Eq => res < 0,
                            Relation::Ge => res < 0,
                            Relation::Le => false,
                        };
                        let need_down = match rel {
                            Relation::Eq | Relation::Le => res > 0,
                            Relation::Ge => false,
                        };
                        if !(need_up || need_down) {
                            continue;
                        }
                        // Raise a free class whose net coefficient moves the sum the right way.
                        let want_sign = if need_up { 1 } else { -1 };
                        let pick = net.iter().filter(|&&(r, c)| c * want_sign > 0 && !is_hard(r)).min_by_key(|&&(r, _)| (val[r], r)).copied();
                        if let Some((r, c)) = pick {
                            let step = (res.abs() + c.abs() - 1) / c.abs();
                            val[r] = val[r].saturating_add(u32::try_from(step).unwrap_or(u32::MAX / 4));
                            changed = true;
                        }
                    }
                    IntervalConstraint::Even { curves } => {
                        let mut cnt: Vec<(usize, u32)> = Vec::new();
                        for c in curves {
                            let r = root[ix(c)?];
                            match cnt.iter_mut().find(|e| e.0 == r) {
                                Some(e) => e.1 += 1,
                                None => cnt.push((r, 1)),
                            }
                        }
                        let s: u64 = cnt.iter().map(|&(r, m)| u64::from(m) * u64::from(val[r])).sum();
                        if s % 2 == 1
                            && let Some(&(r, _)) = cnt.iter().filter(|&&(r, m)| m % 2 == 1 && !is_hard(r)).min_by_key(|&&(r, _)| (val[r], r))
                        {
                            val[r] += 1;
                            changed = true;
                        }
                    }
                    _ => {}
                }
            }
            if !changed {
                break;
            }
        }
        let sol = IntervalSolution::from_pairs(p.curves.iter().enumerate().map(|(i, &c)| (c, val[root[i]])).collect());
        let violated: Vec<String> = p.constraints.iter().filter(|sc| !sc.constraint.is_satisfied(&sol)).map(|sc| format!("{} ({}, {:?})", sc.why, sc.by, sc.constraint)).collect();
        if violated.is_empty() { Ok(sol) } else { Err(IntervalError::Infeasible { violated }) }
    }
}
