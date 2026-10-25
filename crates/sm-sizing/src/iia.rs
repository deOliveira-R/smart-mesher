//! [`IiaSolver`]: exact integer-program interval assignment.
//!
//! # Problem
//!
//! One positive integer per edge, subject to the [`IntervalConstraint`] rows, minimizing the
//! lexicographic objective (plan T3)
//!
//! 1. the maximum relative deviation `|n - g| / g` over all goal terms,
//! 2. then the sum of the relative deviations,
//! 3. then the lexicographically smallest count vector in ascending edge order.
//!
//! The objective has a unique minimizer, so the answer never depends on search order, thread
//! count or the order of the constraint list. Rows are also processed in `ConstraintId` order,
//! so even when the node budget runs out (and a heuristic answer is returned) the result does
//! not depend on how the caller ordered the list.
//!
//! # Goal terms
//!
//! Single-edge equalities (`x - y = 0` rows) merge edges into *classes* that share one count.
//! Each class contributes terms to the objective:
//!
//! * a class with a `Hard` [`IntervalConstraint::Fixed`] is fixed and contributes no term;
//! * else, if any edge of the class has a `Soft` fixed count, one term per soft count (a soft
//!   request beats a default goal, as in `Firmness`; there is no extra weight);
//! * else one term per [`IntervalConstraint::Goal`] and default-firmness `Fixed` count;
//! * else the single implicit term `g = 1`.
//!
//! Non-positive or non-finite goals are ignored. A constraint kind this crate does not know is
//! rejected with a typed [`Infeasible`] rather than ignored.
//!
//! # Exactness
//!
//! Deviations are compared as integers: `key = round(|n - g| / g * 2^40)` (see
//! [`deviation_key`]), so the search can sum, bound and prune in exact integer arithmetic. No
//! floating-point LP is used. Each key carries up to half a unit of rounding error, so two sums
//! of keys that differ by no more than the number of goal terms may be a real tie in disguise
//! (goals 10, 20, 20 have a sum that is flat on `[10, 20]`, yet the keys differ by noise). Such
//! sums are never ordered by their keys: the pruning treats them as ties, and every candidate
//! comparison (leaf, polish, reference pick, rowless classes) re-checks them in double-double
//! arithmetic ([`compare_deviation_sums`], relative accuracy about 1e-30; sums closer than 1e-24
//! are a tie), so a real tie goes to the lexicographically smallest vector as the plan states.
//! Two sums that differ genuinely but by less than `terms * 2^-40` are also read as a tie by the
//! pruning (never by the final comparison). The search is an integer branch and bound with interval bound
//! propagation on the rows and parity propagation on even-sum rows. Domains that the rows do not
//! bound are windowed by a maximum deviation `D`: the optimum of the windowed problem is the
//! global optimum as soon as one feasible point exists inside the window. Small components
//! (at most 12 classes) try `D` = 0.5, 1, 2, ... 2^40. Larger ones first bisect for the smallest
//! `D` whose windows survive bound propagation (a dense row then gives tight domains at once),
//! then grow `D` in small geometric steps; at each window a feasibility-only search (no
//! objective, smallest domain first, three value orders) must find a point before the
//! optimizing search runs, and that point seeds its incumbent. For dense `<=` or `>=` rows the
//! sum bound also adds the cheapest unit moves that repair the row from the goal-nearest point.
//!
//! # Components, parallelism, limits
//!
//! Classes linked by any shared row form a component. The sum and the vector separate over
//! components, but the maximum does not, so the solve has two phases: (1) every component finds
//! its own lexicographic optimum, which fixes the global maximum deviation `D*` as the largest
//! component maximum; (2) every component whose own maximum is below `D*` is re-solved for its
//! minimal sum and smallest vector under the cap `D*`. Components are solved in parallel with
//! rayon and collected in component order (lowest edge first), so 1 and N threads agree bit for
//! bit. A component with more than [`IiaSolver::max_component_classes`] classes,
//! or one that exhausts [`IiaSolver::max_nodes`] search nodes, is handed to
//! [`ReferenceIntervalSolver`] (restricted to that component), and the reference answer is
//! verified exactly against every constraint of the component. A search that runs out of nodes
//! first looks for any feasible point; the best of that, the search incumbent and the reference
//! answer is then improved by a deterministic local polish (single `+-1` moves and `+-1` pair
//! moves between variables that share a row, strict lexicographic improvements only, run to a
//! fixed point) and returned: 1- and 2-move locally optimal, not proven optimal (see DESIGN.md
//! "Known gaps"). The polish also runs under the phase-2 cap `D*`.
//!
//! Work bound: a search visits at most [`IiaSolver::max_nodes`] nodes; the row-aware bound
//! (at most four dense rows, at most 2048 heap moves each, pooled buffers) is charged to the same
//! budget, one node per 64 moves; the polish is capped at 20 million move evaluations. A 30-class
//! exhausted component takes about a second in release mode at the default budget.
//!
//! # Infeasibility reports
//!
//! `Infeasible` is returned only when it is proven (a completed search, a single row that the
//! bounds already violate, or two rows that contradict each other: at the root every pair of
//! rows sharing a class is combined with small integer multipliers and compared with the sum of
//! the domain bounds in exact integers, so `sum_all <= 50` against `sum_odd >= 40` over counts
//! of at least 1 is refused at once, naming both rows). A conflict that comes from interacting rows names every row of the
//! component. When the search gives up (budget or size) and the reference solver finds nothing
//! either, the reason says "search budget exhausted, feasibility unproven": that is not a proof
//! that no assignment exists.
//!
//! Every returned assignment is re-verified in integer arithmetic against all constraints
//! ([`first_violation`]).

use rayon::prelude::*;
use sm_core::EntityId;
use sm_scheme_api::{
    ConstraintId, Firmness, Infeasible, IntervalConstraint, IntervalSolver, Record,
    ReferenceIntervalSolver, Rel,
};

/// Scale of the integer deviation keys: `2^40`.
const KEY_SCALE: f64 = 1_099_511_627_776.0;
/// Goals are clamped below at this value so deviations stay finite.
const MIN_GOAL: f64 = 1e-9;
/// Largest count (counts are `u32`).
const COUNT_MAX: i64 = u32::MAX as i64;
/// Number of deviation windows tried before the unrestricted last level.
const LEVELS: u32 = 42;
/// Components up to this many classes are searched directly at each window (no feasibility
/// seeding, no bisection): the search finishes long before its budget.
const SMALL_VARS: usize = 12;

/// The quantized relative deviation `round(|count - goal| / goal * 2^40)` of a count from a
/// goal. This is the exact integer the solver compares and sums; `goal` is clamped to at least
/// `1e-9`.
pub fn deviation_key(count: u32, goal: f64) -> u128 {
    key_f(f64::from(count), goal.max(MIN_GOAL))
}

fn key_f(x: f64, g: f64) -> u128 {
    ((x - g).abs() / g * KEY_SCALE).round() as u128
}

// Double-double arithmetic (about 2^-104 relative error) for the exact tie checks.
type Dd = (f64, f64);

fn two_sum(a: f64, b: f64) -> Dd {
    let s = a + b;
    let bb = s - a;
    (s, (a - (s - bb)) + (b - bb))
}

fn dd_add(a: Dd, b: Dd) -> Dd {
    let (s, e) = two_sum(a.0, b.0);
    two_sum(s, e + a.1 + b.1)
}

/// `|x - g| / g` as a double-double (`x - g` is exact, the division keeps the remainder).
fn dev_dd(x: f64, g: f64) -> Dd {
    let (dh, dl) = two_sum(x, -g);
    let q1 = dh / g;
    let r = (-q1).mul_add(g, dh) + dl;
    let (h, l) = two_sum(q1, r / g);
    if h < 0.0 { (-h, -l) } else { (h, l) }
}

/// Order of two sums of deviations given as `(count, goal)` terms, to double-double accuracy:
/// sums closer than `1e-24` (relative) are `Equal`.
fn cmp_sums(a: &[(f64, f64)], b: &[(f64, f64)]) -> std::cmp::Ordering {
    let sum = |t: &[(f64, f64)]| {
        t.iter()
            .fold((0.0, 0.0), |acc, &(x, g)| dd_add(acc, dev_dd(x, g)))
    };
    let (sa, sb) = (sum(a), sum(b));
    let d = dd_add(sa, (-sb.0, -sb.1));
    if d.0.abs() <= 1e-24 * (sa.0.abs() + sb.0.abs() + 1.0) {
        std::cmp::Ordering::Equal
    } else if d.0 < 0.0 {
        std::cmp::Ordering::Less
    } else {
        std::cmp::Ordering::Greater
    }
}

/// Order of two sums of relative deviations `|n - g| / g`, each given as `(n, g)` terms, computed
/// in double-double arithmetic: sums within `1e-24` (relative) are `Equal`. This is the exact tie
/// check the solver applies when quantized sums are closer than the number of terms; it is public
/// so oracles can use the same notion of a real tie.
pub fn compare_deviation_sums(a: &[(f64, f64)], b: &[(f64, f64)]) -> std::cmp::Ordering {
    let clamp = |t: &[(f64, f64)]| -> Vec<(f64, f64)> {
        t.iter().map(|&(n, g)| (n, g.max(MIN_GOAL))).collect()
    };
    cmp_sums(&clamp(a), &clamp(b))
}

/// Fills a pooled buffer with the `(count, goal)` terms of one candidate.
type Pairs<'a> = &'a dyn Fn(&mut Vec<(f64, f64)>);
/// Two reusable term buffers for [`cmp_obj`] (no allocation per tie-zone comparison).
type Scratch = (Vec<(f64, f64)>, Vec<(f64, f64)>);

/// Order of two candidates `(max key, sum key, vector)` under the objective: the maximum (when
/// `use_max`), then the sum, then the vector. Quantized sums closer than `eps` (the number of
/// terms: each key carries at most half a unit of rounding error) are not trusted: the exact
/// sums decide, and only a real tie goes to the vector.
fn cmp_obj(
    use_max: bool,
    a: (u128, u128, &[i64]),
    b: (u128, u128, &[i64]),
    eps: u128,
    pa: Pairs,
    pb: Pairs,
    sc: &mut Scratch,
) -> std::cmp::Ordering {
    use std::cmp::Ordering::{Greater, Less};
    if use_max && a.0 != b.0 {
        return a.0.cmp(&b.0);
    }
    let s = if a.1 > b.1 + eps {
        Greater
    } else if a.1 + eps < b.1 {
        Less
    } else {
        sc.0.clear();
        sc.1.clear();
        pa(&mut sc.0);
        pb(&mut sc.1);
        cmp_sums(&sc.0, &sc.1)
    };
    s.then_with(|| a.2.cmp(b.2))
}

/// The first constraint (in list order) that `counts` violates, or `None` when every
/// constraint holds. Exact integer arithmetic; `Goal` rows never fail. `counts` must be aligned
/// with `edges` (ascending); an unknown edge, a constraint kind this crate does not know, or a
/// count of 0 counts as a violation. A zero count is reported as the first constraint that
/// mentions that edge (counts are at least 1 by definition, so no row states the rule itself);
/// if none mentions it, the first constraint, or `ConstraintId(0)` for an empty list.
pub fn first_violation(
    edges: &[EntityId],
    constraints: &[Record<IntervalConstraint>],
    counts: &[u32],
) -> Option<ConstraintId> {
    let at = |e: &EntityId| edges.binary_search(e).ok().map(|i| i64::from(counts[i]));
    for r in constraints {
        let ok = match &r.item {
            IntervalConstraint::Fixed {
                edge,
                count,
                firmness,
            } => match at(edge) {
                Some(v) => *firmness != Firmness::Hard || v == i64::from(*count),
                None => false,
            },
            IntervalConstraint::Goal { edge, .. } => at(edge).is_some(),
            IntervalConstraint::Range { edge, min, max } => match at(edge) {
                Some(v) => v >= i64::from(*min) && max.is_none_or(|m| v <= i64::from(m)),
                None => false,
            },
            IntervalConstraint::Linear { terms, rel, rhs } => {
                let mut sum = 0i128;
                let mut known = true;
                for (e, k) in terms {
                    match at(e) {
                        Some(v) => sum += i128::from(*k) * i128::from(v),
                        None => known = false,
                    }
                }
                known
                    && match rel {
                        Rel::Eq => sum == i128::from(*rhs),
                        Rel::Le => sum <= i128::from(*rhs),
                        Rel::Ge => sum >= i128::from(*rhs),
                    }
            }
            IntervalConstraint::EvenSum { edges: es } => {
                let mut sum = 0i128;
                let mut known = true;
                for e in es {
                    match at(e) {
                        Some(v) => sum += i128::from(v),
                        None => known = false,
                    }
                }
                known && sum % 2 == 0
            }
            // Unknown (future) kinds cannot be checked, so they never pass.
            _ => false,
        };
        if !ok {
            return Some(r.id);
        }
    }
    if let Some(z) = counts.iter().position(|&c| c == 0) {
        // Name a constraint that mentions the zero-count edge; with none, the first constraint
        // (or `ConstraintId(0)` for an empty list) stands in for the implicit "count >= 1" rule.
        let e = edges.get(z);
        let mentions = |item: &IntervalConstraint| match item {
            IntervalConstraint::Fixed { edge, .. }
            | IntervalConstraint::Goal { edge, .. }
            | IntervalConstraint::Range { edge, .. } => Some(edge) == e,
            IntervalConstraint::Linear { terms, .. } => terms.iter().any(|t| Some(&t.0) == e),
            IntervalConstraint::EvenSum { edges: es } => es.iter().any(|x| Some(x) == e),
            _ => false,
        };
        return constraints
            .iter()
            .find(|r| mentions(&r.item))
            .or(constraints.first())
            .map(|r| r.id)
            .or(Some(ConstraintId(0)));
    }
    None
}

/// Optimal interval solver: an exact integer program over the interval constraints, with the
/// lexicographic objective, component decomposition and fallback described in the
/// [module docs](self).
#[derive(Clone, Copy, Debug)]
pub struct IiaSolver {
    /// Components with more classes than this are solved by the reference solver.
    pub max_component_classes: usize,
    /// Search-node budget per component; exhausting it falls back to the reference solver.
    pub max_nodes: u64,
    inner: ReferenceIntervalSolver,
}

impl Default for IiaSolver {
    fn default() -> Self {
        Self {
            max_component_classes: 512,
            max_nodes: 400_000,
            inner: ReferenceIntervalSolver::default(),
        }
    }
}

impl IiaSolver {
    /// A solver with explicit limits (see the field docs).
    pub fn with_limits(max_component_classes: usize, max_nodes: u64) -> Self {
        Self {
            max_component_classes,
            max_nodes,
            ..Self::default()
        }
    }
}

// ---------------------------------------------------------------------------------------------
// problem building

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    Eq,
    Le,
    Ge,
    Even,
}

struct Row {
    cid: ConstraintId,
    kind: Kind,
    rhs: i64,
    off: usize,
    len: usize,
}

fn find(p: &mut [u32], mut i: u32) -> u32 {
    while p[i as usize] != i {
        let g = p[p[i as usize] as usize];
        p[i as usize] = g;
        i = g;
    }
    i
}

fn union(p: &mut [u32], a: u32, b: u32) {
    let (ra, rb) = (find(p, a), find(p, b));
    if ra != rb {
        p[ra.max(rb) as usize] = ra.min(rb);
    }
}

/// If `terms` is `x - y` after merging duplicate edges, the pair `(x, y)`.
fn pair_difference(terms: &[(EntityId, i32)]) -> Option<(EntityId, EntityId)> {
    let mut acc: Vec<(EntityId, i32)> = Vec::with_capacity(terms.len());
    for (e, k) in terms {
        match acc.iter_mut().find(|x| x.0 == *e) {
            Some(x) => x.1 += k,
            None => acc.push((*e, *k)),
        }
    }
    acc.retain(|x| x.1 != 0);
    match acc.as_slice() {
        [(a, 1), (b, -1)] | [(b, -1), (a, 1)] => Some((*a, *b)),
        _ => None,
    }
}

/// The search problem of one component, in local variable numbering (ascending global class).
struct Local {
    nvar: usize,
    lo0: Vec<i64>,
    hi0: Vec<i64>,
    t_off: Vec<usize>,
    goals: Vec<f64>,
    center: Vec<f64>,
    rows: Vec<Row>,
    r_var: Vec<u32>,
    r_coef: Vec<i64>,
    v_off: Vec<usize>,
    v_rows: Vec<u32>,
    /// Coefficient of the variable in the row at the same position of `v_rows`.
    v_coef: Vec<i64>,
}

impl Local {
    fn terms(&self, v: usize) -> &[f64] {
        &self.goals[self.t_off[v]..self.t_off[v + 1]]
    }
    fn row_terms(&self, r: &Row) -> (&[u32], &[i64]) {
        (
            &self.r_var[r.off..r.off + r.len],
            &self.r_coef[r.off..r.off + r.len],
        )
    }
    /// The `(count, goal)` terms of an assignment.
    fn pairs(&self, vals: &[i64], out: &mut Vec<(f64, f64)>) {
        for (v, &n) in vals.iter().enumerate() {
            out.extend(self.terms(v).iter().map(|&g| (n as f64, g)));
        }
    }
    /// Tolerance of the quantized sums: one unit per goal term.
    fn eps(&self) -> u128 {
        self.goals.len() as u128
    }
    fn exact(&self, v: usize, n: i64) -> (u128, u128) {
        let (mut mx, mut sm) = (0u128, 0u128);
        for &g in self.terms(v) {
            let k = key_f(n as f64, g);
            mx = mx.max(k);
            sm += k;
        }
        (mx, sm)
    }
    /// Lower bound of `(max, sum)` over the integers of `[lo, hi]`.
    fn lower(&self, v: usize, lo: i64, hi: i64) -> (u128, u128) {
        let (mut mx, mut sm) = (0u128, 0u128);
        for &g in self.terms(v) {
            let (l, h) = (lo as f64, hi as f64);
            let dist = if g < l {
                l - g
            } else if g > h {
                g - h
            } else {
                0.0
            };
            let k = (dist / g * KEY_SCALE).round() as u128;
            mx = mx.max(k);
            sm += k;
        }
        (mx, sm)
    }
}

// ---------------------------------------------------------------------------------------------
// branch and bound

struct Best {
    max: u128,
    sum: u128,
    vals: Vec<i64>,
}

/// One unit move of a variable in the row-aware bound: `cost` per `w` units of row reduction.
#[derive(Clone, Copy, PartialEq, Eq)]
struct Move {
    cost: i128,
    w: i128,
    j: usize,
    s: i64,
}

impl Ord for Move {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        // cost / w ascending, ties by variable position (deterministic).
        (self.cost * o.w)
            .cmp(&(o.cost * self.w))
            .then(self.j.cmp(&o.j))
            .then(self.s.cmp(&o.s))
    }
}

impl PartialOrd for Move {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}

struct Search<'a> {
    p: &'a Local,
    free: Vec<Vec<(i64, i64)>>,
    queue: Vec<u32>,
    inq: Vec<bool>,
    changed: Vec<usize>,
    /// False under a deviation cap: the maximum is then constrained, not minimized.
    use_max: bool,
    /// Dense rows used by the row-aware sum bound.
    dense: Vec<u32>,
    /// Value order of the feasibility search (see `dfs_feas`).
    order: u8,
    nodes: u64,
    max_nodes: u64,
    exhausted: bool,
    best: Option<Best>,
    /// Pooled buffers of `row_extra` (no allocation per node).
    pool_base: Vec<(i64, u128)>,
    pool_heap: std::collections::BinaryHeap<std::cmp::Reverse<Move>>,
    /// Pooled term buffers of the leaf comparison.
    pool_cmp: Scratch,
}

fn div_floor(a: i128, b: i128) -> i128 {
    let q = a / b;
    if (a % b != 0) && ((a < 0) != (b < 0)) {
        q - 1
    } else {
        q
    }
}

fn div_ceil(a: i128, b: i128) -> i128 {
    let q = a / b;
    if (a % b != 0) && ((a < 0) == (b < 0)) {
        q + 1
    } else {
        q
    }
}

impl<'a> Search<'a> {
    fn new(p: &'a Local, max_nodes: u64) -> Self {
        Self {
            p,
            free: Vec::new(),
            queue: Vec::new(),
            inq: vec![false; p.rows.len()],
            changed: Vec::new(),
            use_max: true,
            order: 0,
            dense: {
                let mut d: Vec<u32> = (0..p.rows.len() as u32)
                    .filter(|&r| p.nvar > SMALL_VARS && p.rows[r as usize].len >= 4)
                    .collect();
                d.sort_by_key(|&r| std::cmp::Reverse(p.rows[r as usize].len));
                d.truncate(4);
                d
            },
            nodes: 0,
            max_nodes,
            exhausted: false,
            best: None,
            pool_base: Vec::new(),
            pool_cmp: Scratch::default(),
            pool_heap: std::collections::BinaryHeap::new(),
        }
    }

    fn ex(&self, v: usize, n: i64) -> (u128, u128) {
        let (m, s) = self.p.exact(v, n);
        (if self.use_max { m } else { 0 }, s)
    }

    fn lb(&self, v: usize, lo: i64, hi: i64) -> (u128, u128) {
        let (m, s) = self.p.lower(v, lo, hi);
        (if self.use_max { m } else { 0 }, s)
    }

    fn push_row(&mut self, r: u32) {
        if !self.inq[r as usize] {
            self.inq[r as usize] = true;
            self.queue.push(r);
        }
    }

    fn push_var_rows(&mut self, v: usize) {
        let p = self.p;
        for &r in &p.v_rows[p.v_off[v]..p.v_off[v + 1]] {
            self.push_row(r);
        }
    }

    /// Propagate the queued rows to a fixpoint. `Err(row)` names a row that cannot hold.
    fn propagate(&mut self, dom: &mut [(i64, i64)]) -> Result<(), u32> {
        let p = self.p;
        let mut changed = std::mem::take(&mut self.changed);
        let mut result = Ok(());
        // Cyclic rows over huge domains converge one unit per visit; stopping early is sound
        // because every leaf is verified against the rows (`rows_hold`).
        let mut budget = 1024 + 64 * p.rows.len();
        while let Some(ri) = self.queue.pop() {
            self.inq[ri as usize] = false;
            if budget == 0 {
                for &r in &self.queue {
                    self.inq[r as usize] = false;
                }
                self.queue.clear();
                break;
            }
            budget -= 1;
            let row = &p.rows[ri as usize];
            let (vars, coefs) = p.row_terms(row);
            changed.clear();
            let ok = match row.kind {
                Kind::Even => even_row(vars, dom, &mut changed),
                Kind::Le => le_row(vars, coefs, 1, i128::from(row.rhs), dom, &mut changed),
                Kind::Ge => le_row(vars, coefs, -1, -i128::from(row.rhs), dom, &mut changed),
                Kind::Eq => {
                    le_row(vars, coefs, 1, i128::from(row.rhs), dom, &mut changed)
                        && le_row(vars, coefs, -1, -i128::from(row.rhs), dom, &mut changed)
                }
            };
            if !ok {
                for &r in &self.queue {
                    self.inq[r as usize] = false;
                }
                self.queue.clear();
                result = Err(ri);
                break;
            }
            for &v in &changed {
                for &r in &p.v_rows[p.v_off[v]..p.v_off[v + 1]] {
                    if !self.inq[r as usize] {
                        self.inq[r as usize] = true;
                        self.queue.push(r);
                    }
                }
            }
        }
        self.changed = changed;
        result
    }

    /// The exact-sum minimizer of variable `v` over `[l, h]` and its sum key.
    fn argmin_sum(&self, v: usize, l: i64, h: i64) -> (i64, u128) {
        let mut best = (l, self.p.exact(v, l).1);
        let mut try_n = |n: i64| {
            if (l..=h).contains(&n) {
                let k = self.p.exact(v, n).1;
                if (k, n) < (best.1, best.0) {
                    best = (n, k);
                }
            }
        };
        try_n(h);
        for &g in self.p.terms(v) {
            let f = g.floor() as i64;
            for n in (f - 1)..=(f + 2) {
                try_n(n);
            }
        }
        best
    }

    /// Row-aware extra for the sum lower bound: for a dense row `sum a x <= R` whose
    /// goal-nearest assignment violates it, the cheapest set of unit moves that restores it
    /// (a relaxation that ignores the order of moves within one variable, so it is a valid
    /// bound), minus what the per-variable bound already counted for the row's variables.
    ///
    /// Cost: at most `MAX_POPS` heap moves per dense row (at most four rows), so
    /// `O(4 * MAX_POPS * log MAX_POPS)` per call; each call is also charged to the node budget
    /// (one node per 64 moves), so the total work of a component stays proportional to
    /// `max_nodes`.
    fn row_extra(&mut self, dom: &[(i64, i64)]) -> u128 {
        const MAX_POPS: usize = 2048;
        let p = self.p;
        let mut out = 0u128;
        let mut work = 0u64;
        let dense = std::mem::take(&mut self.dense);
        for &ri in &dense {
            let row = &p.rows[ri as usize];
            let (vars, coefs) = p.row_terms(row);
            let signs: &[i64] = match row.kind {
                Kind::Le => &[1],
                Kind::Ge => &[-1],
                Kind::Eq => &[1, -1],
                Kind::Even => continue,
            };
            // Base point per variable and what the plain bound already counts.
            let mut base = std::mem::take(&mut self.pool_base);
            base.clear();
            for &v in vars {
                base.push(self.argmin_sum(v as usize, dom[v as usize].0, dom[v as usize].1));
            }
            let counted: u128 = vars
                .iter()
                .map(|&v| self.lb(v as usize, dom[v as usize].0, dom[v as usize].1).1)
                .sum();
            let f_base: u128 = base.iter().map(|b| b.1).sum();
            for &sg in signs {
                let rhs = i128::from(sg) * i128::from(row.rhs);
                let mut need: i128 = coefs
                    .iter()
                    .zip(&base)
                    .map(|(&k, b)| i128::from(sg * k) * i128::from(b.0))
                    .sum::<i128>()
                    - rhs;
                if need <= 0 {
                    continue;
                }
                // Next unit moves, cheapest per unit of row reduction first.
                let step_cost = |j: usize, s: i64| -> i128 {
                    let v = vars[j] as usize;
                    let (from, to) = if sg * coefs[j] > 0 {
                        (base[j].0 - s + 1, base[j].0 - s)
                    } else {
                        (base[j].0 + s - 1, base[j].0 + s)
                    };
                    p.exact(v, to).1 as i128 - p.exact(v, from).1 as i128
                };
                // Every unit move of every variable goes into the heap at once (the relaxation
                // ignores their order within a variable, which also holds when rounding makes
                // the step costs of one variable non-monotone).
                let mut heap = std::mem::take(&mut self.pool_heap);
                heap.clear();
                let mut total_moves = 0i128;
                for (j, &v) in vars.iter().enumerate() {
                    let (l, h) = dom[v as usize];
                    let w = i128::from((sg * coefs[j]).abs());
                    let avail = if sg * coefs[j] > 0 {
                        base[j].0 - l
                    } else {
                        h - base[j].0
                    };
                    let take = i128::from(avail).min((need + w - 1) / w);
                    total_moves += take;
                    if total_moves > MAX_POPS as i128 {
                        break;
                    }
                    for s in 1..=(take as i64) {
                        heap.push(std::cmp::Reverse(Move {
                            cost: step_cost(j, s),
                            w,
                            j,
                            s,
                        }));
                    }
                }
                if total_moves > MAX_POPS as i128 {
                    self.pool_heap = heap;
                    continue;
                }
                work += heap.len() as u64 / 64 + 1;
                let mut greedy = 0i128;
                let mut done = false;
                while let Some(std::cmp::Reverse(m)) = heap.pop() {
                    if need <= m.w {
                        greedy += div_floor(m.cost * need, m.w);
                        done = true;
                        break;
                    }
                    greedy += m.cost;
                    need -= m.w;
                }
                if done {
                    let total = f_base as i128 + greedy - counted as i128;
                    if total > 0 {
                        out = out.max(total as u128);
                    }
                }
                self.pool_heap = heap;
            }
            self.pool_base = base;
        }
        self.dense = dense;
        self.nodes += work;
        out
    }

    fn lex_cmp_prefix(&self, prefix: &[(i64, i64)], best: &Best) -> std::cmp::Ordering {
        for (i, (l, _)) in prefix.iter().enumerate() {
            match l.cmp(&best.vals[i]) {
                std::cmp::Ordering::Equal => {}
                o => return o,
            }
        }
        std::cmp::Ordering::Equal
    }

    fn dfs(&mut self, dom: &[(i64, i64)]) {
        if self.exhausted {
            return;
        }
        self.nodes += 1;
        if self.nodes > self.max_nodes {
            self.exhausted = true;
            return;
        }
        let p = self.p;
        let Some(v) = (0..p.nvar).find(|&i| dom[i].0 < dom[i].1) else {
            self.leaf(dom);
            return;
        };
        // Aggregate lower bound without v.
        let (mut mx_wo, mut sm_wo) = (0u128, 0u128);
        for (i, &(l, h)) in dom.iter().enumerate() {
            if i == v {
                continue;
            }
            let (m, s) = if l == h {
                self.ex(i, l)
            } else {
                self.lb(i, l, h)
            };
            mx_wo = mx_wo.max(m);
            sm_wo += s;
        }
        let (mv, sv) = self.lb(v, dom[v].0, dom[v].1);
        let eps = p.eps();
        if let Some((bmax, bsum)) = self.best.as_ref().map(|b| (b.max, b.sum)) {
            let (tm, mut ts) = (mx_wo.max(mv), sm_wo + sv);
            if tm == bmax && ts <= bsum + eps && !self.dense.is_empty() {
                ts += self.row_extra(dom);
            }
            // Sums within `eps` of the incumbent may be real ties: only a clearly worse bound
            // prunes on the sum, and the vector order prunes within the tie zone.
            let Some(b) = &self.best else { return };
            if tm > bmax
                || (tm == bmax
                    && (ts > bsum + eps
                        || (ts + eps >= bsum
                            && self.lex_cmp_prefix(&dom[..v], b) == std::cmp::Ordering::Greater)))
            {
                return;
            }
        }
        // Candidate values, outward from the minimizer of the max deviation.
        let (lo, hi) = dom[v];
        // Split at floor(center): the max deviation is monotone on each side of it, even when
        // the curve is asymmetric (several goal terms with different slopes).
        let c = (p.center[v].floor() as i64).clamp(lo - 1, hi);
        let (mut left, mut right) = (c, c + 1);
        let (mut left_live, mut right_live) = (true, right <= hi);
        loop {
            let lk = if left_live && left >= lo {
                Some(self.ex(v, left))
            } else {
                None
            };
            let rk = if right_live && right <= hi {
                Some(self.ex(v, right))
            } else {
                None
            };
            let from_left = match (lk, rk) {
                (None, None) => break,
                (Some(_), None) => true,
                (None, Some(_)) => false,
                (Some(a), Some(b)) => (a, left) <= (b, right),
            };
            // Candidates count as work too: a huge domain must not hide an endless scan.
            self.nodes += 1;
            if self.nodes > self.max_nodes {
                self.exhausted = true;
                return;
            }
            let (n, (mk, sk)) = if from_left {
                (left, lk.unwrap_or_default())
            } else {
                (right, rk.unwrap_or_default())
            };
            if from_left {
                left -= 1;
            } else {
                right += 1;
            }
            if let Some(b) = &self.best {
                if mk > b.max {
                    // Max deviation only grows away from the minimizer: this side is done.
                    if from_left {
                        left_live = false;
                    } else {
                        right_live = false;
                    }
                    continue;
                }
                let (tm, ts) = (mx_wo.max(mk), sm_wo + sk);
                if tm > b.max || (tm == b.max && ts > b.sum + eps) {
                    continue;
                }
                if tm == b.max && ts + eps >= b.sum {
                    match self.lex_cmp_prefix(&dom[..v], b) {
                        std::cmp::Ordering::Greater => continue,
                        std::cmp::Ordering::Equal if n > b.vals[v] => continue,
                        _ => {}
                    }
                }
            }
            let mut child = self.free.pop().unwrap_or_default();
            child.clear();
            child.extend_from_slice(dom);
            child[v] = (n, n);
            self.push_var_rows(v);
            if self.propagate(&mut child).is_ok() {
                self.dfs(&child);
            }
            self.free.push(child);
            if self.exhausted {
                return;
            }
        }
    }

    /// Feasibility-only search: propagate and assign, smallest domain first, values outward from
    /// the goal centre. Sets `best` to the first assignment that satisfies the rows.
    fn dfs_feas(&mut self, dom: &[(i64, i64)]) {
        if self.exhausted || self.best.is_some() {
            return;
        }
        self.nodes += 1;
        if self.nodes > self.max_nodes {
            self.exhausted = true;
            return;
        }
        let p = self.p;
        let pick = (0..p.nvar)
            .filter(|&i| dom[i].0 < dom[i].1)
            .min_by_key(|&i| (i128::from(dom[i].1) - i128::from(dom[i].0), i));
        let Some(v) = pick else {
            if self.rows_hold(dom) {
                let vals: Vec<i64> = dom.iter().map(|d| d.0).collect();
                self.best = Some(Best {
                    max: 0,
                    sum: 0,
                    vals,
                });
            }
            return;
        };
        let (lo, hi) = dom[v];
        // Value order: 0 outward from the goal centre, 1 ascending, 2 descending.
        let c = match self.order {
            0 => (p.center[v].round() as i64).clamp(lo, hi),
            1 => lo - 1,
            _ => hi,
        };
        let (mut left, mut right) = (c, c + 1);
        loop {
            let take_left = left >= lo && (right > hi || c - left <= right - c);
            let n = if take_left {
                let n = left;
                left -= 1;
                n
            } else if right <= hi {
                let n = right;
                right += 1;
                n
            } else {
                break;
            };
            self.nodes += 1;
            if self.nodes > self.max_nodes {
                self.exhausted = true;
                return;
            }
            let mut child = self.free.pop().unwrap_or_default();
            child.clear();
            child.extend_from_slice(dom);
            child[v] = (n, n);
            self.push_var_rows(v);
            if self.propagate(&mut child).is_ok() {
                self.dfs_feas(&child);
            }
            self.free.push(child);
            if self.exhausted || self.best.is_some() {
                return;
            }
        }
    }

    /// Exact check of every row at a complete assignment.
    fn rows_hold(&self, dom: &[(i64, i64)]) -> bool {
        let p = self.p;
        p.rows.iter().all(|r| {
            let (vars, coefs) = p.row_terms(r);
            let sum: i128 = vars
                .iter()
                .zip(coefs)
                .map(|(&v, &k)| i128::from(k) * i128::from(dom[v as usize].0))
                .sum();
            match r.kind {
                Kind::Eq => sum == i128::from(r.rhs),
                Kind::Le => sum <= i128::from(r.rhs),
                Kind::Ge => sum >= i128::from(r.rhs),
                Kind::Even => sum % 2 == 0,
            }
        })
    }

    fn leaf(&mut self, dom: &[(i64, i64)]) {
        if !self.rows_hold(dom) {
            return;
        }
        let (mut mx, mut sm) = (0u128, 0u128);
        for (i, &(l, _)) in dom.iter().enumerate() {
            let (m, s) = self.ex(i, l);
            mx = mx.max(m);
            sm += s;
        }
        let vals: Vec<i64> = dom.iter().map(|d| d.0).collect();
        let p = self.p;
        let better = match &self.best {
            None => true,
            Some(b) => {
                cmp_obj(
                    self.use_max,
                    (mx, sm, &vals),
                    (b.max, b.sum, &b.vals),
                    p.eps(),
                    &|o| p.pairs(&vals, o),
                    &|o| p.pairs(&b.vals, o),
                    &mut self.pool_cmp,
                ) == std::cmp::Ordering::Less
            }
        };
        if better {
            self.best = Some(Best {
                max: mx,
                sum: sm,
                vals,
            });
        }
    }
}

/// `sum(sign * coef * x) <= rhs`: tighten the domains. `false` when it cannot hold.
fn le_row(
    vars: &[u32],
    coefs: &[i64],
    sign: i64,
    rhs: i128,
    dom: &mut [(i64, i64)],
    changed: &mut Vec<usize>,
) -> bool {
    let a = |k: usize| i128::from(coefs[k] * sign);
    let mut minsum = 0i128;
    for (k, &v) in vars.iter().enumerate() {
        let (l, h) = dom[v as usize];
        minsum += if a(k) > 0 {
            a(k) * i128::from(l)
        } else {
            a(k) * i128::from(h)
        };
    }
    if minsum > rhs {
        return false;
    }
    for (k, &v) in vars.iter().enumerate() {
        let v = v as usize;
        let (l, h) = dom[v];
        let own = if a(k) > 0 {
            a(k) * i128::from(l)
        } else {
            a(k) * i128::from(h)
        };
        let slack = rhs - (minsum - own);
        if a(k) > 0 {
            let nh = div_floor(slack, a(k));
            if nh < i128::from(h) {
                if nh < i128::from(l) {
                    return false;
                }
                dom[v].1 = nh as i64;
                changed.push(v);
            }
        } else {
            let nl = div_ceil(slack, a(k));
            if nl > i128::from(l) {
                if nl > i128::from(h) {
                    return false;
                }
                dom[v].0 = nl as i64;
                changed.push(v);
            }
        }
    }
    true
}

/// Even-sum row: with at most one undecided variable the parity is forced.
fn even_row(vars: &[u32], dom: &mut [(i64, i64)], changed: &mut Vec<usize>) -> bool {
    let mut fixed = 0i64;
    let mut free: Option<usize> = None;
    for &v in vars {
        let (l, h) = dom[v as usize];
        if l == h {
            fixed += l;
        } else if free.is_some() {
            return true;
        } else {
            free = Some(v as usize);
        }
    }
    match free {
        None => fixed % 2 == 0,
        Some(v) => {
            let want = fixed.rem_euclid(2);
            let (mut l, mut h) = dom[v];
            if l.rem_euclid(2) != want {
                l += 1;
            }
            if h.rem_euclid(2) != want {
                h -= 1;
            }
            if l > h {
                return false;
            }
            if (l, h) != dom[v] {
                dom[v] = (l, h);
                changed.push(v);
            }
            true
        }
    }
}

enum Outcome {
    Solved(Vec<i64>),
    Infeasible(Option<u32>),
    /// Two rows that contradict each other (a combined-bound proof at the root).
    InfeasiblePair(u32, u32),
    /// The node budget ran out; the best assignment found so far, if any.
    Exhausted(Option<Vec<i64>>),
}

/// The counts of `[lo, hi]` whose every goal term has deviation key `<= capq`, as an interval
/// (`d` is the matching real deviation, used for the first estimate).
fn window_of(goals: &[f64], mut lo: i64, mut hi: i64, d: f64, capq: u128) -> (i64, i64) {
    if goals.is_empty() {
        return (lo, hi);
    }
    let wl = goals
        .iter()
        .map(|g| (g * (1.0 - d)).ceil())
        .fold(f64::NEG_INFINITY, f64::max)
        - 1.0;
    let wh = goals
        .iter()
        .map(|g| (g * (1.0 + d)).floor())
        .fold(f64::INFINITY, f64::min)
        + 1.0;
    lo = lo.max(wl.max(-1.0) as i64);
    hi = hi.min(wh.min(COUNT_MAX as f64) as i64);
    if lo > hi {
        return (lo, hi);
    }
    let ok = |n: i64| goals.iter().all(|&g| key_f(n as f64, g) <= capq);
    while lo <= hi && !ok(lo) {
        lo += 1;
    }
    while hi >= lo && !ok(hi) {
        hi -= 1;
    }
    (lo, hi)
}

/// Narrow every goal-bearing domain of `root` to the counts within deviation `d` (keys up to
/// `capq`) and propagate. `None` when a window is empty or propagation fails.
fn windowed(
    p: &Local,
    s: &mut Search,
    root: &[(i64, i64)],
    d: f64,
    capq: u128,
) -> Option<Vec<(i64, i64)>> {
    let mut dom = root.to_vec();
    for (v, slot) in dom.iter_mut().enumerate() {
        if p.terms(v).is_empty() {
            continue;
        }
        let (wl, wh) = window_of(p.terms(v), slot.0, slot.1, d, capq);
        *slot = (wl, wh);
        if wl > wh {
            return None;
        }
    }
    (0..p.rows.len() as u32).for_each(|r| s.push_row(r));
    s.propagate(&mut dom).ok()?;
    Some(dom)
}

/// Branch and bound on one component. With `cap = None` it finds the lexicographic optimum
/// (windowing the domains by a growing deviation); with `Some(capq)` it finds the optimum among
/// the assignments whose every goal term has key `<= capq`.
///
/// The windows start at the smallest deviation `d_min` for which bound propagation leaves a
/// non-empty window (found by bisection, so a dense row immediately gives tight domains), then
/// grow geometrically in small steps. A search that exhausts its budget without an incumbent
/// gets a feasibility-only pass before giving up.
fn solve_local(p: &Local, max_nodes: u64, cap: Option<u128>) -> Outcome {
    let base: Vec<(i64, i64)> = (0..p.nvar).map(|i| (p.lo0[i], p.hi0[i])).collect();
    let mut s = Search::new(p, max_nodes);
    s.use_max = cap.is_none();
    // A row whose coefficients share a factor that its right-hand side lacks has no integer
    // solution, however the bounds fall (propagation cannot see this).
    if cap.is_none()
        && let Some(r) = gcd_conflict(p)
    {
        return Outcome::InfeasiblePair(r, r);
    }
    // Root feasibility on the full domains.
    let mut root = base.clone();
    (0..p.rows.len() as u32).for_each(|r| s.push_row(r));
    if let Err(r) = s.propagate(&mut root) {
        return Outcome::Infeasible(Some(r));
    }
    if cap.is_none()
        && let Some((a, b)) = pair_conflict(p, &root)
    {
        return Outcome::InfeasiblePair(a, b);
    }
    let small = p.nvar <= SMALL_VARS;
    let dkey = |d: f64| (d * KEY_SCALE) as u128;
    // Deviation levels to try; `None` is the unrestricted last level.
    let mut levels: Vec<Option<(f64, u128)>> = Vec::new();
    match cap {
        Some(q) => levels.push(Some((q as f64 / KEY_SCALE, q))),
        None => {
            if small {
                // Small problems finish quickly: plain doubling windows.
                for l in 0..LEVELS {
                    let d = 0.5 * 2f64.powi(l as i32);
                    levels.push(Some((d, dkey(d))));
                }
            } else {
                // Coarse: first power-of-two deviation whose windows survive propagation.
                let mut hi_d = None;
                for l in 0..LEVELS {
                    let d = 0.5 * 2f64.powi(l as i32);
                    if windowed(p, &mut s, &root, d, dkey(d)).is_some() {
                        hi_d = Some(d);
                        break;
                    }
                }
                if let Some(hi_d) = hi_d {
                    // Fine: bisect (hi_d / 2, hi_d] for the smallest passing deviation, then
                    // grow in small geometric steps.
                    let (mut a, mut b) = (if hi_d <= 0.5 { 0.0 } else { hi_d / 2.0 }, hi_d);
                    for _ in 0..24 {
                        let m = 0.5 * (a + b);
                        if windowed(p, &mut s, &root, m, dkey(m)).is_some() {
                            b = m;
                        } else {
                            a = m;
                        }
                    }
                    let base_d = b.max(1.0 / 64.0);
                    levels.push(Some((b, dkey(b))));
                    for l in 1..=(LEVELS + 6) {
                        let d = base_d * (1.0 + 2f64.powi(l as i32 - 6));
                        levels.push(Some((d, dkey(d))));
                    }
                }
            }
            levels.push(None);
        }
    }
    let last = levels.len() - 1;
    for (li, level) in levels.into_iter().enumerate() {
        let dom = match level {
            Some((d, capq)) => match windowed(p, &mut s, &root, d, capq) {
                Some(dom) => dom,
                None => continue,
            },
            None => root.clone(),
        };
        if small {
            // The node budget is shared by all windows.
            s.best = None;
            s.dfs(&dom);
            if let Some(b) = s.best.take() {
                return if s.exhausted {
                    Outcome::Exhausted(Some(polish(p, b.vals, cap)))
                } else {
                    Outcome::Solved(b.vals)
                };
            }
            if !s.exhausted {
                if li == last {
                    return Outcome::Infeasible(None);
                }
                continue; // this window holds no assignment
            }
        }
        // A window with a feasible point has the global optimum inside it, so the search only
        // runs where one is known. Undecided or empty windows are skipped for a larger one.
        let budget = if li == last || small {
            max_nodes / 4
        } else {
            max_nodes / 16
        };
        let seed = match feasible_point(p, &dom, budget) {
            Seed::Found(vals) => vals,
            Seed::Empty if li == last => return Outcome::Infeasible(None),
            Seed::Unknown if li == last => return Outcome::Exhausted(None),
            Seed::Unknown if small => return Outcome::Exhausted(None),
            Seed::Empty | Seed::Unknown => {
                s.nodes = 0;
                s.exhausted = false;
                continue;
            }
        };
        let (mut mx, mut sm) = (0u128, 0u128);
        for (v, &x) in seed.iter().enumerate() {
            let (m, k) = s.ex(v, x);
            mx = mx.max(m);
            sm += k;
        }
        s.best = Some(Best {
            max: mx,
            sum: sm,
            vals: seed,
        });
        s.exhausted = false;
        s.nodes = 0;
        s.dfs(&dom);
        let found = s.best.take().map(|b| b.vals);
        return match found {
            Some(v) if !s.exhausted => Outcome::Solved(v),
            other => Outcome::Exhausted(other.map(|v| polish(p, v, cap))),
        };
    }
    Outcome::Infeasible(None)
}

/// Root-level infeasibility proof from pairs of rows that share a variable. Each non-parity row
/// is put in `a x <= r` form (an equality gives two); the pair is combined with small integer
/// multipliers, and `min sum (combined) x` over the root domains, tightened by the gcd of the
/// coefficients, is compared with the combined right-hand side in exact integers. This sees what
/// per-row propagation cannot, e.g. `sum_all <= 50` against `sum_odd >= 40` with every count at
/// least 1 (the even edges then need 15 more than the row allows). Returns the two row indices.
/// The work is capped (`PAIR_WORK` coefficient visits), after which pairs go unchecked.
fn pair_conflict(p: &Local, dom: &[(i64, i64)]) -> Option<(u32, u32)> {
    const PAIR_WORK: usize = 4_000_000;
    const MULTS: [(i128, i128); 7] = [(1, 1), (1, 2), (2, 1), (1, 3), (3, 1), (2, 3), (3, 2)];
    let forms = |r: &Row| -> &'static [i64] {
        match r.kind {
            Kind::Le => &[1],
            Kind::Ge => &[-1],
            Kind::Eq => &[1, -1],
            Kind::Even => &[],
        }
    };
    let mut work = 0usize;
    let mut coef = vec![0i128; p.nvar];
    let mut touched: Vec<usize> = Vec::new();
    let mut seen = vec![u32::MAX; p.rows.len()];
    for (r1, row1) in p.rows.iter().enumerate() {
        let (vars1, coefs1) = p.row_terms(row1);
        // Rows after r1 that share a variable with it, in ascending order.
        let mut partners: Vec<u32> = Vec::new();
        for &v in vars1 {
            for &r2 in &p.v_rows[p.v_off[v as usize]..p.v_off[v as usize + 1]] {
                if r2 as usize > r1 && seen[r2 as usize] != r1 as u32 {
                    seen[r2 as usize] = r1 as u32;
                    partners.push(r2);
                }
            }
        }
        partners.sort_unstable();
        for r2 in partners {
            let row2 = &p.rows[r2 as usize];
            let (vars2, coefs2) = p.row_terms(row2);
            for &s1 in forms(row1) {
                for &s2 in forms(row2) {
                    for &(m1, m2) in &MULTS {
                        work += vars1.len() + vars2.len();
                        if work > PAIR_WORK {
                            return None;
                        }
                        for &t in &touched {
                            coef[t] = 0;
                        }
                        touched.clear();
                        for (&v, &k) in vars1.iter().zip(coefs1) {
                            coef[v as usize] += m1 * i128::from(s1 * k);
                            touched.push(v as usize);
                        }
                        for (&v, &k) in vars2.iter().zip(coefs2) {
                            coef[v as usize] += m2 * i128::from(s2 * k);
                            touched.push(v as usize);
                        }
                        touched.sort_unstable();
                        touched.dedup();
                        let rhs = m1 * i128::from(s1) * i128::from(row1.rhs)
                            + m2 * i128::from(s2) * i128::from(row2.rhs);
                        let (mut minsum, mut g) = (0i128, 0i128);
                        for &t in &touched {
                            let c = coef[t];
                            if c == 0 {
                                continue;
                            }
                            let (l, h) = dom[t];
                            minsum += if c > 0 {
                                c * i128::from(l)
                            } else {
                                c * i128::from(h)
                            };
                            g = gcd(g, c.abs());
                        }
                        if g == 0 {
                            continue;
                        }
                        if div_ceil(minsum, g) > div_floor(rhs, g) {
                            return Some((r1 as u32, r2));
                        }
                    }
                }
            }
        }
    }
    None
}

/// The first equality row whose coefficient gcd does not divide its right-hand side (exact
/// integers), e.g. `2 x_1 + 4 x_2 = 81`.
fn gcd_conflict(p: &Local) -> Option<u32> {
    p.rows
        .iter()
        .position(|r| {
            if r.kind != Kind::Eq {
                return false;
            }
            let g = p
                .row_terms(r)
                .1
                .iter()
                .fold(0i128, |g, &c| gcd(g, i128::from(c).abs()));
            g > 1 && i128::from(r.rhs) % g != 0
        })
        .map(|i| i as u32)
}

fn gcd(a: i128, b: i128) -> i128 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// Most improvement passes of [`polish`].
const POLISH_PASSES: usize = 10_000;
/// Most move evaluations of one [`polish`] call.
const POLISH_EVALS: u64 = 20_000_000;

/// Deterministic local search for an incumbent of an exhausted search. It tries, in a fixed
/// order, every single-variable `+-1` move and every `+-1, +-1` pair move between variables
/// that share a row (a unit transfer is the `(+1, -1)` case), and keeps a move only if every
/// row still holds (exact integers) and the objective strictly improves: `(max, sum, vector)`
/// lexicographically, with exact tie checks; under `cap` the maximum is a constraint
/// (`max <= cap`) and the objective is `(sum, vector)`. It repeats until a full pass changes
/// nothing, or [`POLISH_PASSES`] passes or [`POLISH_EVALS`] evaluations are used. The result is
/// 1-move and pair-move locally optimal (barring those bounds), still not proven optimal.
fn polish(p: &Local, vals: Vec<i64>, cap: Option<u128>) -> Vec<i64> {
    let mut st = Polisher {
        p,
        k: Vec::new(),
        sum: 0,
        mx: 0,
        top: [(0, usize::MAX); 3],
        cap,
        x: vals,
        evals: 0,
        rsum: Vec::new(),
        pa: Vec::new(),
        pb: Vec::new(),
    };
    if st.x.len() != p.nvar || !rows_ok(p, &st.x, 0..p.rows.len()) {
        return st.x;
    }
    st.rsum = p.rows.iter().map(|r| row_sum(p, r, &st.x)).collect();
    st.k = (0..p.nvar).map(|v| st.key(v, st.x[v])).collect();
    st.refresh();
    for _ in 0..POLISH_PASSES {
        let mut any = false;
        for v in 0..p.nvar {
            for d in [-1, 1] {
                any |= st.try_move(&[(v, d)]);
            }
        }
        for r in &p.rows {
            let (vars, _) = p.row_terms(r);
            for (ai, &a) in vars.iter().enumerate() {
                for &b in &vars[ai + 1..] {
                    if st.evals > POLISH_EVALS {
                        return st.x;
                    }
                    for (da, db) in [(1, -1), (-1, 1), (1, 1), (-1, -1)] {
                        any |= st.try_move(&[(a as usize, da), (b as usize, db)]);
                    }
                }
            }
        }
        if !any || st.evals > POLISH_EVALS {
            break;
        }
    }
    st.x
}

/// Exact check of the given rows at a complete assignment.
fn rows_ok(p: &Local, x: &[i64], rows: impl IntoIterator<Item = usize>) -> bool {
    rows.into_iter()
        .all(|ri| row_holds(&p.rows[ri], row_sum(p, &p.rows[ri], x)))
}

/// Left-hand side of a row at `x` (exact).
fn row_sum(p: &Local, r: &Row, x: &[i64]) -> i128 {
    let (vars, coefs) = p.row_terms(r);
    vars.iter()
        .zip(coefs)
        .map(|(&v, &k)| i128::from(k) * i128::from(x[v as usize]))
        .sum()
}

/// Whether a row holds for the left-hand side `sum`.
fn row_holds(r: &Row, sum: i128) -> bool {
    match r.kind {
        Kind::Eq => sum == i128::from(r.rhs),
        Kind::Le => sum <= i128::from(r.rhs),
        Kind::Ge => sum >= i128::from(r.rhs),
        Kind::Even => sum % 2 == 0,
    }
}

struct Polisher<'a> {
    p: &'a Local,
    x: Vec<i64>,
    /// Per variable: (effective max key, sum key) at its current value.
    k: Vec<(u128, u128)>,
    sum: u128,
    mx: u128,
    /// The three largest effective max keys with their variables.
    top: [(u128, usize); 3],
    cap: Option<u128>,
    evals: u64,
    /// Running left-hand side of every row at `x`.
    rsum: Vec<i128>,
    /// Scratch reused by every move evaluation (no per-move allocation).
    pa: Vec<(f64, f64)>,
    pb: Vec<(f64, f64)>,
}

impl Polisher<'_> {
    fn key(&self, v: usize, n: i64) -> (u128, u128) {
        let (m, s) = self.p.exact(v, n);
        (self.cap.map_or(m, |q| m.saturating_sub(q)), s)
    }

    fn refresh(&mut self) {
        self.sum = self.k.iter().map(|k| k.1).sum();
        self.top = [(0, usize::MAX); 3];
        for (v, k) in self.k.iter().enumerate() {
            let mut e = (k.0, v);
            for slot in &mut self.top {
                if slot.1 == usize::MAX || e.0 > slot.0 {
                    std::mem::swap(slot, &mut e);
                }
            }
        }
        self.mx = self.top[0].0;
    }

    fn max_excluding(&self, skip: &[usize]) -> u128 {
        self.top
            .iter()
            .find(|t| t.1 != usize::MAX && !skip.contains(&t.1))
            .map_or(0, |t| t.0)
    }

    /// Apply `mv` (at most two `(variable, +-1)` steps) if it keeps the bounds and rows and
    /// strictly improves the objective.
    fn try_move(&mut self, mv: &[(usize, i64)]) -> bool {
        use std::cmp::Ordering::Less;
        let p = self.p;
        self.evals += 1;
        let mut skip = [usize::MAX; 2];
        let mut new_sum = self.sum;
        let mut new_loc = 0u128;
        let mut newk = [(0u128, 0u128); 2];
        for (i, &(v, d)) in mv.iter().enumerate() {
            let nv = self.x[v] + d;
            if nv < p.lo0[v] || nv > p.hi0[v] {
                return false;
            }
            newk[i] = self.key(v, nv);
            new_sum = new_sum + newk[i].1 - self.k[v].1;
            new_loc = new_loc.max(newk[i].0);
            skip[i] = v;
        }
        let new_max = self.max_excluding(&skip).max(new_loc);
        let eps = p.eps();
        if new_max > self.mx || (new_max == self.mx && new_sum > self.sum + eps) {
            return false;
        }
        for &(v, d) in mv {
            self.x[v] += d;
        }
        // Update the running row sums by the moved terms only, then check the touched rows.
        for &(v, d) in mv {
            for i in p.v_off[v]..p.v_off[v + 1] {
                self.rsum[p.v_rows[i] as usize] += i128::from(p.v_coef[i]) * i128::from(d);
            }
        }
        let mut better = mv.iter().all(|&(v, _)| {
            p.v_rows[p.v_off[v]..p.v_off[v + 1]]
                .iter()
                .all(|&r| row_holds(&p.rows[r as usize], self.rsum[r as usize]))
        });
        if better {
            better = if new_max < self.mx || new_sum + eps < self.sum {
                true
            } else {
                // Possible tie (same maximum, sums within `eps`): exact sums decide, then the
                // vector. `new` and `old` differ only at the moved variables, so the first
                // differing position is the smallest moved one and its step gives the order.
                self.pa.clear();
                for (v, &n) in self.x.iter().enumerate() {
                    self.pa.extend(p.terms(v).iter().map(|&g| (n as f64, g)));
                }
                self.pb.clear();
                self.pb.extend_from_slice(&self.pa);
                for &(v, d) in mv {
                    for t in &mut self.pb[p.t_off[v]..p.t_off[v + 1]] {
                        t.0 = (self.x[v] - d) as f64;
                    }
                }
                let first = mv.iter().min_by_key(|m| m.0).map_or(1, |m| m.1);
                let vec_order = if first > 0 {
                    std::cmp::Ordering::Greater
                } else {
                    Less
                };
                cmp_sums(&self.pa, &self.pb).then(vec_order) == Less
            };
        }
        if !better {
            for &(v, d) in mv {
                self.x[v] -= d;
                for i in p.v_off[v]..p.v_off[v + 1] {
                    self.rsum[p.v_rows[i] as usize] -= i128::from(p.v_coef[i]) * i128::from(d);
                }
            }
            return false;
        }
        for (i, &(v, _)) in mv.iter().enumerate() {
            self.k[v] = newk[i];
        }
        self.refresh();
        true
    }
}

enum Seed {
    Found(Vec<i64>),
    /// The domains hold no assignment (the search completed).
    Empty,
    /// The budget ran out in some value order.
    Unknown,
}

/// A feasible assignment inside `dom` by feasibility-only search, trying three value orders
/// with `budget` nodes each.
fn feasible_point(p: &Local, dom: &[(i64, i64)], budget: u64) -> Seed {
    for order in 0..3u8 {
        let mut f = Search::new(p, budget);
        f.order = order;
        (0..p.rows.len() as u32).for_each(|r| f.push_row(r));
        let mut d = dom.to_vec();
        if f.propagate(&mut d).is_err() {
            return Seed::Empty;
        }
        f.dfs_feas(&d);
        if let Some(b) = f.best.take() {
            return Seed::Found(b.vals);
        }
        if !f.exhausted {
            return Seed::Empty; // a completed search settles it for every order
        }
    }
    Seed::Unknown
}

/// Exact optimum of a rowless class over `[lo, hi]`: `(max, sum, n)` lexicographic, or with a
/// cap `(sum, n)` among the counts whose every term has key `<= capq`.
fn best_single(lo: i64, hi: i64, goals: &[f64], cap: Option<u128>) -> i64 {
    let cost = |n: i64| {
        let (mut mx, mut sm) = (0u128, 0u128);
        for &g in goals {
            let k = key_f(n as f64, g);
            mx = mx.max(k);
            sm += k;
        }
        (mx, sm, n)
    };
    let (lo, hi) = match cap {
        Some(q) => window_of(goals, lo, hi, q as f64 / KEY_SCALE, q),
        None => (lo, hi),
    };
    let mut cands: Vec<i64> = vec![lo, hi];
    match cap {
        None => {
            // The max deviation is V-shaped around the harmonic point of the extreme goals.
            let (gmin, gmax) = goals
                .iter()
                .fold((f64::INFINITY, 0.0f64), |a, &g| (a.0.min(g), a.1.max(g)));
            let c = (2.0 / (1.0 / gmin + 1.0 / gmax)).floor() as i64;
            cands.extend((c - 2)..=(c + 3));
        }
        Some(_) => {
            // The sum is convex piecewise linear with breakpoints at the goals.
            for &g in goals {
                let f = g.floor() as i64;
                cands.extend([f - 1, f, f + 1, f + 2]);
            }
        }
    }
    let mut best: Option<(u128, u128, i64)> = None;
    let eps = goals.len() as u128;
    let pairs = |n: i64, o: &mut Vec<(f64, f64)>| o.extend(goals.iter().map(|&g| (n as f64, g)));
    let mut sc = Scratch::default();
    for n in cands {
        if (lo..=hi).contains(&n) {
            let c = cost(n);
            let key = if cap.is_some() { (0, c.1, c.2) } else { c };
            let better = best.is_none_or(|b| {
                cmp_obj(
                    cap.is_none(),
                    (key.0, key.1, &[n]),
                    (b.0, b.1, &[b.2]),
                    eps,
                    &|o| pairs(n, o),
                    &|o| pairs(b.2, o),
                    &mut sc,
                ) == std::cmp::Ordering::Less
            });
            if better {
                best = Some(key);
            }
        }
    }
    best.map_or(lo, |b| b.2)
}

// ---------------------------------------------------------------------------------------------

struct Prob<'a> {
    edges: &'a [EntityId],
    class_of: Vec<u32>,
    /// `(class, constraint)` pairs worth naming in a report, sorted.
    class_cids: Vec<(u32, ConstraintId)>,
}

impl Prob<'_> {
    /// Error path only, so a linear scan is fine (the list is not sorted while it is built).
    fn cids_of(&self, class: u32) -> impl Iterator<Item = ConstraintId> + '_ {
        self.class_cids
            .iter()
            .filter(move |x| x.0 == class)
            .map(|x| x.1)
    }

    fn report(&self, reason: &str, classes: &[u32], rows: &[ConstraintId]) -> Infeasible {
        let mut cids: Vec<ConstraintId> = rows.to_vec();
        for &c in classes {
            cids.extend(self.cids_of(c));
        }
        let mut ents: Vec<EntityId> = Vec::new();
        for (i, &c) in self.class_of.iter().enumerate() {
            if classes.contains(&c) {
                ents.push(self.edges[i]);
            }
        }
        Infeasible::new(reason, cids, ents)
    }
}

impl IntervalSolver for IiaSolver {
    fn solve(
        &self,
        edges: &[EntityId],
        constraints: &[Record<IntervalConstraint>],
    ) -> Result<Vec<(EntityId, u32)>, Infeasible> {
        if edges.is_empty() && constraints.is_empty() {
            return Ok(Vec::new());
        }
        // Canonical row order (by id), so node budgets and the fallback path do not depend on
        // the order the caller listed the constraints in.
        let sorted: Vec<Record<IntervalConstraint>>;
        let constraints = if constraints.windows(2).all(|w| w[0].id <= w[1].id) {
            constraints
        } else {
            let mut v = constraints.to_vec();
            v.sort_by_key(|r| r.id);
            sorted = v;
            &sorted[..]
        };
        for r in constraints {
            if !matches!(
                r.item,
                IntervalConstraint::Fixed { .. }
                    | IntervalConstraint::Goal { .. }
                    | IntervalConstraint::Range { .. }
                    | IntervalConstraint::Linear { .. }
                    | IntervalConstraint::EvenSum { .. }
            ) {
                return Err(Infeasible::new(
                    "unsupported interval constraint kind",
                    vec![r.id],
                    Vec::new(),
                ));
            }
        }
        let n = edges.len();
        let unknown = |e: &EntityId, id: ConstraintId| {
            Infeasible::new(
                format!("edge {e} is not in the meshed region"),
                vec![id],
                vec![*e],
            )
        };
        let ix =
            |e: &EntityId, id: ConstraintId| edges.binary_search(e).map_err(|_| unknown(e, id));

        // 1. Classes from single-edge equalities.
        let mut parent: Vec<u32> = (0..n as u32).collect();
        let mut merges: Vec<(u32, ConstraintId)> = Vec::new();
        for r in constraints {
            if let IntervalConstraint::Linear {
                terms,
                rel: Rel::Eq,
                rhs: 0,
            } = &r.item
                && let Some((x, y)) = pair_difference(terms)
            {
                let (a, b) = (ix(&x, r.id)? as u32, ix(&y, r.id)? as u32);
                union(&mut parent, a, b);
                merges.push((a, r.id));
            }
        }
        let mut class_of = vec![u32::MAX; n];
        let mut root_class = vec![u32::MAX; n];
        let mut ncls = 0u32;
        for (i, slot) in class_of.iter_mut().enumerate() {
            let r = find(&mut parent, i as u32) as usize;
            if root_class[r] == u32::MAX {
                root_class[r] = ncls;
                ncls += 1;
            }
            *slot = root_class[r];
        }
        let nc = ncls as usize;
        let mut prob = Prob {
            edges,
            class_of,
            class_cids: Vec::new(),
        };
        for (a, id) in merges {
            prob.class_cids.push((prob.class_of[a as usize], id));
        }

        // 2. Per-class data.
        let mut hard: Vec<i64> = vec![0; nc];
        let mut lo: Vec<i64> = vec![1; nc];
        let mut hi: Vec<i64> = vec![COUNT_MAX; nc];
        let mut soft: Vec<(u32, f64)> = Vec::new();
        let mut dflt: Vec<(u32, f64)> = Vec::new();
        for r in constraints {
            match &r.item {
                IntervalConstraint::Fixed {
                    edge,
                    count,
                    firmness,
                } => {
                    let c = prob.class_of[ix(edge, r.id)?];
                    match firmness {
                        Firmness::Hard => {
                            prob.class_cids.push((c, r.id));
                            if *count == 0 {
                                return Err(prob.report(
                                    "hard count 0 is below the minimum of 1",
                                    &[c],
                                    &[],
                                ));
                            }
                            let h = &mut hard[c as usize];
                            if *h != 0 && *h != i64::from(*count) {
                                let (a, b) = (*h, *count);
                                return Err(prob.report(
                                    &format!("hard interval counts {a} and {b} on equal edges"),
                                    &[c],
                                    &[],
                                ));
                            }
                            *h = i64::from(*count);
                        }
                        Firmness::Soft => soft.push((c, f64::from(*count))),
                        Firmness::Default => dflt.push((c, f64::from(*count))),
                    }
                }
                IntervalConstraint::Goal { edge, goal } => {
                    let c = prob.class_of[ix(edge, r.id)?];
                    dflt.push((c, *goal));
                }
                IntervalConstraint::Range { edge, min, max } => {
                    let c = prob.class_of[ix(edge, r.id)?] as usize;
                    prob.class_cids.push((c as u32, r.id));
                    lo[c] = lo[c].max(i64::from(*min));
                    if let Some(m) = max {
                        hi[c] = hi[c].min(i64::from(*m));
                    }
                }
                _ => {}
            }
        }
        prob.class_cids.sort_unstable();
        prob.class_cids.dedup();
        for c in 0..nc {
            if hard[c] != 0 {
                if hard[c] < lo[c] || hard[c] > hi[c] {
                    return Err(prob.report("hard count outside its range", &[c as u32], &[]));
                }
                lo[c] = hard[c];
                hi[c] = hard[c];
            } else if lo[c] > hi[c] {
                return Err(prob.report("empty interval range", &[c as u32], &[]));
            }
        }
        // Goal terms per class (soft beats default beats the implicit 1.0).
        let valid = |g: f64| g.is_finite() && g > 0.0;
        soft.retain(|x| valid(x.1));
        dflt.retain(|x| valid(x.1));
        soft.sort_by_key(|x| x.0);
        dflt.sort_by_key(|x| x.0);
        let (mut si, mut di) = (0usize, 0usize);
        let mut t_off: Vec<usize> = Vec::with_capacity(nc + 1);
        let mut goals: Vec<f64> = Vec::new();
        for c in 0..nc as u32 {
            t_off.push(goals.len());
            let s_end = si + soft[si..].iter().take_while(|x| x.0 == c).count();
            let d_end = di + dflt[di..].iter().take_while(|x| x.0 == c).count();
            if hard[c as usize] == 0 {
                let pick = if s_end > si {
                    &soft[si..s_end]
                } else if d_end > di {
                    &dflt[di..d_end]
                } else {
                    &[(c, 1.0)][..]
                };
                goals.extend(pick.iter().map(|x| x.1.max(MIN_GOAL)));
            }
            si = s_end;
            di = d_end;
        }
        t_off.push(goals.len());

        // 3. Rows over classes.
        let mut rows: Vec<Row> = Vec::new();
        let mut rterms: Vec<(u32, i64)> = Vec::new();
        let mut scratch: Vec<(u32, i64)> = Vec::new();
        for r in constraints {
            let (kind, rhs) = match &r.item {
                IntervalConstraint::Linear { terms, rel, rhs }
                    if !(*rel == Rel::Eq && *rhs == 0 && pair_difference(terms).is_some()) =>
                {
                    scratch.clear();
                    for (e, k) in terms {
                        scratch.push((prob.class_of[ix(e, r.id)?], i64::from(*k)));
                    }
                    (
                        match rel {
                            Rel::Eq => Kind::Eq,
                            Rel::Le => Kind::Le,
                            Rel::Ge => Kind::Ge,
                        },
                        *rhs,
                    )
                }
                IntervalConstraint::EvenSum { edges: es } => {
                    scratch.clear();
                    for e in es {
                        scratch.push((prob.class_of[ix(e, r.id)?], 1));
                    }
                    (Kind::Even, 0)
                }
                _ => continue,
            };
            scratch.sort_unstable_by_key(|x| x.0);
            let off = rterms.len();
            let mut i = 0;
            while i < scratch.len() {
                let c = scratch[i].0;
                let mut k = 0i64;
                while i < scratch.len() && scratch[i].0 == c {
                    k += scratch[i].1;
                    i += 1;
                }
                if kind == Kind::Even {
                    k = k.rem_euclid(2);
                }
                if k != 0 {
                    rterms.push((c, k));
                }
            }
            let len = rterms.len() - off;
            if len == 0 {
                let ok = match kind {
                    Kind::Eq => rhs == 0,
                    Kind::Le => 0 <= rhs,
                    Kind::Ge => 0 >= rhs,
                    Kind::Even => true,
                };
                if !ok {
                    let ents = match &r.item {
                        IntervalConstraint::Linear { terms, .. } => {
                            terms.iter().map(|t| t.0).collect()
                        }
                        _ => Vec::new(),
                    };
                    return Err(Infeasible::new(
                        "a constant row cannot hold",
                        vec![r.id],
                        ents,
                    ));
                }
                continue;
            }
            rows.push(Row {
                cid: r.id,
                kind,
                rhs,
                off,
                len,
            });
        }

        // 4. Components over rows.
        let mut cp: Vec<u32> = (0..nc as u32).collect();
        for row in &rows {
            let t = &rterms[row.off..row.off + row.len];
            for x in &t[1..] {
                union(&mut cp, t[0].0, x.0);
            }
        }
        let mut comp_of = vec![u32::MAX; nc];
        let mut root_comp = vec![u32::MAX; nc];
        let mut ncomp = 0u32;
        for (c, slot) in comp_of.iter_mut().enumerate() {
            let r = find(&mut cp, c as u32) as usize;
            if root_comp[r] == u32::MAX {
                root_comp[r] = ncomp;
                ncomp += 1;
            }
            *slot = root_comp[r];
        }
        let ncomp = ncomp as usize;
        // Classes and rows per component (counting sort, stable).
        let mut c_off = vec![0usize; ncomp + 1];
        for &k in &comp_of {
            c_off[k as usize + 1] += 1;
        }
        for i in 0..ncomp {
            c_off[i + 1] += c_off[i];
        }
        let mut c_list = vec![0u32; nc];
        let mut fill = c_off.clone();
        for (c, &k) in comp_of.iter().enumerate() {
            let k = k as usize;
            c_list[fill[k]] = c as u32;
            fill[k] += 1;
        }
        let row_comp: Vec<u32> = rows
            .iter()
            .map(|r| comp_of[rterms[r.off].0 as usize])
            .collect();
        let mut r_off = vec![0usize; ncomp + 1];
        for &k in &row_comp {
            r_off[k as usize + 1] += 1;
        }
        for i in 0..ncomp {
            r_off[i + 1] += r_off[i];
        }
        let mut r_list = vec![0u32; rows.len()];
        let mut fill = r_off.clone();
        for (i, &k) in row_comp.iter().enumerate() {
            r_list[fill[k as usize]] = i as u32;
            fill[k as usize] += 1;
        }

        // 5. Solve.
        let mut value = vec![0i64; nc];
        let mut heavy: Vec<usize> = Vec::new();
        for k in 0..ncomp {
            let classes = &c_list[c_off[k]..c_off[k + 1]];
            if r_off[k] == r_off[k + 1] {
                // A rowless component is one class.
                let c = classes[0] as usize;
                if hard[c] != 0 {
                    value[c] = hard[c];
                } else {
                    if lo[c] > hi[c] {
                        return Err(prob.report("empty interval range", &[c as u32], &[]));
                    }
                    value[c] = best_single(lo[c], hi[c], &goals[t_off[c]..t_off[c + 1]], None);
                }
            } else {
                heavy.push(k);
            }
        }
        let build = |k: usize| -> Local {
            let classes = &c_list[c_off[k]..c_off[k + 1]];
            let nvar = classes.len();
            let local_of = |c: u32| classes.binary_search(&c).unwrap_or(0) as u32;
            let mut lt_off = Vec::with_capacity(nvar + 1);
            let mut lgoals = Vec::new();
            let mut center = Vec::with_capacity(nvar);
            for &c in classes {
                lt_off.push(lgoals.len());
                let g = &goals[t_off[c as usize]..t_off[c as usize + 1]];
                lgoals.extend_from_slice(g);
                let (gmin, gmax) = g
                    .iter()
                    .fold((f64::INFINITY, 0.0f64), |a, &x| (a.0.min(x), a.1.max(x)));
                center.push(if g.is_empty() {
                    0.0
                } else {
                    2.0 / (1.0 / gmin + 1.0 / gmax)
                });
            }
            lt_off.push(lgoals.len());
            let mut lrows = Vec::new();
            let (mut r_var, mut r_coef) = (Vec::new(), Vec::new());
            for &ri in &r_list[r_off[k]..r_off[k + 1]] {
                let row = &rows[ri as usize];
                let off = r_var.len();
                for &(c, kf) in &rterms[row.off..row.off + row.len] {
                    r_var.push(local_of(c));
                    r_coef.push(kf);
                }
                lrows.push(Row {
                    cid: row.cid,
                    kind: row.kind,
                    rhs: row.rhs,
                    off,
                    len: row.len,
                });
            }
            let mut v_off = vec![0usize; nvar + 1];
            for &v in &r_var {
                v_off[v as usize + 1] += 1;
            }
            for i in 0..nvar {
                v_off[i + 1] += v_off[i];
            }
            let mut v_rows = vec![0u32; r_var.len()];
            let mut v_coef = vec![0i64; r_var.len()];
            let mut fill = v_off.clone();
            for (ri, row) in lrows.iter().enumerate() {
                for j in row.off..row.off + row.len {
                    let v = r_var[j] as usize;
                    v_rows[fill[v]] = ri as u32;
                    v_coef[fill[v]] = r_coef[j];
                    fill[v] += 1;
                }
            }
            Local {
                nvar,
                lo0: classes.iter().map(|&c| lo[c as usize]).collect(),
                hi0: classes.iter().map(|&c| hi[c as usize]).collect(),
                t_off: lt_off,
                goals: lgoals,
                center,
                rows: lrows,
                r_var,
                r_coef,
                v_off,
                v_rows,
                v_coef,
            }
        };
        let work = |&k: &usize| -> (usize, Outcome, Option<Local>) {
            if c_off[k + 1] - c_off[k] > self.max_component_classes {
                return (k, Outcome::Exhausted(None), None);
            }
            let p = build(k);
            let out = solve_local(&p, self.max_nodes, None);
            (k, out, Some(p))
        };
        let results: Vec<(usize, Outcome, Option<Local>)> = if heavy.len() > 1 {
            heavy.par_iter().map(work).collect()
        } else {
            heavy.iter().map(work).collect()
        };
        let comp_classes = |k: usize| &c_list[c_off[k]..c_off[k + 1]];
        // (max, sum) of a component assignment given in the order of its classes.
        let cost_of = |k: usize, vals: &[i64]| -> (u128, u128) {
            let (mut mx, mut sm) = (0u128, 0u128);
            for (j, &c) in comp_classes(k).iter().enumerate() {
                for &g in &goals[t_off[c as usize]..t_off[c as usize + 1]] {
                    let key = key_f(vals[j] as f64, g);
                    mx = mx.max(key);
                    sm += key;
                }
            }
            (mx, sm)
        };
        // Order of two assignments of component `k` under the objective (the maximum only when
        // `with_max`): exact on ties of the quantized sums, then the smaller vector.
        let cmp_pool = std::cell::RefCell::new(Scratch::default());
        let cmp_cost = |k: usize, a: &[i64], b: &[i64], with_max: bool| -> std::cmp::Ordering {
            let (ca, cb) = (cost_of(k, a), cost_of(k, b));
            let terms = |v: &[i64], out: &mut Vec<(f64, f64)>| {
                for (j, &c) in comp_classes(k).iter().enumerate() {
                    out.extend(
                        goals[t_off[c as usize]..t_off[c as usize + 1]]
                            .iter()
                            .map(|&g| (v[j] as f64, g)),
                    );
                }
            };
            let eps: u128 = comp_classes(k)
                .iter()
                .map(|&c| (t_off[c as usize + 1] - t_off[c as usize]) as u128)
                .sum();
            cmp_obj(
                with_max,
                (ca.0, ca.1, a),
                (cb.0, cb.1, b),
                eps,
                &|o| terms(a, o),
                &|o| terms(b, o),
                &mut cmp_pool.borrow_mut(),
            )
        };
        // The reference solver on one component, verified exactly. The sub-problem is the closure
        // of the constraints that touch the component's edges: a row whose cancelled terms name
        // edges of another class still constrains this component, and those classes then come
        // along (the row does not depend on their values, so ignoring their answers is exact).
        let reference_values = |k: usize| -> Result<Vec<i64>, Infeasible> {
            let mut in_set = vec![false; n];
            for i in 0..n {
                in_set[i] = comp_of[prob.class_of[i] as usize] as usize == k;
            }
            let mut touched: Vec<Vec<usize>> = Vec::with_capacity(constraints.len());
            for r in constraints {
                let mut es: Vec<usize> = Vec::new();
                let mut add = |e: &EntityId| {
                    if let Ok(i) = edges.binary_search(e) {
                        es.push(i);
                    }
                };
                match &r.item {
                    IntervalConstraint::Fixed { edge, .. }
                    | IntervalConstraint::Goal { edge, .. }
                    | IntervalConstraint::Range { edge, .. } => add(edge),
                    IntervalConstraint::Linear { terms, .. } => {
                        terms.iter().for_each(|t| add(&t.0))
                    }
                    IntervalConstraint::EvenSum { edges: es2 } => es2.iter().for_each(&mut add),
                    _ => {}
                }
                touched.push(es);
            }
            loop {
                let mut grew = false;
                for es in &touched {
                    if es.iter().any(|&i| in_set[i]) {
                        for &i in es {
                            if !in_set[i] {
                                in_set[i] = true;
                                grew = true;
                            }
                        }
                    }
                }
                if !grew {
                    break;
                }
            }
            let in_comp: Vec<usize> = (0..n).filter(|&i| in_set[i]).collect();
            let sub_edges: Vec<EntityId> = in_comp.iter().map(|&i| edges[i]).collect();
            let sub_cons: Vec<Record<IntervalConstraint>> = constraints
                .iter()
                .zip(&touched)
                .filter(|(_, es)| es.iter().any(|&i| in_set[i]))
                .map(|(r, _)| r.clone())
                .collect();
            let sol = self.inner.solve(&sub_edges, &sub_cons)?;
            let counts: Vec<u32> = sol.iter().map(|x| x.1).collect();
            if let Some(id) = first_violation(&sub_edges, &sub_cons, &counts) {
                return Err(Infeasible::new(
                    "fallback solution violates a constraint",
                    vec![id],
                    sub_edges,
                ));
            }
            // One value per class, in class order (equal edges share their value).
            let mut by_class: Vec<(u32, i64)> = in_comp
                .iter()
                .zip(&sol)
                .map(|(&e, x)| (prob.class_of[e], i64::from(x.1)))
                .collect();
            by_class.sort_unstable();
            by_class.dedup_by_key(|x| x.0);
            Ok(comp_classes(k)
                .iter()
                .map(|&c| by_class[by_class.partition_point(|x| x.0 < c)].1)
                .collect())
        };
        let mut exact: Vec<(usize, Local)> = Vec::new();
        // Components whose search ran out of budget (or was too large): not proven optimal.
        let mut unproven: Vec<(usize, Option<Local>, Option<Vec<i64>>)> = Vec::new();
        for (k, out, local) in results {
            let classes = comp_classes(k);
            let assign = |value: &mut Vec<i64>, vals: &[i64]| {
                for (i, &c) in classes.iter().enumerate() {
                    value[c as usize] = vals[i];
                }
            };
            match out {
                Outcome::Solved(vals) => {
                    assign(&mut value, &vals);
                    exact.extend(local.map(|p| (k, p)));
                }
                Outcome::InfeasiblePair(a, b) => {
                    let Some(p) = &local else { continue };
                    let (ra, rb) = (&p.rows[a as usize], &p.rows[b as usize]);
                    let single_row = a == b;
                    let mut named: Vec<u32> = Vec::new();
                    for ro in [ra, rb] {
                        named.extend(
                            p.r_var[ro.off..ro.off + ro.len]
                                .iter()
                                .map(|&v| classes[v as usize]),
                        );
                    }
                    named.sort_unstable();
                    named.dedup();
                    if single_row {
                        return Err(prob.report(
                            "a row has no integer solution (the gcd of its coefficients does not divide its right-hand side)",
                            &named,
                            &[ra.cid],
                        ));
                    }
                    return Err(prob.report(
                        "two rows cannot hold together (their combined bounds contradict)",
                        &named,
                        &[ra.cid, rb.cid],
                    ));
                }
                Outcome::Infeasible(row) => {
                    let comp_rows = r_off[k + 1] - r_off[k];
                    let single = match (row, &local) {
                        (Some(r), Some(p)) if comp_rows == 1 => Some((r, p)),
                        _ => None,
                    };
                    let (rows_named, reason): (Vec<ConstraintId>, &str) = match single {
                        Some((r, p)) => (
                            vec![p.rows[r as usize].cid],
                            "a row cannot be met with the hard counts and bounds of its edges",
                        ),
                        // Interacting rows: the conflict is a property of the whole component.
                        None => (
                            r_list[r_off[k]..r_off[k + 1]]
                                .iter()
                                .map(|&r| rows[r as usize].cid)
                                .collect(),
                            "no integer assignment satisfies the rows of this component",
                        ),
                    };
                    let named: Vec<u32> = match single {
                        Some((r, p)) => {
                            let ro = &p.rows[r as usize];
                            p.r_var[ro.off..ro.off + ro.len]
                                .iter()
                                .map(|&v| classes[v as usize])
                                .collect()
                        }
                        None => classes.to_vec(),
                    };
                    return Err(prob.report(reason, &named, &rows_named));
                }
                Outcome::Exhausted(incumbent) => {
                    // Reference fallback; keep the better of it and the search incumbent.
                    let refv = reference_values(k);
                    let chosen = match (incumbent, &refv) {
                        (None, Err(e)) => {
                            // Neither the search nor the reference found an assignment; that is
                            // not a proof, so say so instead of echoing the reference's reason.
                            let rows_named: Vec<ConstraintId> = r_list[r_off[k]..r_off[k + 1]]
                                .iter()
                                .map(|&r| rows[r as usize].cid)
                                .collect();
                            let mut inf = prob.report(
                                "search budget exhausted, feasibility unproven (the reference solver found no assignment either)",
                                classes,
                                &rows_named,
                            );
                            inf.constraints.extend(e.constraints.iter().copied());
                            inf.constraints.sort_unstable();
                            inf.constraints.dedup();
                            return Err(inf);
                        }
                        (None, Ok(r)) => r.clone(),
                        (Some(i), Err(_)) => i,
                        (Some(i), Ok(r)) => {
                            // Phase 1: the maximum counts first (D* is global), then the sum, then the vector.
                            if cmp_cost(k, &i, r, true) != std::cmp::Ordering::Greater {
                                i
                            } else {
                                r.clone()
                            }
                        }
                    };
                    let chosen = match &local {
                        Some(p) => polish(p, chosen, None),
                        None => chosen,
                    };
                    assign(&mut value, &chosen);
                    unproven.push((k, local, refv.ok()));
                }
            }
        }

        // Phase 2. The maximum deviation is global: a component whose own optimum is below the
        // global maximum D* now minimizes its sum (then its vector) under the cap D*.
        let class_max = |c: usize, v: i64| -> u128 {
            goals[t_off[c]..t_off[c + 1]]
                .iter()
                .map(|&g| key_f(v as f64, g))
                .max()
                .unwrap_or(0)
        };
        let dstar = (0..nc).map(|c| class_max(c, value[c])).max().unwrap_or(0);
        for k in 0..ncomp {
            if r_off[k] != r_off[k + 1] {
                continue;
            }
            let c = c_list[c_off[k]] as usize;
            if hard[c] == 0 && class_max(c, value[c]) < dstar {
                value[c] = best_single(lo[c], hi[c], &goals[t_off[c]..t_off[c + 1]], Some(dstar));
            }
        }
        let comp_vals = |k: usize, value: &[i64]| -> Vec<i64> {
            comp_classes(k).iter().map(|&c| value[c as usize]).collect()
        };
        let redo: Vec<usize> = (0..exact.len())
            .filter(|&i| cost_of(exact[i].0, &comp_vals(exact[i].0, &value)).0 < dstar)
            .collect();
        let redo_one = |&i: &usize| solve_local(&exact[i].1, self.max_nodes, Some(dstar));
        let redone: Vec<Outcome> = if redo.len() > 1 {
            redo.par_iter().map(redo_one).collect()
        } else {
            redo.iter().map(redo_one).collect()
        };
        // Candidates for a component under the cap: eligible when their maximum is within D*;
        // the smaller (sum, vector) wins, the current assignment included.
        let pick = |k: usize, current: Vec<i64>, others: Vec<Vec<i64>>| -> Vec<i64> {
            let mut best = current;
            for cand in others {
                let (cm, bm) = (cost_of(k, &cand).0, cost_of(k, &best).0);
                if cm <= dstar
                    && (bm > dstar || cmp_cost(k, &cand, &best, false) == std::cmp::Ordering::Less)
                {
                    best = cand;
                }
            }
            best
        };
        for (&i, out) in redo.iter().zip(redone) {
            let k = exact[i].0;
            let current = comp_vals(k, &value);
            let others = match out {
                Outcome::Solved(v) | Outcome::Exhausted(Some(v)) => vec![v],
                _ => Vec::new(),
            };
            let chosen = pick(k, current, others);
            for (j, &c) in comp_classes(k).iter().enumerate() {
                value[c as usize] = chosen[j];
            }
        }
        for (k, local, refv) in unproven {
            let current = comp_vals(k, &value);
            let mut others: Vec<Vec<i64>> = refv.into_iter().collect();
            if let Some(p) = &local
                && let Outcome::Solved(v) | Outcome::Exhausted(Some(v)) =
                    solve_local(p, self.max_nodes, Some(dstar))
            {
                others.push(v);
            }
            let mut chosen = pick(k, current, others);
            if let Some(p) = &local {
                chosen = polish(p, chosen, Some(dstar));
            }
            for (j, &c) in comp_classes(k).iter().enumerate() {
                value[c as usize] = chosen[j];
            }
        }

        let out: Vec<(EntityId, u32)> = (0..n)
            .map(|i| (edges[i], value[prob.class_of[i] as usize] as u32))
            .collect();
        let counts: Vec<u32> = out.iter().map(|x| x.1).collect();
        if let Some(id) = first_violation(edges, constraints, &counts) {
            return Err(Infeasible::new(
                "internal: a solution failed verification",
                vec![id],
                Vec::new(),
            ));
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cmp::Ordering::{Greater, Less};

    /// Phase 1 compares the maximum first: a candidate with a lower sum but a higher maximum
    /// must lose to one with a lower maximum. Without the maximum (the capped phase-2 pick) the
    /// sum decides.
    #[test]
    fn objective_puts_the_maximum_before_the_sum() {
        let none: Pairs = &|_| ();
        let mut sc = Scratch::default();
        let (inc, refv) = ([1i64, 1], [2i64, 2]);
        let a = (10u128, 100u128, &inc[..]); // lower max, higher sum
        let b = (20u128, 50u128, &refv[..]); // higher max, lower sum
        assert_eq!(cmp_obj(true, a, b, 0, none, none, &mut sc), Less);
        assert_eq!(cmp_obj(false, a, b, 0, none, none, &mut sc), Greater);
    }
}
