//! Solvers: the global interval solve (IIA-style) and corner picking. Both are traits so a
//! stronger solver (ILP / network flow in the sizing crate) can replace the reference ones.

use crate::error::{Infeasible, Result, SchemeError};
use crate::firm::{Firm, Firmness};
use crate::geom;
use crate::problem::{ConstraintId, IntervalConstraint, Record, Rel};
use crate::vertex::{CornerAssignment, FaceLayout, LogicalSide, SideDir, VertexRule, VertexType};
use sm_core::EntityId;
use sm_topo_api::{CornerKey, TopologyView};

/// Solves the interval system: one positive integer per edge satisfying every constraint,
/// as close as possible to the goals. Must be deterministic.
pub trait IntervalSolver: Send + Sync {
    /// Solve for `edges` (ascending). Returns `(edge, count)` ascending by edge.
    fn solve(
        &self,
        edges: &[EntityId],
        constraints: &[Record<IntervalConstraint>],
    ) -> Result<Vec<(EntityId, u32)>, Infeasible>;
}

/// Picks corner types and logical sides of a face for a corner rule.
pub trait CornerPicker: Send + Sync {
    /// Solve the corners of `face` under `rule`, honouring `requests` (rule and request
    /// constraint ids are listed in `ids` for conflict reports).
    fn pick(
        &self,
        topo: &dyn TopologyView,
        face: EntityId,
        rule: VertexRule,
        requests: &[(CornerKey, Firm<VertexType>)],
        ids: &[ConstraintId],
    ) -> Result<FaceLayout>;
}

/// Reference interval solver.
///
/// 1. Single-edge equalities are merged by union-find; each class is named by its **lowest
///    edge id** (not by the union-find root, which depends on union order).
/// 2. Each class starts at its hard value (all hard values must agree), else its largest
///    soft value, else the rounded mean of its goals and default counts, clamped to its range.
/// 3. Violated sum equalities (in constraint-id order) are repaired by the cheapest single
///    `+1`/`-1` step on a non-hard class (cost = relative deviation from the class goal,
///    soft classes cost 10x), then odd parity sums by the cheapest `+1` on a class with odd
///    multiplicity; repeat until everything holds or the step limit is hit.
///
/// Not optimal (a real IIA is an integer program); deterministic and exact for the
/// structured cases (map, submap, sweep, copy, pave parity) the reference schemes produce.
#[derive(Clone, Copy, Debug)]
pub struct ReferenceIntervalSolver {
    /// Maximum number of repair steps.
    pub max_steps: usize,
}

impl Default for ReferenceIntervalSolver {
    fn default() -> Self {
        Self { max_steps: 100_000 }
    }
}

struct Class {
    edges: Vec<usize>,
    hard: Option<(u32, ConstraintId)>,
    soft: Option<u32>,
    goal_sum: f64,
    goal_n: u32,
    min: u32,
    max: u32,
    value: u32,
    goal: f64,
    cids: Vec<ConstraintId>,
}

impl Class {
    fn weight(&self) -> f64 {
        if self.soft.is_some() { 10.0 } else { 1.0 }
    }
    fn cost(&self, n: u32) -> f64 {
        self.weight() * (f64::from(n) - self.goal).abs() / self.goal.max(1e-12)
    }
    fn adjustable(&self) -> bool {
        self.hard.is_none()
    }
}

fn find(p: &mut [usize], mut i: usize) -> usize {
    while p[i] != i {
        p[i] = p[p[i]];
        i = p[i];
    }
    i
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

fn infeasible(
    reason: impl Into<String>,
    mut cids: Vec<ConstraintId>,
    mut entities: Vec<EntityId>,
) -> Infeasible {
    cids.sort_unstable();
    cids.dedup();
    entities.sort_unstable();
    entities.dedup();
    Infeasible {
        reason: reason.into(),
        constraints: cids,
        entities,
        blame: None,
    }
}

impl IntervalSolver for ReferenceIntervalSolver {
    fn solve(
        &self,
        edges: &[EntityId],
        constraints: &[Record<IntervalConstraint>],
    ) -> Result<Vec<(EntityId, u32)>, Infeasible> {
        let ix = |e: &EntityId| edges.binary_search(e).ok();
        let unknown = |e: &EntityId, id: ConstraintId| {
            infeasible(
                format!("edge {e} is not in the meshed region"),
                vec![id],
                vec![*e],
            )
        };
        let n = edges.len();
        let mut parent: Vec<usize> = (0..n).collect();
        let mut single_eq: Vec<(ConstraintId, usize)> = Vec::new();
        for r in constraints {
            if let IntervalConstraint::Linear {
                terms,
                rel: Rel::Eq,
                rhs: 0,
            } = &r.item
                && let Some((x, y)) = pair_difference(terms)
            {
                let a = ix(&x).ok_or_else(|| unknown(&x, r.id))?;
                let b = ix(&y).ok_or_else(|| unknown(&y, r.id))?;
                let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
                if ra != rb {
                    parent[ra.max(rb)] = ra.min(rb);
                }
                single_eq.push((r.id, a));
            }
        }
        // Classes are numbered by their lowest edge index (edges are visited ascending), so the
        // naming never depends on union order.
        let mut class_of = vec![usize::MAX; n];
        let mut classes: Vec<Class> = Vec::new();
        let mut root_class: Vec<usize> = vec![usize::MAX; n];
        for (i, slot) in class_of.iter_mut().enumerate() {
            let r = find(&mut parent, i);
            if root_class[r] == usize::MAX {
                root_class[r] = classes.len();
                classes.push(Class {
                    edges: Vec::new(),
                    hard: None,
                    soft: None,
                    goal_sum: 0.0,
                    goal_n: 0,
                    min: 1,
                    max: u32::MAX,
                    value: 1,
                    goal: 1.0,
                    cids: Vec::new(),
                });
            }
            *slot = root_class[r];
            classes[root_class[r]].edges.push(i);
        }
        for (cid, a) in &single_eq {
            classes[class_of[*a]].cids.push(*cid);
        }
        let ents = |c: &Class| c.edges.iter().map(|&i| edges[i]).collect::<Vec<_>>();
        for r in constraints {
            match &r.item {
                IntervalConstraint::Fixed {
                    edge,
                    count,
                    firmness,
                } => {
                    let c = &mut classes[class_of[ix(edge).ok_or_else(|| unknown(edge, r.id))?]];
                    c.cids.push(r.id);
                    match firmness {
                        Firmness::Hard => match c.hard {
                            Some((v, _)) if v != *count => {
                                let cids = c.cids.clone();
                                return Err(infeasible(
                                    format!("hard interval counts {v} and {count} on equal edges"),
                                    cids,
                                    ents(c),
                                ));
                            }
                            _ => c.hard = Some((*count, r.id)),
                        },
                        Firmness::Soft => c.soft = Some(c.soft.map_or(*count, |s| s.max(*count))),
                        Firmness::Default => {
                            c.goal_sum += f64::from(*count);
                            c.goal_n += 1;
                        }
                    }
                }
                IntervalConstraint::Goal { edge, goal } => {
                    let c = &mut classes[class_of[ix(edge).ok_or_else(|| unknown(edge, r.id))?]];
                    if goal.is_finite() && *goal > 0.0 {
                        c.goal_sum += goal;
                        c.goal_n += 1;
                    }
                }
                IntervalConstraint::Range { edge, min, max } => {
                    let c = &mut classes[class_of[ix(edge).ok_or_else(|| unknown(edge, r.id))?]];
                    c.cids.push(r.id);
                    c.min = c.min.max(*min);
                    if let Some(m) = max {
                        c.max = c.max.min(*m);
                    }
                }
                IntervalConstraint::Linear { .. } | IntervalConstraint::EvenSum { .. } => {}
            }
        }
        for c in &mut classes {
            if c.min > c.max {
                return Err(infeasible("empty interval range", c.cids.clone(), ents(c)));
            }
            c.goal = if let Some(s) = c.soft {
                f64::from(s)
            } else if c.goal_n > 0 {
                c.goal_sum / f64::from(c.goal_n)
            } else {
                1.0
            };
            c.value = match c.hard {
                Some((v, _)) if v < c.min || v > c.max => {
                    return Err(infeasible(
                        "hard count outside its range",
                        c.cids.clone(),
                        ents(c),
                    ));
                }
                Some((v, _)) => v,
                None => (c.goal.round().max(1.0) as u32).clamp(c.min, c.max),
            };
        }
        // Linear rows as sparse integer rows over classes.
        type Row = (ConstraintId, Vec<(usize, i64)>, Rel, i64);
        let mut rows: Vec<Row> = Vec::new();
        let mut parity: Vec<(ConstraintId, Vec<(usize, i64)>)> = Vec::new();
        let add =
            |acc: &mut Vec<(usize, i64)>, c: usize, k: i64| match acc.iter_mut().find(|x| x.0 == c)
            {
                Some(x) => x.1 += k,
                None => acc.push((c, k)),
            };
        for r in constraints {
            match &r.item {
                IntervalConstraint::Linear { terms, rel, rhs }
                    if !(*rel == Rel::Eq && *rhs == 0 && pair_difference(terms).is_some()) =>
                {
                    let mut acc = Vec::new();
                    for (e, k) in terms {
                        add(
                            &mut acc,
                            class_of[ix(e).ok_or_else(|| unknown(e, r.id))?],
                            i64::from(*k),
                        );
                    }
                    acc.retain(|x| x.1 != 0);
                    acc.sort_unstable();
                    if acc.is_empty() {
                        let ok = match rel {
                            Rel::Eq => *rhs == 0,
                            Rel::Le => 0 <= *rhs,
                            Rel::Ge => 0 >= *rhs,
                        };
                        if !ok {
                            return Err(infeasible(
                                "a constant row cannot hold",
                                vec![r.id],
                                terms.iter().map(|t| t.0).collect(),
                            ));
                        }
                    } else {
                        rows.push((r.id, acc, *rel, *rhs));
                    }
                }
                IntervalConstraint::EvenSum { edges: es } => {
                    let mut acc = Vec::new();
                    for e in es {
                        add(
                            &mut acc,
                            class_of[ix(e).ok_or_else(|| unknown(e, r.id))?],
                            1,
                        );
                    }
                    acc.sort_unstable();
                    parity.push((r.id, acc));
                }
                _ => {}
            }
        }
        let row_cids = |id: ConstraintId, cs: &[(usize, i64)], classes: &[Class]| {
            let mut v = vec![id];
            let mut es = Vec::new();
            for (c, _) in cs {
                v.extend_from_slice(&classes[*c].cids);
                es.extend(ents(&classes[*c]));
            }
            (v, es)
        };
        let mut steps = 0usize;
        'outer: loop {
            // Linear rows.
            for row in &rows {
                let resid: i64 = row
                    .1
                    .iter()
                    .map(|(c, k)| k * i64::from(classes[*c].value))
                    .sum::<i64>()
                    - row.3;
                let viol = |r: i64| match row.2 {
                    Rel::Eq => r.abs(),
                    Rel::Le => r.max(0),
                    Rel::Ge => (-r).max(0),
                };
                if viol(resid) == 0 {
                    continue;
                }
                steps += 1;
                if steps > self.max_steps {
                    let (v, es) = row_cids(row.0, &row.1, &classes);
                    return Err(infeasible("interval repair did not converge", v, es));
                }
                // Candidate steps: (cost delta, class, +1/-1), reducing |resid| preferred.
                let mut best: Option<(bool, f64, usize, i64)> = None;
                for (c, k) in &row.1 {
                    let cl = &classes[*c];
                    if !cl.adjustable() {
                        continue;
                    }
                    for d in [1i64, -1] {
                        let nv = i64::from(cl.value) + d;
                        if nv < i64::from(cl.min) || nv > i64::from(cl.max) {
                            continue;
                        }
                        let new_resid = resid + k * d;
                        let improves = viol(new_resid) < viol(resid);
                        // Overshooting an equality (sign flip) is allowed when nothing improves.
                        if !improves && !(row.2 == Rel::Eq && new_resid.signum() != resid.signum())
                        {
                            continue;
                        }
                        let dc = cl.cost(nv as u32) - cl.cost(cl.value);
                        let cand = (!improves, dc, *c, -d);
                        let better = match best {
                            None => true,
                            Some(b) => {
                                (cand.0, cand.1, cand.2, cand.3).partial_cmp(&(b.0, b.1, b.2, b.3))
                                    == Some(core::cmp::Ordering::Less)
                            }
                        };
                        if better {
                            best = Some(cand);
                        }
                    }
                }
                match best {
                    Some((_, _, c, negd)) => {
                        let d = -negd;
                        classes[c].value = (i64::from(classes[c].value) + d) as u32;
                        continue 'outer;
                    }
                    None => {
                        let (v, es) = row_cids(row.0, &row.1, &classes);
                        return Err(infeasible(
                            "linear row cannot be met (hard counts or bounds)",
                            v,
                            es,
                        ));
                    }
                }
            }
            // Parity.
            for row in &parity {
                let sum: i64 = row
                    .1
                    .iter()
                    .map(|(c, k)| k * i64::from(classes[*c].value))
                    .sum();
                if sum % 2 == 0 {
                    continue;
                }
                steps += 1;
                if steps > self.max_steps {
                    let (v, es) = row_cids(row.0, &row.1, &classes);
                    return Err(infeasible("parity repair did not converge", v, es));
                }
                let mut best: Option<(f64, usize)> = None;
                for (c, k) in &row.1 {
                    let cl = &classes[*c];
                    if k % 2 == 0 || !cl.adjustable() || cl.value >= cl.max {
                        continue;
                    }
                    let dc = cl.cost(cl.value + 1) - cl.cost(cl.value);
                    if best.is_none_or(|b| dc < b.0) {
                        best = Some((dc, *c));
                    }
                }
                match best {
                    Some((_, c)) => {
                        classes[c].value += 1;
                        continue 'outer;
                    }
                    None => {
                        let (v, es) = row_cids(row.0, &row.1, &classes);
                        return Err(infeasible(
                            "odd interval sum on a loop that needs an even count",
                            v,
                            es,
                        ));
                    }
                }
            }
            break;
        }
        Ok(edges
            .iter()
            .enumerate()
            .map(|(i, e)| (*e, classes[class_of[i]].value))
            .collect())
    }
}

/// Reference corner picker: measures interior angles ([`geom::corners`]), applies requests
/// (the firmest wins; equal firmness: the later request), then
/// - `Map` / `Ends(n)`: one loop; keeps hard `End`s, adds soft `End`s, then the smallest
///   angles until there are `n`; every other corner becomes `Side` (hard `Corner` /
///   `Reversal` are infeasible);
/// - `Submap`: nearest-angle types; the corner sum must be 4;
/// - `Free`: nearest-angle types, no sides.
///
/// Sides start after the first `End` (Map/Ends) or the first non-`Side` corner (Submap) in
/// loop order.
#[derive(Clone, Copy, Debug, Default)]
pub struct ReferenceCornerPicker;

impl CornerPicker for ReferenceCornerPicker {
    fn pick(
        &self,
        topo: &dyn TopologyView,
        face: EntityId,
        rule: VertexRule,
        requests: &[(CornerKey, Firm<VertexType>)],
        ids: &[ConstraintId],
    ) -> Result<FaceLayout> {
        let corners = geom::corners(topo, face)?;
        let bad =
            |reason: String| -> SchemeError { infeasible(reason, ids.to_vec(), vec![face]).into() };
        let mut out: Vec<CornerAssignment> = corners
            .iter()
            .map(|c| {
                let mut req: Option<Firm<VertexType>> = None;
                for (k, t) in requests {
                    if *k == c.key && req.is_none_or(|r| t.firmness >= r.firmness) {
                        req = Some(*t);
                    }
                }
                match req {
                    Some(r) => CornerAssignment {
                        corner: c.key,
                        vertex_type: r.value,
                        firmness: r.firmness,
                        angle: c.angle,
                    },
                    None => CornerAssignment {
                        corner: c.key,
                        vertex_type: VertexType::nearest(c.angle),
                        firmness: Firmness::Default,
                        angle: c.angle,
                    },
                }
            })
            .collect();
        let rings = corners.iter().map(|c| c.ring).max().map_or(0, |r| r + 1);
        let structured = !matches!(rule, VertexRule::Free);
        if structured && rings != 1 {
            return Err(bad(format!(
                "{rule:?} needs exactly one loop, face has {rings}"
            )));
        }
        match rule {
            VertexRule::Free => {}
            VertexRule::Map | VertexRule::Ends(_) => {
                let want = if let VertexRule::Ends(n) = rule {
                    usize::from(n)
                } else {
                    4
                };
                if let Some(c) = out.iter().find(|c| {
                    c.firmness == Firmness::Hard
                        && matches!(c.vertex_type, VertexType::Corner | VertexType::Reversal)
                }) {
                    return Err(bad(format!(
                        "hard {:?} at {:?} is not allowed by {rule:?}",
                        c.vertex_type, c.corner
                    )));
                }
                let hard_ends = out
                    .iter()
                    .filter(|c| c.firmness == Firmness::Hard && c.vertex_type == VertexType::End)
                    .count();
                if hard_ends > want {
                    return Err(bad(format!(
                        "{hard_ends} hard End corners, {rule:?} needs {want}"
                    )));
                }
                // Rank free corners: soft End first, then by angle, then loop order.
                let mut free: Vec<usize> = (0..out.len())
                    .filter(|&i| out[i].firmness != Firmness::Hard)
                    .collect();
                free.sort_by(|&a, &b| {
                    let sa =
                        out[a].firmness == Firmness::Soft && out[a].vertex_type == VertexType::End;
                    let sb =
                        out[b].firmness == Firmness::Soft && out[b].vertex_type == VertexType::End;
                    sb.cmp(&sa)
                        .then(out[a].angle.total_cmp(&out[b].angle))
                        .then(a.cmp(&b))
                });
                let need = want - hard_ends;
                if free.len() < need {
                    return Err(bad(format!(
                        "{rule:?} needs {want} End corners, face has {}",
                        hard_ends + free.len()
                    )));
                }
                for (rank, &i) in free.iter().enumerate() {
                    out[i].vertex_type = if rank < need {
                        VertexType::End
                    } else {
                        VertexType::Side
                    };
                }
            }
            VertexRule::Submap => {
                let sum: i32 = out.iter().map(|c| c.vertex_type.submap_weight()).sum();
                if sum != 4 {
                    return Err(bad(format!("submap corner sum is {sum}, needs 4")));
                }
            }
        }
        let sides = if structured {
            sides_of(&out, matches!(rule, VertexRule::Submap))
        } else {
            Vec::new()
        };
        Ok(FaceLayout {
            face,
            rule,
            corners: out,
            sides,
        })
    }
}

/// Split the (single) loop into logical sides at `End` corners (or at every non-`Side`
/// corner when `any_turn`). Corner `i` sits at the end of coedge `i`. Each side's direction
/// comes from the cumulative quarter turns of the corners before it (End +1, Side 0,
/// Corner -1, Reversal -2), starting at `PlusI`.
fn sides_of(corners: &[CornerAssignment], any_turn: bool) -> Vec<LogicalSide> {
    let is_break = |c: &CornerAssignment| {
        if any_turn {
            c.vertex_type != VertexType::Side
        } else {
            c.vertex_type == VertexType::End
        }
    };
    let Some(first) = corners.iter().position(is_break) else {
        return Vec::new();
    };
    let n = corners.len();
    let mut sides = Vec::new();
    let mut cur = Vec::new();
    let mut dir = SideDir::PlusI;
    for s in 1..=n {
        let i = (first + s) % n;
        cur.push(corners[i].corner.incoming);
        if is_break(&corners[i]) {
            sides.push(LogicalSide {
                coedges: std::mem::take(&mut cur),
                dir,
            });
            // Non-break corners turn nothing (Map/Ends rules keep them as Side).
            dir = dir.turned(if any_turn {
                corners[i].vertex_type.submap_weight()
            } else {
                1
            });
        }
    }
    sides
}
