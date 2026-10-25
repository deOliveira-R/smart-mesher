//! Optimality against exhaustive enumeration: a seeded property test with at most 6 variables
//! and counts at most 8. The solver's answer must be exactly the lexicographic minimizer
//! (max relative deviation, sum of relative deviations, smallest count vector) that brute force
//! finds, and `Err` exactly when no assignment satisfies the rows.
mod common;

use common::{edge, records};
use proptest::prelude::*;
use proptest::test_runner::{Config, RngAlgorithm, TestRng, TestRunner};
use sm_core::EntityId;
use sm_scheme_api::{Firmness, IntervalConstraint, IntervalSolver, Rel};
use sm_sizing::{IiaSolver, deviation_key};

const MAXC: u32 = 8;
/// Enumeration limit of the unbounded-domain variant.
const MAXU: u32 = 20;

#[derive(Clone, Debug)]
enum RowSpec {
    Pair(usize, usize),
    Lin { coefs: Vec<i32>, rel: u8, rhs: i64 },
    /// A single-variable row, so root propagation can fix or tighten a variable.
    One { var: usize, rel: u8, rhs: i64 },
    Even(Vec<usize>),
}

#[derive(Clone, Debug)]
struct Case {
    n: usize,
    /// Zero to two goals per edge: merged classes then carry several goal terms.
    goals: Vec<Vec<f64>>,
    fixed: Vec<Option<(u32, u8)>>,
    rows: Vec<RowSpec>,
}

/// Cases tuned to multi-goal classes (pair merges, Goal + Fixed on one edge) and to rows that let
/// propagation fix variables. `n_max` bounds the variable count, `gmax` the goals.
fn case_strategy(n_max: usize, gmax: f64) -> impl Strategy<Value = Case> {
    (2usize..=n_max).prop_flat_map(move |n| {
        let goals = prop::collection::vec(prop::collection::vec(0.3f64..gmax, 0..=2), n);
        let soft = prop::collection::vec(prop::option::weighted(0.35, (1u32..=MAXC, 1u8..3)), n);
        let hard = prop::collection::vec(prop::option::weighted(0.08, 1u32..=MAXC), n);
        // A few Hard fixes on top of the mostly Soft/Default ones.
        let fixed = (soft, hard).prop_map(|(f, h)| {
            f.iter().zip(&h).map(|(a, h)| h.map(|c| (c, 0u8)).or(*a)).collect::<Vec<_>>()
        });
        let row = prop_oneof![
            3 => (0..n, 0..n).prop_filter("distinct", |(a, b)| a != b).prop_map(|(a, b)| RowSpec::Pair(a, b)),
            4 => (prop::collection::vec(-3i32..=3, n), 0u8..3, -6i64..=16).prop_map(|(coefs, rel, rhs)| RowSpec::Lin { coefs, rel, rhs }),
            2 => (0..n, 0u8..3, 1i64..=10).prop_map(|(var, rel, rhs)| RowSpec::One { var, rel, rhs }),
            1 => prop::collection::vec(0..n, 1..=4).prop_map(RowSpec::Even),
        ];
        let rows = prop::collection::vec(row, 0..=4);
        (goals, fixed, rows).prop_map(move |(goals, fixed, rows)| Case { n, goals, fixed, rows })
    })
}

fn rel_of(k: u8) -> Rel {
    match k {
        0 => Rel::Eq,
        1 => Rel::Le,
        _ => Rel::Ge,
    }
}

fn firm_of(k: u8) -> Firmness {
    match k {
        0 => Firmness::Hard,
        1 => Firmness::Soft,
        _ => Firmness::Default,
    }
}

fn build(c: &Case, bounded: bool) -> (Vec<EntityId>, Vec<IntervalConstraint>) {
    let e: Vec<EntityId> = (1..=c.n as u64).map(edge).collect();
    let mut items = Vec::new();
    for (i, &ed) in e.iter().enumerate() {
        items.push(IntervalConstraint::Range { edge: ed, min: 1, max: bounded.then_some(MAXC) });
        for &g in &c.goals[i] {
            items.push(IntervalConstraint::Goal { edge: ed, goal: g });
        }
        if let Some((n, f)) = c.fixed[i] {
            items.push(IntervalConstraint::Fixed { edge: ed, count: n, firmness: firm_of(f) });
        }
    }
    for r in &c.rows {
        items.push(match r {
            RowSpec::Pair(a, b) => IntervalConstraint::sum_equal(&[e[*a]], &[e[*b]]),
            RowSpec::Lin { coefs, rel, rhs } => IntervalConstraint::Linear {
                terms: coefs.iter().enumerate().filter(|(_, k)| **k != 0).map(|(i, k)| (e[i], *k)).collect(),
                rel: rel_of(*rel),
                rhs: *rhs,
            },
            RowSpec::One { var, rel, rhs } => {
                IntervalConstraint::Linear { terms: vec![(e[*var], 1)], rel: rel_of(*rel), rhs: *rhs }
            }
            RowSpec::Even(ix) => IntervalConstraint::EvenSum { edges: ix.iter().map(|&i| e[i]).collect() },
        });
    }
    (e, items)
}

/// Independent re-derivation of the classes: edge indices joined by `x - y = 0` rows.
fn classes(items: &[IntervalConstraint], e: &[EntityId]) -> Vec<usize> {
    let ix = |x: &EntityId| e.iter().position(|y| y == x).expect("edge");
    let mut cls: Vec<usize> = (0..e.len()).collect();
    for it in items {
        if let IntervalConstraint::Linear { terms, rel: Rel::Eq, rhs: 0 } = it {
            let live: Vec<_> = terms.iter().filter(|t| t.1 != 0).collect();
            if live.len() == 2 && live[0].1 == -live[1].1 && live[0].1.abs() == 1 {
                let (a, b) = (cls[ix(&live[0].0)], cls[ix(&live[1].0)]);
                let (lo, hi) = (a.min(b), a.max(b));
                for c in &mut cls {
                    if *c == hi {
                        *c = lo;
                    }
                }
            }
        }
    }
    cls
}

/// Constraints compiled to vector indices, so the enumeration does no lookups.
type LinRow = (Vec<(usize, i64)>, Rel, i64);

struct Compiled {
    hard: Vec<(usize, i64)>,
    ranges: Vec<(usize, i64, i64)>,
    lin: Vec<LinRow>,
    even: Vec<Vec<usize>>,
}

fn compile(items: &[IntervalConstraint], e: &[EntityId]) -> Compiled {
    let ix = |x: &EntityId| e.iter().position(|y| y == x).expect("edge");
    let mut c = Compiled { hard: vec![], ranges: vec![], lin: vec![], even: vec![] };
    for it in items {
        match it {
            IntervalConstraint::Fixed { edge, count, firmness: Firmness::Hard } => c.hard.push((ix(edge), i64::from(*count))),
            IntervalConstraint::Range { edge, min, max } => {
                c.ranges.push((ix(edge), i64::from(*min), max.map_or(i64::MAX, i64::from)));
            }
            IntervalConstraint::Linear { terms, rel, rhs } => {
                c.lin.push((terms.iter().map(|(x, k)| (ix(x), i64::from(*k))).collect(), *rel, *rhs));
            }
            IntervalConstraint::EvenSum { edges } => c.even.push(edges.iter().map(ix).collect()),
            _ => {}
        }
    }
    c
}

fn holds(c: &Compiled, v: &[u32]) -> bool {
    let at = |i: usize| i64::from(v[i]);
    c.hard.iter().all(|&(i, k)| at(i) == k)
        && c.ranges.iter().all(|&(i, lo, hi)| at(i) >= lo && at(i) <= hi)
        && c.lin.iter().all(|(t, rel, rhs)| {
            let s: i64 = t.iter().map(|&(i, k)| k * at(i)).sum();
            match rel {
                Rel::Eq => s == *rhs,
                Rel::Le => s <= *rhs,
                Rel::Ge => s >= *rhs,
            }
        })
        && c.even.iter().all(|es| es.iter().map(|&i| at(i)).sum::<i64>() % 2 == 0)
}

/// The goal terms of every class (hard classes: none; soft beats default; else the implicit 1).
fn terms_of(items: &[IntervalConstraint], e: &[EntityId], cls: &[usize]) -> Vec<Vec<f64>> {
    let ix = |x: &EntityId| e.iter().position(|y| y == x).expect("edge");
    let mut hard = vec![false; e.len()];
    let (mut soft, mut dflt) = (vec![Vec::new(); e.len()], vec![Vec::new(); e.len()]);
    for it in items {
        match it {
            IntervalConstraint::Fixed { edge, count, firmness } => {
                let c = cls[ix(edge)];
                match firmness {
                    Firmness::Hard => hard[c] = true,
                    Firmness::Soft => soft[c].push(f64::from(*count)),
                    Firmness::Default => dflt[c].push(f64::from(*count)),
                }
            }
            IntervalConstraint::Goal { edge, goal } => dflt[cls[ix(edge)]].push(*goal),
            _ => {}
        }
    }
    (0..e.len())
        .map(|c| {
            if hard[c] {
                Vec::new()
            } else if !soft[c].is_empty() {
                soft[c].clone()
            } else if !dflt[c].is_empty() {
                dflt[c].clone()
            } else {
                vec![1.0]
            }
        })
        .collect()
}

fn brute(items: &[IntervalConstraint], e: &[EntityId], maxc: u32) -> Option<((u128, u128), Vec<u32>)> {
    let n = e.len();
    let cls = classes(items, e);
    let terms = terms_of(items, e, &cls);
    let comp = compile(items, e);
    let mut v = vec![1u32; n];
    let mut best: Option<((u128, u128), Vec<u32>)> = None;
    loop {
        if holds(&comp, &v) {
            let (mut mx, mut sm) = (0u128, 0u128);
            for c in 0..n {
                if cls[c] != c {
                    continue;
                }
                for &g in &terms[c] {
                    let k = deviation_key(v[c], g);
                    mx = mx.max(k);
                    sm += k;
                }
            }
            let cand = ((mx, sm), v.clone());
            if best.as_ref().is_none_or(|b| cand < *b) {
                best = Some(cand);
            }
        }
        let mut i = 0;
        loop {
            if i == n {
                return best;
            }
            if v[i] < maxc {
                v[i] += 1;
                break;
            }
            v[i] = 1;
            i += 1;
        }
    }
}

fn cases_per_seed() -> u32 {
    if cfg!(debug_assertions) { 400 } else { 3000 }
}

const SEEDS: [u8; 5] = [7, 21, 33, 58, 91];

#[test]
fn matches_exhaustive_enumeration() {
    let solver = IiaSolver::default();
    let (feasible, infeasible) = (std::cell::Cell::new(0u32), std::cell::Cell::new(0u32));
    for seed in SEEDS {
        let cfg = Config { cases: cases_per_seed(), failure_persistence: None, ..Config::default() };
        let mut runner = TestRunner::new_with_rng(cfg, TestRng::from_seed(RngAlgorithm::ChaCha, &[seed; 32]));
        runner
            .run(&case_strategy(5, 9.0), |c| {
                let (e, items) = build(&c, true);
                let want = brute(&items, &e, MAXC);
                let got = solver.solve(&e, &records(items.clone()));
                match (&want, &got) {
                    (Some((_, v)), Ok(out)) => {
                        prop_assert_eq!(&out.iter().map(|x| x.1).collect::<Vec<_>>(), v, "case {:?}", c);
                        feasible.set(feasible.get() + 1);
                    }
                    (None, Err(_)) => infeasible.set(infeasible.get() + 1),
                    (Some(_), Err(err)) => prop_assert!(false, "solver says infeasible ({err}) but a solution exists: {:?}", c),
                    (None, Ok(out)) => prop_assert!(false, "solver returned {out:?} but none exists: {:?}", c),
                }
                Ok(())
            })
            .unwrap_or_else(|e| panic!("seed {seed}: {e}"));
    }
    // The generator must exercise both outcomes.
    let (feasible, infeasible) = (feasible.get(), infeasible.get());
    assert!(feasible > 300 && infeasible > 30, "feasible {feasible}, infeasible {infeasible}");
}

/// No upper bounds: this exercises the deviation windowing. Enumeration stops at `MAXU`; a case
/// counts only when that provably contains the optimum. Every goal term is at most `gmax`, so an
/// assignment with max deviation `D` has every goal-class count at most `gmax (1 + D)` (hard
/// counts are at most `MAXC`). The brute optimum bounds `D*` from above, so if
/// `max(gmax, MAXC) (1 + D) + 1 <= MAXU` the true optimum lies inside the enumerated box (fixed
/// soft/default counts, up to `MAXC`, are goal terms too).
#[test]
fn matches_exhaustive_enumeration_without_upper_bounds() {
    const GMAX: f64 = 6.0;
    let solver = IiaSolver::default();
    let (checked, skipped) = (std::cell::Cell::new(0u32), std::cell::Cell::new(0u32));
    for seed in SEEDS {
        let cfg = Config { cases: cases_per_seed() / 2, failure_persistence: None, ..Config::default() };
        let mut runner = TestRunner::new_with_rng(cfg, TestRng::from_seed(RngAlgorithm::ChaCha, &[seed ^ 0x55; 32]));
        runner
            .run(&case_strategy(4, GMAX), |c| {
                let (e, items) = build(&c, false);
                let want = brute(&items, &e, MAXU);
                let got = solver.solve(&e, &records(items.clone()));
                match (&want, &got) {
                    (Some(((mx, _), v)), Ok(out)) => {
                        let d = *mx as f64 / (1u64 << 40) as f64;
                        if GMAX.max(f64::from(MAXC)) * (1.0 + d) + 1.0 > f64::from(MAXU) {
                            skipped.set(skipped.get() + 1);
                            return Ok(());
                        }
                        prop_assert_eq!(&out.iter().map(|x| x.1).collect::<Vec<_>>(), v, "case {:?}", c);
                        checked.set(checked.get() + 1);
                    }
                    (Some(_), Err(err)) => prop_assert!(false, "solver says infeasible ({err}) but a solution exists: {:?}", c),
                    (None, Ok(out)) => {
                        // Only a bug if the solver's point lies inside the enumerated box.
                        prop_assert!(out.iter().any(|x| x.1 > MAXU), "solver returned {out:?} but none exists: {:?}", c);
                    }
                    (None, Err(_)) => {}
                }
                Ok(())
            })
            .unwrap_or_else(|e| panic!("seed {seed}: {e}"));
    }
    let (checked, skipped) = (checked.get(), skipped.get());
    assert!(checked > 200, "checked {checked}, skipped {skipped}");
}

fn solve_vec(e: &[EntityId], items: Vec<IntervalConstraint>) -> Vec<u32> {
    let out = IiaSolver::default().solve(e, &records(items.clone())).expect("feasible");
    let v: Vec<u32> = out.iter().map(|x| x.1).collect();
    assert_eq!(brute(&items, e, MAXC).expect("brute").1, v, "solver differs from brute force");
    v
}

fn ranges(e: &[EntityId]) -> Vec<IntervalConstraint> {
    e.iter().map(|&edge| IntervalConstraint::Range { edge, min: 1, max: Some(MAXC) }).collect()
}

fn lin(terms: &[(EntityId, i32)], rel: Rel, rhs: i64) -> IntervalConstraint {
    IntervalConstraint::Linear { terms: terms.to_vec(), rel, rhs }
}

/// Repro A: the merged class {e2, e3} has an asymmetric max-deviation curve, and the rounded
/// harmonic centre lands right of the integer minimiser.
#[test]
fn regression_a_merged_class_asymmetric_curve() {
    let e: Vec<EntityId> = (1..=4).map(edge).collect();
    let mut it = ranges(&e);
    for (i, g) in [4.0, 1.0, 3.2, 2.0].into_iter().enumerate() {
        it.push(IntervalConstraint::Goal { edge: e[i], goal: g });
    }
    it.push(IntervalConstraint::sum_equal(&[e[1]], &[e[2]]));
    it.push(lin(&[(e[0], 1), (e[3], 1)], Rel::Eq, 7));
    it.push(lin(&[(e[0], 1), (e[1], 1), (e[3], 1)], Rel::Le, 50));
    assert_eq!(solve_vec(&e, it), vec![5, 1, 1, 2]);
}

/// Repro B: a Goal plus a default Fixed on one edge gives two terms with different slopes.
#[test]
fn regression_b_goal_plus_default_fixed() {
    let e: Vec<EntityId> = (1..=3).map(edge).collect();
    let mut it = ranges(&e);
    for (i, g) in [5.0, 3.4729, 7.5359].into_iter().enumerate() {
        it.push(IntervalConstraint::Goal { edge: e[i], goal: g });
    }
    it.push(IntervalConstraint::Fixed { edge: e[1], count: 1, firmness: Firmness::Default });
    it.push(lin(&[(e[0], 2), (e[1], 1), (e[2], -1)], Rel::Eq, 6));
    assert_eq!(solve_vec(&e, it), vec![6, 1, 7]);
}

/// Repro C: root propagation fixes e1 above the first deviation cap; the other variables must
/// still be windowed by that cap (the level is not allowed to keep a variable above it).
#[test]
fn regression_c_propagation_fixed_variable_is_windowed() {
    let e: Vec<EntityId> = (1..=5).map(edge).collect();
    let mut it = ranges(&e);
    it.push(IntervalConstraint::Goal { edge: e[0], goal: 5.0 });
    it.push(IntervalConstraint::Goal { edge: e[1], goal: 3.6632585078314883 });
    it.push(IntervalConstraint::Goal { edge: e[3], goal: 6.93639931543777 });
    it.push(IntervalConstraint::Fixed { edge: e[3], count: 6, firmness: Firmness::Default });
    it.push(lin(&[(e[0], 2), (e[1], 1), (e[3], -1), (e[4], 1)], Rel::Eq, -3));
    assert_eq!(solve_vec(&e, it), vec![1, 1, 1, 7, 1]);
}

/// Larger random systems without upper bounds: the answer is always feasible and never worse than
/// the reference solver's on `(max deviation, sum of deviations)`.
#[test]
fn feasible_and_never_worse_than_the_reference_on_larger_systems() {
    use sm_scheme_api::ReferenceIntervalSolver;
    let strat = (4usize..=12).prop_flat_map(|n| {
        let goals = prop::collection::vec(prop::collection::vec(0.5f64..60.0, 0..=2), n);
        let fixed = prop::collection::vec(prop::option::weighted(0.15, (1u32..=40, 0u8..3)), n);
        let row = prop_oneof![
            (0..n, 0..n).prop_filter("distinct", |(a, b)| a != b).prop_map(|(a, b)| RowSpec::Pair(a, b)),
            (prop::collection::vec(-1i32..=1, n), 0u8..3, -3i64..=60).prop_map(|(coefs, rel, rhs)| RowSpec::Lin { coefs, rel, rhs }),
            prop::collection::vec(0..n, 1..=6).prop_map(RowSpec::Even),
        ];
        let rows = prop::collection::vec(row, 0..=6);
        (goals, fixed, rows).prop_map(move |(goals, fixed, rows)| Case { n, goals, fixed, rows })
    });
    let cfg = Config { cases: 100, failure_persistence: None, ..Config::default() };
    let mut runner = TestRunner::new_with_rng(cfg, TestRng::from_seed(RngAlgorithm::ChaCha, &[11u8; 32]));
    let objective = |items: &[IntervalConstraint], e: &[EntityId], v: &[u32]| {
        let cls = classes(items, e);
        let terms = terms_of(items, e, &cls);
        let (mut mx, mut sm) = (0u128, 0u128);
        for c in 0..e.len() {
            if cls[c] == c {
                for &g in &terms[c] {
                    let k = deviation_key(v[c], g);
                    mx = mx.max(k);
                    sm += k;
                }
            }
        }
        (mx, sm)
    };
    runner
        .run(&strat, |c| {
            let (e, items) = build(&c, true);
            let items: Vec<_> = items.into_iter().filter(|i| !matches!(i, IntervalConstraint::Range { .. })).collect();
            let recs = records(items.clone());
            let ours = IiaSolver::default().solve(&e, &recs);
            let theirs = ReferenceIntervalSolver::default().solve(&e, &recs);
            if let (Err(err), Ok(r)) = (&ours, &theirs) {
                let rv: Vec<u32> = r.iter().map(|x| x.1).collect();
                // The reference can return an infeasible vector only if it is wrong; then skip.
                prop_assert!(sm_sizing::first_violation(&e, &recs, &rv).is_some(), "ours infeasible ({err}) but the reference found {rv:?}");
            }
            if let Ok(o) = &ours {
                let ov: Vec<u32> = o.iter().map(|x| x.1).collect();
                prop_assert_eq!(sm_sizing::first_violation(&e, &recs, &ov), None);
                if let Ok(r) = &theirs {
                    let rv: Vec<u32> = r.iter().map(|x| x.1).collect();
                    if sm_sizing::first_violation(&e, &recs, &rv).is_none() {
                        prop_assert!(objective(&items, &e, &ov) <= objective(&items, &e, &rv), "ours {ov:?} worse than reference {rv:?}");
                    }
                }
            }
            Ok(())
        })
        .unwrap_or_else(|e| panic!("{e}"));
}
