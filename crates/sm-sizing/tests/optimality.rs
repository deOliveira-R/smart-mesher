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

#[derive(Clone, Debug)]
enum RowSpec {
    Pair(usize, usize),
    Lin { coefs: Vec<i32>, rel: u8, rhs: i64 },
    Even(Vec<usize>),
}

#[derive(Clone, Debug)]
struct Case {
    n: usize,
    goals: Vec<Option<f64>>,
    fixed: Vec<Option<(u32, u8)>>,
    rows: Vec<RowSpec>,
}

fn case_strategy() -> impl Strategy<Value = Case> {
    (2usize..=6).prop_flat_map(|n| {
        let goals = prop::collection::vec(prop::option::weighted(0.7, 0.3f64..9.0), n);
        let fixed = prop::collection::vec(prop::option::weighted(0.25, (1u32..=MAXC, 0u8..3)), n);
        let row = prop_oneof![
            (0..n, 0..n).prop_filter("distinct", |(a, b)| a != b).prop_map(|(a, b)| RowSpec::Pair(a, b)),
            (prop::collection::vec(-2i32..=2, n), 0u8..3, -4i64..=14).prop_map(|(coefs, rel, rhs)| RowSpec::Lin { coefs, rel, rhs }),
            prop::collection::vec(0..n, 1..=4).prop_map(RowSpec::Even),
        ];
        let rows = prop::collection::vec(row, 0..=3);
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

fn build(c: &Case) -> (Vec<EntityId>, Vec<IntervalConstraint>) {
    let e: Vec<EntityId> = (1..=c.n as u64).map(edge).collect();
    let mut items = Vec::new();
    for (i, &ed) in e.iter().enumerate() {
        items.push(IntervalConstraint::Range { edge: ed, min: 1, max: Some(MAXC) });
        if let Some(g) = c.goals[i] {
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

fn holds(items: &[IntervalConstraint], e: &[EntityId], v: &[u32]) -> bool {
    let at = |x: &EntityId| i64::from(v[e.iter().position(|y| y == x).expect("edge")]);
    items.iter().all(|it| match it {
        IntervalConstraint::Fixed { edge, count, firmness } => *firmness != Firmness::Hard || at(edge) == i64::from(*count),
        IntervalConstraint::Range { edge, min, max } => at(edge) >= i64::from(*min) && max.is_none_or(|m| at(edge) <= i64::from(m)),
        IntervalConstraint::Linear { terms, rel, rhs } => {
            let s: i64 = terms.iter().map(|(x, k)| i64::from(*k) * at(x)).sum();
            match rel {
                Rel::Eq => s == *rhs,
                Rel::Le => s <= *rhs,
                Rel::Ge => s >= *rhs,
            }
        }
        IntervalConstraint::EvenSum { edges } => edges.iter().map(at).sum::<i64>() % 2 == 0,
        _ => true,
    })
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

fn brute(items: &[IntervalConstraint], e: &[EntityId]) -> Option<((u128, u128), Vec<u32>)> {
    let n = e.len();
    let cls = classes(items, e);
    let terms = terms_of(items, e, &cls);
    let mut v = vec![1u32; n];
    let mut best: Option<((u128, u128), Vec<u32>)> = None;
    loop {
        if holds(items, e, &v) {
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
            if v[i] < MAXC {
                v[i] += 1;
                break;
            }
            v[i] = 1;
            i += 1;
        }
    }
}

#[test]
fn matches_exhaustive_enumeration() {
    let cfg = Config { cases: 300, failure_persistence: None, ..Config::default() };
    let rng = TestRng::from_seed(RngAlgorithm::ChaCha, &[7u8; 32]);
    let mut runner = TestRunner::new_with_rng(cfg, rng);
    let solver = IiaSolver::default();
    let feasible = std::cell::Cell::new(0u32);
    let infeasible = std::cell::Cell::new(0u32);
    runner
        .run(&case_strategy(), |c| {
            let (e, items) = build(&c);
            let want = brute(&items, &e);
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
        .unwrap_or_else(|e| panic!("{e}"));
    // The generator must exercise both outcomes.
    let (feasible, infeasible) = (feasible.get(), infeasible.get());
    assert!(feasible > 100 && infeasible > 10, "feasible {feasible}, infeasible {infeasible}");
}

/// Larger random systems without upper bounds: the answer is always feasible and never worse than
/// the reference solver's on `(max deviation, sum of deviations)`.
#[test]
fn feasible_and_never_worse_than_the_reference_on_larger_systems() {
    use sm_scheme_api::ReferenceIntervalSolver;
    let strat = (4usize..=12).prop_flat_map(|n| {
        let goals = prop::collection::vec(prop::option::weighted(0.9, 0.5f64..60.0), n);
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
            let (e, items) = build(&c);
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
