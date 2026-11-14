//! `Origin::Preference` goals in `IiaSolver`: replacement rule and agreement with the reference.
mod common;

use common::edge;
use proptest::prelude::*;
use sm_core::EntityId;
use sm_scheme_api::{
    ConstraintId, Firmness, IntervalConstraint, IntervalSolver, Origin, Record,
    ReferenceIntervalSolver, SchemeName, Target,
};
use sm_sizing::IiaSolver;
use sm_testkit::strategies::config;

fn rec(id: u32, pref: bool, item: IntervalConstraint) -> Record<IntervalConstraint> {
    let n = SchemeName::new("bias");
    let origin = if pref {
        Origin::Preference { scheme: n, target: Target::Curve(edge(1)) }
    } else {
        Origin::Driver
    };
    Record { id: ConstraintId(id), origin, item }
}

fn goal(e: EntityId, g: f64) -> IntervalConstraint {
    IntervalConstraint::Goal { edge: e, goal: g }
}

fn fixed(e: EntityId, count: u32, firmness: Firmness) -> IntervalConstraint {
    IntervalConstraint::Fixed { edge: e, count, firmness }
}

fn solve_both(edges: &[EntityId], items: Vec<(bool, IntervalConstraint)>) -> (Vec<u32>, Vec<u32>) {
    let cs: Vec<_> = items.into_iter().enumerate().map(|(i, (p, c))| rec(i as u32, p, c)).collect();
    let a = IiaSolver::default().solve(edges, &cs).expect("iia");
    let b = ReferenceIntervalSolver::default().solve(edges, &cs).expect("reference");
    (a.iter().map(|x| x.1).collect(), b.iter().map(|x| x.1).collect())
}

/// A derived count of 6 next to a model default size that alone would give 2 yields 6.
#[test]
fn preference_replaces_the_default_goal() {
    let e = [edge(1)];
    let (iia, rf) = solve_both(&e, vec![(false, goal(e[0], 2.0)), (true, goal(e[0], 6.0))]);
    assert_eq!((iia, rf), (vec![6], vec![6]));
    // Without the preference the default goal gives 2.
    let (iia, _) = solve_both(&e, vec![(false, goal(e[0], 2.0))]);
    assert_eq!(iia, vec![2]);
}

/// A user soft count on the same class still wins; a hard count wins over both.
#[test]
fn user_soft_and_hard_counts_beat_a_preference() {
    let e = [edge(1)];
    let (iia, rf) = solve_both(
        &e,
        vec![(true, goal(e[0], 6.0)), (false, fixed(e[0], 3, Firmness::Soft))],
    );
    assert_eq!((iia, rf), (vec![3], vec![3]));
    let (iia, rf) = solve_both(
        &e,
        vec![(true, goal(e[0], 6.0)), (false, fixed(e[0], 3, Firmness::Soft)), (false, fixed(e[0], 9, Firmness::Hard))],
    );
    assert_eq!((iia, rf), (vec![9], vec![9]));
}

/// Several preferences average (4 and 8 give 6) and the range still binds.
#[test]
fn preferences_average_and_ranges_bind() {
    let e = [edge(1)];
    let (iia, rf) = solve_both(&e, vec![(true, goal(e[0], 4.0)), (true, goal(e[0], 8.0))]);
    assert_eq!((iia, rf), (vec![6], vec![6]));
    let (iia, rf) = solve_both(
        &e,
        vec![(true, goal(e[0], 6.0)), (false, IntervalConstraint::Range { edge: e[0], min: 1, max: Some(4) })],
    );
    assert_eq!((iia, rf), (vec![4], vec![4]));
}

/// The preference applies to the whole equal-count class.
#[test]
fn preference_reaches_the_equal_count_class() {
    let e = [edge(1), edge(2)];
    let (iia, rf) = solve_both(
        &e,
        vec![
            (false, IntervalConstraint::sum_equal(&[e[0]], &[e[1]])),
            (false, goal(e[1], 2.0)),
            (true, goal(e[0], 6.0)),
        ],
    );
    assert_eq!((iia, rf), (vec![6, 6], vec![6, 6]));
}

#[derive(Clone, Debug)]
struct ClassSpec {
    soft: Option<u32>,
    dflt: Option<f64>,
    prefs: Vec<f64>,
    hard: Option<u32>,
    max: Option<u32>,
}

fn class_spec() -> impl Strategy<Value = ClassSpec> {
    (
        prop::option::of(1u32..40),
        prop::option::of(0.6f64..40.0),
        prop::collection::vec(0.6f64..40.0, 0..3),
        prop::option::weighted(0.15, 1u32..40),
        prop::option::weighted(0.3, 1u32..40),
    )
        .prop_map(|(soft, dflt, prefs, hard, max)| ClassSpec { soft, dflt, prefs, hard, max })
}

fn build(specs: &[ClassSpec]) -> (Vec<EntityId>, Vec<(bool, IntervalConstraint)>) {
    // Each class has two equal-count edges, so the classes are real (merged) classes.
    let edges: Vec<EntityId> = (1..=2 * specs.len() as u64).map(edge).collect();
    let mut items = Vec::new();
    for (k, s) in specs.iter().enumerate() {
        let (a, b) = (edges[2 * k], edges[2 * k + 1]);
        items.push((false, IntervalConstraint::sum_equal(&[a], &[b])));
        if let Some(c) = s.soft {
            items.push((false, fixed(b, c, Firmness::Soft)));
        }
        if let Some(g) = s.dflt {
            items.push((false, goal(a, g)));
        }
        for &g in &s.prefs {
            items.push((true, goal(b, g)));
        }
        if let Some(h) = s.hard {
            items.push((false, fixed(a, h, Firmness::Hard)));
        }
        if let Some(m) = s.max {
            let hi = s.hard.map_or(m, |h| m.max(h));
            items.push((false, IntervalConstraint::Range { edge: a, min: 1, max: Some(hi) }));
        }
    }
    (edges, items)
}

proptest! {
    #![proptest_config(config(200))]

    /// Mixed soft, preference, default and hard goals: both solvers give the same counts, and
    /// the contribution order does not matter.
    #[test]
    fn iia_equals_reference_on_mixed_goals(specs in prop::collection::vec(class_spec(), 1..6)) {
        let (edges, items) = build(&specs);
        let (iia, rf) = solve_both(&edges, items.clone());
        prop_assert_eq!(&iia, &rf);
        let mut rev = items;
        rev.reverse();
        let (iia_rev, _) = solve_both(&edges, rev);
        prop_assert_eq!(iia, iia_rev);
    }
}
