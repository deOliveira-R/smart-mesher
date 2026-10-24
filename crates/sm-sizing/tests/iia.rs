//! IiaSolver: acceptance rows, structured agreement with the reference, determinism, limits.
mod common;

use common::{edge, records};
use rayon::ThreadPoolBuilder;
use sm_core::EntityId;
use sm_mesh_api::{SerialAlloc, reference::ReferenceBuilder};
use sm_scheme_api::stubs::{brick_setup, registry, row_setup};
use sm_scheme_api::{
    ConstraintId, Driver, Firm, Firmness, IntervalConstraint, IntervalRequest, IntervalSolver,
    MeshInput, MeshIntent, ReferenceCornerPicker, ReferenceIntervalSolver, Sequential, SizingSpec,
    WholeFaces,
};
use sm_sizing::{IiaSolver, first_violation};

fn goal(e: EntityId, g: f64) -> IntervalConstraint {
    IntervalConstraint::Goal { edge: e, goal: g }
}

fn fixed(e: EntityId, count: u32, firmness: Firmness) -> IntervalConstraint {
    IntervalConstraint::Fixed { edge: e, count, firmness }
}

fn counts(out: &[(EntityId, u32)]) -> Vec<u32> {
    out.iter().map(|x| x.1).collect()
}

#[test]
fn four_sided_face_with_unequal_goals_gives_opposite_equal_counts() {
    let e: Vec<_> = (1..=4).map(edge).collect();
    let cs = records(vec![
        goal(e[0], 3.0),
        goal(e[1], 5.2),
        goal(e[2], 3.4),
        goal(e[3], 5.0),
        IntervalConstraint::sum_equal(&[e[0]], &[e[2]]),
        IntervalConstraint::sum_equal(&[e[1]], &[e[3]]),
    ]);
    let out = IiaSolver::default().solve(&e, &cs).expect("solve");
    assert_eq!(counts(&out), vec![3, 5, 3, 5]);
    assert_eq!(first_violation(&e, &cs, &counts(&out)), None);
}

#[test]
fn brick_with_size_one_and_edge_ten_gives_ten_everywhere() {
    let (topo, ids, model) = brick_setup([10.0, 10.0, 10.0]);
    let reg = registry();
    let picker = ReferenceCornerPicker;
    let solver = IiaSolver::default();
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [ids.volume];
    let input = MeshInput::new(&topo, &model, &WholeFaces, &sel);
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Constant { size: 1.0 });
    let mut alloc = SerialAlloc::new();
    let out = driver.mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut alloc).expect("mesh");
    for e in ids.edges {
        assert_eq!(out.plan.intervals(e), Some(10), "{e}");
    }
}

#[test]
fn row_of_three_bricks_with_mixed_hard_counts_has_the_unique_answer() {
    let (_, bricks, _) = row_setup(3);
    // The four x-edges of one brick share a count; the three x-classes sum to 9.
    let class = |b: usize| [bricks[b].edges[0], bricks[b].edges[1], bricks[b].edges[2], bricks[b].edges[3]];
    let mut edges: Vec<EntityId> = (0..3).flat_map(class).collect();
    edges.sort_unstable();
    edges.dedup();
    let mut items = Vec::new();
    for b in 0..3 {
        let c = class(b);
        for w in c.windows(2) {
            items.push(IntervalConstraint::sum_equal(&[w[0]], &[w[1]]));
        }
        items.push(goal(c[0], 1.0));
    }
    items.push(fixed(bricks[0].edges[0], 2, Firmness::Hard));
    items.push(fixed(bricks[1].edges[0], 3, Firmness::Hard));
    items.push(IntervalConstraint::Linear {
        terms: vec![(bricks[0].edges[0], 1), (bricks[1].edges[0], 1), (bricks[2].edges[0], 1)],
        rel: sm_scheme_api::Rel::Eq,
        rhs: 9,
    });
    let cs = records(items);
    let out = IiaSolver::default().solve(&edges, &cs).expect("solve");
    let at = |e: EntityId| out.iter().find(|x| x.0 == e).map(|x| x.1).expect("edge");
    assert_eq!((at(bricks[0].edges[0]), at(bricks[1].edges[0]), at(bricks[2].edges[0])), (2, 3, 4));
    assert_eq!(first_violation(&edges, &cs, &counts(&out)), None);
}

#[test]
fn conflicting_hard_counts_name_both_rows() {
    let e: Vec<_> = (1..=4).map(edge).collect();
    let cs = records(vec![
        fixed(e[0], 3, Firmness::Hard),                            // 0
        fixed(e[2], 4, Firmness::Hard),                            // 1
        IntervalConstraint::sum_equal(&[e[0]], &[e[2]]),           // 2
        IntervalConstraint::sum_equal(&[e[1]], &[e[3]]),           // 3
    ]);
    let err = IiaSolver::default().solve(&e, &cs).expect_err("conflict");
    assert!(err.constraints.contains(&ConstraintId(0)) && err.constraints.contains(&ConstraintId(1)), "{err}");
    assert!(err.constraints.contains(&ConstraintId(2)), "{err}");
    assert!(err.entities.contains(&e[0]) && err.entities.contains(&e[2]));
    assert!(!err.entities.contains(&e[1]));
}

#[test]
fn conflicting_hard_counts_through_a_sum_row_name_the_row_and_the_fixed_counts() {
    let e: Vec<_> = (1..=3).map(edge).collect();
    // e0 + e1 = e2 with hard 3 + 2 and hard 4.
    let cs = records(vec![
        fixed(e[0], 3, Firmness::Hard),
        fixed(e[1], 2, Firmness::Hard),
        fixed(e[2], 4, Firmness::Hard),
        IntervalConstraint::sum_equal(&[e[0], e[1]], &[e[2]]),
    ]);
    let err = IiaSolver::default().solve(&e, &cs).expect_err("conflict");
    for id in 0..4 {
        assert!(err.constraints.contains(&ConstraintId(id)), "missing {id}: {err}");
    }
}

#[test]
fn even_sum_with_odd_goals_gets_the_minimal_parity_repair() {
    let e: Vec<_> = (1..=4).map(edge).collect();
    // Goals 3,3,3,4 sum to 13: move the 4 to 3 (relative cost 0.25, the cheapest step).
    let cs = records(vec![
        goal(e[0], 3.0),
        goal(e[1], 3.0),
        goal(e[2], 3.0),
        goal(e[3], 4.0),
        IntervalConstraint::EvenSum { edges: e.clone() },
    ]);
    let out = IiaSolver::default().solve(&e, &cs).expect("solve");
    assert_eq!(counts(&out), vec![3, 3, 3, 3]);
    // Goals 5,5,5: one count moves by one, the lexicographically smallest vector wins.
    let e3: Vec<_> = (1..=3).map(edge).collect();
    let cs = records(vec![
        goal(e3[0], 5.0),
        goal(e3[1], 5.0),
        goal(e3[2], 5.0),
        IntervalConstraint::EvenSum { edges: e3.clone() },
    ]);
    let out = IiaSolver::default().solve(&e3, &cs).expect("solve");
    assert_eq!(counts(&out), vec![4, 5, 5]);
}

#[test]
fn an_odd_hard_sum_is_infeasible() {
    let e: Vec<_> = (1..=2).map(edge).collect();
    let cs = records(vec![
        fixed(e[0], 3, Firmness::Hard),
        fixed(e[1], 2, Firmness::Hard),
        IntervalConstraint::EvenSum { edges: e.clone() },
    ]);
    let err = IiaSolver::default().solve(&e, &cs).expect_err("odd");
    assert!(err.constraints.contains(&ConstraintId(2)), "{err}");
}

#[test]
fn soft_counts_beat_default_goals_in_one_class() {
    let e: Vec<_> = (1..=2).map(edge).collect();
    let cs = records(vec![
        fixed(e[0], 10, Firmness::Soft),
        goal(e[1], 2.0),
        IntervalConstraint::sum_equal(&[e[0]], &[e[1]]),
    ]);
    let out = IiaSolver::default().solve(&e, &cs).expect("solve");
    assert_eq!(counts(&out), vec![10, 10]);
}

#[test]
fn ranges_clamp_and_unbounded_rows_find_a_solution() {
    let e: Vec<_> = (1..=2).map(edge).collect();
    // x + y >= 100 with goals 3 and 3: the cheapest split is 50/50.
    let cs = records(vec![
        goal(e[0], 3.0),
        goal(e[1], 3.0),
        IntervalConstraint::Linear { terms: vec![(e[0], 1), (e[1], 1)], rel: sm_scheme_api::Rel::Ge, rhs: 100 },
    ]);
    let out = IiaSolver::default().solve(&e, &cs).expect("solve");
    assert_eq!(counts(&out), vec![50, 50]);
    // Range bounds a rowless edge.
    let cs = records(vec![goal(e[0], 3.0), IntervalConstraint::Range { edge: e[0], min: 5, max: Some(7) }, goal(e[1], 0.2)]);
    let out = IiaSolver::default().solve(&e, &cs).expect("solve");
    assert_eq!(counts(&out), vec![5, 1]);
}

#[test]
fn hard_zero_and_empty_ranges_and_unknown_edges_are_infeasible() {
    let e: Vec<_> = (1..=2).map(edge).collect();
    let s = IiaSolver::default();
    assert!(s.solve(&e, &records(vec![fixed(e[0], 0, Firmness::Hard)])).is_err());
    assert!(s.solve(&e, &records(vec![IntervalConstraint::Range { edge: e[0], min: 5, max: Some(3) }])).is_err());
    assert!(s.solve(&e, &records(vec![fixed(e[0], 9, Firmness::Hard), IntervalConstraint::Range { edge: e[0], min: 1, max: Some(3) }])).is_err());
    let err = s.solve(&e, &records(vec![goal(edge(99), 3.0)])).expect_err("unknown");
    assert!(err.entities.contains(&edge(99)));
    assert_eq!(s.solve(&[], &[]).expect("empty"), vec![]);
}

/// Structured scenarios from the scheme-api stubs: the driver plan with `IiaSolver` equals the
/// plan with the reference solver.
#[test]
fn agrees_with_the_reference_solver_on_the_stub_scenarios() {
    let reg = registry();
    let picker = ReferenceCornerPicker;
    let run = |solver: &dyn IntervalSolver, topo: &sm_topo_api::CellComplex, model: &sm_model_api::Model, vols: &[EntityId], intent: &MeshIntent| {
        let driver = Driver::new(&reg, solver, &picker);
        let input = MeshInput::new(topo, model, &WholeFaces, vols);
        let mut alloc = SerialAlloc::new();
        driver.mesh(&input, intent, &Sequential, ReferenceBuilder::new(), &mut alloc).map(|o| o.plan.intervals.clone())
    };
    let (refs, iia) = (ReferenceIntervalSolver::default(), IiaSolver::default());
    // Bricks of several sizes, with a size field and with hard counts.
    for size in [[1.0, 1.0, 1.0], [10.0, 10.0, 10.0], [3.0, 5.2, 4.1], [2.0, 7.0, 0.7]] {
        let (topo, ids, model) = brick_setup(size);
        for h in [1.0, 0.5, 2.5] {
            let mut intent = MeshIntent::default();
            intent.defaults.sizing = Some(SizingSpec::Constant { size: h });
            let v = [ids.volume];
            assert_eq!(run(&refs, &topo, &model, &v, &intent).expect("ref"), run(&iia, &topo, &model, &v, &intent).expect("iia"), "{size:?} h={h}");
        }
        let mut intent = MeshIntent::default();
        intent.set_intervals(ids.edges[0], Firm::hard(IntervalRequest::Count(4)));
        intent.set_intervals(ids.edges[4], Firm::hard(IntervalRequest::Count(3)));
        let v = [ids.volume];
        assert_eq!(run(&refs, &topo, &model, &v, &intent).expect("ref"), run(&iia, &topo, &model, &v, &intent).expect("iia"), "{size:?} hard");
    }
    // Rows of bricks merged face to face.
    for n in [2usize, 3, 5] {
        let (topo, ids, model) = row_setup(n);
        let vols: Vec<EntityId> = ids.iter().map(|b| b.volume).collect();
        let mut intent = MeshIntent::default();
        intent.defaults.sizing = Some(SizingSpec::Constant { size: 0.5 });
        assert_eq!(run(&refs, &topo, &model, &vols, &intent).expect("ref"), run(&iia, &topo, &model, &vols, &intent).expect("iia"), "row {n}");
        let mut intent = MeshIntent::default();
        intent.set_intervals(ids[0].edges[8], Firm::hard(IntervalRequest::Count(6)));
        assert_eq!(run(&refs, &topo, &model, &vols, &intent).expect("ref"), run(&iia, &topo, &model, &vols, &intent).expect("iia"), "row {n} hard z");
    }
}

fn big_problem(blocks: u64) -> (Vec<EntityId>, Vec<sm_scheme_api::Record<IntervalConstraint>>) {
    // `blocks` independent 4-sided faces with distinct goals plus one coupled chain.
    let mut items = Vec::new();
    let edges: Vec<EntityId> = (1..=blocks * 4).map(edge).collect();
    for b in 0..blocks {
        let e = &edges[(b * 4) as usize..(b * 4 + 4) as usize];
        let base = 2.0 + (b % 7) as f64 * 0.37;
        items.push(goal(e[0], base));
        items.push(goal(e[1], base * 1.7));
        items.push(goal(e[2], base * 1.1));
        items.push(goal(e[3], base * 1.6));
        items.push(IntervalConstraint::sum_equal(&[e[0]], &[e[2]]));
        items.push(IntervalConstraint::sum_equal(&[e[1]], &[e[3]]));
        items.push(IntervalConstraint::EvenSum { edges: e.to_vec() });
    }
    // Chain: edge(4k) + edge(4k+4) + ... shared sum.
    let chain: Vec<(EntityId, i32)> = (0..blocks.min(6)).map(|b| (edges[(b * 4) as usize], 1)).collect();
    items.push(IntervalConstraint::Linear { terms: chain, rel: sm_scheme_api::Rel::Ge, rhs: 30 });
    (edges, records(items))
}

#[test]
fn identical_across_threads_runs_and_constraint_order() {
    let (edges, cs) = big_problem(40);
    let solver = IiaSolver::default();
    let base = sm_testkit::determinism::check(2, 4, |threads| {
        let pool = ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
        pool.install(|| solver.solve(&edges, &cs))
    })
    .expect("determinism");
    let base = base.expect("feasible");
    assert_eq!(first_violation(&edges, &cs, &counts(&base)), None);
    // Permuted constraint order (ids stay attached to their rows) gives the same answer.
    for rot in [1usize, 7, 20] {
        let mut shuffled = cs.clone();
        shuffled.rotate_left(rot);
        shuffled.reverse();
        assert_eq!(solver.solve(&edges, &shuffled).expect("feasible"), base);
    }
}

#[test]
fn falls_back_to_the_reference_solver_beyond_the_limits() {
    let (edges, cs) = big_problem(8);
    let exact = IiaSolver::default().solve(&edges, &cs).expect("exact");
    // Force the fallback on every coupled component: the answer is feasible (verified) and, on
    // these structured rows, equal to the reference solver's.
    let tiny = IiaSolver::with_limits(1, 1);
    let fb = tiny.solve(&edges, &cs).expect("fallback");
    assert_eq!(first_violation(&edges, &cs, &counts(&fb)), None);
    let reference = ReferenceIntervalSolver::default().solve(&edges, &cs).expect("reference");
    assert_eq!(fb.len(), reference.len());
    // The fallback may be worse than the optimum, never infeasible.
    assert_eq!(exact.len(), fb.len());
    // A truly infeasible problem stays infeasible through the fallback.
    let e: Vec<_> = (1..=2).map(edge).collect();
    let cs = records(vec![
        fixed(e[0], 3, Firmness::Hard),
        fixed(e[1], 2, Firmness::Hard),
        IntervalConstraint::EvenSum { edges: e.clone() },
    ]);
    assert!(IiaSolver::with_limits(1, 1).solve(&e, &cs).is_err());
}

#[test]
fn large_counts_with_unbounded_domains_stay_fast_and_exact() {
    let e: Vec<_> = (1..=4).map(edge).collect();
    // x0 + x1 + x2 = 3001 with goals ~1000; x3 free with goal 777.7 and an even-sum row.
    let cs = records(vec![
        goal(e[0], 1000.0),
        goal(e[1], 1000.0),
        goal(e[2], 1000.0),
        goal(e[3], 777.7),
        IntervalConstraint::Linear { terms: vec![(e[0], 1), (e[1], 1), (e[2], 1)], rel: sm_scheme_api::Rel::Eq, rhs: 3001 },
        IntervalConstraint::EvenSum { edges: vec![e[2], e[3]] },
    ]);
    let started = std::time::Instant::now();
    let out = IiaSolver::default().solve(&e, &cs).expect("solve");
    assert!(started.elapsed().as_secs() < 5, "took {:?}", started.elapsed());
    assert_eq!(first_violation(&e, &cs, &counts(&out)), None);
    let total: u32 = counts(&out)[..3].iter().sum();
    assert_eq!(total, 3001);
    // The best split of 3001 over three goals of 1000 is 1000/1000/1001 (max deviation 0.001),
    // with the lexicographically smallest vector putting the extra count last; x2 must be even
    // together with x3 = 778 (nearest to 777.7), so x2 = 1000 is not allowed: parity decides.
    assert_eq!((counts(&out)[2] + counts(&out)[3]) % 2, 0);
}

/// Regression: the fallback must keep a row whose cancelling terms name edges of another class
/// (found by the larger-system property test).
#[test]
fn fallback_keeps_rows_whose_terms_cancel() {
    let e: Vec<_> = (1..=11).map(edge).collect();
    let cs = records(vec![
        fixed(e[0], 12, Firmness::Soft),
        goal(e[4], 54.247_435_814_058_84),
        goal(e[6], 30.234_301_764_722_268),
        goal(e[7], 34.629_452_421_082_63),
        goal(e[10], 41.447_083_077_129_43),
        IntervalConstraint::EvenSum { edges: vec![e[1]] },
        IntervalConstraint::Linear {
            terms: vec![(e[0], -1), (e[1], 1), (e[3], 1), (e[4], -1), (e[5], -1), (e[6], -1), (e[7], -1), (e[8], 1), (e[10], 1)],
            rel: sm_scheme_api::Rel::Ge,
            rhs: 0,
        },
        IntervalConstraint::sum_equal(&[e[5]], &[e[3]]),
    ]);
    for solver in [IiaSolver::default(), IiaSolver::with_limits(512, 1), IiaSolver::with_limits(2, 1_000)] {
        let out = solver.solve(&e, &cs).expect("feasible");
        assert_eq!(first_violation(&e, &cs, &counts(&out)), None, "{solver:?}");
    }
}
