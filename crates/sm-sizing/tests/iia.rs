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
    assert_eq!(counts(&fb), counts(&reference));
    // The fallback may be worse than the optimum, never infeasible; the exact solve is feasible.
    assert_eq!(first_violation(&edges, &cs, &counts(&exact)), None);
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
    // The best split of 3001 over three goals of 1000 is 1000/1000/1001 or a permutation (max
    // deviation 0.001); x3 = 778 (nearest to 777.7) needs x2 even, so the extra count goes to x1
    // and the smallest vector is [1000, 1001, 1000, 778].
    assert_eq!(counts(&out), vec![1000, 1001, 1000, 778]);
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

// ---------------------------------------------------------------------------------------------
// dense rows, exhausted searches, error reports

fn dense_goals(n: u64) -> (Vec<EntityId>, Vec<f64>, Vec<IntervalConstraint>) {
    let e: Vec<_> = (1..=n).map(edge).collect();
    let g: Vec<f64> = (1..=n).map(|i| 3.0 + ((i * 17) % 7) as f64 * 1.3).collect();
    let items = e.iter().zip(&g).map(|(&ed, &gv)| goal(ed, gv)).collect();
    (e, g, items)
}

fn max_dev(out: &[(EntityId, u32)], g: &[f64]) -> f64 {
    out.iter().zip(g).map(|(x, &gv)| (f64::from(x.1) - gv).abs() / gv).fold(0.0, f64::max)
}

fn sum_dev(out: &[(EntityId, u32)], g: &[f64]) -> f64 {
    out.iter().zip(g).map(|(x, &gv)| (f64::from(x.1) - gv).abs() / gv).sum()
}

fn dense30_rows(e: &[EntityId]) -> Vec<IntervalConstraint> {
    let all: Vec<_> = e.iter().map(|&x| (x, 1)).collect();
    let odd: Vec<_> = e.iter().step_by(2).map(|&x| (x, 1)).collect();
    vec![
        IntervalConstraint::Linear { terms: all, rel: sm_scheme_api::Rel::Le, rhs: 100 },
        IntervalConstraint::Linear { terms: odd, rel: sm_scheme_api::Rel::Ge, rhs: 60 },
    ]
}

#[test]
fn dense30_two_dense_rows_is_feasible_and_not_reported_infeasible() {
    let (e, g, mut items) = dense_goals(30);
    items.extend(dense30_rows(&e));
    let cs = records(items);
    let out = IiaSolver::default().solve(&e, &cs).expect("dense30 is feasible");
    assert_eq!(first_violation(&e, &cs, &counts(&out)), None);
    // Odd edges = 4 and even edges = 1 is feasible with max deviation < 1; the optimum is no worse.
    // The search exhausts its budget here; the local polish makes the incumbent 1- and 2-move
    // optimal (it was max 13/19, sum 18.05 before; a unit transfer 22 -> 10 alone gains 0.2).
    let (md, sd) = (max_dev(&out, &g), sum_dev(&out, &g));
    assert!(md <= 13.0 / 19.0 + 1e-12, "max deviation {md}");
    assert!(sd <= 14.2759, "sum of deviations {sd}");
}

/// The polished incumbent admits no improving single move or row-sharing pair move.
#[test]
fn dense30_incumbent_is_locally_optimal() {
    let (e, g, mut items) = dense_goals(30);
    items.extend(dense30_rows(&e));
    let cs = records(items);
    let out = IiaSolver::default().solve(&e, &cs).expect("dense30 is feasible");
    let base = counts(&out);
    let cost = |v: &[u32]| (max_dev_v(v, &g), sum_dev_v(v, &g));
    let c0 = cost(&base);
    for i in 0..30 {
        for j in 0..30 {
            for (di, dj) in [(1i64, -1i64), (-1, 1), (1, 1), (-1, -1), (1, 0), (-1, 0)] {
                if i == j && dj != 0 {
                    continue;
                }
                let mut v: Vec<i64> = base.iter().map(|&x| i64::from(x)).collect();
                v[i] += di;
                v[j] += dj;
                if v.iter().any(|&x| x < 1) {
                    continue;
                }
                let v: Vec<u32> = v.iter().map(|&x| x as u32).collect();
                if first_violation(&e, &cs, &v).is_some() {
                    continue;
                }
                let c = cost(&v);
                assert!(
                    c.0 > c0.0 + 1e-9 || (c.0 >= c0.0 - 1e-9 && c.1 >= c0.1 - 1e-9),
                    "move ({i},{di}) ({j},{dj}) improves {c0:?} to {c:?}"
                );
            }
        }
    }
}

fn max_dev_v(v: &[u32], g: &[f64]) -> f64 {
    v.iter().zip(g).map(|(&x, &gv)| (f64::from(x) - gv).abs() / gv).fold(0.0, f64::max)
}

fn sum_dev_v(v: &[u32], g: &[f64]) -> f64 {
    v.iter().zip(g).map(|(&x, &gv)| (f64::from(x) - gv).abs() / gv).sum()
}

/// A 40-class variant (sum <= 140, odd >= 90): the polished answer reaches the better maximum.
#[test]
fn dense40_variant_reaches_the_better_maximum() {
    let (e, g, mut items) = dense_goals(40);
    let all: Vec<_> = e.iter().map(|&x| (x, 1)).collect();
    let odd: Vec<_> = e.iter().step_by(2).map(|&x| (x, 1)).collect();
    items.push(IntervalConstraint::Linear { terms: all, rel: sm_scheme_api::Rel::Le, rhs: 140 });
    items.push(IntervalConstraint::Linear { terms: odd, rel: sm_scheme_api::Rel::Ge, rhs: 90 });
    let cs = records(items);
    let out = IiaSolver::default().solve(&e, &cs).expect("feasible");
    assert_eq!(first_violation(&e, &cs, &counts(&out)), None);
    assert!(max_dev(&out, &g) <= 0.8148 + 1e-4, "max deviation {}", max_dev(&out, &g));
}

/// The exhausted path is deterministic across thread counts (two independent components, so the
/// components are solved in parallel).
#[test]
fn exhausted_path_is_identical_on_one_and_four_threads() {
    let (e1, _, mut items) = dense_goals(30);
    items.extend(dense30_rows(&e1));
    // A second copy on edges 101..=130.
    let e2: Vec<_> = (101..=130).map(edge).collect();
    for (i, &ed) in e2.iter().enumerate() {
        items.push(goal(ed, 3.0 + (((i as u64 + 1) * 17) % 7) as f64 * 1.3));
    }
    items.extend(dense30_rows(&e2));
    let e: Vec<_> = e1.iter().chain(&e2).copied().collect();
    let cs = records(items);
    let run = |threads: usize| {
        ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .expect("pool")
            .install(|| IiaSolver::default().solve(&e, &cs).expect("feasible"))
    };
    let (a, b) = (run(1), run(4));
    assert_eq!(a, b);
    assert_eq!(a, run(1));
    assert_eq!(counts(&a)[..30], counts(&a)[30..], "the two copies solve identically");
}

#[test]
fn row_pair_conflicts_are_proven_at_once_and_name_both_rows() {
    let e: Vec<_> = (1..=30).map(edge).collect();
    let all: Vec<_> = e.iter().map(|&x| (x, 1)).collect();
    let odd: Vec<_> = e.iter().step_by(2).map(|&x| (x, 1)).collect();
    for (le, ge) in [(50, 40), (59, 60)] {
        let mut items = vec![
            IntervalConstraint::Linear { terms: all.clone(), rel: sm_scheme_api::Rel::Le, rhs: le },
            IntervalConstraint::Linear { terms: odd.clone(), rel: sm_scheme_api::Rel::Ge, rhs: ge },
        ];
        items.extend(e.iter().map(|&x| goal(x, 3.0)));
        let cs = records(items);
        let t = std::time::Instant::now();
        let err = IiaSolver::default().solve(&e, &cs).expect_err("infeasible");
        assert!(t.elapsed().as_secs_f64() < 1.0, "took {:?}", t.elapsed());
        assert!(!err.reason.contains("unproven"), "{}", err.reason);
        assert!(err.constraints.contains(&ConstraintId(0)) && err.constraints.contains(&ConstraintId(1)), "{err:?}");
    }
}

/// Real ties are exact: the sum of deviations of goals 10, 20, 20 is flat on [10, 20], and the
/// plan's smallest vector wins even though the quantized keys differ by rounding noise.
#[test]
fn real_ties_in_the_sum_go_to_the_smallest_vector() {
    let e: Vec<_> = (1..=4).map(edge).collect();
    let cs = records(vec![
        goal(e[0], 1.0),
        IntervalConstraint::Range { edge: e[0], min: 2, max: None },
        goal(e[1], 10.0),
        goal(e[2], 20.0),
        goal(e[3], 20.0),
        IntervalConstraint::sum_equal(&[e[1]], &[e[2]]),
        IntervalConstraint::sum_equal(&[e[2]], &[e[3]]),
    ]);
    let out = IiaSolver::default().solve(&e, &cs).expect("solve");
    assert_eq!(counts(&out), vec![2, 10, 10, 10]);
}

#[test]
fn single_dense_le_row_over_30_classes_pins_the_optimum() {
    let (e, g, mut items) = dense_goals(30);
    items.push(IntervalConstraint::Linear {
        terms: e.iter().map(|&x| (x, 1)).collect(),
        rel: sm_scheme_api::Rel::Le,
        rhs: 100,
    });
    let cs = records(items);
    let out = IiaSolver::default().solve(&e, &cs).expect("solve");
    assert_eq!(first_violation(&e, &cs, &counts(&out)), None);
    // The goals repeat with period seven (3, 4.3, ..., 10.8); the optimum scales them by about
    // 0.47 to counts 3, 5, 3, 4, 2, 4, 2 (sum 100), with maximum deviation 11/19 (count 4 vs 9.5).
    let want: Vec<u32> = [3, 5, 3, 4, 2, 4, 2].iter().copied().cycle().take(30).collect();
    assert_eq!(counts(&out), want);
    assert_eq!(want.iter().sum::<u32>(), 100);
    // Lower bound: for any smaller deviation d, even the real-valued minimal counts
    // ceil(g (1 - d)) already exceed 100, so no assignment does better.
    let md = max_dev(&out, &g);
    let d = md - 1e-6;
    let min_sum: f64 = g.iter().map(|&gv| (gv * (1.0 - d)).ceil().max(1.0)).sum();
    assert!(min_sum > 100.0, "a smaller deviation {d} would fit: {min_sum}");
}

#[test]
fn exhausted_search_without_proof_says_so_and_does_not_echo_the_reference() {
    let (e, _, mut items) = dense_goals(30);
    items.extend(dense30_rows(&e));
    let cs = records(items);
    // Two nodes cannot even finish the feasibility pass.
    let err = IiaSolver::with_limits(512, 2).solve(&e, &cs).expect_err("budget");
    assert!(err.reason.starts_with("search budget exhausted, feasibility unproven"), "{}", err.reason);
    assert!(err.constraints.contains(&ConstraintId(30)) && err.constraints.contains(&ConstraintId(31)));
}

#[test]
fn interacting_rows_that_conflict_name_every_row_of_the_component() {
    let e: Vec<_> = (1..=2).map(edge).collect();
    let diff = |rhs| IntervalConstraint::Linear {
        terms: vec![(e[0], 1), (e[1], -1)],
        rel: sm_scheme_api::Rel::Eq,
        rhs,
    };
    let cs = records(vec![diff(1), diff(2)]);
    let err = IiaSolver::default().solve(&e, &cs).expect_err("infeasible");
    assert_eq!(err.constraints, vec![ConstraintId(0), ConstraintId(1)]);
    assert!(!err.reason.contains("did not converge"), "{}", err.reason);
}

#[test]
fn permuting_edge_ids_gives_the_mapped_answer() {
    // Renumbering the edges reorders the components; the answer must map one to one.
    let (edges, cs) = big_problem(40);
    let solver = IiaSolver::default();
    let base = solver.solve(&edges, &cs).expect("feasible");
    let n = edges.len() as u64;
    // A bijection on 1..=n (stride 37 is coprime to n = 160).
    let map = |id: EntityId| edge((id.serial() * 37) % n + 1);
    let mut mapped: Vec<EntityId> = edges.iter().map(|&x| map(x)).collect();
    mapped.sort_unstable();
    let cs2: Vec<_> = cs
        .iter()
        .map(|r| {
            let mut r = r.clone();
            r.item = match r.item {
                IntervalConstraint::Goal { edge, goal } => IntervalConstraint::Goal { edge: map(edge), goal },
                IntervalConstraint::Linear { terms, rel, rhs } => IntervalConstraint::Linear {
                    terms: terms.into_iter().map(|(x, k)| (map(x), k)).collect(),
                    rel,
                    rhs,
                },
                IntervalConstraint::EvenSum { edges } => {
                    IntervalConstraint::EvenSum { edges: edges.into_iter().map(map).collect() }
                }
                other => other,
            };
            r
        })
        .collect();
    let out = solver.solve(&mapped, &cs2).expect("feasible");
    for (id, c) in &base {
        let got = out.iter().find(|x| x.0 == map(*id)).expect("edge").1;
        assert_eq!(got, *c, "edge {id}");
    }
}

/// A single row whose coefficient gcd does not divide its right-hand side is proven infeasible
/// at the root, naming only that row, however large the rest of the component is.
#[test]
fn single_row_gcd_conflict_is_proven_at_once_and_names_the_row() {
    let e: Vec<_> = (1..=30).map(edge).collect();
    let all: Vec<_> = e.iter().map(|&x| (x, 1)).collect();
    let odd2: Vec<_> = e.iter().step_by(2).map(|&x| (x, 2)).collect();
    let mut items = vec![
        IntervalConstraint::Linear { terms: all, rel: sm_scheme_api::Rel::Le, rhs: 100 },
        IntervalConstraint::Linear { terms: odd2, rel: sm_scheme_api::Rel::Eq, rhs: 81 },
    ];
    items.extend(e.iter().map(|&x| goal(x, 3.0)));
    let cs = records(items);
    let t = std::time::Instant::now();
    let err = IiaSolver::default().solve(&e, &cs).expect_err("infeasible");
    assert!(t.elapsed().as_secs_f64() < 1.0, "took {:?}", t.elapsed());
    assert!(!err.reason.contains("unproven"), "{}", err.reason);
    assert_eq!(err.constraints, vec![ConstraintId(1)], "{err:?}");
}
