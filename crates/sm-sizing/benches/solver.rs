#![allow(missing_docs)]
//! Interval solver throughput: independent structured faces (parallel components) and a chain
//! of coupled faces (one branch-and-bound component), `IiaSolver` against the reference.

use criterion::{Criterion, criterion_group, criterion_main};
use sm_core::{EntityId, EntityKind};
use sm_scheme_api::{
    ConstraintId, IntervalConstraint, IntervalSolver, Origin, Record, ReferenceIntervalSolver, Rel,
};
use sm_sizing::IiaSolver;
use std::hint::black_box;

fn edge(n: u64) -> EntityId {
    EntityId::new(EntityKind::TopoEdge, n).expect("edge id")
}

/// `faces` quad faces with opposite-equal rows, even loop sums and distinct goals; the first
/// `chain` faces also share one `>=` row (a single coupled component).
fn problem(faces: u64, chain: u64) -> (Vec<EntityId>, Vec<Record<IntervalConstraint>>) {
    let edges: Vec<EntityId> = (1..=faces * 4).map(edge).collect();
    let mut items = Vec::new();
    for b in 0..faces {
        let e = &edges[(b * 4) as usize..(b * 4 + 4) as usize];
        let base = 2.0 + (b % 11) as f64 * 0.37;
        for (k, m) in [1.0, 1.7, 1.1, 1.6].into_iter().enumerate() {
            items.push(IntervalConstraint::Goal { edge: e[k], goal: base * m });
        }
        items.push(IntervalConstraint::sum_equal(&[e[0]], &[e[2]]));
        items.push(IntervalConstraint::sum_equal(&[e[1]], &[e[3]]));
        items.push(IntervalConstraint::EvenSum { edges: vec![e[0], e[1], e[2]] });
    }
    if chain > 0 {
        let terms = (0..chain.min(faces)).map(|b| (edges[(b * 4) as usize], 1)).collect();
        items.push(IntervalConstraint::Linear { terms, rel: Rel::Ge, rhs: 6 * chain as i64 });
    }
    let recs = items
        .into_iter()
        .enumerate()
        .map(|(i, item)| Record { id: ConstraintId(i as u32), origin: Origin::Driver, item })
        .collect();
    (edges, recs)
}

/// Registered benchmark group body.
fn bench(c: &mut Criterion) {
    let iia = IiaSolver::default();
    let reference = ReferenceIntervalSolver::default();
    for (name, faces, chain) in [("5000 independent faces", 5000, 0), ("12 coupled faces", 12, 12), ("200 faces, 8 coupled", 200, 8)] {
        let (edges, cs) = problem(faces, chain);
        c.bench_function(&format!("iia: {name}"), |b| b.iter(|| iia.solve(black_box(&edges), black_box(&cs)).expect("solve")));
        c.bench_function(&format!("reference: {name}"), |b| b.iter(|| reference.solve(black_box(&edges), black_box(&cs)).expect("solve")));
    }
}

criterion_group!(benches, bench);
criterion_main!(benches);
