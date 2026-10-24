//! The scaling workload at test size: identical to the reference at every checkpoint, valid, and
//! a quick timing report (`cargo test --release -p sm-topo --test scale -- --nocapture`).

mod common;
#[path = "common/scenario.rs"]
mod scenario;

use common::{Twin, same_state};
use sm_topo::TopoStore;
use sm_topo_api::{CellComplex, Dim, TopologyView, validate};
use std::time::Instant;

#[test]
fn refinement_matches_the_reference_in_one_op_and_per_op() {
    for per_op in [false, true] {
        let mut tw = Twin::new();
        let (ba, bb) = (scenario::setup(&mut tw.a), scenario::setup(&mut tw.b));
        assert_eq!(ba, bb);
        let (pa, pb) = (scenario::refine(&mut tw.a, &ba, 300, per_op), scenario::refine(&mut tw.b, &bb, 300, per_op));
        assert_eq!(pa, pb);
        tw.same(&format!("refinement per_op={per_op}"));
        assert_eq!(tw.b.cells(Dim::Face).len(), 306);
        assert!(validate(&tw.b).is_empty());
        assert_eq!(tw.b.validate_par(), validate(&tw.b));
    }
}

/// The shipped configuration: default row thresholds (positional rewrites from 24 entries,
/// incremental row checks from 128), paranoid cross-checks on, volume rows growing past both.
#[test]
fn refinement_at_production_thresholds_matches_the_reference() {
    for per_op in [false, true] {
        let mut a = CellComplex::new();
        let mut b = TopoStore::new();
        b.set_paranoid(true);
        let (ba, bb) = (scenario::setup(&mut a), scenario::setup(&mut b));
        let (pa, pb) = (scenario::refine(&mut a, &ba, 700, per_op), scenario::refine(&mut b, &bb, 700, per_op));
        assert_eq!(pa, pb);
        same_state(&a, &b, &format!("production thresholds, per_op={per_op}"));
        assert!(b.counters().sorted_parent_rewrites > 500, "{:?}", b.counters());
        assert!(b.volume_faces(bb.volume).len() > 700);
    }
}

#[test]
#[ignore = "timing: run with --release --ignored --nocapture"]
fn timing_report() {
    for &(n, per_op) in &[(1000usize, false), (1000, true), (4000, false), (4000, true)] {
        let mut a = CellComplex::new();
        let ba = scenario::setup(&mut a);
        let t0 = Instant::now();
        scenario::refine(&mut a, &ba, n, per_op);
        let ta = t0.elapsed();
        let mut b = TopoStore::new();
        let bb = scenario::setup(&mut b);
        let t0 = Instant::now();
        scenario::refine(&mut b, &bb, n, per_op);
        let tb = t0.elapsed();
        println!("n={n:6} per_op={per_op:5}: reference {ta:>10.2?}  sm-topo {tb:>10.2?}  ({:.1}x)", ta.as_secs_f64() / tb.as_secs_f64());
        same_state(&a, &b, "timing run");
    }
}

/// Large sizes for the store alone (`cargo test --release -p sm-topo --test scale -- --ignored --nocapture`).
#[test]
#[ignore = "slow: run with --release --ignored"]
fn large_sizes() {
    for &(n, per_op) in &[(16_000usize, false), (16_000, true), (100_000, false), (100_000, true)] {
        let mut b = TopoStore::new();
        let bb = scenario::setup(&mut b);
        let t0 = Instant::now();
        scenario::refine(&mut b, &bb, n, per_op);
        let tb = t0.elapsed();
        let t1 = Instant::now();
        let h = b.canonical_hash();
        let th = t1.elapsed();
        let t2 = Instant::now();
        let issues = b.validate_par().len();
        let tv = t2.elapsed();
        println!("n={n:7} per_op={per_op:5}: build {tb:>9.2?} ({:.2} us/split)  hash {th:>8.2?}  validate_par {tv:>9.2?} issues={issues} faces={} hash={h:#x}", tb.as_secs_f64() * 1e6 / n as f64, b.cells(Dim::Face).len());
    }
}
