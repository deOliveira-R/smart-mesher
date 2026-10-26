//! Acceptance 14: bit-identical state hashes across runs, thread counts, concurrent runs and
//! hosts; cold replay from the root never diverges.

mod common;

use common::rig::*;
use sm_boolean::KeepSide;
use sm_boolean::ops::{AxisPlane, Imprint, Intersect, Merge, PlaneAxis, PlaneSpec, Subtract, Unite, WebcutCylinder, WebcutPlane};
use sm_model::Rotate;
use sm_op_api::{Hash256, QuietControl};

/// Operations of acceptance 4, 6, 7, 12 and 13, plus the marched and the pole cases.
fn scenario(r: &mut Rig) -> Hash256 {
    // 4: webcut with a plane, with and without merge, and a section.
    let c1 = r.brick([1.0; 3], [0.5; 3]);
    r.run(&WebcutPlane { targets: vec![c1.body], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::Z, offset: 0.25 }), keep: None, merge: None, imprint: None }).unwrap();
    let c2 = r.brick([1.0; 3], [3.5, 0.5, 0.5]);
    r.run(&WebcutPlane { targets: vec![c2.body], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: 3.5 }), keep: Some(KeepSide::Both), merge: Some(false), imprint: None }).unwrap();
    // 6: subtract a cylinder; an interior sphere.
    let b = r.brick([4.0; 3], [12.0, 2.0, 2.0]);
    let cy = r.cylinder(6.0, 1.0, [12.0, 2.0, 2.0]);
    r.run(&Subtract { targets: vec![b.body], tools: vec![cy.body], keep: None, regularize: None, tol: None }).unwrap();
    let big = r.brick([10.0; 3], [30.0, 5.0, 5.0]);
    let sp = r.sphere(2.0, [30.0, 5.0, 5.0]);
    r.run(&Subtract { targets: vec![big.body], tools: vec![sp.body], keep: None, regularize: None, tol: None }).unwrap();
    // 7: unite two bricks, then a brick with a sphere at a pole-on-edge position; intersect.
    let a1 = r.brick([2.0; 3], [50.0, 1.0, 1.0]);
    let a2 = r.brick([2.0; 3], [51.0, 1.0, 1.0]);
    r.run(&Unite { bodies: vec![a1.body, a2.body], keep: None, regularize: None, tol: None }).unwrap();
    let bq = r.brick([2.0; 3], [60.0, 1.0, 1.0]);
    let sq = r.sphere(1.0, [61.0, 1.0, 1.0]);
    r.run(&Unite { bodies: vec![bq.body, sq.body], keep: None, regularize: None, tol: None }).unwrap();
    let i1 = r.brick([2.0; 3], [70.0, 1.0, 1.0]);
    let i2 = r.brick([2.0; 3], [71.0, 1.0, 1.0]);
    r.run(&Intersect { bodies: vec![i1.body, i2.body], keep: Some(true), regularize: None, tol: None }).unwrap();
    // Marched.
    let ca = r.cylinder(4.0, 1.0, [80.0, 0.0, 0.0]);
    let cb = r.cylinder(4.0, 0.5, [80.0, 0.0, 0.0]);
    r.run(&Rotate { entities: vec![cb.body], axis: [0.0, 1.0, 0.0], angle_deg: 90.0, origin: Some([80.0, 0.0, 0.0]) }).unwrap();
    r.run(&Unite { bodies: vec![ca.body, cb.body], keep: None, regularize: None, tol: None }).unwrap();
    // 12: webcut a long brick and a cylinder-tool cut.
    let long = r.brick([4.0, 2.0, 2.0], [92.0, 1.0, 1.0]);
    r.run(&WebcutPlane { targets: vec![long.body], plane: PlaneSpec::Axis(AxisPlane { axis: PlaneAxis::X, offset: 92.0 }), keep: None, merge: None, imprint: None }).unwrap();
    let cut4 = r.brick([4.0; 3], [100.0, 2.0, 2.0]);
    r.run(&WebcutCylinder { targets: vec![cut4.body], radius: 1.0, axis: [0.0, 0.0, 1.0], center: [100.0, 2.0, 2.0], keep: None, merge: None, imprint: None }).unwrap();
    // 13: three bricks in a row, merged, then the first two united.
    let x = r.brick([1.0; 3], [110.5, 0.5, 0.5]);
    let y = r.brick([1.0; 3], [111.5, 0.5, 0.5]);
    let _z = r.brick([1.0; 3], [112.5, 0.5, 0.5]);
    r.run(&Merge { entities: Some(vec![x.body, y.body, _z.body]), kinds: None, tol: None, check_only: None }).unwrap();
    r.run(&Unite { bodies: vec![x.body, y.body], keep: None, regularize: None, tol: None }).unwrap();
    // Imprint and merge of the partially overlapping bricks.
    let p = r.brick([2.0; 3], [120.0, 1.0, 1.0]);
    let q = r.brick([2.0; 3], [122.0, 2.0, 1.0]);
    r.run(&Imprint { bodies: vec![p.body, q.body], merge: Some(true), tol: None }).unwrap();
    r.hash()
}

fn run_in_pool(threads: usize, factory: Factory) -> (Hash256, Hash256) {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
    pool.install(|| {
        let mut r = Rig::new(factory);
        let h = scenario(&mut r);
        (h, r.s.state().layer_hash("model").unwrap())
    })
}

#[test]
fn scenario_is_deterministic_across_runs_threads_and_hosts() {
    let baseline = sm_testkit::determinism::check(2, 4, |threads| run_in_pool(threads, cell_complex)).expect("deterministic");
    let prod = sm_testkit::determinism::check(2, 4, |threads| run_in_pool(threads, topo_store)).expect("deterministic");
    assert_eq!(baseline, prod, "the reference and the production store give one state hash");
}

#[test]
fn cold_replay_from_the_root_does_not_diverge() {
    for factory in [cell_complex as Factory, topo_store] {
        let mut r = Rig::uncached(factory);
        let full = scenario(&mut r);
        let q = QuietControl::new();
        let head = r.s.position().expect("head node");
        let cold = r.s.state_at_from_root(head, &q.control()).expect("cold replay: no Diverged");
        assert_eq!(cold.hash(), full);
        assert_eq!(cold.layer_hashes(), r.s.state().layer_hashes());
    }
}
