//! Booleans with quadrics through the ops (acceptance 6-8).

mod common;

use common::rig::*;
use common::*;
use core::f64::consts::PI;
use sm_boolean::ops::{Intersect, Subtract, Unite};
use sm_topo_api::{TopologyView, TopologyViewExt};

#[test]
fn brick_minus_through_cylinder() {
    let mut r = Rig::cc();
    let b = r.brick([4.0; 3], [2.0; 3]);
    let c = r.cylinder(6.0, 1.0, [2.0, 2.0, 2.0]);
    let d = r.run(&Subtract { targets: vec![b.body], tools: vec![c.body], keep: None, regularize: None, tol: None }).unwrap();
    let t = r.topo();
    assert_valid(t);
    assert_eq!(d.outputs["volumes"].len(), 1);
    let v = d.outputs["volumes"][0];
    assert!((volume_of(t, v) - (64.0 - 4.0 * PI)).abs() < 1e-8 * 64.0, "{}", volume_of(t, v));
    assert!((total_area(t) - (96.0 + 6.0 * PI)).abs() < 1e-8 * 100.0);
    assert_eq!(counts(t), [10, 15, 7, 1, 1]);
    assert_eq!(euler(t), 2);
    let _ = t.volume_faces(v);
}

#[test]
fn dbg_lib_subtract() {
    let mut r = Rig::cc();
    let b = r.brick([4.0; 3], [2.0; 3]);
    let c = r.cylinder(6.0, 1.0, [2.0, 2.0, 2.0]);
    let res = r.lib(|t| sm_boolean::boolean(t, sm_boolean::BoolRule::Subtract, &[b.body], &[c.body], &sm_boolean::BoolOptions::default()));
    match res {
        Ok((h, out)) => println!("ok {out:?} {:?}", counts(h.as_view())),
        Err(e) => panic!("{e}"),
    }
}
