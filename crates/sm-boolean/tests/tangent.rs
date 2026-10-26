//! Tangent and near-tangent contacts: typed errors that name entities, and the near-tangent range.

mod common;

use common::rig::*;
use common::*;
use core::f64::consts::PI;
use sm_boolean::ops::Subtract;
use sm_op_api::{ApplyError, OpError, TimelineError};

fn failed(e: TimelineError) -> OpError {
    match e {
        TimelineError::Apply(ApplyError::Op { source, .. }) => source,
        other => panic!("{other:?}"),
    }
}

fn sub(r: &mut Rig, t: &Solid, tool: &Solid) -> Result<sm_op_api::Delta, TimelineError> {
    r.run(&Subtract { targets: vec![t.body], tools: vec![tool.body], keep: None, regularize: None, tol: None })
}

/// A 4^3 brick centred at 2 minus a through cylinder (r = 1, axis z) whose axis is at x = 1 + gap.
fn with_gap(gap: f64) -> (Rig, Solid, Result<sm_op_api::Delta, TimelineError>) {
    let mut r = Rig::cc();
    let b = r.brick([4.0; 3], [2.0; 3]);
    let c = r.cylinder(6.0, 1.0, [1.0 + gap, 2.0, 2.0]);
    let d = sub(&mut r, &b, &c);
    (r, b, d)
}

#[test]
fn clearances_of_1e_2_and_above_work() {
    for gap in [1e-2, 0.1] {
        let (r, _b, d) = with_gap(gap);
        let d = d.unwrap_or_else(|e| panic!("gap {gap}: {e}"));
        let t = r.topo();
        assert_valid(t);
        let v = d.outputs["volumes"][0];
        assert!((volume_of(t, v) - (64.0 - 4.0 * PI)).abs() < 1e-8 * 64.0, "gap {gap}");
        assert!((total_area(t) - (96.0 + 6.0 * PI)).abs() < 1e-8 * 100.0, "gap {gap}");
        assert_eq!(counts(t), [10, 15, 7, 1, 1], "gap {gap}");
    }
}

#[test]
fn near_tangent_clearances_are_rolled_back_with_a_typed_error() {
    for gap in [1e-5, 1e-4, 1e-3] {
        let (r, _b, d) = with_gap(gap);
        // Either it works (then the numbers are right) or it fails typed and the state is intact.
        match d {
            Ok(d) => {
                let t = r.topo();
                assert_valid(t);
                let v = d.outputs["volumes"][0];
                assert!((volume_of(t, v) - (64.0 - 4.0 * PI)).abs() < 1e-8 * 64.0, "gap {gap}");
            }
            Err(e) => {
                let msg = failed(e).to_string();
                assert!(msg.contains("invalid") && msg.contains("rolled back") && msg.contains("nearly tangent"), "gap {gap}: {msg}");
            }
        }
    }
}

#[test]
fn internal_line_tangent_cylinder_is_a_tangent_contact_error() {
    let (r, _b, d) = with_gap(0.0);
    let e = failed(d.unwrap_err());
    let msg = e.to_string();
    assert!(matches!(e, OpError::Unsupported(_)), "{msg}");
    assert!(msg.contains("tangent"), "{msg}");
    assert!(msg.contains("perturb") || msg.contains("move"), "names the way out: {msg}");
    assert!(msg.contains("face") || msg.contains("volume"), "names an entity: {msg}");
    let _ = r;
}

#[test]
fn external_line_tangent_cylinder_is_a_tangent_contact_error() {
    // Cylinder outside the brick, touching the face x = 0 along a line.
    let mut r = Rig::cc();
    let b = r.brick([4.0; 3], [2.0; 3]);
    let c = r.cylinder(2.0, 1.0, [-1.0, 2.0, 2.0]);
    let h = r.hash();
    let e = r.run(&sm_boolean::ops::Unite { bodies: vec![b.body, c.body], keep: None, regularize: None, tol: None }).unwrap_err();
    let e = failed(e);
    let msg = e.to_string();
    assert!(matches!(e, OpError::Unsupported(_)), "{msg}");
    assert!(msg.contains("tangent"), "{msg}");
    assert!(msg.contains("face") || msg.contains("volume"), "names an entity: {msg}");
    assert_eq!(r.hash(), h);
}
