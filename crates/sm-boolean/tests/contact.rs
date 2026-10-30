//! Contact that is not a proper overlap: edge-only and vertex-only meetings of the operands, and
//! tangent lines that a third operand covers (any operand order).

mod common;

use common::rig::*;
use common::*;
use sm_boolean::ops::{Intersect, Subtract, Unite};
use sm_boolean::{BoolOptions, BoolRule, BooleanError, boolean};
use sm_op_api::{ApplyError, OpError, TimelineError};

fn failed(e: TimelineError) -> OpError {
    match e {
        TimelineError::Apply(ApplyError::Op { source, .. }) => source,
        other => panic!("{other:?}"),
    }
}

const OFFSETS: [(&str, [f64; 3]); 2] = [("edge", [1.0, 1.0, 0.0]), ("vertex", [1.0, 1.0, 1.0])];

fn two(off: [f64; 3], rule: BoolRule) -> (sm_topo_api::CellComplex, Result<sm_boolean::BoolOutcome, BooleanError>) {
    let mut t = new_cc();
    let a = brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    let b = brick(&mut t, 2, off, [1.0; 3]);
    let r = in_op(&mut t, 3, |t| boolean(t, rule, &[a.body], &[b.body], &BoolOptions::default()));
    (t, r)
}

#[test]
fn unite_of_edge_or_vertex_contact_is_a_tangent_contact() {
    for (name, off) in OFFSETS {
        let (_, r) = two(off, BoolRule::Unite);
        match r {
            Err(BooleanError::TangentContact { detail, .. }) => assert!(detail.contains("edge") && detail.contains("manifold"), "{name}: {detail}"),
            other => panic!("{name}: {other:?}"),
        }
    }
}

#[test]
fn intersect_of_edge_or_vertex_contact_is_empty() {
    for (name, off) in OFFSETS {
        let (_, r) = two(off, BoolRule::Intersect);
        assert_eq!(r.unwrap_err(), BooleanError::EmptyResult, "{name}");
    }
}

#[test]
fn subtract_of_edge_or_vertex_contact_leaves_the_target() {
    for (name, off) in OFFSETS {
        let (t, r) = two(off, BoolRule::Subtract);
        let out = r.unwrap_or_else(|e| panic!("{name}: {e}"));
        assert_valid(&t);
        assert_eq!(out.volumes.len(), 1, "{name}");
        assert!((volume_of(&t, out.volumes[0]) - 1.0).abs() < 1e-12, "{name}");
        assert_eq!(counts(&t), [8, 12, 6, 1, 1], "{name}");
    }
}

#[test]
fn unite_op_edge_contact_is_unsupported_and_rolls_back() {
    let mut r = Rig::cc();
    let a = r.brick([1.0; 3], [0.5; 3]);
    let b = r.brick([1.0; 3], [1.5, 1.5, 0.5]);
    let h = r.hash();
    let e = failed(r.run(&Unite { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap_err());
    assert!(matches!(e, OpError::Unsupported(_)), "{e:?}");
    assert!(e.to_string().contains("edge"), "{e}");
    assert_eq!(r.hash(), h);
}

#[test]
fn edge_contact_candidate_is_skipped_when_a_bridge_connects_it() {
    // A=[0,1]^3, E meets A along an edge, B=[0.5,1.5]^2 x [0,1] overlaps both.
    for order in [[0, 1, 2], [0, 2, 1], [1, 0, 2], [2, 0, 1], [1, 2, 0], [2, 1, 0]] {
        let mut t = new_cc();
        let bricks = [brick(&mut t, 1, [0.0; 3], [1.0; 3]), brick(&mut t, 2, [1.0, 1.0, 0.0], [1.0; 3]), brick(&mut t, 3, [0.5, 0.5, 0.0], [1.0; 3])];
        let bodies: Vec<_> = order.iter().map(|&i| bricks[i].body).collect();
        let out = in_op(&mut t, 4, |t| boolean(t, BoolRule::Unite, &bodies[..1], &bodies[1..], &BoolOptions::default())).unwrap_or_else(|e| panic!("{order:?}: {e}"));
        assert_valid(&t);
        assert_eq!(out.volumes.len(), 1, "{order:?}");
        assert!((volume_of(&t, out.volumes[0]) - 2.5).abs() < 1e-12, "{order:?}");
    }
}

#[test]
fn curved_point_tangency_is_a_tangent_contact_for_every_rule() {
    let mut r = Rig::cc();
    let a = r.brick([1.0; 3], [0.5; 3]);
    let s = r.sphere(0.5, [0.5, 0.5, 1.5]);
    let h = r.hash();
    let ops: [&dyn sm_op_api::Op; 3] = [
        &Unite { bodies: vec![a.body, s.body], keep: None, regularize: None, tol: None },
        &Intersect { bodies: vec![a.body, s.body], keep: None, regularize: None, tol: None },
        &Subtract { targets: vec![a.body], tools: vec![s.body], keep: None, regularize: None, tol: None },
    ];
    for op in ops {
        let e = failed(r.run(op).unwrap_err());
        assert!(matches!(e, OpError::Unsupported(_)) && e.to_string().contains("tangent contact"), "{e:?}");
        assert_eq!(r.hash(), h);
    }
}

/// Brick A 4^3 at (2,2,2); cylinder r=1 h=2 at (-1,2,2) tangent to A's face x=0 along a line; a
/// bridge brick (1,1,3) at (0,2,2) covers that line.
#[test]
fn tangent_line_covered_by_a_bridge_works_in_any_operand_order() {
    let mut vols = Vec::new();
    for order in [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]] {
        let mut r = Rig::cc();
        let solids = [r.brick([4.0; 3], [2.0; 3]), r.cylinder(2.0, 1.0, [-1.0, 2.0, 2.0]), r.brick([1.0, 1.0, 3.0], [0.0, 2.0, 2.0])];
        let bodies: Vec<_> = order.iter().map(|&i| solids[i].body).collect();
        let d = r.run(&Unite { bodies, keep: None, regularize: None, tol: None }).unwrap_or_else(|e| panic!("{order:?}: {e}"));
        let t = r.topo();
        assert_valid(t);
        assert_eq!(d.outputs["volumes"].len(), 1, "{order:?}");
        vols.push(volume_of(t, d.outputs["volumes"][0]));
    }
    for v in &vols {
        assert!((v - 70.870).abs() < 5e-3, "{vols:?}");
        assert!((v - vols[0]).abs() < 1e-9, "{vols:?}");
    }
}

#[test]
fn tangent_line_without_a_bridge_still_fails_and_rolls_back() {
    let mut r = Rig::cc();
    let a = r.brick([4.0; 3], [2.0; 3]);
    let c = r.cylinder(2.0, 1.0, [-1.0, 2.0, 2.0]);
    let far = r.brick([1.0; 3], [10.0; 3]);
    let h = r.hash();
    let e = failed(r.run(&Unite { bodies: vec![a.body, c.body, far.body], keep: None, regularize: None, tol: None }).unwrap_err());
    assert!(e.to_string().contains("tangent"), "{e}");
    assert_eq!(r.hash(), h);
}
