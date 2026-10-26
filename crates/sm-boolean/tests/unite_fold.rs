//! Unite of more than two operands: the result must not depend on the operand order.

mod common;

use std::collections::BTreeSet;

use common::*;
use sm_boolean::{BoolOptions, BoolRule, BooleanError, boolean};
use sm_topo_api::{CellComplex, TopologyView};

/// Bricks A=[0,1]^3, C=[2,3]x[0,1]^2, B=[0.5,2.5]x[0,1]^2: A and C do not touch, B bridges them.
fn three_bricks() -> (CellComplex, [sm_topo_api::fixtures::BrickIds; 3]) {
    let mut t = CellComplex::new();
    let a = brick(&mut t, 1, [0.0; 3], [1.0; 3]);
    let c = brick(&mut t, 2, [2.0, 0.0, 0.0], [1.0; 3]);
    let b = brick(&mut t, 3, [0.5, 0.0, 0.0], [2.0, 1.0, 1.0]);
    (t, [a, c, b])
}

#[test]
fn unite_of_a_connected_set_does_not_depend_on_the_order() {
    for order in [[0, 1, 2], [0, 2, 1], [1, 0, 2], [2, 0, 1], [1, 2, 0], [2, 1, 0]] {
        let (mut t, bricks) = three_bricks();
        let bodies: Vec<_> = order.iter().map(|&i| bricks[i].body).collect();
        let out = in_op(&mut t, 4, |t| boolean(t, BoolRule::Unite, &bodies[..1], &bodies[1..], &BoolOptions::default())).unwrap_or_else(|e| panic!("order {order:?}: {e}"));
        assert_valid(&t);
        assert_eq!(out.volumes.len(), 1, "{order:?}");
        assert!((volume_of(&t, out.volumes[0]) - 3.0).abs() < 1e-12, "{order:?}");
        assert!((total_area(&t) - 14.0).abs() < 1e-12, "{order:?}");
        assert_eq!(counts(&t), [8, 12, 6, 1, 1], "{order:?}");
        // Roles are disjoint and cover the faces of the result.
        let all: BTreeSet<_> = t.volume_faces(out.volumes[0]).iter().copied().collect();
        let nf: BTreeSet<_> = out.new_faces.iter().copied().collect();
        let rest: BTreeSet<_> = out.faces.iter().copied().collect();
        assert!(nf.is_disjoint(&rest), "{order:?}");
        assert_eq!(nf.union(&rest).copied().collect::<BTreeSet<_>>(), all, "{order:?}");
    }
}

#[test]
fn unite_new_faces_accumulate_over_all_fuses() {
    let (mut t, [a, c, b]) = three_bricks();
    let o = BoolOptions { regularize: false, ..BoolOptions::default() };
    let out = in_op(&mut t, 4, |t| boolean(t, BoolRule::Unite, &[a.body, b.body], &[c.body], &o)).unwrap();
    assert_valid(&t);
    assert!(out.new_faces.len() >= 2, "{out:?}");
}

#[test]
fn unite_of_an_unconnected_operand_is_invalid() {
    let (mut t, [a, c, _b]) = three_bricks();
    let r = in_op(&mut t, 4, |t| boolean(t, BoolRule::Unite, &[a.body], &[c.body], &BoolOptions::default()));
    assert!(matches!(r, Err(BooleanError::Invalid(ref m)) if m.contains("do not touch")), "{r:?}");
}
