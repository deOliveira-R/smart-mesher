//! Planar booleans (acceptance 7 and 8, bricks).

mod common;

use common::*;
use sm_boolean::{BoolOptions, BoolRule, BooleanError, boolean};
use sm_topo_api::{CellComplex, TopologyView};

fn two(a_min: [f64; 3], a_size: [f64; 3], b_min: [f64; 3], b_size: [f64; 3]) -> (CellComplex, sm_topo_api::fixtures::BrickIds, sm_topo_api::fixtures::BrickIds) {
    let mut t = CellComplex::new();
    let a = brick(&mut t, 1, a_min, a_size);
    let b = brick(&mut t, 2, b_min, b_size);
    (t, a, b)
}

#[test]
fn unite_overlapping_bricks_without_regularize() {
    let (mut t, a, b) = two([0.0; 3], [2.0; 3], [1.0, 0.0, 0.0], [2.0; 3]);
    let o = BoolOptions { regularize: false, ..BoolOptions::default() };
    let out = in_op(&mut t, 3, |t| boolean(t, BoolRule::Unite, &[a.body], &[b.body], &o)).unwrap();
    assert_eq!(out.volumes.len(), 1);
    assert_valid(&t);
    assert!((volume_of(&t, out.volumes[0]) - 12.0).abs() < 1e-12);
    assert!((total_area(&t) - 32.0).abs() < 1e-12);
    assert_eq!(counts(&t), [16, 28, 14, 1, 1]);
    assert_eq!(euler(&t), 2);
}

#[test]
fn intersect_overlapping_bricks() {
    let (mut t, a, b) = two([0.0; 3], [2.0; 3], [1.0, 0.0, 0.0], [2.0; 3]);
    let o = BoolOptions { regularize: false, ..BoolOptions::default() };
    let out = in_op(&mut t, 3, |t| boolean(t, BoolRule::Intersect, &[a.body], &[b.body], &o)).unwrap();
    assert_eq!(out.volumes.len(), 1);
    assert_valid(&t);
    assert!((volume_of(&t, out.volumes[0]) - 4.0).abs() < 1e-12);
    assert!((total_area(&t) - 16.0).abs() < 1e-12);
}

#[test]
fn subtract_overlapping_bricks() {
    let (mut t, a, b) = two([0.0; 3], [2.0; 3], [1.0, 0.0, 0.0], [2.0; 3]);
    let o = BoolOptions { regularize: false, ..BoolOptions::default() };
    let out = in_op(&mut t, 3, |t| boolean(t, BoolRule::Subtract, &[a.body], &[b.body], &o)).unwrap();
    assert_eq!(out.volumes.len(), 1);
    assert_valid(&t);
    assert!((volume_of(&t, out.volumes[0]) - 4.0).abs() < 1e-12);
    assert!((total_area(&t) - 16.0).abs() < 1e-12);
}

#[test]
fn disjoint_bricks() {
    let (mut t, a, b) = two([0.0; 3], [1.0; 3], [5.0, 0.0, 0.0], [1.0; 3]);
    let o = BoolOptions::default();
    let r = in_op(&mut t, 3, |t| boolean(t, BoolRule::Unite, &[a.body], &[b.body], &o));
    assert!(matches!(r, Err(BooleanError::Invalid(_))), "{r:?}");
    let r = in_op(&mut t, 4, |t| boolean(t, BoolRule::Intersect, &[a.body], &[b.body], &o));
    assert!(matches!(r, Err(BooleanError::EmptyResult)), "{r:?}");
}

#[test]
fn unite_overlapping_bricks_regularized() {
    let (mut t, a, b) = two([0.0; 3], [2.0; 3], [1.0, 0.0, 0.0], [2.0; 3]);
    let out = in_op(&mut t, 3, |t| boolean(t, BoolRule::Unite, &[a.body], &[b.body], &BoolOptions::default())).unwrap();
    assert_valid(&t);
    assert_eq!(out.volumes.len(), 1);
    assert!((volume_of(&t, out.volumes[0]) - 12.0).abs() < 1e-12);
    assert!((total_area(&t) - 32.0).abs() < 1e-12);
    assert_eq!(counts(&t), [8, 12, 6, 1, 1]);
    assert_eq!(euler(&t), 2);
}

#[test]
fn intersect_and_subtract_regularized() {
    let (mut t, a, b) = two([0.0; 3], [2.0; 3], [1.0, 0.0, 0.0], [2.0; 3]);
    let out = in_op(&mut t, 3, |t| boolean(t, BoolRule::Intersect, &[a.body], &[b.body], &BoolOptions::default())).unwrap();
    assert_valid(&t);
    assert!((volume_of(&t, out.volumes[0]) - 4.0).abs() < 1e-12);
    assert_eq!(counts(&t), [8, 12, 6, 1, 1]);
    let (mut t, a, b) = two([0.0; 3], [2.0; 3], [1.0, 0.0, 0.0], [2.0; 3]);
    let out = in_op(&mut t, 3, |t| boolean(t, BoolRule::Subtract, &[a.body], &[b.body], &BoolOptions::default())).unwrap();
    assert_valid(&t);
    assert!((volume_of(&t, out.volumes[0]) - 4.0).abs() < 1e-12);
    assert_eq!(counts(&t), [8, 12, 6, 1, 1]);
}
