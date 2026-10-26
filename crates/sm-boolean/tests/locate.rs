//! `locate_point` on the topology fixtures.

mod common;

use sm_boolean::{PointLocation, locate_point};
use sm_core::Point3;
use sm_topo_api::fixtures;
use sm_topo_api::{CellComplex, OpId, TopologyEdit};

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}

fn build<T>(f: impl FnOnce(&mut CellComplex) -> T) -> (CellComplex, T) {
    let mut t = CellComplex::new();
    t.begin_op(OpId(1)).unwrap();
    let r = f(&mut t);
    t.end_op().unwrap();
    (t, r)
}

#[test]
fn brick_in_on_out() {
    let (t, b) = build(|t| fixtures::brick(t, p(0.0, 0.0, 0.0), [2.0, 3.0, 4.0]).unwrap());
    assert_eq!(locate_point(&t, b.volume, p(1.0, 1.0, 1.0)).unwrap(), PointLocation::In);
    assert_eq!(locate_point(&t, b.volume, p(2.0, 1.0, 1.0)).unwrap(), PointLocation::On);
    assert_eq!(locate_point(&t, b.volume, p(0.0, 0.0, 0.0)).unwrap(), PointLocation::On);
    assert_eq!(locate_point(&t, b.volume, p(2.5, 1.0, 1.0)).unwrap(), PointLocation::Out);
    assert_eq!(locate_point(&t, b.volume, p(-1.0, -1.0, -1.0)).unwrap(), PointLocation::Out);
    // Aligned with edges and vertices: the irrational probe directions must not care.
    for k in 0..4 {
        let x = 0.25 + 0.5 * k as f64;
        assert_eq!(locate_point(&t, b.volume, p(x, 1.5, 2.0)).unwrap(), PointLocation::In);
    }
}

#[test]
fn cylinder_and_sphere() {
    let (t, c) = build(|t| fixtures::cylinder(t, 1.0, 2.0).unwrap());
    assert_eq!(locate_point(&t, c.volume, p(0.0, 0.0, 1.0)).unwrap(), PointLocation::In);
    assert_eq!(locate_point(&t, c.volume, p(0.9, 0.0, 1.9)).unwrap(), PointLocation::In);
    assert_eq!(locate_point(&t, c.volume, p(1.0, 0.0, 1.0)).unwrap(), PointLocation::On);
    assert_eq!(locate_point(&t, c.volume, p(1.1, 0.0, 1.0)).unwrap(), PointLocation::Out);
    assert_eq!(locate_point(&t, c.volume, p(0.0, 0.0, 2.5)).unwrap(), PointLocation::Out);
    let (t, s) = build(|t| fixtures::sphere(t, 2.0).unwrap());
    assert_eq!(locate_point(&t, s.volume, p(0.5, 0.5, 0.5)).unwrap(), PointLocation::In);
    assert_eq!(locate_point(&t, s.volume, p(0.0, 0.0, 2.0)).unwrap(), PointLocation::On);
    assert_eq!(locate_point(&t, s.volume, p(1.5, 1.5, 0.0)).unwrap(), PointLocation::Out);
}

#[test]
fn cavity_is_outside_the_volume() {
    let (t, b) = build(|t| fixtures::brick_with_void(t, p(0.0, 0.0, 0.0), [10.0; 3], p(4.0, 4.0, 4.0), [2.0; 3]).unwrap());
    assert_eq!(locate_point(&t, b.outer.volume, p(1.0, 1.0, 1.0)).unwrap(), PointLocation::In);
    assert_eq!(locate_point(&t, b.outer.volume, p(5.0, 5.0, 5.0)).unwrap(), PointLocation::Out);
    assert_eq!(locate_point(&t, b.outer.volume, p(4.0, 5.0, 5.0)).unwrap(), PointLocation::On);
    assert_eq!(locate_point(&t, b.outer.volume, p(11.0, 5.0, 5.0)).unwrap(), PointLocation::Out);
}
