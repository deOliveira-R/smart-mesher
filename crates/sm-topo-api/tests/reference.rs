//! Reference-implementation tests (in progress).

use sm_core::Point3;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::*;

fn assert_valid(v: &dyn TopologyView) {
    let bad = validate(v);
    assert!(bad.is_empty(), "violations: {bad:#?}");
}

#[test]
fn brick_smoke() {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [2.0, 3.0, 4.0])).unwrap();
    assert_valid(&t);
    assert_eq!(t.cells(Dim::Face).len(), 6);
    assert_eq!(euler_characteristic(&t), 2);
    for f in b.faces {
        assert_eq!(t.face_sides(f).unwrap().left, Region::Volume(b.volume));
    }
    let m = t.volume_moments(b.volume).unwrap();
    assert!((m.volume - 24.0).abs() < 1e-9, "{m:?}");
}
