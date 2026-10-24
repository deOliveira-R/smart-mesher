//! Timing budgets, measured on the production store alone (never through the paranoid twin, which
//! runs the reference and compares full state at every `end_op`).

use sm_core::Point3;
use sm_topo::TopoStore;
use sm_topo_api::fixtures;
use sm_topo_api::{Dim, OpId, TopologyEdit, TopologyView};

#[test]
fn building_many_bricks_is_near_linear() {
    let n: usize = if cfg!(debug_assertions) { 1_000 } else { 10_000 };
    let start = std::time::Instant::now();
    let mut t = TopoStore::new();
    t.begin_op(OpId(1)).unwrap();
    for i in 0..n {
        let x = (i % 100) as f64 * 2.0;
        let y = ((i / 100) % 100) as f64 * 2.0;
        fixtures::brick(&mut t, Point3::new(x, y, 0.0), [1.0; 3]).unwrap();
    }
    t.end_op().unwrap();
    let dt = start.elapsed();
    assert_eq!(t.cells(Dim::Volume).len(), n);
    assert_eq!(t.cells(Dim::Face).len(), 6 * n);
    if !cfg!(debug_assertions) {
        assert!(dt.as_secs_f64() < 2.0, "{n} bricks took {dt:?}");
    }
    // Queries stay O(1)/O(log n): the last volume's faces.
    let last = *t.cells(Dim::Volume).last().unwrap();
    assert_eq!(t.volume_faces(last).len(), 6);
}
