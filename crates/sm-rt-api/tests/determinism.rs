//! Determinism of the reference implementor: run twice, and with 1 thread vs the default pool,
//! then compare hashes (the workspace rule for every crate).
#![cfg(feature = "stubs")]

use sm_core::{Aabb, Point3, Vector3};
use sm_geom_api::Ray;
use sm_rt_api::stub::{AnalyticWorld, RectLattice, Shape};
use sm_rt_api::{ContentHash, FaceAction, ResponseKind, Tracker, TraceOptions};

fn world() -> AnalyticWorld {
    let shapes = vec![
        Shape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: 1.0 },
        Shape::Box { lo: Point3::new(1.2, -0.3, -0.3), hi: Point3::new(1.8, 0.3, 0.3) },
    ];
    let l = RectLattice::new(Point3::new(0.0, 0.0, 0.0), [4.0; 3], [6, 5, 3]);
    AnalyticWorld::with_lattice(shapes, l, [FaceAction::Respond(ResponseKind::Vacuum); 6]).unwrap().with_block_size(37)
}

/// A fan of rays with no RNG and no libm trig: direction components are rational.
fn rays(n: usize) -> Vec<Ray> {
    (0..n)
        .map(|i| {
            let (a, b, c) = ((i % 7) as f64 + 1.0, (i % 5) as f64 - 2.0, (i % 3) as f64 + 0.5);
            Ray {
                origin: Point3::new(0.3 + 0.017 * i as f64 % 3.0, 0.2 + 0.013 * i as f64 % 4.0, 0.4 + 0.011 * i as f64 % 2.0),
                dir: Vector3::new(a, b, c),
            }
        })
        .collect()
}

fn bundle_hash(w: &AnalyticWorld, r: &[Ray], threads: Option<usize>) -> ContentHash {
    let opts = TraceOptions::default();
    match threads {
        Some(n) => rayon::ThreadPoolBuilder::new().num_threads(n).build().unwrap().install(|| w.trace_bundle(r, &opts).unwrap().content_hash()),
        None => w.trace_bundle(r, &opts).unwrap().content_hash(),
    }
}

#[test]
fn builds_and_bundles_are_bit_identical_across_runs_and_thread_counts() {
    let (a, b) = (world(), world());
    assert_eq!(a.structure_hash(), b.structure_hash());
    let r = rays(2000);
    let first = bundle_hash(&a, &r, None);
    assert_eq!(first, bundle_hash(&a, &r, None), "run twice");
    assert_eq!(first, bundle_hash(&a, &r, Some(1)), "1 thread");
    assert_eq!(first, bundle_hash(&a, &r, Some(4)), "4 threads");
    assert_eq!(first, bundle_hash(&b, &r, Some(3)), "rebuilt world, 3 threads");
    // The block size is part of the partition, not of the result.
    let c = world().with_block_size(500);
    assert_eq!(first, bundle_hash(&c, &r, Some(2)));
    let d = Aabb { lo: Point3::new(0.0, 0.0, 0.0), hi: Point3::new(24.0, 20.0, 12.0) };
    assert_eq!(a.bounds(), d);
}
