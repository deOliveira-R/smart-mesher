//! The proptest strategies produce valid objects.

use proptest::prelude::*;
use sm_core::{Point3, Transform};
use sm_mesh_api::{CellIx, MeshRead, cell_volume};
use sm_testkit::check::*;
use sm_testkit::strategies::*;
use sm_topo_api::{TopologyView, TopologyViewExt};

fn probes() -> [Point3; 3] {
    [Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 3.0), Point3::new(-2.0, 0.5, 4.0)]
}

fn dist(a: Point3, b: Point3) -> f64 {
    (a - b).norm()
}

fn close(a: Point3, b: Point3) -> bool {
    dist(a, b) < 1e-9
}

proptest! {
    #![proptest_config(config(32))]

    #[test]
    fn bricks_are_valid_with_the_right_volume(spec in brick_spec()) {
        prop_assert!(spec.size.iter().all(|s| *s >= 0.1));
        let f = spec.build().unwrap();
        prop_assert!(topo_valid(&f.complex).is_ok());
        prop_assert!(check_topo_euler(&f.complex, 2).is_ok());
        let v = f.complex.volume_moments(f.complex.volumes()[0]).unwrap().volume;
        prop_assert!((v - spec.volume()).abs() <= 1e-9 * spec.volume());
        let a: f64 = f.complex.faces().iter().map(|x| f.complex.face_moments(*x).unwrap().area).sum();
        prop_assert!((a - f.area).abs() <= 1e-9 * f.area);
    }

    #[test]
    fn hex_blocks_are_valid_with_the_right_volume(spec in hex_block_spec()) {
        let m = spec.build().unwrap();
        prop_assert_eq!(m.cell_count(), spec.cells());
        prop_assert!(mesh_valid(&m).is_ok());
        prop_assert!(watertight(&m).is_ok());
        prop_assert!(classification_complete(&m).is_ok());
        prop_assert!(check_mesh_euler(&m, 1).is_ok());
        let total: f64 = (0..m.cell_count()).map(|c| cell_volume(&m, CellIx::new(c))).sum();
        prop_assert!((total - spec.volume()).abs() <= 1e-12 * spec.volume());
        let carriers = box_carriers(spec.n, spec.h);
        prop_assert!(conformity(&m, &carriers, 1e-9 * spec.volume().cbrt().max(1.0)).is_ok());
    }

    #[test]
    fn proper_transforms_are_rigid(t in proper_transform()) {
        prop_assert!((t.det() - 1.0).abs() < 1e-9);
        prop_assert!(!t.is_improper());
        let p = probes();
        for i in 0..3 {
            for j in 0..i {
                let d0 = dist(p[i], p[j]);
                let d1 = dist(t.apply_point(p[i]), t.apply_point(p[j]));
                prop_assert!((d0 - d1).abs() < 1e-9 * d0.max(1.0));
            }
        }
        let inv = t.inverse().unwrap();
        for q in p {
            prop_assert!(close(inv.apply_point(t.apply_point(q)), q));
            prop_assert!(close(t.compose(&inv).apply_point(q), q));
        }
    }

    #[test]
    fn reflections_are_improper_involutions(t in reflection()) {
        prop_assert!((t.det() + 1.0).abs() < 1e-9);
        prop_assert!(t.is_improper());
        for q in probes() {
            prop_assert!(close(t.apply_point(t.apply_point(q)), q));
        }
    }

    #[test]
    fn any_transform_inverts(t in any_transform()) {
        prop_assert!((t.det().abs() - 1.0).abs() < 1e-9);
        prop_assert_eq!(t.is_improper(), t.det() < 0.0);
        let inv: Transform = t.inverse().unwrap();
        for q in probes() {
            prop_assert!(close(inv.apply_point(t.apply_point(q)), q));
        }
    }

    #[test]
    fn translations_and_rotations_are_proper(t in translation(), r in rotation()) {
        prop_assert!(!t.is_improper() && !r.is_improper());
        prop_assert!((r.det() - 1.0).abs() < 1e-9);
    }
}

#[test]
fn the_seed_is_pinned_so_runs_see_the_same_cases() {
    use proptest::strategy::ValueTree;
    use proptest::test_runner::TestRunner;
    let draw = || {
        let mut r = TestRunner::new(config(8));
        (0..8).map(|_| brick_spec().new_tree(&mut r).unwrap().current()).collect::<Vec<_>>()
    };
    assert_eq!(draw(), draw());
    assert!(matches!(config(5).rng_seed, proptest::test_runner::RngSeed::Fixed(SEED)));
    assert_eq!(config(5).cases, 5);
}
