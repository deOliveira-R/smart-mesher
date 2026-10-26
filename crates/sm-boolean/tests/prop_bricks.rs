//! Acceptance 10: property tests on integer-grid brick pairs.

mod common;

use common::*;
use proptest::prelude::*;
use sm_boolean::{BoolOptions, BoolRule, BooleanError, boolean};
use sm_topo_api::CellComplex;

fn interval_overlap(a0: i32, a1: i32, b0: i32, b1: i32) -> i32 {
    (a1.min(b1) - a0.max(b0)).max(0)
}

/// Overlap extents per axis (>= 0), and whether the closed boxes touch at all.
fn relation(a: &[i32; 6], b: &[i32; 6]) -> ([i32; 3], bool) {
    let mut ov = [0; 3];
    let mut touch = true;
    for k in 0..3 {
        ov[k] = interval_overlap(a[k], a[k] + a[k + 3], b[k], b[k] + b[k + 3]);
        if a[k].max(b[k]) > (a[k] + a[k + 3]).min(b[k] + b[k + 3]) {
            touch = false;
        }
    }
    (ov, touch)
}

fn setup(a: &[i32; 6], b: &[i32; 6]) -> (CellComplex, sm_topo_api::fixtures::BrickIds, sm_topo_api::fixtures::BrickIds) {
    let mut t = CellComplex::new();
    let f = |x: i32| x as f64;
    let ba = brick(&mut t, 1, [f(a[0]), f(a[1]), f(a[2])], [f(a[3]), f(a[4]), f(a[5])]);
    let bb = brick(&mut t, 2, [f(b[0]), f(b[1]), f(b[2])], [f(b[3]), f(b[4]), f(b[5])]);
    (t, ba, bb)
}

fn vol(a: &[i32; 6]) -> f64 {
    (a[3] * a[4] * a[5]) as f64
}

fn arb_brick() -> impl Strategy<Value = [i32; 6]> {
    (0..6i32, 0..6i32, 0..6i32, 1..5i32, 1..5i32, 1..5i32).prop_map(|(x, y, z, sx, sy, sz)| [x, y, z, sx, sy, sz])
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 300, ..ProptestConfig::default() })]

    #[test]
    fn inclusion_exclusion_and_symmetry(a in arb_brick(), b in arb_brick()) {
        prop_assume!(a != b || true);
        let (ov, touch) = relation(&a, &b);
        let v_int = (ov[0] * ov[1] * ov[2]) as f64;
        let (va, vb) = (vol(&a), vol(&b));
        let opts = BoolOptions::default();
        // Intersect.
        let (mut t, ba, bb) = setup(&a, &b);
        let r = in_op(&mut t, 3, |t| boolean(t, BoolRule::Intersect, &[ba.body], &[bb.body], &opts));
        if v_int > 0.0 {
            let out = r.map_err(|e| TestCaseError::fail(format!("intersect {a:?} {b:?}: {e}")))?;
            prop_assert_eq!(out.volumes.len(), 1);
            prop_assert!((volume_of(&t, out.volumes[0]) - v_int).abs() < 1e-12);
            assert_valid(&t);
        } else {
            prop_assert!(matches!(r, Err(BooleanError::EmptyResult)), "intersect {:?} {:?}: {:?}", a, b, r);
        }
        // Subtract.
        let (mut t, ba, bb) = setup(&a, &b);
        let r = in_op(&mut t, 3, |t| boolean(t, BoolRule::Subtract, &[ba.body], &[bb.body], &opts));
        if va - v_int > 0.0 {
            let out = r.map_err(|e| TestCaseError::fail(format!("subtract {a:?} {b:?}: {e}")))?;
            let total: f64 = out.volumes.iter().map(|&v| volume_of(&t, v)).sum();
            prop_assert!((total - (va - v_int)).abs() < 1e-12, "subtract {:?} {:?}: {} vs {}", a, b, total, va - v_int);
            assert_valid(&t);
        } else {
            prop_assert!(r.is_err(), "subtract {:?} {:?} should be empty", a, b);
        }
        // Unite, both orders.
        let mut sig: Vec<(f64, f64, [usize; 5])> = Vec::new();
        for flip in [false, true] {
            let (mut t, ba, bb) = setup(&a, &b);
            let (x, y) = if flip { (bb.body, ba.body) } else { (ba.body, bb.body) };
            let r = in_op(&mut t, 3, |t| boolean(t, BoolRule::Unite, &[x], &[y], &opts));
            let face_overlap = {
                // Touching along a face of positive area: exactly one axis has zero overlap, the others positive.
                let zeros = ov.iter().filter(|&&o| o == 0).count();
                touch && zeros == 1
            };
            if v_int > 0.0 || face_overlap {
                let out = r.map_err(|e| TestCaseError::fail(format!("unite {a:?} {b:?} flip={flip}: {e}")))?;
                prop_assert_eq!(out.volumes.len(), 1);
                let v = volume_of(&t, out.volumes[0]);
                prop_assert!((v - (va + vb - v_int)).abs() < 1e-12, "unite {:?} {:?}: {} vs {}", a, b, v, va + vb - v_int);
                assert_valid(&t);
                sig.push((v, total_area(&t), counts(&t)));
            } else {
                prop_assert!(r.is_err(), "unite {:?} {:?} should fail (no shared face)", a, b);
            }
        }
        if sig.len() == 2 {
            prop_assert!((sig[0].0 - sig[1].0).abs() < 1e-12 && (sig[0].1 - sig[1].1).abs() < 1e-9 && sig[0].2 == sig[1].2, "{:?}", sig);
        }
    }
}
