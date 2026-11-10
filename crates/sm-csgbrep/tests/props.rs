//! Property test: random pin cells (radii and pitch kept clear of the kernel's near-tangent
//! limit) import to the analytic volumes.
mod common;
use common::decks::*;
use common::*;
use core::f64::consts::PI;
use proptest::prelude::*;

proptest! {
    #![proptest_config(sm_testkit::strategies::config(16))]

    #[test]
    fn random_pin_lattices_have_analytic_volumes(r1 in 0.1..0.3_f64, g in 0.03..0.1_f64, c in 0.03..0.1_f64, slack in 0.04..0.3_f64, h in 0.5..3.0_f64, n in 1usize..4) {
        let (r2, r3) = (r1 + g, r1 + g + c);
        let p = 2.0 * (r3 + slack);
        let mut r = Rig::new();
        let d = r.import(&pin_lattice(n, p, (r1, r2, r3), h, false), Some(&materials(4))).unwrap();
        prop_assert_eq!(d.outputs["deck_volumes"].len(), 4 * n * n);
        let total: f64 = d.outputs["deck_volumes"].iter().map(|&v| r.volume(v)).sum();
        let want = (n as f64 * p).powi(2) * h;
        prop_assert!((total - want).abs() <= 1e-9 * want);
        let mut vols: Vec<f64> = d.outputs["deck_volumes"].iter().map(|&v| r.volume(v)).collect();
        vols.sort_by(f64::total_cmp);
        let mut expect: Vec<f64> = (0..n * n).flat_map(|_| [PI * r1 * r1 * h, PI * (r2 * r2 - r1 * r1) * h, PI * (r3 * r3 - r2 * r2) * h, (p * p - PI * r3 * r3) * h]).collect();
        expect.sort_by(f64::total_cmp);
        for (a, b) in vols.iter().zip(&expect) {
            prop_assert!((a - b).abs() <= 1e-9 * b.max(1e-3), "{a} vs {b}");
        }
    }
}
