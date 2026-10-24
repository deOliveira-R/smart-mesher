//! Error-free transformations and expansion arithmetic.

mod common;
use common::Rng;
use proptest::prelude::*;
use sm_math::exact::*;
use sm_testkit::strategies::config;

/// Split a finite nonzero double into `(mantissa, exponent)` with `x = m * 2^e` exactly.
fn decompose(x: f64) -> (i128, i32) {
    if x == 0.0 {
        return (0, 0);
    }
    let bits = x.to_bits();
    let exp = ((bits >> 52) & 0x7ff) as i32;
    let frac = (bits & ((1u64 << 52) - 1)) as i128;
    let (m, e) = if exp == 0 {
        (frac, -1074)
    } else {
        (frac | (1 << 52), exp - 1075)
    };
    (if x < 0.0 { -m } else { m }, e)
}

/// `x` expressed as an integer multiple of `2^unit`; panics if not an exact multiple.
fn in_units(x: f64, unit: i32) -> i128 {
    let (m, e) = decompose(x);
    if m == 0 {
        return 0;
    }
    if e >= unit {
        assert!(e - unit < 70, "test shift too large");
        m << (e - unit)
    } else {
        assert_eq!(m & ((1 << (unit - e)) - 1), 0, "not a multiple of 2^unit");
        m >> (unit - e)
    }
}

fn val(max_exp: i32) -> impl Strategy<Value = f64> {
    (any::<bool>(), (1u64 << 52)..(1u64 << 53), -max_exp..max_exp).prop_map(|(neg, m, e)| {
        let v = m as f64 * 2f64.powi(e - 52);
        if neg { -v } else { v }
    })
}

proptest! {
    #![proptest_config(config(5000))]

    #[test]
    fn two_product_is_exact(a in val(80), b in val(80)) {
        let (x, y) = two_product(a, b);
        prop_assert_eq!(x, a * b);
        let (ma, ea) = decompose(a);
        let (mb, eb) = decompose(b);
        let unit = ea + eb;
        let want = ma * mb; // |.| < 2^106
        prop_assert_eq!(in_units(x, unit) + in_units(y, unit), want);
    }

    #[test]
    fn two_sum_and_two_diff_are_exact(a in val(30), b in val(30)) {
        let unit = decompose(a).1.min(decompose(b).1);
        let (x, y) = two_sum(a, b);
        prop_assert_eq!(x, a + b);
        prop_assert_eq!(in_units(x, unit) + in_units(y, unit), in_units(a, unit) + in_units(b, unit));
        let (x, y) = two_diff(a, b);
        prop_assert_eq!(in_units(x, unit) + in_units(y, unit), in_units(a, unit) - in_units(b, unit));
        if a.abs() >= b.abs() {
            prop_assert_eq!(fast_two_sum(a, b), two_sum(a, b));
        }
    }

    /// Ring identities hold only if every operation is exact.
    #[test]
    fn expansion_ring_identities(a in val(100), b in val(100), c in val(100), d in val(100)) {
        let (a, b, c) = (Expansion::from_diff(a, d), Expansion::from_f64(b), Expansion::from_product(c, d));
        let zero = |e: Expansion| e.is_zero();
        prop_assert!(zero(a.add(&b).sub(&b).sub(&a)));
        prop_assert!(zero(a.mul(&b).sub(&b.mul(&a))));
        prop_assert!(zero(a.mul(&b.add(&c)).sub(&a.mul(&b).add(&a.mul(&c)))));
        prop_assert!(zero(a.mul(&b).mul(&c).sub(&a.mul(&b.mul(&c)))));
        prop_assert!(zero(a.add(&b).add(&c).sub(&a.add(&b.add(&c)))));
        prop_assert!(zero(a.scale(2.0).sub(&a.add(&a))));
        prop_assert!(zero(a.compress().sub(&a)));
        prop_assert!(a.negated().add(&a).is_zero());
        // Structural invariants: nonzero, strictly increasing magnitude, sign = top component.
        for e in [a.mul(&c), a.add(&c), a.compress()] {
            let comps = e.components();
            prop_assert!(comps.iter().all(|&x| x != 0.0));
            prop_assert!(comps.windows(2).all(|w| w[0].abs() < w[1].abs()));
        }
    }
}

#[test]
fn sign_and_estimate() {
    // 2^100 + 1 - 2^100 is exactly 1, which no single double sum survives.
    let big = Expansion::from_f64(2f64.powi(100));
    let e = big.add(&Expansion::from_f64(1.0)).sub(&big);
    assert_eq!(e.components(), &[1.0]);
    assert_eq!(e.sign(), sm_math::Sign::Positive);
    assert_eq!(e.estimate(), 1.0);
    assert!(Expansion::zero().sign().is_zero());
    // (1 + 2^-60)^2 - 1 - 2^-59 == 2^-120 exactly.
    let x = Expansion::from_sum(1.0, 2f64.powi(-60));
    let r = x
        .square()
        .sub(&Expansion::from_f64(1.0))
        .sub(&Expansion::from_f64(2f64.powi(-59)));
    assert_eq!(r.components(), &[2f64.powi(-120)]);
}

#[test]
fn det_exact_matches_integer_determinants() {
    let mut rng = Rng(5);
    for _ in 0..300 {
        let n = 2 + rng.below(4);
        let m: Vec<Vec<i64>> = (0..n)
            .map(|_| (0..n).map(|_| rng.int(1000)).collect())
            .collect();
        let want = common::det(
            &m.iter()
                .map(|r| r.iter().map(|&v| i128::from(v)).collect())
                .collect::<Vec<_>>(),
        );
        let ex: Vec<Vec<Expansion>> = m
            .iter()
            .map(|r| r.iter().map(|&v| Expansion::from_f64(v as f64)).collect())
            .collect();
        assert_eq!(det_exact(&ex).sign(), common::sgn(want));
        assert_eq!(det_exact(&ex).estimate(), want as f64);
    }
}
