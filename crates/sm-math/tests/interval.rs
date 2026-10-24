//! Interval enclosure: every operation must contain the exact result for every sampled pair.

use proptest::prelude::*;
use sm_math::Sign;
use sm_math::exact::Expansion;
use sm_math::interval::{Interval, IntervalError};
use sm_testkit::strategies::config;

fn val() -> impl Strategy<Value = f64> {
    (any::<bool>(), 1.0f64..2.0, -150i32..150).prop_map(|(neg, m, e)| {
        let v = m * 2f64.powi(e);
        if neg { -v } else { v }
    })
}

fn iv() -> impl Strategy<Value = Interval> {
    (val(), val()).prop_map(|(a, b)| Interval::hull_of(a, b))
}

fn samples(i: &Interval) -> [f64; 3] {
    [i.lo(), i.hi(), i.mid()]
}

/// lo <= x <= hi for an exactly represented real `x`.
fn encloses(i: &Interval, x: &Expansion) -> bool {
    let below = Expansion::from_f64(i.lo()).sub(x).sign() != Sign::Positive;
    let above = Expansion::from_f64(i.hi()).sub(x).sign() != Sign::Negative;
    below && above
}

proptest! {
    #![proptest_config(config(2000))]

    #[test]
    fn add_sub_mul_enclose(a in iv(), b in iv()) {
        let (s, d, p) = (a + b, a - b, a * b);
        for x in samples(&a) {
            for y in samples(&b) {
                prop_assert!(encloses(&s, &Expansion::from_sum(x, y)));
                prop_assert!(encloses(&d, &Expansion::from_diff(x, y)));
                prop_assert!(encloses(&p, &Expansion::from_product(x, y)));
            }
        }
    }

    #[test]
    fn division_encloses(a in iv(), b in iv()) {
        match a.checked_div(&b) {
            Err(e) => {
                prop_assert_eq!(e, IntervalError::DivisionByZero);
                prop_assert!(b.contains_zero());
                prop_assert_eq!(a / b, Interval::ENTIRE);
            }
            Ok(q) => {
                prop_assert!(!b.contains_zero());
                for x in samples(&a) {
                    for y in samples(&b) {
                        // q.lo <= x/y <= q.hi  <=>  compare q*y against x with y's sign.
                        let lo_y = Expansion::from_product(q.lo(), y).sub(&Expansion::from_f64(x)).sign();
                        let hi_y = Expansion::from_product(q.hi(), y).sub(&Expansion::from_f64(x)).sign();
                        let (lo_ok, hi_ok) = if y > 0.0 {
                            (lo_y != Sign::Positive, hi_y != Sign::Negative)
                        } else {
                            (lo_y != Sign::Negative, hi_y != Sign::Positive)
                        };
                        prop_assert!(lo_ok && hi_ok, "{x} / {y} not in {q:?}");
                    }
                }
            }
        }
    }

    #[test]
    fn sqr_sqrt_abs_enclose(a in iv()) {
        let sq = a.sqr();
        prop_assert!(sq.lo() >= 0.0);
        let ab = a.abs();
        for x in samples(&a) {
            prop_assert!(encloses(&sq, &Expansion::from_product(x, x)));
            prop_assert!(ab.contains(x.abs()));
        }
        match a.sqrt() {
            Err(e) => { prop_assert_eq!(e, IntervalError::Domain); prop_assert!(a.hi() < 0.0); }
            Ok(r) => {
                for x in samples(&a).into_iter().filter(|&x| x >= 0.0) {
                    let lo = Expansion::from_product(r.lo(), r.lo()).sub(&Expansion::from_f64(x)).sign();
                    let hi = Expansion::from_product(r.hi(), r.hi()).sub(&Expansion::from_f64(x)).sign();
                    prop_assert!(lo != Sign::Positive && hi != Sign::Negative, "sqrt({x}) not in {r:?}");
                }
            }
        }
    }

    #[test]
    fn set_operations(a in iv(), b in iv()) {
        let h = a.hull(&b);
        prop_assert!(a.is_subset_of(&h) && b.is_subset_of(&h));
        match a.intersection(&b) {
            Some(i) => { prop_assert!(i.is_subset_of(&a) && i.is_subset_of(&b)); prop_assert!(a.overlaps(&b)); }
            None => prop_assert!(!a.overlaps(&b)),
        }
        prop_assert!(a.width() >= 0.0);
    }

    #[test]
    fn powi_encloses(a in iv(), n in 0u32..6) {
        let p = a.powi(n);
        for x in samples(&a) {
            let mut e = Expansion::from_f64(1.0);
            for _ in 0..n { e = e.mul(&Expansion::from_f64(x)); }
            if n > 0 && a.lo().abs().max(a.hi().abs()) < 2f64.powi(150) {
                prop_assert!(encloses(&p, &e), "{x}^{n} not in {p:?}");
            }
        }
    }
}

#[test]
fn exactly_representable_results_stay_tight() {
    let (a, b) = (Interval::point(2.0), Interval::point(3.0));
    for r in [a + b, a * b, b - a] {
        assert_eq!(r.lo(), r.hi(), "{r:?}");
    }
    assert_eq!((a + b).lo(), 5.0);
    assert_eq!((a * b).lo(), 6.0);
    // Inexact: the double nearest 0.1 + 0.2 is 0.30000000000000004 and the exact sum is
    // slightly below it, so the enclosure is the single ulp interval [0.3, 0.30000000000000004].
    let r = Interval::point(0.1) + Interval::point(0.2);
    assert_eq!((r.lo(), r.hi()), (0.3, 0.300_000_000_000_000_04));
}

#[test]
fn edge_cases() {
    assert!(Interval::new(2.0, 1.0).is_err());
    assert!(Interval::new(f64::NAN, 1.0).is_err());
    assert!(Interval::new(f64::INFINITY, f64::INFINITY).is_err());
    assert_eq!(Interval::point(f64::NAN), Interval::ENTIRE);
    // 0 * inf = 0 convention; no NaN escapes.
    let z = Interval::point(0.0) * Interval::ENTIRE;
    assert_eq!((z.lo(), z.hi()), (0.0, 0.0));
    let big = Interval::point(1e300) * Interval::point(1e300);
    assert!(big.lo().is_finite() && big.hi() == f64::INFINITY);
    let tiny = Interval::point(1e-300) * Interval::point(1e-300);
    assert!(tiny.lo() <= 1e-600 && tiny.hi() > 0.0);
    let q = Interval::new(1.0, 2.0).unwrap() / Interval::new(-1.0, 1.0).unwrap();
    assert_eq!(q, Interval::ENTIRE);
    assert_eq!(Interval::new(-1.0, 1.0).unwrap().sign(), None);
    assert_eq!(
        Interval::new(1.0, 2.0).unwrap().sign(),
        Some(Sign::Positive)
    );
    assert_eq!(Interval::point(0.0).sign(), Some(Sign::Zero));
    assert_eq!(
        Interval::new(-4.0, -1.0).unwrap().sqrt(),
        Err(IntervalError::Domain)
    );
    let s = Interval::new(-4.0, 9.0).unwrap().sqrt().unwrap();
    assert!(s.lo() == 0.0 && s.hi() >= 3.0);
    let r = Interval::around(1.0, 0.5);
    assert!(r.contains(0.5) && r.contains(1.5));
}

#[test]
fn constructors_keep_invariants_for_nonfinite_input() {
    let valid = |i: Interval| {
        !i.lo().is_nan()
            && !i.hi().is_nan()
            && i.lo() < f64::INFINITY
            && i.hi() > f64::NEG_INFINITY
            && i.lo() <= i.hi()
    };
    let vals = [
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
        0.0,
        1.0,
        -1.0,
        f64::MAX,
    ];
    for &a in &vals {
        for &b in &vals {
            assert!(valid(Interval::around(a, b)), "around({a},{b})");
            assert!(valid(Interval::hull_of(a, b)), "hull_of({a},{b})");
        }
    }
    assert_eq!(Interval::around(1.0, f64::NAN), Interval::ENTIRE);
    assert_eq!(
        Interval::hull_of(f64::INFINITY, f64::INFINITY),
        Interval::ENTIRE
    );
}

#[test]
fn mid_is_inside_for_subnormals() {
    let t = 5e-324;
    let i = Interval::new(t, t).unwrap();
    assert!(i.contains(i.mid()));
    let j = Interval::new(t, 2.0 * t).unwrap();
    assert!(j.contains(j.mid()));
}
