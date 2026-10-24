//! Closed-form checks of the arc-length spacing math.

use sm_scheme_curve::SpacingError;
use sm_scheme_curve::spacing::*;

/// Edge lengths (fractions of the total) from interior fractions.
fn edges(f: &[f64]) -> Vec<f64> {
    let mut v = vec![0.0];
    v.extend_from_slice(f);
    v.push(1.0);
    v.windows(2).map(|w| w[1] - w[0]).collect()
}

fn assert_close(a: f64, b: f64, tol: f64, what: &str) {
    assert!((a - b).abs() <= tol * b.abs().max(1.0), "{what}: {a} vs {b}");
}

#[test]
fn bias_consecutive_edge_ratio_is_r() {
    for (n, r) in [(2u32, 1.7), (7, 1.3), (12, 0.8), (30, 1.05), (5, 1.0), (9, 1.0 + 1e-9), (40, 0.97)] {
        let f = bias_fractions(n, r).unwrap();
        assert_eq!(f.len(), n as usize - 1);
        let e = edges(&f);
        for w in e.windows(2) {
            assert_close(w[1] / w[0], r, 1e-12, &format!("n={n} r={r}"));
        }
        assert_close(e.iter().sum::<f64>(), 1.0, 1e-15, "sum");
        // Closed form: first edge = (r - 1) / (r^n - 1).
        if r != 1.0 && (r - 1.0).abs() > 1e-6 {
            assert_close(e[0], (r - 1.0) / (r.powi(n as i32) - 1.0), 1e-12, "first edge");
        }
    }
}

#[test]
fn bias_ratio_one_is_equal() {
    let f = bias_fractions(8, 1.0).unwrap();
    for (k, x) in f.iter().enumerate() {
        assert_eq!(*x, (k + 1) as f64 / 8.0);
    }
}

#[test]
fn bias_first_size_recovers_ratio_with_closure() {
    for (n, r) in [(2u32, 3.0), (10, 1.25), (10, 0.8), (50, 1.1), (200, 1.03), (6, 1.0), (12, 7.0)] {
        let alpha = bias_first_fraction(n, r).unwrap();
        let got = bias_ratio_from_first(n, alpha).unwrap();
        assert_close(got, r, 1e-12, &format!("n={n}"));
        // Closure: the rebuilt series has the requested first edge and sums to 1.
        let e = edges(&bias_fractions(n, got).unwrap());
        assert_close(e[0], alpha, 1e-12, "first edge");
        assert_close(e.iter().sum::<f64>(), 1.0, 1e-15, "sum");
    }
}

#[test]
fn bias_first_size_equal_spacing_gives_ratio_one() {
    let r = bias_ratio_from_first(8, 1.0 / 8.0).unwrap();
    assert!((r - 1.0).abs() < 1e-12, "{r}");
}

#[test]
fn bias_first_size_errors_are_typed() {
    assert!(matches!(bias_ratio_from_first(5, 1.0), Err(SpacingError::Unreachable(_))));
    assert!(matches!(bias_ratio_from_first(5, 1.5), Err(SpacingError::Unreachable(_))));
    assert!(matches!(bias_ratio_from_first(5, 0.0), Err(SpacingError::BadValue { .. })));
    assert!(matches!(bias_ratio_from_first(5, f64::NAN), Err(SpacingError::BadValue { .. })));
    assert!(matches!(bias_ratio_from_first(0, 0.1), Err(SpacingError::ZeroCount)));
    assert!(matches!(bias_fractions(5, -1.0), Err(SpacingError::BadValue { .. })));
    assert!(matches!(bias_fractions(5, f64::INFINITY), Err(SpacingError::BadValue { .. })));
    // One interval has no interior node and no ratio.
    assert_eq!(bias_ratio_from_first(1, 0.9).unwrap(), 1.0);
    assert!(bias_fractions(1, 3.0).unwrap().is_empty());
}

#[test]
fn bias_extreme_ratio_is_an_error_not_a_panic() {
    assert!(matches!(bias_fractions(200, 1e12), Err(SpacingError::Degenerate(_))));
    assert!(bias_fractions(200, 1e-12).is_err());
}

#[test]
fn bias_count_from_ratio_matches_series_sum() {
    // L = 1, a = 0.1, r = 1.2: n = ln(1 + 10 * 0.2) / ln 1.2 = 6.025.
    assert_eq!(bias_count_from_ratio(1.0, 0.1, 1.2).unwrap(), 6);
    // r = 1: L / a.
    assert_eq!(bias_count_from_ratio(1.0, 0.25, 1.0).unwrap(), 4);
    assert_eq!(bias_count_from_ratio(1.0, 0.3, 1.0).unwrap(), 3); // 3.33 rounds down
    // Contracting series: the infinite sum a / (1 - r) must exceed L.
    // 0.6 (1 - 0.5^n) / 0.5 = 1 -> n = ln(1 / 6) / ln(0.5) ... = 2.58 -> 3.
    assert_eq!(bias_count_from_ratio(1.0, 0.6, 0.5).unwrap(), 3);
    assert!(matches!(bias_count_from_ratio(1.0, 0.5, 0.5), Err(SpacingError::Unreachable(_))));
    assert!(matches!(bias_count_from_ratio(1.0, 0.3, 0.5), Err(SpacingError::Unreachable(_))));
    // A first size above the length still gives one interval.
    assert_eq!(bias_count_from_ratio(1.0, 2.0, 1.5).unwrap(), 1);
}

fn series_sum(fine: f64, coarse: f64, n: u32) -> f64 {
    if n == 1 {
        return fine;
    }
    let r = (coarse / fine).powf(1.0 / (n as f64 - 1.0));
    if (r - 1.0).abs() < 1e-14 { n as f64 * fine } else { (coarse * r - fine) / (r - 1.0) }
}

#[test]
fn bias_count_from_sizes_minimizes_the_length_mismatch() {
    for (l, fine, coarse) in [(1.0, 0.01, 0.1), (1.0, 0.1, 0.01), (10.0, 0.5, 0.5), (3.0, 0.2, 0.9), (1.0, 0.45, 0.45), (2.0, 0.05, 0.4)] {
        let n = bias_count_from_sizes(l, fine, coarse).unwrap();
        let err = |m: u32| (series_sum(fine, coarse, m) - l).abs();
        assert!(err(n) <= err(n + 1) + 1e-12 && (n == 1 || err(n) <= err(n - 1) + 1e-12), "L={l} fine={fine} coarse={coarse}: n={n}");
    }
    // Equal sizes degenerate to round(L / size).
    assert_eq!(bias_count_from_sizes(10.0, 0.5, 0.5).unwrap(), 20);
}

#[test]
fn dualbias_even_and_odd_are_mirror_symmetric_with_ratio_r() {
    for (n, r) in [(4u32, 1.5), (10, 1.2), (6, 0.7), (50, 1.04), (3, 2.0), (7, 1.3), (11, 0.85), (101, 1.02), (8, 1.0), (9, 1.0)] {
        let f = dualbias_fractions(n, r).unwrap();
        let e = edges(&f);
        let n = n as usize;
        assert_eq!(e.len(), n);
        for k in 0..n {
            assert_close(e[k], e[n - 1 - k], 1e-12, &format!("mirror n={n} r={r} k={k}"));
        }
        let m = n / 2;
        for k in 1..m {
            assert_close(e[k] / e[k - 1], r, 1e-12, &format!("ratio n={n} r={r}"));
        }
        if n % 2 == 1 {
            // The middle edge is a r^m.
            assert_close(e[m] / e[0], r.powi(m as i32), 1e-12, &format!("middle n={n} r={r}"));
        }
        // Total length closes: a = 1 / (2 (r^m - 1)/(r - 1) + [odd] r^m).
        if r != 1.0 {
            let g = (r.powi(m as i32) - 1.0) / (r - 1.0);
            let t = 2.0 * g + if n % 2 == 1 { r.powi(m as i32) } else { 0.0 };
            assert_close(e[0], 1.0 / t, 1e-12, &format!("closure n={n} r={r}"));
        }
        assert_close(e.iter().sum::<f64>(), 1.0, 1e-15, "sum");
    }
}

#[test]
fn dualbias_first_size_recovers_ratio() {
    for (n, r) in [(4u32, 1.5f64), (10, 1.2), (6, 0.7), (3, 2.0), (7, 1.3), (11, 0.85), (60, 1.05)] {
        let m = (n / 2) as i32;
        let g = (r.powi(m) - 1.0) / (r - 1.0);
        let alpha = 1.0 / (2.0 * g + if n % 2 == 1 { r.powi(m) } else { 0.0 });
        let got = dualbias_ratio_from_first(n, alpha).unwrap();
        assert_close(got, r, 1e-11, &format!("n={n}"));
        let e = edges(&dualbias_fractions(n, got).unwrap());
        assert_close(e[0], alpha, 1e-12, "first");
    }
}

#[test]
fn dualbias_first_size_limits() {
    assert!(matches!(dualbias_ratio_from_first(6, 0.5), Err(SpacingError::Unreachable(_))));
    assert!(matches!(dualbias_ratio_from_first(6, 0.7), Err(SpacingError::Unreachable(_))));
    assert_eq!(dualbias_ratio_from_first(2, 0.5).unwrap(), 1.0);
    assert!(matches!(dualbias_ratio_from_first(2, 0.3), Err(SpacingError::Unreachable(_))));
    assert!(matches!(dualbias_ratio_from_first(4, -1.0), Err(SpacingError::BadValue { .. })));
    assert!(dualbias_fractions(1, 3.0).unwrap().is_empty());
    assert_eq!(dualbias_fractions(2, 3.0).unwrap(), vec![0.5]);
}

fn ratio_max(e: &[f64]) -> f64 {
    e.windows(2).map(|w| (w[0] / w[1]).max(w[1] / w[0])).fold(1.0, f64::max)
}

#[test]
fn stretch_pins_first_and_last_and_bounds_the_ratio() {
    let l = 10.0;
    let spec = |first, last, max_ratio| StretchSpec { length: l, first, last, max_ratio };
    for (n, first, last) in [(8u32, 0.5, 2.0), (12, 2.0, 0.5), (20, 0.25, 1.0), (4, 2.0, 3.0), (6, 1.5, 1.5)] {
        let f = stretch_fractions(n, &spec(first, Some(last), 2.0)).unwrap();
        assert_eq!(f.len(), n as usize - 1);
        let e: Vec<f64> = edges(&f).iter().map(|x| x * l).collect();
        assert_close(e[0], first, 1e-12, "first");
        assert_close(e[n as usize - 1], last, 1e-12, "last");
        assert_close(e.iter().sum::<f64>(), l, 1e-12, "sum");
        assert!(ratio_max(&e) <= 2.0 * (1.0 + 1e-9), "n={n}: ratio {}", ratio_max(&e));
        // Smooth: the interior sizes are monotone between the end sizes.
        let inner = &e[1..n as usize - 1];
        if first < last {
            assert!(inner.windows(2).all(|w| w[1] >= w[0] - 1e-12), "{inner:?}");
        } else if first > last {
            assert!(inner.windows(2).all(|w| w[1] <= w[0] + 1e-12), "{inner:?}");
        }
    }
}

#[test]
fn stretch_first_only_is_a_geometric_series_within_max_ratio() {
    let spec = StretchSpec { length: 5.0, first: 0.2, last: None, max_ratio: 1.6 };
    let f = stretch_fractions(10, &spec).unwrap();
    let e: Vec<f64> = edges(&f).iter().map(|x| x * 5.0).collect();
    assert_close(e[0], 0.2, 1e-12, "first");
    let r = e[1] / e[0];
    assert!(r > 1.0 && r <= 1.6);
    for w in e.windows(2) {
        assert_close(w[1] / w[0], r, 1e-12, "constant ratio");
    }
}

#[test]
fn stretch_refuses_instead_of_exceeding_max_ratio() {
    // 10 intervals from 0.001 on a length of 5 needs a ratio far above 2.
    let spec = StretchSpec { length: 5.0, first: 0.001, last: None, max_ratio: 2.0 };
    assert!(matches!(stretch_fractions(10, &spec), Err(SpacingError::RatioLimit { allowed, .. }) if allowed == 2.0));
    // Sizes that leave no room for the interior.
    let spec = StretchSpec { length: 1.0, first: 0.6, last: Some(0.6), max_ratio: 2.0 };
    assert!(matches!(stretch_fractions(5, &spec), Err(SpacingError::Unreachable(_))));
    // End sizes inconsistent with the count make the interior jump.
    let spec = StretchSpec { length: 10.0, first: 0.1, last: Some(0.1), max_ratio: 2.0 };
    assert!(matches!(stretch_fractions(5, &spec), Err(SpacingError::RatioLimit { .. })));
    let spec = StretchSpec { length: 10.0, first: 1.0, last: None, max_ratio: 0.5 };
    assert!(matches!(stretch_fractions(5, &spec), Err(SpacingError::BadValue { .. })));
}

#[test]
fn stretch_small_counts() {
    let spec = StretchSpec { length: 4.0, first: 1.0, last: Some(3.0), max_ratio: 3.0 };
    assert!(stretch_fractions(1, &spec).unwrap().is_empty());
    assert_eq!(stretch_fractions(2, &spec).unwrap(), vec![0.25]);
}

#[test]
fn reversed_mirrors_the_edges() {
    let f = bias_fractions(9, 1.4).unwrap();
    let e = edges(&f);
    let er = edges(&reversed(&f));
    for k in 0..9 {
        assert_close(er[k], e[8 - k], 1e-14, "reversed");
    }
}
