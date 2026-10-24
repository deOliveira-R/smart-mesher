//! Summary statistics and histograms against closed forms and naive references.
#![allow(missing_docs)]

use sm_quality::{CHUNK, histogram, histogram_auto, summarize};

fn pool<R: Send>(threads: usize, f: impl FnOnce() -> R + Send) -> R {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn arithmetic_sequence() {
    // 1..=n: mean (n+1)/2, population variance (n^2 - 1)/12.
    let n = 3 * CHUNK + 17;
    let v: Vec<f64> = (1..=n).map(|i| i as f64).collect();
    let s = summarize(&v);
    assert_eq!(s.count, n);
    assert_eq!((s.min, s.max), (1.0, n as f64));
    assert_eq!((s.argmin, s.argmax), (Some(0), Some(n - 1)));
    assert!((s.mean - (n as f64 + 1.0) / 2.0).abs() < 1e-9);
    assert!((s.std_dev - (((n * n - 1) as f64) / 12.0).sqrt()).abs() < 1e-8);
}

#[test]
fn non_finite_values_are_counted_and_excluded() {
    let v = [1.0, f64::NAN, 3.0, f64::INFINITY, f64::NEG_INFINITY];
    let s = summarize(&v);
    assert_eq!((s.count, s.undefined), (2, 3));
    assert_eq!((s.min, s.max, s.mean), (1.0, 3.0, 2.0));
    assert!((s.std_dev - 1.0).abs() < 1e-15);
    let e = summarize(&[f64::NAN]);
    assert_eq!(e.count, 0);
    assert!(e.mean.is_nan() && e.argmin.is_none());
    assert_eq!(summarize(&[]).count, 0);
}

#[test]
fn ties_resolve_to_the_lowest_index_across_chunks() {
    let mut v = vec![5.0; 2 * CHUNK + 3];
    v[CHUNK + 1] = 1.0;
    v[2 * CHUNK + 2] = 1.0;
    v[7] = 9.0;
    v[CHUNK + 5] = 9.0;
    let s = summarize(&v);
    assert_eq!((s.argmin, s.argmax), (Some(CHUNK + 1), Some(7)));
}

#[test]
fn signed_zero_is_ordered_deterministically() {
    // total order puts -0.0 below +0.0, whatever the position or thread count.
    for threads in [1, 3] {
        let a = pool(threads, || summarize(&[0.0, -0.0, 0.0]));
        assert_eq!(a.min.to_bits(), (-0.0f64).to_bits());
        assert_eq!(a.argmin, Some(1));
        assert_eq!(a.max.to_bits(), 0.0f64.to_bits());
        assert_eq!(a.argmax, Some(0));
    }
}

#[test]
fn bit_identical_across_thread_counts_on_awkward_data() {
    // Wide dynamic range, where a naive parallel sum would differ between schedules.
    let v: Vec<f64> = (0..50_000i32).map(|i| ((i * 7919) % 1009) as f64 * 10f64.powi(i % 13 - 6) * if i % 3 == 0 { -1.0 } else { 1.0 }).collect();
    let bits = |s: sm_quality::Summary| (s.count, s.min.to_bits(), s.max.to_bits(), s.mean.to_bits(), s.std_dev.to_bits(), s.argmin, s.argmax);
    let base = bits(pool(1, || summarize(&v)));
    for t in [2, 3, 8] {
        assert_eq!(base, bits(pool(t, || summarize(&v))), "{t} threads");
    }
}

#[test]
fn histogram_counts() {
    let v = [0.0, 0.1, 0.5, 0.99, 1.0, 1.0, -1.0, 2.0, f64::NAN];
    let h = histogram(&v, 0.0, 1.0, 4).unwrap();
    // bins [0,.25) [.25,.5) [.5,.75) [.75,1]
    assert_eq!(h.counts, vec![2, 0, 1, 3]);
    assert_eq!((h.below, h.above, h.undefined), (1, 1, 1));
    assert_eq!(h.total(), 8);
    assert_eq!(h.edge(2), 0.5);
    assert_eq!(h.edge(4), 1.0);
    assert!(histogram(&v, 1.0, 1.0, 4).is_err());
    assert!(histogram(&v, 0.0, 1.0, 0).is_err());
    let a = histogram_auto(&[2.0, 2.0], 3).unwrap();
    assert_eq!(a.counts.iter().sum::<u64>(), 2);
    assert!(histogram_auto(&[f64::NAN], 3).is_err());
}

#[test]
fn histogram_matches_a_naive_count_for_any_thread_count() {
    let v: Vec<f64> = (0..3 * CHUNK + 5).map(|i| ((i * 31) % 997) as f64 / 997.0).collect();
    let mut want = vec![0u64; 10];
    for &x in &v {
        want[((x * 10.0) as usize).min(9)] += 1;
    }
    for t in [1, 4] {
        assert_eq!(pool(t, || histogram(&v, 0.0, 1.0, 10).unwrap()).counts, want);
    }
}
