//! Compensated and deterministic parallel summation.

mod common;
use common::Rng;
use sm_core::Vector3;
use sm_math::exact::Expansion;
use sm_math::sum::*;
use sm_testkit::determinism::hash_f64_bits;

/// Exact sum rounded to nearest-ish (components added small to large): the oracle.
fn exact(xs: &[f64]) -> f64 {
    let mut e = Expansion::zero();
    for &x in xs {
        e = e.add(&Expansion::from_f64(x));
    }
    e.estimate()
}

fn data(n: usize, seed: u64) -> Vec<f64> {
    let mut r = Rng(seed);
    (0..n)
        .map(|_| (r.next() as f64 / u64::MAX as f64 - 0.5) * 2f64.powi(r.int(30) as i32))
        .collect()
}

#[test]
fn neumaier_handles_classic_cancellation() {
    assert_eq!(sum_neumaier(&[1.0, 1e100, 1.0, -1e100]), 2.0);
    assert_eq!(sum_neumaier(&[1e100, 1.0, -1e100]), 1.0);
    // Naive and Kahan fail on this one (the big term arrives after the small ones).
    let xs = [1.0, 1e100, 1.0, -1e100];
    assert_ne!(xs.iter().sum::<f64>(), 2.0);
    // 0.1 ten million times: naive drifts, compensated is exact to the last bit of the sum.
    let n = 10_000_000usize;
    let mut k = KahanSum::new();
    let mut m = NeumaierSum::new();
    let mut naive = 0.0;
    for _ in 0..n {
        k.add(0.1);
        m.add(0.1);
        naive += 0.1;
    }
    let truth = 0.1 * n as f64; // the double nearest 0.1, times 1e7, correctly rounded
    assert!((m.value() - truth).abs() <= 2.0 * f64::EPSILON * truth);
    assert!((k.value() - truth).abs() <= 2.0 * f64::EPSILON * truth);
    assert!((naive - truth).abs() > 1e3 * (m.value() - truth).abs().max(f64::EPSILON * truth));
}

#[test]
fn compensated_sums_are_nearly_exact_on_random_data() {
    for seed in 0..20 {
        let xs = data(20_000, seed);
        let ex = exact(&xs);
        let abs: f64 = xs.iter().map(|x| x.abs()).sum();
        let neu = sum_neumaier(&xs);
        let naive: f64 = xs.iter().sum();
        assert!(
            (neu - ex).abs() <= 4.0 * f64::EPSILON * ex.abs() + 1e-26 * abs,
            "seed {seed}: {neu} vs {ex}"
        );
        assert!((neu - ex).abs() <= (naive - ex).abs() + f64::EPSILON * ex.abs());
    }
}

#[test]
fn parallel_sum_is_bit_identical_across_thread_counts() {
    let xs = data(300_000, 42);
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(threads)
            .build()
            .unwrap();
        pool.install(|| {
            let v = par_sum_vec3_by(xs.len(), 1000, |i| Vector3::new(xs[i], -xs[i], xs[i] * 0.5));
            vec![
                par_sum(&xs),
                par_sum_chunked(&xs, 17),
                par_dot(&xs, &xs),
                par_sum_by(xs.len(), 333, |i| xs[i] * 3.0),
                v.x,
                v.y,
                v.z,
            ]
        })
    };
    let base = run(1);
    for threads in [2, 3, 4, 8] {
        assert_eq!(
            hash_f64_bits(&run(threads)),
            hash_f64_bits(&base),
            "threads = {threads}"
        );
    }
    // The parallel result is accurate too.
    let ex = exact(&xs);
    assert!((base[0] - ex).abs() <= 4.0 * f64::EPSILON * ex.abs() + 1e-20);
    // Same chunk length => same bits as the sequential chunked reference.
    let mut total = NeumaierSum::new();
    for c in xs.chunks(17) {
        total.merge(&c.iter().copied().collect::<NeumaierSum>());
    }
    assert_eq!(total.value().to_bits(), base[1].to_bits());
}

#[test]
fn edge_sizes() {
    assert_eq!(par_sum(&[]), 0.0);
    assert_eq!(par_sum(&[2.5]), 2.5);
    assert_eq!(par_sum_chunked(&[1.0, 2.0, 3.0], 0), 6.0); // chunk 0 is treated as 1
    assert_eq!(par_sum_by(5, 100, |i| i as f64), 10.0);
    let v = par_sum_vec3_by(0, 8, |_| Vector3::new(1.0, 1.0, 1.0));
    assert_eq!((v.x, v.y, v.z), (0.0, 0.0, 0.0));
}

#[test]
fn neumaier_propagates_infinity_like_kahan() {
    use sm_math::sum::sum_neumaier;
    assert_eq!(sum_neumaier(&[1.0, f64::INFINITY]), f64::INFINITY);
    assert_eq!(sum_neumaier(&[f64::INFINITY, 1.0]), f64::INFINITY);
    assert_eq!(
        sum_neumaier(&[f64::NEG_INFINITY, 1.0, 2.0]),
        f64::NEG_INFINITY
    );
    assert!(sum_neumaier(&[f64::INFINITY, f64::NEG_INFINITY]).is_nan());
}
