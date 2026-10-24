//! Determinism: the same bits twice, with 1 thread and with N rayon threads, and under
//! concurrent callers.

mod common;
use common::Rng;
use rayon::prelude::*;
use sm_core::{Point3, Vector3};
use sm_math::interval::Interval;
use sm_math::linalg::{solve3, svd3, sym3_eigen};
use sm_math::roots::{RootOptions, brent};
use sm_math::sos::{Tagged, incircle_sos, insphere_sos};
use sm_math::sum::{par_sum, par_sum_by, par_sum_vec3_by};
use sm_math::{incircle, insphere, orient2d, orient3d};
use sm_testkit::determinism::{check, hash_f64_bits};

/// Every public numeric entry point, over a rayon pool of `threads` workers, as one hash.
fn workload(threads: usize) -> u64 {
    let pool = rayon::ThreadPoolBuilder::new()
        .num_threads(threads)
        .build()
        .expect("pool");
    pool.install(|| {
        let n = 3_000;
        // Predicates over lattice (degenerate) and random inputs, evaluated in parallel.
        let signs: Vec<f64> = (0..n)
            .into_par_iter()
            .map(|i| {
                let mut r = Rng(i as u64);
                let g = |r: &mut Rng| f64::from(r.int(2) as i32);
                let p = |r: &mut Rng| Point3::new(g(r), g(r), g(r));
                let (a, b, c, d, e) = (p(&mut r), p(&mut r), p(&mut r), p(&mut r), p(&mut r));
                let s = f64::from(orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]).as_i32())
                    + 3.0 * f64::from(orient3d(a, b, c, d).as_i32())
                    + 9.0
                        * f64::from(
                            incircle([a.x, a.y], [b.x, b.y], [c.x, c.y], [d.x, d.y]).as_i32(),
                        )
                    + 27.0 * f64::from(insphere(a, b, c, d, e).as_i32());
                let ids = [i as u64, 7, 3, 11, 5];
                let t = |q: Point3, k: usize| Tagged::new(q, ids[k] + 100 * k as u64);
                let sos = insphere_sos(t(a, 0), t(b, 1), t(c, 2), t(d, 3), t(e, 4))
                    .map_or(9.0, |s| f64::from(s.as_i32()));
                let t2 = |q: Point3, k: usize| Tagged::new([q.x, q.y], ids[k] + 100 * k as u64);
                let sos2 = incircle_sos(t2(a, 0), t2(b, 1), t2(c, 2), t2(d, 3))
                    .map_or(9.0, |s| f64::from(s.as_i32()));
                s + 81.0 * sos + 243.0 * sos2
            })
            .collect();
        let mut out = signs;
        // Reductions.
        let xs: Vec<f64> = (0..50_000)
            .map(|i| ((i * 7919) % 10007) as f64 * 1e-3 - 5.0 + 1e9 * f64::from(i % 3 == 0))
            .collect();
        out.push(par_sum(&xs));
        out.push(par_sum_by(xs.len(), 128, |i| xs[i] * xs[i]));
        let v = par_sum_vec3_by(xs.len(), 1000, |i| Vector3::new(xs[i], 1.0, -xs[i]));
        out.extend([v.x, v.y, v.z]);
        // Serial numerics.
        let a = [[4.0, 1.0, 2.0], [1.0, 3.0, 0.5], [2.0, 0.5, 5.0]];
        let e = sym3_eigen(&a).expect("eigen");
        out.extend(e.values);
        out.extend(e.vectors.iter().flatten());
        let s = svd3(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]]).expect("svd");
        out.extend(s.s);
        out.extend(s.u.iter().flatten().chain(s.v.iter().flatten()));
        let x = solve3(&a, Vector3::new(1.0, 2.0, 3.0)).expect("solve");
        out.extend([x.x, x.y, x.z]);
        let r = brent(|x| x.cos() - x, 0.0, 1.0, RootOptions::default()).expect("root");
        out.extend([r.x, r.fx]);
        let iv = (Interval::new(-1.5, 2.5).expect("iv").powi(3) + Interval::point(0.1))
            / Interval::new(1.0, 3.0).expect("iv");
        out.extend([iv.lo(), iv.hi()]);
        hash_f64_bits(&out)
    })
}

#[test]
fn bit_identical_across_runs_threads_and_concurrent_callers() {
    let h = check(2, 4, workload).expect("determinism");
    assert_eq!(h, workload(1));
    // Also across a larger pool.
    assert_eq!(workload(7), h);
}
