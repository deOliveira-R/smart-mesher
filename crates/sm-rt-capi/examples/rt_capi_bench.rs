//! Sanity and overhead benchmark of the C ABI over the contract's analytic stub (record-only).
//!
//! The stub is brute force and exact, so the numbers are NOT a competitor number: they bound the
//! cost of the ABI layer (handle deref, `catch_unwind`, struct conversion) against the same
//! `Tracker` called directly, on the rays of a benchmark case (the geometry of C1/C2/C3 replaced
//! by analytic spheres). Usage:
//!
//! ```text
//! cargo run --release -p sm-rt-capi --features stubs --example rt_capi_bench -- \
//!     --case C1 [--rays bench/data/C1/rays.bin] [--n 200000] [--threads 4]
//! ```
//!
//! Method: 3 warm-up and 7 measured repetitions; median with min and max. One JSON object on stdout.

use sm_core::{Aabb, Point3, Vector3};
use sm_rt_api::stub::{AnalyticWorld, Shape};
use sm_rt_api::{ResponseKind, Scratch, StepEnd, Tracker};
use sm_rt_capi::ffi::*;
use sm_rt_capi::*;
use std::sync::Arc;
use std::time::Instant;

const WARMUP: usize = 3;
const REPS: usize = 7;

struct Rays {
    o: Vec<[f64; 3]>,
    d: Vec<[f64; 3]>,
}

fn fnv1a64(s: &str) -> u64 {
    s.bytes().fold(0xcbf2_9ce4_8422_2325u64, |h, b| (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3))
}

struct SplitMix(u64);

impl SplitMix {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 * (1.0 / 9_007_199_254_740_992.0)
    }
    fn direction(&mut self) -> [f64; 3] {
        loop {
            let v = [2.0 * self.unit() - 1.0, 2.0 * self.unit() - 1.0, 2.0 * self.unit() - 1.0];
            let n2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            if n2 > 1e-6 && n2 <= 1.0 {
                let n = n2.sqrt();
                return [v[0] / n, v[1] / n, v[2] / n];
            }
        }
    }
}

/// The generator of `bench/gen_case.py`, for runs without a rays file (start regions are
/// recomputed by the tracker, so only origins and directions matter).
fn generated(case: &str, n: usize, radii: &[f64]) -> Rays {
    let mut rng = SplitMix(fnv1a64(case));
    let (mut o, mut d) = (Vec::with_capacity(n), Vec::with_capacity(n));
    while o.len() < n {
        let p = [(2.0 * rng.unit() - 1.0) * 2.0, (2.0 * rng.unit() - 1.0) * 2.0, (2.0 * rng.unit() - 1.0) * 2.0];
        let dir = rng.direction();
        let r = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
        if case == "C3" && r <= radii[0] {
            continue;
        }
        o.push(p);
        d.push(dir);
    }
    Rays { o, d }
}

/// Parse "smrays v1": `u32 n`, `n * 6 f64` (origin, unit direction), `n i32` start region.
fn read_rays(path: &str) -> Rays {
    let b = std::fs::read(path).unwrap_or_else(|e| panic!("{path}: {e}"));
    let n = u32::from_le_bytes(b[..4].try_into().expect("n")) as usize;
    assert!(b.len() == 4 + n * 48 + n * 4, "{path}: size does not match n = {n}");
    let f = |i: usize| f64::from_le_bytes(b[4 + 8 * i..12 + 8 * i].try_into().expect("f64"));
    Rays { o: (0..n).map(|i| [f(6 * i), f(6 * i + 1), f(6 * i + 2)]).collect(), d: (0..n).map(|i| [f(6 * i + 3), f(6 * i + 4), f(6 * i + 5)]).collect() }
}

fn world(radii: &[f64]) -> Arc<dyn Tracker> {
    let shapes = radii.iter().map(|&r| Shape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: r }).collect();
    let domain = Aabb { lo: Point3::new(-2.0, -2.0, -2.0), hi: Point3::new(2.0, 2.0, 2.0) };
    Arc::new(AnalyticWorld::simple(shapes, domain, ResponseKind::Vacuum).expect("world"))
}

/// Crossings and lost particles for rays `lo..hi`, directly on the Rust trait.
fn rust_chunk(t: &dyn Tracker, r: &Rays, lo: usize, hi: usize) -> (u64, u64) {
    let (mut cross, mut lost) = (0u64, 0u64);
    let mut sc = Scratch::new();
    for i in lo..hi {
        let Ok(mut st) = t.init_state(Point3::new(r.o[i][0], r.o[i][1], r.o[i][2]), Vector3::new(r.d[i][0], r.d[i][1], r.d[i][2]), None, &mut sc) else {
            lost += 1;
            continue;
        };
        loop {
            match t.fire(&mut st, f64::INFINITY, 1_000_000, &mut sc) {
                Ok(f) => match f.end {
                    StepEnd::RegionBoundary { .. } => cross += 1,
                    StepEnd::Boundary { .. } => {
                        cross += 1;
                        break;
                    }
                    _ => {
                        lost += 1;
                        break;
                    }
                },
                Err(_) => {
                    lost += 1;
                    break;
                }
            }
        }
    }
    (cross, lost)
}

/// The same loop through the C ABI.
fn capi_chunk(h: *const SmrtTracker, r: &Rays, lo: usize, hi: usize) -> (u64, u64) {
    let (mut cross, mut lost) = (0u64, 0u64);
    let mut c: *mut SmrtCtx = std::ptr::null_mut();
    // SAFETY: `h` is a live handle (kept alive by the caller); `c` is a valid out slot; the context
    // is used by this thread only and freed below.
    unsafe {
        assert_eq!(smrt_ctx_new(h, &mut c), 0);
        let mut s = SmrtStep::default();
        for i in lo..hi {
            let (o, d) = (r.o[i], r.d[i]);
            if smrt_init_state(c, o[0], o[1], o[2], d[0], d[1], d[2], -1) < 0 {
                lost += 1;
                continue;
            }
            loop {
                if smrt_fire(c, f64::INFINITY, 0, &mut s) < 0 {
                    lost += 1;
                    break;
                }
                match s.end {
                    SMRT_END_REGION_BOUNDARY => cross += 1,
                    SMRT_END_BOUNDARY => {
                        cross += 1;
                        break;
                    }
                    _ => {
                        lost += 1;
                        break;
                    }
                }
            }
        }
        smrt_ctx_free(c);
    }
    (cross, lost)
}

fn parallel(threads: usize, n: usize, f: impl Fn(usize, usize) -> (u64, u64) + Sync) -> (u64, u64) {
    use rayon::prelude::*;
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
    let chunk = 4096;
    pool.install(|| {
        (0..n.div_ceil(chunk)).into_par_iter().map(|k| f(k * chunk, ((k + 1) * chunk).min(n))).reduce(|| (0, 0), |a, b| (a.0 + b.0, a.1 + b.1))
    })
}

fn stats(mut v: Vec<f64>) -> (f64, f64, f64) {
    v.sort_by(|a, b| a.partial_cmp(b).expect("finite"));
    (v[v.len() / 2], v[0], v[v.len() - 1])
}

fn measure(mut f: impl FnMut() -> f64) -> (f64, f64, f64) {
    for _ in 0..WARMUP {
        f();
    }
    stats((0..REPS).map(|_| f()).collect())
}

/// Rays per second of one run of `f`.
fn rate<'a>(n: usize, f: &'a dyn Fn() -> (u64, u64)) -> impl FnMut() -> f64 + 'a {
    move || {
        let t = Instant::now();
        let _ = f();
        n as f64 / t.elapsed().as_secs_f64()
    }
}

fn cpu_model() -> String {
    if let Ok(s) = std::fs::read_to_string("/proc/cpuinfo") {
        for key in ["model name", "Model", "Hardware", "cpu model"] {
            if let Some((_, v)) = s.lines().find(|l| l.starts_with(key)).and_then(|l| l.split_once(':')) {
                return v.trim().to_string();
            }
        }
    }
    if let Ok(o) = std::process::Command::new("lscpu").output() {
        for l in String::from_utf8_lossy(&o.stdout).lines() {
            if let Some(v) = l.strip_prefix("Model name:") {
                return v.trim().to_string();
            }
        }
    }
    format!("unknown ({})", std::env::consts::ARCH)
}

fn arg(args: &[String], key: &str) -> Option<String> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1)).cloned()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let case = arg(&args, "--case").unwrap_or_else(|| "C1".into());
    let threads: usize = arg(&args, "--threads").and_then(|s| s.parse().ok()).unwrap_or(1);
    let n: usize = arg(&args, "--n").and_then(|s| s.parse().ok()).unwrap_or(200_000);
    let radii: Vec<f64> = match case.as_str() {
        "C2" => vec![0.2, 0.4, 0.6, 0.8, 1.0],
        _ => vec![1.0],
    };
    let rays = arg(&args, "--rays").map_or_else(|| generated(&case, n, &radii), |p| read_rays(&p));
    let n = rays.o.len();

    let t0 = Instant::now();
    let arc = world(&radii);
    let build_s = t0.elapsed().as_secs_f64();
    let h = tracker_into_raw(arc.clone());
    let hp = h as usize;

    let (c_rust, l_rust) = parallel(threads, n, |lo, hi| rust_chunk(&*arc, &rays, lo, hi));
    let (c_capi, l_capi) = parallel(threads, n, |lo, hi| capi_chunk(hp as *const SmrtTracker, &rays, lo, hi));
    assert_eq!((c_rust, l_rust), (c_capi, l_capi), "the ABI must see the same crossings as the trait");

    let rust_f = || parallel(threads, n, |lo, hi| rust_chunk(&*arc, &rays, lo, hi));
    let capi_f = || parallel(threads, n, |lo, hi| capi_chunk(hp as *const SmrtTracker, &rays, lo, hi));
    let (rr, rr_min, rr_max) = measure(rate(n, &rust_f));
    let (cr, cr_min, cr_max) = measure(rate(n, &capi_f));

    // Per-step overhead: one particle bouncing in a reflecting box would hide the geometry, so use
    // the same rays and time `step` to leakage, direct vs ABI, on one thread.
    let step_ns = |through_abi: bool| {
        let f = || {
            let mut steps = 0u64;
            let mut sc = Scratch::new();
            let mut c: *mut SmrtCtx = std::ptr::null_mut();
            // SAFETY: as in `capi_chunk`.
            unsafe { smrt_ctx_new(h, &mut c) };
            let mut s = SmrtStep::default();
            let t = Instant::now();
            for i in 0..n.min(50_000) {
                let (o, d) = (rays.o[i], rays.d[i]);
                if through_abi {
                    // SAFETY: live ctx, valid out slot, this thread only.
                    unsafe {
                        if smrt_init_state(c, o[0], o[1], o[2], d[0], d[1], d[2], -1) < 0 {
                            continue;
                        }
                        loop {
                            steps += 1;
                            if smrt_step(c, f64::INFINITY, &mut s) < 0 || s.end == SMRT_END_BOUNDARY || s.end == SMRT_END_LOST {
                                break;
                            }
                        }
                    }
                } else if let Ok(mut st) = arc.init_state(Point3::new(o[0], o[1], o[2]), Vector3::new(d[0], d[1], d[2]), None, &mut sc) {
                    loop {
                        steps += 1;
                        match arc.step(&mut st, f64::INFINITY, &mut sc) {
                            Ok(s) if !matches!(s.end, StepEnd::Boundary { .. } | StepEnd::Lost(_)) => {}
                            _ => break,
                        }
                    }
                }
            }
            let ns = t.elapsed().as_secs_f64() * 1e9 / steps.max(1) as f64;
            // SAFETY: freed once.
            unsafe { smrt_ctx_free(c) };
            ns
        };
        f()
    };
    // Interleave the two measurements so machine drift hits both; the overhead is the median of
    // the per-repetition differences.
    for _ in 0..WARMUP {
        step_ns(false);
        step_ns(true);
    }
    let (mut dv, mut av, mut ov) = (vec![], vec![], vec![]);
    for _ in 0..REPS {
        let (d, a) = (step_ns(false), step_ns(true));
        ov.push(a - d);
        dv.push(d);
        av.push(a);
    }
    let ((direct, d_min, d_max), (abi, a_min, a_max), (over, o_min, o_max)) = (stats(dv), stats(av), stats(ov));

    println!(
        "{{\"tool\":\"rt_capi_bench\",\"case\":\"{case}\",\"backend\":\"stub AnalyticWorld (analytic; not a competitor number)\",\"rays\":{n},\"threads\":{threads},\"cpu_model\":\"{}\",\
\"build_s\":{build_s:.6},\"lost\":{l_capi},\"crossings\":{c_capi},\
\"rays_per_s_rust\":{{\"median\":{rr:.1},\"min\":{rr_min:.1},\"max\":{rr_max:.1}}},\
\"rays_per_s_capi\":{{\"median\":{cr:.1},\"min\":{cr_min:.1},\"max\":{cr_max:.1}}},\
\"crossings_per_s_capi\":{:.1},\
\"step_ns_direct\":{{\"median\":{direct:.2},\"min\":{d_min:.2},\"max\":{d_max:.2}}},\
\"step_ns_capi\":{{\"median\":{abi:.2},\"min\":{a_min:.2},\"max\":{a_max:.2}}},\
\"step_overhead_ns\":{{\"median\":{over:.2},\"min\":{o_min:.2},\"max\":{o_max:.2}}},\"warmup\":{WARMUP},\"reps\":{REPS}}}",
        cpu_model().replace('"', "'"),
        cr * c_capi as f64 / n as f64
    );
    // SAFETY: freed once, after the last context.
    unsafe { smrt_tracker_free(h) };
}
