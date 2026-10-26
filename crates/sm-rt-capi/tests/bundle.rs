//! `smrt_trace_bundle` (acceptance 6): bytes equal for any thread count, equal to the Rust
//! `Tracker::trace_bundle` result, and stable under the testkit determinism harness.
#![cfg(feature = "stubs")]

mod common;
use common::*;
use sm_core::{Point3, Vector3};
use sm_geom_api::Ray;
use sm_rt_api::{TraceOptions, TrackBundle};
use sm_rt_capi::bundle::{decode, encode};
use sm_rt_capi::ffi::test_hooks::*;
use sm_rt_capi::ffi::*;
use sm_rt_capi::*;

const N: usize = 100_000;

struct Soa {
    c: [Vec<f64>; 6],
    start: Vec<i64>,
}

fn rays() -> (Soa, Vec<Ray>) {
    let mut rng = Rng::named("bundle-1e5");
    let mut soa = Soa { c: Default::default(), start: vec![-1; N] };
    let mut list = Vec::with_capacity(N);
    for _ in 0..N {
        let o = [(rng.unit() - 0.5) * 19.0, (rng.unit() - 0.5) * 19.0, (rng.unit() - 0.5) * 19.0];
        let d = rng.dir();
        for k in 0..3 {
            soa.c[k].push(o[k]);
            soa.c[3 + k].push(d[k]);
        }
        list.push(Ray { origin: Point3::new(o[0], o[1], o[2]), dir: Vector3::new(d[0], d[1], d[2]) });
    }
    (soa, list)
}

fn c_bundle(t: *const SmrtTracker, soa: &Soa, block: i64, opts: Option<&SmrtTraceOpts>) -> Vec<u64> {
    let r = SmrtRays {
        x: soa.c[0].as_ptr(),
        y: soa.c[1].as_ptr(),
        z: soa.c[2].as_ptr(),
        u: soa.c[3].as_ptr(),
        v: soa.c[4].as_ptr(),
        w: soa.c[5].as_ptr(),
        start_cell: soa.start.as_ptr(),
    };
    let op = opts.map_or(std::ptr::null(), |o| o as *const _);
    let mut len = 0u64;
    // SAFETY: arrays hold N entries; null buffer with cap 0 is the size query.
    let q = unsafe { smrt_trace_bundle(t, N as i64, &r, block, op, std::ptr::null_mut(), 0, &mut len) };
    assert_eq!(q, SMRT_ERR_BAD_ARG, "size query");
    let mut buf = vec![0u64; len as usize];
    let mut len2 = 0u64;
    // SAFETY: `buf` holds `len` words.
    let ok = unsafe { smrt_trace_bundle(t, N as i64, &r, block, op, buf.as_mut_ptr(), len, &mut len2) };
    assert_eq!((ok, len2), (0, len));
    buf
}

fn pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool").install(f)
}

#[test]
fn bundle_bytes_equal_across_threads_blocks_and_the_rust_path() {
    let arc = tracker_for(STUB_SHELLS, 5.0).unwrap();
    let t = tracker_into_raw(arc.clone());
    let tp = t as usize;
    let (soa, list) = rays();
    let reference: TrackBundle = arc.trace_bundle(&list, &TraceOptions::default()).unwrap();
    assert!(reference.cell.len() > 20_000, "the rays should hit the shells: {}", reference.cell.len());
    let want = reference.content_hash();

    let one = pool(1, || c_bundle(tp as *const SmrtTracker, &soa, 256, None));
    let many = pool(4, || c_bundle(tp as *const SmrtTracker, &soa, 256, None));
    assert_eq!(one, many, "1 thread vs 4 threads");
    let d = decode(&one).unwrap();
    assert_eq!(d.content_hash(), want, "equals the Rust trace_bundle content hash");
    assert_eq!(d, reference);
    assert_eq!(one, encode(&reference));
    // The block size changes the partition, not the result; 0 uses the tracker's own partition.
    for block in [0, 1, 777, 1 << 20] {
        assert_eq!(pool(3, || c_bundle(tp as *const SmrtTracker, &soa, block, None)), one, "block {block}");
    }
    // Options: a distance cap changes the result and is honoured.
    let o = SmrtTraceOpts { max_dist: 2.0, max_steps: 0, stop_at_deck: 0 };
    let capped = decode(&c_bundle(t, &soa, 256, Some(&o))).unwrap();
    let rust_capped = arc.trace_bundle(&list, &TraceOptions::default().with_max_dist(2.0)).unwrap();
    assert_eq!(capped, rust_capped);
    assert!(capped.length.iter().all(|&l| l <= 2.0));
    // SAFETY: freed once.
    unsafe { smrt_tracker_free(t) };
}

#[test]
fn bundle_handle_equals_the_one_shot_words() {
    let t = tracker_into_raw(tracker_for(STUB_SHELLS, 5.0).unwrap());
    let (soa, _) = rays();
    let want = c_bundle(t, &soa, 256, None);
    let r = SmrtRays {
        x: soa.c[0].as_ptr(),
        y: soa.c[1].as_ptr(),
        z: soa.c[2].as_ptr(),
        u: soa.c[3].as_ptr(),
        v: soa.c[4].as_ptr(),
        w: soa.c[5].as_ptr(),
        start_cell: std::ptr::null(),
    };
    let mut h: *mut SmrtBundle = std::ptr::null_mut();
    let (mut len, mut len2) = (0u64, 0u64);
    let mut words: *const u64 = std::ptr::null();
    // SAFETY: arrays hold N entries; out slots are valid; the handle is freed once.
    unsafe {
        assert_eq!(smrt_trace_bundle_new(t, N as i64, &r, 256, std::ptr::null(), &mut h, &mut len), 0);
        assert!(!h.is_null());
        assert_eq!(smrt_bundle_words(h, &mut words, &mut len2), 0);
        assert_eq!((len, len2), (want.len() as u64, want.len() as u64));
        assert_eq!(std::slice::from_raw_parts(words, want.len()), &want[..]);
        assert_eq!(smrt_bundle_words(std::ptr::null(), &mut words, &mut len2), SMRT_ERR_NULL);
        assert_eq!(smrt_trace_bundle_new(t, -1, &r, 0, std::ptr::null(), &mut h, &mut len), SMRT_ERR_BAD_ARG);
        assert_eq!(smrt_bundle_free(h), 0);
        assert_eq!(smrt_bundle_free(std::ptr::null_mut()), SMRT_ERR_NULL);
        smrt_tracker_free(t);
    }
}

#[test]
fn bundle_passes_the_testkit_determinism_check() {
    let arc = tracker_for(STUB_SHELLS, 5.0).unwrap();
    let t = tracker_into_raw(arc);
    let (soa, _) = rays();
    let tp = t as usize;
    let soa = &soa;
    let out = sm_testkit::determinism::check(2, 4, |threads| pool(threads, || c_bundle(tp as *const SmrtTracker, soa, 256, None))).unwrap();
    assert!(out.len() > 100);
    // SAFETY: freed once.
    unsafe { smrt_tracker_free(t) };
}

#[test]
fn bundle_argument_errors() {
    let t = sphere5();
    let (soa, _) = rays();
    let r = SmrtRays { x: soa.c[0].as_ptr(), y: soa.c[1].as_ptr(), z: std::ptr::null(), u: soa.c[3].as_ptr(), v: soa.c[4].as_ptr(), w: soa.c[5].as_ptr(), start_cell: std::ptr::null() };
    let mut buf = [0u64; 32];
    let mut len = 0u64;
    // SAFETY: valid pointers; a null column is reported, not read.
    unsafe {
        assert_eq!(smrt_trace_bundle(t, 10, &r, 0, std::ptr::null(), buf.as_mut_ptr(), 32, &mut len), SMRT_ERR_NULL);
        assert_eq!(smrt_trace_bundle(t, -1, &r, 0, std::ptr::null(), buf.as_mut_ptr(), 32, &mut len), SMRT_ERR_BAD_ARG);
        // Zero rays is a valid, empty bundle.
        assert_eq!(smrt_trace_bundle(t, 0, &SmrtRays { z: soa.c[2].as_ptr(), ..r }, 0, std::ptr::null(), buf.as_mut_ptr(), 32, &mut len), 0);
    }
    assert_eq!(decode(&buf[..len as usize]).unwrap().ray_count(), 0);
}
