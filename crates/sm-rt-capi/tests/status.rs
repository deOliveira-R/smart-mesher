//! Status codes, null handling, panic containment and the thread-local error message
//! (acceptance 4).
#![cfg(feature = "stubs")]

mod common;
use common::*;
use sm_geom_api::GeomError;
use sm_rt_api::RtError;
use sm_rt_capi::ffi::test_hooks::*;
use sm_rt_capi::ffi::*;
use sm_rt_capi::*;
use std::ptr::{null, null_mut};
use std::sync::{Arc, Barrier};

#[test]
fn every_rt_error_maps_to_its_distinct_negative_code() {
    let all = [
        (RtError::Build(vec![]), SMRT_ERR_BUILD),
        (RtError::BadDirection { norm: 0.0 }, SMRT_ERR_BAD_DIRECTION),
        (RtError::NonFinite("x"), SMRT_ERR_NON_FINITE),
        (RtError::UnknownId { kind: "cell", id: 0 }, SMRT_ERR_UNKNOWN_ID),
        (RtError::Unsupported("x"), SMRT_ERR_UNSUPPORTED),
        (RtError::CacheStale("x"), SMRT_ERR_CACHE_STALE),
        (RtError::CacheCorrupt(String::new()), SMRT_ERR_CACHE_CORRUPT),
        (RtError::Geom(GeomError::Invalid("x")), SMRT_ERR_GEOM),
        (RtError::BadAlbedo { value: 2.0 }, SMRT_ERR_BAD_ALBEDO),
        (RtError::BinCountMismatch { left: 1, right: 2 }, SMRT_ERR_BIN_COUNT),
        (RtError::InvalidInput("x"), SMRT_ERR_INVALID_INPUT),
    ];
    let mut seen = Vec::new();
    for (e, want) in all {
        assert_eq!(e.code(), want, "{e}");
        let f = Fail::from(e);
        assert_eq!(f.code(), want);
        assert!(!f.message().is_empty());
        seen.push(want);
    }
    seen.sort_unstable();
    seen.dedup();
    assert_eq!(seen.len(), 11);
    assert!(seen.iter().all(|&c| (-99..0).contains(&c)));
    // The ABI-level codes sit outside the contract's range and are distinct.
    let abi = [SMRT_ERR_NULL, SMRT_ERR_PANIC, SMRT_ERR_BAD_ARG, SMRT_ERR_LOST];
    assert_eq!(abi, [-100, -101, -102, -103]);
}

#[test]
fn every_function_returns_null_status_for_a_null_pointer() {
    let t = sphere5();
    let c = ctx_of(t);
    let (o, d, p) = ([0.0f64; 3], [1.0f64, 0.0, 0.0], [0.0f64; 3]);
    let rays = SmrtRays { x: p.as_ptr(), y: p.as_ptr(), z: p.as_ptr(), u: d.as_ptr(), v: p.as_ptr(), w: p.as_ptr(), start_cell: null() };
    let mut buf = [0u8; 16];
    // Out slots are leaked raw pointers (Copy), so the row closures can share them.
    fn leak<T: Default>() -> *mut T {
        Box::into_raw(Box::default())
    }
    type Row<'a> = (&'a str, usize, Box<dyn Fn(usize) -> i32 + 'a>);
    let tracker_out: *mut *mut SmrtTracker = Box::into_raw(Box::new(null_mut()));
    let ctx_out: *mut *mut SmrtCtx = Box::into_raw(Box::new(null_mut()));
    let (st, stp, loc, cl, inf, rinfo) = (leak::<SmrtState>(), leak::<SmrtStep>(), leak::<SmrtLocated>(), leak::<SmrtClosest>(), leak::<SmrtInfo>(), leak::<SmrtRegionInfo>());
    let (i32out, i64a, f64a, i64b, outlen) = (leak::<i32>(), leak::<i64>(), leak::<f64>(), leak::<i64>(), leak::<u64>());
    let n3 = leak::<[f64; 3]>().cast::<f64>();
    let words = Box::into_raw(Box::new([0u64; 64])).cast::<u64>();
    let (pt, pc) = (t as usize, c as usize);
    let (tp, cp) = (move || pt as *mut SmrtTracker, move || pc as *mut SmrtCtx);
    let pick = |bad: usize, i: usize| bad == i;
    let rows: Vec<Row<'_>> = vec![
        ("smrt_open", 2, Box::new(|b| unsafe { smrt_open(if pick(b, 0) { null() } else { c"x".as_ptr() }, if pick(b, 1) { null_mut() } else { tracker_out }) })),
        ("smrt_info", 2, Box::new(|b| unsafe { smrt_info(if pick(b, 0) { null() } else { tp() }, if pick(b, 1) { null_mut() } else { inf }) })),
        ("smrt_region_info", 2, Box::new(|b| unsafe { smrt_region_info(if pick(b, 0) { null() } else { tp() }, 0, if pick(b, 1) { null_mut() } else { rinfo }) })),
        ("smrt_ctx_new", 2, Box::new(|b| unsafe { smrt_ctx_new(if pick(b, 0) { null() } else { tp() }, if pick(b, 1) { null_mut() } else { ctx_out }) })),
        ("smrt_ctx_free", 1, Box::new(|_| unsafe { smrt_ctx_free(null_mut()) })),
        ("smrt_tracker_free", 1, Box::new(|_| unsafe { smrt_tracker_free(null_mut()) })),
        ("smrt_init_state", 1, Box::new(|_| unsafe { smrt_init_state(null_mut(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, -1) })),
        ("smrt_get_state", 2, Box::new(|b| unsafe { smrt_get_state(if pick(b, 0) { null() } else { cp() }, if pick(b, 1) { null_mut() } else { st }) })),
        ("smrt_set_state", 2, Box::new(|b| unsafe { smrt_set_state(if pick(b, 0) { null_mut() } else { cp() }, if pick(b, 1) { null() } else { st as *const SmrtState }) })),
        ("smrt_set_direction", 1, Box::new(|_| unsafe { smrt_set_direction(null_mut(), 1.0, 0.0, 0.0) })),
        ("smrt_step", 2, Box::new(|b| unsafe { smrt_step(if pick(b, 0) { null_mut() } else { cp() }, 1.0, if pick(b, 1) { null_mut() } else { stp }) })),
        ("smrt_peek", 2, Box::new(|b| unsafe { smrt_peek(if pick(b, 0) { null_mut() } else { cp() }, 1.0, if pick(b, 1) { null_mut() } else { stp }) })),
        ("smrt_fire", 2, Box::new(|b| unsafe { smrt_fire(if pick(b, 0) { null_mut() } else { cp() }, 1.0, 0, if pick(b, 1) { null_mut() } else { stp }) })),
        ("smrt_resolve_boundary", 2, Box::new(|b| unsafe { smrt_resolve_boundary(if pick(b, 0) { null_mut() } else { cp() }, 0, 1, 0.0, 0.5, 0.5, if pick(b, 1) { null_mut() } else { n3 }) })),
        ("smrt_locate", 3, Box::new(|b| unsafe { smrt_locate(if pick(b, 0) { null_mut() } else { cp() }, if pick(b, 1) { null() } else { p.as_ptr() }, null(), -1, if pick(b, 2) { null_mut() } else { loc }) })),
        ("smrt_closest", 3, Box::new(|b| unsafe { smrt_closest(if pick(b, 0) { null_mut() } else { cp() }, if pick(b, 1) { null() } else { p.as_ptr() }, -1, if pick(b, 2) { null_mut() } else { cl }) })),
        ("smrt_dag_ray_fire", 6, Box::new(|b| unsafe {
            smrt_dag_ray_fire(if pick(b, 0) { null() } else { tp() }, if pick(b, 1) { null_mut() } else { cp() }, 0, if pick(b, 2) { null() } else { o.as_ptr() }, if pick(b, 3) { null() } else { d.as_ptr() }, 1, 0.0, if pick(b, 4) { null_mut() } else { i64a }, if pick(b, 5) { null_mut() } else { f64a }, null_mut())
        })),
        ("smrt_dag_point_in_volume", 4, Box::new(|b| unsafe { smrt_dag_point_in_volume(if pick(b, 0) { null() } else { tp() }, if pick(b, 1) { null_mut() } else { cp() }, 0, if pick(b, 2) { null() } else { p.as_ptr() }, null(), if pick(b, 3) { null_mut() } else { i32out }) })),
        ("smrt_dag_next_vol", 2, Box::new(|b| unsafe { smrt_dag_next_vol(if pick(b, 0) { null() } else { tp() }, 0, 0, if pick(b, 1) { null_mut() } else { i64b }) })),
        ("smrt_dag_surface_sense", 2, Box::new(|b| unsafe { smrt_dag_surface_sense(if pick(b, 0) { null() } else { tp() }, 0, 0, if pick(b, 1) { null_mut() } else { i32out }) })),
        ("smrt_dag_get_angle", 3, Box::new(|b| unsafe { smrt_dag_get_angle(if pick(b, 0) { null() } else { tp() }, 0, if pick(b, 1) { null() } else { p.as_ptr() }, if pick(b, 2) { null_mut() } else { n3 }) })),
        ("smrt_dag_measure_volume", 2, Box::new(|b| unsafe { smrt_dag_measure_volume(if pick(b, 0) { null() } else { tp() }, 0, if pick(b, 1) { null_mut() } else { f64a }) })),
        ("smrt_dag_measure_area", 2, Box::new(|b| unsafe { smrt_dag_measure_area(if pick(b, 0) { null() } else { tp() }, 0, if pick(b, 1) { null_mut() } else { f64a }) })),
        ("smrt_trace_bundle", 4, Box::new(|b| unsafe {
            smrt_trace_bundle(if pick(b, 0) { null() } else { tp() }, 1, if pick(b, 1) { null() } else { &rays }, 0, null(), if pick(b, 2) { null_mut() } else { words }, 64, if pick(b, 3) { null_mut() } else { outlen })
        })),
        ("smrt_last_error_message", 1, Box::new(|_| unsafe { smrt_last_error_message(null_mut(), 8) })),
    ];
    assert!(rows.len() >= 24);
    for (name, n, f) in &rows {
        for bad in 0..*n {
            assert_eq!(f(bad), SMRT_ERR_NULL, "{name}: pointer argument {bad} null");
            assert!(err_msg().contains("null pointer"), "{name}");
        }
    }
    // The same functions are fine with every pointer valid (spot check of the harness itself).
    assert!(rows.iter().find(|r| r.0 == "smrt_step").map(|r| (r.2)(usize::MAX)).is_some_and(|v| v >= 0));
    // SAFETY: valid buffer.
    let _ = unsafe { smrt_last_error_message(buf.as_mut_ptr().cast(), 16) };
}

#[test]
fn a_panic_in_step_is_contained_and_reported_per_thread() {
    // SAFETY: the stub hook writes one handle.
    let mut t: *mut SmrtTracker = null_mut();
    assert_eq!(unsafe { smrt_test_stub(STUB_PANIC_ON_STEP, 5.0, &mut t) }, 0);
    let c = ctx_of(t);
    assert_eq!(init(c, [-9.0, 0.0, 0.0], [1.0, 0.0, 0.0]), 0);
    assert_eq!(step(c, 1.0).0, SMRT_ERR_PANIC);
    assert!(err_msg().contains("deliberate panic in Tracker::step"), "{}", err_msg());
    assert_eq!(fire(c).0, SMRT_ERR_PANIC);
    // The process survived and the tracker still answers queries that do not step.
    let mut loc = SmrtLocated::default();
    // SAFETY: live ctx, valid pointers.
    assert_eq!(unsafe { smrt_locate(c, [0.0; 3].as_ptr(), null(), -1, &mut loc) }, 0);

    // Thread-local message: two threads fail differently at the same time, the main thread's own
    // message is unaffected by both.
    let before = err_msg();
    let (tp, cp) = (t as usize, c as usize);
    let gate = Arc::new(Barrier::new(2));
    let (g1, g2) = (gate.clone(), gate.clone());
    let a = std::thread::spawn(move || {
        let c2 = ctx_of(tp as *const SmrtTracker);
        init(c2, [-9.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
        assert_eq!(step(c2, 1.0).0, SMRT_ERR_PANIC);
        g1.wait();
        g1.wait();
        err_msg()
    });
    let b = std::thread::spawn(move || {
        let c2 = ctx_of(tp as *const SmrtTracker);
        g2.wait();
        assert_eq!(init(c2, [0.0; 3], [2.0, 0.0, 0.0]), SMRT_ERR_BAD_DIRECTION);
        g2.wait();
        err_msg()
    });
    let (ma, mb) = (a.join().unwrap(), b.join().unwrap());
    assert!(ma.contains("deliberate panic"), "{ma}");
    assert!(mb.contains("direction"), "{mb}");
    assert_eq!(err_msg(), before, "main thread message changed by other threads");
    let _ = cp;
}

#[test]
fn errors_reach_the_caller_with_messages() {
    let t = sphere5();
    let c = ctx_of(t);
    assert_eq!(init(c, [f64::NAN, 0.0, 0.0], [1.0, 0.0, 0.0]), SMRT_ERR_NON_FINITE);
    assert_eq!(init(c, [0.0; 3], [0.0, 0.0, 0.0]), SMRT_ERR_BAD_DIRECTION);
    assert_eq!(init(c, [0.0; 3], [1.0, 0.0, 0.0]), 0);
    let neg = step(c, -1.0).0;
    assert!(neg < 0 && neg > -100, "negative max_dist is a contract error, got {neg}");
    let nan = step(c, f64::NAN).0;
    assert!(nan < 0 && nan > -100, "NaN max_dist is a contract error, got {nan}");
    let mut loc = SmrtLocated::default();
    // SAFETY: valid pointers.
    assert_eq!(unsafe { smrt_locate(c, [0.0; 3].as_ptr(), null(), -7, &mut loc) }, SMRT_ERR_UNKNOWN_ID);
    // Message API: size query, truncation, snprintf semantics.
    let full = err_msg();
    // SAFETY: cap 0 allows a null buffer.
    let n = unsafe { smrt_last_error_message(null_mut(), 0) };
    assert_eq!(n as usize, full.len());
    let mut small = [0x55u8; 6];
    // SAFETY: valid buffer.
    let n2 = unsafe { smrt_last_error_message(small.as_mut_ptr().cast(), 6) };
    assert_eq!(n2 as usize, full.len());
    assert_eq!(small[5], 0);
    assert_eq!(&small[..5], &full.as_bytes()[..5]);
}
