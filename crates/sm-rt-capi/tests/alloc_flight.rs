//! No heap allocation in `smrt_flight_next` / `probe` / `check` / `stop_at` once the flight exists
//! (the per-segment calls of a transport loop). One test per file so no other thread perturbs
//! the counter.
#![cfg(feature = "stubs")]

mod common;
use common::*;
use sm_rt_capi::ffi::test_hooks::*;
use sm_rt_capi::ffi::*;
use sm_rt_capi::*;
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

struct Counting;
static ALLOCS: AtomicUsize = AtomicUsize::new(0);

// SAFETY: every method forwards to `System` with the caller's arguments unchanged; only a
// counter is added, so the `GlobalAlloc` contract holds exactly as `System` upholds it.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        // SAFETY: `l` forwarded as given.
        unsafe { System.alloc(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        // SAFETY: `p`/`l` come from a prior allocation through this allocator, i.e. `System`.
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        // SAFETY: `p`/`l` came from `System`; `n` forwarded as given.
        unsafe { System.realloc(p, l, n) }
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        // SAFETY: `l` forwarded as given.
        unsafe { System.alloc_zeroed(l) }
    }
}

#[global_allocator]
static A: Counting = Counting;

fn flights(t: *const SmrtTracker, policy: i64, n: usize) -> Vec<*mut SmrtFlight> {
    let c = ctx_of(t);
    let opts = SmrtFlightOpts { max_dist: 0.0, max_steps: 0, stop_at_deck: 0, policy, scale: SMRT_SCALE_CELL };
    let mut out = Vec::new();
    for i in 0..n {
        let f = i as f64;
        let d = [3.0, 1.0 + 0.01 * f, 0.5];
        let nrm = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        assert_eq!(init(c, [-8.0, -4.0 + 0.04 * f, 0.4], [d[0] / nrm, d[1] / nrm, d[2] / nrm]), 0);
        let mut st = SmrtState::default();
        // SAFETY: live ctx and out slot.
        assert_eq!(unsafe { smrt_get_state(c, &mut st) }, 0);
        let mut fl = std::ptr::null_mut();
        // SAFETY: live tracker, valid state and options, valid out slot.
        assert_eq!(unsafe { smrt_flight_new(t, &st, &opts, &mut fl) }, 0);
        out.push(fl);
    }
    // SAFETY: freed once.
    unsafe { smrt_ctx_free(c) };
    out
}

#[test]
fn flight_calls_allocate_nothing_after_the_flight_exists() {
    let t = stub(STUB_GRID, 5.0);
    let mut nn = 0i64;
    // SAFETY: live handle and out slot.
    assert_eq!(unsafe { smrt_n_nodes(t, &mut nn) }, 0);
    let maj = vec![2.0f64; nn as usize];
    let block = [1u32, 3, 5];
    let all: Vec<*mut SmrtFlight> = [flights(t, SMRT_POLICY_SURFACE, 100), flights(t, SMRT_POLICY_DELTA, 100)].concat();
    let before = ALLOCS.load(Ordering::SeqCst);
    let mut segments = 0usize;
    for (k, &f) in all.iter().enumerate() {
        let mut seg = SmrtSegment::default();
        loop {
            // SAFETY: live flight; arrays of the stated lengths; valid out slot.
            let rc = unsafe { smrt_flight_next(f, maj.as_ptr(), maj.len(), block.as_ptr(), if k % 2 == 0 { 0 } else { block.len() }, &mut seg) };
            if rc != 0 {
                assert_eq!(rc, 1);
                break;
            }
            segments += 1;
            let mut loc = SmrtLocated::default();
            // SAFETY: as above.
            unsafe {
                assert_eq!(smrt_flight_probe(f, maj.as_ptr(), maj.len(), std::ptr::null(), 0, 0.5 * (seg.t0 + seg.t1), &mut loc), 0);
                assert_eq!(smrt_flight_check(f, 0.5 * seg.sigma_m), 0);
            }
        }
    }
    let used = ALLOCS.load(Ordering::SeqCst) - before;
    assert!(segments > 500, "{segments} segments");
    assert_eq!(used, 0, "{used} allocations in {segments} segments");
    for f in all {
        // SAFETY: freed once.
        unsafe { smrt_flight_free(f) };
    }
}
