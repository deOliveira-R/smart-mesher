//! No heap allocation in `smrt_step` / `smrt_peek` / `smrt_fire` after the context exists
//! (acceptance 7). One test per file so no other thread perturbs the counter.
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

fn walk(c: *mut SmrtCtx) -> (usize, f64) {
    let (mut events, mut acc) = (0, 0.0);
    for i in 0..300 {
        let f = f64::from(i);
        let d = [3.0, 1.0 + 0.01 * f, 0.5];
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        assert_eq!(init(c, [0.2, 0.3 + 0.03 * f, 0.4], [d[0] / n, d[1] / n, d[2] / n]), 0);
        let mut p = SmrtStep::default();
        loop {
            // SAFETY: live ctx, valid out slot.
            let _ = unsafe { smrt_peek(c, f64::INFINITY, &mut p) };
            let (_, s) = step(c, f64::INFINITY);
            acc += s.dist;
            events += 1;
            if s.end == SMRT_END_BOUNDARY || s.end == SMRT_END_LOST {
                break;
            }
        }
        // And the fire path from a fresh start.
        init(c, [0.2, 0.3 + 0.03 * f, 0.4], [d[0] / n, d[1] / n, d[2] / n]);
        acc += fire(c).1.dist;
        // SAFETY: as above.
        unsafe { smrt_set_direction(c, d[0] / n, d[1] / n, d[2] / n) };
    }
    (events, acc)
}

#[test]
fn step_peek_fire_allocate_nothing_after_context_creation() {
    let t = stub(STUB_LATTICE, 1.0);
    let c = ctx_of(t);
    let warm = walk(c);
    let before = ALLOCS.load(Ordering::SeqCst);
    let again = walk(c);
    let used = ALLOCS.load(Ordering::SeqCst) - before;
    assert_eq!(warm.0, again.0);
    assert!(again.0 > 1000, "{} events", again.0);
    assert_eq!(used, 0, "{used} allocations in {} events", again.0);
}
