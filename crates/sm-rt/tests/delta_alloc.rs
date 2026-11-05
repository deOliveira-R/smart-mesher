//! W3b-RT-DELTA acceptance 12: after warm-up, `next_segment`, `probe`, `stop_at` and the region
//! jump `fire` allocate nothing. The counting allocator counts only on the thread that switched counting
//! on (a const-initialised thread-local flag, which itself never allocates), so tests running
//! in parallel threads of this binary cannot perturb each other's counts. The counting
//! allocator is the only `unsafe` in the crate's tests (each method forwards to `System`).
#![allow(unsafe_code)]
mod common;
mod dcommon;
use common::*;
use dcommon::*;
use sm_rt_api::{FlightInput, FlightOptions, ModePolicy, Scratch, SegmentScale, Tracker};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

struct Counting;

thread_local! {
    /// Allocations made by this thread while `COUNTING` is set.
    static COUNTING: Cell<bool> = const { Cell::new(false) };
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

fn bump() {
    // `try_with`: allocation can happen while a thread is being torn down.
    let _ = COUNTING.try_with(|c| {
        if c.get() {
            let _ = ALLOCS.try_with(|a| a.set(a.get() + 1));
        }
    });
}

/// Run `f` and return its result with the number of allocations this thread made meanwhile.
fn count_allocs<T>(f: impl FnOnce() -> T) -> (T, usize) {
    ALLOCS.with(|a| a.set(0));
    COUNTING.with(|c| c.set(true));
    let r = f();
    COUNTING.with(|c| c.set(false));
    (r, ALLOCS.with(Cell::get))
}

// SAFETY: every method forwards to `System` with the caller's arguments unchanged, so the
// `GlobalAlloc` contract is upheld exactly as `System` upholds it; only a counter is added.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        bump();
        // SAFETY: `l` is forwarded as given; the caller meets `System.alloc`'s contract.
        unsafe { System.alloc(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        // SAFETY: `p`/`l` come from a prior allocation through this allocator, i.e. from `System`.
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        bump();
        // SAFETY: `p`/`l` came from `System` and `n` is forwarded as given.
        unsafe { System.realloc(p, l, n) }
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        bump();
        // SAFETY: `l` is forwarded as given by the caller.
        unsafe { System.alloc_zeroed(l) }
    }
}

#[global_allocator]
static A: Counting = Counting;


fn flights(t: &sm_rt::MeshTracker, maj: &[f64], sc: &mut Scratch, seed: &str, mode: ModePolicy, scale: SegmentScale, fire: bool) -> (u64, f64) {
    let view = t.majorant_view().unwrap();
    let input = FlightInput::new(maj);
    let mut r = rng(seed);
    let (mut count, mut acc) = (0u64, 0.0);
    for _ in 0..300 {
        let o = [6.0 * unit01(&mut r), 6.0 * unit01(&mut r), 6.0 * unit01(&mut r)];
        let (d, _, _) = random_frame(&mut r);
        let mut st = t.init_state(p3(o), v3(d), None, sc).unwrap();
        if st.cell.is_none() {
            continue;
        }
        if fire {
            let fr = t.fire(&mut st, f64::INFINITY, 1000, sc).unwrap();
            acc += fr.dist;
            count += 1;
            continue;
        }
        let mut cur = view.begin(&st, &FlightOptions::default().with_policy(mode).with_scale(scale)).unwrap();
        while let Some(seg) = cur.next_segment(view, input, &mut st, sc).unwrap() {
            acc += seg.length();
            count += 1;
            let tm = seg.t0 + 0.5 * seg.length();
            let loc = cur.probe(view, input, sc, tm).unwrap();
            acc += f64::from(loc.cell.0);
            cur.check_collision(0.5).unwrap();
            let _ = cur.safety_distance(view, &st, sc).unwrap();
            if count % 7 == 0 {
                cur.stop_at(view, input, &mut st, sc, tm).unwrap();
            }
        }
    }
    (count, acc)
}

#[test]
fn acc12_flights_allocate_nothing_after_warm_up() {
    let fx = sm_testkit::meshes::hex_block_two_volumes([6, 6, 6], [1.0; 3], 3).unwrap();
    let t = build_part(&fx.mesh, &sm_rt_api::RegionRule::ByVolumeClassification, &vacuum(), popts(2, true));
    let maj = vec![1.0; t.majorant_view().unwrap().hierarchy().node_count()];
    for (mode, scale, fire) in [
        (ModePolicy::Delta, SegmentScale::Cell, false),
        (ModePolicy::Surface, SegmentScale::Cell, false),
        (ModePolicy::Surface, SegmentScale::Region, false),
        (ModePolicy::Delta, SegmentScale::Cell, true),
    ] {
        let mut sc = Scratch::new();
        let _ = flights(&t, &maj, &mut sc, "warm_a", mode, scale, fire);
        let _ = flights(&t, &maj, &mut sc, "warm_b", mode, scale, fire);
        let ((count, _), allocs) = count_allocs(|| flights(&t, &maj, &mut sc, "fresh", mode, scale, fire));
        assert!(count > 200, "workload too small: {count}");
        eprintln!("{mode:?} {scale:?} fire={fire}: {count} calls, {allocs} allocations");
        assert_eq!(allocs, 0, "allocations after warm-up: {mode:?} {scale:?} fire={fire}");
    }
}
