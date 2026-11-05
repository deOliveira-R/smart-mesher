//! W3b-RT-ANALYTIC acceptance 13 (allocation): after warm-up, `step`, `locate`, `closest` and
//! `distance_to_boundary` of the analytic tracker allocate nothing, for every surface kind.
//! The counting allocator counts only on the thread that switched counting on (a
//! const-initialised thread-local flag, which itself never allocates); it is the only `unsafe`
//! in the crate's tests (each method forwards to `System`).
#![allow(unsafe_code)]
#[path = "common/csg.rs"]
mod csg;

use csg::*;
use sm_geom_api::{Frame, Quadric};
use sm_rt_api::{CellId, ComplementPolicy, Scratch, StepEnd, Tracker};
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

fn workload(t: &dyn Tracker, sc: &mut Scratch, seed: &str) -> (usize, f64) {
    let mut r = rng(seed);
    let (mut events, mut acc) = (0, 0.0);
    for _ in 0..3000 {
        let o = p3(12.0 * r.unit(), 12.0 * r.unit(), 12.0 * r.unit());
        let Ok(mut st) = t.init_state(o, random_dir(&mut r), None, sc) else { continue };
        acc += t.locate(o, None, Some(CellId::new(0)), sc).unwrap().cell.0 as f64;
        acc += t.closest(o, None, sc).map_or(0.0, |c| c.map_or(0.0, |c| c.dist));
        if st.cell.is_none() {
            continue;
        }
        for _ in 0..200 {
            let _ = t.distance_to_boundary(&st, f64::INFINITY, sc).unwrap();
            let s = t.step(&mut st, f64::INFINITY, sc).unwrap();
            acc += s.dist;
            events += 1;
            if matches!(s.end, StepEnd::Boundary { .. } | StepEnd::Lost(_)) {
                break;
            }
        }
    }
    (events, acc)
}

#[test]
fn warm_scratch_means_zero_allocations_for_every_surface_kind() {
    let mut r = rng("alloc-fixture");
    let mut fx = Fx::new(40.0);
    fx.complement = ComplementPolicy::Outside;
    fx.bounds = None;
    let box_ = fx.box_planes(p3(-20.0, -20.0, -20.0), p3(20.0, 20.0, 20.0));
    let sph = fx.sphere(p3(0.0, 0.0, 0.0), 4.0);
    let cyl = fx.cylinder(Frame::at(p3(0.0, 0.0, 0.0)), 7.0);
    let fk = random_frame(&mut r, p3(5.0, 5.0, -15.0));
    let cone = fx.cone(fk, 0.5);
    let ft = random_frame(&mut r, p3(-8.0, 0.0, 0.0));
    let tor = fx.torus(ft, 3.0, 1.0);
    let ell = fx.quadric(Quadric::from_local(&Frame::at(p3(10.0, -10.0, 0.0)), [1.0 / 4.0, 1.0 / 9.0, 1.0 / 16.0], [0.0; 3], -1.0));
    let inbox: Vec<_> = box_.iter().map(|&p| hs(p, true)).collect();
    let with = |extra: &[sm_geom_api::HalfSpace]| {
        let mut v = inbox.clone();
        v.extend_from_slice(extra);
        v
    };
    fx.cell1("ball", &[hs(sph, true)]);
    fx.cell1("torus", &[hs(tor, true)]);
    fx.cell1("ell", &[hs(ell, true)]);
    fx.cell1("cone", &with(&[hs(cone, true)]));
    fx.cell1("shell", &with(&[hs(sph, false), hs(cyl, true), hs(tor, false), hs(ell, false), hs(cone, false)]));
    fx.cell1("rest", &with(&[hs(cyl, false)]));
    let t = fx.build();
    assert_eq!(t.surface_kinds().iter().copied().collect::<std::collections::BTreeSet<_>>().len(), 6, "{:?}", t.surface_kinds());
    let mut sc = Scratch::new();
    let _ = workload(&t, &mut sc, "warm-a");
    let _ = workload(&t, &mut sc, "warm-b");
    let ((events, _), n) = count_allocs(|| workload(&t, &mut sc, "measured"));
    assert!(events > 1000, "{events}");
    assert_eq!(n, 0, "{n} allocations in {events} events");
}

/// Steps through the complement cell (a `Region` complement borders every surface) allocate
/// nothing either.
#[test]
fn complement_region_steps_allocate_nothing() {
    let mut fx = Fx::new(20.0);
    let sph = fx.sphere(p3(0.0, 0.0, 0.0), 4.0);
    let cyl = fx.cylinder(Frame::at(p3(8.0, 0.0, 0.0)), 2.0);
    let pl = fx.plane(p3(-8.0, 0.0, 0.0), v3(-1.0, 0.0, 0.0));
    fx.cell1("ball", &[hs(sph, true)]);
    fx.cell1("rod", &[hs(cyl, true)]);
    fx.cell1("wall", &[hs(pl, true), hs(sph, false)]);
    let t = fx.build();
    let mut sc = Scratch::new();
    let _ = workload(&t, &mut sc, "warm-a");
    let _ = workload(&t, &mut sc, "warm-b");
    let ((events, _), n) = count_allocs(|| workload(&t, &mut sc, "measured"));
    assert!(events > 1000, "{events}");
    assert_eq!(n, 0, "{n} allocations in {events} events");
}
