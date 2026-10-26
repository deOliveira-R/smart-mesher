//! Acceptance 13: after warm-up, `step`, `locate` and `closest` allocate nothing.
//! One test per file so no other thread perturbs the counter. The counting allocator is the
//! only `unsafe` in the crate's tests (each method forwards to `System`).
#![allow(unsafe_code)]
mod common;
use common::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::SerialAlloc;
use sm_rt_api::{CellId, Scratch, StepEnd, Tracker};
use sm_testkit::meshes::nonconvex_poly;
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

struct Counting;
static ALLOCS: AtomicUsize = AtomicUsize::new(0);

// SAFETY: every method forwards to `System` with the caller's arguments unchanged, so the
// `GlobalAlloc` contract is upheld exactly as `System` upholds it; only a counter is added.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        // SAFETY: `l` is forwarded as given; the caller meets `System.alloc`'s contract.
        unsafe { System.alloc(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        // SAFETY: `p`/`l` come from a prior allocation through this allocator, i.e. from `System`.
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        // SAFETY: `p`/`l` came from `System` and `n` is forwarded as given.
        unsafe { System.realloc(p, l, n) }
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        // SAFETY: `l` is forwarded as given by the caller.
        unsafe { System.alloc_zeroed(l) }
    }
}

#[global_allocator]
static A: Counting = Counting;

fn workload(t: &dyn Tracker, sc: &mut Scratch, lo: [f64; 3], hi: [f64; 3], seed: &str) -> (usize, f64) {
    let mut r = rng(seed);
    let mut events = 0;
    let mut acc = 0.0;
    for _ in 0..1000 {
        let u = |k: usize, r: &mut SplitMix64| lo[k] + (hi[k] - lo[k]) * (r.unit() + 1.0) / 2.0;
        let o = [u(0, &mut r), u(1, &mut r), u(2, &mut r)];
        let (d, _, _) = random_frame(&mut r);
        let Ok(mut st) = t.init_state(p3(o), v3(d), None, sc) else { continue };
        acc += t.locate(p3(o), None, Some(CellId::new(0)), sc).unwrap().cell.0 as f64;
        acc += t.closest(p3(o), None, sc).unwrap().map_or(0.0, |c| c.dist);
        if st.cell.is_none() {
            continue;
        }
        for _ in 0..400 {
            let _ = t.distance_to_boundary(&st, f64::INFINITY, sc).unwrap();
            let s = t.step(&mut st, f64::INFINITY, sc).unwrap();
            acc += s.dist;
            events += 1;
            match s.end {
                StepEnd::Boundary { .. } | StepEnd::Lost(_) => break,
                _ => {}
            }
        }
    }
    (events, acc)
}

#[test]
fn acc13_warm_scratch_means_zero_allocations() {
    let hb = hex_block([6; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let nc = nonconvex_poly().unwrap();
    let ico = nested_facets();
    let mesh_t = build_mesh_tracker(&hb, &vacuum(), &opts_fail());
    let nc_t = build_mesh_tracker(&nc.mesh, &vacuum(), &opts_fail());
    let facet_t = build(&ico, &opts_fail()).unwrap();
    let cases: [(&dyn Tracker, [f64; 3], [f64; 3]); 3] =
        [(&mesh_t, [-0.2; 3], [6.2; 3]), (&nc_t, [-0.2, -0.2, -0.2], [3.2, 2.2, 1.2]), (&*facet_t, [-2.2; 3], [2.2; 3])];
    for (t, lo, hi) in cases {
        let mut sc = Scratch::new();
        // Warm up on two seeds, then measure a third, different set of rays.
        let _ = workload(t, &mut sc, lo, hi, "acc13_warm_a");
        let _ = workload(t, &mut sc, lo, hi, "acc13_warm_b");
        let before = ALLOCS.load(Ordering::SeqCst);
        let again = workload(t, &mut sc, lo, hi, "acc13_fresh");
        let after = ALLOCS.load(Ordering::SeqCst);
        assert!(again.0 > 200, "workload too small: {} events", again.0);
        eprintln!("{} events, 0 allocations", again.0);
        assert_eq!(after - before, 0, "allocations after warm-up");
    }
}
