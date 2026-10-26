//! The `Scratch` contract: after warm-up, locate / step / peek / closest allocate nothing while
//! the hierarchy depth is within `LEVEL_INLINE`. One test per file so no other thread perturbs
//! the counter.
#![cfg(feature = "stubs")]

use sm_core::{Point3, Vector3};
use sm_rt_api::stub::{AnalyticWorld, RectLattice, Shape};
use sm_rt_api::{FaceAction, ResponseKind, Scratch, StepEnd, Tracker};
use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

struct Counting;
static ALLOCS: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        unsafe { System.alloc(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        unsafe { System.realloc(p, l, n) }
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        ALLOCS.fetch_add(1, Ordering::SeqCst);
        unsafe { System.alloc_zeroed(l) }
    }
}

#[global_allocator]
static A: Counting = Counting;

fn walk(w: &dyn Tracker, sc: &mut Scratch) -> (usize, f64) {
    let mut events = 0;
    let mut acc = 0.0;
    for i in 0..200 {
        let f = f64::from(i);
        let d = Vector3::new(3.0, 1.0 + 0.01 * f, 0.5);
        let d = d * (1.0 / d.norm());
        let o = Point3::new(0.2, 0.3 + 0.05 * f, 0.4);
        let mut st = w.init_state(o, d, None, sc).unwrap();
        loop {
            let _ = w.distance_to_boundary(&st, f64::INFINITY, sc).unwrap();
            let s = w.step(&mut st, f64::INFINITY, sc).unwrap();
            acc += s.dist;
            events += 1;
            if matches!(s.end, StepEnd::Boundary { .. } | StepEnd::Lost(_)) {
                break;
            }
        }
        acc += w.closest(o, None, sc).unwrap().map_or(0.0, |c| c.dist);
        acc += f64::from(w.locate(o, None, None, sc).unwrap().cell.0);
    }
    (events, acc)
}

#[test]
fn queries_allocate_nothing_after_warm_up() {
    let shapes = vec![
        Shape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: 1.0 },
        Shape::Box { lo: Point3::new(1.2, -0.3, -0.3), hi: Point3::new(1.8, 0.3, 0.3) },
    ];
    let l = RectLattice { origin: Point3::new(0.0, 0.0, 0.0), pitch: [4.0; 3], dims: [6, 5, 3] };
    let w = AnalyticWorld::with_lattice(shapes, l, [FaceAction::Respond(ResponseKind::Vacuum); 6]).unwrap();
    let mut sc = Scratch::new();
    let warm = walk(&w, &mut sc);
    let before = ALLOCS.load(Ordering::SeqCst);
    let again = walk(&w, &mut sc);
    let used = ALLOCS.load(Ordering::SeqCst) - before;
    assert_eq!(warm.0, again.0);
    assert!(again.0 > 1000, "the walk should be non-trivial: {} events", again.0);
    assert_eq!(used, 0, "{used} allocations in {} events", again.0);
}
