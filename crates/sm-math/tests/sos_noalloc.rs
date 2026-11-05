//! `orient3d_sos_noalloc`: same signs as `orient3d_sos`, zero allocations, determinism.

mod common;
use common::Rng;
use sm_math::sos::{SosError, Tagged, orient3d_sos, orient3d_sos_noalloc};
use sm_math::{Point3, orient3d};
use sm_testkit::determinism::{check, hash_f64_bits, par_map};
use std::alloc::{GlobalAlloc, Layout, System};
use std::cell::Cell;

struct Counting;

thread_local! {
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

// SAFETY: forwards to the system allocator; the counter is a const-initialised thread local.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        ALLOCS.with(|c| c.set(c.get() + 1));
        unsafe { System.alloc(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        ALLOCS.with(|c| c.set(c.get() + 1));
        unsafe { System.realloc(p, l, n) }
    }
}

#[global_allocator]
static A: Counting = Counting;

fn allocs() -> usize {
    ALLOCS.with(Cell::get)
}

fn pt(p: [f64; 3]) -> Point3 {
    Point3::new(p[0], p[1], p[2])
}

/// Four random lattice points in `{-1,0,1}^3` with distinct random ids, degenerate by retry.
fn degenerate_tuple(r: &mut Rng) -> [Tagged<Point3>; 4] {
    loop {
        let mut p = || pt([0; 3].map(|_| r.int(1) as f64));
        let pts = [p(), p(), p(), p()];
        if !orient3d(pts[0], pts[1], pts[2], pts[3]).is_zero() {
            continue;
        }
        let mut ids = [0u64; 4];
        for i in 0..4 {
            loop {
                ids[i] = r.next() % 1000;
                if !ids[..i].contains(&ids[i]) {
                    break;
                }
            }
        }
        return [0, 1, 2, 3].map(|i| Tagged::new(pts[i], ids[i]));
    }
}

#[test]
fn matches_allocating_on_degenerate_grid() {
    let rays = sm_testkit::rays::degenerate_grid(64);
    assert_eq!(rays.len(), 7 * 65 * 65 * 65);
    let mut degenerate = 0usize;
    // Debug builds check every 16th ray (the allocating oracle is slow unoptimised); release
    // builds check all of them.
    let stride = if cfg!(debug_assertions) { 16 } else { 1 };
    for (k, (o, d)) in rays.iter().enumerate().step_by(stride) {
        // Four lattice points on the ray: o + t d for t = 0..3 are collinear, so degenerate.
        let q = |t: f64, id: u64| {
            Tagged::new(Point3::new(o.x + t * d.x, o.y + t * d.y, o.z + t * d.z), id + 4 * k as u64)
        };
        let (a, b, c, e) = (q(0.0, 0), q(1.0, 1), q(2.0, 2), q(3.0, 3));
        // Fourth point off the line but in a lattice plane through the line.
        let f = Tagged::new(Point3::new(o.x + 2.0 * d.x + d.y, o.y + 2.0 * d.y - d.x, o.z + 2.0 * d.z), 9_000_000 + k as u64);
        for tup in [[a, b, c, e], [a, b, c, f], [e, c, b, a]] {
            if orient3d(tup[0].point, tup[1].point, tup[2].point, tup[3].point).is_zero() {
                degenerate += 1;
            }
            assert_eq!(orient3d_sos(tup[0], tup[1], tup[2], tup[3]), orient3d_sos_noalloc(tup[0], tup[1], tup[2], tup[3]));
        }
    }
    assert!(degenerate > rays.len() / stride);
}

#[test]
fn matches_allocating_on_random_degenerate() {
    let mut r = Rng(0x5051);
    for _ in 0..1_000_000 {
        let [a, b, c, d] = degenerate_tuple(&mut r);
        assert_eq!(orient3d_sos(a, b, c, d), orient3d_sos_noalloc(a, b, c, d));
        assert!(orient3d_sos_noalloc(a, b, c, d).is_ok_and(|s| !s.is_zero()));
    }
}

#[test]
fn nondegenerate_and_duplicate_ids() {
    let p = |x, y, z| Tagged::new(Point3::new(x, y, z), 0);
    let t = |x, y, z, id| Tagged::new(Point3::new(x, y, z), id);
    let s = orient3d_sos_noalloc(t(0.0, 0.0, 0.0, 1), t(1.0, 0.0, 0.0, 2), t(0.0, 1.0, 0.0, 3), t(0.0, 0.0, 1.0, 4));
    assert_eq!(s, orient3d_sos(t(0.0, 0.0, 0.0, 1), t(1.0, 0.0, 0.0, 2), t(0.0, 1.0, 0.0, 3), t(0.0, 0.0, 1.0, 4)));
    let dup = orient3d_sos_noalloc(p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(2.0, 0.0, 0.0), p(3.0, 0.0, 0.0));
    assert_eq!(dup, Err(SosError::DuplicateId(0)));
}

#[test]
fn zero_allocations() {
    let mut r = Rng(77);
    let tuples: Vec<_> = (0..20_000).map(|_| degenerate_tuple(&mut r)).collect();
    let before = allocs();
    let mut acc = 0i32;
    for [a, b, c, d] in &tuples {
        acc += orient3d_sos_noalloc(*a, *b, *c, *d).map_or(100, |s| s.as_i32());
    }
    assert_eq!(allocs() - before, 0, "allocated (acc {acc})");
    // The allocating version does allocate, so the counter is live.
    let before = allocs();
    let [a, b, c, d] = tuples[0];
    let _ = orient3d_sos(a, b, c, d);
    assert!(allocs() > before);
}

#[test]
fn deterministic_across_threads() {
    let h = check(2, 4, |threads| {
        let v = par_map(threads, 20_000, |i| {
            let mut r = Rng(i as u64);
            let [a, b, c, d] = degenerate_tuple(&mut r);
            f64::from(orient3d_sos_noalloc(a, b, c, d).map_or(9, |s| s.as_i32()))
        });
        hash_f64_bits(&v)
    });
    assert!(h.is_ok(), "{h:?}");
}
