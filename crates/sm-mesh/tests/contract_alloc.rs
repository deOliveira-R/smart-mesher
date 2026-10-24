//! Hot-path geometry and mesh walking must not allocate. This file holds exactly one test so no
//! other test thread can perturb the counter.

mod common;
use common::shim::hex_block;
use sm_mesh_api::*;
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
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

#[test]
fn geometry_and_walking_over_dyn_mesh_read_allocate_nothing() {
    let m = hex_block([6, 5, 4], [1.0, 0.5, 2.0], &mut SerialAlloc::new()).unwrap();
    let dynm: &dyn MeshRead = black_box(&m);
    let (nf, nc) = (dynm.face_count(), dynm.cell_count());

    let before = ALLOCS.load(Ordering::SeqCst);
    let mut acc = 0.0;
    for f in 0..nf {
        let a = face_area_vector(dynm, FaceIx::new(f));
        acc += a.x + a.y + a.z;
        let c = face_centroid(dynm, FaceIx::new(f));
        acc += c.x;
        let g = face_geometry(dynm, FaceIx::new(f));
        acc += g.apex.y;
    }
    let mut steps = 0usize;
    for c in 0..nc {
        let c = CellIx::new(c);
        acc += cell_volume(dynm, c);
        let (v, p) = cell_volume_centroid(dynm, c);
        acc += v + p.z;
        for &sf in dynm.cell_faces(c) {
            if let Some(n) = dynm.across(sf) {
                steps += n.get() & 1;
            }
            let _ = dynm.other_cell(sf.face(), c);
            acc += face_area_vector(dynm, sf.face()).dot(sm_core::Vector3::new(1.0, 0.0, 0.0))
                * sf.sign();
        }
        let cols = dynm.columns();
        acc += cols.cell_faces.len() as f64;
        let _ = dynm.cell_corners(c);
        let _ = dynm.cell_shape(c);
    }
    let after = ALLOCS.load(Ordering::SeqCst);
    black_box((acc, steps));
    assert_eq!(after - before, 0, "geometry and walking must not allocate");
    assert!(acc.is_finite());
    // The counter works: a real allocation is seen.
    let b4 = ALLOCS.load(Ordering::SeqCst);
    black_box(vec![1u8; 16]);
    assert!(ALLOCS.load(Ordering::SeqCst) > b4);
}
