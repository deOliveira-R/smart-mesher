//! Hot-path geometry and mesh walking must not allocate. The counter is per-thread, so allocations
//! made concurrently by the libtest harness or other threads cannot perturb the measurement.

mod common;
use common::shim::hex_block;
use sm_mesh_api::*;
use std::alloc::{GlobalAlloc, Layout, System};
use std::hint::black_box;
use std::cell::Cell;

struct Counting;
thread_local! {
    // const-init + no destructor: accessing it never allocates or registers a dtor.
    static ALLOCS: Cell<usize> = const { Cell::new(0) };
}

fn bump() {
    // `try_with` so allocations during thread teardown are ignored instead of panicking.
    let _ = ALLOCS.try_with(|c| c.set(c.get() + 1));
}

fn count() -> usize {
    ALLOCS.with(Cell::get)
}

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        bump();
        unsafe { System.alloc(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        bump();
        unsafe { System.realloc(p, l, n) }
    }
    unsafe fn alloc_zeroed(&self, l: Layout) -> *mut u8 {
        bump();
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

    let run = || {
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
        black_box((acc, steps));
        acc
    };
    // Warm up: any first-use init happens outside the measured region.
    black_box(run());
    let before = count();
    let acc = run();
    let after = count();
    assert_eq!(after - before, 0, "geometry and walking must not allocate");
    assert!(acc.is_finite());
    // The counter works: a real allocation is seen.
    let b4 = count();
    black_box(vec![1u8; 16]);
    assert!(count() > b4);
}
