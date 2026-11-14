//! The selection allocates in proportion to what it keeps, not to the host mesh: the same block
//! selected from a 12-cell and from a 600-cell host costs about the same bytes.
//!
//! This binary holds one test, so the counting allocator sees no other thread's allocations
//! except the worker threads of the builder (which is part of the cost being measured).
#![allow(missing_docs)]

#[path = "common/select_rig.rs"]
mod rig;

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use sm_mesh_api::MeshRead;
use sm_meshio::select::{Pick, Roles, Select, apply};

struct Counting;

static BYTES: AtomicUsize = AtomicUsize::new(0);

// SAFETY: every call is forwarded unchanged to the system allocator; the counter only adds.
unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, l: Layout) -> *mut u8 {
        BYTES.fetch_add(l.size(), Ordering::Relaxed);
        // SAFETY: the caller upholds `GlobalAlloc::alloc`'s contract, which `System` shares.
        unsafe { System.alloc(l) }
    }
    unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
        // SAFETY: `p` and `l` come from the matching `alloc` call above.
        unsafe { System.dealloc(p, l) }
    }
    unsafe fn realloc(&self, p: *mut u8, l: Layout, n: usize) -> *mut u8 {
        BYTES.fetch_add(n, Ordering::Relaxed);
        // SAFETY: `p`, `l` and `n` satisfy `GlobalAlloc::realloc`'s contract by the caller.
        unsafe { System.realloc(p, l, n) }
    }
}

#[global_allocator]
static A: Counting = Counting;

/// Bytes allocated by selecting block "A" (the first 3 cells) of a row of `n` cells.
fn cost(n: usize) -> usize {
    let vols: Vec<u64> = (0..n).map(|i| if i < 3 { 1 } else { 2 }).collect();
    let host = rig::row(&vols, &["A", "B"]);
    let sel = Select { blocks: Pick::Names(vec!["A".into()]), ..Select::default() };
    let roles = Roles::by_kind();
    let before = BYTES.load(Ordering::Relaxed);
    let out = apply(&host, &sel, &roles).expect("select");
    let used = BYTES.load(Ordering::Relaxed) - before;
    assert_eq!(MeshRead::cell_count(&out.mesh), 3);
    used
}

#[test]
fn selection_allocation_follows_the_selected_cells() {
    let small = cost(12);
    let big = cost(600);
    // 50 times the host, the same selection: allow a small constant (the sidesets of the
    // host are only read, not copied).
    assert!(big < small + small / 4, "small {small} B, big {big} B");
}
