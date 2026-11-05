//! Acceptance 13 and the world's acceptance 6: after warm-up, `step`, `locate` and `closest`
//! allocate nothing. The counting allocator counts only on the thread that switched counting
//! on (a const-initialised thread-local flag, which itself never allocates), so tests running
//! in parallel threads of this binary cannot perturb each other's counts. The counting
//! allocator is the only `unsafe` in the crate's tests (each method forwards to `System`).
#![allow(unsafe_code)]
mod common;
use common::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::SerialAlloc;
use sm_rt_api::{CellId, Scratch, StepEnd, Tracker};
use sm_testkit::meshes::nonconvex_poly;
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
        let (again, allocs) = count_allocs(|| workload(t, &mut sc, lo, hi, "acc13_fresh"));
        assert!(again.0 > 200, "workload too small: {} events", again.0);
        eprintln!("{} events, {allocs} allocations", again.0);
        assert_eq!(allocs, 0, "allocations after warm-up");
    }
}

/// Acceptance 6 (world): a 3-level world (lattice of unions of hex pins) walks without
/// allocating once the scratch is warm.
#[test]
fn acc6_world_warm_scratch_means_zero_allocations() {
    use sm_core::{Point3, Transform, Vector3};
    use sm_rt::MeshTrackerBuilder;
    use sm_rt_api::{
        BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, ComponentId, ComponentKind, HierInput, InstancePolicy, Lattice, LatticeKind, LeafInput, MeshInput, Placement,
        RegionRule, TrackerBuilder,
    };
    let pin = hex_block([3; 3], [0.5; 3], &mut SerialAlloc::new()).unwrap();
    let (rules, comp, bnd) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside, BoundarySpec::default());
    let leaves = [LeafInput::Mesh(MeshInput::new(&pin, &rules, &bnd, &comp))];
    let c = |i: u32| ComponentId(i);
    let l = 1.5;
    let spec = sm_rt_api::WorldSpec::new(
        vec![
            ComponentKind::Leaf { input: 0 },
            ComponentKind::Union { placements: vec![Placement::new(c(0), Transform::translation(Vector3::new(-0.75, -0.75, -0.75)))] },
            ComponentKind::Lattice(Lattice::new(LatticeKind::Rect { pitch: [l, l, l] }, Point3::new(0.0, 0.0, 0.0), [4, 4, 1], vec![c(1); 16], None)),
        ],
        c(2),
        InstancePolicy::Hierarchical,
    );
    let (t, _) = MeshTrackerBuilder.build(&BuildInput::Hier(HierInput::new(&spec, &leaves, &bnd)), &BuildOptions::default()).unwrap();
    let mut sc = Scratch::new();
    let _ = workload(&*t, &mut sc, [-0.2; 3], [6.2, 6.2, 1.7], "acc6_warm_a");
    let _ = workload(&*t, &mut sc, [-0.2; 3], [6.2, 6.2, 1.7], "acc6_warm_b");
    let (again, allocs) = count_allocs(|| workload(&*t, &mut sc, [-0.2; 3], [6.2, 6.2, 1.7], "acc6_fresh"));
    assert!(again.0 > 1000, "workload too small: {} events", again.0);
    eprintln!("world: {} events, {allocs} allocations", again.0);
    assert_eq!(allocs, 0, "allocations after warm-up");
}

/// Exactly degenerate crossings (rays through edges and vertices of a Kuhn-tetrahedron lattice)
/// resolve through the allocation-free SoS entry: zero allocations on a facet walk (generic fan crossings).
#[test]
fn acc6_degenerate_crossings_allocate_nothing() {
    let t = build(&tet_facets(), &opts_fail()).unwrap();
    let mut sc = Scratch::new();
    let rays = sm_testkit::rays::degenerate_grid(3);
    let run = |sc: &mut Scratch| {
        let mut events = 0usize;
        for (o, d) in &rays {
            let Ok(mut st) = t.init_state(*o, *d * (1.0 / d.norm()), None, sc) else { continue };
            if st.cell.is_none() {
                continue;
            }
            for _ in 0..50 {
                let s = t.step(&mut st, f64::INFINITY, sc).unwrap();
                events += 1;
                if matches!(s.end, StepEnd::Boundary { .. } | StepEnd::Lost(_)) {
                    break;
                }
            }
        }
        events
    };
    let _ = run(&mut sc);
    let _ = run(&mut sc);
    let (events, allocs) = count_allocs(|| run(&mut sc));
    eprintln!("degenerate: {events} events, {allocs} allocations");
    assert!(events > 200, "{events} events");
    assert_eq!(allocs, 0);
}
