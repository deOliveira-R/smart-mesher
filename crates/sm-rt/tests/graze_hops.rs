//! W4-RT-TIDY: what a grazing-trap hop owes. A hop that changes region ends a `fire`-like walk with
//! a `RegionBoundary` event (no surface, no face); a trap with less than `tol` of room left ends the
//! walk at its limit, not at `max_steps`. Covered in the plain walk, the delta flight and the
//! region-jump walk of the mesh tracker (the world tracker shares the check, see `graze_hops` in
//! `world.rs`).
#[path = "common/mod.rs"]
mod common;

use sm_mesh_api::{CellIx, CellShape, MeshEdit, SerialAlloc, SetMembers};
use sm_rt::{MeshTracker, MeshTrackerBuilder};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, FlightInput, FlightOptions, MeshInput, ModePolicy, PartitionOptions, RegionRule, Scratch, StepEnd, Tracker,
};
use sm_testkit::meshes::{CellSpec, build_mesh};

/// `ns` wedges around the z axis (unit radius), three layers of height 1, the wedges alternately in
/// the cell sets `s0` (all but wedge `odd`) and `s1` (wedge `odd` of each layer): a ray along the
/// axis edge crosses about `ns / 2` zero-length cells of one region (a trap), and the hop out of it
/// can land in the other region.
fn checker_fan(ns: usize, odd: usize) -> sm_mesh_api::reference::ReferenceMesh {
    let nz = 3usize;
    let mut pts = vec![];
    for k in 0..=nz {
        pts.push([0.0, 0.0, k as f64]);
        for i in 0..ns {
            let a = std::f64::consts::TAU * i as f64 / ns as f64;
            pts.push([a.cos(), a.sin(), k as f64]);
        }
    }
    let ix = |k: usize, i: usize| k * (ns + 1) + 1 + (i % ns);
    let ax = |k: usize| k * (ns + 1);
    let mut cells = vec![];
    for k in 0..nz {
        for i in 0..ns {
            cells.push(CellSpec::Template(CellShape::Wedge, vec![ax(k), ix(k, i), ix(k, i + 1), ax(k + 1), ix(k + 1, i), ix(k + 1, i + 1)]));
        }
    }
    let mut mesh = build_mesh(&pts, &cells, &mut SerialAlloc::new()).expect("fan mesh");
    let mut alloc = SerialAlloc::new();
    for r in 0..2 {
        let members: Vec<CellIx> = (0..nz * ns).filter(|c| ((c % ns) == odd) == (r == 1)).map(CellIx::new).collect();
        mesh.add_set(&format!("s{r}"), SetMembers::Cells(members), &mut alloc).expect("set");
    }
    mesh
}

/// The plain, partitioned and region-jump trackers of the same mesh.
fn trackers(mesh: &sm_mesh_api::reference::ReferenceMesh) -> Vec<(&'static str, MeshTracker)> {
    let rule = RegionRule::ByCellSets(vec!["s0".into(), "s1".into()]);
    let (bnd, comp) = (BoundarySpec::default(), ComplementPolicy::Outside);
    let input = BuildInput::Mesh(MeshInput::new(mesh, &rule, &bnd, &comp));
    let mut v = vec![("plain", MeshTrackerBuilder.build_concrete(&input, &BuildOptions::default()).expect("build").0)];
    for (name, jump) in [("partition", false), ("jump", true)] {
        let mut p = PartitionOptions::default();
        p.enabled = true;
        p.leaf_cells = 4;
        p.region_jump = jump;
        v.push((name, MeshTrackerBuilder.build_concrete(&input, &BuildOptions::default().with_partition(p)).expect("build").0));
    }
    v
}

const O: [f64; 3] = [-0.5, 0.0, 1.5];

/// `fire` from `O` along +x until the boundary: every end is checked, and the hops (a
/// `RegionBoundary` with neither surface nor face) are counted. Returns `(length, hops)`.
fn fire_through(t: &dyn Tracker) -> (f64, usize) {
    let mut sc = Scratch::new();
    let mut st = t.init_state(common::p3(O), common::v3([1.0, 0.0, 0.0]), None, &mut sc).expect("init");
    let (mut total, mut hops) = (0.0, 0);
    for _ in 0..1000 {
        let r = t.fire(&mut st, f64::INFINITY, 100_000, &mut sc).expect("fire");
        total += r.dist;
        match r.end {
            StepEnd::Boundary { .. } => return (total, hops),
            StepEnd::RegionBoundary { surface, face, to_region, to_cell } => {
                assert_eq!(st.region, to_region, "the state is in the region the event names");
                assert_eq!(st.cell, to_cell, "and in its cell");
                hops += usize::from(surface.is_none() && face.is_none());
            }
            e => panic!("fire ended {e:?}"),
        }
    }
    panic!("fire never ended");
}

#[test]
fn a_hop_that_changes_region_ends_fire_with_a_region_event_in_every_walk() {
    let mut all = 0;
    for ns in [24usize, 48] {
        for odd in [0, 1, ns / 2 - 1, ns / 2, ns - 1] {
            let mesh = checker_fan(ns, odd);
            for (name, t) in trackers(&mesh) {
                let (len, hops) = fire_through(&t);
                eprintln!("ns {ns} odd {odd} {name}: length {len}, {hops} hop events");
                assert!((len - 1.5).abs() < 1e-8, "ns {ns} {name}: {len}");
                all += hops;
            }
        }
    }
    assert!(all > 0, "no hop changed region in any walk");
}

/// A trap with less than `tol` left (limit 0.5 plus 5e-7, `tol` 1e-6) ends at the limit even when
/// `max_steps` would have run out first (the axis edge costs `ns - 1` zero-length steps).
#[test]
fn a_trap_with_less_than_tol_of_room_ends_at_the_limit_not_at_max_steps() {
    for ns in [24usize, 48] {
        let mesh = checker_fan(ns, 0);
        for (name, t) in trackers(&mesh) {
            let mut sc = Scratch::new();
            let lim = 0.5 + 5e-7;
            let mut st = t.init_state(common::p3(O), common::v3([1.0, 0.0, 0.0]), None, &mut sc).expect("init");
            let r = t.fire(&mut st, lim, 12, &mut sc).expect("fire");
            assert!(matches!(r.end, StepEnd::Limit), "ns {ns} {name}: {:?}", r.end);
            assert!((r.dist - lim).abs() < 1e-12, "ns {ns} {name}: {}", r.dist);
            assert!((st.pos.x - (O[0] + lim)).abs() < 1e-12, "ns {ns} {name}: the state is at the limit");
        }
    }
}

/// The delta and surface flights over the axis edge (zero majorants): carried through to the
/// boundary, the same length as `fire`, with region events only where the region changed.
#[test]
fn the_flights_walk_the_axis_edge_with_region_events_for_hops() {
    let mut delta_hops = 0;
    for ns in [24usize, 48] {
        let mesh = checker_fan(ns, 0);
        let (_, t) = trackers(&mesh).into_iter().nth(1).expect("partition");
        let view = t.majorant_view().expect("partition");
        let maj = vec![0.0; view.hierarchy().node_count()];
        for mode in [ModePolicy::Surface, ModePolicy::Delta] {
            let mut sc = Scratch::new();
            let mut st = t.init_state(common::p3(O), common::v3([1.0, 0.0, 0.0]), None, &mut sc).expect("init");
            let mut cur = view.begin(&st, &FlightOptions::default().with_policy(mode)).expect("begin");
            let (mut len, mut n, mut hops) = (0.0, 0, 0);
            while let Some(seg) = cur.next_segment(view, FlightInput::new(&maj), &mut st, &mut sc).expect("segment") {
                len = seg.t1;
                n += 1;
                assert!(n < 10_000, "ns {ns} {mode:?}: a flight that does not end");
                if let StepEnd::RegionBoundary { to_region, surface, face, .. } = seg.end {
                    hops += usize::from(surface.is_none() && face.is_none() && !to_region.is_none());
                    assert!(to_region.is_none() || st.region == to_region, "ns {ns} {mode:?}: region event names the state's region");
                }
            }
            eprintln!("ns {ns} {mode:?}: {n} segments, {hops} hop events");
            assert!((len - 1.5).abs() < 1e-8, "ns {ns} {mode:?}: length {len}");
            delta_hops += usize::from(mode == ModePolicy::Delta) * hops;
        }
    }
    assert!(delta_hops > 0, "the delta flight never hopped out of a trap into another region");
}
