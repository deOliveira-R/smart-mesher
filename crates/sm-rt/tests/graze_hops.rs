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

// ---- the world tracker: a mesh-leaf hex lattice of the same two-region fan ------------------

mod world {
    use super::*;
    use sm_core::{Point3, Transform, Vector3};
    use sm_rt::WorldTracker;
    use sm_rt_api::{ComponentId, ComponentKind, Flow, HierInput, InstancePolicy, Lattice, LatticeKind, LeafInput, NodeId, Placement, Segment, TraceEnd, TraceOptions, WorldSpec};

    /// A 2 x 2 x 1 hex lattice (flat to flat 2.5, height 3) of the fan prototype (unit radius, so it
    /// sits inside a hex cell), element 0 centred on the origin. One node holds the whole prototype
    /// (its outer faces are inside it), so the delta flight walks with `step`.
    fn hex_fan_world(ns: usize, odd: usize) -> WorldTracker {
        let mesh = checker_fan(ns, odd);
        let (rules, bnd, comp) = (RegionRule::ByCellSets(vec!["s0".into(), "s1".into()]), BoundarySpec::default(), ComplementPolicy::Outside);
        let leaves = [LeafInput::Mesh(MeshInput::new(&mesh, &rules, &bnd, &comp))];
        let lat = Lattice::new(LatticeKind::HexYRows { pitch: 2.5, z_pitch: 3.0 }, Point3::new(0.0, 0.0, 0.0), [2, 2, 1], vec![ComponentId(1); 4], None);
        let spec = WorldSpec::new(
            vec![
                ComponentKind::Leaf { input: 0 },
                ComponentKind::Union { placements: vec![Placement::new(ComponentId(0), Transform::translation(Vector3::new(0.0, 0.0, -1.5)))] },
                ComponentKind::Lattice(lat),
            ],
            ComponentId(2),
            InstancePolicy::Hierarchical,
        );
        let mut p = PartitionOptions::default();
        p.enabled = true;
        p.leaf_cells = 10_000;
        MeshTrackerBuilder.build_world(&BuildInput::Hier(HierInput::new(&spec, &leaves, &bnd)), &BuildOptions::default().with_partition(p)).expect("world").0
    }

    const WO: [f64; 3] = [-0.5, 0.0, 0.0];
    const CASES: [(usize, usize); 5] = [(24, 0), (24, 1), (24, 11), (24, 12), (48, 23)];

    fn start(w: &WorldTracker, sc: &mut Scratch) -> sm_rt_api::TrackState {
        w.init_state(common::p3(WO), common::v3([1.0, 0.0, 0.0]), None, sc).expect("init")
    }

    /// `fire` along +x up to 0.9: the hop events (no surface, no face) are checked against the state.
    #[test]
    fn world_fire_ends_a_hop_that_changes_region_with_a_region_event() {
        let mut hops = 0;
        for (ns, odd) in CASES {
            let w = hex_fan_world(ns, odd);
            let mut sc = Scratch::new();
            let mut st = start(&w, &mut sc);
            let mut total = 0.0;
            for _ in 0..1000 {
                let r = w.fire(&mut st, 0.9 - total, 100_000, &mut sc).expect("fire");
                total += r.dist;
                match r.end {
                    StepEnd::Limit => break,
                    StepEnd::RegionBoundary { surface, face, to_region, to_cell } => {
                        assert_eq!((st.region, st.cell), (to_region, to_cell), "ns {ns} odd {odd}");
                        hops += usize::from(surface.is_none() && face.is_none());
                    }
                    e => panic!("ns {ns} odd {odd}: fire ended {e:?}"),
                }
            }
            assert!((total - 0.9).abs() < 1e-8, "ns {ns} odd {odd}: {total}");
        }
        assert!(hops > 0, "no hop changed region");
    }

    /// `trace`: the hop is a chord of the cell it left (no face out), the next segment shows the new region.
    #[test]
    fn world_trace_adds_the_hop_chord_and_then_shows_the_new_region() {
        let mut shown = 0;
        for (ns, odd) in CASES {
            let w = hex_fan_world(ns, odd);
            let mut sc = Scratch::new();
            let mut st = start(&w, &mut sc);
            let mut segs: Vec<Segment> = vec![];
            let mut sink = |s: &Segment| {
                segs.push(*s);
                Flow::Continue
            };
            let end = w.trace(&mut st, &TraceOptions::default().with_max_dist(0.9), &mut sink, &mut sc).expect("trace");
            assert!(matches!(end, TraceEnd::Limit), "ns {ns} odd {odd}: {end:?}");
            let total: f64 = segs.iter().map(|s| s.length).sum();
            assert!((total - 0.9).abs() < 1e-8, "ns {ns} odd {odd}: {total}");
            for p in segs.windows(2) {
                let (a, b) = (&p[0], &p[1]);
                assert!((b.t0 - (a.t0 + a.length)).abs() < 1e-12, "contiguous");
                if a.face_out.is_none() && a.region != b.region {
                    shown += 1;
                }
            }
        }
        assert!(shown > 0, "no hop chord followed by a new region");
    }

    /// Delta flight over the axis edge (zero majorants), the same 0.9 long, region events for hops only.
    #[test]
    fn world_delta_flight_ends_a_hop_with_a_region_event() {
        let mut hops = 0;
        for (ns, odd) in CASES {
            let w = hex_fan_world(ns, odd);
            let view = w.majorant_view().expect("partition");
            let maj = vec![0.0; view.hierarchy().node_count()];
            let mut sc = Scratch::new();
            let mut st = start(&w, &mut sc);
            let opts = FlightOptions::default().with_policy(ModePolicy::Delta).with_max_dist(0.9);
            let mut cur = view.begin(&st, &opts).expect("begin");
            let (mut len, mut n) = (0.0, 0);
            while let Some(seg) = cur.next_segment(view, FlightInput::new(&maj), &mut st, &mut sc).expect("segment") {
                len = seg.t1;
                n += 1;
                assert!(n < 10_000, "ns {ns} odd {odd}: a flight that does not end");
                if let StepEnd::RegionBoundary { surface, face, to_region, to_cell } = seg.end
                    && surface.is_none()
                    && face.is_none()
                    && !to_region.is_none()
                {
                    assert_eq!((st.region, st.cell), (to_region, to_cell), "ns {ns} odd {odd}");
                    hops += 1;
                }
            }
            assert!((len - 0.9).abs() < 1e-8, "ns {ns} odd {odd}: {len}");
        }
        assert!(hops > 0, "the delta flight never hopped into another region");
    }

    /// Room under `tol` in a trap ends the walk at its limit, not at `max_steps` (the axis edge costs
    /// `ns - 1` zero-length steps).
    #[test]
    fn world_trap_with_less_than_tol_of_room_ends_at_the_limit() {
        for ns in [24usize, 48] {
            let w = hex_fan_world(ns, 0);
            let lim = 0.5 + 5e-7;
            let mut sc = Scratch::new();
            let mut st = start(&w, &mut sc);
            let r = w.fire(&mut st, lim, 12, &mut sc).expect("fire");
            assert!(matches!(r.end, StepEnd::Limit), "ns {ns} fire: {:?}", r.end);
            assert!((r.dist - lim).abs() < 1e-12, "ns {ns}: {}", r.dist);
            let mut st = start(&w, &mut sc);
            let end = w.trace(&mut st, &TraceOptions::default().with_max_dist(lim).with_max_steps(12), &mut |_: &Segment| Flow::Continue, &mut sc).expect("trace");
            assert!(matches!(end, TraceEnd::Limit), "ns {ns} trace: {end:?}");
            let _ = NodeId::new(0);
        }
    }
}
