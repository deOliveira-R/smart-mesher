//! The segment-iterator contract on the analytic stub: chords sum, the caller's per-node
//! majorants are never violated (and a violation is an error), and surface and delta modes
//! agree (exactly for optical depth, statistically for ratio-tracking transmission).
#![cfg(feature = "stubs")]

use sm_core::{Aabb, Point3, Vector3};
use sm_rt_api::stub::{AnalyticWorld, RectLattice, Shape};
use sm_rt_api::{
    BinMap, Flight, FlightInput, FlightOptions, FlightTally, MajorantTracker, ModePolicy, NodeId, ResponseKind, RtError,
    Scratch, SegmentScale, TraceEnd, TrackMode, TrackState, Tracker, TrackerCaps, validate_partition,
};

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}
fn unit(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::new(x, y, z) * (1.0 / (x * x + y * y + z * z).sqrt())
}
fn sphere(r: f64) -> Shape {
    Shape::Sphere { center: p(0.0, 0.0, 0.0), radius: r }
}
fn domain() -> Aabb {
    Aabb { lo: p(-5.0, -5.0, -5.0), hi: p(5.0, 5.0, 5.0) }
}
/// Core (r = 1), shell (r = 2), background.
fn flat() -> AnalyticWorld {
    AnalyticWorld::simple(vec![sphere(1.0), sphere(2.0)], domain(), ResponseKind::Vacuum).unwrap()
}
/// `flat` with the element split into four leaves along x: the outer two hold only background.
fn gridded() -> AnalyticWorld {
    flat().with_node_grid([4, 1, 1]).unwrap()
}
/// 3 x 2 x 1 lattice of pitch 4 with a small sphere (r = 1) and a box in each element.
fn lattice() -> AnalyticWorld {
    let shapes = vec![sphere(1.0), Shape::Box { lo: p(-1.5, -1.5, -1.5), hi: p(1.5, 1.5, 1.5) }];
    let lat = RectLattice::new(p(-6.0, -4.0, -2.0), [4.0, 4.0, 4.0], [3, 2, 1]);
    AnalyticWorld::with_lattice(shapes, lat, [sm_rt_api::FaceAction::Respond(ResponseKind::Vacuum); 6]).unwrap()
}
/// Total cross sections per region (shapes, then background).
const SIGMA: [f64; 3] = [0.5, 0.2, 0.05];

fn sigma_of(r: usize) -> f64 {
    SIGMA[r.min(SIGMA.len() - 1)]
}

/// The caller's majorant per node: the largest total cross section among the node's regions. The
/// tracker only reports which regions a node holds.
fn majorants(w: &AnalyticWorld) -> Vec<f64> {
    let h = w.hierarchy();
    (0..h.node_count())
        .map(|i| {
            let set = h.node(NodeId::new(i)).unwrap().set;
            h.set_regions(set).unwrap().iter().map(|r| sigma_of(r.get())).fold(0.0, f64::max)
        })
        .collect()
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        ((z >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }
}

fn start(w: &AnalyticWorld, o: Point3, d: Vector3, sc: &mut Scratch) -> TrackState {
    w.init_state(o, d, None, sc).unwrap()
}

/// Reference chord: total distance of `Tracker::trace` to the vacuum boundary.
fn chord(w: &AnalyticWorld, o: Point3, d: Vector3) -> f64 {
    let mut sc = Scratch::new();
    let mut st = start(w, o, d, &mut sc);
    let mut total = 0.0;
    w.trace(&mut st, &Default::default(), &mut |s: &sm_rt_api::Segment| {
        total += s.length;
        sm_rt_api::Flow::Continue
    }, &mut sc)
    .unwrap();
    total
}

/// Run a flight, returning `(sum of lengths, optical depth, delta segments, contiguous)`.
fn fly_with(w: &AnalyticWorld, o: Point3, d: Vector3, opts: FlightOptions, block: &[NodeId]) -> (f64, f64, u64, bool) {
    let m = majorants(w);
    let mut sc = Scratch::new();
    let mut st = start(w, o, d, &mut sc);
    let mut fl = Flight::new(w, &mut st, &mut sc, FlightInput::new(&m).with_block(block), opts).unwrap();
    let (mut sum, mut tau, mut prev, mut contiguous, mut n_delta) = (0.0, 0.0, 0.0, true, 0);
    while let Some(seg) = fl.next_segment().unwrap() {
        contiguous &= (seg.t0 - prev).abs() < 1e-12;
        prev = seg.t1;
        sum += seg.length();
        if seg.mode == TrackMode::Delta {
            n_delta += 1;
        }
        if seg.mode == TrackMode::Surface && seg.resolved {
            tau += seg.length() * sigma_of(seg.region.get());
        }
    }
    (sum, tau, n_delta, contiguous)
}
fn fly(w: &AnalyticWorld, o: Point3, d: Vector3, opts: FlightOptions) -> (f64, f64, u64, bool) {
    fly_with(w, o, d, opts, &[])
}

#[test]
fn stub_declares_the_capability_and_a_valid_partition() {
    for w in [flat(), lattice(), gridded(), lattice().with_node_grid([2, 2, 1]).unwrap()] {
        assert!(w.caps().has(TrackerCaps::MAJORANT));
        assert!(w.majorant_view().is_some());
        validate_partition(w.hierarchy()).unwrap();
        FlightInput::new(&majorants(&w)).validate(w.hierarchy()).unwrap();
    }
    // NodeIds index the prototype tree: six elements share one leaf, plus the lattice node.
    let w = lattice();
    assert_eq!(w.hierarchy().node_count(), 2, "one prototype leaf and the lattice node");
    assert!(w.hierarchy().node(w.hierarchy().root()).is_some_and(|n| !n.is_leaf()));
    assert_eq!(gridded().hierarchy().node_count(), 5, "four leaves and the prototype node");
}

#[test]
fn leaves_report_which_regions_they_hold() {
    let w = gridded();
    let h = w.hierarchy();
    let regions = |k: usize| h.set_regions(h.node(NodeId::new(k)).unwrap().set).unwrap().iter().map(|r| r.get()).collect::<Vec<_>>();
    assert_eq!(regions(0), vec![2], "x in [-5, -2.5] is background only");
    assert_eq!(regions(1), vec![0, 1, 2]);
    assert_eq!(regions(3), vec![2]);
    // The sets are deduplicated: both outer leaves share one set id.
    assert_eq!(h.node(NodeId::new(0)).unwrap().set, h.node(NodeId::new(3)).unwrap().set);
    // Every region has its singleton set.
    assert_eq!(h.set_regions(h.region_set(sm_rt_api::RegionId::new(1)).unwrap()).unwrap().len(), 1);
    // Majorants follow the sets.
    assert_eq!(majorants(&w)[..4], [0.05, 0.5, 0.5, 0.05]);
}

#[test]
fn segment_lengths_sum_to_the_chord_in_every_mode() {
    let mut rng = Rng(7);
    for w in [flat(), lattice(), gridded()] {
        for _ in 0..200 {
            let d = unit(rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5);
            let o = p(rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5);
            let want = chord(&w, o, d);
            for policy in [ModePolicy::Surface, ModePolicy::Delta] {
                for scale in [SegmentScale::Cell, SegmentScale::Region] {
                    let (sum, _, _, contiguous) = fly(&w, o, d, FlightOptions::default().with_policy(policy).with_scale(scale));
                    assert!((sum - want).abs() < 1e-9, "{policy:?} {scale:?}: {sum} vs {want}");
                    assert!(contiguous);
                }
            }
        }
    }
}

#[test]
fn delta_is_one_segment_per_node_instance_and_region_scale_merges() {
    let w = lattice();
    let d = Vector3::new(1.0, 0.0, 0.0);
    let o = p(-5.9, -2.0, 0.0);
    let (_, _, n_delta, _) = fly(&w, o, d, FlightOptions::default().with_policy(ModePolicy::Delta));
    assert_eq!(n_delta, 3, "three lattice slots along x");
    let count = |scale| {
        let m = majorants(&w);
        let (mut sc, mut n) = (Scratch::new(), 0);
        let mut st = start(&w, o, d, &mut sc);
        let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default().with_scale(scale)).unwrap();
        while fl.next_segment().unwrap().is_some() {
            n += 1;
        }
        n
    };
    assert!(count(SegmentScale::Region) <= count(SegmentScale::Cell));
    // Four leaves along x split the one element into four delta segments.
    let g = gridded();
    let (_, _, n, _) = fly(&g, p(-4.9, 0.0, 0.0), d, FlightOptions::default().with_policy(ModePolicy::Delta));
    assert_eq!(n, 4);
}

#[test]
fn block_mixes_surface_and_delta_in_one_flight() {
    let w = gridded();
    let m = majorants(&w);
    let blocked = [NodeId::new(0), NodeId::new(3)];
    let input = FlightInput::new(&m).with_block(&blocked);
    input.validate(w.hierarchy()).unwrap();
    let (mut sc, d) = (Scratch::new(), Vector3::new(1.0, 0.0, 0.0));
    let mut st = start(&w, p(-4.9, 0.0, 0.0), d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, input, FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    let mut seen = Vec::new();
    while let Some(seg) = fl.next_segment().unwrap() {
        seen.push((seg.node.get(), seg.mode, seg.sigma_m));
    }
    use TrackMode::{Delta, Surface};
    assert_eq!(seen, vec![(0, Surface, 0.05), (1, Delta, 0.5), (2, Delta, 0.5), (3, Surface, 0.05)]);
    drop(fl);
    assert_eq!(sc.counters.delta_segments, 2);
    assert_eq!(sc.counters.surface_segments, 2);
    assert_eq!(sc.counters.segments, 4);
}

#[test]
fn surface_optical_depth_is_analytic_and_never_rejects() {
    // Axis ray through the centre of the flat world: 5 of background, 2 of shell, 2 of core.
    let w = flat();
    let (sum, tau, _, _) = fly(&w, p(-4.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), FlightOptions::default());
    assert!((sum - 9.0).abs() < 1e-12);
    assert!((tau - (0.05 * 5.0 + 0.2 * 2.0 + 0.5 * 2.0)).abs() < 1e-12, "tau = {tau}");
    // The caller samples with its own sigma_t(region); the node majorant bounds it.
    let m = majorants(&w);
    let mut sc = Scratch::new();
    let mut st = start(&w, p(-4.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default()).unwrap();
    while let Some(seg) = fl.next_segment().unwrap() {
        assert_eq!(seg.sigma_m, m[seg.node.get()]);
        fl.check_collision(sigma_of(seg.region.get())).unwrap();
    }
    assert!(matches!(fl.end(), Some(TraceEnd::Boundary { .. })));
}

/// Ratio-tracking transmission along a ray, one particle.
fn ratio_tracking(w: &AnalyticWorld, o: Point3, d: Vector3, policy: ModePolicy, rng: &mut Rng, sc: &mut Scratch) -> f64 {
    let m = majorants(w);
    let mut st = start(w, o, d, sc);
    let mut fl = Flight::new(w, &mut st, sc, FlightInput::new(&m), FlightOptions::default().with_policy(policy)).unwrap();
    let mut weight = 1.0;
    while let Some(seg) = fl.next_segment().unwrap() {
        if seg.sigma_m <= 0.0 {
            continue;
        }
        let mut x = seg.t0;
        loop {
            x += -rng.next().ln() / seg.sigma_m;
            if x >= seg.t1 {
                break;
            }
            let loc = fl.probe(x).unwrap();
            let sigma_t = sigma_of(loc.region.get());
            fl.check_collision(sigma_t).expect("the majorant must never be violated");
            weight *= 1.0 - sigma_t / seg.sigma_m;
        }
    }
    weight
}

#[test]
fn delta_and_surface_modes_agree_on_transmission() {
    let mut rng = Rng(42);
    for w in [flat(), lattice(), gridded()] {
        let (o, d) = (p(-4.5, 0.3, 0.1), unit(1.0, 0.05, 0.02));
        let (_, tau, _, _) = fly(&w, o, d, FlightOptions::default());
        let want = (-tau).exp();
        for policy in [ModePolicy::Surface, ModePolicy::Delta] {
            let n = 20_000;
            let mut sc = Scratch::new();
            let mut sum = 0.0;
            for _ in 0..n {
                sum += ratio_tracking(&w, o, d, policy, &mut rng, &mut sc);
            }
            let got = sum / f64::from(n);
            assert!((got - want).abs() < 0.01, "{policy:?}: {got} vs {want}");
            if policy == ModePolicy::Delta {
                assert!(sc.counters.tentative > 0 && sc.counters.locates >= sc.counters.tentative);
            }
        }
    }
}

#[test]
fn mode_can_switch_mid_flight_without_changing_the_chord() {
    let w = lattice();
    let m = majorants(&w);
    let (o, d) = (p(-5.5, -3.0, 0.2), unit(1.0, 0.4, 0.0));
    let want = chord(&w, o, d);
    let mut sc = Scratch::new();
    let mut st = start(&w, o, d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default()).unwrap();
    let (mut sum, mut modes, mut i) = (0.0, Vec::new(), 0);
    while let Some(seg) = fl.next_segment().unwrap() {
        sum += seg.length();
        modes.push(seg.mode);
        i += 1;
        let next = if i % 2 == 0 { ModePolicy::Surface } else { ModePolicy::Delta };
        fl.set_policy(next).unwrap();
    }
    assert!((sum - want).abs() < 1e-9);
    assert!(modes.contains(&TrackMode::Surface) && modes.contains(&TrackMode::Delta));
}

#[test]
fn a_violated_majorant_is_an_error_not_a_clamp() {
    let w = gridded();
    // Leaf 1 holds the core (sigma 0.5) but the caller understates its majorant as 0.1.
    let mut m = majorants(&w);
    m[1] = 0.1;
    let input = FlightInput::new(&m);
    let (mut sc, d) = (Scratch::new(), Vector3::new(1.0, 0.0, 0.0));
    let mut st = start(&w, p(-1.9, 0.0, 0.0), d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, input, FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    assert!(matches!(fl.check_collision(0.0), Err(RtError::InvalidInput(_))), "before the first segment");
    let seg = fl.next_segment().unwrap().unwrap();
    assert_eq!(seg.node, NodeId::new(1));
    // A point in the shell (sigma 0.2) violates 0.1; the error names node, cell and position.
    let loc = fl.probe(seg.t0 + 0.1).unwrap();
    assert_eq!(loc.region.get(), 1);
    match fl.check_collision(sigma_of(loc.region.get())) {
        Err(e @ RtError::MajorantViolated { .. }) => {
            assert_eq!(e.code(), -12);
            let RtError::MajorantViolated { node, sigma_t, majorant, pos, cell } = e else { unreachable!() };
            assert_eq!((node, sigma_t, majorant, cell), (NodeId::new(1), 0.2, 0.1, loc.cell));
            assert!((pos.x - (-1.8)).abs() < 1e-12, "{pos:?}");
        }
        other => panic!("expected a violation, got {other:?}"),
    }
    assert!(fl.check_collision(0.1).is_ok(), "equal to the majorant is fine");
    assert!(fl.check_collision(0.1 * (1.0 + 1e-13)).is_ok(), "rounding slack");
    assert!(matches!(fl.check_collision(f64::NAN), Err(RtError::NonFinite(_))));
    assert!(matches!(fl.check_collision(-1.0), Err(RtError::NonFinite(_))));
    // Nothing advanced: the flight continues from the same segment end.
    assert_eq!(fl.distance(), seg.t1);
}

#[test]
fn check_collision_is_invalid_after_the_end_or_a_stop() {
    let w = flat();
    let m = majorants(&w);
    let (mut sc, d) = (Scratch::new(), Vector3::new(1.0, 0.0, 0.0));
    let mut st = start(&w, p(-4.0, 0.0, 0.0), d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default()).unwrap();
    while fl.next_segment().unwrap().is_some() {}
    assert!(matches!(fl.check_collision(0.0), Err(RtError::InvalidInput(_))));
    let mut st = start(&w, p(-4.0, 0.0, 0.0), d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default()).unwrap();
    let seg = fl.next_segment().unwrap().unwrap();
    fl.stop_at(seg.t0 + 0.1).unwrap();
    assert!(matches!(fl.check_collision(0.0), Err(RtError::InvalidInput(_))));
}

#[test]
fn per_call_validation_reads_one_entry_and_scans_nothing() {
    let w = gridded();
    let (mut sc, d) = (Scratch::new(), Vector3::new(1.0, 0.0, 0.0));
    let ok = majorants(&w);
    // Wrong length: typed error, nothing moves.
    let short = &ok[..ok.len() - 1];
    assert!(matches!(
        Flight::new(&w, &mut start(&w, p(-4.9, 0.0, 0.0), d, &mut sc), &mut Scratch::new(), FlightInput::new(short), FlightOptions::default()),
        Err(RtError::InvalidInput(_))
    ));
    // A bad entry that no call reads is not found per call (O(1)), only by `validate`.
    let mut m = ok.clone();
    m[2] = f64::NAN;
    assert!(matches!(FlightInput::new(&m).validate(w.hierarchy()), Err(RtError::InvalidMajorant { node, .. }) if node == NodeId::new(2)));
    let mut st = start(&w, p(-4.9, 0.0, 0.0), d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    assert!(fl.next_segment().unwrap().is_some(), "leaf 0 is fine");
    assert!(fl.next_segment().unwrap().is_some(), "leaf 1 is fine");
    // Entering leaf 2 reads its bad entry: the error names the node and the state is unchanged.
    let before_pos = fl.cursor().distance();
    match fl.next_segment() {
        Err(RtError::InvalidMajorant { node, .. }) => assert_eq!(node, NodeId::new(2)),
        other => panic!("expected InvalidMajorant, got {other:?}"),
    }
    assert_eq!(fl.cursor().distance(), before_pos);
    // validate also catches an unsorted block.
    let unsorted = [NodeId::new(2), NodeId::new(1)];
    assert!(FlightInput::new(&ok).with_block(&unsorted).validate(w.hierarchy()).is_err());
    // An unsorted block never panics per call.
    let mut st = start(&w, p(-4.9, 0.0, 0.0), d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&ok).with_block(&unsorted), FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    while fl.next_segment().unwrap().is_some() {}
}

#[test]
fn the_cursor_is_opaque_resumable_and_borrows_nothing() {
    let w = gridded();
    let view = w.majorant_view().unwrap();
    let m = majorants(&w);
    let (o, d) = (p(-4.9, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let mut sc = Scratch::new();
    let mut st = start(&w, o, d, &mut sc);
    let mut cur = view.begin(&st, &FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    let mut lens = Vec::new();
    // The cursor outlives every borrow: the slice is passed anew (and may differ) on each call.
    for _ in 0..2 {
        let copy = m.clone();
        lens.push(cur.next_segment(view, FlightInput::new(&copy), &mut st, &mut sc).unwrap().unwrap().length());
    }
    // Resume from a clone (a particle bank copies cursors) and compare with the original.
    let mut twin = cur.clone();
    let (mut st2, mut sc2) = (st.clone(), Scratch::new());
    let a = cur.next_segment(view, FlightInput::new(&m), &mut st, &mut sc).unwrap().unwrap();
    let b = twin.next_segment(view, FlightInput::new(&m), &mut st2, &mut sc2).unwrap().unwrap();
    assert_eq!(a, b);
    assert_eq!(st, st2);
    let (lo, hi) = (a.t0 + 0.1, a.t1 - 0.1);
    let _ = cur.probe(view, FlightInput::new(&m), &mut sc, 0.5 * (lo + hi)).unwrap();
    assert!((lens[0] - 2.4).abs() < 1e-12 && (lens[1] - 2.5).abs() < 1e-12, "{lens:?}");
}

#[test]
fn probe_is_hinted_and_stop_at_places_the_state() {
    let w = lattice();
    let m = majorants(&w);
    let (o, d) = (p(-5.9, -2.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let mut sc = Scratch::new();
    let mut st = start(&w, o, d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    let seg = fl.next_segment().unwrap().unwrap();
    assert_eq!(seg.mode, TrackMode::Delta);
    assert!(fl.probe(seg.t1 + 0.1).is_err(), "outside the segment");
    // Slot 0 spans x in [-6, -2]; its sphere (r = 1) is centred at x = -4, z = 0, y = -2.
    let mid = fl.probe(seg.t0 + 1.9).unwrap();
    assert_eq!(mid.region.get(), 0, "inside the sphere at x = -4");
    let loc = fl.stop_at(seg.t0 + 1.9).unwrap();
    assert_eq!(loc, mid);
    assert!(matches!(fl.end(), Some(TraceEnd::Stopped)));
    assert!(fl.next_segment().unwrap().is_none());
    drop(fl);
    assert!((st.pos.x - (o.x + 1.9)).abs() < 1e-12);
    assert_eq!(st.cell, loc.cell);
    assert_eq!(st.levels, loc.levels);
    // A later element resolves through the level path, not through the (shared) node id.
    let mut st = start(&w, o, d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    let first = fl.next_segment().unwrap().unwrap();
    let second = fl.next_segment().unwrap().unwrap();
    assert_eq!(first.node, second.node, "elements share the prototype's node");
    let in_second = fl.probe(second.t0 + 1.9).unwrap();
    assert_eq!(in_second.region.get(), 0, "inside the second element's sphere at x = 0");
    assert_ne!(in_second.cell, mid.cell);
    assert_eq!(in_second.levels[0].instance.get(), 1);
}

#[test]
fn region_scale_segments_never_span_an_element_wall() {
    // Background on both sides of the wall at x = -2 used to merge into one unresolved segment,
    // whose probe then located the second half with the first element's level path.
    let w = lattice();
    let m = majorants(&w);
    let (o, d) = (p(-5.9, -3.9, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let mut sc = Scratch::new();
    let mut st = start(&w, o, d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default().with_scale(SegmentScale::Region)).unwrap();
    let mut n = 0;
    while let Some(seg) = fl.next_segment().unwrap() {
        for f in [0.1, 0.5, 0.9] {
            let t = seg.t0 + f * (seg.t1 - seg.t0);
            let x = o.x + t;
            let loc = fl.probe(t).unwrap();
            assert_eq!(loc.levels[0].instance.get(), ((x + 6.0) / 4.0).floor() as usize, "x = {x}");
        }
        n += 1;
    }
    assert!(n >= 3);
}

#[test]
fn safety_distance_is_a_lower_bound() {
    let w = flat();
    let mut rng = Rng(3);
    let mut sc = Scratch::new();
    for _ in 0..200 {
        let o = p(10.0 * rng.next() - 5.0, 10.0 * rng.next() - 5.0, 10.0 * rng.next() - 5.0);
        let d = unit(rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5);
        let mut st = start(&w, o, d, &mut sc);
        let safe = w.majorant_view().unwrap().safety_distance(&st, &mut sc).unwrap();
        let first = w.step(&mut st, f64::INFINITY, &mut sc).unwrap();
        assert!(safe <= first.dist + 1e-12, "safety {safe} > distance to the next event {}", first.dist);
    }
}

#[test]
fn the_safety_cache_lives_in_the_cursor_and_stays_a_lower_bound() {
    let w = flat();
    let view = w.majorant_view().unwrap();
    let mut sc = Scratch::new();
    let mut st = start(&w, p(-4.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), &mut sc);
    let mut cur = view.begin(&st, &FlightOptions::default()).unwrap();
    let d0 = cur.safety_distance(view, &st, &mut sc).unwrap();
    assert!(d0 > 0.9, "the sphere surface is 1 away and the wall 1 away, got {d0}");
    assert_eq!(sc.counters.safety_skips, 0);
    // Move a little inside the same cell: answered from the cache, reduced by the distance moved.
    let before = st.clone();
    st.pos = p(-3.8, 0.0, 0.0);
    let d1 = cur.safety_distance(view, &st, &mut sc).unwrap();
    assert_eq!(sc.counters.safety_skips, 1);
    assert!((d1 - (d0 - 0.2)).abs() < 1e-12);
    let exact = view.safety_distance(&st, &mut sc).unwrap();
    assert!(d1 <= exact + 1e-12, "cached {d1} must not exceed the exact bound {exact}");
    // The state type is untouched by the cache: equality compares only the state.
    assert_ne!(before, st);
}

#[test]
fn counters_are_kept_by_clear_and_reset_by_clear_counters() {
    let w = gridded();
    let m = majorants(&w);
    let mut sc = Scratch::new();
    let mut st = start(&w, p(-4.9, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    while fl.next_segment().unwrap().is_some() {}
    drop(fl);
    let c = sc.counters;
    assert_eq!(c.segments, 4);
    assert!(c.steps >= 4 && c.node_visits == 4);
    sc.clear();
    assert_eq!(sc.counters, c, "clear keeps the counters");
    sc.clear_counters();
    assert_eq!(sc.counters, Default::default());
}

#[test]
fn hybrid_tally_estimators_agree_on_cell_flux() {
    // Track-length (surface segments) versus collision estimator (delta ratio sampling) in the
    // core cell, per unit source weight along one ray.
    let w = flat();
    let (o, d) = (p(-4.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let m = majorants(&w);
    let core = sm_rt_api::CellId::new(0);
    let mut track = FlightTally::new(BinMap::identity(3));
    let mut sc = Scratch::new();
    let mut st = start(&w, o, d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default()).unwrap();
    while let Some(seg) = fl.next_segment().unwrap() {
        track.score_track(&seg, 1.0).unwrap();
    }
    assert!((track.flux(0).unwrap() - 2.0).abs() < 1e-12, "core chord is 2");

    // Collision estimator: sample collisions with the node majorant, score w / sigma_M per
    // tentative collision, which estimates the (uncollided) track length in expectation.
    let (mut rng, n) = (Rng(11), 40_000u32);
    let mut coll = FlightTally::new(BinMap::identity(3));
    for _ in 0..n {
        let mut st = start(&w, o, d, &mut sc);
        let mut fl = Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
        let seg = fl.next_segment().unwrap().unwrap();
        let mut x = seg.t0;
        loop {
            x += -rng.next().ln() / seg.sigma_m;
            if x >= seg.t1 {
                break;
            }
            let loc = fl.probe(x).unwrap();
            coll.score_collision(loc.cell, 1.0, seg.sigma_m).unwrap();
        }
    }
    let est = coll.collision[core.get()] / f64::from(n);
    assert!((est - 2.0).abs() < 0.05, "collision-style estimate {est}");
    let mut sum = track.clone();
    sum.merge(&FlightTally::new(BinMap::identity(3))).unwrap();
    assert_eq!(sum.flux(0), track.flux(0));
}

#[test]
fn flights_are_deterministic() {
    let w = lattice();
    let d = unit(1.0, 0.3, 0.1);
    let a = fly(&w, p(-5.5, -3.5, 0.1), d, FlightOptions::default().with_policy(ModePolicy::Delta));
    let b = fly(&w, p(-5.5, -3.5, 0.1), d, FlightOptions::default().with_policy(ModePolicy::Delta));
    assert_eq!((a.0.to_bits(), a.2), (b.0.to_bits(), b.2));
}

#[test]
fn bad_inputs_are_typed_errors() {
    let w = flat();
    let m = majorants(&w);
    let mut sc = Scratch::new();
    let mut st = start(&w, p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), &mut sc);
    assert!(matches!(
        Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m), FlightOptions::default().with_max_dist(-1.0)),
        Err(RtError::InvalidInput(_))
    ));
    assert!(matches!(
        Flight::new(&w, &mut st, &mut sc, FlightInput::new(&m[..0]), FlightOptions::default()),
        Err(RtError::InvalidInput(_))
    ));
}
