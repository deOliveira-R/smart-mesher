//! The segment-iterator contract on the analytic stub: chords sum, the majorant is never
//! violated, and surface and delta modes agree (exactly for optical depth, statistically for
//! ratio-tracking transmission).
#![cfg(feature = "stubs")]

use sm_core::{Aabb, Point3, Vector3};
use sm_rt_api::stub::{AnalyticWorld, RectLattice, Shape};
use sm_rt_api::{
    BinMap, EnergyBands, Flight, FlightOptions, FlightTally, MajorantEntry, MajorantTable, MajorantTracker, MaterialSetId, ModePolicy,
    ResponseKind, RtError, Scratch, SegmentScale, TraceEnd, TrackMode, TrackState, Tracker, TrackerCaps, validate_partition,
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
/// 3 x 2 x 1 lattice of pitch 4 with a small sphere (r = 1) and a box in each element.
fn lattice() -> AnalyticWorld {
    let shapes = vec![sphere(1.0), Shape::Box { lo: p(-1.5, -1.5, -1.5), hi: p(1.5, 1.5, 1.5) }];
    let lat = RectLattice::new(p(-6.0, -4.0, -2.0), [4.0, 4.0, 4.0], [3, 2, 1]);
    AnalyticWorld::with_lattice(shapes, lat, [sm_rt_api::FaceAction::Respond(ResponseKind::Vacuum); 6]).unwrap()
}
/// Total cross sections per region (shapes, then background).
const SIGMA: [f64; 3] = [0.5, 0.2, 0.05];

fn sigma_of(_w: &AnalyticWorld, r: usize) -> f64 {
    SIGMA[r.min(SIGMA.len() - 1)]
}

fn table(w: &AnalyticWorld) -> MajorantTable {
    MajorantTable::from_fn(w.hierarchy(), EnergyBands::single(), |_, regions, _| {
        let max = regions.iter().map(|r| sigma_of(w, r.get())).fold(0.0, f64::max);
        let mean = regions.iter().map(|r| sigma_of(w, r.get())).sum::<f64>() / regions.len() as f64;
        MajorantEntry::new(max, mean)
    })
    .unwrap()
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
fn fly(w: &AnalyticWorld, o: Point3, d: Vector3, opts: FlightOptions) -> (f64, f64, u64, bool) {
    let t = table(w);
    let mut sc = Scratch::new();
    let mut st = start(w, o, d, &mut sc);
    let mut fl = Flight::new(w, &mut st, &mut sc, &t, opts).unwrap();
    let (mut sum, mut tau, mut prev, mut contiguous) = (0.0, 0.0, 0.0, true);
    while let Some(seg) = fl.next_segment().unwrap() {
        contiguous &= (seg.t0 - prev).abs() < 1e-12;
        prev = seg.t1;
        sum += seg.length();
        if seg.mode == TrackMode::Surface && seg.resolved {
            tau += seg.length() * sigma_of(w, seg.region.get());
        }
    }
    (sum, tau, fl.stats().delta_segments, contiguous)
}

#[test]
fn stub_declares_the_capability_and_a_valid_partition() {
    for w in [flat(), lattice()] {
        assert!(w.caps().has(TrackerCaps::MAJORANT));
        assert!(w.majorant_view().is_some());
        validate_partition(w.hierarchy()).unwrap();
    }
    let w = lattice();
    assert_eq!(w.hierarchy().node_count(), 7, "six leaves and the lattice root");
    assert!(w.hierarchy().node(w.hierarchy().root()).is_some_and(|n| !n.is_leaf()));
}

#[test]
fn segment_lengths_sum_to_the_chord_in_every_mode() {
    let mut rng = Rng(7);
    for w in [flat(), lattice()] {
        for _ in 0..200 {
            let d = unit(rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5);
            let o = p(rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5);
            let want = chord(&w, o, d);
            for policy in [ModePolicy::Surface, ModePolicy::Delta, ModePolicy::Auto { min_efficiency: 0.5 }] {
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
fn delta_is_one_segment_per_node_and_region_scale_merges() {
    let w = lattice();
    let d = Vector3::new(1.0, 0.0, 0.0);
    let o = p(-5.9, -2.0, 0.0);
    let (_, _, n_delta, _) = fly(&w, o, d, FlightOptions::default().with_policy(ModePolicy::Delta));
    assert_eq!(n_delta, 3, "three lattice slots along x");
    let count = |scale| {
        let t = table(&w);
        let (mut sc, mut n) = (Scratch::new(), 0);
        let mut st = start(&w, o, d, &mut sc);
        let mut fl = Flight::new(&w, &mut st, &mut sc, &t, FlightOptions::default().with_scale(scale)).unwrap();
        while fl.next_segment().unwrap().is_some() {
            n += 1;
        }
        n
    };
    assert!(count(SegmentScale::Region) <= count(SegmentScale::Cell));
}

#[test]
fn surface_optical_depth_is_analytic_and_never_rejects() {
    // Axis ray through the centre of the flat world: 5 of background, 2 of shell, 2 of core.
    let w = flat();
    let (sum, tau, _, _) = fly(&w, p(-4.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), FlightOptions::default());
    assert!((sum - 9.0).abs() < 1e-12);
    assert!((tau - (0.05 * 5.0 + 0.2 * 2.0 + 0.5 * 2.0)).abs() < 1e-12, "tau = {tau}");
    // Surface segments carry their own region's total: efficiency 1, no virtual collisions.
    let t = table(&w);
    let mut sc = Scratch::new();
    let mut st = start(&w, p(-4.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, &t, FlightOptions::default()).unwrap();
    while let Some(seg) = fl.next_segment().unwrap() {
        assert_eq!(seg.sigma_m, sigma_of(&w, seg.region.get()));
        t.check(seg.set, 0, sigma_of(&w, seg.region.get())).unwrap();
    }
    assert!(matches!(fl.end(), Some(TraceEnd::Boundary { .. })));
}

/// Ratio-tracking transmission along a ray, one particle.
fn ratio_tracking(w: &AnalyticWorld, o: Point3, d: Vector3, policy: ModePolicy, rng: &mut Rng) -> (f64, u64) {
    let t = table(w);
    let mut sc = Scratch::new();
    let mut st = start(w, o, d, &mut sc);
    let mut fl = Flight::new(w, &mut st, &mut sc, &t, FlightOptions::default().with_policy(policy)).unwrap();
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
            let sigma_t = sigma_of(w, loc.region.get());
            t.check(seg.set, 0, sigma_t).expect("the majorant must never be violated");
            weight *= 1.0 - sigma_t / seg.sigma_m;
        }
    }
    (weight, fl.stats().probes)
}

#[test]
fn delta_and_surface_modes_agree_on_transmission() {
    let mut rng = Rng(42);
    for w in [flat(), lattice()] {
        let (o, d) = (p(-4.5, 0.3, 0.1), unit(1.0, 0.05, 0.02));
        let (_, tau, _, _) = fly(&w, o, d, FlightOptions::default());
        let want = (-tau).exp();
        for policy in [ModePolicy::Surface, ModePolicy::Delta, ModePolicy::Auto { min_efficiency: 0.1 }] {
            let n = 20_000;
            let (mut sum, mut probes) = (0.0, 0);
            for _ in 0..n {
                let (wt, pr) = ratio_tracking(&w, o, d, policy, &mut rng);
                sum += wt;
                probes += pr;
            }
            let got = sum / f64::from(n);
            assert!((got - want).abs() < 0.01, "{policy:?}: {got} vs {want}");
            if policy == ModePolicy::Surface {
                // Surface weights are 1 - sigma_t/sigma_t = 0 at a "collision": ratio tracking
                // through exact segments scores only if a sample lands inside, so probes happen.
                assert!(probes > 0);
            }
        }
    }
}

#[test]
fn mode_can_switch_mid_flight_without_changing_the_chord() {
    let w = lattice();
    let t = table(&w);
    let (o, d) = (p(-5.5, -3.0, 0.2), unit(1.0, 0.4, 0.0));
    let want = chord(&w, o, d);
    let mut sc = Scratch::new();
    let mut st = start(&w, o, d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, &t, FlightOptions::default()).unwrap();
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
    let w = flat();
    // Delta majorant too small for the core: set "all regions" gets 0.1 < 0.5.
    let bad = MajorantTable::from_fn(w.hierarchy(), EnergyBands::single(), |_, regions, _| {
        if regions.len() > 1 { MajorantEntry::new(0.1, 0.05) } else { MajorantEntry::new(sigma_of(&w, regions[0].get()), sigma_of(&w, regions[0].get())) }
    })
    .unwrap();
    let node_set = MaterialSetId::new(w.hierarchy().set_count() - 1);
    match bad.check(node_set, 0, 0.5) {
        Err(RtError::MajorantViolated { sigma_t, majorant, .. }) => assert_eq!((sigma_t, majorant), (0.5, 0.1)),
        other => panic!("expected a violation, got {other:?}"),
    }
    assert!(bad.check(node_set, 0, 0.1).is_ok());
    assert!(MajorantTable::from_fn(w.hierarchy(), EnergyBands::single(), |_, _, _| MajorantEntry::new(1.0, 2.0)).is_err(), "mean above majorant");
}

#[test]
fn probe_is_hinted_and_stop_at_places_the_state() {
    let w = lattice();
    let t = table(&w);
    let (o, d) = (p(-5.9, -2.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let mut sc = Scratch::new();
    let mut st = start(&w, o, d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, &t, FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
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
fn hybrid_tally_estimators_agree_on_cell_flux() {
    // Track-length (surface segments) versus collision estimator (delta ratio sampling) in the
    // core cell, per unit source weight along one ray.
    let w = flat();
    let (o, d) = (p(-4.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
    let t = table(&w);
    let core = sm_rt_api::CellId::new(0);
    let mut track = FlightTally::new(BinMap::identity(3));
    let mut sc = Scratch::new();
    let mut st = start(&w, o, d, &mut sc);
    let mut fl = Flight::new(&w, &mut st, &mut sc, &t, FlightOptions::default()).unwrap();
    while let Some(seg) = fl.next_segment().unwrap() {
        track.score_track(&seg, 1.0).unwrap();
    }
    assert!((track.flux(0).unwrap() - 2.0).abs() < 1e-12, "core chord is 2");

    // Collision estimator: sample collisions with the node majorant, score w / sigma_t at real
    // and virtual alike with weight sigma_t / sigma_M (expected collision density = flux * sigma_t).
    let (mut rng, n) = (Rng(11), 40_000u32);
    let mut coll = FlightTally::new(BinMap::identity(3));
    for _ in 0..n {
        let mut st = start(&w, o, d, &mut sc);
        let mut fl = Flight::new(&w, &mut st, &mut sc, &t, FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
        let seg = fl.next_segment().unwrap().unwrap();
        let mut x = seg.t0;
        loop {
            x += -rng.next().ln() / seg.sigma_m;
            if x >= seg.t1 {
                break;
            }
            let loc = fl.probe(x).unwrap();
            // Collisions occur at rate sigma_M * flux-of-uncollided; score w / sigma_M per tentative
            // collision, which estimates the (uncollided) track length in expectation.
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
    let a = fly(&w, p(-5.5, -3.5, 0.1), d, FlightOptions::default().with_policy(ModePolicy::Auto { min_efficiency: 0.3 }));
    let b = fly(&w, p(-5.5, -3.5, 0.1), d, FlightOptions::default().with_policy(ModePolicy::Auto { min_efficiency: 0.3 }));
    assert_eq!((a.0.to_bits(), a.2), (b.0.to_bits(), b.2));
}

#[test]
fn bad_inputs_are_typed_errors() {
    let w = flat();
    let t = table(&w);
    let mut sc = Scratch::new();
    let mut st = start(&w, p(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0), &mut sc);
    for opts in [
        FlightOptions::default().with_band(5),
        FlightOptions::default().with_max_dist(-1.0),
        FlightOptions::default().with_policy(ModePolicy::Auto { min_efficiency: 0.0 }),
    ] {
        assert!(matches!(Flight::new(&w, &mut st, &mut sc, &t, opts), Err(RtError::InvalidInput(_))));
    }
}
