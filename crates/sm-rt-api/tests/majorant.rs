//! Contract tests for the majorant / delta-tracking extension, run on the stub.
#![cfg(feature = "stubs")]

use sm_core::{Aabb, Point3, Vector3};
use sm_rt_api::stub::{AnalyticWorld, RectLattice, Shape};
use sm_rt_api::{
    BandId, FlightConfig, CollisionOracle, ContentHasher, EnergyBands, FaceAction, Flow, FlightCounters, FlightOutcome, FlightScratch, HybridPolicy,
    MajorantEntry, MajorantSegment, MajorantTable, MajorantTracker, ResponseKind, RtError, Scratch, Segment, TentativeHit, TraceEnd,
    TraceOptions, TrackMode, Tracker,
};

/// sigma_t per local region: sphere r=1 (strong), box (weak), background (void-ish).
const SIG: [f64; 3] = [4.0, 0.5, 0.05];

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}
fn unit(x: f64, y: f64, z: f64) -> Vector3 {
    let n = (x * x + y * y + z * z).sqrt();
    Vector3::new(x / n, y / n, z / n)
}
fn shapes() -> Vec<Shape> {
    vec![Shape::Sphere { center: p(0.0, 0.0, 0.0), radius: 1.0 }, Shape::Box { lo: p(1.2, -0.3, -0.3), hi: p(1.8, 0.3, 0.3) }]
}
fn lattice_world() -> AnalyticWorld {
    AnalyticWorld::with_lattice(shapes(), RectLattice::new(p(0.0, 0.0, 0.0), [4.0, 4.0, 4.0], [3, 2, 1]), [FaceAction::Respond(ResponseKind::Vacuum); 6]).unwrap()
}
fn flat_world() -> AnalyticWorld {
    AnalyticWorld::simple(shapes(), Aabb { lo: p(-5.0, -5.0, -5.0), hi: p(5.0, 5.0, 5.0) }, ResponseKind::Vacuum).unwrap()
}
fn table_with(w: &AnalyticWorld, scale: f64) -> MajorantTable {
    MajorantTable::from_fn(w.majorant_structure(), EnergyBands::single(), |regs, _| {
        let max = regs.iter().map(|r| SIG[r.get()]).fold(0.0, f64::max) * scale;
        let mean = regs.iter().map(|r| SIG[r.get()]).sum::<f64>() / regs.len() as f64;
        MajorantEntry::new(max, mean.min(max))
    })
    .unwrap()
}

/// SplitMix64, seeded from the particle index: reproducible for any thread count.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    }
}
struct Oracle<'a> {
    rng: Rng,
    max_ratio: f64,
    virtual_seen: u32,
    scale_sigma: f64,
    _w: &'a AnalyticWorld,
}
impl CollisionOracle for Oracle<'_> {
    fn uniform(&mut self) -> f64 {
        self.rng.next()
    }
    fn sigma_t(&mut self, at: &TentativeHit) -> f64 {
        let s = SIG[at.region.get()] * self.scale_sigma;
        self.max_ratio = self.max_ratio.max(s / at.sigma_m.max(f64::MIN_POSITIVE));
        s
    }
    fn tentative(&mut self, _a: &TentativeHit, _s: f64, accepted: bool) {
        if !accepted {
            self.virtual_seen += 1;
        }
    }
}

fn segments(w: &AnalyticWorld, t: &MajorantTable, pol: &HybridPolicy, o: Point3, d: Vector3) -> (Vec<MajorantSegment>, TraceEnd) {
    let mut sc = Scratch::new();
    let mut st = w.init_state(o, d, None, &mut sc).unwrap();
    let mut fs = FlightScratch::new();
    let mut out = Vec::new();
    let end = w
        .majorant_segments(
            &mut st,
            &FlightConfig::new(BandId(0), t, pol),
            &mut |s: &MajorantSegment| {
                out.push(*s);
                Flow::Continue
            },
            &mut fs,
        )
        .unwrap();
    (out, end)
}

fn fan(n: usize) -> Vec<(Point3, Vector3)> {
    (0..n)
        .map(|i| {
            let f = i as f64 / n as f64;
            let a = 0.1 + 2.9 * f;
            (p(0.2, 0.3 + 3.0 * f, 0.2 + 0.1 * f), unit(a.cos(), a.sin() * 0.4, 0.1))
        })
        .collect()
}

#[test]
fn segments_partition_the_ray_in_every_mode_and_sum_to_the_chord() {
    for w in [lattice_world(), flat_world()] {
        let t = table_with(&w, 1.0);
        let mut sc = Scratch::new();
        for (o, d) in fan(40) {
            // Reference chord from the plain cell walk.
            let mut st = w.init_state(o, d, None, &mut sc).unwrap();
            let mut chord = 0.0;
            w.trace(&mut st, &TraceOptions::default(), &mut |s: &Segment| {
                chord += s.length;
                Flow::Continue
            }, &mut sc)
            .unwrap();
            for pol in [HybridPolicy::surface_only(), HybridPolicy::delta_only(), HybridPolicy::default()] {
                let (segs, end) = segments(&w, &t, &pol, o, d);
                assert!(matches!(end, TraceEnd::Boundary { .. }));
                let sum: f64 = segs.iter().map(|s| s.length).sum();
                assert!((sum - chord).abs() <= 1e-9 * chord.max(1.0), "{sum} vs {chord}");
                let mut t0 = 0.0;
                for s in &segs {
                    assert!((s.t0 - t0).abs() <= 1e-9, "segments must be contiguous");
                    t0 += s.length;
                    assert!(s.sigma_m >= 0.0);
                    if s.mode == TrackMode::Surface {
                        assert!(s.exact && !s.cell.is_none());
                        assert_eq!(s.sigma_m, SIG[s.region.get()], "surface majorant is the cell's own sigma_t");
                    } else {
                        assert!(s.cell.is_none());
                    }
                }
            }
        }
    }
}

#[test]
fn delta_node_majorant_bounds_every_region_it_covers() {
    let w = lattice_world();
    let t = table_with(&w, 1.0);
    let (segs, _) = segments(&w, &t, &HybridPolicy::delta_only(), p(0.2, 0.1, 0.0), unit(1.0, 0.3, 0.0));
    assert!(segs.len() >= 2 && segs.iter().all(|s| s.mode == TrackMode::Delta));
    assert!(segs.iter().all(|s| s.sigma_m == 4.0 && !s.exact));
    // Auto policy: the lattice element has p_rej = 1 - mean/max = 1 - 1.5167/4 = 0.62 < 0.9, so delta.
    let (auto, _) = segments(&w, &t, &HybridPolicy::default(), p(0.2, 0.1, 0.0), unit(1.0, 0.3, 0.0));
    assert!(auto.iter().all(|s| s.mode == TrackMode::Delta));
    // A strict threshold falls back to surface tracking everywhere.
    let strict = HybridPolicy::default().with_threshold(0.1);
    assert!(segments(&w, &t, &strict, p(0.2, 0.1, 0.0), unit(1.0, 0.3, 0.0)).0.iter().all(|s| s.mode == TrackMode::Surface));
}

fn transmit_fraction(w: &AnalyticWorld, t: &MajorantTable, pol: &HybridPolicy, n: usize) -> (f64, FlightCounters, f64) {
    let (o, d) = (p(-4.9, 0.1, 0.05), unit(1.0, 0.0, 0.0));
    let mut sc = Scratch::new();
    let mut fs = FlightScratch::new();
    let (mut leaked, mut ratio) = (0usize, 0.0f64);
    for i in 0..n {
        let mut st = w.init_state(o, d, None, &mut sc).unwrap();
        let mut orc = Oracle { rng: Rng(1 + i as u64), max_ratio: 0.0, virtual_seen: 0, scale_sigma: 1.0, _w: w };
        match w.fly(&mut st, &FlightConfig::new(BandId(0), t, pol), &mut orc, None, &mut fs).unwrap() {
            FlightOutcome::End { end: TraceEnd::Boundary { .. }, .. } => leaked += 1,
            FlightOutcome::End { end, .. } => panic!("{end:?}"),
            FlightOutcome::Collision { .. } => {}
        }
        ratio = ratio.max(orc.max_ratio);
    }
    (leaked as f64 / n as f64, fs.counters, ratio)
}

#[test]
fn delta_and_surface_modes_agree_with_the_analytic_transmission() {
    let w = flat_world();
    let t = table_with(&w, 1.0);
    // Analytic optical depth along y=0.1, z=0.05 from the exact segments of the plain walk.
    let mut sc = Scratch::new();
    let mut st = w.init_state(p(-4.9, 0.1, 0.05), unit(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    let mut tau = 0.0;
    w.trace(&mut st, &TraceOptions::default(), &mut |s: &Segment| {
        tau += SIG[s.region.get()] * s.length;
        Flow::Continue
    }, &mut sc)
    .unwrap();
    let exact = (-tau).exp();
    let n = 30_000;
    let sigma = (exact * (1.0 - exact) / n as f64).sqrt();
    let (fd, cd, ratio) = transmit_fraction(&w, &t, &HybridPolicy::delta_only(), n);
    let (fs_, cs, _) = transmit_fraction(&w, &t, &HybridPolicy::surface_only(), n);
    assert!((fd - exact).abs() < 5.0 * sigma, "delta {fd} vs {exact}");
    assert!((fs_ - exact).abs() < 5.0 * sigma, "surface {fs_} vs {exact}");
    assert!(ratio <= 1.0, "sigma_t never exceeds the majorant (max ratio {ratio})");
    assert!(cd.rejected > 0 && cd.locates == cd.tentative && cd.surface_segments == 0, "{cd:?}");
    assert_eq!((cs.rejected, cs.locates, cs.delta_segments), (0, 0, 0), "surface mode never rejects or relocates: {cs:?}");
}

#[test]
fn track_length_sink_gets_exactly_the_flown_surface_chords() {
    let w = flat_world();
    let t = table_with(&w, 1.0);
    let (mut sc, mut fs) = (Scratch::new(), FlightScratch::new());
    for i in 0..200 {
        let mut st = w.init_state(p(-4.9, 0.1, 0.05), unit(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        let mut orc = Oracle { rng: Rng(7 + i), max_ratio: 0.0, virtual_seen: 0, scale_sigma: 1.0, _w: &w };
        let mut flown = 0.0;
        let mut sink = |s: &Segment| {
            flown += s.length;
            Flow::Continue
        };
        let out = w.fly(&mut st, &FlightConfig::new(BandId(0), &t, &HybridPolicy::surface_only()), &mut orc, Some(&mut sink), &mut fs).unwrap();
        let dist = match out {
            FlightOutcome::Collision { hit, .. } => hit.dist,
            FlightOutcome::End { dist, .. } => dist,
        };
        assert!((flown - dist).abs() <= 1e-9, "flown {flown} vs flight distance {dist}");
        assert!((st.pos.x + 4.9 - dist).abs() <= 1e-9, "state ends where the flight ended");
    }
}

#[test]
fn an_understated_majorant_is_an_error_not_a_clamp() {
    let w = flat_world();
    let t = table_with(&w, 0.5);
    let (mut sc, mut fs) = (Scratch::new(), FlightScratch::new());
    let mut hit_err = false;
    for i in 0..200 {
        let mut st = w.init_state(p(-4.9, 0.1, 0.05), unit(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        let mut orc = Oracle { rng: Rng(100 + i), max_ratio: 0.0, virtual_seen: 0, scale_sigma: 1.0, _w: &w };
        if let Err(e) = w.fly(&mut st, &FlightConfig::new(BandId(0), &t, &HybridPolicy::delta_only()), &mut orc, None, &mut fs) {
            assert!(matches!(e, RtError::MajorantViolated { .. }), "{e}");
            assert_eq!(e.code(), -12);
            hit_err = true;
            break;
        }
    }
    assert!(hit_err);
}

#[test]
fn tables_are_bound_to_their_structure() {
    let (a, b) = (flat_world(), lattice_world());
    let t = table_with(&a, 1.0);
    let (mut sc, mut fs) = (Scratch::new(), FlightScratch::new());
    let mut st = b.init_state(p(0.5, 0.5, 0.5), unit(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    let r = b.next_tentative(&mut st, &FlightConfig::new(BandId(0), &t, &HybridPolicy::default()), 0.5, None, &mut fs);
    assert!(matches!(r, Err(RtError::CacheStale(_))));
    assert!(matches!(a.next_tentative(&mut a.init_state(p(0.5, 0.5, 0.5), unit(1.0, 0.0, 0.0), None, &mut sc).unwrap(), &FlightConfig::new(BandId(0), &t, &HybridPolicy::default()), 0.0, None, &mut fs), Err(RtError::InvalidInput(_))));
}

#[test]
fn a_homogeneous_world_is_one_exact_segment_and_never_rejects() {
    let w = AnalyticWorld::simple(vec![], Aabb { lo: p(-5.0, -5.0, -5.0), hi: p(5.0, 5.0, 5.0) }, ResponseKind::Vacuum).unwrap();
    let t = MajorantTable::from_fn(w.majorant_structure(), EnergyBands::single(), |_, _| MajorantEntry::new(2.0, 2.0)).unwrap();
    let (segs, _) = segments(&w, &t, &HybridPolicy::delta_only(), p(-4.0, 0.0, 0.0), unit(1.0, 0.0, 0.0));
    assert_eq!(segs.len(), 1);
    assert!(segs[0].exact && segs[0].mode == TrackMode::Delta);
    let (mut sc, mut fs) = (Scratch::new(), FlightScratch::new());
    for i in 0..100 {
        let mut st = w.init_state(p(-4.0, 0.0, 0.0), unit(1.0, 0.0, 0.0), None, &mut sc).unwrap();
        struct O(Rng);
        impl CollisionOracle for O {
            fn uniform(&mut self) -> f64 {
                self.0.next()
            }
            fn sigma_t(&mut self, _: &TentativeHit) -> f64 {
                2.0
            }
        }
        let r = w.fly(&mut st, &FlightConfig::new(BandId(0), &t, &HybridPolicy::delta_only()), &mut O(Rng(i)), None, &mut fs).unwrap();
        assert!(matches!(r, FlightOutcome::Collision { rejected: 0, .. } | FlightOutcome::End { rejected: 0, .. }));
    }
}

fn run_all(threads: usize) -> (u128, FlightCounters) {
    let w = lattice_world();
    let t = table_with(&w, 1.0);
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
    use rayon::prelude::*;
    let per: Vec<(f64, u32, FlightCounters)> = pool.install(|| {
        (0..2000u64)
            .into_par_iter()
            .map(|i| {
                let (mut sc, mut fs) = (Scratch::new(), FlightScratch::new());
                let mut rng = Rng(i);
                let (o, d) = (p(0.1 + 11.0 * rng.next(), 0.1 + 7.0 * rng.next(), 2.0), unit(rng.next() - 0.5, rng.next() - 0.5, rng.next() - 0.5));
                let mut st = w.init_state(o, d, None, &mut sc).unwrap();
                let mut orc = Oracle { rng, max_ratio: 0.0, virtual_seen: 0, scale_sigma: 1.0, _w: &w };
                let r = w.fly(&mut st, &FlightConfig::new(BandId(0), &t, &HybridPolicy::default()), &mut orc, None, &mut fs).unwrap();
                let (dist, rej) = match r {
                    FlightOutcome::Collision { hit, rejected } => (hit.dist, rejected),
                    FlightOutcome::End { dist, rejected, .. } => (-dist, rejected),
                };
                (dist, rej, fs.counters)
            })
            .collect()
    });
    let mut h = ContentHasher::new();
    let mut c = FlightCounters::default();
    for (d, r, k) in &per {
        h.write_f64(*d);
        h.write_u64(u64::from(*r));
        c.merge(k);
    }
    (h.finish().0, c)
}

#[test]
fn flights_are_bit_identical_for_one_and_many_threads() {
    let (h1, c1) = run_all(1);
    let (h4, c4) = run_all(4);
    let (h4b, _) = run_all(4);
    assert_eq!((h1, c1), (h4, c4));
    assert_eq!(h4, h4b);
    assert!(c1.tentative > 0);
}

#[test]
fn locate_in_node_refuses_points_outside_the_node() {
    let w = lattice_world();
    let mut fs = FlightScratch::new();
    let n1 = sm_rt_api::NodeId(1);
    assert!(w.locate_in_node(n1, p(0.1, 0.1, 0.1), None, &mut fs).is_ok());
    assert!(matches!(w.locate_in_node(n1, p(5.0, 0.1, 0.1), None, &mut fs), Err(RtError::InvalidInput(_))));
    assert!(matches!(w.locate_in_node(sm_rt_api::NodeId(99), p(0.1, 0.1, 0.1), None, &mut fs), Err(RtError::UnknownId { .. })));
}
