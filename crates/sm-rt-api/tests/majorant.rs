//! The majorant-hierarchy contract exercised on the analytic stub: segments tile rays and sum to
//! the chord, the majorant is never violated, surface and delta modes agree (exactly on chords,
//! statistically on survival), hinted location stays in its node, and everything is
//! deterministic.
#![cfg(feature = "stubs")]

use sm_core::{Aabb, Point3, Vector3};
use sm_rt_api::MaterialSets;
use sm_rt_api::stub::RectLattice;
use sm_rt_api::stub::{AnalyticWorld, Shape};
use sm_rt_api::{
    BandId, CellId, CostModel, EnergyBands, FaceAction, FlightEnd, FlightEvent, FlightStats, HierarchyOptions, Majorant, MajorantHierarchy,
    MajorantTable, MarchEnd, MarchOptions, MaterialMap, MaterialSetId, NodeId, NodeInfo, NodeKind, NodeMode, NodeTable, RefinePolicy,
    RegionId, ResponseKind, RtError, Scratch, SegmentMode, TraceOptions, TrackState, Tracker, TrackerCaps, sample_flight,
};
use std::sync::Arc;

fn p(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(x, y, z)
}
fn unit(x: f64, y: f64, z: f64) -> Vector3 {
    let n = (x * x + y * y + z * z).sqrt();
    Vector3::new(x / n, y / n, z / n)
}

/// 3 x 2 x 1 elements of pitch 4: sphere r = 1 (region 0), a small box (region 1), background
/// (region 2), one material per region.
fn world(actions: [FaceAction; 6]) -> AnalyticWorld {
    let shapes = vec![Shape::Sphere { center: p(0.0, 0.0, 0.0), radius: 1.0 }, Shape::Box { lo: p(1.2, -0.3, -0.3), hi: p(1.8, 0.3, 0.3) }];
    AnalyticWorld::with_lattice(shapes, RectLattice::new(p(0.0, 0.0, 0.0), [4.0; 3], [3, 2, 1]), actions).unwrap()
}
fn vac() -> [FaceAction; 6] {
    [FaceAction::Respond(ResponseKind::Vacuum); 6]
}

const BASE: [f64; 3] = [0.6, 0.3, 0.05];

/// Bounds per material and band: `Σ_t in [0.9 s, s]` with `s = base * (1 + band)`.
fn xs(m: u32, b: BandId) -> Majorant {
    let s = BASE[m as usize] * (1.0 + b.get() as f64);
    Majorant { min: 0.9 * s, max: s }
}
/// The true cross section at the particle's energy: strictly inside the bounds.
fn sigma(band: BandId) -> impl Fn(RegionId) -> f64 {
    move |r| 0.95 * BASE[r.get()] * (1.0 + band.get() as f64)
}

fn setup() -> (AnalyticWorld, Arc<dyn MajorantHierarchy>, EnergyBands, MajorantTable) {
    let w = world(vac());
    let h = w.majorant_hierarchy(&MaterialMap::identity(3), &HierarchyOptions::default()).unwrap();
    let bands = EnergyBands::new(vec![0.0, 1.0, f64::INFINITY]).unwrap();
    let table = MajorantTable::build(&*h, &bands, &xs, &CostModel::default()).unwrap();
    (w, h, bands, table)
}

/// Deterministic uniforms in `[0, 1)` (xorshift64*, explicit seed).
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        (self.0.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn fan() -> Vec<(Point3, Vector3)> {
    (0..40)
        .map(|i| {
            let (a, b, c) = ((i % 7) as f64 + 1.0, (i % 5) as f64 - 2.0, (i % 3) as f64 * 0.3 - 0.2);
            (p(0.3 + 0.17 * f64::from(i % 9), 0.2 + 0.31 * f64::from(i % 5), 0.4 + 0.11 * f64::from(i % 3)), unit(a, b, c))
        })
        .collect()
}

fn march_all(
    h: &dyn MajorantHierarchy,
    w: &AnalyticWorld,
    o: Point3,
    d: Vector3,
    opts: &MarchOptions,
    table: &MajorantTable,
) -> (Vec<sm_rt_api::MajorantSegment>, Option<MarchEnd>) {
    let mut sc = Scratch::new();
    let st = w.init_state(o, d, None, &mut sc).unwrap();
    let mut v = Vec::new();
    let end = h
        .march(
            &st,
            opts,
            table,
            &mut |s| {
                v.push(*s);
                sm_rt_api::Flow::Continue
            },
            &mut sc,
        )
        .unwrap();
    (v, end)
}

#[test]
fn caps_and_unsupported_default() {
    let (w, h, _, _) = setup();
    assert!(w.caps().has(TrackerCaps::MAJORANT));
    assert_eq!(h.nodes().len(), 1 + 6);
    assert!(h.nodes().validate(h.material_sets()).is_empty());
    // A material map that misses a region is a located build error.
    assert!(matches!(w.majorant_hierarchy(&MaterialMap::new(vec![0, 1]), &HierarchyOptions::default()), Err(RtError::Build(_))));
}

#[test]
fn cell_segments_equal_the_tracker_trace_and_tile_the_ray() {
    let (w, h, _, table) = setup();
    let mut sc = Scratch::new();
    let band = BandId(0);
    for (o, d) in fan() {
        let (segs, end) = march_all(&*h, &w, o, d, &MarchOptions::new(1e9, band).with_refine(RefinePolicy::Cells), &table);
        assert!(matches!(end, Some(MarchEnd::Boundary { .. })), "{end:?}");
        // Same cells and chords as Tracker::trace.
        let mut st = w.init_state(o, d, None, &mut sc).unwrap();
        let mut want = Vec::new();
        w.trace(
            &mut st,
            &TraceOptions::default(),
            &mut |s: &sm_rt_api::Segment| {
                want.push((s.cell, s.length));
                sm_rt_api::Flow::Continue
            },
            &mut sc,
        )
        .unwrap();
        assert_eq!(segs.len(), want.len(), "ray {o:?} {d:?}");
        for (s, (c, l)) in segs.iter().zip(&want) {
            assert_eq!(s.cell, *c);
            assert!((s.length() - l).abs() <= 1e-12 * (1.0 + l), "{} vs {l}", s.length());
            assert_eq!(s.mode, SegmentMode::Surface);
            assert_eq!(h.node_of_cell(s.cell), Some(s.node));
        }
        // Tiling: starts at 0, contiguous bitwise.
        assert_eq!(segs[0].t0, 0.0);
        for pair in segs.windows(2) {
            assert_eq!(pair[0].t1.to_bits(), pair[1].t0.to_bits());
        }
    }
}

#[test]
fn delta_segments_tile_and_sum_to_the_same_chord_as_cells() {
    let (w, h, _, table) = setup();
    for (o, d) in fan() {
        let (cells, _) = march_all(&*h, &w, o, d, &MarchOptions::new(1e9, BandId(0)).with_refine(RefinePolicy::Cells), &table);
        let (delta, end) = march_all(&*h, &w, o, d, &MarchOptions::new(1e9, BandId(0)), &table);
        assert!(matches!(end, Some(MarchEnd::Boundary { .. })));
        let sum = |v: &[sm_rt_api::MajorantSegment]| v.iter().map(|s| s.length()).sum::<f64>();
        assert!((sum(&cells) - sum(&delta)).abs() < 1e-12, "{} vs {}", sum(&cells), sum(&delta));
        assert_eq!(delta[0].t0, 0.0);
        for pair in delta.windows(2) {
            assert_eq!(pair[0].t1.to_bits(), pair[1].t0.to_bits());
        }
        assert!(delta.len() <= cells.len(), "coarse segments never outnumber cell chords");
        assert!(delta.iter().all(|s| s.mode == SegmentMode::Delta && s.cell == CellId::NONE));
        assert_eq!(delta.last().unwrap().t1.to_bits(), cells.last().unwrap().t1.to_bits());
    }
    // A limit inside the model ends exactly there.
    let (o, d) = (p(0.5, 0.3, 0.2), unit(1.0, 0.2, 0.1));
    let (segs, end) = march_all(&*h, &w, o, d, &MarchOptions::new(5.0, BandId(0)), &table);
    assert_eq!(end, Some(MarchEnd::Limit));
    assert_eq!(segs.last().unwrap().t1, 5.0);
}

#[test]
fn the_majorant_bounds_every_cell_below_its_node() {
    let (w, h, bands, table) = setup();
    let mut sc = Scratch::new();
    for band in 0..bands.len() {
        let band = BandId::new(band);
        let truth = sigma(band);
        for (o, d) in fan() {
            let (segs, _) = march_all(&*h, &w, o, d, &MarchOptions::new(1e9, band), &table);
            for s in segs {
                let m = table.get(s.set, band).unwrap();
                for k in 0..=20 {
                    let t = s.t0 + s.length() * (f64::from(k) + 0.37) / 21.0;
                    let q = o + d * t;
                    let loc = h.locate_in(s.node, q, Some(d), &mut sc).unwrap();
                    assert!(!loc.cell.is_none(), "hinted location stays inside the segment's node");
                    assert!(h.material_sets().contains(s.set, loc.region.get() as u32), "set contains the material below");
                    assert!(truth(loc.region) <= m.max && truth(loc.region) >= m.min);
                    table.check(s.set, band, truth(loc.region)).unwrap();
                }
            }
        }
    }
    // A cross section above the majorant is an error, never clamped.
    assert!(matches!(table.check(MaterialSetId(0), BandId(0), 100.0), Err(RtError::MajorantViolation { .. })));
}

#[test]
fn hinted_location_agrees_with_the_tracker_and_never_leaves_its_node() {
    let (w, h, _, _) = setup();
    let mut sc = Scratch::new();
    let q = p(5.0, 0.3, 0.1); // element (1, 0, 0)
    let truth = w.locate(q, None, None, &mut sc).unwrap();
    let node = h.node_of_cell(truth.cell).unwrap();
    assert_eq!(h.locate_in(node, q, None, &mut sc).unwrap().cell, truth.cell);
    assert_eq!(h.locate_in(NodeId(0), q, None, &mut sc).unwrap().cell, truth.cell);
    // The same point asked of another leaf is outside it: NONE, not a fallback to the root.
    let other = NodeId::new(node.get() % 6 + 1);
    assert_ne!(other, node);
    assert!(h.locate_in(other, q, None, &mut sc).unwrap().cell.is_none());
    assert!(matches!(h.locate_in(NodeId(99), q, None, &mut sc), Err(RtError::UnknownId { .. })));
    // Safety distance of a box node is the exact distance to its nearest face.
    assert!((h.safety(node, q) - 0.1).abs() < 1e-15);
    assert!((h.safety(node, p(6.0, 2.0, 2.0)) - 2.0).abs() < 1e-15);
    assert_eq!(h.safety(node, p(0.0, 0.0, 0.0)), 0.0);
}

fn survival(
    h: &dyn MajorantHierarchy,
    w: &AnalyticWorld,
    table: &MajorantTable,
    band: BandId,
    policy: RefinePolicy,
    n: usize,
    seed: u64,
) -> (f64, FlightStats) {
    let mut sc = Scratch::new();
    let (o, d) = (p(0.5, 0.3, 0.2), unit(1.0, 0.2, 0.1));
    let st = w.init_state(o, d, None, &mut sc).unwrap();
    let opts = MarchOptions::new(8.0, band).with_refine(policy);
    let truth = sigma(band);
    let mut rng = Rng(seed);
    let (mut alive, mut stats) = (0usize, FlightStats::default());
    for _ in 0..n {
        match sample_flight(h, &st, &opts, table, &truth, &mut || rng.next(), &mut |_: &FlightEvent<'_>| {}, &mut stats, &mut sc).unwrap() {
            FlightEnd::Ended(MarchEnd::Limit) => alive += 1,
            FlightEnd::Collision { state, dist } => {
                assert!(dist <= 8.0 && !state.cell.is_none());
            }
            other => panic!("{other:?}"),
        }
    }
    (alive as f64 / n as f64, stats)
}

#[test]
fn delta_surface_and_hybrid_flights_agree_with_the_analytic_survival() {
    let (w, h, _, table) = setup();
    let (o, d) = (p(0.5, 0.3, 0.2), unit(1.0, 0.2, 0.1));
    for band in [BandId(0), BandId(1)] {
        // Optical depth from the exact chords.
        let (cells, _) = march_all(&*h, &w, o, d, &MarchOptions::new(8.0, band).with_refine(RefinePolicy::Cells), &table);
        let truth = sigma(band);
        let tau: f64 = cells.iter().map(|s| truth(s.region) * s.length()).sum();
        let want = (-tau).exp();
        let n = 40_000;
        let tol = 4.0 * (want * (1.0 - want) / n as f64).sqrt() + 1e-3;
        let mut seg_count = Vec::new();
        for policy in [RefinePolicy::Cells, RefinePolicy::Coarsest, RefinePolicy::Threshold(0.9), RefinePolicy::PerNode] {
            let (got, stats) = survival(&*h, &w, &table, band, policy, n, 12345 + band.get() as u64);
            assert!((got - want).abs() < tol, "band {band:?} {policy:?}: {got} vs {want} (tol {tol})");
            seg_count.push((policy, stats));
        }
        // Surface mode never rejects; delta mode does.
        assert_eq!(seg_count[0].1.rejected, 0);
        assert_eq!(seg_count[0].1.tentative, 0);
        assert!(seg_count[1].1.tentative > 0 && seg_count[1].1.rejected > 0);
        // Hybrid policies pick surface here (a strong absorber raises the set's rejection bound).
        assert_eq!(seg_count[2].1.tentative, 0);
    }
}

#[test]
fn flights_are_deterministic_and_the_table_is_thread_independent() {
    let (w, h, bands, table) = setup();
    let a = survival(&*h, &w, &table, BandId(0), RefinePolicy::Coarsest, 2000, 7);
    let b = survival(&*h, &w, &table, BandId(0), RefinePolicy::Coarsest, 2000, 7);
    assert_eq!((a.0.to_bits(), a.1), (b.0.to_bits(), b.1));
    let build = || MajorantTable::build(&*h, &bands, &xs, &CostModel::default()).unwrap().content_hash();
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap().install(build);
    assert_eq!(one, build());
    assert_eq!(one, table.content_hash());
    // A different bound changes the hash.
    let other =
        MajorantTable::build(&*h, &bands, &|m: u32, b: BandId| Majorant { min: 0.0, max: xs(m, b).max }, &CostModel::default()).unwrap();
    assert_ne!(other.content_hash(), table.content_hash());
    // The hierarchy hash tracks the material map.
    let h2 = w.majorant_hierarchy(&MaterialMap::new(vec![0, 0, 1]), &HierarchyOptions::default()).unwrap();
    assert_ne!(h2.structure_hash(), h.structure_hash());
    assert_eq!(h.structure_hash(), w.majorant_hierarchy(&MaterialMap::identity(3), &HierarchyOptions::default()).unwrap().structure_hash());
}

#[test]
fn a_violated_majorant_is_an_error_not_a_clamp() {
    let (w, h, bands, _) = setup();
    // Bounds that are too small by a factor 20 for the cross sections used.
    let low = |m: u32, b: BandId| Majorant { min: 0.0, max: xs(m, b).max / 20.0 };
    let table = MajorantTable::build(&*h, &bands, &low, &CostModel::default()).unwrap();
    let mut sc = Scratch::new();
    let st = w.init_state(p(0.5, 0.3, 0.2), unit(1.0, 0.2, 0.1), None, &mut sc).unwrap();
    let opts = MarchOptions::new(8.0, BandId(0));
    let truth = sigma(BandId(0));
    let mut rng = Rng(99);
    let mut hit = false;
    for _ in 0..200 {
        let mut stats = FlightStats::default();
        if let Err(RtError::MajorantViolation { sigma, majorant, .. }) =
            sample_flight(&*h, &st, &opts, &table, &truth, &mut || rng.next(), &mut |_: &FlightEvent<'_>| {}, &mut stats, &mut sc)
        {
            assert!(sigma > majorant);
            hit = true;
            break;
        }
    }
    assert!(hit);
    // Malformed bounds are rejected at build time.
    let bad = |_: u32, _: BandId| Majorant { min: 2.0, max: 1.0 };
    assert!(matches!(MajorantTable::build(&*h, &bands, &bad, &CostModel::default()), Err(RtError::InvalidInput(_))));
}

#[test]
fn boundary_end_finishes_with_the_action_applied() {
    let mut acts = vac();
    acts[1] = FaceAction::Reflect; // +x
    let w = world(acts);
    let h = w.majorant_hierarchy(&MaterialMap::identity(3), &HierarchyOptions::default()).unwrap();
    let bands = EnergyBands::single();
    let table = MajorantTable::build(&*h, &bands, &|m: u32, _: BandId| xs(m, BandId(0)), &CostModel::default()).unwrap();
    let mut sc = Scratch::new();
    let st = w.init_state(p(0.5, 0.3, 0.2), unit(1.0, 0.2, 0.1), None, &mut sc).unwrap();
    let opts = MarchOptions::new(1e9, BandId(0));
    let mut cur = h.begin(&st, &opts, &table, &mut sc).unwrap();
    let mut last = 0.0;
    while let Some(s) = h.next(&mut cur, &table, &mut sc).unwrap() {
        last = s.t1;
    }
    let Some(MarchEnd::Boundary { action, .. }) = cur.end().copied() else { panic!("{:?}", cur.end()) };
    assert_eq!(action, FaceAction::Reflect);
    // The ray hits x = 12 at t = (12 - 0.5) / dir.x.
    let want = (12.0 - 0.5) / unit(1.0, 0.2, 0.1).x;
    assert!((last - want).abs() < 1e-12);
    let st2 = h.finish(&cur, &mut sc).unwrap();
    assert!(st2.dir.x < 0.0 && (st2.pos.x - 12.0).abs() < 1e-12 && !st2.cell.is_none());
    // Walking on from the finished state with a surface march works.
    let st3: TrackState = st2;
    let (segs, _) = march_all(&*h, &w, st3.pos, st3.dir, &MarchOptions::new(1.0, BandId(0)).with_refine(RefinePolicy::Cells), &table);
    assert!(!segs.is_empty());
}

#[test]
fn start_outside_is_lost_and_bad_options_are_typed_errors() {
    let (w, h, _, table) = setup();
    let mut sc = Scratch::new();
    let st = TrackState::new(p(100.0, 0.0, 0.0), unit(1.0, 0.0, 0.0));
    let mut cur = h.begin(&st, &MarchOptions::new(1.0, BandId(0)), &table, &mut sc).unwrap();
    assert!(h.next(&mut cur, &table, &mut sc).unwrap().is_none());
    assert!(matches!(cur.end(), Some(MarchEnd::Lost(_))));
    let inside = w.init_state(p(0.5, 0.3, 0.2), unit(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    assert!(matches!(h.begin(&inside, &MarchOptions::new(-1.0, BandId(0)), &table, &mut sc), Err(RtError::InvalidInput(_))));
    assert!(matches!(h.begin(&inside, &MarchOptions::new(1.0, BandId(9)), &table, &mut sc), Err(RtError::UnknownId { .. })));
    // The segment cap ends a march with its own reason.
    let mut cur = h
        .begin(&inside, &MarchOptions::new(1e9, BandId(0)).with_refine(RefinePolicy::Cells).with_max_segments(2), &table, &mut sc)
        .unwrap();
    while h.next(&mut cur, &table, &mut sc).unwrap().is_some() {}
    assert_eq!(cur.end(), Some(&MarchEnd::MaxSegments));
}

#[test]
fn per_node_policy_follows_the_build_time_decision_per_band() {
    let (_, h, bands, _) = setup();
    // Band 0 optically thin in an element (Σ_M * l ~ 0.6 * 4 = 2.4 vs 7^(1/3) = 1.9 crossings):
    // make delta cheap by pricing locations low; band 1 doubles Σ and we price them high.
    let cheap = CostModel::new(0.95, 0.25, 1.0);
    let t = MajorantTable::build(&*h, &bands, &xs, &cheap).unwrap();
    let leaf = NodeId(1);
    let set = h.nodes().get(leaf).unwrap().set;
    assert_eq!(t.mode(leaf, BandId(0)), Some(NodeMode::Delta));
    assert!(RefinePolicy::PerNode.stop_at(&t, leaf, set, BandId(0)));
    let dear = CostModel::new(0.95, 4.0, 1.0);
    let t2 = MajorantTable::build(&*h, &bands, &xs, &dear).unwrap();
    assert_eq!(t2.mode(leaf, BandId(1)), Some(NodeMode::Surface));
    assert!(!RefinePolicy::PerNode.stop_at(&t2, leaf, set, BandId(1)));
    assert!(RefinePolicy::Coarsest.stop_at(&t2, leaf, set, BandId(1)) && !RefinePolicy::Cells.stop_at(&t2, leaf, set, BandId(1)));
    // Threshold is the rejection bound of the set: 1 - 0.045/0.6 = 0.925 in band 0 (the weak
    // material 2 sets the floor), so Serpent's 0.9 refines and 0.93 stops.
    assert!(RefinePolicy::Threshold(0.93).stop_at(&t2, leaf, set, BandId(0)));
    assert!(!RefinePolicy::Threshold(0.9).stop_at(&t2, leaf, set, BandId(0)));
}

#[test]
fn validate_rejects_overlapping_and_non_tiling_children() {
    let b = |x0: f64, x1: f64| Aabb { lo: p(x0, 0.0, 0.0), hi: p(x1, 1.0, 1.0) };
    let mut sets = MaterialSets::new();
    let s = sets.intern(&[0]);
    let parent = NodeInfo::new(b(0.0, 2.0), s, 4).with_kind(NodeKind::Partition).with_children(NodeId(1), 2);
    let good = NodeTable(vec![
        parent.clone(),
        NodeInfo::new(b(0.0, 1.0), s, 2).with_parent(NodeId(0)),
        NodeInfo::new(b(1.0, 2.0), s, 2).with_parent(NodeId(0)),
    ]);
    assert!(good.validate(&sets).is_empty());
    let overlap = NodeTable(vec![
        parent.clone(),
        NodeInfo::new(b(0.0, 1.2), s, 2).with_parent(NodeId(0)),
        NodeInfo::new(b(1.0, 2.0), s, 2).with_parent(NodeId(0)),
    ]);
    let codes: Vec<_> = overlap.validate(&sets).iter().map(|i| i.code).collect();
    assert!(codes.contains(&"majorant.siblings_overlap"), "{codes:?}");
    let gap = NodeTable(vec![
        parent,
        NodeInfo::new(b(0.0, 0.8), s, 2).with_parent(NodeId(0)),
        NodeInfo::new(b(1.0, 2.0), s, 2).with_parent(NodeId(0)),
    ]);
    assert!(gap.validate(&sets).iter().any(|i| i.code == "majorant.not_a_tiling"));
}
