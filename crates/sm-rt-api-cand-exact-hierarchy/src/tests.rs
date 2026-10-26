use crate::stub::{AnalyticWorld, StubShape};
use crate::*;
use sm_core::{Aabb, Point3, Vector3};
use sm_geom_api::{ModelTolerance, Ray};
use sm_model_api::{Deck, DeckSide};
use sm_topo_api::Region;

const R: f64 = 2.0;

fn opts() -> RtOptions {
    RtOptions::new(ModelTolerance::DEFAULT)
}

fn world_box() -> Aabb {
    Aabb { lo: Point3::new(-5.0, -5.0, -5.0), hi: Point3::new(5.0, 5.0, 5.0) }
}

fn sphere_world(b: BoundarySpec) -> AnalyticWorld {
    AnalyticWorld::new(world_box(), vec![StubShape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: R }], b, opts())
}

fn trace(w: &dyn Tracker, o: [f64; 3], d: [f64; 3], limit: f64) -> (Vec<Segment>, TraceEnd) {
    let mut segs = Vec::new();
    let mut sc = Scratch::new();
    let ray = Ray { origin: Point3::new(o[0], o[1], o[2]), dir: Vector3::new(d[0], d[1], d[2]) };
    let end = w.trace(&ray, limit, &mut segs, &mut sc).expect("trace");
    (segs, end)
}

fn sphere_vol() -> Region {
    Region::Volume(AnalyticWorld::volume(0))
}

#[test]
fn chord_through_sphere_center_is_exact() {
    let w = sphere_world(BoundarySpec::new());
    let (segs, end) = trace(&w, [-4.0, 0.0, 0.0], [1.0, 0.0, 0.0], 1e9);
    assert_eq!(segs.len(), 3);
    assert_eq!(segs[1].at.region, sphere_vol());
    assert!((segs[1].length - 2.0 * R).abs() <= 1e-14, "{}", segs[1].length);
    assert_eq!(segs[0].at.region, Region::Exterior);
    assert!((segs[0].length - 2.0).abs() <= 1e-14 && (segs[2].length - 3.0).abs() <= 1e-14);
    assert!(matches!(end, TraceEnd::Response { kind: ResponseKind::Vacuum, .. }));
}

#[test]
fn offset_chords_match_the_analytic_formula() {
    let w = sphere_world(BoundarySpec::new());
    for k in 0..40 {
        let d = 0.05 * f64::from(k); // impact parameter 0 .. 1.95
        // Tilted ray: direction (1, 0.3, 0.2); offset applied perpendicular to it.
        let dir = [1.0, 0.3, 0.2];
        let n = (1.0f64 + 0.09 + 0.04).sqrt();
        let u = [dir[0] / n, dir[1] / n, dir[2] / n];
        // Perpendicular vector: u x z, normalized.
        let (px, py, pz) = (u[1], -u[0], 0.0);
        let pn = (px * px + py * py + pz * pz).sqrt();
        let o = [-4.0 * u[0] + d * px / pn, -4.0 * u[1] + d * py / pn, -4.0 * u[2] + d * pz / pn];
        let (segs, _) = trace(&w, o, dir, 1e9);
        let chord: f64 = segs.iter().filter(|s| s.at.region == sphere_vol()).map(|s| s.length).sum();
        let exact = 2.0 * (R * R - d * d).sqrt();
        assert!((chord - exact).abs() <= 1e-12 * exact.max(1.0), "d={d}: {chord} vs {exact}");
    }
}

#[test]
fn locate_on_the_surface_uses_the_direction() {
    let w = sphere_world(BoundarySpec::new());
    let mut sc = Scratch::new();
    let p = Point3::new(-R, 0.0, 0.0);
    let into = w.locate(p, Some(Vector3::new(1.0, 0.0, 0.0)), &mut sc).expect("locate");
    let away = w.locate(p, Some(Vector3::new(-1.0, 0.0, 0.0)), &mut sc).expect("locate");
    assert_eq!(into.at.region, sphere_vol());
    assert_eq!(away.at.region, Region::Exterior);
    // From the surface heading inward the chord is the full diameter; the self-hit at t=0 is
    // rejected by side consistency, not by a distance cutoff.
    let hit = w.distance_to_boundary(&into, 1e9, &mut sc).expect("hit").expect("some");
    assert!((hit.distance - 2.0 * R).abs() <= 1e-14);
    assert_eq!(hit.face, AnalyticWorld::shape_face(0, 0));
    assert_eq!(hit.next.map(|l| l.region), Some(Region::Exterior));
    // Heading away from the surface: the next face is the world wall, not the sphere again.
    let hit = w.distance_to_boundary(&away, 1e9, &mut sc).expect("hit").expect("some");
    assert_eq!(hit.face, AnalyticWorld::world_face(0));
    assert!((hit.distance - 3.0).abs() <= 1e-14);
}

#[test]
fn outside_the_world_is_a_lost_value_not_a_panic() {
    let w = sphere_world(BoundarySpec::new());
    let mut sc = Scratch::new();
    match w.locate(Point3::new(9.0, 0.0, 0.0), None, &mut sc) {
        Err(RtError::Lost(r)) => assert_eq!(r.reason, LostReason::NoRegionAtPoint),
        other => panic!("{other:?}"),
    }
    let (_, end) = trace(&w, [f64::NAN, 0.0, 0.0], [1.0, 0.0, 0.0], 1.0);
    assert!(matches!(end, TraceEnd::Lost(r) if r.reason == LostReason::NonFinite));
}

#[test]
fn mirror_faces_reflect_in_the_structure() {
    let mut b = BoundarySpec::new();
    for k in 0..2 {
        b.set(AnalyticWorld::world_face(k), FaceAction::Reflect);
    }
    let w = AnalyticWorld::new(world_box(), vec![], b, opts());
    // Travels +x with a transverse component, bounces off x=5 and x=-5, stops by distance.
    let (segs, end) = trace(&w, [0.0, 0.0, 0.0], [1.0, 0.1, 0.0], 40.0);
    assert!(matches!(end, TraceEnd::LimitReached));
    assert!(segs.len() >= 4);
    assert!(segs[1].dir.x < 0.0 && segs[0].dir.x > 0.0);
    let total: f64 = segs.iter().map(|s| s.length).sum();
    assert!((total - 40.0).abs() <= 1e-12, "{total}");
}

#[test]
fn translation_deck_wraps_position_and_direction() {
    let deck = Deck::Translation { vector: Vector3::new(-10.0, 0.0, 0.0) };
    let (a, bside) = (AnalyticWorld::world_face(1), AnalyticWorld::world_face(0));
    let mut b = BoundarySpec::new();
    b.set(a, FaceAction::Wrap { transform: wrap_transform(&deck, DeckSide::A).expect("g"), mate: bside });
    b.set(bside, FaceAction::Wrap { transform: wrap_transform(&deck, DeckSide::B).expect("g^-1"), mate: a });
    assert!(wrap_transform(&Deck::Mirror { plane: sm_model_api::Plane::new(Vector3::new(1.0, 0.0, 0.0), 0.0).expect("plane") }, DeckSide::A).is_none());
    let w = sphere_world(b);
    let mut sc = Scratch::new();
    let mut st = w.locate(Point3::new(3.0, 0.0, 0.0), Some(Vector3::new(1.0, 0.0, 0.0)), &mut sc).expect("locate");
    let s = w.step(&mut st, 1e9, &mut sc).expect("step");
    assert!((s.distance - 2.0).abs() <= 1e-14);
    assert!(matches!(s.event, Event::Deck { action: DeckAction::Wrapped { mate }, .. } if mate == bside));
    assert!((st.pos.x + 5.0).abs() <= 1e-12);
    // Re-entered at x=-5 and the next surface is the sphere at distance 3.
    let s = w.step(&mut st, 1e9, &mut sc).expect("step");
    assert!((s.distance - 3.0).abs() <= 1e-12);
    assert!(matches!(s.event, Event::Crossing { to, .. } if to.region == sphere_vol()));
    assert_eq!(st.deck_events, 1);
}

#[test]
fn nested_shape_priority_and_hidden_faces() {
    // Small sphere inside a big one: crossing the big sphere's surface while inside the small
    // one never happens, and the region sequence is complement, big, small, big, complement.
    let shapes = vec![StubShape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: 3.0 }, StubShape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: 1.0 }];
    let w = AnalyticWorld::new(world_box(), shapes, BoundarySpec::new(), opts());
    let (segs, _) = trace(&w, [-4.5, 0.0, 0.0], [1.0, 0.0, 0.0], 1e9);
    let lens: Vec<f64> = segs.iter().map(|s| s.length).collect();
    assert_eq!(segs.len(), 5, "{lens:?}");
    assert!((lens[0] - 1.5).abs() < 1e-13 && (lens[1] - 2.0).abs() < 1e-13 && (lens[2] - 2.0).abs() < 1e-13 && (lens[3] - 2.0).abs() < 1e-13 && (lens[4] - 2.0).abs() < 1e-13, "{lens:?}");
    assert_eq!(segs[2].at.region, Region::Volume(AnalyticWorld::volume(1)));
}

fn hash_batch(b: &TraceBatch) -> u64 {
    let mut h = Fnv64::default();
    for s in &b.segments {
        for v in [s.start.x, s.start.y, s.start.z, s.dir.x, s.dir.y, s.dir.z, s.length] {
            h.word(v.to_bits());
        }
    }
    b.offsets.iter().for_each(|o| h.word(u64::from(*o)));
    h.finish()
}

fn rays(n: usize) -> Vec<Ray> {
    // Explicitly seeded LCG; no time or thread dependence.
    let mut s = 0x9e37_79b9_7f4a_7c15u64;
    let mut next = || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (s >> 11) as f64 / (1u64 << 53) as f64
    };
    (0..n)
        .map(|_| {
            let o = Point3::new(next() * 8.0 - 4.0, next() * 8.0 - 4.0, next() * 8.0 - 4.0);
            let d = Vector3::new(next() - 0.5, next() - 0.5, next() - 0.5);
            Ray { origin: o, dir: d }
        })
        .collect()
}

#[test]
fn trace_many_is_deterministic_across_runs_and_threads() {
    let w = sphere_world(BoundarySpec::new());
    let rs = rays(2000);
    let mut a = TraceBatch::default();
    let mut b = TraceBatch::default();
    w.trace_many(&rs, 1e9, &mut a).expect("a");
    w.trace_many(&rs, 1e9, &mut b).expect("b");
    assert_eq!(hash_batch(&a), hash_batch(&b));
    // Same rays split over 4 threads, concatenated in ray order.
    let parts: Vec<TraceBatch> = std::thread::scope(|sc| {
        let hs: Vec<_> = rs
            .chunks(500)
            .map(|c| {
                let w = &w;
                sc.spawn(move || {
                    let mut t = TraceBatch::default();
                    w.trace_many(c, 1e9, &mut t).expect("part");
                    t
                })
            })
            .collect();
        hs.into_iter().map(|h| h.join().expect("join")).collect()
    });
    let mut merged = TraceBatch { offsets: vec![0], ..Default::default() };
    for p in parts {
        let base = merged.segments.len() as u32;
        merged.offsets.extend(p.offsets.iter().skip(1).map(|o| o + base));
        merged.segments.extend(p.segments);
        merged.ends.extend(p.ends);
    }
    assert_eq!(hash_batch(&a), hash_batch(&merged));
    assert_eq!(a.ends.len(), 2000);
}

#[test]
fn cache_blob_round_trips_and_rejects_other_keys() {
    let spec = BoundarySpec::new();
    let key = RtCacheKey::new(11, 22, &spec, &opts(), 3);
    let blob = RtCacheBlob {
        key,
        arrays: vec![
            RtArray { name: "bvh/lo".into(), dtype: Dtype::F64, count: 3, bytes: [1.0f64, 2.0, 3.0].iter().flat_map(|v| v.to_le_bytes()).collect() },
            RtArray { name: "proto/face_ptr".into(), dtype: Dtype::U32, count: 2, bytes: [0u32, 7].iter().flat_map(|v| v.to_le_bytes()).collect() },
        ],
    };
    let bytes = blob.to_bytes().expect("encode");
    assert_eq!(RtCacheBlob::from_bytes(&bytes, Some(&key)).expect("decode"), blob);
    let other = RtCacheKey::new(12, 22, &spec, &opts(), 3);
    assert!(matches!(RtCacheBlob::from_bytes(&bytes, Some(&other)), Err(RtError::Cache(_))));
    assert!(RtCacheBlob::from_bytes(&bytes[..bytes.len() - 1], None).is_err());
    assert_ne!(key.digest(), other.digest());
    // Boundary changes change the key.
    let mut b2 = BoundarySpec::new();
    b2.set(AnalyticWorld::world_face(0), FaceAction::Reflect);
    assert_ne!(boundary_hash(&spec), boundary_hash(&b2));
}

#[cfg(feature = "capi")]
#[test]
fn c_abi_walks_a_chord() {
    use crate::capi::*;
    use std::sync::Arc;
    let w: Arc<dyn Tracker> = Arc::new(sphere_world(BoundarySpec::new()));
    let t = smrt_tracker_from(w);
    // SAFETY: handles are fresh and freed once; arrays have 3 elements.
    unsafe {
        let c = smrt_ctx_new(t);
        let (p, d) = ([-4.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
        let mut region = 99u64;
        assert_eq!(smrt_locate(c, p.as_ptr(), d.as_ptr(), &mut region), 0);
        assert_eq!(region, 0);
        let mut out = SmrtStep { distance: 0.0, event: -1, face: 0, region: 0, inward: [0.0; 3] };
        assert_eq!(smrt_step(c, 1e9, &mut out), 0);
        assert_eq!(out.event, SmrtEvent::Crossing as i32);
        assert_eq!(smrt_step(c, 1e9, &mut out), 0);
        assert!((out.distance - 4.0).abs() <= 1e-14);
        assert_eq!(smrt_locate(std::ptr::null_mut(), p.as_ptr(), d.as_ptr(), &mut region), SmrtStatus::NullArg as i32);
        smrt_ctx_free(c);
        smrt_tracker_free(t);
    }
}
