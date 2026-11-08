//! W3b-RT-DELTA acceptance 1-4, 8, 10-12 (structure, segments, violation, location, safety,
//! determinism, cache, errors). The statistical runs are in `delta_stats.rs`.
#![allow(clippy::needless_range_loop, clippy::neg_cmp_op_on_partial_ord, clippy::too_many_arguments)]
mod common;
mod dcommon;
use common::*;
use dcommon::*;
use sm_core::Point3;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, MeshRead, SerialAlloc};
use sm_rt::{MeshTracker, MeshTrackerBuilder, load_image, read_smrt};
use sm_rt_api::{
    BuildInput, BuildOptions, CacheImage, ComplementPolicy, FlightInput, FlightOptions, Located, ModePolicy, NodeId, NodeKind, RegionId, RegionRule, RtError, Scratch,
    SectionTag, SegmentScale, StepEnd, TraceEnd, TraceOptions, TrackMode, Tracker, TrackerBuilder, TrackerCaps, validate_partition,
};
use sm_testkit::meshes::{hex_block_two_volumes, mixed_transition, with_sets, SetSpec};

fn cell_bbox(m: &dyn MeshRead, c: usize) -> [f64; 6] {
    let co = m.coords();
    let mut b = [f64::INFINITY, f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    for sf in m.cell_faces(CellIx::new(c)) {
        for v in m.face_vertices(sf.face()) {
            let p = [co.x[v.get()], co.y[v.get()], co.z[v.get()]];
            for k in 0..3 {
                b[k] = b[k].min(p[k]);
                b[3 + k] = b[3 + k].max(p[k]);
            }
        }
    }
    b
}

/// Independent brute-force definition of "the cell may meet the box" (bounding box and, for
/// planar cells, the separating-plane test against the eight corners), padded by `pad`.
fn meets(m: &dyn MeshRead, c: usize, bx: &[f64; 6], pad: f64) -> bool {
    let b = cell_bbox(m, c);
    if (0..3).any(|k| b[k] - pad > bx[3 + k] || b[3 + k] + pad < bx[k]) {
        return false;
    }
    let poly = cell_polytope(m, c);
    for (n, d) in &poly.planes {
        let mut all_out = true;
        for i in 0..8 {
            let x = [if i & 1 == 0 { bx[0] } else { bx[3] }, if i & 2 == 0 { bx[1] } else { bx[4] }, if i & 4 == 0 { bx[2] } else { bx[5] }];
            if dot(*n, x) - d <= pad {
                all_out = false;
            }
        }
        if all_out {
            return false;
        }
    }
    true
}

fn two_region_block(n: [u32; 3], split: u32) -> sm_testkit::meshes::MeshFixture {
    hex_block_two_volumes(n, [1.0; 3], split).unwrap()
}

/// 4 materials by `(i + j + k) mod 4` on an `n^3` block (cells numbered x fastest).
fn checker_block(n: u32) -> sm_testkit::meshes::MeshFixture {
    let base = sm_testkit::meshes::MeshFixture {
        name: "checker",
        mesh: hex_block([n; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap(),
        cell_volumes: vec![1.0; (n * n * n) as usize],
        volume: f64::from(n * n * n),
        area: 6.0 * f64::from(n * n),
        boundary_faces: (6 * n * n) as usize,
    };
    let mut sets: Vec<Vec<CellIx>> = vec![Vec::new(); 4];
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                let c = (i + n * (j + n * k)) as usize;
                sets[((i + j + k) % 4) as usize].push(CellIx::new(c));
            }
        }
    }
    let specs: Vec<SetSpec> = sets.into_iter().enumerate().map(|(m, cs)| SetSpec::new(&format!("m{m}"), sm_mesh_api::SetMembers::Cells(cs))).collect();
    with_sets(base, &specs).unwrap()
}

fn checker_rule() -> RegionRule {
    RegionRule::ByCellSets((0..4).map(|m| format!("m{m}")).collect())
}

fn leaf_boxes(t: &MeshTracker) -> Vec<(NodeId, [f64; 6])> {
    let h = t.majorant_view().unwrap().hierarchy();
    (0..h.node_count())
        .filter_map(|i| {
            let n = h.node(NodeId::new(i)).unwrap();
            n.is_leaf().then_some((n.id, [n.bounds.lo.x, n.bounds.lo.y, n.bounds.lo.z, n.bounds.hi.x, n.bounds.hi.y, n.bounds.hi.z]))
        })
        .collect()
}

#[test]
fn acc02_partition_tiles_and_sets_match_brute_force() {
    let fx = checker_block(6);
    let rule = checker_rule();
    let t = build_part(&fx.mesh, &rule, &vacuum(), popts(4, false));
    assert!(t.caps().has(TrackerCaps::MAJORANT));
    let view = t.majorant_view().unwrap();
    let h = view.hierarchy();
    validate_partition(h).unwrap();
    assert!(h.node_count() > 1);
    let pad = 2.0 * BuildOptions::default().tol.linear;
    let mut multi = 0;
    for (id, bx) in leaf_boxes(&t) {
        let info = h.node(id).unwrap();
        let mut want: Vec<RegionId> = Vec::new();
        let mut ncells = 0u32;
        for c in 0..216 {
            if meets(&fx.mesh, c, &bx, pad) {
                ncells += 1;
                let r = t.cell_region(sm_rt_api::CellId::new(c)).unwrap();
                if !want.contains(&r) {
                    want.push(r);
                }
            }
        }
        want.sort();
        let got = h.set_regions(info.set).unwrap().to_vec();
        // A node whose box lies inside one cell stores that cell only (preassigned).
        if ncells > 1 && info.cells > 1 {
            assert_eq!(got, want, "set of node {}", id.0);
            assert_eq!(info.cells, ncells, "cell count of node {}", id.0);
        } else {
            assert!(got.iter().all(|r| want.contains(r)));
        }
        if got.len() > 1 {
            multi += 1;
        }
        // Soundness by sampling: every located point of the leaf lies in a region of the set.
        let mut r = rng(&format!("sets_{}", id.0));
        let mut sc = Scratch::new();
        for _ in 0..40 {
            let mut p = [0.0; 3];
            for k in 0..3 {
                p[k] = bx[k] + (bx[3 + k] - bx[k]) * unit01(&mut r);
            }
            let loc = t.locate(p3(p), None, None, &mut sc).unwrap();
            if !loc.cell.is_none() {
                assert!(got.contains(&loc.region), "point {p:?} in region {:?} not in the set of node {}", loc.region, id.0);
            }
        }
    }
    assert!(multi > 0, "some leaf must mix regions");
    // Every region has a singleton set; sets are deduplicated (distinct) and ascending.
    for r in 0..t.regions().0.len() {
        let s = h.region_set(RegionId::new(r)).unwrap();
        assert_eq!(h.set_regions(s).unwrap(), &[RegionId::new(r)]);
    }
    let mut seen = std::collections::BTreeSet::new();
    for s in 0..h.set_count() {
        let l = h.set_regions(sm_rt_api::MaterialSetId::new(s)).unwrap().to_vec();
        assert!(l.windows(2).all(|w| w[0] < w[1]));
        assert!(seen.insert(l), "duplicate set");
    }
    // node_of_cell names a leaf that meets the cell.
    for c in 0..216 {
        let n = h.node_of_cell(sm_rt_api::CellId::new(c)).unwrap();
        assert!(h.node(n).unwrap().is_leaf());
    }
}

#[test]
fn split_rule_is_independent_of_sigma_and_stops_on_one_region() {
    // One region: the root is one node, however many cells it holds (the rule never splits it).
    let mesh = hex_block([8; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let t = build_part(&mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(4, false));
    let h = t.majorant_view().unwrap().hierarchy();
    assert_eq!(h.node_count(), 1);
    let n = h.node(NodeId::new(0)).unwrap();
    assert_eq!(n.kind, NodeKind::Mesh);
    assert_eq!(n.cells, 512);
    // Two regions with leaf_cells = 1: split until the regions separate.
    let fx = two_region_block([4, 2, 2], 2);
    let t = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(1, false));
    let h = t.majorant_view().unwrap().hierarchy();
    assert!(h.node_count() > 1);
    validate_partition(h).unwrap();
    // The same options build the same partition bytes (nothing depends on sigma: there is none).
    let t2 = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(1, false));
    assert_eq!(t.partition_hash(), t2.partition_hash());
    assert!(t.partition_hash().is_some());
    // Without the option the tracker is a plain one.
    let plain = build_plain(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum());
    assert!(plain.majorant_view().is_none());
    assert!(!plain.caps().has(TrackerCaps::MAJORANT));
    assert!(plain.partition_hash().is_none());
}

struct SegLog {
    mode: TrackMode,
    cell: u32,
    t0: f64,
    t1: f64,
    end: StepEnd,
}

/// Run a flight to its end collecting the segments (majorants all 1).
fn fly(t: &MeshTracker, p: Point3, d: [f64; 3], mode: ModePolicy, scale: SegmentScale, sc: &mut Scratch) -> (Vec<SegLog>, Option<TraceEnd>, f64) {
    let view = t.majorant_view().unwrap();
    let n = view.hierarchy().node_count();
    let maj = vec![1.0; n];
    let input = FlightInput::new(&maj);
    let mut st = t.init_state(p, v3(d), None, sc).unwrap();
    let mut cur = view.begin(&st, &FlightOptions::default().with_policy(mode).with_scale(scale)).unwrap();
    let mut out = Vec::new();
    let h = view.hierarchy();
    while let Some(seg) = {
        let before = st.pos;
        let s = cur.next_segment(view, input, &mut st, sc).unwrap();
        if let Some(s) = &s {
            let bx = h.node(s.node).unwrap().bounds;
            let inside = |p: Point3| p.x >= bx.lo.x - 1e-9 && p.x <= bx.hi.x + 1e-9 && p.y >= bx.lo.y - 1e-9 && p.y <= bx.hi.y + 1e-9 && p.z >= bx.lo.z - 1e-9 && p.z <= bx.hi.z + 1e-9;
            assert!(inside(before), "segment start outside its node box");
            if s.mode == TrackMode::Delta {
                assert!(inside(st.pos), "delta segment end outside its node box: {:?} {:?}", st.pos, bx);
            }
        }
        s
    } {
        out.push(SegLog { mode: seg.mode, cell: seg.cell.0, t0: seg.t0, t1: seg.t1, end: seg.end });
    }
    let total = cur.distance();
    (out, cur.end().copied(), total)
}

fn check_segments(t: &MeshTracker, bbox: ([f64; 3], [f64; 3]), n: usize, name: &str, hull: Option<&Convex>, min_frac: usize) {
    let mut r = rng(name);
    let mut sc = Scratch::new();
    let mut checked = 0;
    for _ in 0..n {
        let mut o = [0.0; 3];
        for k in 0..3 {
            o[k] = bbox.0[k] + (bbox.1[k] - bbox.0[k]) * unit01(&mut r);
        }
        let (d, _, _) = random_frame(&mut r);
        let st = t.init_state(p3(o), v3(d), None, &mut sc).unwrap();
        if st.cell.is_none() {
            continue;
        }
        // The reference: the walk.
        let mut tr = Vec::new();
        let mut st2 = st.clone();
        let end = t
            .trace(
                &mut st2,
                &TraceOptions::default(),
                &mut |s: &sm_rt_api::Segment| {
                    tr.push((s.cell.0, s.t0, s.length));
                    sm_rt_api::Flow::Continue
                },
                &mut sc,
            )
            .unwrap();
        let walked: f64 = tr.iter().map(|x| x.2).sum();
        if let Some(h) = hull {
            let (_, t_out) = h.clip(o, d).unwrap();
            assert!((walked - t_out).abs() <= 1e-12 * (1.0 + t_out), "walk {walked} vs hull chord {t_out}");
        }
        for mode in [ModePolicy::Surface, ModePolicy::Delta] {
            let (segs, fend, total) = fly(t, p3(o), d, mode, SegmentScale::Cell, &mut sc);
            let sum: f64 = segs.iter().map(|s| s.t1 - s.t0).sum();
            assert!((sum - walked).abs() <= 1e-12 * (1.0 + walked), "{mode:?}: sum {sum} vs chord {walked}");
            assert!((total - sum).abs() <= 1e-12 * (1.0 + sum));
            for w in segs.windows(2) {
                assert_eq!(w[0].t1.to_bits(), w[1].t0.to_bits(), "contiguity");
            }
            assert!(!matches!(fend, Some(TraceEnd::Lost(_))), "lost");
            assert!(!matches!(end, TraceEnd::Lost(_)));
            if mode == ModePolicy::Surface {
                assert_eq!(segs.len(), tr.len());
                for (s, w) in segs.iter().zip(&tr) {
                    assert_eq!(s.mode, TrackMode::Surface);
                    assert_eq!(s.cell, w.0);
                    assert_eq!(s.t0.to_bits(), w.1.to_bits());
                    assert!((s.t1 - s.t0 - w.2).abs() <= 1e-15 * (1.0 + s.t1));
                }
            } else {
                assert!(segs.iter().all(|s| s.mode == TrackMode::Delta || s.cell == u32::MAX - 1));
                // Interior walls are `SlotChange`; every other event is the walk's.
                for s in &segs {
                    assert!(!matches!(s.end, StepEnd::Lost(_)));
                }
            }
            // Region scale: same total.
            let (rsegs, _, rtot) = fly(t, p3(o), d, ModePolicy::Surface, SegmentScale::Region, &mut sc);
            assert!((rtot - walked).abs() <= 1e-12 * (1.0 + walked), "region scale {rtot} vs {walked} ({} segs)", rsegs.len());
        }
        checked += 1;
    }
    assert!(checked > n / min_frac, "{checked} of {n} lines started inside");
}

#[test]
fn acc01_segment_sums_on_hex_block_and_mixed_transition() {
    let n = if cfg!(debug_assertions) { 3_000 } else { 100_000 };
    let fx = two_region_block([10, 10, 10], 5);
    let t = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(4, false));
    let hull = Convex::boxed([0.0; 3], [10.0; 3]);
    check_segments(&t, ([0.0; 3], [10.0; 3]), n, "acc01_hex10", Some(&hull), 2);
    let mt = mixed_transition().unwrap();
    let t = build_part(&mt.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(1, false));
    check_segments(&t, ([0.0, -1.0, 0.0], [2.0, 1.0, 2.0]), n, "acc01_mixed", None, 8);
    // Single region: one node, delta segments run from wall to wall of the whole model.
    let mesh = hex_block([10; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let t = build_part(&mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(4, false));
    check_segments(&t, ([0.0; 3], [10.0; 3]), n / 3, "acc01_hex10_one", Some(&hull), 2);
}

fn located_equal(a: &Located, b: &Located) -> bool {
    a.cell == b.cell && a.region == b.region && a.on_face == b.on_face
}

#[test]
fn acc04_hinted_location_equals_locate() {
    let n = if cfg!(debug_assertions) { 40_000 } else { 1_000_000 };
    let fx = two_region_block([10, 10, 10], 5);
    let t = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(4, false));
    let view = t.majorant_view().unwrap();
    let h = view.hierarchy();
    let maj = vec![1.0; h.node_count()];
    let input = FlightInput::new(&maj);
    let mut r = rng("acc04");
    let mut sc = Scratch::new();
    let mut compared = 0u64;
    let mut probe_descents = 0u64;
    let mut seg_descents = 0u64;
    // Random points: located through the flight machinery by stopping at a random distance.
    for i in 0..n {
        let near = i % 10 == 0;
        let mut o = [0.0; 3];
        for k in 0..3 {
            o[k] = 10.0 * unit01(&mut r);
            if near {
                // within 1e-9 of a face, edge or vertex
                let g = (o[k] * 10.0).round() / 10.0;
                let g = if unit01(&mut r) < 0.7 { g.round() } else { g };
                o[k] = g + 1e-9 * r.unit();
            }
        }
        let (d, _, _) = random_frame(&mut r);
        let loc0 = t.locate(p3(o), Some(v3(d)), None, &mut sc).unwrap();
        if loc0.cell.is_none() {
            continue;
        }
        // The node that owns the point, then locate from it.
        let st = t.init_state(p3(o), v3(d), None, &mut sc).unwrap();
        let mut st2 = st.clone();
        let mut cur = view.begin(&st, &FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
        let bs = sc.counters.bvh_descents;
        let first = cur.next_segment(view, input, &mut st2, &mut sc).unwrap();
        seg_descents += sc.counters.bvh_descents - bs;
        let Some(seg) = first else { continue };
        // The reference `locate` calls below use the BVH (and are counted); only the probes are
        // measured, so each is bracketed.
        let b0 = sc.counters.bvh_descents;
        let loc1 = cur.probe(view, input, &mut sc, seg.t0).unwrap();
        probe_descents += sc.counters.bvh_descents - b0;
        // At t0 the delta segment locates the start point from its node.
        assert!(located_equal(&loc0, &loc1) || seg.resolved, "i={i} p={o:?}: locate {loc0:?} vs hinted {loc1:?}");
        // And a point inside the segment.
        let tm = seg.t0 + 0.37 * (seg.t1 - seg.t0);
        let pm = st.pos + st.dir * tm;
        let want = t.locate(pm, Some(st.dir), None, &mut sc).unwrap();
        let b1 = sc.counters.bvh_descents;
        let got = cur.probe(view, input, &mut sc, tm).unwrap();
        probe_descents += sc.counters.bvh_descents - b1;
        assert!(located_equal(&want, &got), "i={i} mid-segment point {pm:?}: {want:?} vs {got:?}");
        compared += 1;
    }
    assert!(compared > n as u64 / 2);
    assert_eq!(probe_descents, 0, "no BVH descent in a hinted location");
    assert_eq!(seg_descents, 0, "no BVH descent in a delta segment that starts at an owning node");
}

#[test]
fn acc04_preassigned_leaf_does_zero_tests_and_hinted_equals_locate_on_mixed() {
    // A fine single-region block: leaves deep inside a cell are preassigned.
    let fx = two_region_block([3, 3, 3], 1);
    let t = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(1, false));
    let view = t.majorant_view().unwrap();
    let maj = vec![1.0; view.hierarchy().node_count()];
    let input = FlightInput::new(&maj);
    let mut sc = Scratch::new();
    // Centre of a cell: the owning leaf is inside the cell; locating there costs no candidate test.
    let st = t.init_state(p3([1.5, 1.5, 1.5]), v3([1.0, 0.0, 0.0]), None, &mut sc).unwrap();
    let mut s2 = st.clone();
    let mut cur = view.begin(&st, &FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    let seg = cur.next_segment(view, input, &mut s2, &mut sc).unwrap().unwrap();
    sc.clear_counters();
    let l = cur.probe(view, input, &mut sc, seg.t0).unwrap();
    assert_eq!(l.cell, st.cell);
    assert_eq!(sc.counters.locate_candidates, 0);
    // The mixed-shape fixture agrees with locate everywhere we try.
    let mt = mixed_transition().unwrap();
    let t = build_part(&mt.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(1, false));
    let view = t.majorant_view().unwrap();
    let maj = vec![1.0; view.hierarchy().node_count()];
    let input = FlightInput::new(&maj);
    let mut r = rng("acc04_mixed");
    let n = if cfg!(debug_assertions) { 20_000 } else { 300_000 };
    for i in 0..n {
        let o = [-0.2 + 2.6 * unit01(&mut r), -1.1 + 2.5 * unit01(&mut r), -0.2 + 2.6 * unit01(&mut r)];
        let (d, _, _) = random_frame(&mut r);
        let st = t.init_state(p3(o), v3(d), None, &mut sc).unwrap();
        if st.cell.is_none() {
            continue;
        }
        let mut s2 = st.clone();
        let mut cur = view.begin(&st, &FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
        let Some(seg) = cur.next_segment(view, input, &mut s2, &mut sc).unwrap() else { continue };
        let tm = seg.t0 + unit01(&mut r) * (seg.t1 - seg.t0);
        let pm = st.pos + st.dir * tm;
        let want = t.locate(pm, Some(st.dir), None, &mut sc).unwrap();
        let got = cur.probe(view, input, &mut sc, tm).unwrap();
        assert!(located_equal(&want, &got), "i={i}: {want:?} vs {got:?}");
    }
}

#[test]
fn acc03_understated_majorant_is_a_violation() {
    let fx = two_region_block([2, 1, 1], 1);
    let t = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(1, false));
    let view = t.majorant_view().unwrap();
    let sigma = sigma_by_name(&t, |_| 2.0);
    let mut maj = node_majorants(&t, &sigma, 1.0);
    // Understate every node by half.
    for m in &mut maj {
        *m *= 0.5;
    }
    let input = FlightInput::new(&maj);
    let mut sc = Scratch::new();
    let st = t.init_state(p3([0.25, 0.5, 0.5]), v3([1.0, 0.0, 0.0]), None, &mut sc).unwrap();
    let mut s2 = st.clone();
    let mut cur = view.begin(&st, &FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    let seg = cur.next_segment(view, input, &mut s2, &mut sc).unwrap().unwrap();
    let tm = seg.t0 + 0.5 * seg.length();
    let loc = cur.probe(view, input, &mut sc, tm).unwrap();
    let before = s2.clone();
    let e = cur.check_collision(2.0).unwrap_err();
    match &e {
        RtError::MajorantViolated { node, sigma_t, majorant, pos, cell } => {
            assert_eq!(*node, seg.node);
            assert_eq!(*sigma_t, 2.0);
            assert_eq!(*majorant, maj[seg.node.get()]);
            assert_eq!(*cell, loc.cell);
            assert!((pos.x - (st.pos.x + tm)).abs() < 1e-12);
        }
        other => panic!("{other:?}"),
    }
    assert_eq!(e.code(), -12);
    assert_eq!(s2, before, "check_collision does not move the state");
    // Exactly at the majorant is fine.
    cur.check_collision(maj[seg.node.get()]).unwrap();
}

#[test]
fn acc08_safety_is_a_lower_bound_and_exact_along_the_nearest_normal() {
    let n = if cfg!(debug_assertions) { 1_000 } else { 100_000 };
    let fx = two_region_block([5, 5, 5], 2);
    let t = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(4, false));
    let view = t.majorant_view().unwrap();
    let mut r = rng("acc08");
    let mut sc = Scratch::new();
    for _ in 0..n {
        let o = [5.0 * unit01(&mut r), 5.0 * unit01(&mut r), 5.0 * unit01(&mut r)];
        let (d, _, _) = random_frame(&mut r);
        let st = t.init_state(p3(o), v3(d), None, &mut sc).unwrap();
        if st.cell.is_none() {
            continue;
        }
        let s = view.safety_distance(&st, &mut sc).unwrap();
        assert!(s >= 0.0);
        for _ in 0..(if cfg!(debug_assertions) { 10 } else { 1000 }) {
            let (d2, _, _) = random_frame(&mut r);
            let mut st2 = t.init_state(p3(o), v3(d2), None, &mut sc).unwrap();
            let fr = t.step(&mut st2, f64::INFINITY, &mut sc).unwrap();
            assert!(fr.dist >= s - 1e-12, "fired {} < safety {s}", fr.dist);
        }
        // Along the inward normal of the nearest plane the cell exit equals the safety.
        let c = st.cell.get();
        let mut best = (f64::INFINITY, [0.0; 3]);
        for (nn, dd) in cell_polytope(&fx.mesh, c).planes {
            let sl = dd - dot(nn, o);
            if sl < best.0 {
                best = (sl, nn);
            }
        }
        assert!((best.0 - s).abs() <= 1e-12, "safety {s} vs nearest plane {}", best.0);
        let dir = [best.1[0], best.1[1], best.1[2]];
        let mut st3 = t.init_state(p3(o), v3(dir), None, &mut sc).unwrap();
        let fr = t.step(&mut st3, f64::INFINITY, &mut sc).unwrap();
        assert!((fr.dist - s).abs() <= 1e-12, "exit along the normal {} vs safety {s}", fr.dist);
    }
}

#[test]
fn acc08_cursor_cache_and_safety_skips_leave_flights_unchanged() {
    // A flight that uses the safety distance to skip locating a collision still in the same
    // cell gives the same cells, positions and rng draws as one that always locates.
    let fx = two_region_block([4, 4, 4], 2);
    let t = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(2, false));
    let view = t.majorant_view().unwrap();
    let sigma = sigma_by_name(&t, |_| 3.0);
    let maj = node_majorants(&t, &sigma, 1.0);
    let input = FlightInput::new(&maj);
    let mut skips = 0u64;
    let run = |use_safety: bool, skips: &mut u64| -> (Vec<u64>, u64) {
        let mut sc = Scratch::new();
        let mut trail = Vec::new();
        let mut rng = SplitMix64(7);
        for h in 0..300 {
            let o = [4.0 * unit01(&mut rng), 4.0 * unit01(&mut rng), 4.0 * unit01(&mut rng)];
            let d = iso_dir(&mut rng);
            let mut st = t.init_state(p3(o), v3(d), None, &mut sc).unwrap();
            let mut cur = view.begin(&st, &FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
            let mut known: Option<(Point3, sm_rt_api::CellId, f64)> = None;
            'f: while let Some(seg) = cur.next_segment(view, input, &mut st, &mut sc).unwrap() {
                let mut tt = seg.t0;
                loop {
                    let u = unit01(&mut rng);
                    tt += -(1.0 - u).ln() / seg.sigma_m;
                    if !(tt < seg.t1) {
                        break;
                    }
                    let pos = Point3::new(0.0, 0.0, 0.0);
                    let _ = pos;
                    let p = st.pos + st.dir * (tt - seg.t1);
                    let cell = match known {
                        Some((kp, kc, ks)) if use_safety && (p - kp).norm() < ks => {
                            *skips += 1;
                            kc
                        }
                        _ => {
                            let loc = cur.probe(view, input, &mut sc, tt).unwrap();
                            // Safety at the located point (state moved to it temporarily).
                            let mut probe_st = st.clone();
                            probe_st.pos = p;
                            probe_st.cell = loc.cell;
                            let s = cur.safety_distance(view, &probe_st, &mut sc).unwrap();
                            known = Some((p, loc.cell, s));
                            loc.cell
                        }
                    };
                    trail.push(u64::from(cell.0));
                    if unit01(&mut rng) < 0.3 {
                        break 'f;
                    }
                }
            }
            trail.push(h);
        }
        (trail, rng.0)
    };
    let (a, ra) = run(false, &mut 0);
    let (b, rb) = run(true, &mut skips);
    assert_eq!(a, b, "cells of the tentative collisions");
    assert_eq!(ra, rb, "rng draws");
    assert!(skips > 0, "the safety distance must skip some locations");
}

#[test]
fn acc10_determinism_partition_bytes_and_batch() {
    let fx = checker_block(4);
    let rule = checker_rule();
    let hashes = sm_testkit::determinism::check(2, 4, |th| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(th).build().unwrap();
        pool.install(|| {
            let t = build_part(&fx.mesh, &rule, &vacuum(), popts(2, true));
            let img = t.export_cache().unwrap();
            let mut h = sm_rt_api::ContentHasher::new();
            h.write_hash(t.partition_hash().unwrap());
            h.write_bytes(&img.encode());
            h.finish()
        })
    })
    .unwrap();
    assert_ne!(hashes.0, 0);
    // A batch of histories: bit-equal tallies for 1 and N threads.
    let t = build_part(&fx.mesh, &rule, &mirror_all(), popts(2, false));
    let sigma = sigma_by_name(&t, |n| 0.5 + 0.25 * f64::from(n.as_bytes()[1] - b'0'));
    let scat: Vec<f64> = vec![0.6; sigma.len()];
    let maj = node_majorants(&t, &sigma, 1.0);
    for mode in [ModePolicy::Surface, ModePolicy::Delta] {
        let run = Run { t: &t, sigma: &sigma, scatter: Some(&scat), maj: &maj, block: &[], mode };
        let src = |r: &mut SplitMix64| (p3([4.0 * unit01(r), 4.0 * unit01(r), 4.0 * unit01(r)]), iso_dir(r));
        let n = if cfg!(debug_assertions) { 3_000 } else { 100_000 };
        let s1 = sm_testkit::determinism::check(1, 4, |th| {
            let pool = rayon::ThreadPoolBuilder::new().num_threads(th).build().unwrap();
            let s = pool.install(|| run.batch(n, 99, &src));
            (s.fingerprint, s.absorbed, s.tentative, s.rejected, s.track.iter().map(|x| x.to_bits()).collect::<Vec<_>>(), s.coll.iter().map(|x| x.to_bits()).collect::<Vec<_>>())
        })
        .unwrap();
        assert!(s1.1 > 0);
    }
}

#[test]
fn acc10_w3a_trace_bundle_unchanged_for_hierarchy_less_fixtures() {
    // A partition does not change what `trace_bundle` records.
    let fx = two_region_block([6, 6, 6], 3);
    let plain = build_plain(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum());
    let part = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(4, true));
    assert_eq!(plain.structure_hash(), part.structure_hash());
    let mut r = rng("acc10_bundle");
    let rays: Vec<sm_geom_api::Ray> = (0..20_000)
        .map(|_| {
            let (d, _, _) = random_frame(&mut r);
            sm_geom_api::Ray { origin: p3([-1.0 + 8.0 * unit01(&mut r), -1.0 + 8.0 * unit01(&mut r), -1.0 + 8.0 * unit01(&mut r)]), dir: v3(d) }
        })
        .collect();
    let a = plain.trace_bundle(&rays, &TraceOptions::default()).unwrap();
    let b = part.trace_bundle(&rays, &TraceOptions::default()).unwrap();
    assert_eq!(a.content_hash(), b.content_hash());
}

#[test]
fn acc11_cache_round_trip_with_partition() {
    let fx = checker_block(4);
    let rule = checker_rule();
    let bnd = mirror_all();
    let comp = ComplementPolicy::Outside;
    let opts = BuildOptions::default().with_partition(popts(2, true));
    let input = BuildInput::Mesh(sm_rt_api::MeshInput::new(&fx.mesh, &rule, &bnd, &comp));
    let b = MeshTrackerBuilder;
    let (built, _) = b.build_concrete(&input, &opts).unwrap();
    let img = built.export_cache().unwrap();
    assert!(img.sections.iter().any(|s| s.0 == SectionTag::Private(5)));
    let bytes = img.encode();
    let back = read_smrt(&bytes).unwrap();
    let (loaded, rep) = b.load(&input, &opts, &back).unwrap();
    assert!(rep.from_cache);
    assert_eq!(loaded.structure_hash(), built.structure_hash());
    assert_eq!(loaded.info(), built.info());
    assert!(loaded.caps().has(TrackerCaps::MAJORANT.union(TrackerCaps::REGION_JUMP)));
    assert_eq!(loaded.export_cache().unwrap().encode(), bytes, "re-export is canonical");
    let ct = load_image(&back).unwrap();
    assert_eq!(ct.partition_hash(), built.partition_hash());
    // A batch of histories is bit-identical on both.
    let sigma = sigma_by_name(&built, |n| 0.4 + 0.2 * f64::from(n.as_bytes()[1] - b'0'));
    let scat = vec![0.5; sigma.len()];
    let maj = node_majorants(&built, &sigma, 1.0);
    let src = |r: &mut SplitMix64| (p3([4.0 * unit01(r), 4.0 * unit01(r), 4.0 * unit01(r)]), iso_dir(r));
    let n = if cfg!(debug_assertions) { 2_000 } else { 100_000 };
    let run_a = Run { t: &built, sigma: &sigma, scatter: Some(&scat), maj: &maj, block: &[], mode: ModePolicy::Delta };
    let run_b = Run { t: &ct, sigma: &sigma, scatter: Some(&scat), maj: &maj, block: &[], mode: ModePolicy::Delta };
    assert_eq!(run_a.batch(n, 5, &src), run_b.batch(n, 5, &src));
    // An option change is stale.
    let other = BuildOptions::default().with_partition(popts(3, true));
    assert!(matches!(b.load(&input, &other, &back), Err(RtError::CacheStale(_))), "option change");
    // Hierarchy-less images are byte-identical to before (no partition sections).
    let plain = build_plain(&fx.mesh, &rule, &bnd);
    assert!(plain.export_cache().unwrap().sections.iter().all(|s| !matches!(s.0, SectionTag::Private(5..=7))));
}

fn corrupt_section(img: &CacheImage, tag: u16, f: impl Fn(&mut Vec<u8>)) -> CacheImage {
    let mut out = img.clone();
    for s in &mut out.sections {
        if s.0 == SectionTag::Private(tag) {
            f(&mut s.1);
        }
    }
    out
}

#[test]
fn acc11_hostile_partition_sections_never_panic() {
    let fx = checker_block(3);
    let rule = checker_rule();
    let t = build_part(&fx.mesh, &rule, &vacuum(), popts(2, false));
    let img = t.export_cache().unwrap();
    assert!(load_image(&img).is_ok());
    // kd child index not after its parent / out of range, a candidate out of range, bad counts.
    let kd_off = 8; // count, then entries of 28 bytes: split f64, axis, flags, node, a, b
    let bad = [
        corrupt_section(&img, 6, |b| b[kd_off + 20..kd_off + 24].copy_from_slice(&0u32.to_le_bytes())),
        corrupt_section(&img, 6, |b| b[kd_off + 20..kd_off + 24].copy_from_slice(&u32::MAX.to_le_bytes())),
        corrupt_section(&img, 6, |b| b[kd_off + 8..kd_off + 12].copy_from_slice(&9u32.to_le_bytes())),
        corrupt_section(&img, 6, |b| b.truncate(b.len() - 5)),
        corrupt_section(&img, 6, |b| b[0..8].copy_from_slice(&u64::MAX.to_le_bytes())),
        corrupt_section(&img, 7, |b| b[8..12].copy_from_slice(&u32::MAX.to_le_bytes())),
        corrupt_section(&img, 7, |b| b[0..8].copy_from_slice(&u64::MAX.to_le_bytes())),
        corrupt_section(&img, 7, |b| b.clear()),
        corrupt_section(&img, 5, |b| b[0..4].copy_from_slice(&7u32.to_le_bytes())),
        corrupt_section(&img, 5, |b| b[16..24].copy_from_slice(&f64::NAN.to_le_bytes())),
    ];
    for (i, b) in bad.iter().enumerate() {
        let r = std::panic::catch_unwind(|| load_image(b).map(|_| ()));
        assert!(r.is_ok(), "case {i} panicked");
        assert!(r.unwrap().is_err(), "case {i} accepted a hostile image");
    }
    // Every single-byte flip of the partition sections either loads or errors, never panics.
    for tag in [5u16, 6, 7] {
        let len = img.sections.iter().find(|s| s.0 == SectionTag::Private(tag)).unwrap().1.len();
        for at in (0..len).step_by((len / 40).max(1)) {
            let flipped = corrupt_section(&img, tag, |b| b[at] ^= 0x5a);
            let r = std::panic::catch_unwind(|| load_image(&flipped).map(|_| ()));
            assert!(r.is_ok(), "byte {at} of section {tag} panicked");
        }
    }
    // A flipped byte inside a section payload of the encoded file is caught by the section hashes.
    let bytes = img.encode();
    let mut caught = 0;
    for at in (600..bytes.len()).step_by(bytes.len() / 50) {
        let mut b = bytes.clone();
        b[at] ^= 1;
        if matches!(read_smrt(&b), Err(RtError::CacheCorrupt(_))) {
            caught += 1;
        }
    }
    assert!(caught > 20, "{caught} of the flipped bytes were caught");
}

#[test]
fn acc12_errors() {
    let fx = two_region_block([2, 1, 1], 1);
    let t = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(1, false));
    let view = t.majorant_view().unwrap();
    let h = view.hierarchy();
    let n = h.node_count();
    let sigma = sigma_by_name(&t, |_| 1.0);
    let maj = node_majorants(&t, &sigma, 1.0);
    let mut sc = Scratch::new();
    let st = t.init_state(p3([0.25, 0.5, 0.5]), v3([1.0, 0.0, 0.0]), None, &mut sc).unwrap();
    let mut s2 = st.clone();
    let mut cur = view.begin(&st, &FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    // Wrong slice length.
    let short = vec![1.0; n - 1];
    assert!(matches!(cur.next_segment(view, FlightInput::new(&short), &mut s2, &mut sc), Err(RtError::InvalidInput(_))));
    let blk: Vec<NodeId> = (0..=n).map(NodeId::new).collect();
    assert!(matches!(cur.next_segment(view, FlightInput::new(&maj).with_block(&blk), &mut s2, &mut sc), Err(RtError::InvalidInput(_))));
    assert_eq!(s2, st, "errors leave the state unchanged");
    // A non-finite or negative majorant at the node read.
    for bad in [f64::NAN, -1.0, f64::INFINITY] {
        let mut m2 = maj.clone();
        let seg = cur.next_segment(view, FlightInput::new(&maj), &mut s2.clone(), &mut sc).unwrap().unwrap();
        m2[seg.node.get()] = bad;
        let mut c2 = view.begin(&st, &FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
        let mut s3 = st.clone();
        let e = c2.next_segment(view, FlightInput::new(&m2), &mut s3, &mut sc).unwrap_err();
        assert!(matches!(e, RtError::InvalidMajorant { .. }), "{e:?}");
        assert_eq!(s3, st);
        cur = view.begin(&st, &FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
    }
    // validate on unsorted and duplicated block, and on bad entries.
    let mut m3 = maj.clone();
    m3[0] = f64::NAN;
    assert!(FlightInput::new(&m3).validate(h).is_err());
    let unsorted = [NodeId::new(1), NodeId::new(0)];
    assert!(FlightInput::new(&maj).with_block(&unsorted).validate(h).is_err());
    let dup = [NodeId::new(0), NodeId::new(0)];
    assert!(FlightInput::new(&maj).with_block(&dup).validate(h).is_err());
    FlightInput::new(&maj).validate(h).unwrap();
    // Unknown node, non-finite position and direction.
    assert!(matches!(view.locate_in_node(NodeId::new(n), &[], p3([0.5; 3]), None, &mut sc), Err(RtError::UnknownId { .. })));
    assert!(matches!(view.locate_in_node(NodeId::new(0), &[], p3([f64::NAN; 3]), None, &mut sc), Err(RtError::NonFinite(_))));
    // Region jump needs the partition.
    let mut bad = popts(1, true);
    bad.enabled = false;
    let comp = ComplementPolicy::Outside;
    let rule = RegionRule::ByVolumeClassification;
    let bnd = vacuum();
    let input = BuildInput::Mesh(sm_rt_api::MeshInput::new(&fx.mesh, &rule, &bnd, &comp));
    assert!(matches!(MeshTrackerBuilder.build_concrete(&input, &BuildOptions::default().with_partition(bad)), Err(RtError::Build(_))));
}

/// Node visits of the first delta segment of one fixed ray on `checker_block(m)`, and the
/// node count.
fn first_segment_visits(m: u32) -> (usize, u64) {
    let fx = checker_block(m);
    let t = build_part(&fx.mesh, &checker_rule(), &vacuum(), popts(1, false));
    let view = t.majorant_view().unwrap();
    let n = view.hierarchy().node_count();
    let mut sc = Scratch::new();
    let st = t.init_state(p3([0.3, 0.4, 0.5]), v3([0.6, 0.0, 0.8]), None, &mut sc).unwrap();
    let mut visits = [0u64; 2];
    // Two different slices of the same length: identical work.
    for (k, v) in [1.0, 5.0].into_iter().enumerate() {
        let maj = vec![v; n];
        let mut s2 = st.clone();
        let mut cur = view.begin(&st, &FlightOptions::default().with_policy(ModePolicy::Delta)).unwrap();
        sc.clear_counters();
        cur.next_segment(view, FlightInput::new(&maj), &mut s2, &mut sc).unwrap().unwrap();
        visits[k] = sc.counters.node_visits;
    }
    assert_eq!(visits[0], visits[1]);
    (n, visits[0])
}

#[test]
fn node_visits_do_not_scan_the_slice() {
    // The work of one call depends on the depth of the descent, not on the slice length: the
    // partition grows by a factor of 27 in cell count (4^3 to 12^3) between the two fixtures, the visits by far less.
    let (n_small, v_small) = first_segment_visits(4);
    let (n_big, v_big) = first_segment_visits(12);
    assert!(n_big >= 10_000, "{n_big} nodes");
    assert!(v_small > 0 && v_big > 0);
    assert!(v_big < 200, "{v_big} visits for {n_big} nodes");
    // Descent depth grows with log(nodes): far from the growth in nodes.
    assert!(v_big <= 3 * v_small + 40, "visits {v_small} -> {v_big} scale with nodes {n_small} -> {n_big}");
    eprintln!("node visits: {v_small} for {n_small} nodes, {v_big} for {n_big} nodes");
}

/// Records the size of the partition of `checker_block(12)` (1728 cells, 4 regions by
/// `(i + j + k) mod 4`): with `leaf_cells = 1` no plane at a ninth of the box lines up with the
/// cells, so isolating a region runs to the width limit and the tree is huge. Counters, not clocks;
/// the numbers are in DESIGN.md. `rt.partition` reports them before a code commits to a partition.
#[test]
fn partition_size_of_a_checker_block_is_recorded() {
    let fx = checker_block(12);
    let rule = checker_rule();
    let mut rows = Vec::new();
    for leaf in [8, 1] {
        let t = build_part(&fx.mesh, &rule, &vacuum(), popts(leaf, false));
        let h = t.majorant_view().unwrap().hierarchy();
        rows.push((leaf, h.node_count(), t.info().memory_bytes));
    }
    eprintln!("checker_block(12) (leaf_cells, nodes, tracker memory bytes): {rows:?}");
    assert!(rows[1].1 > rows[0].1);
}
