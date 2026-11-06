//! W3b-RT-DELTA-WORLD: flights across lattices and universes. The world's nodes are the
//! prototype tree (a lattice is one node, elements filled with one universe share its subtree),
//! so the majorant slice does not grow with the element count. Cross sections live in the tests,
//! indexed by region id (regions are prototypes); sm-rt never sees one.
#![allow(clippy::needless_range_loop, clippy::neg_cmp_op_on_partial_ord, clippy::too_many_arguments)]
mod common;
mod dcommon;
use common::*;
use dcommon::*;
use sm_core::{Point3, Transform, Vector3};
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{MeshRead, SerialAlloc};
use sm_rt::{MeshTracker, MeshTrackerBuilder, WorldTracker};
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, CellId, ComplementPolicy, ComponentId, ComponentKind, FlightInput, FlightOptions, HierInput, InstancePolicy, Lattice, LatticeKind,
    LeafInput, MeshInput, ModePolicy, NodeId, NodeKind, Placement, RegionRule, RtError, Scratch, TraceEnd, TraceOptions, Tracker, WorldSpec, validate_partition,
};
use sm_testkit::meshes::hex_block_two_volumes;

fn cid(i: u32) -> ComponentId {
    ComponentId(i)
}

fn translate(x: f64, y: f64, z: f64) -> Transform {
    Transform::translation(Vector3::new(x, y, z))
}

fn part_opts(leaf_cells: u32) -> BuildOptions {
    BuildOptions::default().with_partition(popts(leaf_cells, false))
}

/// A single-layer lattice of elements of side `side` filled from `meshes` by `fill` (each
/// element is its own prototype per distinct mesh index), element `(0, 0)` at the origin.
fn lattice(meshes: &[&dyn MeshRead], side: [f64; 3], dims: [u32; 3], fill: &[usize], opts: &BuildOptions) -> Result<WorldTracker, RtError> {
    let m = meshes.len();
    let (rules, bnd, comp) = (RegionRule::ByVolumeClassification, BoundarySpec::default(), ComplementPolicy::Outside);
    let leaves: Vec<LeafInput<'_>> = meshes.iter().map(|&mm| LeafInput::Mesh(MeshInput::new(mm, &rules, &bnd, &comp))).collect();
    let mut comps: Vec<ComponentKind> = (0..m).map(|i| ComponentKind::Leaf { input: i }).collect();
    for i in 0..m {
        comps.push(ComponentKind::Union { placements: vec![Placement::new(cid(i as u32), translate(-0.5 * side[0], -0.5 * side[1], -0.5 * side[2]))] });
    }
    comps.push(ComponentKind::Lattice(Lattice::new(LatticeKind::Rect { pitch: side }, Point3::new(0.0, 0.0, 0.0), dims, fill.iter().map(|&f| cid((m + f) as u32)).collect(), None)));
    let spec = WorldSpec::new(comps, cid((2 * m) as u32), InstancePolicy::Hierarchical);
    MeshTrackerBuilder.build_world(&BuildInput::Hier(HierInput::new(&spec, &leaves, &bnd)), opts).map(|b| b.0)
}

fn region_at(t: &dyn Tracker, p: [f64; 3]) -> usize {
    t.locate(p3(p), None, None, &mut Scratch::new()).unwrap().region.get()
}

/// Sigma by region id: `a` for the region at `pa`, `b` for every other.
fn sigma_two(t: &dyn Tracker, pa: [f64; 3], a: f64, b: f64) -> Vec<f64> {
    let ra = region_at(t, pa);
    (0..t.regions().0.len()).map(|r| if r == ra { a } else { b }).collect()
}

fn within(what: &str, got: f64, want: f64, se: f64, k: f64) {
    assert!((got - want).abs() <= k * se.max(1e-300), "{what}: {got} vs {want} (se {se}, {k} sigma = {})", k * se);
}

/// Odd-numbered leaf nodes: the mixed policy (even leaves delta, odd leaves surface).
fn odd_leaves(t: &dyn Tracker) -> Vec<NodeId> {
    let h = t.majorant_view().unwrap().hierarchy();
    (0..h.node_count()).filter(|&i| h.node(NodeId::new(i)).unwrap().is_leaf() && i % 2 == 1).map(NodeId::new).collect()
}

fn beam(t: &dyn Tracker, sigma: &[f64], mode: ModePolicy, mixed: bool, n: usize, seed: u64) -> Score {
    let maj = node_majorants(t, sigma, 1.0);
    let block = if mixed { odd_leaves(t) } else { Vec::new() };
    let run = Run { t, sigma, scatter: None, maj: &maj, block: &block, mode };
    run.batch(n, seed, &|_r| (p3([0.0, 0.5, 0.5]), [1.0, 0.0, 0.0]))
}

// ---- acceptance 1: RT-DELTA 6 (two-region beam) on a 3x1x1 lattice ------------------------

#[test]
fn acc1_two_region_beam_through_a_lattice_agrees_with_analytic_and_single_mesh() {
    let (s1, s2, n) = (0.1f64, 0.3f64, 20_000usize);
    let want = (-(s1 * 2.0 + s2 * 4.0)).exp();
    let cube = hex_block([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    // Element 0 is prototype A (sigma s1), elements 1 and 2 are prototype B (sigma s2).
    let w = lattice(&[&cube, &cube], [2.0; 3], [3, 1, 1], &[0, 1, 1], &part_opts(8)).unwrap();
    let sw = sigma_two(&w, [0.5, 0.5, 0.5], s1, s2);
    let fx = hex_block_two_volumes([6, 2, 2], [1.0; 3], 2).unwrap();
    let single = build_part(&fx.mesh, &RegionRule::ByVolumeClassification, &vacuum(), popts(8, false));
    let ss = sigma_two(&single, [0.5, 0.5, 0.5], s1, s2);
    let se = binom_se(want, n as f64);
    for (mode, mixed) in [(ModePolicy::Surface, false), (ModePolicy::Delta, false), (ModePolicy::Delta, true)] {
        let a = beam(&w, &sw, mode, mixed, n, 5);
        let b = beam(&single, &ss, mode, mixed, n, 5);
        assert_eq!((a.lost, b.lost), (0, 0));
        let (ta, tb) = (a.leaked as f64 / n as f64, b.leaked as f64 / n as f64);
        within(&format!("lattice {mode:?} mixed={mixed}"), ta, want, se, 4.0);
        within(&format!("single {mode:?} mixed={mixed}"), tb, want, se, 4.0);
        within(&format!("lattice vs single {mode:?} mixed={mixed}"), ta, tb, se * 2f64.sqrt(), 4.0);
        // Acceptance 2: every tentative collision found its element by index arithmetic.
        assert_eq!(a.counters.lattice_index_locates, a.counters.tentative, "{mode:?} mixed={mixed}");
        assert_eq!(a.counters.bvh_descents, 0, "{mode:?} mixed={mixed}");
        if mode == ModePolicy::Surface {
            assert_eq!(a.counters.tentative, 0);
        } else {
            assert!(a.counters.tentative > 0);
        }
    }
}

/// RT-DELTA 5 on the lattice: an isotropic source in a lattice of one prototype, in surface and
/// in delta mode with a loosened majorant, against the same geometry as a single mesh. (A
/// world has one default boundary rule, so the slab's mirror sides are not available; the
/// analytic E3 leakage is checked on the single mesh in `delta_stats.rs`.)
#[test]
fn acc1_isotropic_source_in_a_uniform_lattice_agrees_with_the_single_mesh() {
    let n = 20_000;
    let cube = hex_block([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let w = lattice(&[&cube], [2.0; 3], [3, 1, 1], &[0, 0, 0], &part_opts(8)).unwrap();
    let single = {
        let m = hex_block([6, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
        build_part(&m, &RegionRule::ByVolumeClassification, &vacuum(), popts(8, false))
    };
    let src = |r: &mut SplitMix64| (p3([6.0 * unit01(r), 2.0 * unit01(r), 2.0 * unit01(r)]), iso_dir(r));
    for sg in [0.1, 0.5] {
        let (sw, ss) = (vec![sg; w.regions().0.len()], vec![sg; single.regions().0.len()]);
        let leak = |t: &dyn Tracker, sig: &[f64], mode, scale| {
            let maj = node_majorants(t, sig, scale);
            let run = Run { t, sigma: sig, scatter: None, maj: &maj, block: &[], mode };
            run.batch(n, 21, &src)
        };
        let reference = leak(&single, &ss, ModePolicy::Surface, 1.0);
        let p = reference.leaked as f64 / n as f64;
        let se = (2.0 * p * (1.0 - p) / n as f64).sqrt();
        for (mode, scale) in [(ModePolicy::Surface, 1.0), (ModePolicy::Delta, 1.0), (ModePolicy::Delta, 3.0)] {
            let s = leak(&w, &sw, mode, scale);
            assert_eq!(s.lost, 0);
            within(&format!("sigma {sg} {mode:?} x{scale} leakage vs single"), s.leaked as f64 / n as f64, p, se, 4.0);
            assert_eq!(s.counters.lattice_index_locates, s.counters.tentative);
            assert_eq!(s.counters.bvh_descents, 0);
            if scale > 1.0 {
                let rej = s.rejected as f64 / s.tentative as f64;
                within(&format!("sigma {sg} rejection"), rej, 1.0 - 1.0 / scale, binom_se(1.0 - 1.0 / scale, s.tentative as f64), 4.0);
            }
        }
    }
}

// ---- exact segment sums ---------------------------------------------------------------------

/// One flight from `o` along `d` to the world boundary: contiguous segments, the sum of the
/// lengths and the number of segments.
fn flight_sum(t: &dyn Tracker, maj: &[f64], block: &[NodeId], mode: ModePolicy, o: [f64; 3], d: [f64; 3], sc: &mut Scratch) -> (f64, usize) {
    let view = t.majorant_view().unwrap();
    let bvh0 = sc.counters.bvh_descents;
    let mut st = t.init_state(p3(o), v3(d), None, sc).unwrap();
    sc.counters.bvh_descents = bvh0;
    assert!(!st.cell.is_none(), "start inside");
    let mut cur = view.begin(&st, &FlightOptions::default().with_policy(mode)).unwrap();
    let input = FlightInput::new(maj).with_block(block);
    let (mut prev, mut n) = (0.0f64, 0usize);
    while let Some(seg) = cur.next_segment(view, input, &mut st, sc).unwrap() {
        assert_eq!(seg.t0.to_bits(), prev.to_bits(), "contiguous");
        assert!(seg.t1 >= seg.t0);
        prev = seg.t1;
        n += 1;
        let at = [o[0] + d[0] * prev, o[1] + d[1] * prev, o[2] + d[2] * prev];
        let e = (0..3).map(|k| (at[k] - [st.pos.x, st.pos.y, st.pos.z][k]).abs()).fold(0.0, f64::max);
        assert!(e <= 1e-9, "state position {e} off the flight line");
    }
    assert!(matches!(cur.end(), Some(TraceEnd::Boundary { .. })), "ended {:?}", cur.end());
    (prev, n)
}

fn chords(t: &dyn Tracker, lo: [f64; 3], hi: [f64; 3], name: &str, n: usize) {
    let h = t.majorant_view().unwrap().hierarchy();
    // Zero majorants: delta segments never collide, so a flight is its segment stream.
    let maj = vec![0.0; h.node_count()];
    let odd = odd_leaves(t);
    let boxed = Convex::boxed(lo, hi);
    let mut r = rng(name);
    let mut sc = Scratch::new();
    let (mut worst, mut segs) = (0.0f64, [0usize; 3]);
    for _ in 0..n {
        let o = [lo[0] + (hi[0] - lo[0]) * unit01(&mut r), lo[1] + (hi[1] - lo[1]) * unit01(&mut r), lo[2] + (hi[2] - lo[2]) * unit01(&mut r)];
        let (d, _, _) = random_frame(&mut r);
        let (_, tout) = boxed.clip(o, d).unwrap();
        for (k, (mode, block)) in [(ModePolicy::Surface, &[][..]), (ModePolicy::Delta, &[][..]), (ModePolicy::Delta, &odd[..])].into_iter().enumerate() {
            let (sum, ns) = flight_sum(t, &maj, block, mode, o, d, &mut sc);
            // A global position carries an absolute rounding error of about an ulp, so the exit of a
            // line that grazes a wall is off by that over |d.n| (ruling R1's argument): 1e-12
            // for ordinary lines, scaled by 0.05 / min|d_k| below that.
            let dmin = d.iter().map(|x| x.abs()).fold(f64::INFINITY, f64::min).max(1e-6);
            worst = worst.max((sum - tout).abs() / (0.05 / dmin).max(1.0));
            segs[k] += ns;
        }
    }
    eprintln!("{name}: {n} lines, worst scaled chord error {worst:.2e}, segments surface/delta/mixed {segs:?}");
    assert!(worst <= 1e-12, "{name}: chord error {worst}");
    assert_eq!(sc.counters.bvh_descents, 0, "{name}: no BVH descents");
}

#[test]
fn acc1_segment_sums_equal_the_chord_in_a_mixed_lattice() {
    let n = if cfg!(debug_assertions) { 4_000 } else { 100_000 };
    let pin = hex_block_two_volumes([4, 4, 4], [0.3; 3], 2).unwrap();
    let plain = hex_block([4, 4, 4], [0.3; 3], &mut SerialAlloc::new()).unwrap();
    let side = [1.2; 3];
    // A 5x5 lattice of two kinds of pin in a checker pattern.
    let fill: Vec<usize> = (0..25).map(|e| (e % 5 + e / 5) % 2).collect();
    let t = lattice(&[&pin.mesh, &plain], side, [5, 5, 1], &fill, &part_opts(4)).unwrap();
    chords(&t, [0.0; 3], [6.0, 6.0, 1.2], "acc1_chords_lattice", n);
}

/// Two levels: a 3x3 lattice of pins inside a 2x2 lattice (the world of RT-WORLD acceptance 2).
fn two_level(opts: &BuildOptions) -> WorldTracker {
    let pin = hex_block_two_volumes([3, 3, 3], [0.5; 3], 1).unwrap();
    let l = 1.5;
    let (rules, bnd, comp) = (RegionRule::ByVolumeClassification, BoundarySpec::default(), ComplementPolicy::Outside);
    let leaves = [LeafInput::Mesh(MeshInput::new(&pin.mesh, &rules, &bnd, &comp))];
    let spec = WorldSpec::new(
        vec![
            ComponentKind::Leaf { input: 0 },
            ComponentKind::Union { placements: vec![Placement::new(cid(0), translate(-0.5 * l, -0.5 * l, -0.5 * l))] },
            ComponentKind::Lattice(Lattice::new(LatticeKind::Rect { pitch: [l, l, l] }, Point3::new(-1.5 * l, -1.5 * l, -0.5 * l), [3, 3, 1], vec![cid(1); 9], None)),
            ComponentKind::Lattice(Lattice::new(LatticeKind::Rect { pitch: [3.0 * l, 3.0 * l, l] }, Point3::new(0.0, 0.0, -0.5 * l), [2, 2, 1], vec![cid(2); 4], None)),
        ],
        cid(3),
        InstancePolicy::Hierarchical,
    );
    MeshTrackerBuilder.build_world(&BuildInput::Hier(HierInput::new(&spec, &leaves, &bnd)), opts).unwrap().0
}

#[test]
fn acc1_segment_sums_equal_the_chord_in_a_two_level_lattice() {
    let n = if cfg!(debug_assertions) { 4_000 } else { 100_000 };
    let t = two_level(&part_opts(4));
    chords(&t, [0.0, 0.0, -0.75], [9.0, 9.0, 0.75], "acc1_chords_two_level", n);
}

#[test]
fn acc1_segment_sums_equal_the_chord_with_a_rotated_placement() {
    // Two placements of one pin side by side, the second turned 90 degrees about z, so the
    // prototype's kd walls are crossed in a rotated frame (`past_wall`).
    let n = if cfg!(debug_assertions) { 4_000 } else { 100_000 };
    let pin = hex_block_two_volumes([4, 4, 4], [0.3; 3], 2).unwrap();
    let (rules, bnd, comp) = (RegionRule::ByVolumeClassification, BoundarySpec::default(), ComplementPolicy::Outside);
    let leaves = [LeafInput::Mesh(MeshInput::new(&pin.mesh, &rules, &bnd, &comp))];
    let rot = Transform { m: [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(2.5, 0.0, 0.0) };
    let spec = WorldSpec::new(
        vec![ComponentKind::Leaf { input: 0 }, ComponentKind::Union { placements: vec![Placement::new(cid(0), translate(0.0, 0.0, 0.0)), Placement::new(cid(0), rot)] }],
        cid(1),
        InstancePolicy::Hierarchical,
    );
    let t = MeshTrackerBuilder.build_world(&BuildInput::Hier(HierInput::new(&spec, &leaves, &bnd)), &part_opts(4)).unwrap().0;
    // A gap of 0.1 keeps the placements disjoint: a flight from either box ends at its exit.
    chords(&t, [0.0; 3], [1.2; 3], "acc1_chords_rotated_a", n);
    chords(&t, [1.3, 0.0, 0.0], [2.5, 1.2, 1.2], "acc1_chords_rotated_b", n);
}

// ---- acceptance 3: a 17x17 assembly ---------------------------------------------------------

#[test]
fn acc3_assembly_17x17_has_no_bvh_descents_and_records_segment_counts() {
    let pin = hex_block_two_volumes([4, 4, 4], [0.3; 3], 2).unwrap();
    let t = lattice(&[&pin.mesh], [1.2; 3], [17, 17, 1], &vec![0; 289], &part_opts(8)).unwrap();
    assert_eq!(t.cell_count(), 289 * 64);
    let n = if cfg!(debug_assertions) { 600 } else { 20_000 };
    // Fuel (the region at the pin's low-x end) 0.8, moderator 0.2 per cm.
    let sigma = sigma_two(&t, [0.15, 0.6, 0.6], 0.8, 0.2);
    let maj = node_majorants(&t, &sigma, 1.0);
    let src = |r: &mut SplitMix64| (p3([20.4 * unit01(r), 20.4 * unit01(r), 1.2 * unit01(r)]), iso_dir(r));
    let mut rows = Vec::new();
    for mode in [ModePolicy::Surface, ModePolicy::Delta] {
        let run = Run { t: &t, sigma: &sigma, scatter: None, maj: &maj, block: &[], mode };
        let s = run.batch(n, 17, &src);
        assert_eq!(s.lost, 0, "{mode:?}");
        assert_eq!(s.counters.bvh_descents, 0, "{mode:?}: no BVH descent in a flight");
        assert_eq!(s.counters.lattice_index_locates, s.counters.tentative, "{mode:?}");
        rows.push((mode, s.counters.segments, s.counters.tentative, s.counters.node_visits));
    }
    eprintln!("17x17, {n} histories (mode, segments, tentative, node_visits): {rows:?}");
    assert_eq!(rows[0].2, 0);
    assert!(rows[1].1 > 0 && rows[1].2 > 0);
}

// ---- hierarchy ------------------------------------------------------------------------------

#[test]
fn the_node_table_does_not_grow_with_the_element_count() {
    let pin = hex_block_two_volumes([4, 4, 4], [0.3; 3], 2).unwrap();
    let small = lattice(&[&pin.mesh], [1.2; 3], [3, 1, 1], &[0; 3], &part_opts(4)).unwrap();
    let big = lattice(&[&pin.mesh], [1.2; 3], [30, 3, 1], &vec![0; 90], &part_opts(4)).unwrap();
    let (hs, hb) = (small.majorant_view().unwrap().hierarchy(), big.majorant_view().unwrap().hierarchy());
    assert_eq!(hs.node_count(), hb.node_count());
    assert_eq!(hs.set_count(), hb.set_count());
    for t in [&small, &big, &two_level(&part_opts(4))] {
        let h = t.majorant_view().unwrap().hierarchy();
        validate_partition(h).unwrap();
        let root = h.node(h.root()).unwrap();
        assert_eq!(root.kind, NodeKind::Lattice);
        assert_eq!(root.cells as usize, t.cell_count());
        // Every cell has an owning node that is a leaf; singleton sets exist for every region.
        for c in [0, t.cell_count() / 2, t.cell_count() - 1] {
            let n = h.node_of_cell(CellId::new(c)).unwrap();
            assert!(h.node(n).unwrap().is_leaf());
        }
        for r in 0..t.regions().0.len() {
            let s = h.region_set(sm_rt_api::RegionId::new(r)).unwrap();
            assert_eq!(h.set_regions(s).unwrap(), [sm_rt_api::RegionId::new(r)]);
        }
    }
    // The majorant slice has the prototype tree's length: a wrong length is an error.
    let view = big.majorant_view().unwrap();
    let mut sc = Scratch::new();
    let mut st = big.init_state(p3([0.5, 0.5, 0.5]), v3([1.0, 0.0, 0.0]), None, &mut sc).unwrap();
    let mut cur = view.begin(&st, &FlightOptions::default()).unwrap();
    let short = vec![1.0; hb.node_count() - 1];
    assert!(matches!(cur.next_segment(view, FlightInput::new(&short), &mut st, &mut sc), Err(RtError::InvalidInput(_))));
}

#[test]
fn partition_options_on_a_world_work_or_are_refused_never_dropped() {
    let cube = hex_block([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    // Enabled: the world flies.
    let t = lattice(&[&cube], [2.0; 3], [2, 1, 1], &[0, 0], &part_opts(8)).unwrap();
    assert!(t.majorant_view().is_some() && t.caps().has(sm_rt_api::TrackerCaps::MAJORANT));
    assert!(t.partition_hash().is_some());
    // Disabled: no majorant view, and the world is the one W3b-1 built.
    let plain = lattice(&[&cube], [2.0; 3], [2, 1, 1], &[0, 0], &BuildOptions::default()).unwrap();
    assert!(plain.majorant_view().is_none() && plain.partition_hash().is_none());
    assert_eq!(plain.structure_hash(), t.structure_hash());
    // leaf_cells and max_depth reach the prototypes.
    let fine = lattice(&[&cube], [2.0; 3], [2, 1, 1], &[0, 0], &BuildOptions::default().with_partition({
        let mut p = popts(1, false);
        p.max_depth = 3;
        p
    }))
    .unwrap();
    assert_ne!(fine.partition_hash(), t.partition_hash());
    // Region jump on a world is refused.
    let e = lattice(&[&cube], [2.0; 3], [2, 1, 1], &[0, 0], &BuildOptions::default().with_partition(popts(8, true))).err().unwrap();
    assert!(matches!(e, RtError::Unsupported(_)), "{e:?}");
    // A facet leaf has no partition: refused, not dropped.
    let facets = nested_facets();
    let (bnd, spec) = (BoundarySpec::default(), WorldSpec::new(vec![ComponentKind::Leaf { input: 0 }], cid(0), InstancePolicy::Hierarchical));
    let leaves = [LeafInput::Facets(&facets)];
    let r = MeshTrackerBuilder.build_world(&BuildInput::Hier(HierInput::new(&spec, &leaves, &bnd)), &part_opts(8));
    assert!(matches!(r, Err(RtError::Unsupported(_))), "{:?}", r.err());
    assert!(MeshTrackerBuilder.build_world(&BuildInput::Hier(HierInput::new(&spec, &leaves, &bnd)), &BuildOptions::default()).is_ok());
}

#[test]
fn a_partition_does_not_change_what_a_world_trace_bundle_records() {
    let pin = hex_block_two_volumes([4, 4, 4], [0.3; 3], 2).unwrap();
    let plain_pin = hex_block([4, 4, 4], [0.3; 3], &mut SerialAlloc::new()).unwrap();
    let fill: Vec<usize> = (0..9).map(|e| (e % 3 + e / 3) % 2).collect();
    let meshes: [&dyn MeshRead; 2] = [&pin.mesh, &plain_pin];
    let a = lattice(&meshes, [1.2; 3], [3, 3, 1], &fill, &BuildOptions::default()).unwrap();
    let b = lattice(&meshes, [1.2; 3], [3, 3, 1], &fill, &part_opts(4)).unwrap();
    let mut r = rng("world_bundle");
    let rays: Vec<sm_geom_api::Ray> = (0..20_000)
        .map(|_| {
            let (d, _, _) = random_frame(&mut r);
            sm_geom_api::Ray { origin: p3([-0.5 + 4.6 * unit01(&mut r), -0.5 + 4.6 * unit01(&mut r), -0.5 + 2.2 * unit01(&mut r)]), dir: v3(d) }
        })
        .collect();
    let (ha, hb) = (a.trace_bundle(&rays, &TraceOptions::default()).unwrap(), b.trace_bundle(&rays, &TraceOptions::default()).unwrap());
    assert_eq!(ha.content_hash(), hb.content_hash());
}

// ---- determinism ----------------------------------------------------------------------------

#[test]
fn determinism_world_partition_and_a_batch_are_bit_equal_for_one_and_n_threads() {
    let pin = hex_block_two_volumes([4, 4, 4], [0.3; 3], 2).unwrap();
    let n = if cfg!(debug_assertions) { 2_000 } else { 50_000 };
    let hashes = sm_testkit::determinism::check(1, 4, |th| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(th).build().unwrap();
        pool.install(|| {
            let t = lattice(&[&pin.mesh], [1.2; 3], [4, 4, 1], &[0; 16], &part_opts(4)).unwrap();
            let sigma = sigma_two(&t, [0.15, 0.6, 0.6], 1.0, 0.3);
            let maj = node_majorants(&t, &sigma, 1.5);
            let scat = vec![0.5; sigma.len()];
            let run = Run { t: &t, sigma: &sigma, scatter: Some(&scat), maj: &maj, block: &[], mode: ModePolicy::Delta };
            let s = run.batch(n, 7, &|r| (p3([4.8 * unit01(r), 4.8 * unit01(r), 1.2 * unit01(r)]), iso_dir(r)));
            (t.partition_hash().unwrap().0, s.fingerprint, s.absorbed, s.tentative, s.track.iter().map(|x| x.to_bits()).collect::<Vec<_>>(), s.coll.iter().map(|x| x.to_bits()).collect::<Vec<_>>())
        })
    })
    .unwrap();
    assert!(hashes.2 > 0);
}

#[allow(dead_code)]
fn _types(_: &MeshTracker) {}
