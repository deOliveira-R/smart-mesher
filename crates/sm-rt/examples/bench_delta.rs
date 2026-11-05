//! Work counters of surface and delta flights and of the region jump (record only: counters,
//! never clocks, and no claim about speed).
//!
//! `cargo run --release -p sm-rt --example bench_delta`
//!
//! Cases: C-B a large one-region block (`hex_block(40)`), `fire` by the cell walk and by the
//! region jump; C-C a two-region slab with a 20:1 contrast, flights in surface mode, delta mode
//! and a mixed `block` policy. Cross sections are the example's own (keyed by region id);
//! sm-rt never sees one. The 17x17 assembly case needs flights across lattices (a later task).

use sm_core::{Point3, Vector3};
use sm_mesh_api::SerialAlloc;
use sm_mesh_api::reference::hex_block;
use sm_rt::MeshTrackerBuilder;
use sm_rt_api::{
    BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, FlightInput, FlightOptions, MeshInput, ModePolicy, NodeId, PartitionOptions, RegionRule, Scratch,
    TrackMode, Tracker, WorkCounters,
};

struct Rng(u64);
impl Rng {
    fn u01(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    }
    fn dir(&mut self) -> Vector3 {
        let z = 2.0 * self.u01() - 1.0;
        let phi = std::f64::consts::TAU * self.u01();
        let s = (1.0 - z * z).max(0.0).sqrt();
        Vector3::new(s * phi.cos(), s * phi.sin(), z)
    }
}

fn build(mesh: &dyn sm_mesh_api::MeshRead, p: PartitionOptions) -> sm_rt::MeshTracker {
    let (rule, comp, bnd) = (RegionRule::ByVolumeClassification, ComplementPolicy::Outside, BoundarySpec::default());
    let input = BuildInput::Mesh(MeshInput::new(mesh, &rule, &bnd, &comp));
    MeshTrackerBuilder.build_concrete(&input, &BuildOptions::default().with_partition(p)).expect("build").0
}

fn row(name: &str, tracks: usize, c: &WorkCounters) {
    let per = |x: u64| x as f64 / tracks as f64;
    println!(
        "{name:<26} segments {:>8.2} steps {:>8.2} node_visits {:>8.2} face_tests {:>9.2} locate_cand {:>7.2} tentative {:>8.2} rejected {:>8.2} safety_skips {:>6.2} jumps {:>5.2}",
        per(c.segments),
        per(c.steps),
        per(c.node_visits),
        per(c.face_tests),
        per(c.locate_candidates),
        per(c.tentative),
        per(c.rejected),
        per(c.safety_skips),
        per(c.jumps)
    );
}

fn main() {
    let n = 5_000;
    // C-B: fire across one region.
    let mesh = hex_block([40; 3], [1.0; 3], &mut SerialAlloc::new()).expect("mesh");
    let mut p = PartitionOptions::default();
    p.enabled = true;
    let walk = build(&mesh, p);
    p.region_jump = true;
    let jump = build(&mesh, p);
    println!("C-B hex_block(40), {n} fires per row");
    for (name, t) in [("walk", &walk), ("region_jump", &jump)] {
        let mut sc = Scratch::new();
        let mut r = Rng(1);
        for _ in 0..n {
            let o = Point3::new(40.0 * r.u01(), 40.0 * r.u01(), 40.0 * r.u01());
            let mut st = t.init_state(o, r.dir(), None, &mut sc).expect("state");
            t.fire(&mut st, f64::INFINITY, 1_000_000, &mut sc).expect("fire");
        }
        row(name, n, &sc.counters);
    }
    // C-C: two-region slab, 20:1 contrast, pure absorber histories.
    let fx = sm_testkit::meshes::hex_block_two_volumes([8, 1, 1], [1.0; 3], 4).expect("mesh").mesh;
    let mut p = PartitionOptions::default();
    p.enabled = true;
    p.leaf_cells = 1;
    let t = build(&fx, p);
    let view = t.majorant_view().expect("view");
    let h = view.hierarchy();
    let low = {
        let mut sc = Scratch::new();
        t.locate(Point3::new(0.5, 0.5, 0.5), None, None, &mut sc).expect("locate").region
    };
    let sigma = |r: usize| if r == low.get() { 0.05 } else { 1.0 };
    let maj: Vec<f64> = (0..h.node_count()).map(|i| h.set_regions(h.node(NodeId::new(i)).expect("node").set).expect("set").iter().map(|r| sigma(r.get())).fold(0.0, f64::max)).collect();
    let odd: Vec<NodeId> = (0..h.node_count()).filter(|&i| h.node(NodeId::new(i)).is_some_and(|x| x.is_leaf()) && i % 2 == 1).map(NodeId::new).collect();
    println!("C-C two-region slab (sigma 0.05 | 1.0), {n} histories per row");
    for (name, mode, block) in [("surface", ModePolicy::Surface, &[][..]), ("delta", ModePolicy::Delta, &[][..]), ("mixed (odd leaves surface)", ModePolicy::Delta, &odd[..])] {
        let input = FlightInput::new(&maj).with_block(block);
        let mut sc = Scratch::new();
        let mut r = Rng(2);
        for _ in 0..n {
            let o = Point3::new(8.0 * r.u01(), 0.5 + 0.0 * r.u01(), 0.5);
            let mut st = t.init_state(o, r.dir(), None, &mut sc).expect("state");
            if st.cell.is_none() {
                continue;
            }
            let mut cur = view.begin(&st, &FlightOptions::default().with_policy(mode)).expect("begin");
            'f: while let Some(seg) = cur.next_segment(view, input, &mut st, &mut sc).expect("segment") {
                match seg.mode {
                    TrackMode::Surface => {
                        let s = -(1.0 - r.u01()).ln() / sigma(seg.region.get());
                        if s < seg.length() {
                            cur.stop_at(view, input, &mut st, &mut sc, seg.t0 + s).expect("stop");
                            break 'f;
                        }
                    }
                    _ => {
                        let mut tt = seg.t0;
                        loop {
                            tt += -(1.0 - r.u01()).ln() / seg.sigma_m;
                            if tt >= seg.t1 || tt.is_nan() {
                                break;
                            }
                            let loc = cur.probe(view, input, &mut sc, tt).expect("probe");
                            if r.u01() * seg.sigma_m < sigma(loc.region.get()) {
                                cur.stop_at(view, input, &mut st, &mut sc, tt).expect("stop");
                                break 'f;
                            }
                            sc.counters.rejected += 1;
                        }
                    }
                }
            }
        }
        row(name, n, &sc.counters);
    }
}
