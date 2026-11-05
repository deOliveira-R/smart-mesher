//! Helpers of the delta-flight tests: partitioned trackers, a test-local cross-section model
//! keyed by region name (sigma never enters sm-rt), and a small Monte Carlo history engine
//! that drives `FlightCursor` exactly as the module docs of `sm_rt_api::segments` show.
#![allow(dead_code, clippy::neg_cmp_op_on_partial_ord)]

use crate::common::*;
use rayon::prelude::*;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh_api::MeshRead;
use sm_rt::{MeshTracker, MeshTrackerBuilder};
use sm_rt_api::{
    BoundaryDef, BoundarySelector, BoundarySpec, BuildInput, BuildOptions, ComplementPolicy, FlightInput, FlightOptions, ModePolicy, NodeId, PartitionOptions,
    RegionRule, Scratch, TraceEnd, TrackMode, Tracker,
};

pub fn popts(leaf_cells: u32, jump: bool) -> PartitionOptions {
    let mut p = PartitionOptions::default();
    p.enabled = true;
    p.leaf_cells = leaf_cells;
    p.region_jump = jump;
    p
}

pub fn build_part(mesh: &dyn MeshRead, rule: &RegionRule, bnd: &BoundarySpec, p: PartitionOptions) -> MeshTracker {
    let opts = BuildOptions::default().with_partition(p);
    let comp = ComplementPolicy::Outside;
    let input = BuildInput::Mesh(sm_rt_api::MeshInput::new(mesh, rule, bnd, &comp));
    MeshTrackerBuilder.build_concrete(&input, &opts).expect("build").0
}

pub fn build_plain(mesh: &dyn MeshRead, rule: &RegionRule, bnd: &BoundarySpec) -> MeshTracker {
    let opts = BuildOptions::default();
    let comp = ComplementPolicy::Outside;
    let input = BuildInput::Mesh(sm_rt_api::MeshInput::new(mesh, rule, bnd, &comp));
    MeshTrackerBuilder.build_concrete(&input, &opts).expect("build").0
}

pub fn face(n: u64) -> EntityId {
    EntityId::new(EntityKind::TopoFace, n).unwrap()
}

/// `hex_block` boundary: faces 3..=6 (lateral) are mirrors, the x ends are vacuum.
pub fn mirror_lateral() -> BoundarySpec {
    let mut spec = BoundarySpec::default();
    for k in 3..=6u64 {
        spec.rules.push((BoundarySelector::Entity(face(k)), BoundaryDef::Reflect));
    }
    spec
}

/// All six faces mirrors.
pub fn mirror_all() -> BoundarySpec {
    let mut spec = BoundarySpec::default();
    for k in 1..=6u64 {
        spec.rules.push((BoundarySelector::Entity(face(k)), BoundaryDef::Reflect));
    }
    spec
}

/// Majorant of every node: the largest sigma of the regions of its set, times `scale`.
pub fn node_majorants(t: &dyn Tracker, sigma_region: &[f64], scale: f64) -> Vec<f64> {
    let h = t.majorant_view().expect("partition").hierarchy();
    (0..h.node_count())
        .map(|n| {
            let info = h.node(NodeId::new(n)).unwrap();
            let regs = h.set_regions(info.set).unwrap();
            scale * regs.iter().map(|r| sigma_region[r.get()]).fold(0.0, f64::max)
        })
        .collect()
}

/// Sigma by region from a closure of the region name.
pub fn sigma_by_name(t: &dyn Tracker, f: impl Fn(&str) -> f64) -> Vec<f64> {
    t.regions().0.iter().map(|r| f(&r.name)).collect()
}

pub fn unit01(r: &mut SplitMix64) -> f64 {
    (r.unit() + 1.0) * 0.5
}

/// Exponential flight length with rate `rate > 0`.
fn expo(r: &mut SplitMix64, rate: f64) -> f64 {
    -(1.0 - unit01(r)).ln() / rate
}

pub fn iso_dir(r: &mut SplitMix64) -> [f64; 3] {
    let z = r.unit();
    let phi = std::f64::consts::PI * r.unit();
    let s = (1.0 - z * z).max(0.0).sqrt();
    [s * phi.cos(), s * std::hint::black_box(phi).sin(), z]
}

/// Tallies of a batch of histories.
#[derive(Clone, Debug, PartialEq)]
pub struct Score {
    pub histories: u64,
    pub leaked: u64,
    pub absorbed: u64,
    pub lost: u64,
    pub tentative: u64,
    pub rejected: u64,
    /// Track-length flux per cell (surface segments), sum of weights times length.
    pub track: Vec<f64>,
    /// Collision flux per cell (`w / sigma_t` at each real delta collision).
    pub coll: Vec<f64>,
    /// Sum over histories of the squared per-history total path (surface) for the variance.
    pub sq_track_total: f64,
    pub sum_track_total: f64,
    pub counters: sm_rt_api::WorkCounters,
    /// FNV of every final position bit pattern and rng state (determinism fingerprint).
    pub fingerprint: u64,
}

impl Score {
    pub fn new(ncells: usize) -> Self {
        Self {
            histories: 0,
            leaked: 0,
            absorbed: 0,
            lost: 0,
            tentative: 0,
            rejected: 0,
            track: vec![0.0; ncells],
            coll: vec![0.0; ncells],
            sq_track_total: 0.0,
            sum_track_total: 0.0,
            counters: Default::default(),
            fingerprint: 0xcbf2_9ce4_8422_2325,
        }
    }
    pub fn merge(&mut self, o: &Score) {
        self.histories += o.histories;
        self.leaked += o.leaked;
        self.absorbed += o.absorbed;
        self.lost += o.lost;
        self.tentative += o.tentative;
        self.rejected += o.rejected;
        for (a, b) in self.track.iter_mut().zip(&o.track) {
            *a += b;
        }
        for (a, b) in self.coll.iter_mut().zip(&o.coll) {
            *a += b;
        }
        self.sq_track_total += o.sq_track_total;
        self.sum_track_total += o.sum_track_total;
        self.counters.add(&o.counters);
        self.fingerprint = (self.fingerprint ^ o.fingerprint).wrapping_mul(0x0000_0100_0000_01b3);
    }
}

/// The physics of a run: sigma per region, scattering probability per region (`None`: pure
/// absorber), majorants per node, blocked nodes and the mode policy.
pub struct Run<'a> {
    pub t: &'a dyn Tracker,
    pub sigma: &'a [f64],
    pub scatter: Option<&'a [f64]>,
    pub maj: &'a [f64],
    pub block: &'a [NodeId],
    pub mode: ModePolicy,
}

fn mix(h: u64, x: u64) -> u64 {
    (h ^ x).wrapping_mul(0x0000_0100_0000_01b3)
}

impl Run<'_> {
    /// One history from `(p, d)`. Weight is 1 throughout (analog absorption and scattering).
    pub fn history(&self, p: Point3, d: [f64; 3], rng: &mut SplitMix64, sc: &mut Scratch, score: &mut Score) {
        let view = self.t.majorant_view().expect("MAJORANT");
        let input = FlightInput::new(self.maj).with_block(self.block);
        // A flight start with no hint is located through the BVH (counted); the flight itself is not.
        let bvh0 = sc.counters.bvh_descents;
        let mut st = self.t.init_state(p, v3(d), None, sc).expect("start");
        sc.counters.bvh_descents = bvh0;
        score.histories += 1;
        let mut path = 0.0;
        loop {
            let opts = FlightOptions::default().with_policy(self.mode);
            let mut cur = view.begin(&st, &opts).expect("begin");
            let mut collided = false;
            while let Some(seg) = cur.next_segment(view, input, &mut st, sc).expect("segment") {
                match seg.mode {
                    TrackMode::Surface => {
                        let sig = self.sigma[seg.region.get()];
                        let len = seg.length();
                        if sig > 0.0 {
                            let s = expo(rng, sig);
                            if s < len {
                                score.track[seg.cell.get()] += s;
                                path += s;
                                cur.stop_at(view, input, &mut st, sc, seg.t0 + s).expect("stop");
                                collided = true;
                                break;
                            }
                        }
                        score.track[seg.cell.get()] += len;
                        path += len;
                    }
                    _ => {
                        if seg.sigma_m <= 0.0 {
                            continue;
                        }
                        let mut t = seg.t0;
                        loop {
                            t += expo(rng, seg.sigma_m);
                            if !(t < seg.t1) {
                                break;
                            }
                            let loc = cur.probe(view, input, sc, t).expect("probe");
                            let sg = self.sigma[loc.region.get()];
                            cur.check_collision(sg).expect("majorant");
                            score.tentative += 1;
                            if unit01(rng) * seg.sigma_m < sg {
                                score.coll[loc.cell.get()] += 1.0 / sg;
                                cur.stop_at(view, input, &mut st, sc, t).expect("stop");
                                collided = true;
                                break;
                            }
                            score.rejected += 1;
                        }
                        if collided {
                            break;
                        }
                    }
                }
            }
            if collided {
                let r = st.region.get();
                match self.scatter {
                    Some(c) if unit01(rng) < c[r] => {
                        st.dir = v3(iso_dir(rng));
                        st.entered_via = None;
                        continue;
                    }
                    _ => {
                        score.absorbed += 1;
                        break;
                    }
                }
            }
            match cur.end() {
                Some(TraceEnd::Boundary { .. }) | Some(TraceEnd::LeftModel) => score.leaked += 1,
                _ => score.lost += 1,
            }
            break;
        }
        score.sum_track_total += path;
        score.sq_track_total += path * path;
        score.fingerprint = mix(mix(mix(mix(score.fingerprint, st.pos.x.to_bits()), st.pos.y.to_bits()), st.pos.z.to_bits()), rng.0);
    }

    /// `n` histories in blocks of 256, each with the rng `SplitMix64(seed ^ history_id * k)` and
    /// a start drawn from `src(&mut rng)`; blocks are merged in index order, so the result is
    /// bit-identical for any thread count.
    pub fn batch(&self, n: usize, seed: u64, src: &(dyn Fn(&mut SplitMix64) -> (Point3, [f64; 3]) + Sync)) -> Score {
        let ncells = self.t.cell_count();
        let blocks: Vec<usize> = (0..n.div_ceil(256)).collect();
        let parts: Vec<Score> = blocks
            .par_iter()
            .map(|&b| {
                let mut sc = Scratch::new();
                let mut s = Score::new(ncells);
                for h in b * 256..((b + 1) * 256).min(n) {
                    let mut rng = SplitMix64(mix64(seed ^ (h as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15)));
                    let (p, d) = src(&mut rng);
                    self.history(p, d, &mut rng, &mut sc, &mut s);
                }
                s.counters.add(&sc.counters);
                s
            })
            .collect();
        let mut tot = Score::new(ncells);
        for p in &parts {
            tot.merge(p);
        }
        tot
    }
}

pub fn mix64(x: u64) -> u64 {
    sm_testkit::surf3::mix64(x)
}

/// `E3(tau) = int_0^1 mu exp(-tau/mu) dmu` by composite Simpson (smooth, vanishing at 0).
pub fn e3(tau: f64) -> f64 {
    let n = 200_000usize;
    let h = 1.0 / n as f64;
    let f = |m: f64| if m <= 0.0 { 0.0 } else { m * (-tau / m).exp() };
    let mut s = f(0.0) + f(1.0);
    for i in 1..n {
        s += f(i as f64 * h) * if i % 2 == 1 { 4.0 } else { 2.0 };
    }
    s * h / 3.0
}

/// Leakage of a uniform isotropic source in a pure absorbing slab of optical thickness `tau`.
pub fn slab_leak(tau: f64) -> f64 {
    (0.5 - e3(tau)) / tau
}

/// Binomial standard error.
pub fn binom_se(p: f64, n: f64) -> f64 {
    (p * (1.0 - p) / n).sqrt()
}
