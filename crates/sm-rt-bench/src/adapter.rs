//! The tracker adapter: one trait through which the runner drives every tracker (Rust types here,
//! the same JSON protocol for C++ drivers in RTBENCH-B), and the generic implementation over
//! `sm_rt_api::Tracker`, used with the contract's analytic stub in this task.
//!
//! Workloads:
//! * **W-track** follows each ray to leakage and returns its region sequence and chords.
//! * **W-fire** fires from a known interior point to the first region boundary; `FireLocate`
//!   includes the point location, `Fire` starts from a precomputed hint (separate rows).
//! * **W-flight** runs seeded Monte Carlo histories in surface mode (isotropic scatter,
//!   absorption, exponential flights from a per-history splitmix64 stream, per-region collision
//!   and track-length tallies). Delta and `block` modes are RTBENCH-B (they need the delta
//!   tracker).
//!
//! Counters are work counts (steps, segments, fires, collisions), never times.

use crate::cases::CaseData;
use crate::error::{BenchError, Result};
use crate::formats::{Rays, RegionSpec};
use crate::rng::{SplitMix64, sha256_hex};
use crate::truth::{Shape, Track};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use sm_core::{Aabb, Point3, Vector3};
use sm_geom_api::Ray;
use sm_rt_api::stub::{AnalyticWorld, Shape as StubShape};
use sm_rt_api::{EndCode, FlightInput, FlightOptions, ModePolicy, NodeId, ResponseKind, Scratch, StepEnd, TraceEnd, TraceOptions, TrackMode, TrackState, Tracker};

/// What an adapter is asked to do.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Workload {
    /// W-track.
    Track,
    /// W-fire from a hint.
    Fire,
    /// W-fire including point location.
    FireLocate,
    /// W-flight (surface mode).
    Flight,
}

impl Workload {
    /// Name used in result files.
    pub fn name(self) -> &'static str {
        match self {
            Workload::Track => "w-track",
            Workload::Fire => "w-fire",
            Workload::FireLocate => "w-fire-locate",
            Workload::Flight => "w-flight",
        }
    }
}

/// Work counters of one run (counts, no times).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Counters {
    /// `Tracker::step` calls (flight) or segments (track).
    pub steps: u64,
    /// Segments produced (track).
    pub segments: u64,
    /// Fires (fire, flight).
    pub fires: u64,
    /// Point locations.
    pub locates: u64,
    /// Histories run.
    pub histories: u64,
    /// Collisions scored.
    pub collisions: u64,
    /// Lost rays or histories.
    pub lost: u64,
    /// Surface segments of cursor flights (`WorkCounters::surface_segments`).
    pub surface_segments: u64,
    /// Delta segments of cursor flights.
    pub delta_segments: u64,
    /// Tentative collisions (delta sampling points that were located).
    pub tentative: u64,
    /// Tentative collisions rejected as virtual.
    pub rejected: u64,
    /// Partition nodes entered.
    pub node_visits: u64,
    /// Face tests (surface crossings tested).
    pub face_tests: u64,
    /// BVH descents (point locations from the root).
    pub bvh_descents: u64,
    /// Lattice index locates.
    pub lattice_index_locates: u64,
    /// Lattice elements (prototype instances) entered by W-track rays (hierarchy rows).
    pub pins_touched: u64,
}

impl Counters {
    /// Add the sm-rt work counters that the benchmark records.
    pub fn add_work(&mut self, w: &sm_rt_api::WorkCounters) {
        self.surface_segments += w.surface_segments;
        self.delta_segments += w.delta_segments;
        self.tentative += w.tentative;
        self.rejected += w.rejected;
        self.node_visits += w.node_visits;
        self.face_tests += w.face_tests;
        self.bvh_descents += w.bvh_descents;
        self.lattice_index_locates += w.lattice_index_locates;
    }

    /// The counters that only sm-rt rows fill, in a fixed order.
    fn extra(&self) -> [u64; 9] {
        [self.surface_segments, self.delta_segments, self.tentative, self.rejected, self.node_visits, self.face_tests, self.bvh_descents, self.lattice_index_locates, self.pins_touched]
    }
}

/// Everything an adapter produced for one workload run.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Output {
    /// Tracks (W-track).
    pub tracks: Vec<Track>,
    /// `(distance, region entered or -1)` per fire (W-fire).
    pub fires: Vec<(f64, i32)>,
    /// Tally vector (W-flight): collisions per region, track length per region, leaked, absorbed.
    pub tally: Vec<f64>,
    /// Sum over history blocks of the squared block tally (same layout as `tally`): the variance
    /// estimate of the tally comes from the block-to-block spread.
    pub tally_sq: Vec<f64>,
    /// Number of history blocks behind `tally_sq`.
    pub tally_blocks: usize,
    /// Work counters.
    pub counters: Counters,
}

impl Output {
    /// SHA-256 of the result bytes (tracks, fires, tally bits, counters): the determinism hash and
    /// the anti-dead-code checksum.
    pub fn content_hash(&self) -> String {
        let mut b = Vec::new();
        for t in &self.tracks {
            b.extend_from_slice(&(t.regions.len() as u32).to_le_bytes());
            for (r, c) in t.regions.iter().zip(&t.chords) {
                b.extend_from_slice(&r.to_le_bytes());
                b.extend_from_slice(&c.to_bits().to_le_bytes());
            }
        }
        for (d, r) in &self.fires {
            b.extend_from_slice(&d.to_bits().to_le_bytes());
            b.extend_from_slice(&r.to_le_bytes());
        }
        for v in &self.tally {
            b.extend_from_slice(&v.to_bits().to_le_bytes());
        }
        let c = &self.counters;
        for v in [c.steps, c.segments, c.fires, c.locates, c.histories, c.collisions, c.lost] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        // sm-rt-only counters join the hash when any is set, so rows of the stub keep their hash.
        let extra = c.extra();
        if extra.iter().any(|&v| v != 0) {
            for v in extra {
                b.extend_from_slice(&v.to_le_bytes());
            }
        }
        sha256_hex(&b)
    }
}

/// Input of one run.
#[derive(Clone, Copy, Debug)]
pub struct WorkInput<'a> {
    /// The rays (starts and directions; flights use ray `i % len` for history `i`).
    pub rays: &'a Rays,
    /// Regions with the benchmark's cross sections.
    pub regions: &'a [RegionSpec],
    /// Worker threads.
    pub threads: usize,
    /// Histories for W-flight.
    pub histories: usize,
    /// Seed of the history streams.
    pub seed: u64,
}

/// Build-side numbers of a tracker.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BuildInfo {
    /// Heap bytes of the built structure (`TrackerInfo::memory_bytes`).
    pub structure_bytes: u64,
    /// Prototypes.
    pub prototypes: u64,
    /// Placed copies.
    pub copies: u64,
}

/// A tracker the runner can drive.
pub trait Adapter {
    /// Tracker name in result files.
    fn name(&self) -> String;
    /// `f64`, `mixed`, ... (precision is a column, not a footnote).
    fn precision(&self) -> &'static str;
    /// Traversal mode of this adapter's rows (`None`: the mode the runner was given).
    fn mode(&self) -> Option<String> {
        None
    }
    /// True when the adapter traces the exact geometry (judged against the analytic truth);
    /// false for a facet or mesh twin (judged against the first twin adapter).
    fn exact(&self) -> bool {
        true
    }
    /// Build-side numbers.
    fn build_info(&self) -> BuildInfo;
    /// Untimed set-up for a workload (hints, preloaded states); the runner calls it before the clock starts.
    fn prepare(&mut self, _w: Workload, _input: &WorkInput<'_>) -> Result<()> {
        Ok(())
    }
    /// Run a workload.
    fn run(&mut self, w: Workload, input: &WorkInput<'_>) -> Result<Output>;
}

fn trk<E: core::fmt::Display>(e: E) -> BenchError {
    BenchError::Tracker(e.to_string())
}

fn to_vec(a: [f64; 3]) -> Vector3 {
    Vector3::new(a[0], a[1], a[2])
}

fn to_pt(a: [f64; 3]) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}

/// How W-flight flies on a tracker.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FlightMode {
    /// One `Tracker::step` per region or cell crossing (the cell-walking surface flight).
    Surface,
    /// Pure delta tracking through the majorant partition (`ModePolicy::Delta`, nothing blocked).
    Delta,
    /// Caller-driven mixed policy (ruling O3): delta tracking with `block` forcing surface
    /// segments in every leaf that holds more than one region.
    Block,
}

impl FlightMode {
    /// Name used in the `mode` column.
    pub fn name(self) -> &'static str {
        match self {
            FlightMode::Surface => "surface",
            FlightMode::Delta => "delta",
            FlightMode::Block => "block",
        }
    }
}

/// Adapter over any `sm_rt_api::Tracker`.
pub struct TrackerAdapter {
    label: String,
    mode: Option<String>,
    flight: FlightMode,
    /// Tracker region index to benchmark region id (empty: identity, `NONE` is `-1`).
    map: Vec<i32>,
    tracker: Box<dyn Tracker>,
    hinted: Vec<TrackState>,
    count_touched: bool,
    touched: Option<u64>,
    twin: bool,
}

impl TrackerAdapter {
    /// Wrap a tracker under `label`.
    pub fn new(label: impl Into<String>, tracker: Box<dyn Tracker>) -> Self {
        Self { label: label.into(), mode: None, flight: FlightMode::Surface, map: Vec::new(), tracker, hinted: Vec::new(), count_touched: false, touched: None, twin: false }
    }

    /// Name the traversal mode of this adapter's rows.
    pub fn with_mode(mut self, mode: impl Into<String>) -> Self {
        self.mode = Some(mode.into());
        self
    }

    /// Fly W-flight in `mode` (the tracker needs a majorant view for `Delta` and `Block`); the
    /// mode column becomes the flight mode unless a mode was named.
    pub fn with_flight_mode(mut self, mode: FlightMode) -> Self {
        self.flight = mode;
        if self.mode.is_none() && mode != FlightMode::Surface {
            self.mode = Some(mode.name().to_string());
        }
        self
    }

    /// Map tracker region indices to benchmark region ids (`-1`: the complement).
    pub fn with_region_map(mut self, map: Vec<i32>) -> Self {
        self.map = map;
        self
    }

    /// Mark the geometry as a facet or mesh twin (not the exact model).
    pub fn on_twin(mut self) -> Self {
        self.twin = true;
        self
    }

    /// Count the lattice elements each W-track ray enters (hierarchy rows), untimed in `prepare`.
    pub fn counting_touched(mut self) -> Self {
        self.count_touched = true;
        self
    }

    /// The wrapped tracker.
    pub fn tracker(&self) -> &dyn Tracker {
        &*self.tracker
    }

    fn bench(&self, r: sm_rt_api::RegionId) -> i32 {
        if r.is_none() {
            -1
        } else if self.map.is_empty() {
            r.0 as i32
        } else {
            self.map.get(r.get()).copied().unwrap_or(-1)
        }
    }

    fn pool(threads: usize) -> Result<rayon::ThreadPool> {
        rayon::ThreadPoolBuilder::new().num_threads(threads.max(1)).build().map_err(trk)
    }

    fn rays_of(input: &WorkInput<'_>) -> Vec<Ray> {
        (0..input.rays.len()).map(|i| Ray { origin: to_pt(input.rays.origin[i]), dir: to_vec(input.rays.dir[i]) }).collect()
    }

    fn track(&self, input: &WorkInput<'_>) -> Result<Output> {
        let rays = Self::rays_of(input);
        let pool = Self::pool(input.threads)?;
        let bundle = pool.install(|| self.tracker.trace_bundle(&rays, &TraceOptions::default())).map_err(trk)?;
        let mut out = Output::default();
        for i in 0..bundle.ray_count() {
            let range = bundle.ray_range(i).ok_or_else(|| BenchError::Tracker("malformed bundle offsets".into()))?;
            let mut t = Track { regions: Vec::new(), chords: Vec::new() };
            let mut left = false;
            for s in range {
                let r = self.bench(sm_rt_api::RegionId(bundle.region[s]));
                // W-track ends at the first entry into the complement (the truth scene and the
                // DAGMC loop do the same); segments past it are not counted either.
                left |= r < 0 && !t.regions.is_empty();
                if left {
                    continue;
                }
                out.counters.segments += 1;
                if t.regions.last() == Some(&r) {
                    if let Some(l) = t.chords.last_mut() {
                        *l += bundle.length[s];
                    }
                } else {
                    t.regions.push(r);
                    t.chords.push(bundle.length[s]);
                }
            }
            if bundle.end[i] == EndCode::Lost {
                out.counters.lost += 1;
            }
            out.tracks.push(t);
        }
        out.counters.steps = out.counters.segments;
        out.counters.pins_touched = self.touched.unwrap_or(0);
        Ok(out)
    }

    /// Lattice elements entered by the rays, by stepping each to leakage: the start element plus
    /// one per `SlotChange`. Counts the work of the walker, not time.
    fn count_pins(&self, input: &WorkInput<'_>) -> Result<u64> {
        let mut scratch = Scratch::new();
        let mut total = 0u64;
        for i in 0..input.rays.len() {
            let mut st = self.tracker.init_state(to_pt(input.rays.origin[i]), to_vec(input.rays.dir[i]), None, &mut scratch).map_err(trk)?;
            if st.cell.is_none() {
                continue;
            }
            total += 1;
            for _ in 0..self.tracker.info().max_events {
                let s = self.tracker.step(&mut st, f64::INFINITY, &mut scratch).map_err(trk)?;
                match s.end {
                    StepEnd::SlotChange { .. } => total += 1,
                    StepEnd::CellFace { .. } => {}
                    StepEnd::RegionBoundary { to_region, .. } => {
                        if to_region.is_none() || st.cell.is_none() {
                            break;
                        }
                    }
                    _ => break,
                }
            }
        }
        Ok(total)
    }

    fn fire(&self, input: &WorkInput<'_>, locate: bool) -> Result<Output> {
        let mut scratch = Scratch::new();
        let n = input.rays.len();
        if !locate && self.hinted.len() != n {
            return Err(BenchError::Usage("hinted fire needs `prepare` first".into()));
        }
        let mut out = Output::default();
        for i in 0..n {
            let mut st = if locate {
                out.counters.locates += 1;
                self.tracker.init_state(to_pt(input.rays.origin[i]), to_vec(input.rays.dir[i]), None, &mut scratch).map_err(trk)?
            } else {
                self.hinted[i].clone()
            };
            let r = self.tracker.fire(&mut st, f64::INFINITY, self.tracker.info().max_events, &mut scratch).map_err(trk)?;
            out.counters.fires += 1;
            let to = match r.end {
                StepEnd::RegionBoundary { to_region, .. } => self.bench(to_region),
                StepEnd::Lost(_) => {
                    out.counters.lost += 1;
                    -2
                }
                _ => -1,
            };
            out.fires.push((r.dist, to));
        }
        Ok(out)
    }

    fn flight(&self, input: &WorkInput<'_>) -> Result<Output> {
        let nreg = input.regions.iter().map(|r| r.id).max().map_or(0, |m| m as usize + 1);
        let mut sig_t = vec![0.0; nreg];
        let mut sig_a = vec![0.0; nreg];
        for r in input.regions {
            if r.id < 0 {
                return Err(BenchError::Usage("region ids must be non-negative".into()));
            }
            sig_t[r.id as usize] = r.sigma_t;
            sig_a[r.id as usize] = r.sigma_a;
        }
        if input.rays.is_empty() {
            return Err(BenchError::Usage("no rays".into()));
        }
        let cursor = if self.flight == FlightMode::Surface { None } else { Some(self.majorants(&sig_t)?) };
        const BLOCK: usize = 256;
        let width = 2 * nreg + 2;
        let n_blocks = input.histories.div_ceil(BLOCK);
        let pool = Self::pool(input.threads)?;
        let parts: Vec<Result<(Vec<f64>, Counters)>> = pool.install(|| {
            (0..n_blocks)
                .into_par_iter()
                .map(|b| {
                    let mut tally = vec![0.0; width];
                    let mut c = Counters::default();
                    let mut scratch = Scratch::new();
                    for h in (b * BLOCK)..((b + 1) * BLOCK).min(input.histories) {
                        let k = h % input.rays.len();
                        match &cursor {
                            None => self.history(h as u64, k, input, &sig_t, &sig_a, &mut tally, &mut c, &mut scratch)?,
                            Some(m) => self.history_cursor(h as u64, k, input, &sig_t, &sig_a, m, &mut tally, &mut c, &mut scratch)?,
                        }
                    }
                    c.add_work(&scratch.counters);
                    Ok((tally, c))
                })
                .collect()
        });
        let mut out = Output { tally: vec![0.0; width], tally_sq: vec![0.0; width], tally_blocks: n_blocks, ..Output::default() };
        for p in parts {
            let (t, c) = p?; // fixed block order: the reduction does not depend on the thread count
            for ((a, q), b) in out.tally.iter_mut().zip(out.tally_sq.iter_mut()).zip(&t) {
                *a += b;
                *q += b * b;
            }
            out.counters.steps += c.steps;
            out.counters.histories += c.histories;
            out.counters.collisions += c.collisions;
            out.counters.lost += c.lost;
            out.counters.locates += c.locates;
            out.counters.surface_segments += c.surface_segments;
            out.counters.delta_segments += c.delta_segments;
            out.counters.tentative += c.tentative;
            out.counters.rejected += c.rejected;
            out.counters.node_visits += c.node_visits;
            out.counters.face_tests += c.face_tests;
            out.counters.bvh_descents += c.bvh_descents;
            out.counters.lattice_index_locates += c.lattice_index_locates;
        }
        Ok(out)
    }

    /// Majorant of every partition node (the largest `sigma_t` of the regions of its set) and the
    /// nodes the `Block` mode forces to surface (leaves holding more than one region).
    fn majorants(&self, sig_t: &[f64]) -> Result<Majorants> {
        let view = self.tracker.majorant_view().ok_or_else(|| BenchError::Unavailable("the tracker has no majorant partition (build it with PartitionOptions::enabled on a volume mesh)".into()))?;
        let h = view.hierarchy();
        let sigma_of = |r: sm_rt_api::RegionId| -> f64 { usize::try_from(self.bench(r)).ok().and_then(|i| sig_t.get(i)).copied().unwrap_or(0.0) };
        let mut maj = Vec::with_capacity(h.node_count());
        let mut block = Vec::new();
        for n in 0..h.node_count() {
            let id = NodeId::new(n);
            let info = h.node(id).ok_or_else(|| BenchError::Tracker("node out of range".into()))?;
            let regs = h.set_regions(info.set).ok_or_else(|| BenchError::Tracker("material set out of range".into()))?;
            maj.push(regs.iter().map(|&r| sigma_of(r)).fold(0.0, f64::max));
            if self.flight == FlightMode::Block && info.is_leaf() && regs.len() > 1 {
                block.push(id);
            }
        }
        let policy = ModePolicy::Delta;
        Ok(Majorants { maj, block, policy })
    }

    #[allow(clippy::too_many_arguments)]
    fn history(&self, h: u64, k: usize, input: &WorkInput<'_>, sig_t: &[f64], sig_a: &[f64], tally: &mut [f64], c: &mut Counters, scratch: &mut Scratch) -> Result<()> {
        let nreg = sig_t.len();
        let mut rng = SplitMix64::for_index(input.seed, h);
        let mut st = self.tracker.init_state(to_pt(input.rays.origin[k]), to_vec(input.rays.dir[k]), None, scratch).map_err(trk)?;
        c.locates += 1;
        c.histories += 1;
        if st.cell.is_none() {
            c.lost += 1;
            return Ok(());
        }
        let mut tau = -(1.0 - rng.unit()).ln();
        loop {
            // Only the tracker's implicit complement (the facet world's exterior) is a leak, like
            // a RegionBoundary into `NONE`. Any other state the map sends to -1 (no entity, a
            // name without a spec, no region) is a mapping error: counted as lost.
            let Ok(reg) = usize::try_from(self.bench(st.region)) else {
                if !st.region.is_none() && self.tracker.regions().0.get(st.region.get()).is_some_and(|r| r.implicit_complement) {
                    tally[2 * nreg] += 1.0;
                } else {
                    c.lost += 1;
                }
                return Ok(());
            };
            if reg >= nreg {
                return Err(BenchError::Tracker(format!("region {reg} has no cross section")));
            }
            let sig = sig_t[reg];
            let max = if sig > 0.0 { tau / sig } else { f64::INFINITY };
            let s = self.tracker.step(&mut st, max, scratch).map_err(trk)?;
            c.steps += 1;
            tally[nreg + reg] += s.dist;
            tau -= sig * s.dist;
            match s.end {
                StepEnd::Limit => {
                    tally[reg] += 1.0;
                    c.collisions += 1;
                    if rng.unit() * sig < sig_a[reg] {
                        tally[2 * nreg + 1] += 1.0;
                        return Ok(());
                    }
                    let d = rng.direction();
                    st.dir = to_vec(d);
                    st.entered_via = None;
                    tau = -(1.0 - rng.unit()).ln();
                }
                StepEnd::CellFace { .. } | StepEnd::SlotChange { .. } => {}
                StepEnd::RegionBoundary { to_region, .. } => {
                    if to_region.is_none() || st.cell.is_none() {
                        tally[2 * nreg] += 1.0;
                        return Ok(());
                    }
                }
                StepEnd::Boundary { .. } => {
                    tally[2 * nreg] += 1.0;
                    return Ok(());
                }
                StepEnd::Lost(_) => {
                    c.lost += 1;
                    return Ok(());
                }
                _ => return Err(BenchError::Tracker("unknown step end".into())),
            }
        }
    }

    /// One history through the segment iterator (surface segments sample with the benchmark's
    /// own `sigma_t`; delta segments sample with the node majorant and locate each tentative
    /// collision). The delta modes leave the track-length slots at zero except in surface segments.
    #[allow(clippy::too_many_arguments)]
    fn history_cursor(&self, h: u64, k: usize, input: &WorkInput<'_>, sig_t: &[f64], sig_a: &[f64], m: &Majorants, tally: &mut [f64], c: &mut Counters, scratch: &mut Scratch) -> Result<()> {
        let view = self.tracker.majorant_view().ok_or_else(|| BenchError::Unavailable("no majorant view".into()))?;
        let nreg = sig_t.len();
        let sigma = |r: sm_rt_api::RegionId| -> Result<(usize, f64)> {
            let i = usize::try_from(self.bench(r)).map_err(|_| BenchError::Tracker("history is in the complement".into()))?;
            sig_t.get(i).map(|&s| (i, s)).ok_or_else(|| BenchError::Tracker(format!("region {i} has no cross section")))
        };
        let expo = |rng: &mut SplitMix64, rate: f64| -(1.0 - rng.unit()).ln() / rate;
        let flight_in = FlightInput::new(&m.maj).with_block(&m.block);
        let mut rng = SplitMix64::for_index(input.seed, h);
        let mut st = self.tracker.init_state(to_pt(input.rays.origin[k]), to_vec(input.rays.dir[k]), None, scratch).map_err(trk)?;
        c.locates += 1;
        c.histories += 1;
        if st.cell.is_none() {
            c.lost += 1;
            return Ok(());
        }
        let opts = FlightOptions::default().with_policy(m.policy);
        loop {
            let mut cur = view.begin(&st, &opts).map_err(trk)?;
            let mut hit: Option<usize> = None;
            while let Some(seg) = cur.next_segment(view, flight_in, &mut st, scratch).map_err(trk)? {
                c.steps += 1;
                match seg.mode {
                    TrackMode::Surface => {
                        let (reg, sig) = sigma(seg.region)?;
                        let len = seg.length();
                        if sig > 0.0 {
                            let s = expo(&mut rng, sig);
                            if s < len {
                                tally[nreg + reg] += s;
                                cur.stop_at(view, flight_in, &mut st, scratch, seg.t0 + s).map_err(trk)?;
                                hit = Some(reg);
                                break;
                            }
                        }
                        tally[nreg + reg] += len;
                    }
                    _ => {
                        if seg.sigma_m <= 0.0 {
                            continue;
                        }
                        let mut t = seg.t0;
                        loop {
                            t += expo(&mut rng, seg.sigma_m);
                            if t.partial_cmp(&seg.t1) != Some(core::cmp::Ordering::Less) {
                                break;
                            }
                            let loc = cur.probe(view, flight_in, scratch, t).map_err(trk)?;
                            let (reg, sg) = sigma(loc.region)?;
                            cur.check_collision(sg).map_err(trk)?;
                            if rng.unit() * seg.sigma_m < sg {
                                cur.stop_at(view, flight_in, &mut st, scratch, t).map_err(trk)?;
                                hit = Some(reg);
                                break;
                            }
                            c.rejected += 1; // a virtual collision: sm-rt cannot know the caller's verdict
                        }
                        if hit.is_some() {
                            break;
                        }
                    }
                }
            }
            let Some(reg) = hit else {
                match cur.end() {
                    Some(TraceEnd::Boundary { .. }) | Some(TraceEnd::LeftModel) => tally[2 * nreg] += 1.0,
                    _ => c.lost += 1,
                }
                return Ok(());
            };
            tally[reg] += 1.0;
            c.collisions += 1;
            if rng.unit() * sig_t[reg] < sig_a[reg] {
                tally[2 * nreg + 1] += 1.0;
                return Ok(());
            }
            st.dir = to_vec(rng.direction());
            st.entered_via = None;
        }
    }
}

/// Per-run slices of a cursor flight.
struct Majorants {
    maj: Vec<f64>,
    block: Vec<NodeId>,
    policy: ModePolicy,
}

impl Adapter for TrackerAdapter {
    fn name(&self) -> String {
        self.label.clone()
    }

    fn mode(&self) -> Option<String> {
        self.mode.clone()
    }

    fn exact(&self) -> bool {
        !self.twin
    }

    fn precision(&self) -> &'static str {
        "f64"
    }

    fn build_info(&self) -> BuildInfo {
        let i = self.tracker.info();
        BuildInfo { structure_bytes: i.memory_bytes as u64, prototypes: i.prototypes as u64, copies: i.copies as u64 }
    }

    fn prepare(&mut self, w: Workload, input: &WorkInput<'_>) -> Result<()> {
        match w {
            Workload::Fire => {
                let mut scratch = Scratch::new();
                self.hinted.clear();
                for i in 0..input.rays.len() {
                    self.hinted.push(self.tracker.init_state(to_pt(input.rays.origin[i]), to_vec(input.rays.dir[i]), None, &mut scratch).map_err(trk)?);
                }
            }
            Workload::Track if self.count_touched => {
                self.touched = Some(self.count_pins(input)?);
            }
            _ => {}
        }
        Ok(())
    }

    fn run(&mut self, w: Workload, input: &WorkInput<'_>) -> Result<Output> {
        if w == Workload::Track && self.count_touched && self.touched.is_none() {
            self.touched = Some(self.count_pins(input)?);
        }
        match w {
            Workload::Track => self.track(input),
            Workload::Fire => self.fire(input, false),
            Workload::FireLocate => self.fire(input, true),
            Workload::Flight => self.flight(input),
        }
    }
}

/// Build the contract's analytic stub from a case whose scene is one domain box with nested or
/// disjoint spheres and boxes inside (C1, C2, H1). Shape `i` is region `i`; the background is the
/// domain's region.
pub fn stub_world(data: &CaseData) -> Result<AnalyticWorld> {
    let scene = data.scene.as_ref().ok_or_else(|| BenchError::Unavailable("the case has no analytic scene".into()))?;
    let dom = scene
        .solids
        .iter()
        .find(|s| s.depth == 0 && matches!(s.shape, Shape::Box { .. }))
        .ok_or_else(|| BenchError::Unavailable("the stub needs a domain box".into()))?;
    let Shape::Box { lo, hi } = dom.shape else { return Err(BenchError::Unavailable("domain is not a box".into())) };
    let mut inner: Vec<&crate::truth::Solid> = scene.solids.iter().filter(|s| s.depth > 0).collect();
    inner.sort_by_key(|s| s.region);
    let mut shapes = Vec::new();
    for (i, s) in inner.iter().enumerate() {
        if s.region != i as i32 || s.region == dom.region {
            return Err(BenchError::Unavailable("the stub needs regions numbered like its shapes".into()));
        }
        shapes.push(match s.shape {
            Shape::Sphere { c, r } => StubShape::Sphere { center: to_pt(c), radius: r },
            Shape::Box { lo, hi } => StubShape::Box { lo: to_pt(lo), hi: to_pt(hi) },
            Shape::Cyl { .. } => return Err(BenchError::Unavailable("the stub has no cylinders (analytic tracker is RTBENCH-B)".into())),
        });
    }
    if dom.region != shapes.len() as i32 {
        return Err(BenchError::Unavailable("the stub background must carry the next region id".into()));
    }
    AnalyticWorld::simple(shapes, Aabb { lo: to_pt(lo), hi: to_pt(hi) }, ResponseKind::Vacuum).map_err(trk)
}

/// A stub adapter for a case, or `Unavailable`.
pub fn stub_adapter(data: &CaseData) -> Result<TrackerAdapter> {
    Ok(TrackerAdapter::new("stub", Box::new(stub_world(data)?)))
}
