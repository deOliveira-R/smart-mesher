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
use sm_rt_api::{EndCode, ResponseKind, Scratch, StepEnd, TraceOptions, TrackState, Tracker};

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

fn region_i32(r: u32) -> i32 {
    if r == u32::MAX { -1 } else { r as i32 } // NONE maps to the complement
}

fn to_vec(a: [f64; 3]) -> Vector3 {
    Vector3::new(a[0], a[1], a[2])
}

fn to_pt(a: [f64; 3]) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}

/// Adapter over any `sm_rt_api::Tracker`.
pub struct TrackerAdapter {
    label: String,
    tracker: Box<dyn Tracker>,
    hinted: Vec<TrackState>,
}

impl TrackerAdapter {
    /// Wrap a tracker under `label`.
    pub fn new(label: impl Into<String>, tracker: Box<dyn Tracker>) -> Self {
        Self { label: label.into(), tracker, hinted: Vec::new() }
    }

    fn pool(threads: usize) -> Result<rayon::ThreadPool> {
        rayon::ThreadPoolBuilder::new().num_threads(threads.max(1)).build().map_err(trk)
    }

    fn track(&self, input: &WorkInput<'_>) -> Result<Output> {
        let rays: Vec<Ray> = (0..input.rays.len()).map(|i| Ray { origin: to_pt(input.rays.origin[i]), dir: to_vec(input.rays.dir[i]) }).collect();
        let pool = Self::pool(input.threads)?;
        let bundle = pool.install(|| self.tracker.trace_bundle(&rays, &TraceOptions::default())).map_err(trk)?;
        let mut out = Output::default();
        for i in 0..bundle.ray_count() {
            let range = bundle.ray_range(i).ok_or_else(|| BenchError::Tracker("malformed bundle offsets".into()))?;
            let mut t = Track { regions: Vec::new(), chords: Vec::new() };
            for s in range {
                let r = region_i32(bundle.region[s]);
                if t.regions.last() == Some(&r) {
                    if let Some(l) = t.chords.last_mut() {
                        *l += bundle.length[s];
                    }
                } else {
                    t.regions.push(r);
                    t.chords.push(bundle.length[s]);
                }
                out.counters.segments += 1;
            }
            if bundle.end[i] == EndCode::Lost {
                out.counters.lost += 1;
            }
            out.tracks.push(t);
        }
        out.counters.steps = out.counters.segments;
        Ok(out)
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
                StepEnd::RegionBoundary { to_region, .. } => region_i32(to_region.0),
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
                        self.history(h as u64, k, input, &sig_t, &sig_a, &mut tally, &mut c, &mut scratch)?;
                    }
                    Ok((tally, c))
                })
                .collect()
        });
        let mut out = Output { tally: vec![0.0; width], ..Output::default() };
        for p in parts {
            let (t, c) = p?; // fixed block order: the reduction does not depend on the thread count
            for (a, b) in out.tally.iter_mut().zip(&t) {
                *a += b;
            }
            out.counters.steps += c.steps;
            out.counters.histories += c.histories;
            out.counters.collisions += c.collisions;
            out.counters.lost += c.lost;
            out.counters.locates += c.locates;
        }
        Ok(out)
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
            let reg = st.region.get();
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
}

impl Adapter for TrackerAdapter {
    fn name(&self) -> String {
        self.label.clone()
    }

    fn precision(&self) -> &'static str {
        "f64"
    }

    fn build_info(&self) -> BuildInfo {
        let i = self.tracker.info();
        BuildInfo { structure_bytes: i.memory_bytes as u64, prototypes: i.prototypes as u64, copies: i.copies as u64 }
    }

    fn prepare(&mut self, w: Workload, input: &WorkInput<'_>) -> Result<()> {
        if w == Workload::Fire {
            let mut scratch = Scratch::new();
            self.hinted.clear();
            for i in 0..input.rays.len() {
                self.hinted.push(self.tracker.init_state(to_pt(input.rays.origin[i]), to_vec(input.rays.dir[i]), None, &mut scratch).map_err(trk)?);
            }
        }
        Ok(())
    }

    fn run(&mut self, w: Workload, input: &WorkInput<'_>) -> Result<Output> {
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
