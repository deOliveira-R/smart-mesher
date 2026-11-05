//! The runner: ABAB interleaving of trackers per repetition, row construction, agreement and
//! determinism bookkeeping.
//!
//! Rates come from `std::time::Instant` and are only ever recorded, never asserted. Work counters,
//! hashes and analytic values are what tests check.

use crate::adapter::{Adapter, Output, WorkInput, Workload};
use crate::cases::CaseData;
use crate::compare::{self, Agreement};
use crate::error::Result;
use crate::result::{AgreementSummary, Determinism, Env, GeomError, ResultFile, Row, Stat};
use crate::truth::Track;
use std::time::Instant;

/// One recorded call of the interleaved loop.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TraceEntry {
    /// Repetition index, warm-up repetitions first.
    pub rep: usize,
    /// Adapter index.
    pub adapter: usize,
    /// True during warm-up.
    pub warmup: bool,
}

/// Samples of one adapter for one workload.
#[derive(Clone, Debug, Default)]
pub struct Samples {
    /// Nanoseconds of every timed repetition.
    pub ns: Vec<f64>,
    /// Output of the last repetition.
    pub output: Output,
    /// Output hash of every repetition (warm-up included).
    pub hashes: Vec<String>,
}

/// Run `warmup + reps` repetitions of `w`; in each repetition every adapter runs once, in index
/// order (fairness rule b: ABAB per repetition). Returns per-adapter samples and the call trace.
pub fn run_interleaved(adapters: &mut [Box<dyn Adapter>], w: Workload, input: &WorkInput<'_>, reps: usize, warmup: usize) -> Result<(Vec<Samples>, Vec<TraceEntry>)> {
    let mut samples: Vec<Samples> = adapters.iter().map(|_| Samples::default()).collect();
    let mut trace = Vec::new();
    for a in adapters.iter_mut() {
        a.prepare(w, input)?;
    }
    for rep in 0..warmup + reps {
        for (i, a) in adapters.iter_mut().enumerate() {
            let t0 = Instant::now();
            let out = a.run(w, input)?;
            let ns = t0.elapsed().as_nanos() as f64;
            trace.push(TraceEntry { rep, adapter: i, warmup: rep < warmup });
            samples[i].hashes.push(out.content_hash());
            if rep >= warmup {
                samples[i].ns.push(ns);
            }
            samples[i].output = out;
        }
    }
    Ok((samples, trace))
}

/// Settings of a case run.
#[derive(Clone, Debug)]
pub struct RunConfig {
    /// Timed repetitions.
    pub reps: usize,
    /// Warm-up repetitions.
    pub warmup: usize,
    /// Thread counts to run (each gives its own rows).
    pub threads: Vec<usize>,
    /// Workloads.
    pub workloads: Vec<Workload>,
    /// Histories of W-flight.
    pub histories: usize,
    /// Seed of the history streams.
    pub seed: u64,
}

impl Default for RunConfig {
    fn default() -> Self {
        Self { reps: 7, warmup: 3, threads: vec![1], workloads: vec![Workload::Track, Workload::Fire, Workload::FireLocate, Workload::Flight], histories: 10_000, seed: 0x5eed }
    }
}

/// Compare the tracks of an output with the analytic truth of the case.
pub fn geom_error(data: &CaseData, tracks: &[Track], n: usize) -> Result<(GeomError, Agreement)> {
    let Some(scene) = &data.scene else { return Ok((GeomError::default(), Agreement::default())) };
    let truth: Vec<Track> = (0..n).map(|i| scene.track(data.rays.origin[i], data.rays.dir[i])).collect();
    let rep = compare::compare(tracks, &truth, data.manifest.diag, data.manifest.chord_tol)?;
    let g = GeomError { max_chord_err: rep.max_chord_err, mean_chord_err: rep.mean_chord_err, sequence_failures_per_1e5: if rep.checked == 0 { 0.0 } else { rep.sequence_failures as f64 * 1e5 / rep.checked as f64 }, tracks: rep.checked };
    Ok((g, rep))
}

/// Run `cfg` on every adapter for one case and collect a sealed result file. `mode` names the
/// traversal mode of the adapters (all adapters of one call share it).
pub fn run_case(data: &CaseData, adapters: &mut [Box<dyn Adapter>], mode: &str, cfg: &RunConfig, env: Env) -> Result<(ResultFile, Vec<Agreement>)> {
    let mut file = ResultFile::new(&data.case, &data.manifest.scale, env);
    let mut agreements = Vec::new();
    let regions = data.manifest.regions.clone();
    let mut hashes: Vec<(usize, Workload, Vec<String>)> = Vec::new(); // (adapter, workload, per-thread-count hash)
    for &threads in &cfg.threads {
        for &w in &cfg.workloads {
            let input = WorkInput { rays: &data.rays, regions: &regions, threads, histories: cfg.histories, seed: cfg.seed };
            let (samples, _trace) = run_interleaved(adapters, w, &input, cfg.reps, cfg.warmup)?;
            for (i, s) in samples.iter().enumerate() {
                let units = if w == Workload::Flight { cfg.histories } else { data.rays.len() } as f64;
                let rates: Vec<f64> = s.ns.iter().map(|ns| units / (ns.max(1.0) * 1e-9)).collect();
                let c = s.output.counters;
                let (geom, agree) = if w == Workload::Track { geom_error(data, &s.output.tracks, data.rays.len())? } else { (GeomError::default(), Agreement::default()) };
                let median_ns = Stat::of(&s.ns).median;
                let bi = adapters[i].build_info();
                let row = Row {
                    tracker: adapters[i].name(),
                    mode: mode.to_string(),
                    workload: w.name().to_string(),
                    threads,
                    precision: adapters[i].precision().to_string(),
                    rays_per_s: Stat::of(&rates),
                    segments_per_track: if w == Workload::Track && !data.rays.is_empty() { c.segments as f64 / data.rays.len() as f64 } else { 0.0 },
                    ns_per_segment: if c.segments > 0 { median_ns / c.segments as f64 } else { 0.0 },
                    fires_per_history: if w == Workload::Flight && c.histories > 0 { c.steps as f64 / c.histories as f64 } else if w == Workload::Fire || w == Workload::FireLocate { 1.0 } else { 0.0 },
                    structure_bytes: bi.structure_bytes,
                    lost: c.lost,
                    geom_error: geom,
                    agreement: AgreementSummary::from(&agree),
                    determinism: Determinism { hash: s.output.content_hash(), stable: None },
                    counters: c,
                    ..Row::default()
                };
                let same = s.hashes.iter().all(|h| *h == s.hashes[0]);
                hashes.push((i, w, vec![row.determinism.hash.clone()]));
                let mut row = row;
                row.determinism.stable = Some(same);
                file.rows.push(row);
                if w == Workload::Track {
                    agreements.push(agree);
                }
            }
        }
    }
    // Stability across thread counts: the same (adapter, workload) must hash equal for every count.
    for row in &mut file.rows {
        let all_equal = hashes.iter().filter(|(i, w, _)| adapters[*i].name() == row.tracker && w.name() == row.workload).all(|(_, _, h)| h[0] == row.determinism.hash);
        row.determinism.stable = row.determinism.stable.map(|s| s && all_equal);
    }
    file.seal()?;
    Ok((file, agreements))
}
