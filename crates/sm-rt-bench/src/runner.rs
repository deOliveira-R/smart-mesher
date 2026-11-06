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

/// Agreement of W-flight tallies (4 sigma): per-region collision counts, leaked and absorbed
/// histories of `judged` against `reference`. The variance of each tally is estimated from the
/// spread of the history blocks (a block is 256 consecutive histories, the same blocks for every
/// adapter); a tally without block statistics uses Poisson variance. Slots that are zero on both
/// sides are not counted. Track-length slots are not compared: delta segments do not score them.
pub fn compare_tallies(judged: &Output, reference: &Output) -> AgreementSummary {
    let mut s = AgreementSummary::default();
    if judged.tally.len() != reference.tally.len() || judged.tally.len() < 2 {
        s.failed = 1;
        return s;
    }
    let nreg = (judged.tally.len() - 2) / 2;
    let var = |o: &Output, k: usize| -> f64 {
        let b = o.tally_blocks as f64;
        if o.tally_blocks < 2 || o.tally_sq.len() != o.tally.len() {
            return o.tally[k].abs();
        }
        let sample = (o.tally_sq[k] - o.tally[k] * o.tally[k] / b) / (b - 1.0);
        b * sample.max(0.0)
    };
    let slots = (0..nreg).chain([2 * nreg, 2 * nreg + 1]);
    for k in slots {
        let (a, b) = (judged.tally[k], reference.tally[k]);
        if a == 0.0 && b == 0.0 {
            continue;
        }
        s.checked += 1;
        let sigma = (var(judged, k) + var(reference, k)).sqrt();
        if (a - b).abs() > 4.0 * sigma {
            s.failed += 1;
        }
    }
    s
}

/// Run `cfg` on every adapter for one case and collect a sealed result file. `mode` names the
/// traversal mode of adapters that do not name their own.
///
/// Agreement: an adapter on the exact geometry is judged against the analytic truth; adapters on
/// a facet or mesh twin are judged against the first of them (they hold the same twin; their
/// distance from the truth is the `geom_error` column, not a disagreement). W-flight tallies
/// are judged against the first adapter's tally ([`compare_tallies`]).
pub fn run_case(data: &CaseData, adapters: &mut [Box<dyn Adapter>], mode: &str, cfg: &RunConfig, env: Env) -> Result<(ResultFile, Vec<Agreement>)> {
    let mut file = ResultFile::new(&data.case, &data.manifest.scale, env);
    let mut agreements = Vec::new();
    let regions = data.manifest.regions.clone();
    // Output hash of every (adapter, workload) per thread count, for the stability check.
    let mut hashes: Vec<Vec<Vec<String>>> = vec![vec![Vec::new(); cfg.workloads.len()]; adapters.len()];
    let mut row_of: Vec<Vec<Vec<usize>>> = vec![vec![Vec::new(); cfg.workloads.len()]; adapters.len()];
    let twin_ref = adapters.iter().position(|a| !a.exact());
    for &threads in &cfg.threads {
        for (wi, &w) in cfg.workloads.iter().enumerate() {
            let input = WorkInput { rays: &data.rays, regions: &regions, threads, histories: cfg.histories, seed: cfg.seed };
            let (samples, _trace) = run_interleaved(adapters, w, &input, cfg.reps, cfg.warmup)?;
            for (i, s) in samples.iter().enumerate() {
                let units = if w == Workload::Flight { cfg.histories } else { data.rays.len() } as f64;
                let rates: Vec<f64> = s.ns.iter().map(|ns| units / (ns.max(1.0) * 1e-9)).collect();
                let c = s.output.counters;
                let (geom, agree) = if w == Workload::Track {
                    let (g, truth_agree) = geom_error(data, &s.output.tracks, data.rays.len())?;
                    let agree = match twin_ref {
                        Some(r) if !adapters[i].exact() => {
                            if r == i {
                                Agreement { checked: s.output.tracks.len(), ..Agreement::default() }
                            } else {
                                compare::compare(&s.output.tracks, &samples[r].output.tracks, data.manifest.diag, data.manifest.chord_tol)?
                            }
                        }
                        _ => truth_agree,
                    };
                    (g, agree)
                } else {
                    (GeomError::default(), Agreement::default())
                };
                let tally_agree = if w == Workload::Flight && i > 0 { Some(compare_tallies(&s.output, &samples[0].output)) } else { None };
                let median_ns = Stat::of(&s.ns).median;
                let bi = adapters[i].build_info();
                let row = Row {
                    tracker: adapters[i].name(),
                    mode: adapters[i].mode().unwrap_or_else(|| mode.to_string()),
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
                    agreement: tally_agree.unwrap_or_else(|| AgreementSummary::from(&agree)),
                    determinism: Determinism { hash: s.output.content_hash(), stable: None },
                    counters: c,
                    ..Row::default()
                };
                let mut row = row;
                row.determinism.stable = Some(s.hashes.iter().all(|h| *h == s.hashes[0]));
                hashes[i][wi].push(row.determinism.hash.clone());
                row_of[i][wi].push(file.rows.len());
                file.rows.push(row);
                if w == Workload::Track {
                    agreements.push(agree);
                }
            }
        }
    }
    // Stability across thread counts: the same (adapter, workload) must hash equal for every count.
    for i in 0..adapters.len() {
        for wi in 0..cfg.workloads.len() {
            let equal = hashes[i][wi].windows(2).all(|p| p[0] == p[1]);
            for &r in &row_of[i][wi] {
                file.rows[r].determinism.stable = file.rows[r].determinism.stable.map(|s| s && equal);
            }
        }
    }
    file.seal()?;
    Ok((file, agreements))
}
