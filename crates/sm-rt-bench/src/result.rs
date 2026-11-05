//! Result schema `smbench-result v1`, the environment record and the never-overwrite writer.
//!
//! A result file holds one row per `(tracker, workload, threads)` of one case. `result_hash` is a
//! SHA-256 over the non-timing fields only, so two runs can be compared by one string.

use crate::compare::Agreement;
use crate::error::{BenchError, Result};
use crate::rng::sha256_hex;
use serde::{Deserialize, Serialize};
use std::io::Write;
use std::path::{Path, PathBuf};

/// Schema tag.
pub const SCHEMA: &str = "smbench-result v1";

/// Median, minimum and maximum of the timed repetitions.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Stat {
    /// Median.
    pub median: f64,
    /// Minimum.
    pub min: f64,
    /// Maximum.
    pub max: f64,
    /// Number of repetitions.
    pub reps: usize,
}

impl Stat {
    /// Statistics of `v` (empty gives zeros).
    pub fn of(v: &[f64]) -> Self {
        if v.is_empty() {
            return Self::default();
        }
        let mut s = v.to_vec();
        s.sort_by(f64::total_cmp);
        let n = s.len();
        let median = if n % 2 == 1 { s[n / 2] } else { 0.5 * (s[n / 2 - 1] + s[n / 2]) };
        Self { median, min: s[0], max: s[n - 1], reps: n }
    }
}

/// Determinism record of a row.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Determinism {
    /// Hash of the outputs ordered by ray index.
    pub hash: String,
    /// Whether the hash was equal across thread counts and repetitions (`None` when not observed).
    /// Asserted for sm-rt, recorded as observed for competitors.
    pub stable: Option<bool>,
}

/// Geometry fidelity against the analytic reference (`geom_error`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct GeomError {
    /// Largest absolute chord error.
    pub max_chord_err: f64,
    /// Mean absolute chord error.
    pub mean_chord_err: f64,
    /// Tracks with a different region sequence, per `1e5` tracks.
    pub sequence_failures_per_1e5: f64,
    /// Tracks compared (0 when the case has no analytic truth).
    pub tracks: usize,
}

/// Summary of an agreement check as stored in a row.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct AgreementSummary {
    /// Tracks checked.
    pub checked: usize,
    /// Tracks that disagree.
    pub failed: usize,
    /// Chords dropped as grazing.
    pub grazing: usize,
}

impl From<&Agreement> for AgreementSummary {
    fn from(a: &Agreement) -> Self {
        Self { checked: a.checked, failed: a.failed, grazing: a.grazing }
    }
}

/// One row: a tracker in a mode on a workload with a thread count.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Row {
    /// Tracker name.
    pub tracker: String,
    /// Mode (`surface`, `delta`, `block`, `walk`, ...).
    pub mode: String,
    /// Workload name.
    pub workload: String,
    /// Threads.
    pub threads: usize,
    /// Precision of the traversal.
    pub precision: String,
    /// Rays (or histories) per second.
    pub rays_per_s: Stat,
    /// Segments per track (W-track).
    pub segments_per_track: f64,
    /// Nanoseconds per segment (derived from the same run).
    pub ns_per_segment: f64,
    /// Steps per history (W-flight, surface mode: one fire per flight).
    pub fires_per_history: f64,
    /// Build seconds (`load_s + tree_s`).
    pub build_s: f64,
    /// Load seconds.
    pub load_s: f64,
    /// Tree build seconds.
    pub tree_s: f64,
    /// Structure bytes as the tracker reports them (competitors: peak RSS minus empty-model RSS).
    pub structure_bytes: u64,
    /// Peak resident set size, bytes (0 when not measured).
    pub peak_rss_bytes: u64,
    /// Lost rays or histories.
    pub lost: u64,
    /// Geometry fidelity.
    pub geom_error: GeomError,
    /// Agreement with the reference.
    pub agreement: AgreementSummary,
    /// Determinism.
    pub determinism: Determinism,
    /// Work counters as recorded by the adapter.
    pub counters: crate::adapter::Counters,
}

/// The machine and moment of a run.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct Env {
    /// CPU model string.
    pub cpu_model: String,
    /// Frequency governor (`n/a` off Linux).
    pub governor: String,
    /// Logical CPUs.
    pub ncpu: usize,
    /// Operating system.
    pub os: String,
    /// Architecture.
    pub arch: String,
    /// UTC time, `YYYY-MM-DDTHH:MM:SSZ`.
    pub date_utc: String,
    /// Hash of cpu model, ncpu, os and arch (no host names).
    pub machine_id: String,
}

impl Env {
    /// Detect the current machine.
    pub fn detect() -> Self {
        let os = std::env::consts::OS.to_string();
        let arch = std::env::consts::ARCH.to_string();
        let ncpu = std::thread::available_parallelism().map_or(1, usize::from);
        let cpu_model = cpu_model(&os);
        let governor = std::fs::read_to_string("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor").map_or_else(|_| "n/a".to_string(), |s| s.trim().to_string());
        let machine_id = sha256_hex(format!("{cpu_model}|{ncpu}|{os}|{arch}").as_bytes())[..16].to_string();
        Self { cpu_model, governor, ncpu, os, arch, date_utc: utc_now(), machine_id }
    }

    /// True unless this is Linux on x86_64: such results are smoke results only (ruling Q9).
    pub fn smoke(&self) -> bool {
        self.os != "linux" || self.arch != "x86_64"
    }

    /// The UTC day, `YYYY-MM-DD`.
    pub fn day(&self) -> &str {
        self.date_utc.get(..10).unwrap_or(&self.date_utc)
    }
}

fn cpu_model(os: &str) -> String {
    let cmd = |prog: &str, args: &[&str]| std::process::Command::new(prog).args(args).output().ok().filter(|o| o.status.success()).map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string());
    let found = match os {
        "macos" => cmd("sysctl", &["-n", "machdep.cpu.brand_string"]),
        "linux" => std::fs::read_to_string("/proc/cpuinfo").ok().and_then(|t| t.lines().find(|l| l.starts_with("model name")).and_then(|l| l.split_once(':')).map(|(_, v)| v.trim().to_string())),
        _ => None,
    };
    found.filter(|s| !s.is_empty()).unwrap_or_else(|| "unknown".to_string())
}

/// Current UTC time as `YYYY-MM-DDTHH:MM:SSZ` (civil-from-days, no calendar dependency).
pub fn utc_now() -> String {
    let secs = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_secs());
    utc_from_secs(secs)
}

/// Format Unix seconds as `YYYY-MM-DDTHH:MM:SSZ`.
pub fn utc_from_secs(secs: u64) -> String {
    let days = (secs / 86_400) as i64;
    let rem = secs % 86_400;
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}-{m:02}-{d:02}T{:02}:{:02}:{:02}Z", rem / 3600, (rem % 3600) / 60, rem % 60)
}

/// One result file.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ResultFile {
    /// `smbench-result v1`.
    pub schema: String,
    /// Case name.
    pub case: String,
    /// Scale name of the case.
    pub scale: String,
    /// Tracker set (names joined with `+`).
    pub tracker_set: String,
    /// True for `--model` runs (never written under the repository).
    pub model_run: bool,
    /// Machine and date.
    pub env: Env,
    /// Source commit of the harness (empty when unknown).
    pub git_sha: String,
    /// Docker image digest of the competitor (empty for in-process trackers).
    pub image_digest: String,
    /// Rows.
    pub rows: Vec<Row>,
    /// Rows that could not be produced, each with the failing command or reason (never hashed).
    #[serde(default)]
    pub partial: Vec<String>,
    /// SHA-256 over the non-timing fields of the rows.
    pub result_hash: String,
}

/// The non-timing view of a row that `result_hash` covers.
#[derive(Serialize)]
struct StableRow<'a> {
    tracker: &'a str,
    mode: &'a str,
    workload: &'a str,
    threads: usize,
    precision: &'a str,
    segments_per_track: f64,
    fires_per_history: f64,
    lost: u64,
    geom_error: &'a GeomError,
    agreement: &'a AgreementSummary,
    determinism_hash: &'a str,
    counters: &'a crate::adapter::Counters,
}

impl ResultFile {
    /// A file with the schema tag set and no rows.
    pub fn new(case: &str, scale: &str, env: Env) -> Self {
        Self { schema: SCHEMA.to_string(), case: case.into(), scale: scale.into(), env, ..Self::default() }
    }

    /// Hash over case, scale and the non-timing fields of every row.
    pub fn compute_hash(&self) -> Result<String> {
        let stable: Vec<StableRow<'_>> = self
            .rows
            .iter()
            .map(|r| StableRow {
                tracker: &r.tracker,
                mode: &r.mode,
                workload: &r.workload,
                threads: r.threads,
                precision: &r.precision,
                segments_per_track: r.segments_per_track,
                fires_per_history: r.fires_per_history,
                lost: r.lost,
                geom_error: &r.geom_error,
                agreement: &r.agreement,
                determinism_hash: &r.determinism.hash,
                counters: &r.counters,
            })
            .collect();
        let body = serde_json::to_vec(&(&self.case, &self.scale, stable))?;
        Ok(sha256_hex(&body))
    }

    /// Set `result_hash` from the rows and the tracker set from their names.
    pub fn seal(&mut self) -> Result<()> {
        let mut names: Vec<&str> = self.rows.iter().map(|r| r.tracker.as_str()).collect();
        names.sort_unstable();
        names.dedup();
        self.tracker_set = names.join("+");
        self.result_hash = self.compute_hash()?;
        Ok(())
    }

    /// Parse a result file.
    pub fn from_json(text: &str) -> Result<Self> {
        let f: Self = serde_json::from_str(text)?;
        if f.schema != SCHEMA {
            return Err(BenchError::Format { what: "smbench-result", why: format!("schema {:?}, expected {SCHEMA:?}", f.schema) });
        }
        Ok(f)
    }
}

/// True when `path` is inside a git work tree (`.git` as a directory or a file, as in worktrees,
/// in the nearest existing ancestor or any parent of it).
pub fn inside_repo(path: &Path) -> bool {
    let mut p = path.to_path_buf();
    while !p.exists() {
        if !p.pop() {
            return false;
        }
    }
    let Ok(real) = p.canonicalize() else { return true }; // cannot prove it is outside
    real.ancestors().any(|a| a.join(".git").exists())
}

/// Write `f` into `dir` as `<case>-<tracker_set>-<UTC>-<pid>-<n>.json` (`n` counts writes of this process), opened with `create_new`.
/// A `model_run` file is refused when `dir` is inside a git work tree: the model and everything
/// derived from it must not reach the repository.
pub fn write_result(dir: &Path, f: &ResultFile) -> Result<PathBuf> {
    if f.model_run && inside_repo(dir) {
        return Err(BenchError::Fairness(format!("--model results must not be written inside the repository ({})", dir.display())));
    }
    std::fs::create_dir_all(dir).map_err(|e| BenchError::io(dir, e))?;
    let stamp: String = f.env.date_utc.chars().filter(|c| c.is_ascii_digit() || *c == 'T' || *c == 'Z').collect();
    static SEQ: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let seq = SEQ.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let name = format!("{}-{}-{}-{}-{}.json", f.case, if f.tracker_set.is_empty() { "none" } else { &f.tracker_set }, stamp, std::process::id(), seq);
    let path = dir.join(name);
    let mut file = std::fs::OpenOptions::new().write(true).create_new(true).open(&path).map_err(|e| BenchError::io(&path, e))?;
    let mut text = serde_json::to_vec_pretty(f)?;
    text.push(b'\n');
    file.write_all(&text).map_err(|e| BenchError::io(&path, e))?;
    Ok(path)
}

/// Toolchain record written next to the results as `bench-env.json` (no host names).
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct BenchEnv {
    /// `rustc --version` output.
    pub rustc: String,
    /// Machine and moment.
    pub env: Env,
}

impl BenchEnv {
    /// Detect the current environment.
    pub fn detect() -> Self {
        let rustc = std::process::Command::new("rustc").arg("--version").output().ok().map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string()).unwrap_or_default();
        Self { rustc, env: Env::detect() }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn utc_formatting_is_exact() {
        assert_eq!(utc_from_secs(0), "1970-01-01T00:00:00Z");
        assert_eq!(utc_from_secs(951_782_400), "2000-02-29T00:00:00Z");
        assert_eq!(utc_from_secs(1_793_836_800 + 3661), "2026-11-05T01:01:01Z");
    }

    #[test]
    fn stat_of_odd_and_even() {
        let s = Stat::of(&[3.0, 1.0, 2.0]);
        assert_eq!((s.median, s.min, s.max, s.reps), (2.0, 1.0, 3.0, 3));
        assert_eq!(Stat::of(&[1.0, 2.0, 3.0, 4.0]).median, 2.5);
        assert_eq!(Stat::of(&[]).reps, 0);
    }

    #[test]
    fn hash_ignores_timing_and_env() {
        let mut a = ResultFile::new("P1", "tiny", Env::default());
        a.rows.push(Row { tracker: "stub".into(), workload: "w-track".into(), threads: 1, ..Row::default() });
        a.seal().unwrap();
        let mut b = a.clone();
        b.env.cpu_model = "other".into();
        b.env.date_utc = "2030-01-01T00:00:00Z".into();
        b.rows[0].rays_per_s = Stat { median: 9.0, min: 1.0, max: 10.0, reps: 7 };
        b.rows[0].build_s = 3.0;
        assert_eq!(b.compute_hash().unwrap(), a.result_hash);
        b.rows[0].lost = 1;
        assert_ne!(b.compute_hash().unwrap(), a.result_hash);
    }

    #[test]
    fn json_round_trip() {
        let mut a = ResultFile::new("P1", "tiny", Env::detect());
        a.rows.push(Row { tracker: "stub".into(), ..Row::default() });
        a.seal().unwrap();
        let text = serde_json::to_string(&a).unwrap();
        assert_eq!(ResultFile::from_json(&text).unwrap(), a);
        assert!(ResultFile::from_json(&text.replace(SCHEMA, "smbench-result v0")).is_err());
    }
}
