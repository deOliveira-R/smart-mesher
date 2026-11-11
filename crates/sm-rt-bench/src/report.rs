//! `report`: renders result files into Markdown tables, enforcing the fairness gates as code.
//!
//! * Refuses to render when the files come from different machines or UTC days (fairness rule b).
//! * A tracker whose agreement check failed in a case shows `n/c (disagrees)` in every rate cell
//!   of that case (rule g); the failure counts stay visible.
//! * Results that are not from bare-metal Linux x86_64 with the performance governor are marked
//!   as smoke results (ruling Q9).
//! * The pre-registered verdict (`VERDICT.md`) is computed from the rows: [`verdicts`] per
//!   session, [`render_sessions`] across three sessions on different days (signs only).
//! * Polyhedral rows are a separately labelled table; no verdict reads them (ruling D6).
//! * No ratio between trackers is ever computed: the tables hold measured values only.

use crate::error::{BenchError, Result};
use crate::result::{ResultFile, Row};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write;

/// Trackers a full benchmark covers; absent ones are listed under "not measured".
pub const KNOWN_TRACKERS: [&str; 4] = ["smrt", "dagmc-moab", "dagmc-dd", "xdg"];

/// What the benchmark does not do, whatever the numbers say (fairness rule h).
pub const NOT_MEASURED: [&str; 11] = [
    "GPU rows (XDG GPRT) and the libMesh backend",
    "mesh-tally workloads (MOC/Sn segment lists, segment-callback API)",
    "multi-node scaling and Serpent",
    "reflective faces and instancing in DAGMC (DAGMC has neither; reflective stays an sm-rt-only test)",
    "cell walking on tets is not claimed novel (XDG does it): rows separate walking the same tets from analytic and hierarchy modes",
    "the C-ABI call cost (it stays in sm-rt-capi/bench)",
    "x86_64 and aarch64 results are never merged into one table",
    "H1 analytic row (the analytic world cannot hold a box inside a solid: needs a union of half-spaces in sm-rt)",
    "P2 tet twin (the pins stand in vacuum, so no volume fills their box)",
    "--model runs on imported periodic faces (sm-cadio leaves their pcurves unset)",
    "core pinning inside Docker (not possible in Docker Desktop's VM): rows say `cores not pinned`",
];

/// Extra limitation of a Mac smoke run, printed when the machine is macOS.
pub const MAC_ASYMMETRY: &str = "Apple Silicon has performance and efficiency cores (M1 Max: 8P+2E): thread counts above the performance cores mix core types, and the VM scheduler hides which is which";

/// Check that `files` may share a table: same CPU model, OS, architecture, machine and UTC day.
pub fn check_same_machine(files: &[ResultFile]) -> Result<()> {
    let Some(first) = files.first() else { return Err(BenchError::Fairness("no result files".into())) };
    for f in &files[1..] {
        let (a, b) = (&first.env, &f.env);
        let what = if a.cpu_model != b.cpu_model {
            Some(("cpu_model", a.cpu_model.clone(), b.cpu_model.clone()))
        } else if a.os != b.os || a.arch != b.arch {
            Some(("os/arch", format!("{}-{}", a.os, a.arch), format!("{}-{}", b.os, b.arch)))
        } else if a.machine_id != b.machine_id {
            Some(("machine_id", a.machine_id.clone(), b.machine_id.clone()))
        } else if a.day() != b.day() {
            Some(("date_utc day", a.day().to_string(), b.day().to_string()))
        } else {
            None
        };
        if let Some((field, x, y)) = what {
            return Err(BenchError::Fairness(format!("refusing to compare results with different {field}: {x:?} vs {y:?}")));
        }
    }
    Ok(())
}

/// Refuse a challenger row whose `geometry` is empty (legacy file, custom adapter): it could be
/// the stair-stepped tets, so it fails closed instead of entering the k-of-n verdict as non-L3.
pub fn check_geometry(files: &[ResultFile]) -> Result<()> {
    for f in files {
        if let Some(r) = f.rows.iter().find(|r| r.tracker == CHALLENGER && r.geometry.is_empty()) {
            return Err(BenchError::Fairness(format!("refusing {} row {}/{}/{}: empty `geometry` (re-run with a current adapter; the report cannot tell whether it is L3)", r.tracker, f.case, r.mode, r.workload)));
        }
    }
    Ok(())
}

/// True when any row of `r`'s tracker and mode in `f` failed its agreement check (rule g).
/// Polyhedral rows are a separate population (ruling D6): they neither judge nor are judged by
/// non-polyhedral rows, however they are named (mode or workload).
fn disagrees(f: &ResultFile, r: &Row) -> bool {
    let class = (is_polyhedral(r), is_l3(r));
    f.rows.iter().any(|x| x.tracker == r.tracker && x.mode == r.mode && (is_polyhedral(x), is_l3(x)) == class && x.agreement.failed > 0)
}

/// True for an sm-rt row on the stair-stepped tets (`geometry` = `tets`: the W-fire `walk` and
/// `jump` rows, the W-flight `walk`/`delta`/`block` rows of the mesh twin): geometry level L3 of
/// `DESIGN.md`. The baselines hold the facet twin (icospheres), so a cell of these rows would compare
/// two geometries; they render in their own table and no verdict reads them (their own checks stay in
/// the agreement column). The adapter stamps the field, so no mode name decides it. Polyhedral rows
/// keep their own table ([`is_polyhedral`]).
pub fn is_l3(r: &Row) -> bool {
    r.tracker == CHALLENGER && r.geometry == "tets" && !is_polyhedral(r)
}

/// True for a polyhedral-mesh row: its `mode` or `workload` starts with `poly` (the L2b rows;
/// W4-RT-ROWS names them `poly-*`). Such rows are a different mesh, so no verdict reads them.
pub fn is_polyhedral(r: &Row) -> bool {
    r.mode.starts_with("poly") || r.workload.starts_with("poly")
}

fn rate_cell(failed: bool, f: &crate::result::Stat) -> String {
    if failed { "n/c (disagrees)".to_string() } else { format!("{:.4e} [{:.3e}, {:.3e}]", f.median, f.min, f.max) }
}

/// Render the report for `files` (one per case and tracker set).
pub fn render(files: &[ResultFile]) -> Result<String> {
    render_with_docker(files, None)
}

/// Like [`render`], with the Docker VM block of `bench-env.json` (CPUs, memory, kernel) printed
/// under the banner, followed by the identity of every competitor image in the files.
pub fn render_with_docker(files: &[ResultFile], vm: Option<&crate::result::DockerVm>) -> Result<String> {
    check_same_machine(files)?;
    check_geometry(files)?;
    let env = &files[0].env;
    let mut out = String::new();
    let _ = writeln!(out, "# smrt-bench report\n");
    if env.smoke() {
        let _ = writeln!(out, "> **SMOKE RESULT.** Produced on {}-{}, not on bare-metal Linux x86_64 with the performance governor ({}). It is a check that the harness runs, not a published table (ruling Q9).\n", env.os, env.arch, env.smoke_reasons().join("; "));
    }
    let _ = writeln!(out, "Machine: {} ({} logical CPUs, governor {}), OS {}-{}, UTC day {}, machine id {}.\n", env.cpu_model, env.ncpu, env.governor, env.os, env.arch, env.day(), env.machine_id);
    if let Some(v) = vm {
        let _ = writeln!(out, "Docker VM: {} CPUs, {:.1} GiB memory, kernel {}, {} (server {}); container CPU line: {}. Competitor rows ran inside this VM, cores not pinned.\n", v.ncpu, v.mem_total_bytes as f64 / (1u64 << 30) as f64, v.kernel, v.operating_system, v.server_version, v.container_cpu);
    }
    let mut images: std::collections::BTreeMap<&str, &crate::result::ImageIdentity> = std::collections::BTreeMap::new();
    for f in files {
        for (t, id) in &f.images {
            images.entry(t.as_str()).or_insert(id);
        }
    }
    if !images.is_empty() {
        let _ = writeln!(out, "Images:\n");
        for (t, id) in &images {
            let _ = writeln!(out, "- {t}: {} {}, base index sha256:{}, sources [{}], MOAB {}, conda lock sha256:{}, versions sha256:{}", id.image_id, id.platform, id.base_index_digest, id.source_commits.join("; "), if id.moab_version.is_empty() { "n/a" } else { &id.moab_version }, if id.conda_lock_sha256.is_empty() { "none" } else { &id.conda_lock_sha256 }, id.versions_sha256);
        }
        out.push('\n');
    }
    let cases: BTreeSet<&str> = files.iter().map(|f| f.case.as_str()).collect();
    for case in &cases {
        let _ = writeln!(out, "## {case}\n");
        let cfiles: Vec<&ResultFile> = files.iter().filter(|f| f.case == *case).collect();
        let _ = writeln!(out, "| tracker | mode | workload | threads | precision | rays/s median [min, max] | segments/track | ns/segment | steps/history | build s | structure bytes | lost | max chord err | seq. failures per 1e5 | agreement failed/checked (grazing) | determinism |");
        let _ = writeln!(out, "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
        let mut seen: BTreeSet<&str> = BTreeSet::new();
        let mut poly_rows: Vec<(&ResultFile, &Row)> = Vec::new();
        let mut l3_rows: Vec<(&ResultFile, &Row)> = Vec::new();
        for f in &cfiles {
            for r in &f.rows {
                seen.insert(r.tracker.as_str());
                if is_l3(r) {
                    l3_rows.push((f, r));
                    continue;
                }
                if is_polyhedral(r) {
                    poly_rows.push((f, r));
                    continue;
                }
                let failed = disagrees(f, r);
                let det = format!("{} {}", r.determinism.hash.get(..12).unwrap_or(""), match r.determinism.stable {
                    Some(true) => "stable",
                    Some(false) => "UNSTABLE",
                    None => "not observed",
                });
                let _ = writeln!(
                    out,
                    "| {} | {} | {} | {} | {} | {} | {:.3} | {:.1} | {:.3} | {:.3} | {} | {} | {:.3e} | {:.2} | {}/{} ({}) | {} |",
                    if env.smoke() { format!("{} (smoke)", r.tracker) } else { r.tracker.clone() },
                    r.mode,
                    r.workload,
                    r.threads,
                    r.precision,
                    rate_cell(failed, &r.rays_per_s),
                    r.segments_per_track,
                    r.ns_per_segment,
                    r.fires_per_history,
                    r.build_s,
                    r.structure_bytes,
                    r.lost,
                    r.geom_error.max_chord_err,
                    r.geom_error.sequence_failures_per_1e5,
                    r.agreement.failed,
                    r.agreement.checked,
                    r.agreement.grazing,
                    det
                );
            }
        }
        if !l3_rows.is_empty() {
            let _ = writeln!(out, "\nL3 rows (stair-stepped tets, not the facet twin the baselines hold; never read by the verdict, VERDICT.md):\n");
            let _ = writeln!(out, "| tracker | mode | workload | threads | precision | rays/s median [min, max] | build s | structure bytes | lost | agreement failed/checked (grazing) |");
            let _ = writeln!(out, "|---|---|---|---|---|---|---|---|---|---|");
            for (f, r) in l3_rows {
                let failed = disagrees(f, r);
                let _ = writeln!(out, "| {} | {} | {} | {} | {} | {} | {:.3} | {} | {} | {}/{} ({}) |", r.tracker, r.mode, r.workload, r.threads, r.precision, rate_cell(failed, &r.rays_per_s), r.build_s, r.structure_bytes, r.lost, r.agreement.failed, r.agreement.checked, r.agreement.grazing);
            }
        }
        if !poly_rows.is_empty() {
            let _ = writeln!(out, "\nPolyhedral rows (different mesh, same geometry; never read by the verdict, ruling D6):\n");
            let _ = writeln!(out, "| tracker | mode | workload | threads | precision | rays/s median [min, max] | build s | structure bytes | lost | agreement failed/checked (grazing) |");
            let _ = writeln!(out, "|---|---|---|---|---|---|---|---|---|---|");
            for (f, r) in poly_rows {
                let failed = disagrees(f, r);
                let _ = writeln!(out, "| {} | {} | {} | {} | {} | {} | {:.3} | {} | {} | {}/{} ({}) |", r.tracker, r.mode, r.workload, r.threads, r.precision, rate_cell(failed, &r.rays_per_s), r.build_s, r.structure_bytes, r.lost, r.agreement.failed, r.agreement.checked, r.agreement.grazing);
            }
        }
        let partial: Vec<&String> = cfiles.iter().flat_map(|f| f.partial.iter()).collect();
        if !partial.is_empty() {
            let _ = writeln!(out, "\nPartial (not run, nothing invented):\n");
            for p in partial {
                let _ = writeln!(out, "- {p}");
            }
        }
        let notes: Vec<&String> = cfiles.iter().flat_map(|f| f.notes.iter()).collect();
        if !notes.is_empty() {
            let _ = writeln!(out, "\nObservations:\n");
            for n in notes {
                let _ = writeln!(out, "- {n}");
            }
        }
        let missing: Vec<&str> = KNOWN_TRACKERS.iter().copied().filter(|t| !seen.contains(t)).collect();
        if !missing.is_empty() {
            let _ = writeln!(out, "\nNot measured in this report for {case}: {}.", missing.join(", "));
        }
        out.push('\n');
    }
    out.push_str(&verdict_section(files, env.smoke()));
    let _ = writeln!(out, "## What was not measured\n");
    for l in NOT_MEASURED {
        let _ = writeln!(out, "- {l}");
    }
    if env.os == "macos" {
        let _ = writeln!(out, "- {MAC_ASYMMETRY}");
    }
    Ok(out)
}

// Pre-registered verdict (VERDICT.md). Pure functions of the rows: no wall-clock gating.

/// The tracker whose rows are judged.
pub const CHALLENGER: &str = "smrt";
/// A win carries a qualifier when the challenger is more than this factor worse on memory or build.
pub const QUALIFIER_FACTOR: f64 = 2.0;

/// Outcome of one cell (or of one session in the aggregate).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Sign {
    /// Faster by more than the larger spread at every compared thread count.
    Win,
    /// The baseline is faster by more than the larger spread at every compared thread count.
    Loss,
    /// Neither: medians within the spread, or mixed signs across thread counts.
    NoClaim,
    /// An agreement check failed or was not run (`n/c`, rule g).
    NotComparable,
    /// Rows needed by the criteria are missing.
    Incomplete,
}

impl Sign {
    /// Short label used in tables.
    pub fn label(self) -> &'static str {
        match self {
            Sign::Win => "win",
            Sign::Loss => "loss",
            Sign::NoClaim => "no claim",
            Sign::NotComparable => "n/c",
            Sign::Incomplete => "incomplete",
        }
    }
}

/// Verdict for one (case, workload, challenger mode) against one baseline tracker.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CellVerdict {
    /// Case name.
    pub case: String,
    /// Workload.
    pub workload: String,
    /// Challenger mode.
    pub mode: String,
    /// Baseline tracker.
    pub baseline: String,
    /// Outcome.
    pub sign: Sign,
    /// Memory and build qualifiers of a win (more than [`QUALIFIER_FACTOR`] worse; numbers shown).
    pub qualifiers: Vec<String>,
    /// Why, in words.
    pub detail: String,
}

impl CellVerdict {
    /// `case/workload/mode`.
    pub fn name(&self) -> String {
        format!("{}/{}/{}", self.case, self.workload, self.mode)
    }

    /// The table text: `win`, `win (qualified: ...)` or `<sign> (<detail>)`.
    pub fn text(&self) -> String {
        match (self.sign, self.qualifiers.is_empty()) {
            (Sign::Win, false) => format!("win (qualified: {})", self.qualifiers.join("; ")),
            (Sign::Win, true) => "win".to_string(),
            (s, _) => format!("{} ({})", s.label(), self.detail),
        }
    }
}

type Pair<'a> = (&'a ResultFile, &'a Row);

fn agrees(f: &ResultFile, r: &Row) -> bool {
    r.agreement.checked > 0 && !disagrees(f, r)
}

fn spread(r: &Row) -> f64 {
    r.rays_per_s.max - r.rays_per_s.min
}

fn timed(r: &Row) -> bool {
    let s = &r.rays_per_s;
    s.reps > 0 && s.median.is_finite() && s.min.is_finite() && s.max.is_finite()
}

fn qualifiers(c: &Row, b: &Row) -> Vec<String> {
    let mut q = Vec::new();
    let worse = |x: f64, y: f64| x > 0.0 && y > 0.0 && x > QUALIFIER_FACTOR * y;
    if worse(c.build_s, b.build_s) {
        q.push(format!("build {:.3} s vs {:.3} s, more than 2x", c.build_s, b.build_s));
    }
    if worse(c.structure_bytes as f64, b.structure_bytes as f64) {
        q.push(format!("structure {} B vs {} B, more than 2x", c.structure_bytes, b.structure_bytes));
    }
    if worse(c.peak_rss_bytes as f64, b.peak_rss_bytes as f64) {
        q.push(format!("peak RSS {} B vs {} B, more than 2x", c.peak_rss_bytes, b.peak_rss_bytes));
    }
    q
}

/// One thread count: the challenger row against the best agreeing baseline row of the same
/// (case, workload, threads) across baseline modes (best effort for each side, rule c).
fn point(c: Pair, baselines: &[Pair]) -> (Sign, Vec<String>, String) {
    let (cf, cr) = c;
    let rows: Vec<Pair> = baselines.iter().copied().filter(|(_, r)| r.workload == cr.workload && r.threads == cr.threads).collect();
    if rows.is_empty() {
        return (Sign::Incomplete, vec![], format!("no baseline row at {} threads", cr.threads));
    }
    if !agrees(cf, cr) {
        return (Sign::NotComparable, vec![], "challenger agreement failed or not checked".into());
    }
    let mut ok: Vec<Pair> = rows.into_iter().filter(|&(f, r)| agrees(f, r)).collect();
    if ok.is_empty() {
        return (Sign::NotComparable, vec![], "baseline agreement failed or not checked".into());
    }
    if !timed(cr) || ok.iter().any(|(_, r)| !timed(r)) {
        return (Sign::Incomplete, vec![], "no timing samples".into());
    }
    ok.sort_by(|a, b| b.1.rays_per_s.median.total_cmp(&a.1.rays_per_s.median).then_with(|| a.1.mode.cmp(&b.1.mode)));
    let b = ok[0].1;
    let gap = cr.rays_per_s.median - b.rays_per_s.median;
    let margin = spread(cr).max(spread(b));
    if gap > margin {
        (Sign::Win, qualifiers(cr, b), String::new())
    } else if -gap > margin {
        (Sign::Loss, vec![], format!("baseline mode {} is faster at {} threads", b.mode, cr.threads))
    } else {
        (Sign::NoClaim, vec![], format!("medians within the larger min-max spread at {} threads", cr.threads))
    }
}

/// Pre-registered verdicts of `files` (one session) against `baseline`: one per challenger cell
/// (case, workload, mode), in that order. Same machine and UTC day are enforced as for
/// [`render`]. Polyhedral rows and L3 rows ([`is_l3`]) are dropped before anything is read, so they cannot change a verdict.
pub fn verdicts(files: &[ResultFile], baseline: &str) -> Result<Vec<CellVerdict>> {
    check_same_machine(files)?;
    check_geometry(files)?;
    let mut cells: BTreeMap<(&str, &str, &str), Vec<Pair>> = BTreeMap::new();
    let mut base: BTreeMap<&str, Vec<Pair>> = BTreeMap::new();
    for f in files {
        for r in f.rows.iter().filter(|r| !is_polyhedral(r) && !is_l3(r)) {
            if r.tracker == CHALLENGER {
                cells.entry((f.case.as_str(), r.workload.as_str(), r.mode.as_str())).or_default().push((f, r));
            } else if r.tracker == baseline {
                base.entry(f.case.as_str()).or_default().push((f, r));
            }
        }
    }
    let none = Vec::new();
    let mut out = Vec::new();
    for ((case, workload, mode), mut rows) in cells {
        rows.sort_by_key(|(_, r)| r.threads);
        rows.dedup_by_key(|(_, r)| r.threads);
        let bl = base.get(case).unwrap_or(&none);
        let (lo, hi) = (rows[0], rows[rows.len() - 1]);
        let cell = |sign, qualifiers, detail: String| CellVerdict { case: case.into(), workload: workload.into(), mode: mode.into(), baseline: baseline.into(), sign, qualifiers, detail };
        if lo.1.threads != 1 || hi.1.threads == 1 {
            out.push(cell(Sign::Incomplete, vec![], "needs rows at 1 thread and at the full core count".into()));
            continue;
        }
        let pts = [point(lo, bl), point(hi, bl)];
        let v = if let Some(p) = pts.iter().find(|p| p.0 == Sign::NotComparable) {
            cell(Sign::NotComparable, vec![], p.2.clone())
        } else if let Some(p) = pts.iter().find(|p| p.0 == Sign::Incomplete) {
            cell(Sign::Incomplete, vec![], p.2.clone())
        } else if pts.iter().all(|p| p.0 == Sign::Win) {
            let mut q: Vec<String> = Vec::new();
            for t in pts.iter().flat_map(|p| p.1.iter()) {
                if !q.contains(t) {
                    q.push(t.clone());
                }
            }
            cell(Sign::Win, q, String::new())
        } else if pts.iter().all(|p| p.0 == Sign::Loss) {
            cell(Sign::Loss, vec![], pts[0].2.clone())
        } else {
            let d = pts.iter().find(|p| p.0 == Sign::NoClaim).map_or_else(|| "win at one thread count and loss at the other".to_string(), |p| p.2.clone());
            cell(Sign::NoClaim, vec![], d)
        };
        out.push(v);
    }
    Ok(out)
}

/// The result statement: k of n with every cell named, no headline number.
pub fn statement(v: &[CellVerdict], baseline: &str) -> String {
    let wins: Vec<String> = v.iter().filter(|c| c.sign == Sign::Win).map(CellVerdict::name).collect();
    let rest: Vec<String> = v.iter().filter(|c| c.sign != Sign::Win).map(|c| format!("{} ({})", c.name(), c.sign.label())).collect();
    let mut s = format!("sm-rt beats {baseline} in {} of {} cells", wins.len(), v.len());
    if !wins.is_empty() {
        let _ = write!(s, " [{}]", wins.join(", "));
    }
    if rest.is_empty() {
        s.push('.');
    } else {
        let _ = write!(s, " and does not beat it in the rest: {}.", rest.join(", "));
    }
    s
}

fn baselines_of(files: &[ResultFile]) -> Vec<&'static str> {
    let present: BTreeSet<&str> = files.iter().flat_map(|f| f.rows.iter()).filter(|r| !is_polyhedral(r) && !is_l3(r)).map(|r| r.tracker.as_str()).collect();
    if !present.contains(CHALLENGER) {
        return vec![];
    }
    KNOWN_TRACKERS.iter().copied().filter(|t| *t != CHALLENGER && present.contains(t)).collect()
}

fn verdict_section(files: &[ResultFile], smoke: bool) -> String {
    let mut out = String::new();
    for b in baselines_of(files) {
        let Ok(v) = verdicts(files, b) else { continue };
        let _ = writeln!(out, "## Verdict against {b} (pre-registered, VERDICT.md)\n");
        if smoke {
            let _ = writeln!(out, "Smoke result: these signs check the harness and are not a published verdict.\n");
        }
        let _ = writeln!(out, "| cell | verdict |\n|---|---|");
        for c in &v {
            let _ = writeln!(out, "| {} | {} |", c.name(), c.text());
        }
        let _ = writeln!(out, "\n{}\n", statement(&v, b));
    }
    out
}

/// Aggregate three single-session result sets. Rule b stays intact: each session passes
/// [`check_same_machine`] on its own, and no number from any session is printed. A cell is a
/// published win only with the same sign in all three. Sessions must be bare-metal Linux x86_64
/// (not smoke), from one machine, on three different UTC days.
pub fn render_sessions(sessions: &[Vec<ResultFile>]) -> Result<String> {
    if sessions.len() != 3 {
        return Err(BenchError::Fairness(format!("--sessions needs exactly 3 sessions, got {}", sessions.len())));
    }
    let mut days = BTreeSet::new();
    for s in sessions {
        check_same_machine(s)?;
        check_geometry(s)?;
        let e = &s[0].env;
        if e.smoke() {
            return Err(BenchError::Fairness(format!("refusing a published verdict from a smoke session ({})", e.smoke_reasons().join("; "))));
        }
        if !days.insert(e.day().to_string()) {
            return Err(BenchError::Fairness(format!("sessions must be on three different UTC days, {} repeats", e.day())));
        }
    }
    let first = &sessions[0][0].env;
    if sessions.iter().any(|s| s[0].env.machine_id != first.machine_id || s[0].env.cpu_model != first.cpu_model) {
        return Err(BenchError::Fairness("sessions come from different machines".into()));
    }
    let mut out = String::from("# smrt-bench three-session verdict\n\n");
    let _ = writeln!(out, "Sessions (UTC days): {}. Signs only: numbers of different days are never shown together.\n", days.iter().cloned().collect::<Vec<_>>().join(", "));
    let mut bl: BTreeSet<&str> = BTreeSet::new();
    for s in sessions {
        bl.extend(baselines_of(s));
    }
    for b in bl {
        let per: Vec<Vec<CellVerdict>> = sessions.iter().map(|s| verdicts(s, b)).collect::<Result<_>>()?;
        let mut keys: BTreeSet<(String, String, String)> = BTreeSet::new();
        for v in &per {
            keys.extend(v.iter().map(|c| (c.case.clone(), c.workload.clone(), c.mode.clone())));
        }
        let _ = writeln!(out, "## Against {b}\n\n| cell | session 1 | session 2 | session 3 | result |\n|---|---|---|---|---|");
        let mut combined = Vec::new();
        for k in keys {
            let found: Vec<Option<&CellVerdict>> = per.iter().map(|v| v.iter().find(|c| c.case == k.0 && c.workload == k.1 && c.mode == k.2)).collect();
            let signs: Vec<Option<Sign>> = found.iter().map(|c| c.map(|c| c.sign)).collect();
            let sign = if signs.iter().all(|s| *s == Some(Sign::Win)) {
                Sign::Win
            } else if signs.iter().all(|s| *s == Some(Sign::Loss)) {
                Sign::Loss
            } else {
                Sign::NoClaim
            };
            let lab = |s: &Option<Sign>| s.map_or("missing", Sign::label);
            let _ = writeln!(out, "| {}/{}/{} | {} | {} | {} | {} |", k.0, k.1, k.2, lab(&signs[0]), lab(&signs[1]), lab(&signs[2]), sign.label());
            combined.push(CellVerdict { case: k.0, workload: k.1, mode: k.2, baseline: b.into(), sign, qualifiers: vec![], detail: String::new() });
        }
        let _ = writeln!(out, "\n{}\n", statement(&combined, b));
    }
    Ok(out)
}
