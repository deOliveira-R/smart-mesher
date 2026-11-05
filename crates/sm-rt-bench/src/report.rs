//! `report`: renders result files into Markdown tables, enforcing the fairness gates as code.
//!
//! * Refuses to render when the files come from different machines or UTC days (fairness rule b).
//! * A tracker whose agreement check failed in a case shows `n/c (disagrees)` in every rate cell
//!   of that case (rule g); the failure counts stay visible.
//! * Results that are not from Linux x86_64 are marked as smoke results (ruling Q9).
//! * No ratio between trackers is ever computed: the tables hold measured values only.

use crate::error::{BenchError, Result};
use crate::result::ResultFile;
use std::collections::BTreeSet;
use std::fmt::Write;

/// Trackers a full benchmark covers; absent ones are listed under "not measured".
pub const KNOWN_TRACKERS: [&str; 4] = ["smrt", "dagmc-moab", "dagmc-dd", "xdg"];

/// What the benchmark does not do, whatever the numbers say (fairness rule h).
pub const NOT_MEASURED: [&str; 7] = [
    "GPU rows (XDG GPRT) and the libMesh backend",
    "mesh-tally workloads (MOC/Sn segment lists, segment-callback API)",
    "multi-node scaling and Serpent",
    "reflective faces and instancing in DAGMC (DAGMC has neither; reflective stays an sm-rt-only test)",
    "cell walking on tets is not claimed novel (XDG does it): rows separate walking the same tets from analytic and hierarchy modes",
    "the C-ABI call cost (it stays in sm-rt-capi/bench)",
    "x86_64 and aarch64 results are never merged into one table",
];

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

fn rate_cell(failed: bool, f: &crate::result::Stat) -> String {
    if failed { "n/c (disagrees)".to_string() } else { format!("{:.4e} [{:.3e}, {:.3e}]", f.median, f.min, f.max) }
}

/// Render the report for `files` (one per case and tracker set).
pub fn render(files: &[ResultFile]) -> Result<String> {
    check_same_machine(files)?;
    let env = &files[0].env;
    let mut out = String::new();
    let _ = writeln!(out, "# smrt-bench report\n");
    if env.smoke() {
        let _ = writeln!(out, "> **SMOKE RESULT.** Produced on {}-{}, not on bare-metal Linux x86_64. It is a check that the harness runs, not a published table (ruling Q9).\n", env.os, env.arch);
    }
    let _ = writeln!(out, "Machine: {} ({} logical CPUs, governor {}), OS {}-{}, UTC day {}, machine id {}.\n", env.cpu_model, env.ncpu, env.governor, env.os, env.arch, env.day(), env.machine_id);
    let cases: BTreeSet<&str> = files.iter().map(|f| f.case.as_str()).collect();
    for case in &cases {
        let _ = writeln!(out, "## {case}\n");
        let cfiles: Vec<&ResultFile> = files.iter().filter(|f| f.case == *case).collect();
        let _ = writeln!(out, "| tracker | mode | workload | threads | precision | rays/s median [min, max] | segments/track | ns/segment | steps/history | build s | structure bytes | lost | max chord err | seq. failures per 1e5 | agreement failed/checked (grazing) | determinism |");
        let _ = writeln!(out, "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
        let mut seen: BTreeSet<&str> = BTreeSet::new();
        for f in &cfiles {
            for r in &f.rows {
                seen.insert(r.tracker.as_str());
                let failed = f.rows.iter().any(|x| x.tracker == r.tracker && x.mode == r.mode && x.agreement.failed > 0);
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
        let partial: Vec<&String> = cfiles.iter().flat_map(|f| f.partial.iter()).collect();
        if !partial.is_empty() {
            let _ = writeln!(out, "\nPartial (not run, nothing invented):\n");
            for p in partial {
                let _ = writeln!(out, "- {p}");
            }
        }
        let missing: Vec<&str> = KNOWN_TRACKERS.iter().copied().filter(|t| !seen.contains(t)).collect();
        if !missing.is_empty() {
            let _ = writeln!(out, "\nNot measured in this report for {case}: {}.", missing.join(", "));
        }
        out.push('\n');
    }
    let _ = writeln!(out, "## What was not measured\n");
    for l in NOT_MEASURED {
        let _ = writeln!(out, "- {l}");
    }
    Ok(out)
}
