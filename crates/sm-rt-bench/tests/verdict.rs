//! The pre-registered verdict on fabricated rows (no timing, no wall clock): golden verdict
//! text, the three-session aggregator, the polyhedral exclusion and the bare-metal banner.

use sm_rt_bench::report::{self, Sign, is_polyhedral, render, render_sessions, verdicts};
use sm_rt_bench::result::{AgreementSummary, Env, ResultFile, Row, Stat};

fn env(day: &str, os: &str, arch: &str, virt: &str, governor: &str) -> Env {
    Env { cpu_model: "CPU A".into(), governor: governor.into(), ncpu: 16, os: os.into(), arch: arch.into(), date_utc: format!("{day}T10:00:00Z"), machine_id: "m1".into(), virt: virt.into(), virt_source: "systemd-detect-virt".into() }
}

fn bare(day: &str) -> Env {
    env(day, "linux", "x86_64", "none", "performance")
}

#[derive(Clone)]
struct R {
    tracker: &'static str,
    mode: &'static str,
    workload: &'static str,
    threads: usize,
    median: f64,
    spread: f64,
    failed: usize,
    build_s: f64,
    structure: u64,
}

fn r(tracker: &'static str, mode: &'static str, workload: &'static str, threads: usize, median: f64, spread: f64) -> R {
    R { tracker, mode, workload, threads, median, spread, failed: 0, build_s: 1.0, structure: 1000 }
}

fn row(x: &R) -> Row {
    Row {
        tracker: x.tracker.into(),
        mode: x.mode.into(),
        workload: x.workload.into(),
        threads: x.threads,
        precision: "f64".into(),
        geometry: match (x.tracker, x.mode) {
            ("smrt", "jump" | "walk") => "tets".into(),
            ("smrt", _) => "exact".into(),
            _ => String::new(),
        },
        rays_per_s: Stat { median: x.median, min: x.median - x.spread / 2.0, max: x.median + x.spread / 2.0, reps: 7 },
        build_s: x.build_s,
        structure_bytes: x.structure,
        agreement: AgreementSummary { checked: 100, failed: x.failed, grazing: 0 },
        ..Row::default()
    }
}

fn file(case: &str, e: &Env, rows: &[R]) -> ResultFile {
    let mut f = ResultFile::new(case, "tiny", e.clone());
    f.rows = rows.iter().map(row).collect();
    f.seal().unwrap();
    f
}

/// One session whose cells exercise every verdict path against `dagmc-dd`.
fn session(e: &Env, flip: bool) -> Vec<ResultFile> {
    let mut a = vec![
        // win at 1 and 16 threads (gap 50 > spread 20)
        r("smrt", "analytic", "w-fire", 1, 200.0, 20.0),
        r("smrt", "analytic", "w-fire", 16, 3000.0, 20.0),
        r("dagmc-dd", "surface", "w-fire", 1, 100.0, 10.0),
        r("dagmc-dd", "surface", "w-fire", 16, 1500.0, 10.0),
        // spread overlap: gap 5 < spread 30
        r("smrt", "facets", "w-fire", 1, 105.0, 30.0),
        r("smrt", "facets", "w-fire", 16, 1505.0, 30.0),
        // a loss (the claim-9 shape): walk-only is slower
        r("smrt", "slow", "w-fire", 1, 20.0, 2.0),
        r("smrt", "slow", "w-fire", 16, 300.0, 2.0),
    ];
    if flip {
        // The win flips to a loss on another day.
        a[0].median = 50.0;
        a[1].median = 500.0;
    }
    // memory exception: a win with 3x the structure bytes and build time
    let mut mem = r("smrt", "hierarchy", "w-fire", 1, 400.0, 10.0);
    mem.structure = 3000;
    mem.build_s = 3.0;
    let mut mem16 = mem.clone();
    mem16.threads = 16;
    mem16.median = 6000.0;
    a.extend([mem, mem16]);
    // disagreement: the challenger failed its check in this mode
    let mut bad = r("smrt", "delta", "w-fire", 1, 900.0, 1.0);
    bad.failed = 2;
    let mut bad16 = bad.clone();
    bad16.threads = 16;
    a.extend([bad, bad16]);
    // missing thread count
    a.push(r("smrt", "short", "w-fire", 1, 900.0, 1.0));
    vec![file("P1", e, &a)]
}

#[test]
fn golden_verdict_on_fabricated_rows() {
    let v = verdicts(&session(&bare("2026-11-10"), false), "dagmc-dd").unwrap();
    let got: Vec<(String, String)> = v.iter().map(|c| (c.name(), c.text())).collect();
    let want: Vec<(&str, &str)> = vec![
        ("P1/w-fire/analytic", "win"),
        ("P1/w-fire/delta", "n/c (challenger agreement failed or not checked)"),
        ("P1/w-fire/facets", "no claim (medians within the larger min-max spread at 1 threads)"),
        ("P1/w-fire/hierarchy", "win (qualified: build 3.000 s vs 1.000 s, more than 2x; structure 3000 B vs 1000 B, more than 2x)"),
        ("P1/w-fire/short", "incomplete (needs rows at 1 thread and at the full core count)"),
        ("P1/w-fire/slow", "loss (baseline mode surface is faster at 1 threads)"),
    ];
    let want: Vec<(String, String)> = want.into_iter().map(|(a, b)| (a.to_string(), b.to_string())).collect();
    assert_eq!(got, want);
    assert_eq!(
        report::statement(&v, "dagmc-dd"),
        "sm-rt beats dagmc-dd in 2 of 6 cells [P1/w-fire/analytic, P1/w-fire/hierarchy] and does not beat it in the rest: P1/w-fire/delta (n/c), P1/w-fire/facets (no claim), P1/w-fire/short (incomplete), P1/w-fire/slow (loss)."
    );
    // No number of a tracker ratio appears in the statement.
    assert!(!report::statement(&v, "dagmc-dd").contains('x'));
}

#[test]
fn a_win_needs_both_thread_counts_and_a_baseline_that_agrees() {
    let e = bare("2026-11-10");
    let mut rows = vec![
        r("smrt", "analytic", "w-fire", 1, 200.0, 20.0),
        r("smrt", "analytic", "w-fire", 16, 1000.0, 20.0),
        r("dagmc-dd", "surface", "w-fire", 1, 100.0, 10.0),
        r("dagmc-dd", "surface", "w-fire", 16, 1500.0, 10.0),
    ];
    let v = verdicts(&[file("P1", &e, &rows)], "dagmc-dd").unwrap();
    assert_eq!(v[0].sign, Sign::NoClaim, "win at 1 thread, loss at 16: no claim");
    // A baseline mode that disagrees is not the best-of reference; the agreeing one is used.
    rows[3].median = 500.0;
    rows.push(R { failed: 1, mode: "bad", ..r("dagmc-dd", "bad", "w-fire", 16, 9000.0, 1.0) });
    rows.push(R { failed: 1, mode: "bad", ..r("dagmc-dd", "bad", "w-fire", 1, 9000.0, 1.0) });
    let v = verdicts(&[file("P1", &e, &rows)], "dagmc-dd").unwrap();
    assert_eq!(v[0].sign, Sign::Win);
    // Only disagreeing baseline rows: n/c.
    let only_bad = vec![rows[0].clone(), rows[1].clone(), rows[4].clone(), rows[5].clone()];
    assert_eq!(verdicts(&[file("P1", &e, &only_bad)], "dagmc-dd").unwrap()[0].sign, Sign::NotComparable);
}

#[test]
fn mixed_days_are_refused_by_render_and_verdicts() {
    let a = session(&bare("2026-11-10"), false).remove(0);
    let mut b = session(&bare("2026-11-11"), false).remove(0);
    b.case = "P2".into();
    assert!(render(&[a.clone(), b.clone()]).unwrap_err().to_string().contains("day"));
    assert!(verdicts(&[a, b], "dagmc-dd").is_err());
}

#[test]
fn rendered_report_carries_the_verdict_and_the_smoke_banner() {
    let md = render(&session(&bare("2026-11-10"), false)).unwrap();
    assert!(md.contains("## Verdict against dagmc-dd"), "{md}");
    assert!(md.contains("| P1/w-fire/analytic | win |"), "{md}");
    assert!(!md.contains("SMOKE"));
    let smoke = render(&session(&env("2026-11-10", "macos", "aarch64", "n/a", "n/a"), false)).unwrap();
    assert!(smoke.contains("SMOKE RESULT") && smoke.contains("macos-aarch64 is not Linux x86_64"), "{smoke}");
    assert!(smoke.contains("not a published verdict"));
}

#[test]
fn banner_follows_virtualisation_and_governor_in_the_report() {
    for (e, banner) in [
        (env("2026-11-10", "linux", "x86_64", "none", "performance"), false),
        (env("2026-11-10", "linux", "x86_64", "kvm", "performance"), true),
        (env("2026-11-10", "linux", "x86_64", "none", "powersave"), true),
        (env("2026-11-10", "macos", "aarch64", "n/a", "n/a"), true),
    ] {
        let md = render(&session(&e, false)).unwrap();
        assert_eq!(md.contains("SMOKE RESULT"), banner, "{e:?}");
    }
}

#[test]
fn three_sessions_one_sign_flip_is_no_claim() {
    let days = ["2026-11-10", "2026-11-12", "2026-11-15"];
    let all: Vec<Vec<ResultFile>> = days.iter().map(|d| session(&bare(d), false)).collect();
    let md = render_sessions(&all).unwrap();
    assert!(md.contains("| P1/w-fire/analytic | win | win | win | win |"), "{md}");
    assert!(md.contains("| P1/w-fire/slow | loss | loss | loss | loss |"), "{md}");
    assert!(md.contains("sm-rt beats dagmc-dd in 2 of 6 cells"), "{md}");
    // Signs only: no number from a session appears.
    assert!(!md.contains("e2") && !md.contains("200"));

    let flipped = vec![session(&bare(days[0]), false), session(&bare(days[1]), true), session(&bare(days[2]), false)];
    let md = render_sessions(&flipped).unwrap();
    assert!(md.contains("| P1/w-fire/analytic | win | loss | win | no claim |"), "{md}");
    assert!(md.contains("sm-rt beats dagmc-dd in 1 of 6 cells [P1/w-fire/hierarchy]"), "{md}");
}

#[test]
fn sessions_must_be_three_distinct_bare_metal_days() {
    let s = |d: &str| session(&bare(d), false);
    assert!(render_sessions(&[s("2026-11-10"), s("2026-11-12")]).is_err());
    let err = render_sessions(&[s("2026-11-10"), s("2026-11-10"), s("2026-11-12")]).unwrap_err().to_string();
    assert!(err.contains("different UTC days"), "{err}");
    let vm = session(&env("2026-11-12", "linux", "x86_64", "kvm", "performance"), false);
    let err = render_sessions(&[s("2026-11-10"), vm, s("2026-11-15")]).unwrap_err().to_string();
    assert!(err.contains("smoke session"), "{err}");
}

#[test]
fn a_polyhedral_row_cannot_change_a_verdict() {
    let e = bare("2026-11-10");
    let base = session(&e, false);
    let want = verdicts(&base, "dagmc-dd").unwrap();
    // Polyhedral rows, enormously fast for sm-rt and slow for the baseline, in an existing cell's
    // case, in a cell of their own, and in both name forms (mode, workload).
    let extra = vec![
        r("smrt", "poly-walk", "w-fire", 1, 1e12, 1.0),
        r("smrt", "poly-walk", "w-fire", 16, 1e12, 1.0),
        r("smrt", "walk", "poly-w-track", 1, 1e12, 1.0),
        r("smrt", "walk", "poly-w-track", 16, 1e12, 1.0),
        r("dagmc-dd", "poly-surface", "w-fire", 1, 1.0, 0.1),
        r("dagmc-dd", "poly-surface", "w-fire", 16, 1.0, 0.1),
    ];
    assert!(extra.iter().all(|x| is_polyhedral(&row(x))));
    let mut with = base.clone();
    with.push(file("P1", &e, &extra));
    assert_eq!(verdicts(&with, "dagmc-dd").unwrap(), want, "a polyhedral row must not move any verdict");
    // Failing-agreement polyhedral rows in the SAME files as the judged rows, on both the
    // challenger and the baseline side, in both name forms, must not move a verdict either.
    let mut same = base.clone();
    for f in &mut same {
        let mut bad = vec![
            r("smrt", "walk", "poly-w-track", 1, 5.0, 1.0),
            r("smrt", "poly-walk", "w-fire", 1, 5.0, 1.0),
            r("smrt", "analytic", "poly-w-fire", 1, 5.0, 1.0),
            r("dagmc-dd", "surface", "poly-w-fire", 1, 5.0, 1.0),
            r("dagmc-dd", "poly-surface", "w-fire", 1, 5.0, 1.0),
        ];
        for x in &mut bad {
            x.failed = 3;
        }
        f.rows.extend(bad.iter().map(row));
        f.seal().unwrap();
    }
    assert_eq!(verdicts(&same, "dagmc-dd").unwrap(), want, "a failing polyhedral row must not make a cell n/c");
    // The main table (above the polyhedral section) shows the same disagreements as without them.
    let main = |md: &str| md.split("Polyhedral rows (").next().unwrap().matches("n/c (disagrees)").count();
    assert_eq!(main(&render(&same).unwrap()), main(&render(&base).unwrap()));
    // A polyhedral row also cannot be the baseline (a slow one cannot make a loss a win, a fast one
    // cannot make a win a loss).
    let fast_base = vec![r("dagmc-dd", "poly-surface", "w-fire", 1, 1e12, 1.0), r("dagmc-dd", "poly-surface", "w-fire", 16, 1e12, 1.0)];
    let mut with = base.clone();
    with.push(file("P1", &e, &fast_base));
    assert_eq!(verdicts(&with, "dagmc-dd").unwrap(), want);
    // The rows are rendered, in their own labelled table, and the verdict section never lists them.
    let md = render(&with).unwrap();
    assert!(md.contains("Polyhedral rows (different mesh, same geometry; never read by the verdict"), "{md}");
    assert!(md.contains("poly-surface"));
    let verdict_part = md.split("## Verdict against").nth(1).unwrap();
    assert!(!verdict_part.contains("poly"), "{verdict_part}");
    // Three-session aggregate too.
    let days = ["2026-11-10", "2026-11-12", "2026-11-15"];
    let plain: Vec<Vec<ResultFile>> = days.iter().map(|d| session(&bare(d), false)).collect();
    let mut poly = plain.clone();
    for (s, d) in poly.iter_mut().zip(days) {
        s.push(file("P1", &bare(d), &extra));
    }
    assert_eq!(render_sessions(&poly).unwrap(), render_sessions(&plain).unwrap());
}

/// `report --sessions A,B,C` reads three results directories and prints the aggregate; a wrong
/// count and an empty directory are usage errors. The files go to a directory that is removed.
#[test]
fn report_sessions_is_wired_to_the_aggregator() {
    use sm_rt_bench::cli::main_with;
    use sm_rt_bench::result::write_result;
    struct Rm(std::path::PathBuf);
    impl Drop for Rm {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }
    let root = std::env::temp_dir().join(format!("smrt-bench-sessions-{}", std::process::id()));
    let _guard = Rm(root.clone());
    let days = ["2026-11-10", "2026-11-12", "2026-11-15"];
    let mut dirs = Vec::new();
    for (i, d) in days.iter().enumerate() {
        let dir = root.join(format!("s{i}"));
        for f in session(&bare(d), false) {
            write_result(&dir, &f).unwrap();
        }
        dirs.push(dir.display().to_string());
    }
    let out = root.join("sessions.md");
    let a = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
    let code = main_with(&a(&["report", "--sessions", &dirs.join(","), "--out", &out.display().to_string()])).unwrap();
    assert_eq!(code, 0);
    let md = std::fs::read_to_string(&out).unwrap();
    assert!(md.contains("three-session verdict") && md.contains("| P1/w-fire/analytic | win | win | win | win |"), "{md}");
    let err = main_with(&a(&["report", "--sessions", &dirs[..2].join(",")])).unwrap_err().to_string();
    assert!(err.contains("exactly 3 sessions"), "{err}");
    std::fs::create_dir_all(root.join("empty")).unwrap();
    let err = main_with(&a(&["report", "--sessions", &format!("{},{},{}", dirs[0], dirs[1], root.join("empty").display())])).unwrap_err().to_string();
    assert!(err.contains("holds no result files"), "{err}");
}

/// Two rows of one cell at the same thread count: the first in file order is the cell's row (the
/// rule written in VERDICT.md); a later row with a different rate cannot change the sign.
#[test]
fn a_duplicate_row_at_one_thread_count_is_ignored_after_the_first() {
    let e = bare("2026-11-10");
    let base = [r("smrt", "analytic", "w-fire", 1, 200.0, 20.0), r("smrt", "analytic", "w-fire", 16, 3000.0, 20.0), r("dagmc-dd", "surface", "w-fire", 1, 100.0, 10.0), r("dagmc-dd", "surface", "w-fire", 16, 1000.0, 10.0)];
    let plain = file("P1", &e, &base);
    let mut dup = base.to_vec();
    dup.insert(1, r("smrt", "analytic", "w-fire", 1, 1.0, 20.0)); // would be a loss if it counted
    let doubled = file("P1", &e, &dup);
    let sign = |f: &ResultFile| verdicts(std::slice::from_ref(f), "dagmc-dd").unwrap()[0].sign;
    assert_eq!(sign(&plain), Sign::Win);
    assert_eq!(sign(&doubled), Sign::Win);
}

/// The jump group's W-fire rows (`walk`, `jump`) run on stair-stepped tets, the baselines on the facets:
/// L3 rows never reach a verdict, as challenger rows, and they cannot make another cell `n/c`.
#[test]
fn l3_fire_rows_are_outside_the_verdict() {
    use sm_rt_bench::report::is_l3;
    let e = bare("2026-11-10");
    let base = session(&e, false);
    let want = verdicts(&base, "dagmc-dd").unwrap();
    let mut bad = r("smrt", "jump", "w-fire", 1, 1e12, 1.0);
    bad.failed = 3;
    let extra = vec![bad, r("smrt", "jump", "w-fire", 16, 1e12, 1.0), r("smrt", "walk", "w-fire-locate", 1, 1e12, 1.0), r("smrt", "walk", "w-fire-locate", 16, 1e12, 1.0)];
    assert!(extra.iter().all(|x| is_l3(&row(x))));
    assert!(is_l3(&row(&r("smrt", "walk", "w-flight", 1, 1.0, 1.0))), "the mesh_flight walk row is on the tets: L3");
    let mut plain = row(&r("smrt", "jump", "w-fire", 1, 1.0, 1.0));
    plain.geometry = "facets".into();
    assert!(!is_l3(&plain), "L3 follows the stamped geometry, not the mode name");
    let mut with = base.clone();
    with.push(file("P1", &e, &extra));
    assert_eq!(verdicts(&with, "dagmc-dd").unwrap(), want);
    assert!(render(&with).unwrap().contains("L3 rows"));
}

/// A W-walk row on `shared-tets` (the mesh the baselines hold) is not L3: it stays in the verdict.
#[test]
fn a_shared_tets_walk_row_is_not_l3() {
    use sm_rt_bench::report::is_l3;
    let mut shared = row(&r("smrt", "walk", "w-flight", 1, 1.0, 1.0));
    shared.geometry = "shared-tets".into();
    assert!(!is_l3(&shared));
}

/// A challenger row with an empty geometry (legacy file, custom adapter) fails closed: render,
/// verdicts and `report` on a results directory all refuse it, so it never enters a k-of-n.
#[test]
fn an_empty_geometry_challenger_row_is_refused_by_report() {
    use sm_rt_bench::cli::main_with;
    use sm_rt_bench::result::write_result;
    let e = bare("2026-11-10");
    let mut files = session(&e, false);
    files[0].rows.iter_mut().find(|x| x.tracker == "smrt").unwrap().geometry.clear();
    files[0].seal().unwrap();
    for err in [render(&files).unwrap_err().to_string(), verdicts(&files, "dagmc-dd").unwrap_err().to_string()] {
        assert!(err.contains("empty `geometry`"), "{err}");
    }
    let dir = std::env::temp_dir().join(format!("smrt-bench-geom-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    for f in &files {
        write_result(&dir, f).unwrap();
    }
    let out = dir.join("t.md");
    let a = |v: &[&str]| v.iter().map(|s| s.to_string()).collect::<Vec<_>>();
    let res = main_with(&a(&["report", "--results", &dir.display().to_string(), "--out", &out.display().to_string()]));
    let _ = std::fs::remove_dir_all(&dir);
    let err = res.unwrap_err().to_string();
    assert!(err.contains("empty `geometry`"), "{err}");
}
