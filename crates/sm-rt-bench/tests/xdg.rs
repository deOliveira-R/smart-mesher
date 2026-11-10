//! W4-RT-XDG-DRIVER acceptance: the XDG container's rows and dumps judged against the sm-rt facets
//! twin and the analytic truth, with work counts, hashes and analytic values only.
//!
//! The judges run on a [`Run`] (rows plus dumps). Two entry points feed them: the container through
//! `run_competitor_with` (`#[ignore]`, needs Docker and a native `smrt-bench-xdg` image, which
//! only an x86_64 host can have: XDG includes `<immintrin.h>` unconditionally), and the saved
//! stdout of `run_case.sh` (`SMRT_XDG_OUT_DIR=DIR` with `P1.out` and `C1.out` of the reduced
//! scale), e.g. from an emulated amd64 image on an Apple-silicon host. Without either they skip.

use sm_rt_bench::adapter::{Adapter, Output, WorkInput, Workload};
use sm_rt_bench::cases::{Case, CaseData, Scale, build};
use sm_rt_bench::compare::{compare, compare_fires};
use sm_rt_bench::docker::{Competitor, CompetitorOpts, parse_driver_output, run_competitor_with};
use sm_rt_bench::result::Row;
use sm_rt_bench::runner::compare_tallies_with;
use sm_rt_bench::truth::Track;
use std::collections::BTreeMap;

struct Run {
    rows: Vec<Row>,
    dumps: BTreeMap<String, Output>,
    tracks: Option<Vec<Track>>,
}

fn xdg_ready() -> bool {
    let ok = |args: &[&str]| std::process::Command::new("docker").args(args).output().is_ok_and(|o| o.status.success());
    ok(&["version", "--format", "{{.Server.Version}}"]) && ok(&["image", "inspect", "smrt-bench-xdg"])
}

/// Run the container on `data` (written to a scratch directory that is removed afterwards).
fn container_run(data: &CaseData, workloads: &[&str], threads: &[usize], histories: usize) -> Run {
    let dir = std::env::temp_dir().join(format!("smrt-bench-xdg-{}-{}", std::process::id(), data.case));
    data.write(&dir).unwrap();
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let opts = CompetitorOpts { workloads: workloads.iter().map(|w| (*w).to_string()).collect(), histories, seed: 0x5eed, regions: data.manifest.regions.clone() };
    let res = run_competitor_with(root, "xdg", &dir, &data.case, threads, 1, 0, &opts);
    let _ = std::fs::remove_dir_all(&dir);
    let Competitor::Rows { rows, dumps, tracks, .. } = res else { panic!("competitor did not run: {res:?}") };
    Run { rows, dumps, tracks }
}

fn saved_run(case: &str) -> Option<Run> {
    let dir = std::env::var("SMRT_XDG_OUT_DIR").ok()?;
    let text = std::fs::read_to_string(std::path::Path::new(&dir).join(format!("{case}.out"))).unwrap();
    let (rows, tracks, dumps) = parse_driver_output("xdg", &text).unwrap();
    Some(Run { rows, dumps, tracks })
}

/// Acceptance 5 and 7: precision `mixed`, nothing lost, every row stable, and one hash per workload
/// across thread counts.
fn judge_rows(run: &Run) {
    assert!(!run.rows.is_empty());
    for r in &run.rows {
        assert_eq!(r.precision, "mixed", "{}", r.workload);
        assert_eq!((r.tracker.as_str(), r.mode.as_str()), ("xdg", "embree"));
        assert_eq!(r.lost, 0, "{} lost", r.workload);
        assert_eq!(r.determinism.stable, Some(true), "{}", r.workload);
    }
    for w in ["w-track", "w-fire", "w-fire-locate", "w-flight", "w-walk"] {
        let hashes: Vec<&String> = run.rows.iter().filter(|r| r.workload == w).map(|r| &r.determinism.hash).collect();
        assert!(hashes.windows(2).all(|p| p[0] == p[1]), "{w}: hash differs across thread counts");
    }
}

/// Acceptance 1: hinted and located fire equal the sm-rt facets fire on every ray (grazing rays
/// are counted, not failed).
fn judge_fire(data: &CaseData, run: &Run) {
    let w = WorkInput { rays: &data.rays, regions: &data.manifest.regions, threads: 1, histories: 0, seed: 1 };
    let mut a = sm_rt_bench::smrt::facets_adapter(data).unwrap();
    a.prepare(Workload::Fire, &w).unwrap();
    let reference = a.run(Workload::Fire, &w).unwrap();
    for wl in ["w-fire", "w-fire-locate"] {
        let (s, bad) = compare_fires(&run.dumps[wl].fires, &reference.fires, data.manifest.diag, data.manifest.chord_tol).unwrap();
        assert_eq!((s.checked, s.failed, bad.len()), (data.rays.len(), 0, 0), "{wl}: {} grazing", s.grazing);
    }
}

/// W-track against the facets twin: equal region sequences and chords within the tolerance.
fn judge_track(data: &CaseData, run: &Run) {
    let w = WorkInput { rays: &data.rays, regions: &data.manifest.regions, threads: 1, histories: 0, seed: 1 };
    let reference = sm_rt_bench::smrt::facets_adapter(data).unwrap().run(Workload::Track, &w).unwrap();
    let tracks = run.tracks.as_ref().expect("a track dump");
    let rep = compare(tracks, &reference.tracks, data.manifest.diag, data.manifest.chord_tol).unwrap();
    assert_eq!((rep.checked, rep.failed), (data.rays.len(), 0), "{:?}", rep.listed.first());
}

/// Acceptance 2: flight tallies agree with the sm-rt surface flight at 4 sigma, over every slot
/// (two per region plus leaked and absorbed).
fn judge_flight(data: &CaseData, run: &Run, histories: usize) {
    let w = WorkInput { rays: &data.rays, regions: &data.manifest.regions, threads: 1, histories, seed: 0x5eed };
    let reference = sm_rt_bench::smrt::facets_adapter(data).unwrap().run(Workload::Flight, &w).unwrap();
    let s = compare_tallies_with(&run.dumps["w-flight"], &reference, true);
    let regions = data.manifest.regions.iter().filter(|r| r.id >= 0).count();
    assert_eq!((s.checked, s.failed), (2 * regions + 2, 0));
}

/// Acceptance 3: on a case with an analytic scene, the fire distance from the hinted region is the
/// analytic chord within the facet `sag` bound of `bound.rs`, and the region entered is right.
fn judge_chord(data: &CaseData, run: &Run) {
    let scene = data.scene.as_ref().expect("the case has a truth scene");
    let fires = &run.dumps["w-fire"].fires;
    assert_eq!(fires.len(), data.rays.len());
    let (mut judged, mut grazing) = (0usize, 0usize);
    for (i, (dist, to)) in fires.iter().enumerate() {
        let (o, d) = (data.rays.origin[i], data.rays.dir[i]);
        let truth = scene.track(o, d);
        let bound = sm_rt_bench::bound::ray_bound(scene, o, d, data.manifest.sag);
        if bound.grazing {
            grazing += 1;
            continue;
        }
        judged += 1;
        assert!((dist - truth.chords[0]).abs() <= bound.chord + 1e-9 * data.manifest.diag, "ray {i}: fire {dist} vs analytic {} (bound {})", truth.chords[0], bound.chord);
        if truth.chords[0] > 5e-9 * data.manifest.diag {
            assert_eq!(*to, truth.regions.get(1).copied().unwrap_or(-1), "ray {i}: region entered");
        }
    }
    println!("{judged} rays judged against the analytic chord, {grazing} grazing");
    assert!(judged * 10 >= data.rays.len() * 9, "too few rays judged");
}

#[test]
fn a_missing_xdg_image_is_partial_with_the_command_not_a_number() {
    if xdg_ready() {
        println!("skipped: a native smrt-bench-xdg image exists");
        return;
    }
    let data = build(Case::C1, &Scale::tiny()).unwrap();
    let dir = std::env::temp_dir().join(format!("smrt-bench-xdg-partial-{}", std::process::id()));
    data.write(&dir).unwrap();
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let res = run_competitor_with(root, "xdg", &dir, &data.case, &[1], 1, 0, &CompetitorOpts::default());
    let _ = std::fs::remove_dir_all(&dir);
    match res {
        Competitor::Partial(why) => assert!(why.contains("docker"), "the command is named: {why}"),
        other => panic!("expected Partial, got {other:?}"),
    }
}

#[test]
fn saved_xdg_p1_output_is_judged_like_the_container() {
    let Some(run) = saved_run("P1") else {
        println!("skipped: SMRT_XDG_OUT_DIR not set");
        return;
    };
    let data = build(Case::P1, &Scale::reduced()).unwrap();
    judge_rows(&run);
    judge_fire(&data, &run);
    judge_track(&data, &run);
    judge_flight(&data, &run, 20_000);
}

#[test]
fn saved_xdg_c1_fire_is_the_analytic_chord() {
    let Some(run) = saved_run("C1") else {
        println!("skipped: SMRT_XDG_OUT_DIR not set");
        return;
    };
    judge_rows(&run);
    judge_chord(&build(Case::C1, &Scale::reduced()).unwrap(), &run);
}

#[test]
#[ignore = "needs Docker and a native smrt-bench-xdg image (x86_64 hosts)"]
fn the_xdg_container_agrees_with_sm_rt_on_p1() {
    if !xdg_ready() {
        println!("skipped: no docker (or no image smrt-bench-xdg)");
        return;
    }
    let data = build(Case::P1, &Scale::reduced()).unwrap();
    let run = container_run(&data, &["track", "fire", "fire-locate", "flight"], &[1, 2], 20_000);
    judge_rows(&run);
    judge_fire(&data, &run);
    judge_track(&data, &run);
    judge_flight(&data, &run, 20_000);
}

#[test]
#[ignore = "needs Docker and a native smrt-bench-xdg image (x86_64 hosts)"]
fn the_xdg_container_fire_is_the_analytic_chord_on_c1() {
    if !xdg_ready() {
        println!("skipped: no docker (or no image smrt-bench-xdg)");
        return;
    }
    let data = build(Case::C1, &Scale::reduced()).unwrap();
    let run = container_run(&data, &["fire"], &[1], 0);
    judge_rows(&run);
    judge_chord(&data, &run);
}
