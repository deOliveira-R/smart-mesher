//! Harness acceptance on the stub: adapter vs analytic truth, agreement checker with a perturbed
//! competitor, fairness gates, determinism, `--model` dry run. Work counters, hashes and analytic
//! values only; no timing is asserted.

use sm_core::{Aabb, Point3};
use sm_rt_api::ResponseKind;
use sm_rt_api::stub::{AnalyticWorld, Shape as StubShape};
use sm_rt_bench::adapter::{Adapter, BuildInfo, Output, TrackerAdapter, WorkInput, Workload, stub_adapter, stub_world};
use sm_rt_bench::cases::{Case, Scale, build, gen_rays};
use sm_rt_bench::compare::{compare, write_disagreements};
use sm_rt_bench::formats::RegionSpec;
use sm_rt_bench::report;
use sm_rt_bench::result::{AgreementSummary, Env, ResultFile, Row, Stat, inside_repo, write_result};
use sm_rt_bench::runner::{RunConfig, TraceEntry, run_case, run_interleaved};
use sm_rt_bench::truth::{Shape, Solid, Track};
use std::sync::{Arc, Mutex};

fn pool(t: usize) -> rayon::ThreadPool {
    rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap()
}

fn input<'a>(d: &'a sm_rt_bench::cases::CaseData, threads: usize, histories: usize) -> WorkInput<'a> {
    WorkInput { rays: &d.rays, regions: &d.manifest.regions, threads, histories, seed: 11 }
}

fn tracks_of(a: &mut TrackerAdapter, d: &sm_rt_bench::cases::CaseData) -> Vec<Track> {
    a.run(Workload::Track, &input(d, 1, 0)).unwrap().tracks
}

#[test]
fn stub_tracks_equal_analytic_truth_with_counters() {
    for case in [Case::C2, Case::H1] {
        let d = build(case, &Scale { rays: 400, ..Scale::reduced() }).unwrap();
        let mut a = stub_adapter(&d).unwrap();
        let out = a.run(Workload::Track, &input(&d, 1, 0)).unwrap();
        let truth: Vec<Track> = (0..d.rays.len()).map(|i| d.scene.as_ref().unwrap().track(d.rays.origin[i], d.rays.dir[i])).collect();
        let rep = compare(&out.tracks, &truth, d.manifest.diag, d.manifest.chord_tol).unwrap();
        assert_eq!(rep.failed, 0, "{}: {:?}", case.name(), rep.listed.first());
        assert!(rep.max_chord_err < 1e-9);
        // Work counters: one segment per chord entry at least; no lost rays.
        assert!(out.counters.segments as usize >= out.tracks.iter().map(|t| t.regions.len()).sum::<usize>());
        assert_eq!(out.counters.lost, 0);
        assert_eq!(out.counters.steps, out.counters.segments);
    }
}

#[test]
fn a_perturbed_competitor_is_detected_located_and_written() {
    let mut d = build(Case::C2, &Scale { rays: 400, ..Scale::reduced() }).unwrap();
    let mut good = stub_adapter(&d).unwrap();
    let reference = tracks_of(&mut good, &d);
    // The competitor's sphere 2 is 1e-3 too large: chords through it differ by more than the tolerance.
    let scene = d.scene.as_mut().unwrap();
    let k = scene.solids.iter().position(|s| s.region == 2).unwrap();
    let Shape::Sphere { c, r } = scene.solids[k].shape else { unreachable!() };
    let mut solids = scene.solids.clone();
    solids[k] = Solid { shape: Shape::Sphere { c, r: r + 1e-3 }, ..solids[k] };
    let mut bad_data = d.clone();
    bad_data.scene = Some(sm_rt_bench::truth::Scene::new(solids));
    let mut bad = TrackerAdapter::new("perturbed", Box::new(stub_world(&bad_data).unwrap()));
    let judged = tracks_of(&mut bad, &d);
    let tol = 1e-6;
    let rep = compare(&judged, &reference, d.manifest.diag, tol).unwrap();
    assert!(rep.failed > 0, "the perturbation must be seen");
    assert!(rep.failed < rep.checked, "rays that miss sphere 2 still agree");
    for dis in &rep.listed {
        let t = &reference[dis.ray];
        assert!(t.regions.contains(&2) || dis.kind == "region_sequence", "disagreement on ray {} does not involve the perturbed sphere", dis.ray);
        assert_eq!(dis.b, t.clone(), "the listed reference sequence is the one of that ray index");
    }
    let dir = std::env::temp_dir().join(format!("smrt-bench-dis-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let p = dir.join("disagreements.jsonl");
    write_disagreements(&p, &rep.listed).unwrap();
    assert_eq!(std::fs::read_to_string(&p).unwrap().lines().count(), rep.listed.len().min(1000));
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn fire_from_a_hint_equals_fire_with_location_and_truth() {
    let d = build(Case::C2, &Scale { rays: 300, ..Scale::reduced() }).unwrap();
    let mut a = stub_adapter(&d).unwrap();
    let inp = input(&d, 1, 0);
    a.prepare(Workload::Fire, &inp).unwrap();
    let hinted = a.run(Workload::Fire, &inp).unwrap();
    let located = a.run(Workload::FireLocate, &inp).unwrap();
    assert_eq!(hinted.fires, located.fires);
    assert_eq!(located.counters.locates as usize, d.rays.len());
    assert_eq!(hinted.counters.locates, 0, "the hinted row does not locate");
    for i in 0..d.rays.len() {
        let t = d.scene.as_ref().unwrap().track(d.rays.origin[i], d.rays.dir[i]);
        assert!((hinted.fires[i].0 - t.chords[0]).abs() < 1e-9, "ray {i}");
        assert_eq!(hinted.fires[i].1, t.regions.get(1).copied().unwrap_or(-1), "ray {i}");
    }
}

#[test]
fn flight_leakage_of_a_pure_absorber_sphere_is_unbiased() {
    // Sigma_t = Sigma_a = 0.3 in a sphere of radius 5 in a vacuum box. Given each ray's exit
    // distance, a history leaks with probability exp(-0.3 chord) independently: the leaked count
    // has mean sum(p) and variance sum(p (1 - p)).
    let sphere = Solid { shape: Shape::Sphere { c: [0.0; 3], r: 5.0 }, region: 0, depth: 1 };
    let rays = gen_rays("absorber", ([-5.0; 3], [5.0; 3]), 20_000, &|p| (sphere.contains(p)).then_some(0)).unwrap();
    let world = AnalyticWorld::simple(vec![StubShape::Sphere { center: Point3::new(0.0, 0.0, 0.0), radius: 5.0 }], Aabb { lo: Point3::new(-10.0, -10.0, -10.0), hi: Point3::new(10.0, 10.0, 10.0) }, ResponseKind::Vacuum).unwrap();
    let mut a = TrackerAdapter::new("stub", Box::new(world));
    let regions = vec![RegionSpec { id: 0, name: "absorber".into(), sigma_t: 0.3, sigma_a: 0.3 }, RegionSpec { id: 1, name: "vacuum".into(), sigma_t: 0.0, sigma_a: 0.0 }];
    let inp = WorkInput { rays: &rays, regions: &regions, threads: 2, histories: rays.len(), seed: 5 };
    let out = a.run(Workload::Flight, &inp).unwrap();
    let (mut mean, mut var) = (0.0, 0.0);
    for i in 0..rays.len() {
        let (_, t_out) = sphere.interval(rays.origin[i], rays.dir[i]).unwrap();
        let p = (-0.3 * t_out).exp();
        mean += p;
        var += p * (1.0 - p);
    }
    let leaked = out.tally[4];
    let absorbed = out.tally[5];
    assert_eq!(leaked + absorbed, rays.len() as f64, "every history ends leaked or absorbed");
    assert!((leaked - mean).abs() <= 4.0 * var.sqrt(), "leaked {leaked} vs {mean} +- {}", var.sqrt());
    assert_eq!(out.counters.lost, 0);
    assert_eq!(out.counters.histories as usize, rays.len());
}

#[test]
fn stub_outputs_are_deterministic_across_threads_and_runs() {
    let d = build(Case::H1, &Scale { rays: 500, ..Scale::tiny() }).unwrap();
    let h = sm_testkit::determinism::check(2, 4, |t| {
        let mut a = stub_adapter(&d).unwrap();
        let tr = a.run(Workload::Track, &input(&d, t, 0)).unwrap().content_hash();
        let fl = a.run(Workload::Flight, &input(&d, t, 700)).unwrap().content_hash();
        format!("{tr} {fl}")
    })
    .unwrap();
    assert_eq!(h.len(), 64 + 1 + 64);
    // Flight tallies differ when the seed differs (the hash is not constant).
    let mut a = stub_adapter(&d).unwrap();
    let mut other = input(&d, 1, 700);
    other.seed = 12;
    assert_ne!(a.run(Workload::Flight, &input(&d, 1, 700)).unwrap().content_hash(), a.run(Workload::Flight, &other).unwrap().content_hash());
    let _ = pool(1);
}

/// Records the order of calls.
struct Rec {
    name: &'static str,
    log: Arc<Mutex<Vec<String>>>,
}

impl Adapter for Rec {
    fn name(&self) -> String {
        self.name.to_string()
    }
    fn precision(&self) -> &'static str {
        "f64"
    }
    fn build_info(&self) -> BuildInfo {
        BuildInfo::default()
    }
    fn prepare(&mut self, _w: Workload, _i: &WorkInput<'_>) -> sm_rt_bench::Result<()> {
        self.log.lock().unwrap().push(format!("prepare {}", self.name));
        Ok(())
    }
    fn run(&mut self, _w: Workload, _i: &WorkInput<'_>) -> sm_rt_bench::Result<Output> {
        self.log.lock().unwrap().push(format!("run {}", self.name));
        Ok(Output::default())
    }
}

#[test]
fn trackers_are_interleaved_abab_per_repetition() {
    let log = Arc::new(Mutex::new(Vec::new()));
    let mut adapters: Vec<Box<dyn Adapter>> = vec![Box::new(Rec { name: "A", log: log.clone() }), Box::new(Rec { name: "B", log: log.clone() })];
    let d = build(Case::H1, &Scale { rays: 5, ..Scale::tiny() }).unwrap();
    let (samples, trace) = run_interleaved(&mut adapters, Workload::Track, &input(&d, 1, 0), 3, 2).unwrap();
    let calls = log.lock().unwrap().clone();
    let mut want = vec!["prepare A".to_string(), "prepare B".to_string()];
    for _ in 0..5 {
        want.extend(["run A".to_string(), "run B".to_string()]);
    }
    assert_eq!(calls, want, "ABAB per repetition, warm-up first, preparation untimed and first");
    let t: Vec<(usize, usize, bool)> = trace.iter().map(|e: &TraceEntry| (e.rep, e.adapter, e.warmup)).collect();
    assert_eq!(t, vec![(0, 0, true), (0, 1, true), (1, 0, true), (1, 1, true), (2, 0, false), (2, 1, false), (3, 0, false), (3, 1, false), (4, 0, false), (4, 1, false)]);
    assert_eq!(samples[0].ns.len(), 3, "warm-up repetitions are not sampled");
}

fn env(cpu: &str, day: &str, os: &str, arch: &str) -> Env {
    Env { cpu_model: cpu.into(), governor: "performance".into(), ncpu: 8, os: os.into(), arch: arch.into(), date_utc: format!("{day}T10:00:00Z"), machine_id: "m1".into() }
}

fn file(case: &str, tracker: &str, e: Env, failed: usize) -> ResultFile {
    let mut f = ResultFile::new(case, "tiny", e);
    f.rows.push(Row {
        tracker: tracker.into(),
        mode: "surface".into(),
        workload: "w-track".into(),
        threads: 1,
        precision: "f64".into(),
        rays_per_s: Stat { median: 123456.0, min: 100000.0, max: 130000.0, reps: 7 },
        agreement: AgreementSummary { checked: 100, failed, grazing: 0 },
        ..Row::default()
    });
    f.seal().unwrap();
    f
}

#[test]
fn report_refuses_mixed_machines_and_days() {
    let a = file("P1", "smrt", env("CPU A", "2026-11-04", "linux", "x86_64"), 0);
    let b = file("P2", "xdg", env("CPU B", "2026-11-04", "linux", "x86_64"), 0);
    let err = report::render(&[a.clone(), b]).unwrap_err().to_string();
    assert!(err.contains("cpu_model"), "{err}");
    let c = file("P2", "xdg", env("CPU A", "2026-11-05", "linux", "x86_64"), 0);
    assert!(report::render(&[a.clone(), c]).unwrap_err().to_string().contains("day"));
    let d = file("P2", "xdg", env("CPU A", "2026-11-04", "linux", "aarch64"), 0);
    assert!(report::render(&[a.clone(), d]).unwrap_err().to_string().contains("os/arch"));
    assert!(report::render(&[]).is_err());
    let ok = report::render(&[a.clone(), file("P2", "xdg", env("CPU A", "2026-11-04", "linux", "x86_64"), 0)]).unwrap();
    assert!(ok.contains("1.2346e5"), "{ok}");
    assert!(!ok.contains("SMOKE"), "Linux x86_64 is not a smoke result");
    assert!(ok.contains("What was not measured"));
    assert!(ok.contains("Not measured in this report for P1: dagmc-moab, dagmc-dd, xdg"));
}

#[test]
fn a_failed_agreement_hides_the_rates_and_macos_is_smoke() {
    let bad = file("P1", "xdg", env("CPU A", "2026-11-04", "macos", "aarch64"), 3);
    let md = report::render(&[bad]).unwrap();
    assert!(md.contains("n/c (disagrees)"), "{md}");
    assert!(!md.contains("1.2346e5"), "the rate of a disagreeing tracker is not shown");
    assert!(md.contains("3/100"), "the failure count stays visible");
    assert!(md.contains("SMOKE RESULT"));
    // No ratio between trackers is ever computed.
    assert!(!md.to_lowercase().contains("speedup"));
}

#[test]
fn result_files_are_never_overwritten_and_model_runs_stay_out_of_the_repo() {
    let tmp = std::env::temp_dir().join(format!("smrt-bench-res-{}", std::process::id()));
    let mut f = file("P1", "smrt", Env::detect(), 0);
    let p1 = write_result(&tmp, &f).unwrap();
    assert!(p1.exists());
    let before = std::fs::read(&p1).unwrap();
    let p1b = write_result(&tmp, &f).unwrap();
    assert_ne!(p1, p1b, "a second write of the same case gets its own file (counter suffix)");
    assert_eq!(std::fs::read(&p1).unwrap(), before, "the first file is untouched");
    f.model_run = true;
    f.case = "model".into();
    write_result(&tmp, &f).unwrap();
    assert!(!inside_repo(&tmp));
    std::fs::remove_dir_all(&tmp).unwrap();
    // A model run into the crate's own results directory is refused before anything is created.
    let inside = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("results").join("model-test-never-created");
    assert!(inside_repo(&inside));
    let err = write_result(&inside, &f).unwrap_err().to_string();
    assert!(err.contains("repository"), "{err}");
    assert!(!inside.exists());
}

fn listing(root: &std::path::Path) -> Vec<String> {
    fn walk(p: &std::path::Path, out: &mut Vec<String>) {
        let mut ents: Vec<_> = std::fs::read_dir(p).unwrap().map(|e| e.unwrap().path()).collect();
        ents.sort();
        for e in ents {
            out.push(format!("{} {}", e.display(), std::fs::metadata(&e).map_or(0, |m| m.len())));
            if e.is_dir() {
                walk(&e, out);
            }
        }
    }
    let mut v = Vec::new();
    walk(root, &mut v);
    v
}

#[test]
fn model_dry_run_writes_nothing_under_the_repo() {
    let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let tmp = std::env::temp_dir().join(format!("smrt-bench-model-{}", std::process::id()));
    std::fs::create_dir_all(&tmp).unwrap();
    let step = tmp.join("tiny.step");
    // w3a_part is planar: imported periodic faces (cylinders, spheres) have no pcurves yet, so
    // sm_tess refuses them, and the B-spline loft takes minutes in a debug build (DESIGN.md,
    // limits of --model).
    std::fs::write(&step, sm_testkit::step::W3A_PART.bytes).unwrap();
    let before = listing(crate_dir);
    let args = |extra: &[&str]| -> Vec<String> {
        ["run", "--model", step.to_str().unwrap(), "--sag", "0.1", "--rays", "50", "--threads", "1"].iter().chain(extra.iter()).map(|s| s.to_string()).collect()
    };
    assert_eq!(sm_rt_bench::cli::main_with(&args(&["--dry-run"])).unwrap(), 0);
    assert_eq!(listing(crate_dir), before, "dry run: nothing new under the crate (and so nothing under the repo)");
    // Into the crate results directory: refused, nothing created.
    let inside = crate_dir.join("results").join("model-run-test");
    let err = sm_rt_bench::cli::main_with(&args(&["--out", inside.to_str().unwrap()])).unwrap_err().to_string();
    assert!(err.contains("repository"), "{err}");
    assert_eq!(listing(crate_dir), before);
    // Outside the repo it is written.
    let out = tmp.join("out");
    assert_eq!(sm_rt_bench::cli::main_with(&args(&["--out", out.to_str().unwrap()])).unwrap(), 0);
    let written: Vec<_> = std::fs::read_dir(&out).unwrap().collect();
    assert_eq!(written.len(), 1);
    let text = std::fs::read_to_string(written[0].as_ref().unwrap().path()).unwrap();
    assert!(!text.contains(step.to_str().unwrap()), "the model path is not recorded");
    assert_eq!(listing(crate_dir), before);
    std::fs::remove_dir_all(&tmp).unwrap();
}

#[test]
fn run_case_produces_a_sealed_stable_result_and_cli_self_test_passes() {
    let d = build(Case::C2, &Scale { rays: 200, ..Scale::tiny() }).unwrap();
    let mut adapters: Vec<Box<dyn Adapter>> = vec![Box::new(stub_adapter(&d).unwrap())];
    let cfg = RunConfig { reps: 2, warmup: 1, threads: vec![1, 3], workloads: vec![Workload::Track, Workload::Flight], histories: 300, seed: 3 };
    let (f1, ag) = run_case(&d, &mut adapters, "surface", &cfg, Env::detect()).unwrap();
    let (f2, _) = run_case(&d, &mut adapters, "surface", &cfg, Env::detect()).unwrap();
    assert_eq!(f1.result_hash, f2.result_hash, "two runs, one string");
    assert_eq!(f1.rows.len(), 4);
    assert!(f1.rows.iter().all(|r| r.determinism.stable == Some(true)));
    assert!(ag.iter().all(|a| a.failed == 0));
    let track = f1.rows.iter().find(|r| r.workload == "w-track").unwrap();
    assert_eq!(track.geom_error.tracks, 200);
    assert!(track.segments_per_track >= 1.0);
    for c in sm_rt_bench::selftest::self_test().unwrap() {
        assert!(c.ok, "{}: {}", c.name, c.detail);
    }
}
