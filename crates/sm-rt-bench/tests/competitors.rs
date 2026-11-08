//! W3c-BENCH-WORKLOADS: driver protocol of W-fire and W-flight in the DAGMC drivers (parsing,
//! `sigma.txt`, tally comparison) without Docker, and the Docker runs as `#[ignore]` tests
//! (`cargo test -p sm-rt-bench --test competitors -- --ignored`; without Docker or the images they
//! print `skipped: no docker` and pass). Work counts, hashes and analytic values only.

use sm_rt_bench::adapter::{Adapter, Output, WorkInput, Workload};
use sm_rt_bench::cases::{Case, Scale, build};
use sm_rt_bench::docker::{Competitor, CompetitorOpts, parse_driver_output, run_competitor_with, sigma_text};
use sm_rt_bench::formats::RegionSpec;
use sm_rt_bench::runner::{compare_tallies, compare_tallies_with};

fn spec(id: i32, sigma_t: f64, sigma_a: f64) -> RegionSpec {
    RegionSpec { id, name: format!("r{id}"), sigma_t, sigma_a }
}

const ROW: &str = r#"{"tracker":"x","mode":"moab-obb","workload":"w-fire","threads":1,"precision":"f64","rays_per_s":{"median":1.0,"min":1.0,"max":1.0,"reps":1},"determinism":{"hash":"00","stable":true},"counters":{"steps":2,"fires":2}}"#;

#[test]
fn sigma_text_round_trips_every_double() {
    let regions = [spec(0, 0.5, 0.15), spec(1, 0.75, 0.3 * 0.75), spec(-1, 9.0, 9.0), spec(2, 1.0, 0.3)];
    let text = sigma_text(&regions);
    assert_eq!(text.lines().count(), 3, "the complement (id -1) has no line");
    for (line, r) in text.lines().zip(regions.iter().filter(|r| r.id >= 0)) {
        let f: Vec<&str> = line.split(' ').collect();
        assert_eq!(f[0].parse::<i32>().unwrap(), r.id);
        assert_eq!(f[1].parse::<f64>().unwrap().to_bits(), r.sigma_t.to_bits());
        assert_eq!(f[2].parse::<f64>().unwrap().to_bits(), r.sigma_a.to_bits());
    }
}

#[test]
fn driver_output_attaches_each_dump_to_the_row_before_it() {
    let out = format!(
        "noise\n{ROW}\n{{\"fires\":[[1.5,2],[0.25,-1]]}}\n{}\n{{\"tally\":[1,2,3],\"tally_sq\":[1,4,9],\"blocks\":2}}\n{{\"tracks\":[{{\"r\":[0,1],\"c\":[1.0,2.0]}}]}}\n",
        ROW.replace("w-fire", "w-flight")
    );
    let (rows, tracks, dumps) = parse_driver_output("dagmc-moab", &out).unwrap();
    assert_eq!(rows.len(), 2);
    assert!(rows.iter().all(|r| r.tracker == "dagmc-moab"));
    assert_eq!(rows[0].counters.fires, 2);
    assert_eq!(dumps["w-fire"].fires, vec![(1.5, 2), (0.25, -1)]);
    assert_eq!((dumps["w-flight"].tally.clone(), dumps["w-flight"].tally_blocks), (vec![1.0, 2.0, 3.0], 2));
    assert_eq!(tracks.unwrap()[0].regions, vec![0, 1]);
    assert!(parse_driver_output("t", "{\"fires\":[[1,2]]}").is_err(), "a dump needs a row before it");
    assert!(parse_driver_output("t", "{\"tally\":3}").is_err());
}

#[test]
fn track_length_slots_join_the_tally_comparison_only_on_request() {
    // Two regions: [coll0, coll1, len0, len1, leaked, absorbed]; six blocks of equal spread.
    let mk = |t: [f64; 6]| Output { tally: t.to_vec(), tally_sq: t.iter().map(|v| v * v / 6.0 + 6.0).collect(), tally_blocks: 6, ..Output::default() };
    let a = mk([100.0, 200.0, 50.0, 80.0, 30.0, 70.0]);
    let mut shifted = a.clone();
    shifted.tally[2] += 1e6; // only a track-length slot is far off
    assert_eq!(compare_tallies(&shifted, &a).failed, 0);
    let with = compare_tallies_with(&shifted, &a, true);
    assert_eq!((with.checked, with.failed), (6, 1));
    assert_eq!(compare_tallies_with(&a, &a, true).failed, 0);
}

fn docker_ready(tracker: &str) -> bool {
    let ok = |args: &[&str]| std::process::Command::new("docker").args(args).output().is_ok_and(|o| o.status.success());
    let image = format!("smrt-bench-{tracker}");
    let up = ok(&["version", "--format", "{{.Server.Version}}"]) && ok(&["image", "inspect", &image]);
    if !up {
        println!("skipped: no docker (or no image {image})");
    }
    up
}

/// Run the pure-absorber ball of C1 (Sigma_t = Sigma_a = 0.3 inside the ball, vacuum outside)
/// through the container: a history leaks with probability `exp(-0.3 * ball chord)` of its ray, so
/// the leaked count has mean `sum p` and variance `sum p (1 - p)` (4 sigma, plus 1% for the facet
/// sag of the icosphere).
fn absorber_unbiased(tracker: &str) {
    if !docker_ready(tracker) {
        return;
    }
    let data = build(Case::C1, &Scale::tiny()).unwrap();
    let regions = vec![spec(0, 0.3, 0.3), spec(1, 0.0, 0.0)];
    let dir = std::env::temp_dir().join(format!("smrt-bench-abs-{}-{tracker}", std::process::id()));
    data.write(&dir).unwrap();
    let scene = data.scene.as_ref().unwrap();
    let n = data.rays.len();
    let (mut mean, mut var) = (0.0, 0.0);
    for i in 0..n {
        let t = scene.track(data.rays.origin[i], data.rays.dir[i]);
        let chord: f64 = t.regions.iter().zip(&t.chords).filter(|(r, _)| **r == 0).map(|(_, c)| c).sum();
        let p = (-0.3 * chord).exp();
        mean += p;
        var += p * (1.0 - p);
    }
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let opts = CompetitorOpts { workloads: vec!["flight".into()], histories: n, seed: 7, regions };
    let res = run_competitor_with(root, tracker, &dir, &data.case, &[2], 1, 0, &opts);
    let _ = std::fs::remove_dir_all(&dir);
    let Competitor::Rows { rows, dumps, .. } = res else { panic!("competitor did not run: {res:?}") };
    let out = &dumps["w-flight"];
    let (leaked, absorbed) = (out.tally[4], out.tally[5]);
    assert_eq!(rows[0].lost, 0);
    assert_eq!(leaked + absorbed, n as f64, "every history ends leaked or absorbed");
    let tol = 4.0 * var.sqrt() + 0.01 * mean;
    assert!((leaked - mean).abs() <= tol, "{tracker}: leaked {leaked} vs {mean} +- {tol}");
}

#[test]
#[ignore = "needs Docker and the smrt-bench-dagmc-moab image"]
fn dagmc_moab_flight_of_an_absorber_is_unbiased() {
    absorber_unbiased("dagmc-moab");
}

#[test]
#[ignore = "needs Docker and the smrt-bench-dagmc-dd image"]
fn dagmc_dd_flight_of_an_absorber_is_unbiased() {
    absorber_unbiased("dagmc-dd");
}

/// Hinted and located fire of a container equal the sm-rt facets fire ray by ray, and a second
/// run reproduces the driver's output hash (also across thread counts).
fn fire_equals_the_facets_twin(tracker: &str) {
    if !docker_ready(tracker) {
        return;
    }
    let data = build(Case::C2, &Scale::tiny()).unwrap();
    let dir = std::env::temp_dir().join(format!("smrt-bench-fire-{}-{tracker}", std::process::id()));
    data.write(&dir).unwrap();
    let w = WorkInput { rays: &data.rays, regions: &data.manifest.regions, threads: 1, histories: 0, seed: 1 };
    let mut a = sm_rt_bench::smrt::facets_adapter(&data).unwrap();
    a.prepare(Workload::Fire, &w).unwrap();
    let reference = a.run(Workload::Fire, &w).unwrap();
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let opts = CompetitorOpts { workloads: vec!["fire".into(), "fire-locate".into()], ..CompetitorOpts::default() };
    let res = run_competitor_with(root, tracker, &dir, &data.case, &[1, 2], 1, 0, &opts);
    let _ = std::fs::remove_dir_all(&dir);
    let Competitor::Rows { rows, dumps, .. } = res else { panic!("competitor did not run: {res:?}") };
    for wl in ["w-fire", "w-fire-locate"] {
        let (s, bad) = sm_rt_bench::compare::compare_fires(&dumps[wl].fires, &reference.fires, data.manifest.diag, data.manifest.chord_tol).unwrap();
        assert_eq!((s.failed, bad.len()), (0, 0), "{tracker} {wl}");
        let hashes: Vec<&String> = rows.iter().filter(|r| r.workload == wl).map(|r| &r.determinism.hash).collect();
        assert_eq!(hashes.len(), 2);
        assert_eq!(hashes[0], hashes[1], "{wl}: hash differs across thread counts");
    }
}

#[test]
#[ignore = "needs Docker and the smrt-bench-dagmc-moab image"]
fn dagmc_moab_fire_equals_the_sm_rt_facets_fire() {
    fire_equals_the_facets_twin("dagmc-moab");
}

#[test]
#[ignore = "needs Docker and the smrt-bench-dagmc-dd image"]
fn dagmc_dd_fire_equals_the_sm_rt_facets_fire() {
    fire_equals_the_facets_twin("dagmc-dd");
}
