//! W4-RT-TETS (a, b): the tet walk of sm-rt against an independent reference, the analytic cube
//! chord and the XDG driver protocol (`walk` dump). Work counts, hashes and analytic values only.
//!
//! The XDG container itself is exercised by the `#[ignore]` tests at the end (they need Docker and
//! a native `smrt-bench-xdg` image, which only a host where XDG builds can have: XDG includes
//! `<immintrin.h>` unconditionally, so there is no linux/aarch64 image). `SMRT_XDG_STDOUT=FILE`
//! (the scale follows from its ray count, or `SMRT_XDG_SCALE`) judges the saved stdout of a driver run
//! (`run_case.sh H1` with `SMRT_WORKLOADS=walk`) the same way, e.g. from an emulated amd64 image.

use sm_rt_bench::adapter::{Adapter, Output, WorkInput, Workload};
use sm_rt_bench::cases::{Case, CaseData, Scale, build};
use sm_rt_bench::compare::compare;
use sm_rt_bench::docker::{Competitor, CompetitorOpts, parse_driver_output, run_competitor_with};
use sm_rt_bench::formats::Tets;
use sm_rt_bench::runner::walk_truth;
use sm_rt_bench::smrt::walk_adapter;
use sm_rt_bench::truth::Track;

fn h1() -> CaseData {
    build(Case::H1, &Scale::tiny()).unwrap()
}

fn walk(d: &CaseData, threads: usize) -> Output {
    let mut a = walk_adapter(d.tets.as_ref().unwrap()).unwrap();
    a.run(Workload::Walk, &WorkInput { rays: &d.rays, regions: &d.manifest.regions, threads, histories: 0, seed: 1 }).unwrap()
}

/// The chord of the ray inside tet `t`: its intersection with the four half-spaces, by clipping
/// (independent of every tracker). `None` when the ray misses the tet.
fn clip(nodes: &[[f64; 3]], t: [u32; 4], o: [f64; 3], d: [f64; 3]) -> Option<(f64, f64)> {
    let v: Vec<[f64; 3]> = t.iter().map(|&i| nodes[i as usize]).collect();
    let sub = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let cross = |a: [f64; 3], b: [f64; 3]| [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let (mut lo, mut hi) = (f64::NEG_INFINITY, f64::INFINITY);
    for k in 0..4 {
        // Face opposite vertex k, normal pointing away from vertex k.
        let f: Vec<[f64; 3]> = (0..4).filter(|&j| j != k).map(|j| v[j]).collect();
        let mut n = cross(sub(f[1], f[0]), sub(f[2], f[0]));
        if dot(n, sub(v[k], f[0])) > 0.0 {
            n = [-n[0], -n[1], -n[2]];
        }
        let nd = dot(n, d);
        let dist = dot(n, sub(f[0], o)); // inside: n . (p - f0) <= 0, p = o + t d
        if nd.abs() < 1e-300 {
            if dist < 0.0 {
                return None;
            }
            continue;
        }
        let t = dist / nd;
        if nd > 0.0 { hi = hi.min(t) } else { lo = lo.max(t) }
    }
    (lo < hi).then_some((lo, hi))
}

/// Every ray's tets ordered by entry parameter, from the clip of all tets (O(rays x tets)).
fn brute(d: &CaseData) -> Vec<Track> {
    let tets = d.tets.as_ref().unwrap();
    (0..d.rays.len())
        .map(|i| {
            let mut hits: Vec<(f64, usize, f64)> = tets
                .tets
                .iter()
                .enumerate()
                .filter_map(|(k, &t)| clip(&tets.nodes, t, d.rays.origin[i], d.rays.dir[i]).map(|(a, b)| (a.max(0.0), k, b - a.max(0.0))).filter(|h| h.2 > 0.0))
                .collect();
            hits.sort_by(|a, b| a.0.total_cmp(&b.0));
            Track { regions: hits.iter().map(|h| h.1 as i32).collect(), chords: hits.iter().map(|h| h.2).collect() }
        })
        .collect()
}

#[test]
fn the_tet_walk_equals_an_independent_clip_of_every_tet() {
    let d = h1();
    let out = walk(&d, 1);
    let reference = brute(&d);
    let rep = compare(&out.tracks, &reference, d.manifest.diag, 1e-9 * d.manifest.diag).unwrap();
    assert_eq!((rep.checked, rep.failed), (d.rays.len(), 0), "{:?}", rep.listed.first());
    assert!(out.counters.segments as usize > d.rays.len(), "rays cross more than one tet");
    assert_eq!(out.counters.lost, 0);
}

#[test]
fn walk_chords_sum_to_the_analytic_cube_chord_and_tets_are_adjacent() {
    let d = h1();
    let out = walk(&d, 1);
    let tets = d.tets.as_ref().unwrap();
    let (g, failed) = walk_truth(tets, &d.rays, &out.tracks, d.manifest.diag).unwrap();
    assert_eq!((g.tracks, failed), (d.rays.len(), 0));
    assert!(g.max_chord_err <= 1e-9 * d.manifest.diag, "{}", g.max_chord_err);
    let shared = |a: usize, b: usize| tets.tets[a].iter().filter(|v| tets.tets[b].contains(v)).count();
    let (mut pairs, mut faces) = (0usize, 0usize);
    for t in &out.tracks {
        for w in t.regions.windows(2) {
            let s = shared(w[0] as usize, w[1] as usize);
            assert!(s >= 1, "tets {} and {} do not touch", w[0], w[1]);
            pairs += 1;
            faces += usize::from(s == 3);
        }
    }
    assert!(pairs > 0 && faces * 100 >= pairs * 98, "{faces} of {pairs} steps cross a face");
}

#[test]
fn the_cell_to_tet_map_is_a_bijection_and_the_walk_is_deterministic() {
    let d = h1();
    let nt = d.tets.as_ref().unwrap().tets.len();
    let one = walk(&d, 1);
    assert!(one.tracks.iter().flat_map(|t| &t.regions).all(|&r| r >= 0 && (r as usize) < nt));
    let h = one.content_hash();
    assert_eq!(walk(&d, 1).content_hash(), h, "run twice");
    for t in [2, 3] {
        assert_eq!(walk(&d, t).content_hash(), h, "{t} threads");
    }
}

#[test]
fn walk_truth_flags_a_wrong_chord_sum_and_a_wrong_track_count() {
    let d = h1();
    let tets: &Tets = d.tets.as_ref().unwrap();
    let mut tracks = walk(&d, 1).tracks;
    assert_eq!(walk_truth(tets, &d.rays, &tracks, d.manifest.diag).unwrap().1, 0);
    tracks[3].chords[0] += 1e-6;
    tracks[5].chords.pop();
    tracks[5].regions.pop();
    let (g, failed) = walk_truth(tets, &d.rays, &tracks, d.manifest.diag).unwrap();
    assert_eq!(failed, 2);
    assert!(g.max_chord_err >= 1e-6 * 0.99);
    tracks.pop();
    assert!(walk_truth(tets, &d.rays, &tracks, d.manifest.diag).is_err());
}

const WALK_ROW: &str = r#"{"tracker":"x","mode":"embree","workload":"w-walk","threads":1,"precision":"mixed","rays_per_s":{"median":1.0,"min":1.0,"max":1.0,"reps":1},"determinism":{"hash":"00","stable":true},"counters":{"steps":3,"segments":3}}"#;

#[test]
fn a_walk_dump_is_kept_with_its_row_as_tet_indices_and_chords() {
    let out = format!("noise\n{WALK_ROW}\n{{\"walk\":[{{\"r\":[7,9],\"c\":[1.5,0.25]}},{{\"r\":[],\"c\":[]}}]}}\n");
    let (rows, tracks, dumps) = parse_driver_output("xdg", &out).unwrap();
    assert_eq!((rows.len(), rows[0].workload.as_str(), rows[0].precision.as_str()), (1, "w-walk", "mixed"));
    assert!(tracks.is_none(), "a walk dump is not the track dump");
    let t = &dumps["w-walk"].tracks;
    assert_eq!((t[0].regions.clone(), t[0].chords.clone(), t[1].regions.len()), (vec![7, 9], vec![1.5, 0.25], 0));
    assert!(parse_driver_output("xdg", "{\"walk\":[]}").is_err(), "a dump needs a row before it");
}

/// What the XDG walk must satisfy against the sm-rt walk: tet sequences equal and chords within
/// `1e-9 diag` (after dropping grazing chords), chord sums equal the cube chord.
fn judge_xdg_walk(d: &CaseData, xdg: &Output) {
    let reference = walk(d, 1);
    let diag = d.manifest.diag;
    let rep = compare(&xdg.tracks, &reference.tracks, diag, 1e-9 * diag).unwrap();
    assert_eq!((rep.checked, rep.failed), (d.rays.len(), 0), "{:?}", rep.listed.first());
    let (g, failed) = walk_truth(d.tets.as_ref().unwrap(), &d.rays, &xdg.tracks, diag).unwrap();
    assert_eq!(failed, 0, "chord sums: max error {}", g.max_chord_err);
}

#[test]
fn saved_xdg_walk_output_agrees_with_the_sm_rt_walk() {
    let Ok(path) = std::env::var("SMRT_XDG_STDOUT") else {
        println!("skipped: SMRT_XDG_STDOUT not set");
        return;
    };
    let (rows, _, dumps) = parse_driver_output("xdg", &std::fs::read_to_string(path).unwrap()).unwrap();
    // The scale is the one whose ray count equals the dump's (tiny 200, reduced 10000, full 200000),
    // unless SMRT_XDG_SCALE names it.
    let n = dumps["w-walk"].tracks.len();
    let scale = match std::env::var("SMRT_XDG_SCALE") {
        Ok(s) => Scale::parse(&s).unwrap_or_else(|| panic!("SMRT_XDG_SCALE={s}: unknown scale")),
        Err(_) => [Scale::tiny(), Scale::reduced(), Scale::full()].into_iter().find(|s| s.rays == n).unwrap_or_else(|| panic!("the saved walk output has {n} tracks, which is no scale's ray count; set SMRT_XDG_SCALE")),
    };
    assert_eq!(n, scale.rays, "the saved walk output has {n} tracks but scale {} has {} rays", scale.name, scale.rays);
    let d = build(Case::H1, &scale).unwrap();
    assert!(rows.iter().all(|r| r.precision == "mixed" && r.workload == "w-walk" && r.lost == 0));
    judge_xdg_walk(&d, &dumps["w-walk"]);
}

fn xdg_ready() -> bool {
    let ok = |args: &[&str]| std::process::Command::new("docker").args(args).output().is_ok_and(|o| o.status.success());
    let up = ok(&["version", "--format", "{{.Server.Version}}"]) && ok(&["image", "inspect", "smrt-bench-xdg"]);
    if !up {
        println!("skipped: no docker (or no image smrt-bench-xdg)");
    }
    up
}

#[test]
#[ignore = "needs Docker and a native smrt-bench-xdg image (x86_64 hosts)"]
fn the_xdg_container_walks_the_same_tets_as_sm_rt() {
    if !xdg_ready() {
        return;
    }
    let d = h1();
    let dir = std::env::temp_dir().join(format!("smrt-bench-walk-{}", std::process::id()));
    d.write(&dir).unwrap();
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let opts = CompetitorOpts { workloads: vec!["walk".into()], ..CompetitorOpts::default() };
    let res = run_competitor_with(root, "xdg", &dir, &d.case, &[1, 2], 1, 0, &opts);
    let _ = std::fs::remove_dir_all(&dir);
    let Competitor::Rows { rows, dumps, .. } = res else { panic!("competitor did not run: {res:?}") };
    judge_xdg_walk(&d, &dumps["w-walk"]);
    let hashes: Vec<&String> = rows.iter().filter(|r| r.workload == "w-walk").map(|r| &r.determinism.hash).collect();
    assert_eq!(hashes.len(), 2);
    assert_eq!(hashes[0], hashes[1], "walk hash differs across thread counts");
    assert!(rows.iter().all(|r| r.precision == "mixed"));
}
