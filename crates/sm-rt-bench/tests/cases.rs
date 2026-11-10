//! Generator acceptance: hashes, determinism across thread counts, closed-form counts and
//! volumes, facet-twin tolerance, analytic truth.

use core::f64::consts::PI;
use sm_rt_bench::cases::{self, Case, Scale, build, check_against};
use sm_rt_bench::formats::{Facets, Rays, Tets, platform_tag};
use sm_rt_bench::legacy::{self, Legacy};
use sm_rt_bench::rng::sha256_hex;
use sm_rt_bench::twin;

const EXPECTED: &str = include_str!("../expected_sha256.txt");

fn pool(t: usize) -> rayon::ThreadPool {
    rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap()
}

/// Hashes of every file of a case, built inside a pool of `t` threads.
fn case_hashes(case: Case, scale: &Scale, t: usize) -> Vec<String> {
    pool(t).install(|| build(case, scale).unwrap().files().unwrap().iter().map(|(n, b)| format!("{n} {}", sha256_hex(b))).collect())
}

#[test]
fn legacy_cases_equal_the_w3a_files() {
    // sha256 of sm-rt-capi/bench/expected_sha256.txt (200 000 rays), produced there by gen_case.py.
    let w3a = [
        ("C1", "6b2e378f02c9d2f112f5ca47cf66f7eb7a7f546cc34c5811b591d136d877d2c4", "42f6c62bae567e93724d60fddd5e682b8307a932cc33b0e8dd4c533a94c66047"),
        ("C2", "cd4171181096f065ef35c578020776cbc6fca4427d1e12339d29fc6b21db2b94", "a4bad00372dbb1d3200ed2c418cce0387579de684ad39c5772615493055adc54"),
        ("C3", "de41d3385e45b7545bb13c226e68f12de488e0467b80608139da64af65438ef1", "93c78ca90f72004e4943b2de7a62be7f9cf931fbbbfee0968af45ba1afc39209"),
    ];
    for (name, facets, rays) in w3a {
        let c = Legacy::parse(name).unwrap();
        let g = legacy::build(c).unwrap();
        assert_eq!(sha256_hex(&g.facets.to_bytes()), facets, "{name} facets");
        let r = legacy::rays(c, &g, 200_000).unwrap();
        assert_eq!(sha256_hex(&r.to_bytes()), rays, "{name} rays");
    }
}

#[test]
fn recorded_hashes_match_twice_and_across_thread_counts() {
    let blessed_here = EXPECTED.lines().any(|l| l.trim_end().ends_with(&format!("@{}", platform_tag())));
    for scale in [Scale::tiny(), Scale::reduced()] {
        for case in Case::ALL {
            let data = build(case, &scale).unwrap();
            let rep = check_against(EXPECTED, &scale, &data).unwrap();
            assert!(rep.mismatched.is_empty(), "{} {}: {:?}", scale.name, case.name(), rep.mismatched);
            // Platform-independent files (C1-C3 and H1 facets, rays, tets) are blessed for every
            // platform and must match. The rest is blessed per os-arch: on a platform with no
            // blessed lines it is reported, not failed (blessing needs a run on that machine).
            let independent: Vec<String> = data.files().unwrap().iter().map(|(n, _)| n.clone()).filter(|n| n != "smcase.json" && matches!(case, Case::C1 | Case::C2 | Case::C3 | Case::H1)).map(|n| format!("{}/{}/{n}", scale.name, case.name())).collect();
            assert!(independent.iter().all(|k| !rep.unblessed.contains(k) && !rep.mismatched.contains(k)), "{} {}: platform-independent file unblessed: {:?}", scale.name, case.name(), rep.unblessed);
            assert!(rep.matched >= independent.len().max(if blessed_here { 3 } else { 0 }), "{} {}: matched {}", scale.name, case.name(), rep.matched);
            if blessed_here {
                assert!(rep.unblessed.is_empty(), "{} {} has no recorded hash for {}: {:?} (bless with `smrt-bench gen --print`)", scale.name, case.name(), platform_tag(), rep.unblessed);
            } else if !rep.unblessed.is_empty() {
                eprintln!("note: no blessed hashes for {} ({} {}): {:?}", platform_tag(), scale.name, case.name(), rep.unblessed);
            }
            if scale.name != "tiny" {
                continue; // the reduced scale is slow in a debug build: hashes once, threads at tiny
            }
            let h1 = case_hashes(case, &scale, 1);
            let hn = case_hashes(case, &scale, 4);
            assert_eq!(h1, hn, "{} {}: 1 vs 4 threads", scale.name, case.name());
            assert_eq!(h1, case_hashes(case, &scale, 1), "{} {}: repeated", scale.name, case.name());
        }
    }
}

#[test]
fn determinism_harness_agrees_on_a_curved_case() {
    let tiny = Scale::tiny();
    let h = sm_testkit::determinism::check(2, 4, |t| case_hashes(Case::P1, &tiny, t)).unwrap();
    assert!(!h.is_empty());
}

#[test]
fn p2_triangles_are_copies_times_one_pin_and_the_lattice_is_17() {
    // Coarse sag keeps the 17 x 17 lattice small enough for a test.
    let s = Scale { name: "tiny", rays: 20, f1_rays: 20, sag_pin: 2e-2, lattice_n: 17, ..Scale::tiny() };
    let d = build(Case::P2, &s).unwrap();
    assert_eq!(d.manifest.copies, 9 * 17 * 17, "pin_assembly(17) is a 3x3 cluster of 17x17 lattices");
    assert_eq!(d.manifest.prototypes, 1);
    let one = Scale { lattice_n: 1, ..s.clone() };
    let base = build(Case::P2, &one).unwrap();
    assert_eq!(base.manifest.copies, 9);
    let per_pin = base.manifest.triangles / 9;
    assert_eq!(base.manifest.triangles, 9 * per_pin);
    assert_eq!(d.manifest.triangles, d.manifest.copies * per_pin, "exactly copies times the triangles of one pin");
    assert_eq!(d.facets.verts.len(), d.manifest.copies * (base.facets.verts.len() / 9));
}

fn polygon_area(n: usize, r: f64) -> f64 {
    0.5 * n as f64 * r * r * (2.0 * PI / n as f64).sin()
}

fn ring_count(f: &Facets, r: f64, z: f64) -> usize {
    let mut ids: Vec<u32> = f.tris.iter().flatten().copied().filter(|&v| {
        let p = f.verts[v as usize];
        ((p[0] * p[0] + p[1] * p[1]).sqrt() - r).abs() < 1e-5 && (p[2] - z).abs() < 1e-9
    }).collect();
    ids.sort_unstable();
    ids.dedup();
    ids.len()
}

#[test]
fn p1_facet_twin_respects_the_recorded_sag_and_is_watertight() {
    let (rf, rc, h, pitch) = (0.4096, 0.475, 100.0, 1.26);
    for sag in [1e-2, 1e-3, 1e-4] {
        let s = Scale { name: "tiny", rays: 10, sag_pin: sag, ..Scale::tiny() };
        let d = build(Case::P1, &s).unwrap();
        assert_eq!(d.manifest.sag, sag);
        for r in [rf, rc] {
            let dev = twin::max_cylinder_deviation(&d.facets, r);
            assert!(dev <= sag, "r {r}: max radial deviation {dev} exceeds the recorded sag {sag}");
            assert!(dev > 0.0);
        }
        // Closed-form volumes of the inscribed polygons (n measured from the vertices) and the
        // divergence-theorem volumes of the soup agree: orientation and watertightness.
        let (nf, nc) = (ring_count(&d.facets, rf, 0.0), ring_count(&d.facets, rc, 0.0));
        let (vf, vc_total, vw) = (twin::region_volume(&d.facets, 0), twin::region_volume(&d.facets, 0) + twin::region_volume(&d.facets, 1), twin::region_volume(&d.facets, 2));
        assert!(((vf - polygon_area(nf, rf) * h) / vf).abs() < 1e-5, "fuel {vf} vs polygon n={nf}");
        assert!(((vc_total - polygon_area(nc, rc) * h) / vc_total).abs() < 1e-5, "fuel+clad {vc_total} vs polygon n={nc}");
        let total = vf + (vc_total - vf) + vw;
        assert!(((total - pitch * pitch * h) / total).abs() < 1e-6, "all regions fill the cell: {total}");
    }
}

#[test]
fn analytic_chords_through_the_cylinders_are_closed_form() {
    let (rf, rc, h) = (0.4096f64, 0.475f64, 100.0f64);
    let s = Scale { rays: 10_000, ..Scale::tiny() };
    let d = build(Case::P1, &s).unwrap();
    let scene = d.scene.as_ref().unwrap();
    let mut checked = 0;
    for i in 0..d.rays.len() {
        let (o, dir) = (d.rays.origin[i], d.rays.dir[i]);
        let tr = scene.track(o, dir);
        // Per-track chord sum equals the distance to the box boundary (independent slab formula).
        let half = 0.63;
        let mut texit = f64::INFINITY;
        for (a, (lo, hi)) in [(-half, half), (-half, half), (0.0, h)].into_iter().enumerate() {
            if dir[a] > 0.0 {
                texit = texit.min((hi - o[a]) / dir[a]);
            } else if dir[a] < 0.0 {
                texit = texit.min((lo - o[a]) / dir[a]);
            }
        }
        let sum: f64 = tr.chords.iter().sum();
        assert!((sum - texit).abs() <= 1e-10 * texit.max(1.0), "ray {i}: sum {sum} vs exit {texit}");
        // Fuel chord 2 sqrt(r^2 - b^2) / |d_xy| when the chord lies inside the z slab.
        if let Some(k) = tr.regions.iter().position(|&r| r == 0) {
            let t_in: f64 = tr.chords[..k].iter().sum();
            let (z0, z1) = (o[2] + dir[2] * t_in, o[2] + dir[2] * (t_in + tr.chords[k]));
            if z0 > 0.0 && z0 < h && z1 > 0.0 && z1 < h && tr.regions.len() == 5 {
                let dxy = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();
                let b = (o[0] * dir[1] - o[1] * dir[0]).abs() / dxy;
                let want = 2.0 * (rf * rf - b * b).sqrt() / dxy;
                assert!(((tr.chords[k] - want) / want).abs() < 1e-11, "ray {i}: fuel chord {} vs {want}", tr.chords[k]);
                let clad = 2.0 * (rc * rc - b * b).sqrt() / dxy - want;
                assert!(((tr.chords[k - 1] + tr.chords[k + 1] - clad) / clad).abs() < 1e-10);
                checked += 1;
            }
        }
    }
    assert!(checked > 50, "only {checked} rays crossed the fuel inside the slab");
}

#[test]
fn start_regions_are_exact_for_facets_and_analytic_model() {
    // The facet twin and the scene agree on every start label (the band rejection at work).
    let s = Scale { rays: 300, ..Scale::reduced() };
    for case in [Case::P1, Case::C2, Case::H1] {
        let d = build(case, &s).unwrap();
        let scene = d.scene.as_ref().unwrap();
        for i in 0..d.rays.len() {
            assert_eq!(scene.region_at(d.rays.origin[i]), d.rays.region[i]);
            // Facet label: parity of the facet soup gives inside/outside of the whole model.
            assert_eq!(sm_rt_bench::parity::point_in_facets(&d.facets, d.rays.origin[i]), 1, "{} ray {i}", case.name());
        }
    }
}

#[test]
fn h1_tets_are_positive_tile_the_cube_and_boxes_conform() {
    let d = build(Case::H1, &Scale::reduced()).unwrap();
    let t = d.tets.as_ref().unwrap();
    let vol = |m: &[[u32; 4]], i: usize| {
        let p: Vec<[f64; 3]> = m[i].iter().map(|&v| t.nodes[v as usize]).collect();
        let (a, b, c) = ([p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]], [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]], [p[3][0] - p[0][0], p[3][1] - p[0][1], p[3][2] - p[0][2]]);
        (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    };
    let mut total = 0.0;
    let mut per_region = std::collections::BTreeMap::new();
    for i in 0..t.tets.len() {
        let v = vol(&t.tets, i);
        assert!(v > 0.0, "tet {i} has volume {v}");
        total += v;
        *per_region.entry(t.region[i]).or_insert(0.0) += v;
    }
    assert!(((total - 8e6) / 8e6).abs() < 1e-12, "{total}");
    // Box inclusions (odd bit parity: ids 1, 2, 4, 7) are grid aligned: exact volume (20 cm)^3.
    for k in [1, 2, 4, 7] {
        assert!((per_region[&k] - 8000.0).abs() < 1e-6, "box {k}: {}", per_region[&k]);
    }
    // The facet twin of a box inclusion has the same volume by the divergence theorem.
    for k in [1, 2, 4, 7] {
        assert!((twin::region_volume(&d.facets, k) - 8000.0).abs() < 1e-6);
    }
}

#[test]
fn f1_shells_enclose_the_torus_volumes_within_the_sag() {
    let s = Scale { rays: 50, f1_rays: 50, sag_f1: 0.5, f1_shells: 6, ..Scale::tiny() };
    let d = build(Case::F1, &s).unwrap();
    let minors = cases::f1_minors(6);
    let torus = |r: f64| 2.0 * PI * PI * 600.0 * r * r;
    let mut prev = 0.0;
    for (k, &r) in minors.iter().enumerate() {
        let want = torus(r) - prev;
        prev = torus(r);
        let got = twin::region_volume(&d.facets, k as i32);
        assert!(got > 0.0, "shell {k} orientation");
        // Inscribed tessellation within the sag: relative deficit at most about 2 sag / r per surface.
        let tol = 4.0 * s.sag_f1 / minors[0];
        assert!(((got - want) / want).abs() < tol, "region {k}: {got} vs {want}");
    }
}

#[test]
fn file_formats_round_trip_and_reject_garbage() {
    let d = build(Case::H1, &Scale::tiny()).unwrap();
    assert_eq!(Facets::from_bytes(&d.facets.to_bytes()).unwrap(), d.facets);
    assert_eq!(Rays::from_bytes(&d.rays.to_bytes()).unwrap(), d.rays);
    assert_eq!(Tets::from_bytes(&d.tets.as_ref().unwrap().to_bytes()).unwrap(), *d.tets.as_ref().unwrap());
    // Truncated and corrupt inputs give errors, never panics.
    for bytes in [d.facets.to_bytes(), d.rays.to_bytes(), d.tets.as_ref().unwrap().to_bytes()] {
        for cut in [0, 1, 4, 7, bytes.len() / 2, bytes.len() - 1] {
            let b = &bytes[..cut];
            assert!(Facets::from_bytes(b).is_err() || Rays::from_bytes(b).is_err());
            let _ = Tets::from_bytes(b);
        }
    }
    let mut bad = d.facets.to_bytes();
    bad[8 + d.facets.verts.len() * 24] = 0xff; // first triangle index out of range
    bad[8 + d.facets.verts.len() * 24 + 3] = 0xff;
    assert!(Facets::from_bytes(&bad).is_err());
    let mut r = d.rays.to_bytes();
    r[4 + 24..4 + 32].copy_from_slice(&5.0f64.to_le_bytes()); // non-unit direction
    assert!(Rays::from_bytes(&r).is_err());
}

#[test]
fn manifests_carry_the_workload_and_the_hashes() {
    let d = build(Case::P1, &Scale::tiny()).unwrap();
    assert_eq!(d.manifest.schema, "smcase v1");
    assert_eq!(d.manifest.regions.len(), 3);
    assert!(d.manifest.regions.iter().all(|r| r.sigma_t > 0.0 && r.sigma_a < r.sigma_t));
    assert_eq!(d.manifest.files["facets.bin"], sha256_hex(&d.facets.to_bytes()));
    assert_eq!(d.manifest.files["rays.bin"], sha256_hex(&d.rays.to_bytes()));
    assert_eq!(d.manifest.platform, platform_tag());
    let json = serde_json::to_string(&d.manifest).unwrap();
    assert_eq!(serde_json::from_str::<sm_rt_bench::formats::Manifest>(&json).unwrap(), d.manifest);
}

/// P1 at `tiny` used to have zero-area moderator triangles (a ring point exactly on the diagonal of
/// the square cell got a second vertex there), so its `facets` row was `partial`. None remain, and
/// the tracker accepts the twin.
#[test]
fn p1_has_no_zero_area_triangles_and_its_facets_twin_builds_at_tiny() {
    for scale in [Scale::tiny(), Scale::reduced()] {
        let d = build(Case::P1, &scale).unwrap();
        let f = &d.facets;
        let zero = f
            .tris
            .iter()
            .filter(|t| {
                let p = |i: u32| f.verts[i as usize];
                let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
                let (u, v) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
                let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
                n[0] * n[0] + n[1] * n[1] + n[2] * n[2] == 0.0
            })
            .count();
        assert_eq!(zero, 0, "{}: zero-area triangles", scale.name);
        assert!(sm_rt_bench::smrt::facets_adapter(&d).is_ok(), "{}: facets twin must build", scale.name);
    }
}

/// The only lines of `expected_sha256.txt` that W3c-BENCH-PIN changed against the blessed file of
/// `9ac71ca` (kept next to this test) are the `tiny/P1/` ones; every other hash is unchanged.
#[test]
fn only_tiny_p1_lines_of_the_recorded_hashes_changed() {
    const BEFORE: &str = include_str!("expected_sha256.9ac71ca.txt");
    const NOW: &str = include_str!("../expected_sha256.txt");
    // Compare only the keys present in both files, so later blesses (new keys) do not break it.
    let map = |t: &'static str| -> std::collections::BTreeMap<&'static str, &'static str> { t.lines().filter_map(|l| l.split_once("  ").map(|(h, k)| (k, h))).collect() };
    let (before, now) = (map(BEFORE), map(NOW));
    let mut changed = Vec::new();
    for (k, h) in &before {
        if let Some(n) = now.get(k)
            && n != h
        {
            changed.push(*k);
        }
    }
    assert!(!changed.is_empty(), "the weld moved the tiny P1 facets: its lines are re-blessed");
    for k in &changed {
        assert!(k.starts_with("tiny/P1/"), "changed line outside tiny/P1: {k}");
    }
}

// ---- W4-RT-ROWS: A17, the refcorpus 17x17 assembly deck ---------------------------------------------

/// The two deck hashes in the code are the ones `tools/refcorpus/manifest.toml` records for
/// `openmc-assembly-17x17` (the manifest is the source; this keeps the constants honest).
#[test]
fn the_a17_deck_hashes_are_the_manifest_values() {
    let manifest = std::fs::read_to_string(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/refcorpus/manifest.toml")).unwrap();
    let entry = manifest.split("[[entry]]").find(|e| e.contains("id = \"openmc-assembly-17x17\"")).expect("the manifest lists openmc-assembly-17x17");
    let sha_of = |file: &str| -> String {
        let after = entry.split(&format!("path = \"openmc-assembly-17x17/{file}\"")).nth(1).unwrap_or_else(|| panic!("no output {file}"));
        let line = after.lines().find(|l| l.starts_with("sha256")).unwrap();
        line.split('"').nth(1).unwrap().to_string()
    };
    assert_eq!(sha_of("geometry.xml"), cases::A17_GEOMETRY_SHA256);
    assert_eq!(sha_of("materials.xml"), cases::A17_MATERIALS_SHA256);
}

/// A17 is built only when named; a default run never needs the corpus.
#[test]
fn a17_is_not_in_the_default_case_list_but_parses() {
    assert!(!Case::ALL.contains(&Case::A17));
    assert_eq!(Case::parse("A17"), Some(Case::A17));
    assert_eq!(Case::parse("A17").map(Case::name), Some("A17"));
}

/// Where the corpus is not built the case is `Unavailable` (a skip, never a failure); a wrong deck
/// is refused by its hash.
#[test]
fn a17_without_the_corpus_is_unavailable_and_a_changed_deck_is_refused() {
    let dir = std::env::temp_dir().join(format!("smrt-bench-a17-{}", std::process::id()));
    struct Rm(std::path::PathBuf);
    impl Drop for Rm {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }
    let _g = Rm(dir.clone());
    let e = cases::a17_deck_in(&dir).unwrap_err();
    assert!(matches!(e, sm_rt_bench::error::BenchError::Unavailable(_)), "{e}");
    std::fs::create_dir_all(dir.join("openmc-assembly-17x17")).unwrap();
    std::fs::write(dir.join("openmc-assembly-17x17/geometry.xml"), "<geometry/>").unwrap();
    let e = cases::a17_deck_in(&dir).unwrap_err();
    assert!(matches!(e, sm_rt_bench::error::BenchError::Check(_)) && e.to_string().contains("not the manifest's"), "{e}");
}

/// The recorded `reduced/A17` bytes (blessed on macOS arm64; another platform reports them
/// unblessed, a mismatch on a blessed platform fails).
#[test]
fn a17_reduced_bytes_equal_the_recorded_hashes() {
    if !corpus_present() {
        return;
    }
    let d = build(Case::A17, &Scale::reduced()).unwrap();
    let rep = check_against(EXPECTED, &Scale::reduced(), &d).unwrap();
    assert!(rep.ok(), "{rep:?}");
    if !rep.unblessed.is_empty() {
        println!("unblessed on {}: {:?}", platform_tag(), rep.unblessed);
    }
}

fn corpus_present() -> bool {
    let p = cases::refcorpus_dir().join("openmc-assembly-17x17/geometry.xml");
    let ok = p.exists();
    if !ok {
        println!("skipped: {} is not built (python3 -I tools/refcorpus/build.py build)", p.display());
    }
    ok
}

/// With the corpus built: the case is deterministic across thread counts, platform-keyed, and the
/// exact deck tracker gives the closed-form chords along a row of 17 fuel pins (analytic check).
#[test]
fn a17_deck_chords_along_a_pin_row_equal_the_closed_form() {
    use sm_core::{Point3, Vector3};
    use sm_rt_api::{Flow, Scratch, TraceOptions};
    if !corpus_present() {
        return;
    }
    let scale = Scale { rays: 200, ..Scale::reduced() };
    let d = build(Case::A17, &scale).unwrap();
    assert_eq!((d.manifest.rays, d.manifest.regions.len(), d.manifest.triangles), (200, 4, 0));
    let names: Vec<&str> = d.manifest.regions.iter().map(|r| r.name.as_str()).collect();
    assert_eq!(names, ["Borated water", "Helium", "UO2 3.1%", "Zircaloy"]);
    assert_eq!(case_hashes(Case::A17, &scale, 1), case_hashes(Case::A17, &scale, 3));
    // y = -10.71 + 0.63 is the first row of pins, all fuel pins (universe 1). The x range is
    // the deck's own, 21.42, started 1e-7 inside the reflective side and stopped just before the other.
    let a = sm_rt_bench::smrt::analytic_deck(&d.manifest.regions).unwrap();
    let t = a.tracker();
    let (x0, len) = (-10.71 + 1e-7, 21.42 - 2e-7);
    let mut sc = Scratch::new();
    let mut st = t.init_state(Point3::new(x0, -10.71 + 0.63, 0.0), Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    let mut by_name = std::collections::BTreeMap::<String, f64>::new();
    let mut sink = |s: &sm_rt_api::Segment| {
        *by_name.entry(t.regions().get(s.region).map(|r| r.name.clone()).unwrap_or_default()).or_default() += s.length;
        Flow::Continue
    };
    t.trace(&mut st, &TraceOptions::default().with_max_dist(len), &mut sink, &mut sc).unwrap();
    let (rf, rg, rc) = (0.4096, 0.418, 0.475);
    let want = [("UO2 3.1%", 17.0 * 2.0 * rf), ("Helium", 17.0 * 2.0 * (rg - rf)), ("Zircaloy", 17.0 * 2.0 * (rc - rg)), ("Borated water", len - 17.0 * 2.0 * rc)];
    for (name, w) in want {
        let got = by_name.get(name).copied().unwrap_or(0.0);
        assert!((got - w).abs() < 1e-9 * len, "{name}: {got} against {w}");
    }
    assert!((by_name.values().sum::<f64>() - len).abs() < 1e-9 * len);
}
