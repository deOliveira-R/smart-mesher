//! Cross checks of the OpenMC reader: agreement with the model bridge (`csg_world` of
//! `sm_testkit::topo::assembly::pin_assembly`), determinism of whole imports, and invariance of
//! the flat model under renumbering and reordering (property test).
mod common;
use common::*;
use sm_core::{Point3, Vector3};
use sm_csgio::{ImportOptions, ImportedCsg, openmc};
use sm_rt::{AnalyticTracker, CsgWorldOptions, MAX_FLAT_CELLS, csg_world};
use sm_rt_api::{BoundarySpec, BuildOptions, Flow, Scratch, TraceOptions, Tracker};
use sm_testkit::topo::pin_assembly;
use std::collections::BTreeMap;
use std::path::PathBuf;

fn chords_to(t: &AnalyticTracker, o: [f64; 3], d: [f64; 3], max: f64) -> (BTreeMap<String, f64>, f64) {
    let mut sc = Scratch::new();
    let mut st = t.init_state(Point3::new(o[0], o[1], o[2]), Vector3::new(d[0], d[1], d[2]), None, &mut sc).unwrap();
    let mut out: BTreeMap<String, f64> = BTreeMap::new();
    let mut total = 0.0;
    let mut sink = |s: &sm_rt_api::Segment| {
        *out.entry(t.regions().get(s.region).map(|r| r.name.clone()).unwrap_or_default()).or_default() += s.length;
        total += s.length;
        Flow::Continue
    };
    t.trace(&mut st, &TraceOptions::default().with_max_dist(max), &mut sink, &mut sc).unwrap();
    (out, total)
}

struct Rng(u64);
impl Rng {
    fn unit(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }
}

const PITCH: f64 = 1.26;
const RF: f64 = 0.4096;
const RC: f64 = 0.475;
const H: f64 = 3.0;

/// The `pin_assembly(n)` geometry as an OpenMC deck: `3n x 3n` pins at `(i, j) * pitch`, `z` in
/// `[0, H]`, fuel and clad; the space outside the clad is `void`.
fn assembly_xml(n: usize) -> String {
    let side = 3 * n;
    let universes = vec!["1"; side * side].join(" ");
    let (lo, hi) = (-PITCH / 2.0, (side as f64 - 0.5) * PITCH);
    format!(
        r#"<geometry>
  <cell id="1" material="1" region="-1" universe="1"/>
  <cell id="2" material="2" region="1 -2" universe="1"/>
  <cell id="3" material="void" region="2" universe="1"/>
  <cell id="4" fill="9" region="3 -4 5 -6 7 -8" universe="0"/>
  <lattice id="9"><pitch>{PITCH} {PITCH}</pitch><dimension>{side} {side}</dimension><lower_left>{lo} {lo}</lower_left><universes>{universes}</universes></lattice>
  <surface id="1" type="z-cylinder" coeffs="0 0 {RF}"/>
  <surface id="2" type="z-cylinder" coeffs="0 0 {RC}"/>
  <surface id="3" type="x-plane" coeffs="{lo}" boundary="vacuum"/><surface id="4" type="x-plane" coeffs="{hi}" boundary="vacuum"/>
  <surface id="5" type="y-plane" coeffs="{lo}" boundary="vacuum"/><surface id="6" type="y-plane" coeffs="{hi}" boundary="vacuum"/>
  <surface id="7" type="z-plane" coeffs="0" boundary="vacuum"/><surface id="8" type="z-plane" coeffs="{H}" boundary="vacuum"/>
</geometry>"#
    )
}

#[test]
fn the_model_bridge_and_the_openmc_deck_agree_on_the_pin_assembly() {
    for n in [1, 2] {
        bridge_vs_deck(n);
    }
}

fn bridge_vs_deck(n: usize) {
    let fix = pin_assembly(n, PITCH, RF, RC, H).unwrap();
    let w = csg_world(&fix.model, &fix.complex, &CsgWorldOptions::default()).unwrap();
    let bridge = w.analytic_tracker(&BoundarySpec::default(), &BuildOptions::default(), MAX_FLAT_CELLS).unwrap().0;
    let materials = r#"<materials><material id="1" name="fuel"/><material id="2" name="clad"/></materials>"#;
    let ir = openmc::import(&assembly_xml(n), Some(materials)).unwrap();
    let m = ImportedCsg::import(&ir, &ImportOptions::default()).unwrap();
    assert_eq!(m.region_names(), ["clad", "fuel", "void"]);
    let deck = tracker(&m);
    let side = 3 * n;
    let mut r = Rng(0x1234_5678_9ABC_DEF1);
    let delta = 1e-3;
    let (mut worst, mut compared) = (0.0f64, 0);
    for k in 0..10_000 {
        let (i, j) = ((r.unit() * side as f64).floor().min(side as f64 - 1.0), (r.unit() * side as f64).floor().min(side as f64 - 1.0));
        let b = (2.0 * r.unit() - 1.0) * 0.999 * RC;
        let th = std::f64::consts::TAU * r.unit();
        let raw = [th.cos(), th.sin(), 0.6 * r.unit() - 0.3];
        let nrm = (raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]).sqrt();
        let d = [raw[0] / nrm, raw[1] / nrm, raw[2] / nrm];
        let rho = d[0].hypot(d[1]);
        let nv = (-d[1] / rho, d[0] / rho);
        let p0 = [i * PITCH + b * nv.0, j * PITCH + b * nv.1, 0.8 + 1.4 * r.unit()];
        let half_clad = (RC * RC - b * b).sqrt() / rho;
        let o = [p0[0] - d[0] * (half_clad - delta), p0[1] - d[1] * (half_clad - delta), p0[2] - d[2] * (half_clad - delta)];
        let (want, total) = chords_to(&bridge, o, d, f64::INFINITY);
        let (got, _) = chords_to(&deck, o, d, total);
        for name in ["fuel", "clad"] {
            let (a, c) = (want.get(name).copied().unwrap_or(0.0), got.get(name).copied().unwrap_or(0.0));
            worst = worst.max((a - c).abs());
            assert!((a - c).abs() < 1e-11 * a.abs().max(1.0), "ray {k} {o:?} {d:?}: {name} bridge {a}, deck {c}");
        }
        compared += 1;
    }
    assert_eq!(compared, 10_000);
    eprintln!("n={n}: worst chord difference {worst:e}");
}

fn in_pool<R: Send>(threads: usize, f: impl FnOnce() -> R + Send) -> R {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

fn fixture(name: &str, f: &str) -> String {
    std::fs::read_to_string(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/openmc").join(name).join(f)).unwrap()
}

/// Hash, warnings and a chord table of a fixture import, as comparable bits.
fn fingerprint(name: &str) -> (u128, Vec<String>, Vec<u64>) {
    let ir = openmc::import(&fixture(name, "geometry.xml"), Some(&fixture(name, "materials.xml"))).unwrap();
    // The smallest check the importer allows keeps four imports per model quick.
    let quick = ImportOptions { check: sm_csgio::CheckOptions { samples: sm_csgio::MIN_SAMPLES, rays: sm_csgio::MIN_RAYS, seed: 0 }, ..ImportOptions::default() };
    let m = ImportedCsg::import(&ir, &quick).unwrap();
    let t = tracker(&m);
    let mut table = vec![m.surface_count() as u64];
    // The oracle's rays (origin, direction, length) are a convenient fixed set.
    let o: serde_json::Value = serde_json::from_str(&fixture(name, "oracle.json")).unwrap();
    let v3 = |v: &serde_json::Value| -> [f64; 3] { std::array::from_fn(|i| v[i].as_f64().unwrap()) };
    for c in o["chords"].as_array().unwrap() {
        let total = c["segments"].as_array().unwrap().last().map_or(0.0, |s| s["t1"].as_f64().unwrap());
        for (_, l) in chords_to(&t, v3(&c["o"]), v3(&c["d"]), total).0 {
            table.push(l.to_bits());
        }
    }
    let warnings = m.report().warnings.iter().map(|w| format!("{} {:?}", w.code, w.point.map(|p| p.map(f64::to_bits)))).collect();
    (m.content_hash().0, warnings, table)
}

#[test]
fn fixture_imports_are_identical_across_runs_and_thread_counts() {
    for name in ["deck_o", "pin", "lattice2d", "lattice3d", "rotations", "nested", "lattice_xf", "surfaces", "grammar", "translated"] {
        let a = fingerprint(name);
        assert_eq!(a, fingerprint(name), "{name}: two runs");
        assert_eq!(a, in_pool(1, || fingerprint(name)), "{name}: one thread");
        assert_eq!(a, in_pool(4, || fingerprint(name)), "{name}: four threads");
    }
}

mod property {
    use super::*;
    use proptest::prelude::*;
    use sm_testkit::strategies::config;

    /// A random nested deck: `k` concentric z-cylinders in a box, one cell per ring, with surface
    /// ids and the text order of cells permuted.
    fn deck(radii: &[f64], perm: &[usize], shift: u32, rot: usize) -> String {
        let k = radii.len();
        let sid = |s: usize| 100 + shift + perm[s] as u32 * 7; // surface ids of the k cylinders
        let boxes = [200 + shift, 201 + shift, 202 + shift, 203 + shift, 204 + shift, 205 + shift];
        let mut cells = Vec::new();
        for i in 0..=k {
            let region = if i == 0 {
                format!("-{}", sid(0))
            } else if i < k {
                format!("{} -{}", sid(i - 1), sid(i))
            } else {
                format!("{}", sid(k - 1))
            };
            let region = format!("{region} {} -{} {} -{} {} -{}", boxes[0], boxes[1], boxes[2], boxes[3], boxes[4], boxes[5]);
            cells.push(format!(r#"<cell id="{}" material="{}" region="{region}" universe="0"/>"#, 10 + i as u32 * 3 + shift, 1 + i % 3));
        }
        let nc = cells.len();
        cells.rotate_left(rot % nc);
        let mut surf = Vec::new();
        for (s, r) in radii.iter().enumerate() {
            surf.push(format!(r#"<surface id="{}" type="z-cylinder" coeffs="0 0 {r}"/>"#, sid(s)));
        }
        for (b, (ty, c)) in ["x-plane", "x-plane", "y-plane", "y-plane", "z-plane", "z-plane"].iter().zip([-3.0, 3.0, -3.0, 3.0, -1.0, 1.0]).enumerate() {
            surf.push(format!(r#"<surface id="{}" type="{ty}" coeffs="{c}" boundary="vacuum"/>"#, boxes[b]));
        }
        let ns = surf.len();
        surf.rotate_left(rot % ns);
        format!("<geometry>{}{}</geometry>", cells.join(""), surf.join(""))
    }

    proptest! {
        #![proptest_config(config(24))]
        #[test]
        fn renumbering_and_reordering_keep_the_hash_and_the_chords(
            raw in proptest::collection::vec(0.1f64..2.5, 1..5),
            perm_seed in 0u64..1000,
            shift in 0u32..50,
            rot in 0usize..9,
        ) {
            let mut radii = raw;
            radii.sort_by(f64::total_cmp);
            radii.dedup_by(|a, b| (*a - *b).abs() < 0.05);
            let k = radii.len();
            let ident: Vec<usize> = (0..k).collect();
            let mut perm = ident.clone();
            // deterministic shuffle from the seed
            let mut s = perm_seed + 1;
            for i in (1..k).rev() {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                perm.swap(i, (s >> 33) as usize % (i + 1));
            }
            let a = deck(&radii, &ident, 0, 0);
            let b = deck(&radii, &perm, shift, rot);
            let ma = ImportedCsg::import(&openmc::import(&a, None).unwrap(), &ImportOptions::default()).unwrap();
            let mb = ImportedCsg::import(&openmc::import(&b, None).unwrap(), &ImportOptions::default()).unwrap();
            prop_assert_eq!(ma.content_hash(), mb.content_hash());
            let (ta, tb) = (tracker(&ma), tracker(&mb));
            for y in [-2.2, -0.35, 0.05, 0.9, 2.4] {
                let ca = chords(&ta, [-3.0, y, 0.1], [1.0, 0.0, 0.0]);
                let cb = chords(&tb, [-3.0, y, 0.1], [1.0, 0.0, 0.0]);
                prop_assert_eq!(ca.len(), cb.len());
                for (k, v) in &ca {
                    prop_assert!((v - cb[k]).abs() < 1e-12);
                }
            }
        }
    }
}
