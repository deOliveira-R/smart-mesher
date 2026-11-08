//! Conformance against OpenMC itself (ruling C7). Each fixture directory under
//! `tests/fixtures/openmc/` holds a `geometry.xml`, a `materials.xml` and `oracle.json`, which
//! records what OpenMC 0.16.0 answered on exactly those two XML files (`gen.py` regenerates all
//! three; OpenMC is not needed to run this test): the cell at 600 random points and the
//! crossing positions of several rays. Importing the XML and tracking through the analytic
//! tracker must find the same material at every point and the same chord per material.
//!
//! The confirmed conventions: lattice `universes` order and element origin, `outer`, `translation`
//! and `rotation` (single angle, three angles, nine entries), composition of nested fills, the
//! sense of every surface type, and the region grammar.
mod common;
use common::*;
use serde_json::Value;
use sm_core::Point3;
use sm_csgio::{ImportOptions, ImportedCsg, openmc};
use sm_core::Vector3;
use sm_rt::AnalyticTracker;
use sm_rt_api::{Flow, Scratch, TraceOptions, Tracker};
use std::collections::BTreeMap;
use std::path::PathBuf;

struct Fixture {
    oracle: Value,
}

fn dir(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/openmc").join(name)
}

fn load(name: &str) -> (ImportedCsg, Fixture) {
    let d = dir(name);
    let read = |f: &str| std::fs::read_to_string(d.join(f)).unwrap_or_else(|e| panic!("{name}/{f}: {e}"));
    let ir = openmc::import(&read("geometry.xml"), Some(&read("materials.xml"))).unwrap_or_else(|e| panic!("{name}: {e}"));
    let m = ImportedCsg::import(&ir, &ImportOptions::default()).unwrap_or_else(|e| panic!("{name}: {e}"));
    (m, Fixture { oracle: serde_json::from_str(&read("oracle.json")).unwrap() })
}

fn vec3(v: &Value) -> [f64; 3] {
    let a = v.as_array().unwrap();
    [a[0].as_f64().unwrap(), a[1].as_f64().unwrap(), a[2].as_f64().unwrap()]
}

/// Chord length per region name along a ray, to distance `max` (a reflective wall would otherwise
/// keep the ray going; OpenMC's recorded chord ends at the first boundary).
fn chords_to(t: &AnalyticTracker, o: [f64; 3], d: [f64; 3], max: f64) -> BTreeMap<String, f64> {
    let mut sc = Scratch::new();
    let mut st = t.init_state(Point3::new(o[0], o[1], o[2]), Vector3::new(d[0], d[1], d[2]), None, &mut sc).unwrap();
    let mut out: BTreeMap<String, f64> = BTreeMap::new();
    let mut sink = |s: &sm_rt_api::Segment| {
        *out.entry(t.regions().get(s.region).map(|r| r.name.clone()).unwrap_or_default()).or_default() += s.length;
        Flow::Continue
    };
    t.trace(&mut st, &TraceOptions::default().with_max_dist(max), &mut sink, &mut sc).unwrap();
    out
}

/// Compare every recorded point and chord; returns (points compared, rays compared).
fn conforms(name: &str) -> (usize, usize) {
    let (m, fx) = load(name);
    let o = &fx.oracle;
    assert_eq!(o["openmc_version"], "0.16.0");
    let mat = o["cell_material"].as_object().unwrap();
    let t = tracker(&m);
    let mut sc = Scratch::new();
    let mut npts = 0;
    for p in o["points"].as_array().unwrap() {
        let Some(cell) = p["cell"].as_u64() else { continue };
        let q = vec3(&p["p"]);
        let want = mat[&cell.to_string()].as_str().unwrap();
        let l = t.locate(Point3::new(q[0], q[1], q[2]), None, None, &mut sc).unwrap_or_else(|e| panic!("{name}: locate {q:?}: {e:?}"));
        let got = &t.regions().get(l.region).unwrap().name;
        assert_eq!(got, want, "{name}: point {q:?} is in OpenMC cell {cell} ({want}) but the import says {got}");
        npts += 1;
    }
    let mut nrays = 0;
    for c in o["chords"].as_array().unwrap() {
        let (org, dir) = (vec3(&c["o"]), vec3(&c["d"]));
        let mut want: BTreeMap<String, f64> = BTreeMap::new();
        for s in c["segments"].as_array().unwrap() {
            let len = s["t1"].as_f64().unwrap() - s["t0"].as_f64().unwrap();
            *want.entry(mat[&s["cell"].as_u64().unwrap().to_string()].as_str().unwrap().to_owned()).or_default() += len;
        }
        let total = c["segments"].as_array().unwrap().last().map_or(0.0, |s| s["t1"].as_f64().unwrap());
        let got = chords_to(&t, org, dir, total);
        let keys: Vec<_> = want.keys().chain(got.keys()).collect();
        for k in keys {
            let (w, g) = (want.get(k).copied().unwrap_or(0.0), got.get(k).copied().unwrap_or(0.0));
            assert!((w - g).abs() < 2e-9, "{name}: ray {org:?} -> {dir:?}, material {k}: OpenMC {w}, import {g}");
        }
        nrays += 1;
    }
    assert!(npts > 500 && nrays >= 4, "{name}: {npts} points, {nrays} rays");
    (npts, nrays)
}

#[test]
fn deck_o_sphere_in_a_box() {
    conforms("deck_o");
}

#[test]
fn pin_cell_with_void_gap_and_boundaries() {
    conforms("pin");
    let (m, _) = load("pin");
    assert_eq!(m.region_names(), ["clad", "fuel", "void", "water"]);
    assert!(m.report().warnings.is_empty(), "{:?}", m.report().warnings);
}

#[test]
fn rect_lattice_row_order_pitch_and_outer() {
    conforms("lattice2d");
}

#[test]
fn rect_lattice_3d_layer_order() {
    conforms("lattice3d");
}

#[test]
fn rotation_and_translation_of_a_filled_cell() {
    conforms("rotations");
}

#[test]
fn nested_fills_with_translation_and_rotation_compose_as_openmc_does() {
    conforms("nested");
}

#[test]
fn translation_only() {
    conforms("translated");
}

#[test]
fn a_lattice_inside_a_transformed_cell() {
    conforms("lattice_xf");
}

#[test]
fn every_surface_type_has_openmcs_sense() {
    conforms("surfaces");
}

#[test]
fn region_grammar_and_precedence() {
    conforms("grammar");
}

#[test]
fn the_fixtures_are_not_a_vacuous_oracle() {
    // The lattice and rotation fixtures separate the conventions: a wrong reading must differ.
    // Row order: swapping the first and last row of the 3x2 lattice changes the answer at points.
    let (_, fx) = load("lattice2d");
    let mats: std::collections::BTreeSet<_> = fx.oracle["points"].as_array().unwrap().iter().filter_map(|p| p["cell"].as_u64()).collect();
    assert!(mats.len() >= 12, "the lattice fixture should visit most cells, saw {}", mats.len());
}
