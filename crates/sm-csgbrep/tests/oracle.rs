//! CSGBREP-ORACLE layer 1: the B-rep against OpenMC itself (ruling C7). Every fixture under
//! `crates/sm-csgio/tests/fixtures/openmc/` carries `oracle.json`, which `gen.py` recorded from
//! `openmc.lib.find_cell` on exactly its two XML files (OpenMC is not needed to run this test).
//! The deck is imported with `model.import_openmc` and the volume that contains each of the 600
//! recorded points (`sm_boolean::locate_point`, the exact point-in-B-rep test) must carry the
//! material OpenMC found there. For the lattice decks the lattice element of OpenMC's own cell path
//! must equal the region tag of that volume (ruling D9).
mod common;

use std::collections::BTreeMap;

use common::*;
use serde_json::Value;
use sm_boolean::{PointLocation, locate_point};
use sm_core::{EntityId, Point3};
use sm_model_api::{Domain, SlotIx};

/// The oracle of fixture `name`.
fn oracle(name: &str) -> Value {
    serde_json::from_str(&fixture(name, "oracle.json")).unwrap_or_else(|e| panic!("{name}/oracle.json: {e}"))
}

fn vec3(v: &Value) -> [f64; 3] {
    let a = v.as_array().expect("array");
    [a[0].as_f64().expect("x"), a[1].as_f64().expect("y"), a[2].as_f64().expect("z")]
}

/// Block (material) name of every imported volume.
fn block_of(r: &Rig, volumes: &[EntityId]) -> BTreeMap<EntityId, String> {
    let m = r.model();
    let mut out = BTreeMap::new();
    for c in m.colors().filter(|c| c.domain == Domain::Volume) {
        for v in m.volumes_of_color(c.id) {
            if volumes.contains(&v) {
                out.insert(v, c.name.clone());
            }
        }
    }
    out
}

/// What the B-rep says at `p`: the volumes that contain it (strictly), and whether it was on a
/// boundary or ambiguous (then the point is not decided).
struct Found {
    inside: Vec<EntityId>,
    undecided: bool,
}

/// `model.measure` bounding boxes of the volumes, used only to skip volumes far from a point (every
/// 25th point is also checked against all volumes, so the filter cannot hide a containing one).
fn boxes(r: &Rig, volumes: &[EntityId]) -> Vec<([f64; 3], [f64; 3])> {
    let q = sm_op_api::QuietControl::new();
    volumes
        .iter()
        .map(|v| {
            let out = r.s.engine().run_query(r.s.state(), "model.measure", &serde_json::json!({ "entity": v.to_string() }), &q.control()).expect("model.measure");
            (vec3(&out["bbox"]["lo"]), vec3(&out["bbox"]["hi"]))
        })
        .collect()
}

fn locate_in(r: &Rig, volumes: &[EntityId], q: Point3) -> Found {
    let mut f = Found { inside: Vec::new(), undecided: false };
    for &v in volumes {
        match locate_point(r.topo(), v, q) {
            Ok(PointLocation::In) => f.inside.push(v),
            Ok(PointLocation::Out) => {}
            Ok(PointLocation::On) | Err(_) => f.undecided = true,
        }
    }
    f
}

fn locate_all(r: &Rig, volumes: &[EntityId], bb: &[([f64; 3], [f64; 3])], p: [f64; 3], full: bool) -> Found {
    let q = Point3::new(p[0], p[1], p[2]);
    let near: Vec<EntityId> = volumes.iter().zip(bb).filter(|(_, (lo, hi))| full || (0..3).all(|k| p[k] >= lo[k] - 1e-3 && p[k] <= hi[k] + 1e-3)).map(|(v, _)| *v).collect();
    locate_in(r, &near, q)
}

/// Totals of one membership run.
#[derive(Debug, Default)]
struct Tally {
    /// Points decided (inside some volume or in none).
    points: usize,
    /// Of those, the points OpenMC puts in no cell.
    outside: usize,
    /// Points on a boundary or ambiguous: not compared.
    undecided: usize,
    /// One line per point where the B-rep and OpenMC differ.
    disagreements: Vec<String>,
    /// Lattice-tag comparisons made.
    lattice_checked: usize,
    /// Points that several volumes claim.
    contested: usize,
}

/// Import `geometry` (materials of the fixture) and compare with the recorded oracle of `name`,
/// with the bounding-box prefilter.
fn compare(name: &str, geometry: &str) -> Result<Tally, String> {
    compare_ex(name, geometry, true)
}

/// As [`compare`]; without `prefilter` every point is located against every volume.
fn compare_ex(name: &str, geometry: &str, prefilter: bool) -> Result<Tally, String> {
    let mut r = Rig::new();
    let d = r.import(geometry, Some(&fixture(name, "materials.xml"))).map_err(|e| e.to_string())?;
    let volumes = d.outputs["deck_volumes"].clone();
    let blocks = block_of(&r, &volumes);
    let o = oracle(name);
    let mat = o["cell_material"].as_object().expect("cell_material");
    let bb = boxes(&r, &volumes);
    let mut t = Tally::default();
    for (i, p) in o["points"].as_array().expect("points").iter().enumerate() {
        let q = vec3(&p["p"]);
        let f = locate_all(&r, &volumes, &bb, q, !prefilter);
        if prefilter && i % 25 == 0 {
            let all = locate_all(&r, &volumes, &bb, q, true);
            assert_eq!((&all.inside, all.undecided), (&f.inside, f.undecided), "{name}: the bounding-box filter changed the answer at {q:?}");
        }
        if f.inside.len() > 1 {
            t.contested += 1;
        }
        if f.undecided {
            t.undecided += 1;
            continue;
        }
        t.points += 1;
        match p["cell"].as_u64() {
            None => {
                t.outside += 1;
                if !f.inside.is_empty() {
                    t.disagreements.push(format!("{q:?}: OpenMC finds no cell, the B-rep has {:?}", f.inside));
                }
            }
            Some(cell) => {
                let want = mat[&cell.to_string()].as_str().expect("material name");
                match f.inside.as_slice() {
                    [v] => {
                        if blocks[v] != want {
                            t.disagreements.push(format!("{q:?}: OpenMC cell {cell} is {want}, the B-rep volume {v} is {}", blocks[v]));
                        }
                        if let Some(lats) = p.get("lattices").and_then(Value::as_array) {
                            for l in lats {
                                let l: Vec<i64> = l.as_array().expect("lattice entry").iter().map(|x| x.as_i64().expect("int")).collect();
                                let frame = format!("lat{}", l[0]);
                                if let Some(slot) = r.tag(*v).and_then(|tag| tag.slots.get(&frame)) {
                                    t.lattice_checked += 1;
                                    let want = SlotIx::new(l[1] as i32, l[2] as i32, l[3] as i32);
                                    if *slot != want {
                                        t.disagreements.push(format!("{q:?}: OpenMC element {want:?} of {frame}, the volume {v} is tagged {slot:?}"));
                                    }
                                }
                            }
                        }
                    }
                    other => t.disagreements.push(format!("{q:?}: OpenMC cell {cell} ({want}) but {} B-rep volumes contain the point", other.len())),
                }
            }
        }
    }
    Ok(t)
}

/// Fixtures that import today, with a floor on the lattice-tag checks (OpenMC's own lattice path
/// of the point against the region tag of the volume that contains it).
const IMPORTING: &[(&str, usize)] = &[
    ("pin", 0),
    ("lattice2d", 0),
    ("lattice_xf", 0),
    ("quadric_cyl", 0),
    ("disconnected", 0),
    ("placed_z", 0),
    ("ref_pin_h2", 0),
    ("ref_pin", 0),
    ("ref_mini3x3_h2", 590),
    ("ref_assembly17_h2", 590),
    ("ref_mini3x3", 0),
    ("ref_assembly17", 590),
];

#[test]
fn membership_at_the_600_oracle_points_equals_openmc() {
    // Every deck is run and judged, so one failure does not hide the others.
    let mut problems = Vec::new();
    for &(name, lattice_floor) in IMPORTING {
        let t0 = std::time::Instant::now();
        let t = match compare(name, &fixture(name, "geometry.xml")) {
            Ok(t) => t,
            Err(e) => {
                problems.push(format!("{name}: {e}"));
                continue;
            }
        };
        eprintln!("{name}: {:.1}s, {} points, {} outside, {} undecided, {} lattice checks, {} contested", t0.elapsed().as_secs_f64(), t.points, t.outside, t.undecided, t.lattice_checked, t.contested);
        if !t.disagreements.is_empty() {
            problems.push(format!("{name}: {} disagreements, first: {:?}", t.disagreements.len(), t.disagreements.iter().take(3).collect::<Vec<_>>()));
        }
        if !(t.points >= 595 && t.undecided <= 5) {
            problems.push(format!("{name}: {} decided, {} undecided", t.points, t.undecided));
        }
        if t.contested != 0 {
            problems.push(format!("{name}: {} points inside several volumes", t.contested));
        }
        if t.lattice_checked < lattice_floor {
            problems.push(format!("{name}: {} lattice checks", t.lattice_checked));
        }
    }
    assert!(problems.is_empty(), "{problems:#?}");
}

/// Without the bounding-box prefilter (`locate_point` against every volume), the 600 oracle
/// points of `ref_mini3x3_h2` have `CONTESTED` points that two volumes claim. Before W4-BOOL-KFIX
/// the direction dependent false `In` of `sm_boolean::locate_point` on the thin annulus gave 3;
/// the kernel fix made it 0.
const CONTESTED: usize = 0;

#[test]
fn locate_point_without_the_prefilter_has_no_contested_points_on_mini3x3_h2() {
    let t = compare_ex("ref_mini3x3_h2", &fixture("ref_mini3x3_h2", "geometry.xml"), false).unwrap_or_else(|e| panic!("{e}"));
    eprintln!("ref_mini3x3_h2 unfiltered: {} points, {} contested, {} disagreements: {:?}", t.points, t.contested, t.disagreements.len(), t.disagreements);
    assert_eq!(t.contested, CONTESTED, "{} contested, disagreements {:?}", t.contested, t.disagreements);
}

/// The partition probe of the false `In`: 72 angles x 7 radii x 3 heights around each of the nine
/// pin centres of `ref_mini3x3_h2` (13608 points, none on a deck surface), each located against
/// every volume. A point in two volumes is a `locate_point` false `In` (15 points before
/// W4-BOOL-KFIX, for example Borated water claiming UO2 at r = 0.2). Wanted, and now true: none.
#[test]
fn no_point_around_the_mini3x3_h2_pins_is_in_two_volumes() {
    let name = "ref_mini3x3_h2";
    let mut r = Rig::new();
    let d = r.import(&fixture(name, "geometry.xml"), Some(&fixture(name, "materials.xml"))).expect("import");
    let volumes = d.outputs["deck_volumes"].clone();
    let bb = boxes(&r, &volumes);
    let mut contested = Vec::new();
    for (cx, cy) in (-1..=1).flat_map(|i| (-1..=1).map(move |j| (1.26 * f64::from(i), 1.26 * f64::from(j)))) {
        for a in 0..72 {
            let th = (f64::from(a) + 0.5) * core::f64::consts::TAU / 72.0;
            for rad in [0.2, 0.405, 0.414, 0.45, 0.5, 0.58, 0.618] {
                for z in [-0.6, 0.0, 0.6] {
                    let p = [cx + rad * th.cos(), cy + rad * th.sin(), z];
                    let f = locate_all(&r, &volumes, &bb, p, true);
                    if f.inside.len() > 1 {
                        contested.push((p, f.inside));
                    }
                }
            }
        }
    }
    eprintln!("{} of 13608 points are in two volumes", contested.len());
    assert!(contested.is_empty(), "{} points in two volumes, first {:?}", contested.len(), contested.iter().take(3).collect::<Vec<_>>());
}

/// The 460 cm BEAVRS assembly imports (W4-BOOL-KFIX fixed the unsplit grid-layer pieces; the
/// import's piece check stays as a guard): the Fuel 1.6% volumes total the analytic
/// 264 pi 0.39218^2 (402.508 - 36.748) = 46657.4 cm3, and the 600 oracle points equal OpenMC's.
/// The grid spacer boxes with their pin holes are checked on one pin cell in
/// `beavrs_repro.rs` (exact fuel and end-grid Inconel).
#[test]
#[ignore = "slow: about 940 s in debug, 69 s in release; run with --ignored"]
fn the_beavrs_assembly_imports_with_the_analytic_fuel_and_agrees_with_openmc() {
    let name = "ref_beavrs_assembly";
    let mut r = Rig::new();
    let d = r.import(&fixture(name, "geometry.xml"), Some(&fixture(name, "materials.xml"))).unwrap_or_else(|e| panic!("{name}: {e}"));
    let mut fuel_got = 0.0;
    for c in r.model().colors().filter(|c| c.domain == Domain::Volume && c.name == "Fuel 1.6%") {
        for v in r.model().volumes_of_color(c.id) {
            if d.outputs["deck_volumes"].contains(&v) {
                fuel_got += r.volume(v);
            }
        }
    }
    let fuel = 264.0 * core::f64::consts::PI * 0.39218_f64.powi(2) * 365.76;
    assert!((fuel_got - fuel).abs() < 1e-6 * fuel, "Fuel 1.6%: {fuel_got} vs analytic {fuel} ({:.1}%)", 100.0 * (fuel_got - fuel) / fuel);
    let t = compare(name, &fixture(name, "geometry.xml")).unwrap_or_else(|e| panic!("{name}: {e}"));
    assert!(t.points >= 595 && t.undecided <= 5 && t.contested == 0, "{name}: {} decided, {} undecided, {} contested", t.points, t.undecided, t.contested);
    assert!(t.disagreements.is_empty(), "{name}: {} disagreements, first {:?}", t.disagreements.len(), t.disagreements.iter().take(3).collect::<Vec<_>>());
    assert!(t.lattice_checked >= 590, "{name}: {} lattice checks", t.lattice_checked);
}

/// The text between `<universes>` and `</universes>`, mapped by `f` on its rows.
fn map_rows(g: &str, f: impl Fn(Vec<&str>) -> Vec<String>) -> String {
    let (a, b) = g.split_once("<universes>").expect("universes");
    let (rows, c) = b.split_once("</universes>").expect("universes end");
    let lines = f(rows.trim().lines().collect());
    format!("{a}<universes>\n{}\n</universes>{c}", lines.join("\n"))
}

fn swap_rows(g: &str) -> String {
    map_rows(g, |mut l| {
        let last = l.len() - 1;
        l.swap(0, last);
        l.into_iter().map(String::from).collect()
    })
}

fn reverse_columns(g: &str) -> String {
    map_rows(g, |l| l.into_iter().map(|r| r.split_whitespace().rev().collect::<Vec<_>>().join(" ")).collect())
}

/// Mutating the lattice row or column order or the rotation sense in the deck (what a reader bug
/// amounts to) must contradict OpenMC at some of the 600 points, in at least five cases: the
/// oracle is not vacuous (as `the_fixtures_are_not_a_vacuous_oracle` of sm-csgio).
#[test]
fn the_fixtures_are_not_a_vacuous_oracle() {
    type Edit = fn(&str) -> String;
    let cases: [(&str, &str, Edit); 6] = [
        ("lattice2d", "rows swapped", swap_rows),
        ("lattice2d", "columns reversed", reverse_columns),
        ("lattice_xf", "rows swapped", swap_rows),
        ("lattice_xf", "columns reversed", reverse_columns),
        ("lattice_xf", "rotation transposed (angle negated)", |g| g.replace("rotation=\"0 0 30\"", "rotation=\"0 0 -30\"")),
        ("placed_z", "rotation transposed (angle negated)", |g| g.replace("rotation=\"0 0 35\"", "rotation=\"0 0 -35\"")),
    ];
    let mut seen = 0;
    for (name, what, edit) in cases {
        let g = fixture(name, "geometry.xml");
        let edited = edit(&g);
        assert_ne!(edited, g, "{name}: `{what}` changed nothing");
        let t = compare(name, &edited).unwrap_or_else(|e| panic!("{name} with {what} no longer imports: {e}"));
        eprintln!("{name}, {what}: {} points contradict OpenMC", t.disagreements.len());
        assert!(!t.disagreements.is_empty(), "{name}, {what}: agrees with OpenMC everywhere, the oracle cannot see it");
        seen += 1;
    }
    assert!(seen >= 5);
}
