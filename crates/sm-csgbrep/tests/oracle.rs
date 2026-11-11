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

/// Import `geometry` (materials of the fixture) and compare with the recorded oracle of `name`.
fn compare(name: &str, geometry: &str) -> Result<Tally, String> {
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
        let f = locate_all(&r, &volumes, &bb, q, false);
        if i % 25 == 0 {
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
    ("ref_mini3x3_h2", 590),
    ("ref_assembly17_h2", 590),
];

#[test]
fn membership_at_the_600_oracle_points_equals_openmc() {
    for &(name, lattice_floor) in IMPORTING {
        let t = compare(name, &fixture(name, "geometry.xml")).unwrap_or_else(|e| panic!("{name}: {e}"));
        eprintln!("{name}: {} points, {} outside, {} undecided, {} lattice checks, {} contested", t.points, t.outside, t.undecided, t.lattice_checked, t.contested);
        assert!(t.disagreements.is_empty(), "{name}: {} disagreements, first: {:?}", t.disagreements.len(), t.disagreements.iter().take(3).collect::<Vec<_>>());
        assert!(t.points >= 595 && t.undecided <= 5, "{name}: {} decided, {} undecided", t.points, t.undecided);
        assert_eq!(t.contested, 0, "{name}: points inside several volumes");
        assert!(t.lattice_checked >= lattice_floor, "{name}: {} lattice checks", t.lattice_checked);
    }
}

/// The W4-REFCORPUS decks as the corpus writes them (20 cm tall pins, the 460 cm BEAVRS assembly)
/// are refused by the kernel today with a typed `csg.kernel` error naming a deck surface: the
/// `_h2` variants above carry the same decks at height 2. When a kernel fix lets one import, this
/// test fails and the fixture moves to `IMPORTING`.
#[test]
fn the_tall_corpus_decks_are_refused_typed_by_the_kernel() {
    for name in ["ref_pin", "ref_mini3x3", "ref_assembly17", "ref_beavrs_assembly"] {
        let e = compare(name, &fixture(name, "geometry.xml")).err().unwrap_or_else(|| panic!("{name} imports now: move it to IMPORTING"));
        assert!(e.contains("csg.kernel") && e.contains("deck surface"), "{name}: {e}");
    }
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
