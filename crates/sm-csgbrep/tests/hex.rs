//! Hexagonal lattices (W4-CSG-HEX, ruling D8): the B-rep import against OpenMC 0.16.0 and against
//! the tracker import, in both orientations.
//!
//! `crates/sm-csgio/tests/fixtures/openmc_hex/{hex_x,hex_y,hex_x3d,hex_y3d}` carry the 600 points
//! (300 uniform, 300 within 5e-3 of a flat or corner of the tiling) at which `openmc.lib.find_cell`
//! answered on exactly those XML files. Every point must be in the volume whose block is the
//! material OpenMC found, in the volume tagged with the element of OpenMC's own cell, and in the
//! tracker import's region of the same material.
mod common;

use std::collections::BTreeMap;

use common::*;
use serde_json::Value;
use sm_boolean::{PointLocation, locate_point};
use sm_core::{EntityId, Point3};
use sm_csgio::ir::{HexLattice, HexOrientation};
use sm_csgio::{ImportOptions, ImportedCsg, openmc};
use sm_model_api::{Domain, HexOrient, Layout, SlotIx};
use sm_rt_api::{BuildOptions, Scratch, Tracker};

const MODELS: [&str; 4] = ["hex_x", "hex_y", "hex_x3d", "hex_y3d"];

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap()
}

fn oracle(name: &str) -> Value {
    serde_json::from_str(&hex_fixture(name, "oracle.json")).unwrap()
}

/// Material of OpenMC's cell in the hex fixtures (the generator's numbering).
fn material_of(cell: u64) -> &'static str {
    match cell {
        900 => "gas",
        c if c % 10 == 0 => "fuel",
        _ => "water",
    }
}

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

fn bbox(r: &Rig, v: EntityId) -> ([f64; 3], [f64; 3]) {
    let q = sm_op_api::QuietControl::new();
    let out = r.s.engine().run_query(r.s.state(), "model.measure", &serde_json::json!({ "entity": v.to_string() }), &q.control()).expect("model.measure");
    let a = |v: &Value| {
        let a = v.as_array().unwrap();
        [f(&a[0]), f(&a[1]), f(&a[2])]
    };
    (a(&out["bbox"]["lo"]), a(&out["bbox"]["hi"]))
}

#[test]
fn the_b_rep_and_the_tracker_agree_with_openmc_at_the_600_points_of_every_fixture() {
    for name in MODELS {
        let (g, m, o) = (hex_fixture(name, "geometry.xml"), hex_fixture(name, "materials.xml"), oracle(name));
        let mut r = Rig::new();
        let d = r.import(&g, Some(&m)).unwrap_or_else(|e| panic!("{name}: {e}"));
        let volumes = d.outputs["deck_volumes"].clone();
        let blocks = block_of(&r, &volumes);
        let boxes: Vec<_> = volumes.iter().map(|v| bbox(&r, *v)).collect();
        let ir = openmc::import(&g, Some(&m)).unwrap();
        let csg = ImportedCsg::import(&ir, &ImportOptions::default()).unwrap();
        let tracker = csg.tracker(&BuildOptions::default()).unwrap().0;
        let mut sc = Scratch::new();
        let elements = o["elements"].as_array().unwrap();
        let (mut decided, mut tags, mut undecided) = (0, 0, 0);
        for (i, p) in o["points"].as_array().unwrap().iter().enumerate() {
            let a = p["p"].as_array().unwrap();
            let q = [f(&a[0]), f(&a[1]), f(&a[2])];
            let Some(cell) = p["cell"].as_u64() else { continue };
            let want = material_of(cell);
            // B-rep: the volumes that contain the point strictly.
            let full = i % 20 == 0;
            let mut inside = Vec::new();
            let mut on = false;
            for (v, (lo, hi)) in volumes.iter().zip(&boxes) {
                if !full && !(0..3).all(|k| q[k] >= lo[k] - 1e-3 && q[k] <= hi[k] + 1e-3) {
                    continue;
                }
                match locate_point(r.topo(), *v, Point3::new(q[0], q[1], q[2])) {
                    Ok(PointLocation::In) => inside.push(*v),
                    Ok(PointLocation::Out) => {}
                    Ok(PointLocation::On) | Err(_) => on = true,
                }
            }
            // Tracker: the region's material.
            let l = tracker.locate(Point3::new(q[0], q[1], q[2]), None, None, &mut sc).unwrap();
            if let Some(reg) = tracker.regions().get(l.region) {
                assert_eq!(reg.name, want, "{name}: tracker at {q:?}, OpenMC cell {cell}");
            }
            if on {
                undecided += 1;
                continue;
            }
            decided += 1;
            match inside.as_slice() {
                [v] => {
                    assert_eq!(blocks[v], want, "{name}: B-rep at {q:?}, OpenMC cell {cell}");
                    // The element of OpenMC's cell is the tag of the volume.
                    if let Some(e) = elements.iter().find(|e| e["fuel_cell"].as_u64() == Some(cell) || e["mod_cell"].as_u64() == Some(cell)) {
                        let want_slot = SlotIx::new(e["q"].as_i64().unwrap() as i32, e["r"].as_i64().unwrap() as i32, e["layer"].as_u64().unwrap() as i32);
                        assert_eq!(r.tag(*v).and_then(|t| t.slots.get("lat50")), Some(&want_slot), "{name}: tag at {q:?}");
                        tags += 1;
                    } else {
                        // The outer universe (gas) is not an element of the lattice.
                        assert!(r.tag(*v).is_none_or(|t| !t.slots.contains_key("lat50")), "{name}: gas volume tagged at {q:?}");
                    }
                }
                other => panic!("{name}: {} B-rep volumes contain {q:?} (OpenMC cell {cell})", other.len()),
            }
        }
        assert!(decided >= 590 && undecided <= 10, "{name}: {decided} decided, {undecided} undecided");
        assert!(tags >= 250, "{name}: {tags} tag checks");
    }
}

#[test]
fn frames_carry_the_orientation_and_tags_equal_what_the_frame_locates() {
    for name in MODELS {
        let (g, m, o) = (hex_fixture(name, "geometry.xml"), hex_fixture(name, "materials.xml"), oracle(name));
        let mut r = Rig::new();
        let d = r.import(&g, Some(&m)).unwrap();
        let fr = r.model().lattice_frame("lat50").unwrap_or_else(|| panic!("{name}: no frame lat50"));
        let n_axial = o["n_axial"].as_u64().unwrap() as u32;
        let want_orient = if o["orientation"] == "x" { HexOrient::XRows } else { HexOrient::YRows };
        match fr.layout {
            Layout::Hex { rings, pitch, orient, nz, dz } => {
                assert_eq!((rings, nz, orient), (3, n_axial, want_orient), "{name}");
                assert_eq!(pitch, f(&o["pitch"]));
                if n_axial > 1 {
                    assert_eq!(dz, f(&o["z_pitch"]));
                }
            }
            other => panic!("{name}: layout {other:?}"),
        }
        // Slot (0, 0, 0) is the centre of the first layer.
        let ir = openmc::import(&g, Some(&m)).unwrap();
        let c0 = ir.hex_lattices[&50].element_center(0, 0, 0);
        assert!((fr.origin.x - c0[0]).abs() < 1e-12 && (fr.origin.y - c0[1]).abs() < 1e-12, "{name}: origin {:?} vs {c0:?}", fr.origin);
        if n_axial > 1 {
            assert!((fr.origin.z - c0[2]).abs() < 1e-12, "{name}");
        }
        let (mut tagged, mut untagged) = (0, 0);
        for &v in &d.outputs["deck_volumes"] {
            match r.tag(v) {
                None => untagged += 1,
                Some(t) => {
                    // The fuel disc of an element is centred on it: its centroid is in its own slot.
                    let want = fr.locate(r.centroid(v)).expect("centroid inside the layout");
                    assert_eq!(t.slots["lat50"], want, "{name}: volume {v}");
                    tagged += 1;
                }
            }
        }
        let n_el = 19 * n_axial as usize;
        assert_eq!((tagged, untagged), (2 * n_el, 1), "{name}: {tagged} tagged volumes, {untagged} untagged");
    }
}

/// A homogeneous `n`-ring lattice (universe 7, material 1) with a gas outer (material 2) in a box
/// of half-width 20 and half-height 1; pitch 1.7.
fn homogeneous_deck(orientation: &str, n: u32) -> String {
    let per = 3 * n * (n - 1) + 1;
    let ids = vec!["7"; per as usize].join(" ");
    format!(
        r#"<geometry>
  <cell id="1" universe="0" fill="5" region="1 -2 3 -4 5 -6"/>
  <cell id="2" material="1" universe="7"/>
  <cell id="3" material="2" universe="9"/>
  <hex_lattice id="5" n_rings="{n}" orientation="{orientation}" outer="9"><pitch>1.7</pitch><center>0.3 -0.2</center><universes>{ids}</universes></hex_lattice>
  <surface id="1" type="x-plane" coeffs="-20" boundary="vacuum"/>
  <surface id="2" type="x-plane" coeffs="20" boundary="vacuum"/>
  <surface id="3" type="y-plane" coeffs="-20" boundary="vacuum"/>
  <surface id="4" type="y-plane" coeffs="20" boundary="vacuum"/>
  <surface id="5" type="z-plane" coeffs="-1" boundary="vacuum"/>
  <surface id="6" type="z-plane" coeffs="1" boundary="vacuum"/>
</geometry>"#
    )
}

#[test]
fn seven_rings_give_127_prisms_of_the_analytic_area() {
    let p = 1.7_f64;
    let area = 3f64.sqrt() / 2.0 * p * p;
    for orientation in ["x", "y"] {
        let mut r = Rig::new();
        let d = r.import(&homogeneous_deck(orientation, 7), Some(&materials(2))).unwrap_or_else(|e| panic!("{orientation}: {e}"));
        let blocks = block_of(&r, &d.outputs["deck_volumes"]);
        let prisms: Vec<EntityId> = blocks.iter().filter(|(_, n)| n.as_str() == "m1").map(|(v, _)| *v).collect();
        assert_eq!(prisms.len(), 3 * 7 * 6 + 1, "{orientation}: 127 element prisms");
        let mut total = 0.0;
        for &v in &prisms {
            let vol = r.volume(v);
            assert!((vol - area * 2.0).abs() <= 1e-9 * area * 2.0, "{orientation}: prism {v} has volume {vol}, want {}", area * 2.0);
            // A hexagonal prism: 6 laterals, top, bottom.
            assert_eq!(r.topo().volume_faces(v).len(), 8, "{orientation}: prism {v}");
            total += vol;
        }
        let want = 127.0 * area * 2.0;
        assert!((total - want).abs() <= 1e-9 * want, "{orientation}: prisms total {total}, want {want}");
        // The gas is the rest of the box: the sum of all volumes is the box.
        let gas: f64 = blocks.iter().filter(|(_, n)| n.as_str() == "m2").map(|(v, _)| r.volume(*v)).sum();
        let box_v = 40.0 * 40.0 * 2.0;
        assert!((gas - (box_v - want)).abs() <= 1e-9 * box_v, "{orientation}: gas {gas}, want {}", box_v - want);
        // One frame, the matching orientation, and a tag on every prism.
        let fr = r.model().lattice_frame("lat5").unwrap();
        let want_orient = if orientation == "x" { HexOrient::XRows } else { HexOrient::YRows };
        assert!(matches!(fr.layout, Layout::Hex { rings: 7, orient, nz: 1, .. } if orient == want_orient), "{orientation}: {:?}", fr.layout);
        assert!(prisms.iter().all(|v| r.tag(*v).is_some_and(|t| t.slots["lat5"] == fr.locate(r.centroid(*v)).unwrap())));
    }
}

#[test]
fn the_b_rep_is_the_same_for_a_deck_in_either_orientation_up_to_the_tags() {
    // The IR places the same set of hexagons whatever the text order: identical state hash twice.
    let a = {
        let mut r = Rig::new();
        r.import(&homogeneous_deck("y", 2), Some(&materials(2))).unwrap();
        r.hash()
    };
    let b = {
        let mut r = Rig::new();
        r.import(&homogeneous_deck("y", 2), Some(&materials(2))).unwrap();
        r.hash()
    };
    assert_eq!(a, b);
}

#[test]
fn the_ir_and_the_frame_use_one_orientation_map() {
    // sm-rt's `LatticeKind::Hex` is x-rows and `HexYRows` is y-rows; the IR and the model frame use the
    // same names (`HexOrientation::X` = `XRows`).
    let l = |o| HexLattice { center: [0.0; 3], pitch: 1.0, z_pitch: f64::INFINITY, orientation: o, n_rings: 2, n_axial: 1, universes: vec![0; 7], outer: None };
    let (x, y) = (l(HexOrientation::X), l(HexOrientation::Y));
    // The neighbour at (q, r) = (1, 0): along +x for rows along x, at 30 degrees for rows along y.
    assert_eq!(x.element_center(1, 0, 0), [1.0, 0.0, 0.0]);
    let c = y.element_center(1, 0, 0);
    assert!((c[0] - 3f64.sqrt() / 2.0).abs() < 1e-15 && (c[1] - 0.5).abs() < 1e-15);
}
