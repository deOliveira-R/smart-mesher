//! `model.import_openmc`: volumes, blocks, sidesets, lattice tags, refusals.
mod common;
use common::decks::*;
use common::*;
use core::f64::consts::PI;
use sm_core::EntityId;
use sm_csgbrep::{OpenmcOptions, OverlapMode};
use sm_model_api::{Domain, Layout};

fn close(what: &str, got: f64, want: f64, tol: f64) {
    assert!((got - want).abs() <= tol * want.abs().max(1.0), "{what}: got {got}, want {want}");
}

fn block_names(r: &Rig) -> Vec<(String, Vec<EntityId>)> {
    let m = r.model();
    let mut v: Vec<(String, Vec<EntityId>)> = m.colors().filter(|c| c.domain == Domain::Volume).map(|c| (c.name.clone(), m.volumes_of_color(c.id))).collect();
    v.sort();
    v
}

#[test]
fn pin_has_one_block_per_name_and_sidesets_on_the_flagged_planes() {
    let mut r = Rig::new();
    let d = r.import(&fixture("pin", "geometry.xml"), Some(&fixture("pin", "materials.xml"))).unwrap();
    let blocks = block_names(&r);
    let names: Vec<&str> = blocks.iter().map(|b| b.0.as_str()).collect();
    assert_eq!(names, ["clad", "fuel", "void", "water"], "{blocks:?}");
    assert!(blocks.iter().all(|b| b.1.len() == 1));
    // Sidesets: vacuum on the two z planes (each of the four volumes has a top and a bottom
    // face), reflective on the four x and y planes (only the moderator reaches them).
    let m = r.model();
    let side = |name: &str| m.color_by_name(name).map(|c| m.faces_of_color(c).len());
    assert_eq!(side("vacuum"), Some(8));
    assert_eq!(side("reflective"), Some(4));
    assert_eq!(side("white"), None);
    assert_eq!(d.outputs["deck_volumes"].len(), 4);
    assert_eq!(d.outputs["deck_bodies"].len(), 4);
    assert!(diag(&d, "import.gap").is_empty());
}

/// Lattice tags: every element volume's tag equals the deck index and what the frame would give
/// from the volume's centroid (the rule `model.lattice_tag` will apply).
fn check_tags(r: &Rig, d: &sm_op_api::Delta, frame: &str, n: [u32; 3], untagged: usize) {
    let f = r.model().lattice_frame(frame).expect("frame");
    assert!(matches!(f.layout, Layout::Cartesian { n: m, .. } if m == n));
    let mut none = 0;
    for &v in &d.outputs["deck_volumes"] {
        match r.tag(v) {
            None => none += 1,
            Some(t) => {
                let want = f.locate(r.centroid(v)).expect("centroid inside the layout");
                assert_eq!(t.slots[frame], want, "volume {v}");
                assert!(t.ring.is_none() && t.sector.is_none());
            }
        }
    }
    assert_eq!(none, untagged);
}

#[test]
fn lattice2d_fixture_tags_equal_the_deck_indices() {
    let mut r = Rig::new();
    let d = r.import(&fixture("lattice2d", "geometry.xml"), Some(&fixture("lattice2d", "materials.xml"))).unwrap();
    // 3 x 2 elements of 2 volumes (pin and moderator) and one outer volume.
    assert_eq!(d.outputs["deck_volumes"].len(), 13);
    check_tags(&r, &d, "lat20", [3, 2, 1], 1);
    // Centroids: element [i, j] is centred at lower_left + (i + 1/2, j + 1/2) * pitch.
    for &v in &d.outputs["deck_volumes"] {
        if let Some(t) = r.tag(v) {
            let ix = t.slots["lat20"];
            let c = r.centroid(v);
            let (cx, cy) = (-1.8 + (f64::from(ix.i) + 0.5) * 1.2, -1.0 + (f64::from(ix.j) + 0.5) * 1.0);
            close("x", c.x, cx, 1e-9);
            close("y", c.y, cy, 1e-9);
        }
    }
    // The deck writes the universes `10 11 12 / 13 14 15` top to bottom, so row j = 0 holds 13..15
    // and j = 1 holds 10..12 (the reader reverses OpenMC's row order). Each universe has its own
    // pin radius, so the tag index decides which radius the element's volumes have.
    let radius = |u: u32| 0.15 + 0.06 * f64::from(u - 10);
    for i in 0..3i32 {
        for j in 0..2i32 {
            let u = if j == 0 { 13 + i as u32 } else { 10 + i as u32 };
            let mut vols: Vec<f64> = d.outputs["deck_volumes"].iter().filter(|&&v| r.tag(v).is_some_and(|t| t.slots["lat20"] == sm_model_api::SlotIx::new(i, j, 0))).map(|&v| r.volume(v)).collect();
            vols.sort_by(f64::total_cmp);
            let pin = PI * radius(u) * radius(u) * 2.0;
            let mut want = vec![pin, 1.2 * 1.0 * 2.0 - pin];
            want.sort_by(f64::total_cmp);
            assert_eq!(vols.len(), 2, "element [{i},{j}]");
            for (g, w) in vols.iter().zip(&want) {
                close(&format!("element [{i},{j}] (universe {u})"), *g, *w, 1e-9);
            }
        }
    }
    // The pin volumes of the first universe row: pi r^2 h with h = 2 (z in [-1, 1]).
    let mut pins: Vec<f64> = d.outputs["deck_volumes"].iter().filter(|&&v| r.tag(v).is_some()).map(|&v| r.volume(v)).collect();
    pins.sort_by(f64::total_cmp);
    let mut want: Vec<f64> = [0.15, 0.21, 0.27, 0.33, 0.39, 0.45].iter().flat_map(|&rad: &f64| [PI * rad * rad * 2.0, 1.2 * 1.0 * 2.0 - PI * rad * rad * 2.0]).collect();
    want.sort_by(f64::total_cmp);
    for (g, w) in pins.iter().zip(&want) {
        close("element volume", *g, *w, 1e-9);
    }
    // The outer volume: container minus the array box.
    let outer: f64 = d.outputs["deck_volumes"].iter().filter(|&&v| r.tag(v).is_none()).map(|&v| r.volume(v)).sum();
    close("outer", outer, 5.2 * 3.2 * 2.0 - 3.6 * 2.0 * 2.0, 1e-9);
}

#[test]
fn lattice_3x3_has_4n_volumes_and_deck_index_tags() {
    let mut r = Rig::new();
    let (r1, r2, r3) = (0.35, 0.37, 0.42);
    let d = r.import(&pin_lattice(3, 1.26, (r1, r2, r3), 2.0, false), Some(&materials(4))).unwrap();
    assert_eq!(d.outputs["deck_volumes"].len(), 36);
    check_tags(&r, &d, "lat20", [3, 3, 1], 0);
    let mut by_name: std::collections::BTreeMap<String, f64> = Default::default();
    for (name, vols) in block_names(&r) {
        by_name.insert(name, vols.iter().map(|&v| r.volume(v)).sum());
    }
    close("fuel", by_name["m1"], 9.0 * PI * r1 * r1 * 2.0, 1e-9);
    close("clad", by_name["m2"], 9.0 * PI * (r3 * r3 - r2 * r2) * 2.0, 1e-9);
    close("gap", by_name["void"], 9.0 * PI * (r2 * r2 - r1 * r1) * 2.0, 1e-9);
    close("moderator", by_name["m3"], 9.0 * (1.26 * 1.26 - PI * r3 * r3) * 2.0, 1e-9);
    let p = diag(&d, "import.partition");
    println!("3x3: {p:?}");
    // The kernel cuts of the flat partition: 3 circles per element, plus the interior lattice
    // planes (n - 1 x-planes cut the box into strips, n - 1 y-planes cut each strip). The box
    // planes coincide with the box faces and only report a side (a webcut call, no cut).
    let n = 3;
    let want = 3 * n * n + (n - 1) + (n - 1) * n;
    assert!(p[0].starts_with(&format!("{want} kernel cuts")), "{p:?}");
}

#[test]
fn lattice_with_outer_has_one_extra_ring_volume() {
    let mut r = Rig::new();
    let d = r.import(&pin_lattice(3, 1.0, (0.2, 0.25, 0.3), 1.0, true), Some(&materials(4))).unwrap();
    assert_eq!(d.outputs["deck_volumes"].len(), 37);
    check_tags(&r, &d, "lat20", [3, 3, 1], 1);
    let ring: f64 = block_names(&r).into_iter().find(|b| b.0 == "m4").unwrap().1.iter().map(|&v| r.volume(v)).sum();
    close("outer ring", ring, 4.0 * 4.0 - 9.0, 1e-9);
}

#[test]
fn a_disconnected_cell_becomes_several_volumes_in_one_block() {
    let mut r = Rig::new();
    let d = r.import(&disconnected_moderator(1.0, 1.0), Some(&materials(2))).unwrap();
    assert_eq!(d.outputs["deck_volumes"].len(), 3);
    let blocks = block_names(&r);
    let m2 = blocks.iter().find(|b| b.0 == "m2").unwrap();
    assert_eq!(m2.1.len(), 2);
    for &v in &m2.1 {
        close("part", r.volume(v), 0.75 * 2.0 * 1.0, 1e-9);
    }
    close("slab", r.volume(blocks.iter().find(|b| b.0 == "m1").unwrap().1[0]), 0.5 * 2.0 * 1.0, 1e-9);
}

#[test]
fn a_quadric_that_is_a_cylinder_equals_the_cylinder() {
    let mut r = Rig::new();
    let d = r.import(&quadric_cylinder(0.4, (0.1, -0.2), 1.0, 2.0), Some(&materials(2))).unwrap();
    let vols = &d.outputs["deck_volumes"];
    assert_eq!(vols.len(), 2);
    let mut v: Vec<f64> = vols.iter().map(|&x| r.volume(x)).collect();
    v.sort_by(f64::total_cmp);
    close("cyl", v[0], PI * 0.16 * 2.0, 1e-9);
    close("rest", v[1], 8.0 - PI * 0.16 * 2.0, 1e-9);
}

#[test]
fn translated_and_rotated_fills_land_at_the_analytic_places() {
    let fuel = |r: &Rig, d: &sm_op_api::Delta| {
        let blocks = block_names(r);
        let id = blocks.iter().find(|b| b.0 == "m1").unwrap().1[0];
        assert!(d.outputs["deck_volumes"].contains(&id));
        (r.centroid(id), r.volume(id))
    };
    let mats = materials(2);
    // Translation: the cylinder centre moves by t.
    let mut r = Rig::new();
    let d = r.import(&placed_cylinder([6.0, -1.0, 0.5], None, 0.3), Some(&mats)).unwrap();
    let (c, v) = fuel(&r, &d);
    close("cx", c.x, 6.2, 1e-9);
    close("cy", c.y, -0.9, 1e-9);
    close("cz", c.z, 0.5, 1e-9);
    close("vol", v, PI * 0.09 * 2.0, 1e-9);
    // Rotation about z by 90 degrees (OpenMC `u = M (r - t)`, `M = Rz(-90)`): r = Rz(90) u + t.
    let mut r = Rig::new();
    let d = r.import(&placed_cylinder([6.0, 0.0, 0.0], Some("0 0 90"), 0.3), Some(&mats)).unwrap();
    let (c, v) = fuel(&r, &d);
    close("rot cx", c.x, 6.0 - 0.1, 1e-9);
    close("rot cy", c.y, 0.2, 1e-9);
    close("rot vol", v, PI * 0.09 * 2.0, 1e-9);
}

#[test]
fn a_rotated_lattice_imports_with_a_warning() {
    let mut r = Rig::new();
    let d = r.import(&fixture("lattice_xf", "geometry.xml"), Some(&fixture("lattice_xf", "materials.xml"))).unwrap();
    assert_eq!(d.outputs["deck_volumes"].len(), 9);
    assert_eq!(diag(&d, "import.lattice_rotated").len(), 1);
    // Tags still carry the deck indices; one tag per element volume.
    let tagged = d.outputs["deck_volumes"].iter().filter(|&&v| r.tag(v).is_some()).count();
    assert_eq!(tagged, 8);
}

#[test]
fn unsupported_decks_name_the_element() {
    for (name, needle) in [("deck_o", "surface 1: a sphere"), ("outer_structured", "line 7")] {
        let mut r = Rig::new();
        let e = r.import(&fixture(name, "geometry.xml"), None).unwrap_err().to_string();
        assert!(e.contains("csg.unsupported") && e.contains(needle), "{name}: {e}");
    }
    assert!(Rig::new().import("<geometry>", None).is_err());
}

fn overlap_deck() -> String {
    "<?xml version='1.0' encoding='UTF-8'?><geometry>\
     <cell id='1' material='1' region='1 -2 3 -4 5 -6' universe='1'/>\
     <cell id='2' material='2' region='7 -8 3 -4 5 -6' universe='1'/>\
     <surface id='1' type='x-plane' boundary='vacuum' coeffs='0'/><surface id='2' type='x-plane' coeffs='2'/>\
     <surface id='7' type='x-plane' coeffs='1.5'/><surface id='8' type='x-plane' boundary='vacuum' coeffs='3'/>\
     <surface id='3' type='y-plane' boundary='vacuum' coeffs='0'/><surface id='4' type='y-plane' boundary='vacuum' coeffs='1'/>\
     <surface id='5' type='z-plane' boundary='vacuum' coeffs='0'/><surface id='6' type='z-plane' boundary='vacuum' coeffs='1'/></geometry>"
        .replace('\'', "\"")
}

#[test]
fn overlaps_fail_by_default_and_are_kept_by_the_first_on_request() {
    let mats = materials(2);
    let mut r = Rig::new();
    let e = r.import(&overlap_deck(), Some(&mats)).unwrap_err().to_string();
    assert!(e.contains("csg.overlap") && e.contains("e-1"), "{e}");
    let mut r = Rig::new();
    let d = r.import_with(&overlap_deck(), Some(&mats), Some(OpenmcOptions { overlap: Some(OverlapMode::Report), ..Default::default() })).unwrap();
    assert_eq!(diag(&d, "import.overlap").len(), 1);
    let total: f64 = d.outputs["deck_volumes"].iter().map(|&v| r.volume(v)).sum();
    close("union", total, 3.0, 1e-9);
}

#[test]
#[ignore = "slow in debug (about 30 s); run with --ignored"]
fn lattice_17x17_imports() {
    let mut r = Rig::new();
    let t = std::time::Instant::now();
    let d = r.import(&pin_lattice(17, 1.26, (0.4096, 0.418, 0.475), 2.0, false), Some(&materials(4))).unwrap();
    println!("17x17: {:?} in {:.1}s", diag(&d, "import.partition"), t.elapsed().as_secs_f64());
    assert_eq!(d.outputs["deck_volumes"].len(), 4 * 289);
    check_tags(&r, &d, "lat20", [17, 17, 1], 0);
}

/// The real PWR pin (fuel 0.4096, clad 0.418 to 0.475; the gap clearance 0.0084 is below the
/// kernel's near-tangent 1e-2) imports once the cylinder tools' seam is turned (the
/// W4-REGIONS-CUT finding), with analytic volumes.
#[test]
fn the_real_pwr_gap_imports() {
    let mut r = Rig::new();
    let (r1, r2, r3, p, h) = (0.4096, 0.418, 0.475, 1.26, 2.0);
    let d = r.import(&pin_lattice(1, p, (r1, r2, r3), h, false), Some(&materials(4))).unwrap();
    let mut v: Vec<f64> = d.outputs["deck_volumes"].iter().map(|&x| r.volume(x)).collect();
    v.sort_by(f64::total_cmp);
    let mut want = vec![PI * r1 * r1 * h, PI * (r2 * r2 - r1 * r1) * h, PI * (r3 * r3 - r2 * r2) * h, (p * p - PI * r3 * r3) * h];
    want.sort_by(f64::total_cmp);
    for (g, w) in v.iter().zip(&want) {
        close("pin volume", *g, *w, 1e-9);
    }
}

/// A kernel limit the import still meets, pinned so a kernel fix shows up here: a pin tangent
/// to its element box (line contact) is a typed `csg.kernel` error naming the deck surface.
#[test]
fn a_pin_tangent_to_its_box_is_a_typed_kernel_error() {
    let mut r = Rig::new();
    let e = r.import(&pin_lattice(1, 1.0, (0.2, 0.3, 0.5), 1.0, false), Some(&materials(4))).unwrap_err().to_string();
    assert!(e.contains("csg.kernel") && e.contains("deck surface"), "{e}");
}

/// A curved outer boundary: a pin inside a vacuum z-cylinder (the box corners beyond the vacuum
/// surface are outside the model, not gaps).
#[test]
fn a_curved_vacuum_boundary_leaves_no_gap() {
    let deck = "<?xml version='1.0' encoding='UTF-8'?><geometry>\
        <cell id='1' material='1' region='-1 3 -4' universe='1'/>\
        <cell id='2' material='2' region='1 -2 3 -4' universe='1'/>\
        <surface id='1' type='z-cylinder' coeffs='0.0 0.0 0.4'/>\
        <surface id='2' type='z-cylinder' boundary='vacuum' coeffs='0.0 0.0 1.0'/>\
        <surface id='3' type='z-plane' boundary='vacuum' coeffs='0'/><surface id='4' type='z-plane' boundary='vacuum' coeffs='3'/></geometry>"
        .replace('\'', "\"");
    let mut r = Rig::new();
    let d = r.import(&deck, Some(&materials(2))).unwrap();
    assert!(diag(&d, "import.gap").is_empty(), "{:?}", diag(&d, "import.gap"));
    let mut v: Vec<f64> = d.outputs["deck_volumes"].iter().map(|&x| r.volume(x)).collect();
    v.sort_by(f64::total_cmp);
    assert_eq!(v.len(), 2);
    close("pin", v[0], PI * 0.16 * 3.0, 1e-9);
    close("annulus", v[1], PI * 0.84 * 3.0, 1e-9);
    // Vacuum faces: the two caps of each volume (4) and the outer cylindrical face (1).
    let m = r.model();
    let faces = m.color_by_name("vacuum").map(|c| m.faces_of_color(c).len());
    assert_eq!(faces, Some(5));
}

#[test]
fn a_gap_is_dropped_with_a_warning() {
    let deck = overlap_deck().replace("coeffs=\"1.5\"", "coeffs=\"2.5\"");
    let mut r = Rig::new();
    let d = r.import(&deck, Some(&materials(2))).unwrap();
    let g = diag(&d, "import.gap");
    assert_eq!(g.len(), 1, "{g:?}");
    assert!(g[0].contains("e-1"), "{g:?}");
    assert_eq!(d.outputs["deck_volumes"].len(), 2);
}
