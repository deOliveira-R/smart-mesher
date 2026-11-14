//! The BEAVRS import defect (W4-CSG-KFOLLOW review), reproduced small.
//!
//! Cause (sm-boolean, routed): `webcut` by a z-cylinder does not split a body whose cap faces
//! already carry the cylinder's circle, because the bodies above and below it (which share the cap
//! planes) were cut first; it returns the whole body as the negative piece. A pin universe whose
//! inside of cylinder 37 is a z-layered rod and whose outside is a z-layered grid universe (a
//! box fill in one layer) hits this in every grid layer: the layer's whole box goes to the fuel
//! cell and the pin hole of the grid water vanishes.
//!
//! The import therefore checks every piece it hands to a cell (a vertex outside the region of its
//! cell is a kernel defect) and refuses with a typed `csg.kernel` error instead of returning wrong
//! geometry. These tests hold both facts: the refusal today, exact volumes once the kernel is
//! fixed (the `#[ignore]` tests then pass and the refusal branches are never taken).
mod common;
use common::*;
use core::f64::consts::PI;
use std::collections::BTreeMap;

use sm_model_api::Domain;

/// Pin of radius 0.4572 (clad outer) and pellet 0.39218 in a 1.25984 square pin cell, rod height
/// 0..10, grid layer `z in [3, 4]` with a 1.2203 box. Materials: 1 fuel, 2 clad, 3 water, 4 grid.
fn deck() -> String {
    let (b, hc) = (0.61015, 0.62992);
    format!(
        "<?xml version='1.0' encoding='UTF-8'?><geometry>\
        <cell id='1' fill='15' region='11 -12 13 -14 1 -6' universe='0'/>\
        <cell id='2' material='3' region='21 -22 23 -24' universe='2'/>\
        <cell id='3' material='4' region='-21 | 22 | -23 | 24' universe='2'/>\
        <cell id='10' material='3' region='-2' universe='6'/>\
        <cell id='11' fill='2' region='2 -3' universe='6'/>\
        <cell id='12' material='3' region='3' universe='6'/>\
        <cell id='52' material='1' region='-35' universe='8'/>\
        <cell id='53' material='2' region='35 -37' universe='8'/>\
        <cell id='79' fill='8' region='-37' universe='15'/>\
        <cell id='80' fill='6' region='37' universe='15'/>\
        <surface id='11' type='x-plane' boundary='vacuum' coeffs='{nh}'/>\
        <surface id='12' type='x-plane' boundary='vacuum' coeffs='{hc}'/>\
        <surface id='13' type='y-plane' boundary='vacuum' coeffs='{nh}'/>\
        <surface id='14' type='y-plane' boundary='vacuum' coeffs='{hc}'/>\
        <surface id='21' type='x-plane' coeffs='{nb}'/><surface id='22' type='x-plane' coeffs='{b}'/>\
        <surface id='23' type='y-plane' coeffs='{nb}'/><surface id='24' type='y-plane' coeffs='{b}'/>\
        <surface id='1' type='z-plane' boundary='vacuum' coeffs='0'/>\
        <surface id='6' type='z-plane' boundary='vacuum' coeffs='10'/>\
        <surface id='2' type='z-plane' coeffs='3'/><surface id='3' type='z-plane' coeffs='4'/>\
        <surface id='35' type='z-cylinder' coeffs='0 0 0.39218'/>\
        <surface id='37' type='z-cylinder' coeffs='0 0 0.4572'/></geometry>",
        nb = -b,
        nh = -hc
    )
}

/// Import and return the volume per material, or the refusal text.
fn per_material(geometry: &str, materials: &str) -> Result<BTreeMap<String, f64>, String> {
    let mut r = Rig::new();
    let d = r.import(geometry, Some(materials)).map_err(|e| e.to_string())?;
    let mut by: BTreeMap<String, f64> = BTreeMap::new();
    for c in r.model().colors().filter(|c| c.domain == Domain::Volume) {
        for v in r.model().volumes_of_color(c.id) {
            if d.outputs["deck_volumes"].contains(&v) {
                *by.entry(c.name.clone()).or_default() += r.volume(v);
            }
        }
    }
    Ok(by)
}

/// The refusal of a piece the kernel left unsplit: typed, naming the cell and the surface ids.
fn assert_typed_refusal(e: &str) {
    assert!(e.contains("csg.kernel") && e.contains("left the piece unsplit") && e.contains("surface ids"), "{e}");
}

#[test]
fn a_grid_layer_pin_is_never_imported_with_wrong_volumes() {
    let (rp, rc) = (0.39218_f64, 0.4572_f64);
    match per_material(&deck(), &materials(4)) {
        Err(e) => assert_typed_refusal(&e),
        Ok(by) => {
            let want = [("m1", PI * rp * rp * 10.0), ("m2", PI * (rc * rc - rp * rp) * 10.0)];
            for (m, w) in want {
                let got = by.get(m).copied().unwrap_or(0.0);
                assert!((got - w).abs() < 1e-9 * w, "{m}: {got} vs analytic {w} ({by:?})");
            }
            let total: f64 = by.values().sum();
            assert!((total - (2.0 * 0.62992_f64).powi(2) * 10.0).abs() < 1e-9, "total {total}");
        }
    }
}

/// Wanted: the small pin imports with its analytic fuel and clad (fails today, see the module
/// doc: the refusal is the best the import can do until sm-boolean is fixed).
#[test]
#[ignore = "sm-boolean finding: webcut by a cylinder on a body between two bodies it already cut"]
fn the_grid_layer_pin_imports_with_exact_volumes() {
    let (rp, rc) = (0.39218_f64, 0.4572_f64);
    let by = per_material(&deck(), &materials(4)).unwrap_or_else(|e| panic!("{e}"));
    for (m, w) in [("m1", PI * rp * rp * 10.0), ("m2", PI * (rc * rc - rp * rp) * 10.0)] {
        let got = by.get(m).copied().unwrap_or(0.0);
        assert!((got - w).abs() < 1e-9 * w, "{m}: {got} vs analytic {w} ({by:?})");
    }
}

/// The bare kernel cut behind it: a box `[-h, h]^2 x [0, 10]` is cut by the planes z = 3 and
/// z = 4, the pieces below and above are cut by the cylinder of radius `r`, then the slab between
/// is cut by it (the order of the import's queue). With `trim` the slab is first cut to the grid
/// box by four planes. Returns the slab's negative and positive volumes.
fn slab_after_neighbours(trim: bool, r: f64) -> (Vec<f64>, Vec<f64>) {
    use sm_boolean::{CutTool, WebcutOptions, webcut};
    use sm_core::{EntityId, Point3, Vector3};
    use sm_geom_api::{Cylinder, Frame, SurfaceDef};
    use sm_topo_api::{OpId, TopologyEdit, TopologyView, fixtures::brick};
    let mut t = sm_topo::TopoStore::new();
    t.begin_op(OpId(1)).unwrap();
    let (h, g) = (0.62992, 0.61015);
    let b = brick(&mut t, Point3::new(-h, -h, 0.0), [2.0 * h, 2.0 * h, 10.0]).unwrap();
    let vol = |t: &sm_topo::TopoStore, b: EntityId| {
        let v = t.boundary(b).iter().map(|i| i.cell).find(|c| t.dim_of(*c) == Some(sm_topo_api::Dim::Volume)).unwrap();
        t.volume_moments(v).unwrap().volume
    };
    let plane = |o: [f64; 3], n: [f64; 3]| CutTool::Plane { origin: Point3::new(o[0], o[1], o[2]), normal: Vector3::new(n[0], n[1], n[2]) };
    // The seam reference turned by 1 rad, as the import does.
    let f0 = Frame::at(Point3::new(0.0, 0.0, 0.0));
    let x = f0.x * 1.0_f64.cos() + f0.y * 1.0_f64.sin();
    let cyl = CutTool::Surface { surface: SurfaceDef::Cylinder(Cylinder::new(Frame::from_zx(f0.origin, f0.z, x).unwrap(), r).unwrap()), window: None };
    let cut = |t: &mut sm_topo::TopoStore, body: EntityId, tool: &CutTool| webcut(t, &[body], tool, &WebcutOptions::default()).unwrap();
    let below = cut(&mut t, b.body, &plane([0.0, 0.0, 3.0], [0.0, 0.0, 1.0]));
    let above = cut(&mut t, below.positive[0], &plane([0.0, 0.0, 4.0], [0.0, 0.0, 1.0]));
    let (mut slab, above) = (above.negative[0], above.positive[0]);
    cut(&mut t, below.negative[0], &cyl);
    cut(&mut t, above, &cyl);
    if trim {
        for tool in [plane([0.0, g, 0.0], [0.0, -1.0, 0.0]), plane([0.0, -g, 0.0], [0.0, 1.0, 0.0]), plane([g, 0.0, 0.0], [-1.0, 0.0, 0.0]), plane([-g, 0.0, 0.0], [1.0, 0.0, 0.0])] {
            let c = cut(&mut t, slab, &tool);
            slab = c.negative.into_iter().chain(c.positive).max_by(|a, b| vol(&t, *a).total_cmp(&vol(&t, *b))).unwrap();
        }
    }
    let c = cut(&mut t, slab, &cyl);
    (c.negative.iter().map(|&b| vol(&t, b)).collect(), c.positive.iter().map(|&b| vol(&t, b)).collect())
}

/// KERNEL FINDING for sm-boolean: expected the negative piece to be the cylinder, `pi r^2 (4 - 3)
/// = 0.4832`, and one positive piece; today `webcut` returns the whole slab unsplit (as the
/// positive piece for the untrimmed slab, as the negative piece for the trimmed one).
#[test]
#[ignore = "sm-boolean finding: webcut by a cylinder on a body between two bodies it already cut"]
fn the_cylinder_cuts_a_slab_between_two_bodies_it_has_already_cut() {
    let want = PI * 0.39218_f64.powi(2);
    for trim in [false, true] {
        let (neg, pos) = slab_after_neighbours(trim, 0.39218);
        assert!(neg.len() == 1 && (neg[0] - want).abs() < 1e-9 && pos.len() == 1, "trim {trim}: neg {neg:?} pos {pos:?} (cylinder {want})");
    }
}

/// The real BEAVRS cells and surfaces with the lattice replaced by one universe-15 pin cell of
/// half-pitch `h` (root cell 1253 filled with 15, vacuum on the box).
fn beavrs_pin(h: f64) -> String {
    let g = fixture("ref_beavrs_assembly", "geometry.xml");
    let mut out = String::from("<?xml version='1.0' encoding='UTF-8'?><geometry>\n");
    let used = ["2", "3", "6", "8", "13", "14", "15", "763"];
    let keep = |l: &str| l.trim_start().starts_with("<surface ") || (l.trim_start().starts_with("<cell ") && used.iter().any(|u| l.contains(&format!("universe=\"{u}\""))));
    for l in g.lines().filter(|l| keep(l)) {
        if l.contains("id=\"1253\"") {
            out.push_str("<cell id=\"1253\" fill=\"15\" region=\"150 -151 152 -153 154 -155\" universe=\"763\"/>\n");
        } else if ["150", "151", "152", "153"].iter().any(|i| l.contains(&format!("<surface id=\"{i}\""))) {
            let (id, ty) = (l.split('"').nth(1).unwrap(), if l.contains("x-plane") { "x-plane" } else { "y-plane" });
            let c = if id == "150" || id == "152" { -h } else { h };
            out.push_str(&format!("<surface id=\"{id}\" type=\"{ty}\" boundary=\"vacuum\" coeffs=\"{c}\"/>\n"));
        } else {
            out.push_str(l);
            out.push('\n');
        }
    }
    out.push_str("</geometry>\n");
    out
}

/// One real BEAVRS pin cell (universe 15, 460 cm, eight grid layers): the Fuel 1.6% volume must
/// be the analytic `pi * 0.39218^2 * (402.508 - 36.748)`; today the import refuses it.
#[test]
fn the_beavrs_pin_cell_is_refused_or_has_the_analytic_fuel() {
    let materials = fixture("ref_beavrs_assembly", "materials.xml");
    match per_material(&beavrs_pin(0.62992), &materials) {
        Err(e) => assert_typed_refusal(&e),
        Ok(by) => {
            let fuel = PI * 0.39218_f64.powi(2) * (402.508 - 36.748);
            let got = by["Fuel 1.6%"];
            assert!((got - fuel).abs() < 1e-6 * fuel, "Fuel 1.6%: {got} vs analytic {fuel} ({by:?})");
        }
    }
}
