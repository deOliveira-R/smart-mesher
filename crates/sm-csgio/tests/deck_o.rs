//! Sphere in a box, built by hand as an IR (drafts/csg-import.md acceptance 3, 4, 5 and C9 names).
mod common;
use common::*;
use sm_csgio::ir::{Cell, Expr, Material};
use sm_csgio::{ImportError, ImportOptions, ImportedCsg};
use std::collections::BTreeMap;

#[test]
fn chords_are_analytic_and_the_exit_is_a_vacuum_boundary() {
    let m = import(&deck_o());
    assert_eq!(m.region_names(), ["material_1", "material_2"]);
    assert!(m.report().warnings.is_empty(), "a clean import has no warnings: {:?}", m.report().warnings);
    let t = tracker(&m);
    // y = 0.6: the sphere chord is 2 sqrt(1 - 0.36) = 1.6 of the 4.0 box chord.
    let c = chords(&t, [-2.0, 0.6, 0.0], [1.0, 0.0, 0.0]);
    assert!((c["material_1"] - 1.6).abs() < 1e-12, "{c:?}");
    assert!((c["material_2"] - 2.4).abs() < 1e-12, "{c:?}");
    let c = chords(&t, [-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!((c["material_1"] - 2.0).abs() < 1e-12 && (c["material_2"] - 2.0).abs() < 1e-12, "{c:?}");
    // y = 1.5 misses the sphere.
    let c = chords(&t, [-2.0, 1.5, 0.0], [1.0, 0.0, 0.0]);
    assert!(!c.contains_key("material_1") && (c["material_2"] - 4.0).abs() < 1e-12, "{c:?}");
}

#[test]
fn volumes_from_a_midpoint_grid_of_rays() {
    // 400 x 400 midpoint rays along z; tolerance 1e-3 relative (the chord test above is exact).
    let t = tracker(&import(&deck_o()));
    let n = 400;
    let h = 4.0 / n as f64;
    let (mut v1, mut v2) = (0.0, 0.0);
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (-2.0 + (i as f64 + 0.5) * h, -2.0 + (j as f64 + 0.5) * h);
            let c = chords(&t, [x, y, -2.0], [0.0, 0.0, 1.0]);
            v1 += c.get("material_1").copied().unwrap_or(0.0) * h * h;
            v2 += c.get("material_2").copied().unwrap_or(0.0) * h * h;
        }
    }
    let sphere = 4.0 * std::f64::consts::PI / 3.0;
    assert!((v1 - sphere).abs() / sphere < 1e-3, "sphere volume {v1} vs {sphere}");
    assert!((v2 - (64.0 - sphere)).abs() / (64.0 - sphere) < 1e-3, "box volume {v2}");
}

#[test]
fn overlap_is_rejected_with_the_deck_cells_and_a_point_in_the_sphere() {
    let e = ImportedCsg::import(&deck_o_overlap(), &ImportOptions::default()).unwrap_err();
    let ImportError::Overlap { findings } = e else { panic!("expected Overlap, got {e:?}") };
    assert!(!findings.is_empty() && findings.len() <= 16);
    let f = &findings[0];
    let r = (f.point[0].powi(2) + f.point[1].powi(2) + f.point[2].powi(2)).sqrt();
    assert!(r < 1.0, "the overlap is the whole sphere, got |p| = {r}");
    let mut cells = [f.a.cell, f.b.cell];
    cells.sort_unstable();
    assert_eq!(cells, [1, 2]);
    assert!(f.a.path.is_empty());
}

#[test]
fn gap_is_a_warning_with_a_point_in_the_unclaimed_sphere() {
    let m = import(&deck_o_gap());
    let gaps: Vec<_> = m.report().warnings.iter().filter(|w| w.code == "import.gap").collect();
    assert!(!gaps.is_empty(), "{:?}", m.report());
    for g in gaps {
        let p = g.point.unwrap();
        let d = ((p[0] - 1.4).powi(2) + (p[1] - 1.4).powi(2) + (p[2] - 1.4).powi(2)).sqrt();
        assert!(d < 0.3 + 1e-9, "gap point {p:?} is not in the carved sphere");
    }
}

#[test]
fn names_follow_c9() {
    let named = |a: Option<&str>, b: Option<&str>| {
        let mut ir = deck_o();
        ir.materials.insert(1, Material { name: a.map(str::to_owned) });
        ir.materials.insert(2, Material { name: b.map(str::to_owned) });
        ir
    };
    let names = |ir: &sm_csgio::CsgIr, o: &ImportOptions| ImportedCsg::instantiate(ir, o).unwrap().region_names();
    let d = ImportOptions::default();
    assert_eq!(names(&named(Some("fuel"), Some("water")), &d), ["fuel", "water"]);
    // Empty and duplicate names fall back to material_<id>.
    assert_eq!(names(&named(Some(""), Some("water")), &d), ["material_1", "water"]);
    assert_eq!(names(&named(Some("x"), Some("x")), &d), ["material_1", "material_2"]);
    // The override map renames.
    let o = ImportOptions { material_names: BTreeMap::from([(1, "uo2".to_string())]), ..ImportOptions::default() };
    assert_eq!(names(&named(Some("fuel"), None), &o), ["material_2", "uo2"]);
    // Void cells are "void"; an undefined material is an error.
    let mut ir = deck_o();
    ir.cells.get_mut(&2).unwrap().material = None;
    assert_eq!(names(&ir, &d), ["material_1", "void"]);
    ir.cells.insert(3, Cell::new(0, Expr::signed(&[2]), Some(9)));
    assert!(matches!(ImportedCsg::instantiate(&ir, &d), Err(ImportError::UnknownMaterial { cell: 3, material: 9 })));
}

#[test]
fn boundary_flags_become_rules_and_reflective_curved_warns() {
    use sm_csgio::ir::{BoundaryFlag, Surf};
    let m = import(&deck_o());
    assert_eq!(m.boundaries().rules.len(), 6);
    let mut ir = deck_o();
    ir.surfaces.insert(1, Surf::sphere([0.0; 3], 1.0).unwrap().with_flag(BoundaryFlag::Reflective));
    let m = ImportedCsg::instantiate(&ir, &ImportOptions::default()).unwrap();
    assert!(m.report().warnings.iter().any(|w| w.code == "import.reflective_curved"));
}

#[test]
fn reflective_planes_reflect_exactly() {
    use sm_csgio::ir::{BoundaryFlag, Surf};
    let mut ir = deck_o();
    for id in [2, 3] {
        let c = if id == 2 { -2.0 } else { 2.0 };
        ir.surfaces.insert(id, Surf::axis_plane(0, c).with_flag(BoundaryFlag::Reflective));
    }
    let t = tracker(&import(&ir));
    let mut sc = sm_rt_api::Scratch::new();
    use sm_rt_api::Tracker;
    let mut st = t.init_state(sm_core::Point3::new(0.0, 0.0, 0.0), sm_core::Vector3::new(1.0, 0.0, 0.0), None, &mut sc).unwrap();
    // Half a crossing (2) to the first wall, then four full crossings (4 each).
    let opts = sm_rt_api::TraceOptions::default().with_max_dist(2.0 + 4.0 * 4.0);
    let mut sphere = 0.0;
    let mut sink = |s: &sm_rt_api::Segment| {
        if t.regions().get(s.region).map(|r| r.name.as_str()) == Some("material_1") {
            sphere += s.length;
        }
        sm_rt_api::Flow::Continue
    };
    t.trace(&mut st, &opts, &mut sink, &mut sc).unwrap();
    // 1 inside the sphere on the first half crossing, then 2 on each full crossing.
    assert!((sphere - 9.0).abs() < 1e-9, "sphere chord sum {sphere}");
}
