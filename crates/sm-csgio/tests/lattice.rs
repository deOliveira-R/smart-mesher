//! Universes, rectangular lattices, fill transforms, clipping and `outer`, on hand-built IR.
//! Conventions under test are the IR's own (see the `ir` module docs); the format readers convert.
mod common;
use common::*;
use sm_core::{Transform, Vector3};
use sm_csgio::ir::{Cell, CsgIr, Expr, Fill, Lattice, Material, Surf};
use sm_csgio::{ImportOptions, ImportedCsg, PathStep};

const CHORD: f64 = 0.692_820_323_027_550_9; // 2 sqrt(0.4^2 - 0.2^2)

/// Universe 1: pin (fuel `-1` r = 0.4, moderator `+1`); universe 3: moderator only. Root cell 14
/// is the box x in [-1, 3], y and z in [-1, 1], filled with lattice 5.
fn pin_lattice(universes: Vec<u32>, pitch_z: f64, outer: Option<u32>, dims: [u32; 3]) -> CsgIr {
    let mut ir = CsgIr::new(0);
    ir.materials.insert(1, Material { name: Some("fuel".into()) });
    ir.materials.insert(2, Material { name: Some("water".into()) });
    ir.surfaces.insert(1, Surf::cylinder(2, [0.0; 3], 0.4).unwrap());
    for (id, axis, c) in [(2, 0, -1.0), (3, 0, 3.0), (4, 1, -1.0), (5, 1, 1.0), (6, 2, -1.0), (7, 2, 1.0)] {
        ir.surfaces.insert(id, Surf::axis_plane(axis, c));
    }
    ir.cells.insert(10, Cell::new(1, Expr::signed(&[-1]), Some(1)));
    ir.cells.insert(11, Cell::new(1, Expr::signed(&[1]), Some(2)));
    ir.cells.insert(12, Cell::new(3, Expr::And(vec![]), Some(2)));
    ir.lattices.insert(5, Lattice { lower_left: [-1.0, -1.0, if pitch_z.is_finite() { -1.0 } else { 0.0 }], pitch: [2.0, 2.0, pitch_z], dims, universes, outer });
    let id = Transform::IDENTITY;
    ir.cells.insert(14, Cell::filled(0, Expr::signed(&[2, -3, 4, -5, 6, -7]), Fill::Lattice { id: 5, xf: id }));
    ir
}

fn m1_m2(t: &sm_rt::AnalyticTracker, y: f64) -> (f64, f64) {
    let c = chords(t, [-1.0, y, 0.0], [1.0, 0.0, 0.0]);
    (c.get("fuel").copied().unwrap_or(0.0), c.get("water").copied().unwrap_or(0.0))
}

#[test]
fn lattice_direction_i_is_plus_x_and_swapping_moves_the_pin() {
    for (u, pin_first) in [(vec![1, 3], true), (vec![3, 1], false)] {
        let m = import(&pin_lattice(u, 2.0, None, [2, 1, 1]));
        assert!(m.report().warnings.is_empty(), "{:?}", m.report().warnings);
        let t = tracker(&m);
        let (f, w) = m1_m2(&t, 0.2);
        assert!((f - CHORD).abs() < 1e-12, "fuel chord {f}");
        assert!((w - (4.0 - CHORD)).abs() < 1e-12, "water chord {w}");
        let mut sc = sm_rt_api::Scratch::new();
        use sm_rt_api::Tracker;
        let fuel_at = |x: f64, sc: &mut sm_rt_api::Scratch| {
            let l = t.locate(sm_core::Point3::new(x, 0.0, 0.0), None, None, sc).unwrap();
            t.regions().get(l.region).unwrap().name.clone()
        };
        assert_eq!(fuel_at(0.0, &mut sc) == "fuel", pin_first);
        assert_eq!(fuel_at(2.0, &mut sc) == "fuel", !pin_first);
    }
}

#[test]
fn flat_cells_know_their_lattice_element() {
    let m = import(&pin_lattice(vec![1, 3], 2.0, None, [2, 1, 1]));
    // Pin: fuel + moderator in element 0; moderator in element 1.
    assert_eq!(m.cell_count(), 3);
    let elements: Vec<_> = m.sources().iter().map(|s| s.path.clone()).collect();
    assert!(elements.contains(&vec![PathStep { cell: 14, element: Some([1, 0, 0]) }]));
    assert!(elements.contains(&vec![PathStep { cell: 14, element: Some([0, 0, 0]) }]));
}

#[test]
fn infinite_pitch_axis_is_not_clipped() {
    let a = import(&pin_lattice(vec![1, 3], 2.0, None, [2, 1, 1]));
    let b = import(&pin_lattice(vec![1, 3], f64::INFINITY, None, [2, 1, 1]));
    let (ta, tb) = (tracker(&a), tracker(&b));
    assert_eq!(m1_m2(&ta, 0.2), m1_m2(&tb, 0.2));
    assert_eq!(b.cell_count(), 3);
}

#[test]
fn outer_fills_the_container_outside_the_array() {
    // 1 x 1 array in the 4-wide container. The gap check samples the bounds of the cells, so the
    // container is given as bounds: without `outer` it finds the gap, with it none.
    let bounds = sm_core::Aabb { lo: sm_core::Point3::new(-1.0, -1.0, -1.0), hi: sm_core::Point3::new(3.0, 1.0, 1.0) };
    let opts = ImportOptions { bounds: Some(bounds), ..ImportOptions::default() };
    let without = ImportedCsg::import(&pin_lattice(vec![1], 2.0, None, [1, 1, 1]), &opts).unwrap();
    assert!(without.report().warnings.iter().any(|w| w.code == "import.gap"));
    let with = ImportedCsg::import(&pin_lattice(vec![1], 2.0, Some(3), [1, 1, 1]), &opts).unwrap();
    assert!(with.report().warnings.is_empty(), "{:?}", with.report().warnings);
    let (f, w) = m1_m2(&tracker(&with), 0.2);
    assert!((f - CHORD).abs() < 1e-12);
    assert!((w - (4.0 - CHORD)).abs() < 1e-12);
}

#[test]
fn oversize_and_tangent_pins_are_clipped_not_overlapping() {
    for r in [1.0, 1.2] {
        let mut ir = pin_lattice(vec![1, 1], 2.0, None, [2, 1, 1]);
        ir.surfaces.insert(1, Surf::cylinder(2, [0.0; 3], r).unwrap());
        let m = import(&ir); // no Overlap
        let t = tracker(&m);
        // At y = 0 each element holds fuel across its whole width (clipped by the element box).
        let (f, _) = m1_m2(&t, 0.0);
        assert!((f - 4.0).abs() < 1e-12, "r = {r}: fuel chord {f}");
    }
}

fn rotated_pin_deck(xf: Transform) -> CsgIr {
    let mut ir = CsgIr::new(0);
    ir.materials.insert(1, Material { name: Some("fuel".into()) });
    ir.materials.insert(2, Material { name: Some("water".into()) });
    // Universe 7: a pin at (0.5, 0) in the universe frame.
    ir.surfaces.insert(1, Surf::cylinder(2, [0.5, 0.0, 0.0], 0.2).unwrap());
    ir.cells.insert(20, Cell::new(7, Expr::signed(&[-1]), Some(1)));
    ir.cells.insert(21, Cell::new(7, Expr::signed(&[1]), Some(2)));
    for (id, axis, c) in [(2, 0, -2.0), (3, 0, 2.0), (4, 1, -2.0), (5, 1, 2.0), (6, 2, -1.0), (7, 2, 1.0)] {
        ir.surfaces.insert(id, Surf::axis_plane(axis, c));
    }
    ir.cells.insert(30, Cell::filled(0, Expr::signed(&[2, -3, 4, -5, 6, -7]), Fill::Universe { id: 7, xf }));
    ir
}

#[test]
fn fill_translation_and_rotation_place_the_universe() {
    // Translation by (1, 0, 0): the pin is at (1.5, 0).
    let t = tracker(&import(&rotated_pin_deck(Transform::translation(Vector3::new(1.0, 0.0, 0.0)))));
    let c = chords(&t, [-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    assert!((c["fuel"] - 0.4).abs() < 1e-12 && (c["water"] - 3.6).abs() < 1e-12, "{c:?}");
    assert!(!chords(&t, [-2.0, 0.5, 0.0], [1.0, 0.0, 0.0]).contains_key("fuel"));
    // Rotation by 90 degrees about z maps (0.5, 0) to (0, 0.5).
    let rot = Transform { m: [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) };
    let t = tracker(&import(&rotated_pin_deck(rot)));
    let c = chords(&t, [-2.0, 0.5, 0.0], [1.0, 0.0, 0.0]);
    assert!((c["fuel"] - 0.4).abs() < 1e-12, "{c:?}");
    assert!(!chords(&t, [-2.0, 0.0, 0.0], [1.0, 0.0, 0.0]).contains_key("fuel"));
}

#[test]
fn nested_universes_clip_by_every_container() {
    // Universe 7 (infinite water) inside cell 30, inside a smaller box cell of the root: the
    // water must end at the box, not at the outer container.
    let mut ir = rotated_pin_deck(Transform::IDENTITY);
    ir.cells.remove(&30);
    ir.cells.insert(30, Cell::filled(0, Expr::signed(&[2, -3, 4, -5, 6, -7]), Fill::Universe { id: 8, xf: Transform::IDENTITY }));
    ir.surfaces.insert(8, Surf::axis_plane(0, 0.0));
    // Universe 8 holds only the half x < 0 of universe 7.
    ir.cells.insert(40, Cell::filled(8, Expr::signed(&[-8]), Fill::Universe { id: 7, xf: Transform::IDENTITY }));
    let t = tracker(&ImportedCsg::instantiate(&ir, &ImportOptions::default()).unwrap());
    let c = chords(&t, [-2.0, 1.0, 0.0], [1.0, 0.0, 0.0]);
    assert!((c["water"] - 2.0).abs() < 1e-12, "water must stop at x = 0: {c:?}");
}

#[test]
fn surfaces_are_shared_and_unused_ones_dropped() {
    let mut ir = pin_lattice(vec![1, 3], 2.0, None, [2, 1, 1]);
    ir.surfaces.insert(99, Surf::sphere([50.0; 3], 1.0).unwrap()); // used by no cell
    let m = ImportedCsg::instantiate(&ir, &ImportOptions::default()).unwrap();
    // cylinder + planes x=-1,1,3 + y=-1,1 + z=-1,1: the shared wall x = 1 exists once.
    assert_eq!(m.surface_count(), 1 + 3 + 2 + 2);
}
