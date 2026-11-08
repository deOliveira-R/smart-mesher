//! Typed errors of the core: cycles, unknown ids, DNF blow-up, cell cap, bad lattices, options.
mod common;
use common::*;
use sm_core::Transform;
use sm_csgio::ir::{Cell, CsgIr, Expr, Fill, Lattice, Material, Surf};
use sm_csgio::{CheckOptions, ImportError, ImportOptions, ImportedCsg};

fn inst(ir: &CsgIr) -> Result<ImportedCsg, ImportError> {
    ImportedCsg::instantiate(ir, &ImportOptions::default())
}

fn box_cell(ir: &mut CsgIr, id: u32, universe: u32, fill: Fill) {
    // Planes 1..=6 bound the box [-1, 1]^3.
    for (s, axis, c) in [(1, 0, -1.0), (2, 0, 1.0), (3, 1, -1.0), (4, 1, 1.0), (5, 2, -1.0), (6, 2, 1.0)] {
        ir.surfaces.entry(s).or_insert_with(|| Surf::axis_plane(axis, c));
    }
    ir.cells.insert(id, Cell::filled(universe, Expr::signed(&[1, -2, 3, -4, 5, -6]), fill));
}

fn leaf_cell(ir: &mut CsgIr, id: u32, universe: u32) {
    ir.materials.entry(1).or_default();
    ir.cells.insert(id, Cell::new(universe, Expr::And(vec![]), Some(1)));
}

fn fill_u(id: u32) -> Fill {
    Fill::Universe { id, xf: Transform::IDENTITY }
}

#[test]
fn a_universe_that_contains_itself_is_a_cycle() {
    let mut ir = CsgIr::new(0);
    box_cell(&mut ir, 1, 0, fill_u(1));
    leaf_cell(&mut ir, 2, 1);
    box_cell(&mut ir, 3, 1, fill_u(1)); // universe 1 holds a cell filled with universe 1
    assert_eq!(inst(&ir).unwrap_err(), ImportError::UniverseCycle { universe: 1 });
}

#[test]
fn mutual_containment_is_a_cycle() {
    let mut ir = CsgIr::new(0);
    box_cell(&mut ir, 1, 0, fill_u(1));
    box_cell(&mut ir, 2, 1, fill_u(2));
    box_cell(&mut ir, 3, 2, fill_u(1));
    assert!(matches!(inst(&ir), Err(ImportError::UniverseCycle { .. })));
}

#[test]
fn unknown_ids_are_named() {
    let mut ir = CsgIr::new(0);
    box_cell(&mut ir, 1, 0, fill_u(9));
    assert_eq!(inst(&ir).unwrap_err(), ImportError::UnknownUniverse { cell: Some(1), universe: 9 });
    let mut ir = CsgIr::new(0);
    box_cell(&mut ir, 1, 0, Fill::Lattice { id: 4, xf: Transform::IDENTITY });
    assert_eq!(inst(&ir).unwrap_err(), ImportError::UnknownLattice { cell: 1, lattice: 4 });
    // The root universe must exist.
    let mut ir = CsgIr::new(5);
    leaf_cell(&mut ir, 1, 0);
    assert_eq!(inst(&ir).unwrap_err(), ImportError::UnknownUniverse { cell: None, universe: 5 });
    // A region naming an undefined surface.
    let mut ir = CsgIr::new(0);
    leaf_cell(&mut ir, 1, 0);
    ir.cells.insert(2, Cell::new(0, Expr::signed(&[77]), Some(1)));
    assert_eq!(inst(&ir).unwrap_err(), ImportError::UnknownSurface { cell: 2, surface: 77 });
}

#[test]
fn dnf_blow_up_names_the_cell() {
    let mut ir = CsgIr::new(0);
    ir.materials.insert(1, Material::default());
    for i in 1..=26 {
        ir.surfaces.insert(i, Surf::axis_plane(0, f64::from(i)));
    }
    // 13 two-way unions multiplied out: 2^13 terms.
    let parts = (0..13).map(|i| Expr::Or(vec![Expr::neg(2 * i + 1), Expr::neg(2 * i + 2)])).collect();
    ir.cells.insert(7, Cell::new(0, Expr::And(parts), Some(1)));
    match inst(&ir).unwrap_err() {
        ImportError::TooManyTerms { cell, terms } => assert!(cell == 7 && terms > sm_csgio::MAX_TERMS),
        e => panic!("{e:?}"),
    }
}

#[test]
fn flat_cell_cap_and_hostile_lattice_sizes() {
    let mut ir = CsgIr::new(0);
    leaf_cell(&mut ir, 1, 1);
    ir.lattices.insert(5, Lattice { lower_left: [0.0; 3], pitch: [1.0; 3], dims: [100, 100, 1], universes: vec![1; 10_000], outer: None });
    box_cell(&mut ir, 2, 0, Fill::Lattice { id: 5, xf: Transform::IDENTITY });
    let o = ImportOptions { max_cells: 1000, ..ImportOptions::default() };
    assert_eq!(ImportedCsg::instantiate(&ir, &o).unwrap_err(), ImportError::TooManyCells { limit: 1000 });
    assert!(inst(&ir).is_ok());
    // A lattice that claims far more elements than it lists, or than any cap allows.
    ir.lattices.get_mut(&5).unwrap().dims = [u32::MAX, u32::MAX, 1];
    assert!(matches!(inst(&ir), Err(ImportError::Invalid(_) | ImportError::TooManyCells { .. })));
    ir.lattices.get_mut(&5).unwrap().dims = [100, 100, 1];
    ir.lattices.get_mut(&5).unwrap().universes.pop();
    assert!(matches!(inst(&ir), Err(ImportError::Invalid(_))));
}

#[test]
fn bad_lattice_pitch_and_deep_nesting() {
    let mut ir = CsgIr::new(0);
    leaf_cell(&mut ir, 1, 1);
    ir.lattices.insert(5, Lattice { lower_left: [0.0; 3], pitch: [0.0, 1.0, 1.0], dims: [1, 1, 1], universes: vec![1], outer: None });
    box_cell(&mut ir, 2, 0, Fill::Lattice { id: 5, xf: Transform::IDENTITY });
    assert!(matches!(inst(&ir), Err(ImportError::Invalid(_))));
    // A chain of 70 universes is over the depth cap.
    let mut ir = CsgIr::new(0);
    for u in 0..70 {
        box_cell(&mut ir, 100 + u, u, fill_u(u + 1));
    }
    leaf_cell(&mut ir, 999, 70);
    assert!(matches!(inst(&ir), Err(ImportError::Invalid(m)) if m.contains("deeper")));
}

#[test]
fn empty_decks_and_check_effort_are_refused() {
    let mut ir = CsgIr::new(0);
    ir.materials.insert(1, Material::default());
    ir.surfaces.insert(1, Surf::axis_plane(0, 0.0));
    // The only cell is `+1 -1`: contradictory, so nothing is left.
    ir.cells.insert(1, Cell::new(0, Expr::signed(&[1, -1]), Some(1)));
    assert!(matches!(inst(&ir), Err(ImportError::Invalid(_))));
    let low = ImportOptions { check: CheckOptions { samples: 10, ..CheckOptions::default() }, ..ImportOptions::default() };
    assert!(matches!(ImportedCsg::import(&deck_o(), &low), Err(ImportError::Invalid(_))));
}

#[test]
fn errors_display_the_cells() {
    let e = ImportedCsg::import(&deck_o_overlap(), &ImportOptions::default()).unwrap_err();
    let s = e.to_string();
    assert!(s.contains("overlap") && s.contains('1') && s.contains('2'), "{s}");
}
