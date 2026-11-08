//! A stepped vacuum boundary must not hide overlaps behind `<outside>` report slots.
use sm_csgio::ir::{BoundaryFlag, Cell, CsgIr, Expr, Material, Surf};
use sm_csgio::{ImportError, ImportOptions, ImportedCsg};

/// A vacuum z-cylinder core (r=1, z in [0, 2]) on `n` slab cells over z in [-1, 0], so the
/// cylinder is flagged and one-sided while many real cells lie beyond it. Cell `n + 2`
/// overlaps the core (z in [1.5, 2]). With `<outside>` pairs competing for the 16 report slots
/// of the sampled check, the real pair used to be lost.
fn deck_stepped(n: u32) -> CsgIr {
    let mut ir = CsgIr::new(0);
    ir.surfaces.insert(1, Surf::cylinder(2, [0.0; 3], 1.0).unwrap().with_flag(BoundaryFlag::Vacuum));
    ir.surfaces.insert(3, Surf::axis_plane(2, 0.0));
    ir.surfaces.insert(4, Surf::axis_plane(2, 2.0).with_flag(BoundaryFlag::Vacuum));
    ir.surfaces.insert(5, Surf::axis_plane(2, -1.0).with_flag(BoundaryFlag::Vacuum));
    ir.surfaces.insert(6, Surf::axis_plane(1, -2.0).with_flag(BoundaryFlag::Vacuum));
    ir.surfaces.insert(7, Surf::axis_plane(1, 2.0).with_flag(BoundaryFlag::Vacuum));
    ir.surfaces.insert(9, Surf::axis_plane(2, 1.5));
    ir.materials.insert(1, Material::default());
    ir.cells.insert(1, Cell::new(0, Expr::signed(&[-1, 3, -4]), Some(1)));
    for i in 0..=n {
        let c = -2.0 + 4.0 * f64::from(i) / f64::from(n);
        let flag = if i == 0 || i == n { BoundaryFlag::Vacuum } else { BoundaryFlag::None };
        ir.surfaces.insert(100 + i, Surf::axis_plane(0, c).with_flag(flag));
    }
    for i in 0..n {
        let (lo, hi) = (i64::from(100 + i), i64::from(101 + i));
        ir.cells.insert(2 + i, Cell::new(0, Expr::signed(&[lo, -hi, 6, -7, 5, -3]), Some(1)));
    }
    ir.cells.insert(2 + n, Cell::new(0, Expr::signed(&[-1, 9, -4]), Some(1)));
    ir
}

#[test]
fn overlap_survives_many_cells_beyond_a_flagged_surface() {
    for n in [4, 16, 20] {
        let e = ImportedCsg::import(&deck_stepped(n), &ImportOptions::default()).unwrap_err();
        let ImportError::Overlap { findings } = e else { panic!("n = {n}: expected Overlap, got {e:?}") };
        assert!(!findings.is_empty());
        for f in &findings {
            let mut cells = [f.a.cell, f.b.cell];
            cells.sort_unstable();
            assert_eq!(cells, [1, 2 + n], "n = {n}: only the real pair is an overlap");
        }
    }
}
