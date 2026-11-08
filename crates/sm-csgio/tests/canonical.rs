//! Canonical order (acceptance 22): the flat model's hash and chords are invariant under surface
//! renumbering, cell id permutation, and writing a shared plane once or once per use.
mod common;
use common::*;
use proptest::prelude::*;
use sm_csgio::ir::{BoundaryFlag, Cell, CsgIr, Expr, Material, Surf};
use sm_csgio::{ImportOptions, ImportedCsg};

const RADII: [f64; 3] = [0.2, 0.3, 0.4];

/// A row of boxes along x, widths `w`, each with a fuel cylinder of radius `RADII[r]` on its axis.
/// `perm_seed` permutes surface and cell ids, `dup` writes every plane once per use.
fn row(boxes: &[(u32, usize)], perm_seed: u64, dup: bool) -> CsgIr {
    // Logical surfaces: planes x_0..x_n, y -1, y 1, z -1, z 1, then one cylinder per box.
    let n = boxes.len();
    let mut xs = vec![0.0];
    for (w, _) in boxes {
        xs.push(xs.last().unwrap() + f64::from(*w));
    }
    let mut ids: Vec<u32> = (1..=(100 + 3 * n as u32)).collect();
    if perm_seed != 0 {
        let mut s = perm_seed;
        for i in (1..ids.len()).rev() {
            s = s.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
            ids.swap(i, (s >> 33) as usize % (i + 1));
        }
    }
    let mut next = 0usize;
    let mut fresh = || {
        next += 1;
        ids[next - 1]
    };
    let mut ir = CsgIr::new(0);
    ir.materials.insert(1, Material { name: Some("fuel".into()) });
    ir.materials.insert(2, Material { name: Some("water".into()) });
    let xplane: Vec<u32> = (0..=n).map(|_| fresh()).collect();
    for (i, id) in xplane.iter().enumerate() {
        ir.surfaces.insert(*id, Surf::axis_plane(0, xs[i]).with_flag(if i == 0 || i == n { BoundaryFlag::Vacuum } else { BoundaryFlag::None }));
    }
    let side: Vec<u32> = (0..4).map(|_| fresh()).collect();
    for (k, (axis, c)) in [(1, -1.0), (1, 1.0), (2, -1.0), (2, 1.0)].into_iter().enumerate() {
        ir.surfaces.insert(side[k], Surf::axis_plane(axis, c));
    }
    for (i, (w, r)) in boxes.iter().enumerate() {
        let cyl = fresh();
        ir.surfaces.insert(cyl, Surf::cylinder(2, [xs[i] + f64::from(*w) / 2.0, 0.0, 0.0], RADII[*r]).unwrap());
        let (lo, hi) = if dup {
            let (a, b) = (fresh(), fresh());
            let flag = |edge: bool| if edge { BoundaryFlag::Vacuum } else { BoundaryFlag::None };
            ir.surfaces.insert(a, Surf::axis_plane(0, xs[i]).with_flag(flag(i == 0)));
            ir.surfaces.insert(b, Surf::axis_plane(0, xs[i + 1]).with_flag(flag(i + 1 == n)));
            (a, b)
        } else {
            (xplane[i], xplane[i + 1])
        };
        let b = |extra: i64| Expr::signed(&[i64::from(lo), -i64::from(hi), i64::from(side[0]), -i64::from(side[1]), i64::from(side[2]), -i64::from(side[3]), extra]);
        // Cell ids follow the same permutation so they are not in text order either.
        let (fid, mid) = (ids[ids.len() - 1 - 2 * i], ids[ids.len() - 2 - 2 * i]);
        ir.cells.insert(fid, Cell::new(0, b(-i64::from(cyl)), Some(1)));
        ir.cells.insert(mid, Cell::new(0, b(i64::from(cyl)), Some(2)));
    }
    ir
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(24))]
    #[test]
    fn hash_and_chords_do_not_depend_on_ids_or_text_order(
        boxes in prop::collection::vec((1u32..=3, 0usize..3), 1..=4),
        seed in 1u64..u64::MAX,
        dup in any::<bool>(),
    ) {
        let base = ImportedCsg::instantiate(&row(&boxes, 0, false), &ImportOptions::default()).unwrap();
        let var = ImportedCsg::instantiate(&row(&boxes, seed, dup), &ImportOptions::default()).unwrap();
        prop_assert_eq!(base.content_hash(), var.content_hash());
        prop_assert_eq!(base.cell_count(), var.cell_count());
        prop_assert_eq!(base.surface_count(), var.surface_count());
        let (tb, tv) = (tracker(&base), tracker(&var));
        let total: f64 = boxes.iter().map(|(w, _)| f64::from(*w)).sum();
        for k in 0..7 {
            let y = -0.9 + 1.8 * f64::from(k) / 6.0;
            let o = [-0.5, y, 0.1];
            // Start inside the model: the first box begins at x = 0, so start at x = 0.25.
            let o = [0.25, o[1], o[2]];
            let (cb, cv) = (chords(&tb, o, [1.0, 0.0, 0.0]), chords(&tv, o, [1.0, 0.0, 0.0]));
            prop_assert_eq!(&cb, &cv);
            let sum: f64 = cb.values().sum();
            prop_assert!((sum - (total - 0.25)).abs() < 1e-12);
        }
    }
}

#[test]
fn dedup_and_renumbering_on_deck_o() {
    let base = deck_o();
    let mut renum = CsgIr::new(0);
    // The same deck with every surface id shifted and cells reversed.
    fn shift(e: &Expr) -> Expr {
        match e {
            Expr::Half(s, g) => Expr::Half(s + 40, *g),
            Expr::And(v) => Expr::And(v.iter().map(shift).collect()),
            _ => unreachable!(),
        }
    }
    renum.materials = base.materials.clone();
    for (id, s) in &base.surfaces {
        renum.surfaces.insert(id + 40, s.clone());
    }
    for (id, c) in base.cells.iter().rev() {
        renum.cells.insert(100 - id, Cell { region: shift(&c.region), ..c.clone() });
    }
    let h = |ir: &CsgIr| ImportedCsg::instantiate(ir, &ImportOptions::default()).unwrap().content_hash();
    assert_eq!(h(&base), h(&renum));
}
