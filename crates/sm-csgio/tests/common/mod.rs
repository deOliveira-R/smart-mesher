//! Shared fixtures: hand-built decks and chord tracing.
#![allow(dead_code)]

use sm_core::{Point3, Vector3};
use sm_csgio::ir::{BoundaryFlag, Cell, CsgIr, Expr, Material, Surf};
use sm_csgio::{ImportOptions, ImportedCsg};
use sm_rt::AnalyticTracker;
use sm_rt_api::{BuildOptions, Flow, Scratch, Tracker, TraceOptions};
use std::collections::BTreeMap;

/// The geometry of OpenMC's "sphere in a box" deck: a unit sphere (material 1) in a box
/// `[-2, 2]^3` (material 2) with vacuum walls. `cell2_region` is the box cell's region.
pub fn deck_o_with(cell2_region: Expr, extra: impl FnOnce(&mut CsgIr)) -> CsgIr {
    let mut ir = CsgIr::new(0);
    ir.surfaces.insert(1, Surf::sphere([0.0; 3], 1.0).unwrap());
    for (id, axis, c) in [(2, 0, -2.0), (3, 0, 2.0), (4, 1, -2.0), (5, 1, 2.0), (6, 2, -2.0), (7, 2, 2.0)] {
        ir.surfaces.insert(id, Surf::axis_plane(axis, c).with_flag(BoundaryFlag::Vacuum));
    }
    ir.materials.insert(1, Material::default());
    ir.materials.insert(2, Material::default());
    ir.cells.insert(1, Cell::new(0, Expr::signed(&[-1]), Some(1)));
    ir.cells.insert(2, Cell::new(0, cell2_region, Some(2)));
    extra(&mut ir);
    ir
}

/// Deck O as written: cell 2 is `1 2 -3 4 -5 6 -7`.
pub fn deck_o() -> CsgIr {
    deck_o_with(Expr::signed(&[1, 2, -3, 4, -5, 6, -7]), |_| {})
}

/// Deck O with cell 2 missing the `1` (it overlaps the sphere).
pub fn deck_o_overlap() -> CsgIr {
    deck_o_with(Expr::signed(&[2, -3, 4, -5, 6, -7]), |_| {})
}

/// Deck O with a small sphere carved out of cell 2 and claimed by no cell (a gap inside the bounds).
pub fn deck_o_gap() -> CsgIr {
    deck_o_with(Expr::signed(&[1, 2, -3, 4, -5, 6, -7, 8]), |ir| {
        ir.surfaces.insert(8, Surf::sphere([1.4, 1.4, 1.4], 0.3).unwrap());
    })
}

/// One cell `-1 2 -3` in a vacuum z-cylinder (r=1) between vacuum planes z=+-1, plus an inner
/// fuel cell when `two` (the pin-cell shape). The outer boundary is not the bounding box.
pub fn deck_cyl(two: bool) -> CsgIr {
    let mut ir = CsgIr::new(0);
    ir.surfaces.insert(1, Surf::cylinder(2, [0.0; 3], 1.0).unwrap().with_flag(BoundaryFlag::Vacuum));
    ir.surfaces.insert(2, Surf::axis_plane(2, -1.0).with_flag(BoundaryFlag::Vacuum));
    ir.surfaces.insert(3, Surf::axis_plane(2, 1.0).with_flag(BoundaryFlag::Vacuum));
    ir.materials.insert(1, Material::default());
    ir.materials.insert(2, Material::default());
    if two {
        ir.surfaces.insert(4, Surf::cylinder(2, [0.0; 3], 0.5).unwrap());
        ir.cells.insert(1, Cell::new(0, Expr::signed(&[-4, 2, -3]), Some(1)));
        ir.cells.insert(2, Cell::new(0, Expr::signed(&[4, -1, 2, -3]), Some(2)));
    } else {
        ir.cells.insert(1, Cell::new(0, Expr::signed(&[-1, 2, -3]), Some(1)));
    }
    ir
}

/// A fuel sphere in a water sphere (vacuum, r=2): the outer boundary is curved.
pub fn deck_sph() -> CsgIr {
    let mut ir = CsgIr::new(0);
    ir.surfaces.insert(1, Surf::sphere([0.0; 3], 1.0).unwrap());
    ir.surfaces.insert(2, Surf::sphere([0.0; 3], 2.0).unwrap().with_flag(BoundaryFlag::Vacuum));
    ir.materials.insert(1, Material::default());
    ir.materials.insert(2, Material::default());
    ir.cells.insert(1, Cell::new(0, Expr::signed(&[-1]), Some(1)));
    ir.cells.insert(2, Cell::new(0, Expr::signed(&[1, -2]), Some(2)));
    ir
}

pub fn import(ir: &CsgIr) -> ImportedCsg {
    ImportedCsg::import(ir, &ImportOptions::default()).unwrap()
}

pub fn tracker(m: &ImportedCsg) -> AnalyticTracker {
    m.tracker(&BuildOptions::default()).unwrap().0
}

/// Chord length per region name along the ray from `o` in direction `d` (unit), to the end of the
/// trace. A start outside the model gives an empty map.
pub fn chords(t: &AnalyticTracker, o: [f64; 3], d: [f64; 3]) -> BTreeMap<String, f64> {
    let mut sc = Scratch::new();
    let mut st = t.init_state(Point3::new(o[0], o[1], o[2]), Vector3::new(d[0], d[1], d[2]), None, &mut sc).unwrap();
    let mut out: BTreeMap<String, f64> = BTreeMap::new();
    let mut sink = |s: &sm_rt_api::Segment| {
        let name = t.regions().get(s.region).map(|r| r.name.clone()).unwrap_or_default();
        *out.entry(name).or_default() += s.length;
        Flow::Continue
    };
    t.trace(&mut st, &TraceOptions::default(), &mut sink, &mut sc).unwrap();
    out
}

pub fn chord(t: &AnalyticTracker, o: [f64; 3], d: [f64; 3], name: &str) -> f64 {
    chords(t, o, d).get(name).copied().unwrap_or(0.0)
}
