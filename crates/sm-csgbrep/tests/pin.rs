//! Deck A: the `pin` fixture (fuel / void gap / clad / moderator in a box).
mod common;
use common::*;
use core::f64::consts::PI;

#[test]
fn pin_volumes_are_analytic() {
    let mut r = Rig::new();
    let geom = fixture("pin", "geometry.xml");
    let mats = fixture("pin", "materials.xml");
    let d = r.import(&geom, Some(&mats)).expect("import");
    println!("{:#?}", d.diagnostics);
    let vols = &d.outputs["deck_volumes"];
    assert_eq!(vols.len(), 4);
    let mut got: Vec<f64> = vols.iter().map(|&v| r.volume(v)).collect();
    got.sort_by(f64::total_cmp);
    let h = 4.0;
    let circle = |a: f64, b: f64| PI * (b * b - a * a) * h;
    let mut want = vec![circle(0.0, 0.4), circle(0.4, 0.42), circle(0.42, 0.46), 1.26 * 1.26 * h - PI * 0.46 * 0.46 * h];
    want.sort_by(f64::total_cmp);
    for (g, w) in got.iter().zip(&want) {
        assert!((g - w).abs() <= 1e-9 * w, "{g} vs {w}");
    }
}
