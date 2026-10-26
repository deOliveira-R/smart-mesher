//! Acceptance 9: quadric-quadric intersections traced by marching.

mod common;

use common::rig::*;
use common::*;
use sm_boolean::ops::{Intersect, Subtract, Unite};
use sm_model::Rotate;
use sm_topo_api::{Dim, TopologyView, TopologyViewExt};

fn crossing(r: &mut Rig) -> (Solid, Solid) {
    let a = r.cylinder(4.0, 1.0, [0.0; 3]);
    let b = r.cylinder(4.0, 0.5, [0.0; 3]);
    r.run(&Rotate { entities: vec![b.body], axis: [0.0, 1.0, 0.0], angle_deg: 90.0, origin: None }).unwrap();
    (a, b)
}

fn volume_after(f: impl FnOnce(&mut Rig, &Solid, &Solid) -> sm_op_api::Delta) -> (f64, f64) {
    let mut r = Rig::cc();
    let (a, b) = crossing(&mut r);
    let d = f(&mut r, &a, &b);
    let t = r.topo();
    assert_valid(t);
    let v: f64 = d.outputs["volumes"].iter().map(|&v| volume_of(t, v)).sum();
    (v, total_area(t))
}

#[test]
fn inclusion_exclusion_on_crossing_cylinders() {
    let va = std::f64::consts::PI * 1.0 * 4.0;
    let vb = std::f64::consts::PI * 0.25 * 4.0;
    let (vi, _) = volume_after(|r, a, b| r.run(&Intersect { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap());
    let (vu, _) = volume_after(|r, a, b| r.run(&Unite { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap());
    let (vs, _) = volume_after(|r, a, b| r.run(&Subtract { targets: vec![a.body], tools: vec![b.body], keep: None, regularize: None, tol: None }).unwrap());
    // Reference value by quadrature: the thin cylinder's cross-section disk times the chord of
    // the fat cylinder.
    let n = 20000;
    let mut want = 0.0;
    for k in 0..n {
        let y = -0.5 + (k as f64 + 0.5) / n as f64;
        want += 2.0 * (1.0 - y * y).sqrt() * 2.0 * (0.25 - y * y).max(0.0).sqrt() / n as f64;
    }
    assert!((vi - want).abs() < 1e-5 * want, "intersection {vi} vs quadrature {want}");
    assert!((vu - (va + vb - vi)).abs() < 1e-5 * (va + vb), "union {vu} vs {}", va + vb - vi);
    assert!((vs - (va - vi)).abs() < 1e-5 * va, "subtract {vs} vs {}", va - vi);
    let _ = Dim::Face;
}

#[test]
fn intersection_edges_lie_on_both_surfaces() {
    use sm_geom_api::SurfaceDef;
    use sm_topo_api::{Carrier, Dim};
    let mut r = Rig::cc();
    let (a, b) = crossing(&mut r);
    r.run(&Unite { bodies: vec![a.body, b.body], keep: None, regularize: None, tol: None }).unwrap();
    let t = r.topo();
    let mut checked = 0;
    for &e in t.cells(Dim::Edge) {
        let faces: Vec<_> = t.coboundary(e).iter().filter(|i| t.dim_of(i.cell) == Some(Dim::Face)).map(|i| i.cell).collect();
        let surfs: Vec<sm_core::EntityId> = faces.iter().filter_map(|&f| match t.carrier(f) { Carrier::Surface(s) => Some(s.surface), _ => None }).collect();
        let cyl: Vec<_> = surfs.iter().filter(|&&s| matches!(t.shapes().surface_def(s), Some(SurfaceDef::Cylinder(_)))).copied().collect();
        let (Carrier::Curve(cc), true) = (t.carrier(e), cyl.len() >= 2 && cyl[0] != cyl[1]) else { continue };
        let curve = t.shapes().curve(cc.curve).unwrap();
        for k in 0..=20 {
            let p = curve.eval(cc.range.lo + (cc.range.hi - cc.range.lo) * k as f64 / 20.0);
            for &s in &cyl[..2] {
                let imp = t.shapes().implicit(s).unwrap();
                let d = imp.value(p).abs() / imp.gradient(p).norm();
                assert!(d < 1e-6, "edge {e} point {p:?} is {d:e} off a cylinder");
            }
        }
        checked += 1;
    }
    assert!(checked >= 4, "{checked} intersection edges checked");
}
