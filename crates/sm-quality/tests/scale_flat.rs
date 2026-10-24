//! Scale invariance over 1e-12 .. 1e12 and the flat/collapsed 3-D cells' dihedral angles.
#![allow(missing_docs)]

mod common;
use common::*;
use sm_mesh_api::CellShape;
use sm_quality::{Context, Metric, V, eval};

fn scaled(p: &[V], s: f64) -> Vec<V> {
    p.iter().map(|q| [q[0] * s, q[1] * s, q[2] * s]).collect()
}

fn e(m: Metric, shape: CellShape, p: &[V]) -> f64 {
    eval(m, shape, p, &Context::default()).unwrap()
}

#[test]
fn ideal_elements_keep_their_ideal_metrics_at_any_scale() {
    for &s in &[1e-12, 1e-9, 1e-5, 1.0, 1e6, 1e12] {
        for (shape, base) in ideal_elements() {
            let p = scaled(&base, s);
            let dim = if shape.dim() == sm_mesh_api::MeshDim::Two { 2 } else { 3 };
            let c = Context { normal: None, ref_size: e(Metric::Volume, shape, &base) * s.powi(dim) };
            for &m in Metric::ALL.iter().filter(|m| m.applies(shape)) {
                let v = eval(m, shape, &p, &c).unwrap();
                let w = eval(m, shape, &base, &Context { normal: None, ref_size: e(Metric::Volume, shape, &base) }).unwrap();
                let w = match m {
                    Metric::Volume | Metric::Jacobian => w * s.powi(dim),
                    Metric::Dimension => w * s,
                    _ => w,
                };
                assert!((v - w).abs() <= 1e-9 * (1.0 + w.abs()), "{m:?} on {shape:?} at scale {s}: {v} vs {w}");
            }
        }
    }
    let cube = scaled(&hex(), 1e-12);
    assert!((e(Metric::Condition, CellShape::Hex, &cube) - 1.0).abs() < 1e-12);
    assert!((e(Metric::ScaledJacobian, CellShape::Hex, &cube) - 1.0).abs() < 1e-12);
    assert!((e(Metric::Dimension, CellShape::Hex, &cube) - 3f64.sqrt() / 3.0 * 1e-12).abs() < 1e-24);
}

#[test]
fn a_truly_degenerate_element_is_still_degenerate_at_any_scale() {
    for &s in &[1e-12, 1.0, 1e12] {
        let mut p = scaled(&tet(), s);
        p[3] = p[2];
        assert!(e(Metric::Volume, CellShape::Tet, &p).abs() <= 1e-12 * s.powi(3));
        assert_eq!(e(Metric::Condition, CellShape::Tet, &p), sm_quality::DBL_MAX);
    }
}

#[test]
fn flat_hex_and_collapsed_tet_are_not_reported_as_flat_dihedrals() {
    // Flat hex: the top face lies on the bottom face.
    let mut flat = hex();
    for q in flat.iter_mut() {
        q[2] = 0.0;
    }
    assert_eq!(e(Metric::MinAngle, CellShape::Hex, &flat), 0.0);
    assert_eq!(e(Metric::MaxAngle, CellShape::Hex, &flat), 180.0);
    // Collapsed tet: all four vertices coplanar and one edge-sharing face of zero area.
    let mut t = tet();
    t[3] = [0.25, 0.0, 0.0]; // on edge 0-1
    assert_eq!(e(Metric::MinAngle, CellShape::Tet, &t), 0.0);
    assert_eq!(e(Metric::MaxAngle, CellShape::Tet, &t), 180.0);
    // A healthy cell is unaffected.
    assert!((e(Metric::MinAngle, CellShape::Hex, &hex()) - 90.0).abs() < 1e-9);
}
