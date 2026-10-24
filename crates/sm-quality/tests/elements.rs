//! Element-level metrics on ideal elements against analytic values.
#![allow(missing_docs)]

mod common;
use common::*;
use sm_mesh_api::CellShape;
use sm_quality::{Context, Metric, V, eval};

fn ctx_for(shape: CellShape, p: &[V]) -> Context {
    // Reference size = the element's own size, so relative size is 1.
    let v = eval(Metric::Volume, shape, p, &Context::default()).unwrap();
    Context { normal: None, ref_size: v }
}

#[test]
fn ideal_elements_hit_ideal_values() {
    let mut checked = 0;
    for (shape, p) in ideal_elements() {
        let ctx = ctx_for(shape, &p);
        for &m in Metric::ALL {
            let got = eval(m, shape, &p, &ctx);
            assert_eq!(got.is_some(), m.applies(shape), "{m:?} on {shape:?}");
            if let (Some(want), Some(g)) = (m.ideal(shape), got) {
                assert!((g - want).abs() < 1e-12, "{m:?} on {shape:?}: got {g}, want {want}");
                checked += 1;
            }
        }
    }
    assert!(checked > 60, "only {checked} ideal checks ran");
}

#[test]
fn volumes_of_ideal_elements() {
    let c = Context::default();
    let v = |s, p: &[V]| eval(Metric::Volume, s, p, &c).unwrap();
    assert!((v(CellShape::Tri, &tri()) - S3 / 4.0).abs() < 1e-14);
    assert!((v(CellShape::Quad, &quad()) - 1.0).abs() < 1e-14);
    assert!((v(CellShape::Tet, &tet()) - 2.0f64.sqrt() / 12.0).abs() < 1e-14);
    assert!((v(CellShape::Hex, &hex()) - 1.0).abs() < 1e-14);
    assert!((v(CellShape::Wedge, &wedge()) - S3 / 4.0).abs() < 1e-14);
    assert!((v(CellShape::Pyramid, &pyramid()) - 0.5f64.sqrt() / 3.0).abs() < 1e-14);
    // Regular hexagon with circumradius 1: 3 sqrt(3) / 2.
    assert!((v(CellShape::Polygon, &polygon(6)) - 3.0 * S3 / 2.0).abs() < 1e-14);
}
