//! Acceptance 13: twisted prisms of random size, twist, height, layers and source resolution.

mod common;

use common::{hex_gauss_volume, mesh_with, min_scaled_jacobian, registry};
use proptest::prelude::*;
use sm_mesh_api::validate;
use sm_model_api::Model;
use sm_scheme_api::{Firm, Firmness, IntervalRequest, MeshIntent, SchemeChoice, Sequential};
use sm_testkit::topo::twisted_prism;

/// Sweeps the twisted prism and checks validity, the exact volume (the 2x2x2 Gauss sum of a
/// conforming trilinear mesh equals the volume bounded by its boundary bilinear patches, which
/// are exact sub-patches of the fixture faces) and positive Jacobians.
fn check(twist_deg: f64, side: f64, h: f64, layers: u32, m: u32) -> Result<(), String> {
    let twist = twist_deg.to_radians();
    let fx = twisted_prism(side, h, twist).map_err(|e| e.to_string())?;
    let topo = &fx.topo.complex;
    let mut intent = MeshIntent::default();
    for e in sm_topo_api::TopologyView::face_edges(topo, fx.source).to_vec() {
        intent.set_intervals(e, Firm::hard(IntervalRequest::Count(m)));
    }
    for e in &fx.columns {
        intent.set_intervals(*e, Firm::hard(IntervalRequest::Count(layers)));
    }
    intent.set_scheme(
        fx.volume,
        SchemeChoice::with_params(
            "sweep",
            serde_json::json!({ "source": [fx.source], "target": [fx.target] }),
            Firmness::Hard,
        ),
    );
    intent.set_scheme(fx.source, SchemeChoice::new("map", Firmness::Hard));
    let out = mesh_with(
        &registry(),
        topo,
        &Model::new(),
        &[fx.volume],
        &intent,
        &Sequential,
    )
    .map_err(|e| e.to_string())?;
    let mesh = &out.finished.mesh;
    if !validate(mesh).is_empty() {
        return Err("invalid mesh".into());
    }
    let (vol, least) = hex_gauss_volume(mesh);
    let exact = side * side * h * (2.0 + twist.cos()) / 3.0;
    if (vol - exact).abs() / exact >= 1e-10 {
        return Err(format!("volume {vol} vs {exact}"));
    }
    if least <= 0.0 {
        return Err(format!("Gauss-point Jacobian {least}"));
    }
    if min_scaled_jacobian(mesh) <= 0.0 {
        return Err("non-positive scaled Jacobian".into());
    }
    Ok(())
}

/// CI regression: at high twist the linking-face projection used to stop at a 1/16 seed-grid
/// sample, so lateral nodes left the bilinear iso-lines and the volume was off by 3.3e-3.
#[test]
fn twisted_prism_high_twist_regression() {
    check(120.1275823096648, 2.447990487719083, 0.5, 5, 4).unwrap();
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(48))]

    #[test]
    fn twisted_prism_volume_and_jacobians(
        twist_deg in 0.0f64..150.0,
        side in 0.5f64..3.0,
        h in 0.5f64..4.0,
        layers in 1u32..8,
        m in 1u32..6,
    ) {
        let r = check(twist_deg, side, h, layers, m);
        prop_assert!(r.is_ok(), "{:?}", r);
    }
}
