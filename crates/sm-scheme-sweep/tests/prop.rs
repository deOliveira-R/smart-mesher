//! Acceptance 13: twisted prisms of random size, twist, height, layers and source resolution.

mod common;

use common::{hex_gauss_volume, mesh_with, min_scaled_jacobian, registry};
use proptest::prelude::*;
use sm_mesh_api::validate;
use sm_model_api::Model;
use sm_scheme_api::{Firm, Firmness, IntervalRequest, MeshIntent, SchemeChoice, Sequential};
use sm_testkit::topo::twisted_prism;

proptest! {
    #![proptest_config(ProptestConfig { cases: 48, ..ProptestConfig::default() })]

    #[test]
    fn twisted_prism_volume_and_jacobians(
        twist_deg in 0.0f64..150.0,
        side in 0.5f64..3.0,
        h in 0.5f64..4.0,
        layers in 1u32..8,
        m in 1u32..6,
    ) {
        let twist = twist_deg.to_radians();
        let fx = twisted_prism(side, h, twist).expect("twisted");
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
        let out = mesh_with(&registry(), topo, &Model::new(), &[fx.volume], &intent, &Sequential)
            .expect("mesh");
        let mesh = &out.finished.mesh;
        prop_assert!(validate(mesh).is_empty());
        let (vol, least) = hex_gauss_volume(mesh);
        let exact = side * side * h * (2.0 + twist.cos()) / 3.0;
        prop_assert!((vol - exact).abs() / exact < 1e-10, "volume {} vs {}", vol, exact);
        prop_assert!(least > 0.0, "Gauss-point Jacobian {}", least);
        prop_assert!(min_scaled_jacobian(mesh) > 0.0);
    }
}
