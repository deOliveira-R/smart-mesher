//! Size fields and auto size.
mod common;

use sm_core::{EntityKind, Point3};
use sm_scheme_api::geom::edge_size_goal;
use sm_scheme_api::stubs::brick_setup;
use sm_scheme_api::{Params, SchemeError, SizingSpec};
use sm_sizing::{SizingError, auto_size, size_field};
use sm_topo_api::{TopologyView, TopologyViewExt};

fn func(name: &str, params: serde_json::Value) -> SizingSpec {
    SizingSpec::Function {
        name: name.into(),
        params: Params(params),
    }
}

fn field(name: &str, params: serde_json::Value) -> Box<dyn sm_scheme_api::SizeField> {
    size_field(&func(name, params))
        .expect("field")
        .expect("some")
}

#[test]
fn constant_goes_through_the_contract_builder() {
    let f = size_field(&SizingSpec::Constant { size: 0.25 })
        .expect("ok")
        .expect("some");
    assert_eq!(f.size(Point3::new(1.0, 2.0, 3.0)), 0.25);
    assert!(size_field(&SizingSpec::Constant { size: -1.0 }).is_err());
}

#[test]
fn linear_size_integral_on_a_unit_edge_is_ln_101() {
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    // h = 0.01 + x: p0 = origin (h 0.01), p1 = (1,0,0) (h 1.01).
    let f = field(
        "linear",
        serde_json::json!({ "p0": [0.0, 0.0, 0.0], "h0": 0.01, "p1": [1.0, 0.0, 0.0], "h1": 1.01 }),
    );
    let goal = edge_size_goal(&topo, ids.edges[0], &*f).expect("goal");
    assert!(
        (goal - 101f64.ln()).abs() < 1e-9 * 101f64.ln(),
        "{goal} vs {}",
        101f64.ln()
    );
    // Held at the end values beyond the segment.
    assert_eq!(f.size(Point3::new(-5.0, 7.0, 0.0)), 0.01);
    assert_eq!(f.size(Point3::new(9.0, -3.0, 2.0)), 1.01);
    assert!((f.size(Point3::new(0.25, 4.0, 4.0)) - 0.26).abs() < 1e-15);
}

#[test]
fn geometric_interpolation_is_exponential_between_the_ends() {
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    let f = field(
        "geometric",
        serde_json::json!({ "p0": [0.0, 0.0, 0.0], "h0": 0.1, "p1": [1.0, 0.0, 0.0], "h1": 0.4 }),
    );
    assert!((f.size(Point3::new(0.5, 0.0, 0.0)) - 0.2).abs() < 1e-15);
    // integral of dx / (h0 r^x) = (1 - 1/r) / (h0 ln r), r = 4.
    let want = (1.0 - 0.25) / (0.1 * 4f64.ln());
    let got = edge_size_goal(&topo, ids.edges[0], &*f).expect("goal");
    assert!((got - want).abs() < 1e-9 * want, "{got} vs {want}");
}

#[test]
fn bias_is_the_continuous_geometric_series() {
    // First element 0.1, ratio 1.5: the element starting at x_k = 0.1 (1.5^k - 1) / 0.5 has size
    // 0.1 * 1.5^k, which is h0 + (ratio - 1) x_k.
    let f = field(
        "bias",
        serde_json::json!({ "origin": [0.0, 0.0, 0.0], "axis": [0.0, 2.0, 0.0], "h0": 0.1, "ratio": 1.5 }),
    );
    for k in 0..6 {
        let x = 0.1 * (1.5f64.powi(k) - 1.0) / 0.5;
        let want = 0.1 * 1.5f64.powi(k);
        assert!(
            (f.size(Point3::new(3.0, x, -2.0)) - want).abs() < 1e-14,
            "k={k}"
        );
    }
    // Behind the origin the size stays h0; a shrinking series is floored at h_min.
    assert_eq!(f.size(Point3::new(0.0, -4.0, 0.0)), 0.1);
    let g = field(
        "bias",
        serde_json::json!({ "origin": [0.0, 0.0, 0.0], "axis": [1.0, 0.0, 0.0], "h0": 1.0, "ratio": 0.5, "h_min": 0.2 }),
    );
    assert_eq!(g.size(Point3::new(100.0, 0.0, 0.0)), 0.2);
    assert!(g.size(Point3::new(1.0, 0.0, 0.0)) > 0.2);
}

#[test]
fn bad_functions_and_parameters_are_typed_errors() {
    for (name, p) in [
        ("nope", serde_json::json!({})),
        ("linear", serde_json::json!({})),
        (
            "linear",
            serde_json::json!({ "p0": [0, 0, 0], "h0": 0.1, "p1": [0, 0, 0], "h1": 0.2 }),
        ),
        (
            "linear",
            serde_json::json!({ "p0": [0, 0, 0], "h0": -0.1, "p1": [1, 0, 0], "h1": 0.2 }),
        ),
        (
            "linear",
            serde_json::json!({ "p0": [0, 0], "h0": 0.1, "p1": [1, 0, 0], "h1": 0.2 }),
        ),
        (
            "geometric",
            serde_json::json!({ "p0": [0, 0, 0], "h0": 0.0, "p1": [1, 0, 0], "h1": 0.2 }),
        ),
        (
            "bias",
            serde_json::json!({ "origin": [0, 0, 0], "axis": [0, 0, 0], "h0": 0.1, "ratio": 1.2 }),
        ),
        (
            "bias",
            serde_json::json!({ "origin": [0, 0, 0], "axis": [1, 0, 0], "h0": 0.1, "ratio": 1.2, "h_min": 2.0, "h_max": 1.0 }),
        ),
    ] {
        match size_field(&func(name, p.clone())) {
            Err(SchemeError::BadParams { reason, .. }) => assert!(!reason.is_empty()),
            other => panic!("{name} {p}: {:?}", other.map(|o| o.is_some())),
        }
    }
}

#[test]
fn auto_size_is_exact_on_a_unit_cube() {
    let (topo, ids, _) = brick_setup([1.0, 1.0, 1.0]);
    // d = sqrt 3, l_min = 1: factors up to 8 sit on the l_min / 4 floor.
    for f in 1..=8 {
        let h = auto_size(&topo, &[ids.volume], f).expect("size");
        assert!((h - 0.25).abs() < 1e-12, "factor {f}: {h}");
    }
    let d = 3f64.sqrt();
    let h9 = auto_size(&topo, &[ids.volume], 9).expect("size");
    assert!((h9 - 0.01 * d * 16.0).abs() < 1e-12);
    let h10 = auto_size(&topo, &[ids.volume], 10).expect("size");
    assert!((h10 - 0.01 * d * 2f64.powf(4.5)).abs() < 1e-12);
    // The documented default factor 5.
    assert!((auto_size(&topo, &[ids.volume], 5).expect("size") - 0.25).abs() < 1e-12);
}

#[test]
fn auto_size_follows_the_formula_when_the_floor_does_not_bind() {
    // A long thin brick: d is dominated by the long edge, l_min by the short ones.
    let (topo, ids, _) = brick_setup([100.0, 100.0, 100.0]);
    let d = 300f64.sqrt() * 10.0; // 100 * sqrt 3
    for f in 1..=10u32 {
        let want = (0.01 * d * 2f64.powf((f64::from(f) - 1.0) / 2.0)).clamp(100.0 / 4.0, d / 2.0);
        let got = auto_size(&topo, &[ids.volume], f).expect("size");
        assert!((got - want).abs() < 1e-9 * want, "{f}: {got} vs {want}");
    }
    // Unconstrained region of the formula: a cube with edge 1000 would have l_min/4 = 250; use
    // a box whose shortest edge is tiny so the floor is far below the formula.
    let (topo, ids, _) = brick_setup([100.0, 100.0, 0.001]);
    let d = (100f64 * 100.0 * 2.0 + 0.001 * 0.001).sqrt();
    let want = 0.01 * d * 2f64.powf(2.0);
    let got = auto_size(&topo, &[ids.volume], 5).expect("size");
    assert!((got - want).abs() < 1e-9 * want, "{got} vs {want}");
}

#[test]
fn auto_size_is_monotone_in_the_factor() {
    for dims in [
        [1.0, 1.0, 1.0],
        [10.0, 2.0, 0.5],
        [100.0, 100.0, 0.001],
        [3.0, 5.2, 4.1],
    ] {
        let (topo, ids, _) = brick_setup(dims);
        let mut prev = 0.0;
        for f in 1..=10u32 {
            let h = auto_size(&topo, &[ids.volume], f).expect("size");
            assert!(
                h >= prev && h > 0.0,
                "{dims:?} factor {f}: {h} after {prev}"
            );
            prev = h;
        }
    }
}

#[test]
fn auto_size_selection_rules_and_errors() {
    let (topo, ids, _) = brick_setup([2.0, 2.0, 2.0]);
    // Volume, a face of it, one edge, and the whole model (empty targets).
    let whole = auto_size(&topo, &[ids.volume], 7).expect("volume");
    assert_eq!(auto_size(&topo, &[], 7).expect("model"), whole);
    assert_eq!(
        auto_size(&topo, &[ids.volume, ids.volume, ids.faces[0]], 7).expect("dedup"),
        whole
    );
    let edge = auto_size(&topo, &[ids.edges[0]], 5).expect("edge");
    // One edge of length 2: d = 2, l_min = 2 -> clamp(0.04 * 2 .. ) floored at 0.5.
    assert!((edge - 0.5).abs() < 1e-12, "{edge}");
    assert_eq!(
        auto_size(&topo, &[ids.volume], 0),
        Err(SizingError::BadFactor { factor: 0 })
    );
    assert_eq!(
        auto_size(&topo, &[ids.volume], 11),
        Err(SizingError::BadFactor { factor: 11 })
    );
    let ghost = sm_core::EntityId::new(EntityKind::TopoVolume, 99).expect("id");
    assert_eq!(
        auto_size(&topo, &[ghost], 5),
        Err(SizingError::UnknownEntity(ghost))
    );
    assert_eq!(
        auto_size(&topo, &[ids.vertices[0]], 5),
        Err(SizingError::NoGeometry)
    );
    let empty = sm_topo_api::CellComplex::new();
    assert_eq!(auto_size(&empty, &[], 5), Err(SizingError::NoGeometry));
    assert!(topo.is_alive(ids.volume) && topo.edge_ends(ids.edges[0]).is_some());
}

#[test]
fn auto_size_counts_the_bulge_of_curved_edges() {
    // A cylinder's circular edges: vertices alone would not span the radius.
    let mut topo = sm_topo_api::CellComplex::new();
    let cyl = sm_topo_api::fixtures::with_op(&mut topo, sm_topo_api::OpId(1), |t| {
        sm_topo_api::fixtures::cylinder(t, 1.0, 4.0)
    })
    .expect("cylinder");
    let h = auto_size(&topo, &[cyl.volume], 10).expect("size");
    // Bounding box is 2 x 2 x 4: d = sqrt 24; shortest edge is the seam of length 4 or the circle 2 pi.
    let d = 24f64.sqrt();
    let want = (0.01 * d * 2f64.powf(4.5)).clamp(4.0 / 4.0, d / 2.0);
    assert!((h - want).abs() < 2e-2 * want, "{h} vs {want}");
}

/// A lone closed circular edge has length 2 pi r > 2 d (d is about 2.83 r): the clamp bounds
/// cross. This used to panic; the lower bound `l_min / 4` now wins.
#[test]
fn auto_size_on_a_closed_circular_edge_does_not_panic() {
    let mut topo = sm_topo_api::CellComplex::new();
    let cyl = sm_topo_api::fixtures::with_op(&mut topo, sm_topo_api::OpId(1), |t| {
        sm_topo_api::fixtures::cylinder(t, 1.0, 4.0)
    })
    .expect("cylinder");
    for factor in 1..=10 {
        let h = auto_size(&topo, &[cyl.edges[0]], factor).expect("size");
        assert!(
            (h - std::f64::consts::PI / 2.0).abs() < 1e-6,
            "factor {factor}: {h}"
        );
    }
}
