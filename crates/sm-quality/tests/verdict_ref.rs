//! Differential fixture: reference values of distorted elements computed from the Verdict C++
//! formulas (reviewed in W1 fix round 0). Each row pins one metric on one element to the value
//! the Verdict formula produces; the ideal-element and closed-form tests alone would not catch
//! a wrong convention. Tolerance `1e-4` where the reference is a rounded literal, else tight.
#![allow(missing_docs)]

mod common;
use common::*;
use sm_mesh_api::CellShape as S;
use sm_quality::{Context, DBL_MAX, Metric as M, V, eval};

fn e(m: M, s: S, p: &[V]) -> f64 {
    eval(m, s, p, &Context::default()).unwrap_or_else(|| panic!("{m:?} undefined for {s:?}"))
}
fn with_ref(m: M, s: S, p: &[V], r: f64) -> f64 {
    eval(m, s, p, &Context { normal: None, ref_size: r }).unwrap()
}
fn near(got: f64, want: f64, tol: f64, what: &str) {
    assert!((got - want).abs() <= tol * want.abs().max(1.0), "{what}: got {got}, want {want}");
}

fn brick(a: f64, b: f64, c: f64) -> Vec<V> {
    hex().iter().map(|q| [a * q[0], b * q[1], c * q[2]]).collect()
}
fn tapered_hex(t: f64) -> Vec<V> {
    hex().iter().map(|q| if q[2] > 0.5 { [0.5 + t * (q[0] - 0.5), 0.5 + t * (q[1] - 0.5), 1.0] } else { *q }).collect()
}
fn right_wedge() -> Vec<V> {
    vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [0., 1., 1.]]
}
fn tall_pyramid() -> Vec<V> {
    vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0.5, 0.5, 2.0]]
}

#[test]
fn hex_aspect_frobenius_is_condition() {
    let p = brick(1.0, 1.0, 2.0);
    near(e(M::MaxAspectFrobenius, S::Hex, &p), 1.5f64.sqrt(), 1e-12, "max af");
    near(e(M::Condition, S::Hex, &p), 1.5f64.sqrt(), 1e-12, "condition");
    // All eight corners of a brick agree, so the mean equals the max.
    near(e(M::MeanAspectFrobenius, S::Hex, &p), 1.5f64.sqrt(), 1e-12, "mean af");
    let t = tapered_hex(0.5);
    assert!(e(M::MeanAspectFrobenius, S::Hex, &t) < e(M::MaxAspectFrobenius, S::Hex, &t));
    near(e(M::Condition, S::Hex, &t), e(M::MaxAspectFrobenius, S::Hex, &t), 1e-14, "hex condition == max af");
}

#[test]
fn hex_dimension_is_pronto_characteristic_length() {
    near(e(M::Dimension, S::Hex, &hex()), 3f64.sqrt() / 3.0, 1e-12, "unit cube");
    // V / sqrt(b^2 c^2 + a^2 c^2 + a^2 b^2) for a brick.
    near(e(M::Dimension, S::Hex, &brick(1.0, 2.0, 4.0)), 8.0 / 84f64.sqrt(), 1e-12, "1x2x4");
}

#[test]
fn hex_oddy_rejects_inverted_cells() {
    let mirrored: Vec<V> = hex().iter().map(|q| [-q[0], q[1], q[2]]).collect();
    assert_eq!(e(M::Oddy, S::Hex, &mirrored), DBL_MAX);
    near(e(M::Oddy, S::Hex, &hex()), 0.0, 1e-12, "unit cube");
}

#[test]
fn hex_relative_size_uses_mean_corner_determinant() {
    let t = tapered_hex(0.5);
    let vol = e(M::Volume, S::Hex, &t);
    // Mean corner det = (4 * 1 + 4 * 0.25) / 8 = 0.625; tau = 0.625 / vol.
    let tau: f64 = 0.625 / vol;
    near(with_ref(M::RelativeSizeSquared, S::Hex, &t, vol), tau.min(1.0 / tau).powi(2), 1e-12, "rss");
    near(with_ref(M::RelativeSizeSquared, S::Hex, &t, vol), 0.871, 1e-3, "rss literal");
}

#[test]
fn distortion_samples_nodes_as_well_as_gauss_points() {
    let trap: Vec<V> = vec![[0., 0., 0.], [2., 0., 0.], [1.5, 1., 0.], [0.5, 1., 0.]];
    near(e(M::Distortion, S::Quad, &trap), 2.0 / 3.0, 1e-12, "trapezoid quad");
    // Top face scaled by 1/2: smallest corner det 0.25 over the mean Gauss det (= V).
    let t = tapered_hex(0.5);
    near(e(M::Distortion, S::Hex, &t), 0.25 / e(M::Volume, S::Hex, &t), 1e-12, "tapered hex");
    assert!(e(M::Distortion, S::Hex, &t) <= 0.429);
}

#[test]
fn wedge_distortion_is_min_corner_det_over_twice_volume() {
    near(e(M::Distortion, S::Wedge, &wedge()), 1.0, 1e-12, "ideal");
    // Top face scaled by 1/2 about its centroid: corner dets are the base area-ish products.
    let p = right_wedge();
    let q: Vec<V> = p.iter().map(|v| if v[2] > 0.5 { [v[0] * 0.5, v[1] * 0.5, 1.0] } else { *v }).collect();
    let vol = e(M::Volume, S::Wedge, &q);
    // Corners at the apex-side vertices have det 0.25 (edges 0.5, 0.5, 1); base corners have 1.
    near(e(M::Distortion, S::Wedge, &q), 0.25 / (2.0 * vol), 1e-12, "tapered wedge");
}

#[test]
fn wedge_shape_and_aspect_frobenius_follow_verdict() {
    let p = right_wedge();
    near(e(M::Shape, S::Wedge, &p), 0.75 * (2.0 / 3f64.sqrt()).powf(2.0 / 3.0), 1e-12, "shape formula");
    near(e(M::Shape, S::Wedge, &p), 0.8254, 1e-4, "shape");
    near(e(M::MaxAspectFrobenius, S::Wedge, &p), 1.1357, 1e-4, "max af");
    near(e(M::MeanAspectFrobenius, S::Wedge, &p), 1.0978, 1e-4, "mean af");
    near(e(M::Condition, S::Wedge, &p), 1.1357, 1e-4, "condition");
    assert_eq!(e(M::Condition, S::Wedge, &p), e(M::MaxAspectFrobenius, S::Wedge, &p));
}

#[test]
fn pyramid_scaled_jacobian_and_shape_follow_verdict() {
    let p = tall_pyramid();
    // min tet SJ (sqrt2 * 2 / 4.5^1.5) times sqrt2.
    near(e(M::ScaledJacobian, S::Pyramid, &p), 4.0 / 4.5f64.powf(1.5), 1e-12, "sj");
    near(e(M::ScaledJacobian, S::Pyramid, &p), 0.419, 1e-3, "sj literal");
    near(e(M::Shape, S::Pyramid, &p), 0.75, 1e-12, "shape");
    // Folded into [0, 1]: a flat-ish short pyramid and an inverted one stay in range.
    let mut inv = p.clone();
    inv[4][2] = -2.0;
    assert_eq!(e(M::ScaledJacobian, S::Pyramid, &inv), 0.0);
    assert_eq!(e(M::Shape, S::Pyramid, &inv), 0.0);
    for h in [0.2, 0.7, 1.5, 3.0] {
        let mut q = pyramid();
        q[4][2] = h;
        let sj = e(M::ScaledJacobian, S::Pyramid, &q);
        assert!((0.0..=1.0).contains(&sj), "sj {sj} at h={h}");
    }
}

#[test]
fn collapsed_quad_routes_to_triangle_metrics() {
    let q: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [1., 1., 0.]];
    near(e(M::ScaledJacobian, S::Quad, &q), 0.5f64.sqrt() * 2.0 / 3f64.sqrt(), 1e-12, "sj");
    near(e(M::Jacobian, S::Quad, &q), 1.0, 1e-12, "jacobian");
    near(e(M::Condition, S::Quad, &q), 4.0 / (4.0 * 3f64.sqrt() * 0.5), 1e-12, "condition");
    near(e(M::MinAngle, S::Quad, &q), 45.0, 1e-12, "min angle");
    near(e(M::MaxAngle, S::Quad, &q), 90.0, 1e-12, "max angle");
}

#[test]
fn quad_skew_of_a_degenerate_element_is_zero() {
    let q: Vec<V> = vec![[0., 0., 0.]; 4];
    assert_eq!(e(M::Skew, S::Quad, &q), 0.0);
}
