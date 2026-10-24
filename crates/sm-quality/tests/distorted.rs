//! Element-level metrics on distorted elements against closed-form values derived by hand from
//! the formulas in DESIGN.md.
#![allow(missing_docs)]

mod common;
use common::*;
use sm_mesh_api::CellShape as S;
use sm_quality::{Context, Metric as M, V, eval};

fn e(m: M, s: S, p: &[V]) -> f64 {
    eval(m, s, p, &Context::default()).unwrap_or_else(|| panic!("{m:?} undefined for {s:?}"))
}
fn near(got: f64, want: f64, what: &str) {
    assert!((got - want).abs() <= 1e-12 * want.abs().max(1.0), "{what}: got {got}, want {want}");
}

#[test]
fn rectangle_2_by_1() {
    let p: Vec<V> = vec![[0., 0., 0.], [2., 0., 0.], [2., 1., 0.], [0., 1., 0.]];
    near(e(M::Volume, S::Quad, &p), 2.0, "area");
    near(e(M::Jacobian, S::Quad, &p), 2.0, "jacobian");
    near(e(M::ScaledJacobian, S::Quad, &p), 1.0, "sj");
    near(e(M::AspectRatio, S::Quad, &p), 1.5, "aspect = lmax P / 4A = 2*6/8");
    near(e(M::EdgeRatio, S::Quad, &p), 2.0, "edge ratio");
    near(e(M::MaxEdgeRatio, S::Quad, &p), 2.0, "max edge ratio");
    near(e(M::Skew, S::Quad, &p), 0.0, "skew");
    near(e(M::Taper, S::Quad, &p), 0.0, "taper");
    near(e(M::Warpage, S::Quad, &p), 1.0, "warpage");
    near(e(M::Stretch, S::Quad, &p), 2f64.sqrt() / 5f64.sqrt(), "stretch");
    near(e(M::Condition, S::Quad, &p), 1.25, "condition (4+1)/(2*2)");
    near(e(M::Shape, S::Quad, &p), 0.8, "shape");
    near(e(M::Oddy, S::Quad, &p), 1.125, "oddy 9/8");
    near(e(M::MinAngle, S::Quad, &p), 90.0, "min angle");
    near(e(M::MaxAngle, S::Quad, &p), 90.0, "max angle");
    near(e(M::Distortion, S::Quad, &p), 1.0, "distortion");
    near(e(M::Shear, S::Quad, &p), 1.0, "shear");
}

#[test]
fn sheared_quad_45_degrees() {
    let p: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [2., 1., 0.], [1., 1., 0.]];
    let h = 0.5f64.sqrt();
    near(e(M::ScaledJacobian, S::Quad, &p), h, "sj = sin 45");
    near(e(M::Skew, S::Quad, &p), h, "skew = s/sqrt(1+s^2)");
    near(e(M::Taper, S::Quad, &p), 0.0, "parallelogram taper");
    near(e(M::MinAngle, S::Quad, &p), 45.0, "min angle");
    near(e(M::MaxAngle, S::Quad, &p), 135.0, "max angle");
    near(e(M::Shape, S::Quad, &p), 2.0 / 3.0, "shape = 2J/(l1^2+l2^2) = 2/(1+2)");
    near(e(M::Volume, S::Quad, &p), 1.0, "area");
    near(e(M::Distortion, S::Quad, &p), 1.0, "constant jacobian");
}

#[test]
fn trapezoid_taper() {
    // bottom 2, top 1, height 1: taper = |b - a| / min(a + b, 2h) = 1/2
    let p: Vec<V> = vec![[0., 0., 0.], [2., 0., 0.], [1.5, 1., 0.], [0.5, 1., 0.]];
    near(e(M::Taper, S::Quad, &p), 0.5, "taper");
    near(e(M::Volume, S::Quad, &p), 1.5, "area");
    assert!(e(M::Distortion, S::Quad, &p) < 1.0);
}

#[test]
fn warped_quad() {
    let w = 0.5f64;
    let p: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [1., 1., w], [0., 1., 0.]];
    let want = (1.0 + w * w).powi(-3).min((1.0 + 2.0 * w * w).powf(-1.5));
    near(e(M::Warpage, S::Quad, &p), want, "warpage");
}

#[test]
fn clockwise_quad_is_inverted_against_reference_normal() {
    let p: Vec<V> = vec![[0., 0., 0.], [0., 1., 0.], [1., 1., 0.], [1., 0., 0.]];
    let ctx = Context { normal: Some([0., 0., 1.]), ref_size: 1.0 };
    let v = |m| eval(m, S::Quad, &p, &ctx).unwrap();
    near(v(M::Volume), -1.0, "signed area");
    near(v(M::ScaledJacobian), -1.0, "sj");
    near(v(M::Jacobian), -1.0, "jacobian");
    near(v(M::Shape), 0.0, "shape of an inverted element is 0");
    // Against its own normal it is simply flipped, hence valid.
    near(e(M::ScaledJacobian, S::Quad, &p), 1.0, "own normal");
}

#[test]
fn dart_quad_has_a_reflex_corner() {
    let p: Vec<V> = vec![[0., 0., 0.], [2., 0., 0.], [0.25, 0.25, 0.], [0., 2., 0.]];
    let ctx = Context { normal: Some([0., 0., 1.]), ref_size: 1.0 };
    let cos = -0.875 / 3.125;
    near(eval(M::MaxAngle, S::Quad, &p, &ctx).unwrap(), 360.0 - f64::acos(cos).to_degrees(), "reflex angle");
    near(eval(M::ScaledJacobian, S::Quad, &p, &ctx).unwrap(), -3.0 / 3.125, "reflex corner sj");
}

#[test]
fn right_triangle() {
    let p: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]];
    let r2 = 2f64.sqrt();
    let s3 = 3f64.sqrt();
    near(e(M::Volume, S::Tri, &p), 0.5, "area");
    near(e(M::AspectRatio, S::Tri, &p), (r2 + 1.0) / s3, "aspect");
    near(e(M::Condition, S::Tri, &p), 2.0 / s3, "condition = sum l^2 / (4 sqrt3 A)");
    near(e(M::AspectFrobenius, S::Tri, &p), 2.0 / s3, "aspect frobenius");
    near(e(M::Shape, S::Tri, &p), s3 / 2.0, "shape");
    near(e(M::RadiusRatio, S::Tri, &p), (2.0 * r2 + 2.0) / 4.0, "radius ratio");
    near(e(M::EdgeRatio, S::Tri, &p), r2, "edge ratio");
    near(e(M::MinAngle, S::Tri, &p), 45.0, "min angle");
    near(e(M::MaxAngle, S::Tri, &p), 90.0, "max angle");
    near(e(M::ScaledJacobian, S::Tri, &p), 0.5f64.sqrt() * 2.0 / s3, "scaled jacobian");
    near(e(M::Jacobian, S::Tri, &p), 1.0, "jacobian = 2A");
}

#[test]
fn polygon_generalizations() {
    // Regular n-gon: aspect ratio 1, interior angle (n-2)*180/n, area n R^2 sin(2pi/n)/2.
    for n in [5usize, 6, 8] {
        let p = polygon(n);
        let nf = n as f64;
        near(e(M::AspectRatio, S::Polygon, &p), 1.0, "aspect");
        near(e(M::MinAngle, S::Polygon, &p), (nf - 2.0) * 180.0 / nf, "angle");
        near(e(M::Volume, S::Polygon, &p), nf * (std::f64::consts::TAU / nf).sin() / 2.0, "area");
    }
}

#[test]
fn right_corner_tet() {
    let p: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    let s3 = 3f64.sqrt();
    let beta = (1.0 + s3) / 2.0;
    near(e(M::Volume, S::Tet, &p), 1.0 / 6.0, "volume");
    near(e(M::Jacobian, S::Tet, &p), 1.0, "jacobian = 6V");
    near(e(M::ScaledJacobian, S::Tet, &p), 2f64.sqrt() / 2.0, "sj");
    near(e(M::AspectRatio, S::Tet, &p), beta, "aspect beta");
    near(e(M::RadiusRatio, S::Tet, &p), beta, "radius ratio R/(3r) with R = sqrt3/2, r = (3-sqrt3)/6");
    near(e(M::AspectGamma, S::Tet, &p), 1.5f64.powf(1.5) * 2f64.sqrt() / 2.0, "gamma");
    near(e(M::EdgeRatio, S::Tet, &p), 2f64.sqrt(), "edge ratio");
    near(e(M::MinAngle, S::Tet, &p), (1.0 / s3).acos().to_degrees(), "min dihedral");
    near(e(M::MaxAngle, S::Tet, &p), 90.0, "max dihedral");
    near(e(M::CollapseRatio, S::Tet, &p), 1.0 / 6f64.sqrt(), "collapse");
    // T = W^-1: ||W^-1||_F^2 = ||W||_F^2 * 1.5 = 4.5, det = sqrt 2.
    near(e(M::Shape, S::Tet, &p), 3.0 * 2f64.powf(1.0 / 3.0) / 4.5, "shape");
    near(e(M::Condition, S::Tet, &p), 1.5f64.sqrt(), "condition");
    near(e(M::AspectFrobenius, S::Tet, &p), 4.5 / (3.0 * 2f64.powf(1.0 / 3.0)), "aspect frobenius = 1/shape");
}

#[test]
fn inverted_tet_and_hex() {
    let mut t = tet();
    t.swap(1, 2);
    near(e(M::ScaledJacobian, S::Tet, &t), -1.0, "tet sj");
    near(e(M::Distortion, S::Tet, &t), -1.0, "tet distortion");
    near(e(M::Shape, S::Tet, &t), 0.0, "tet shape");
    assert!(e(M::Volume, S::Tet, &t) < 0.0);
    let h = hex();
    let flipped: Vec<V> = (0..8).map(|i| h[(i + 4) % 8]).collect();
    near(e(M::ScaledJacobian, S::Hex, &flipped), -1.0, "hex sj");
    near(e(M::Volume, S::Hex, &flipped), -1.0, "hex volume");
}

#[test]
fn brick_1_2_4() {
    let p: Vec<V> = hex().iter().map(|q| [q[0], 2.0 * q[1], 4.0 * q[2]]).collect();
    near(e(M::Volume, S::Hex, &p), 8.0, "volume");
    near(e(M::Jacobian, S::Hex, &p), 8.0, "jacobian");
    near(e(M::ScaledJacobian, S::Hex, &p), 1.0, "sj");
    near(e(M::EdgeRatio, S::Hex, &p), 4.0, "edge ratio");
    near(e(M::MaxEdgeRatio, S::Hex, &p), 4.0, "max edge ratio");
    near(e(M::Stretch, S::Hex, &p), 3f64.sqrt() / 21f64.sqrt(), "stretch");
    near(e(M::Diagonal, S::Hex, &p), 1.0, "diagonal");
    near(e(M::Skew, S::Hex, &p), 0.0, "skew");
    near(e(M::Taper, S::Hex, &p), 0.0, "taper");
    near(e(M::Dimension, S::Hex, &p), 2.0, "dimension");
    near(e(M::Condition, S::Hex, &p), 21f64.sqrt() * (1.0 + 0.25 + 1.0 / 16.0f64).sqrt() / 3.0, "condition");
    near(e(M::Oddy, S::Hex, &p), (273.0 - 147.0) / 16.0, "oddy");
    near(e(M::Shape, S::Hex, &p), 12.0 / 21.0, "shape = 3 det^(2/3) / sum l^2");
    near(e(M::MaxAspectFrobenius, S::Hex, &p), 21.0 / 12.0, "max aspect frobenius");
    near(e(M::MeanAspectFrobenius, S::Hex, &p), 21.0 / 12.0, "mean aspect frobenius");
    near(e(M::Distortion, S::Hex, &p), 1.0, "distortion");
    near(e(M::MinAngle, S::Hex, &p), 90.0, "dihedral");
}

#[test]
fn sheared_hex() {
    let p: Vec<V> = hex().iter().map(|q| [q[0] + q[1], q[1], q[2]]).collect();
    let h = 0.5f64.sqrt();
    near(e(M::Skew, S::Hex, &p), h, "skew");
    near(e(M::ScaledJacobian, S::Hex, &p), h, "sj");
    near(e(M::Shear, S::Hex, &p), h, "shear");
    near(e(M::Volume, S::Hex, &p), 1.0, "volume");
    near(e(M::MinAngle, S::Hex, &p), 45.0, "dihedral");
    near(e(M::MaxAngle, S::Hex, &p), 135.0, "dihedral");
}

#[test]
fn tapered_hex() {
    // top face scaled by t about its centre.
    let t = 0.5;
    let p: Vec<V> = hex()
        .iter()
        .map(|q| if q[2] > 0.5 { [0.5 + t * (q[0] - 0.5), 0.5 + t * (q[1] - 0.5), 1.0] } else { *q })
        .collect();
    near(e(M::Taper, S::Hex, &p), 2.0 * (1.0 - t) / (2.0 * (1.0 + t)).min(4.0), "taper");
    // frustum volume h (A1 + A2 + sqrt(A1 A2)) / 3
    near(e(M::Volume, S::Hex, &p), (1.0 + t * t + t) / 3.0, "frustum volume");
}

#[test]
fn right_wedge() {
    let p: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [0., 1., 1.]];
    near(e(M::Volume, S::Wedge, &p), 0.5, "volume");
    near(e(M::EdgeRatio, S::Wedge, &p), 2f64.sqrt(), "edge ratio");
    near(e(M::ScaledJacobian, S::Wedge, &p), 0.5f64.sqrt() * 2.0 / 3f64.sqrt(), "sj");
    near(e(M::MinAngle, S::Wedge, &p), 45.0, "dihedral");
    near(e(M::MaxAngle, S::Wedge, &p), 90.0, "dihedral");
    near(e(M::Distortion, S::Wedge, &p), 1.0, "distortion");
}

#[test]
fn tall_pyramid() {
    let p: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0.5, 0.5, 1.0]];
    near(e(M::Volume, S::Pyramid, &p), 1.0 / 3.0, "volume");
    near(e(M::ScaledJacobian, S::Pyramid, &p), 2f64.sqrt() / 1.5f64.sqrt(), "sj = sqrt2 * 1/|c|");
    near(e(M::Jacobian, S::Pyramid, &p), 1.0, "jacobian");
}

#[test]
fn relative_size() {
    let p = hex();
    let v = |m, r| eval(m, S::Hex, &p, &Context { normal: None, ref_size: r }).unwrap();
    near(v(M::RelativeSizeSquared, 2.0), 0.25, "tau = 1/2");
    near(v(M::RelativeSizeSquared, 0.5), 0.25, "tau = 2");
    near(v(M::ShapeAndSize, 2.0), 0.25, "shape and size");
    near(v(M::ShearAndSize, 2.0), 0.25, "shear and size");
    assert!(v(M::RelativeSizeSquared, f64::NAN).is_nan());
}

#[test]
fn wrong_arity_and_nonfinite() {
    let c = Context::default();
    assert!(eval(M::Volume, S::Hex, &tet(), &c).is_none());
    assert!(eval(M::Skew, S::Tet, &tet(), &c).is_none());
    let mut p = hex();
    p[3][1] = f64::NAN;
    assert!(eval(M::Volume, S::Hex, &p, &c).unwrap().is_nan());
}

#[test]
fn degenerate_elements_use_verdict_sentinels() {
    let c = Context::default();
    let flat: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]];
    assert_eq!(eval(M::AspectRatio, S::Tet, &flat, &c).unwrap(), sm_quality::DBL_MAX);
    assert_eq!(eval(M::Shape, S::Tet, &flat, &c).unwrap(), 0.0);
    let line: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]];
    assert_eq!(eval(M::AspectRatio, S::Tri, &line, &c).unwrap(), sm_quality::DBL_MAX);
}
