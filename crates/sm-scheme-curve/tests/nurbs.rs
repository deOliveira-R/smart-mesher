//! Spacing on NURBS with repeated knots and non-uniform speed: the arc-length map must stay
//! accurate where the speed jumps (non-dyadic knots), and curvature must go through arc length.

use sm_core::Point3;
use sm_geom_api::{Interval, NurbsCurve, ParametricCurve};
use sm_scheme_curve::SpacingError;
use sm_scheme_curve::engine::{Spec, node_params};
use sm_scheme_curve::params::BiasMode;
use sm_topo_api::{CurveCarrier, Sense};

const KNOT: f64 = 0.37;

fn carrier(sense: Sense) -> CurveCarrier {
    CurveCarrier { curve: sm_core::EntityId::new(sm_core::EntityKind::Curve, 0).unwrap(), range: Interval::new(0.0, 1.0), sense }
}

const GL_X: [f64; 5] = [0.0, -0.538_469_310_105_683_1, 0.538_469_310_105_683_1, -0.906_179_845_938_664, 0.906_179_845_938_664];
const GL_W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_1, 0.236_926_885_056_189_1];

/// Independent arc length on `[a, b]`, split at the knot so each piece is smooth.
fn length(c: &dyn ParametricCurve, a: f64, b: f64) -> f64 {
    let piece = |a: f64, b: f64| {
        let panels = 2000;
        let h = (b - a) / panels as f64;
        let mut s = 0.0;
        for k in 0..panels {
            let mid = a + (k as f64 + 0.5) * h;
            for (x, w) in GL_X.iter().zip(GL_W) {
                s += w * c.d1(mid + 0.5 * h * x).norm();
            }
        }
        s * 0.5 * h
    };
    if a < KNOT && b > KNOT { piece(a, KNOT) + piece(KNOT, b) } else { piece(a, b) }
}

/// Quadratic B-spline with a double interior knot at the non-dyadic 0.37 (C0, speed jumps).
fn double_knot() -> NurbsCurve {
    let pts = [(0.0, 0.0), (1.0, 2.0), (2.5, -1.0), (3.0, 1.5), (5.0, 0.0)];
    let control = pts.iter().map(|&(x, y)| Point3::new(x, y, 0.1 * x)).collect();
    NurbsCurve::new(2, vec![0.0, 0.0, 0.0, KNOT, KNOT, 1.0, 1.0, 1.0], control, None).unwrap()
}

/// Cubic with a triple interior knot at 0.37 (C0).
fn triple_knot() -> NurbsCurve {
    let pts = [(0.0, 0.0), (1.0, 2.0), (2.5, -1.0), (3.0, 1.5), (4.0, 0.2), (5.0, 1.0), (6.0, -0.5)];
    let control = pts.iter().map(|&(x, y)| Point3::new(x, y, 0.1 * x)).collect();
    NurbsCurve::new(3, vec![0.0, 0.0, 0.0, 0.0, KNOT, KNOT, KNOT, 1.0, 1.0, 1.0, 1.0], control, None).unwrap()
}

fn segments(c: &dyn ParametricCurve, fr: &[f64]) -> Vec<f64> {
    let mut ts = vec![0.0];
    ts.extend(fr);
    ts.push(1.0);
    ts.windows(2).map(|w| length(c, w[0], w[1])).collect()
}

#[test]
fn equal_on_repeated_knot_nurbs_has_equal_arc_lengths() {
    for (c, name) in [(&double_knot() as &dyn ParametricCurve, "double"), (&triple_knot(), "triple")] {
        let total = length(c, 0.0, 1.0);
        for sense in [Sense::Forward, Sense::Reversed] {
            for n in [3u32, 8, 20] {
                let fr = node_params(c, carrier(sense), n, &Spec::Equal).unwrap();
                // Reversed edges measure fractions from the end of the range.
                let fr: Vec<f64> = if sense == Sense::Reversed { fr.iter().rev().map(|f| 1.0 - f).collect() } else { fr };
                for s in segments(c, &fr) {
                    let want = total / n as f64;
                    assert!((s - want).abs() <= 1e-9 * want, "{name} n={n}: {s} vs {want}");
                }
            }
        }
    }
}

#[test]
fn bias_first_size_closes_on_repeated_knot_nurbs() {
    let c = double_knot();
    let total = length(&c, 0.0, 1.0);
    let first = 0.02 * total;
    let fr = node_params(&c, carrier(Sense::Forward), 12, &Spec::Bias { mode: BiasMode::First(first), reverse: false }).unwrap();
    let lens = segments(&c, &fr);
    assert!((lens[0] - first).abs() <= 1e-9 * first, "first {} vs {first}", lens[0]);
    let r = lens[1] / lens[0];
    for w in lens.windows(2) {
        assert!((w[1] / w[0] - r).abs() <= 1e-8, "ratio {} vs {r}", w[1] / w[0]);
    }
}

#[test]
fn curvature_is_equal_for_constant_curvature_on_nonuniform_parametrizations() {
    let w = core::f64::consts::FRAC_1_SQRT_2;
    let quarter = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
        Some(vec![1.0, w, 1.0]),
    )
    .unwrap();
    // Straight NURBS with strongly non-uniform speed (collinear control points).
    let line = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.1, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
        None,
    )
    .unwrap();
    for (c, name) in [(&quarter as &dyn ParametricCurve, "quarter circle"), (&line, "nurbs line")] {
        for sense in [Sense::Forward, Sense::Reversed] {
            for factor in [3.0, -3.0] {
                for n in [4u32, 9] {
                    let a = node_params(c, carrier(sense), n, &Spec::Curvature { factor }).unwrap();
                    let e = node_params(c, carrier(sense), n, &Spec::Equal).unwrap();
                    for (x, y) in a.iter().zip(&e) {
                        assert!((x - y).abs() <= 1e-12, "{name} factor={factor} n={n}: {x} vs {y}");
                    }
                }
            }
        }
    }
}

#[test]
fn curvature_at_a_cusp_is_a_geometry_error() {
    // Coincident first control points: zero speed at the start.
    let c = NurbsCurve::new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)],
        None,
    )
    .unwrap();
    match node_params(&c, carrier(Sense::Forward), 4, &Spec::Curvature { factor: 1.0 }) {
        Err(SpacingError::Geometry(_)) => {}
        other => panic!("expected a geometry error, got {other:?}"),
    }
}
