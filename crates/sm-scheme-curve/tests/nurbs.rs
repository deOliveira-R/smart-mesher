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

// ---- short and non-dyadic knot spans ----

/// Reference arc length on `[a, b]`, integrating each knot span separately.
fn length_knots(c: &dyn ParametricCurve, knots: &[f64], a: f64, b: f64) -> f64 {
    let mut cuts = vec![a];
    cuts.extend(knots.iter().copied().filter(|k| *k > a && *k < b));
    cuts.dedup();
    cuts.push(b);
    let mut total = 0.0;
    for w in cuts.windows(2) {
        let panels = 3000;
        let h = (w[1] - w[0]) / panels as f64;
        let mut s = 0.0;
        for k in 0..panels {
            let mid = w[0] + (k as f64 + 0.5) * h;
            for (x, wt) in GL_X.iter().zip(GL_W) {
                s += wt * c.d1(mid + 0.5 * h * x).norm();
            }
        }
        total += s * 0.5 * h;
    }
    total
}

fn seg_lengths(c: &dyn ParametricCurve, knots: &[f64], fr: &[f64]) -> Vec<f64> {
    let mut ts = vec![0.0];
    ts.extend(fr);
    ts.push(1.0);
    ts.windows(2).map(|w| length_knots(c, knots, w[0], w[1])).collect()
}

fn check_equal(c: &dyn ParametricCurve, knots: &[f64], name: &str, ns: &[u32], tol: f64) {
    let total = length_knots(c, knots, 0.0, 1.0);
    for sense in [Sense::Forward, Sense::Reversed] {
        for &n in ns {
            let fr = node_params(c, carrier(sense), n, &Spec::Equal).unwrap();
            let fr: Vec<f64> = if sense == Sense::Reversed { fr.iter().rev().map(|f| 1.0 - f).collect() } else { fr };
            let want = total / f64::from(n);
            for s in seg_lengths(c, knots, &fr) {
                assert!((s - want).abs() <= tol * want, "{name} {sense:?} n={n}: {s} vs {want}");
            }
        }
    }
}

fn spline(degree: usize, knots: Vec<f64>, npts: usize) -> NurbsCurve {
    let control = (0..npts)
        .map(|i| {
            let x = i as f64;
            Point3::new(x + 0.3 * (1.7 * x).sin(), 2.0 * (0.9 * x + 0.4).cos(), 0.1 * x)
        })
        .collect();
    NurbsCurve::new(degree, knots, control, None).unwrap()
}

fn check_bias(c: &dyn ParametricCurve, knots: &[f64], n: u32, mode: BiasMode, ratio: Option<f64>) {
    let total = length_knots(c, knots, 0.0, 1.0);
    let fr = node_params(c, carrier(Sense::Forward), n, &Spec::Bias { mode, reverse: false }).unwrap();
    let lens = seg_lengths(c, knots, &fr);
    assert!((lens.iter().sum::<f64>() - total).abs() <= 1e-9 * total);
    if let BiasMode::First(f) = mode {
        assert!((lens[0] - f).abs() <= 1e-9 * f, "first {} vs {f}", lens[0]);
    }
    let r = ratio.unwrap_or(lens[1] / lens[0]);
    for w in lens.windows(2) {
        assert!((w[1] / w[0] - r).abs() <= 1e-8 * r, "ratio {} vs {r}", w[1] / w[0]);
    }
}

#[test]
fn double_knots_at_irrational_looking_positions() {
    let k = vec![0.0, 0.0, 0.0, 0.3141, 0.3141, 0.7183, 0.7183, 1.0, 1.0, 1.0];
    let c = spline(2, k.clone(), 7);
    check_equal(&c, &k, "case a", &[3, 7, 25], 1e-9);
    let total = length_knots(&c, &k, 0.0, 1.0);
    check_bias(&c, &k, 10, BiasMode::First(0.03 * total), None);
}

#[test]
fn degree_one_polyline_with_simple_knots() {
    let k = vec![0.0, 0.0, 0.123, 0.456, 0.789, 1.0, 1.0];
    let c = spline(1, k.clone(), 5);
    check_equal(&c, &k, "case b", &[3, 9, 25], 1e-9);
}

#[test]
fn double_knot_near_the_start() {
    let k = vec![0.0, 0.0, 0.0, 0.0013, 0.0013, 1.0, 1.0, 1.0];
    let c = spline(2, k.clone(), 5);
    check_equal(&c, &k, "case c", &[3, 7, 25], 1e-9);
    // Curvature through the corrected arc map stays well formed.
    let fr = node_params(&c, carrier(Sense::Forward), 12, &Spec::Curvature { factor: 2.0 }).unwrap();
    assert!(fr.windows(2).all(|w| w[0] < w[1]) && fr[0] > 0.0 && *fr.last().unwrap() < 1.0);
}

#[test]
fn short_interior_span_c2_and_c1() {
    let k2 = vec![0.0, 0.0, 0.0, 0.0, 0.499, 0.501, 1.0, 1.0, 1.0, 1.0];
    let c2 = spline(3, k2.clone(), 6);
    check_equal(&c2, &k2, "cubic C2 short span", &[7, 50], 1e-9);
    let k1 = vec![0.0, 0.0, 0.0, 0.0, 0.499, 0.499, 0.501, 0.501, 1.0, 1.0, 1.0, 1.0];
    let c1 = spline(3, k1.clone(), 8);
    check_equal(&c1, &k1, "cubic C1 short span", &[7, 50], 1e-9);
    check_bias(&c1, &k1, 12, BiasMode::Ratio(1.2), Some(1.2));
}

#[test]
fn short_end_span() {
    let k = vec![0.0, 0.0, 0.0, 0.002, 1.0, 1.0, 1.0];
    let c = spline(2, k.clone(), 4);
    check_equal(&c, &k, "quadratic end span", &[7, 50], 1e-9);
    let total = length_knots(&c, &k, 0.0, 1.0);
    check_bias(&c, &k, 9, BiasMode::First(0.05 * total), None);
    let k3 = vec![0.0, 0.0, 0.0, 0.0, 0.002, 0.3, 0.7, 1.0, 1.0, 1.0, 1.0];
    let c3 = spline(3, k3.clone(), 7);
    check_equal(&c3, &k3, "cubic C2 {0.002,0.3,0.7}", &[7, 50], 1e-9);
}

#[test]
fn huge_interval_counts_are_refused_before_allocation() {
    let k = vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0];
    let c = spline(2, k, 3);
    for spec in [Spec::Equal, Spec::Curvature { factor: 1.0 }] {
        match node_params(&c, carrier(Sense::Forward), u32::MAX, &spec) {
            Err(SpacingError::Unreachable(_)) => {}
            other => panic!("expected a typed cap error, got {other:?}"),
        }
    }
}
