//! Spacing on concrete curves (line, circle, ellipse, NURBS): analytic arc-length checks.

use core::f64::consts::{FRAC_PI_2, PI};
use sm_core::Point3;
use sm_core::Vector3;
use sm_geom_api::{Circle, Ellipse, Frame, Interval, Line, NurbsCurve, ParametricCurve};
use sm_topo_api::Sense;
use sm_scheme_curve::SpacingError;
use sm_scheme_curve::engine::{Spec, node_params};
use sm_scheme_curve::params::{BiasMode, DualMode};
use sm_topo_api::CurveCarrier;

fn carrier(lo: f64, hi: f64, sense: Sense) -> CurveCarrier {
    CurveCarrier { curve: sm_core::EntityId::new(sm_core::EntityKind::Curve, 0).unwrap(), range: Interval::new(lo, hi), sense }
}

/// Independent arc length over the parameter interval `[a, b]`: 5-point Gauss-Legendre on 4000
/// panels (the reference NURBS integrator stops well short of 1e-11).
fn fine_length(c: &dyn ParametricCurve, a: f64, b: f64) -> f64 {
    const X: [f64; 5] = [0.0, -0.538_469_310_105_683_1, 0.538_469_310_105_683_1, -0.906_179_845_938_664, 0.906_179_845_938_664];
    const W: [f64; 5] = [0.568_888_888_888_888_9, 0.478_628_670_499_366_5, 0.478_628_670_499_366_5, 0.236_926_885_056_189_1, 0.236_926_885_056_189_1];
    let panels = 4000;
    let h = (b - a) / panels as f64;
    let mut s = 0.0;
    for k in 0..panels {
        let mid = a + (k as f64 + 0.5) * h;
        for (x, w) in X.iter().zip(W) {
            s += w * c.d1(mid + 0.5 * h * x).norm();
        }
    }
    s * 0.5 * h
}

fn param(cc: &CurveCarrier, f: f64) -> f64 {
    match cc.sense {
        Sense::Forward => cc.range.lo + f * (cc.range.hi - cc.range.lo),
        Sense::Reversed => cc.range.hi - f * (cc.range.hi - cc.range.lo),
    }
}

/// Arc lengths of the segments between consecutive nodes, measured independently with a very
/// fine composite Gauss-Legendre over the curve parameter.
fn segments(c: &dyn ParametricCurve, cc: &CurveCarrier, fr: &[f64]) -> Vec<f64> {
    let mut ts = vec![param(cc, 0.0)];
    ts.extend(fr.iter().map(|f| param(cc, *f)));
    ts.push(param(cc, 1.0));
    ts.windows(2)
        .map(|w| {
            let (a, b) = (w[0].min(w[1]), w[0].max(w[1]));
            fine_length(c, a, b)
        })
        .collect()
}

fn circle(r: f64) -> Circle {
    Circle::new(Frame::from_zx(Point3::new(1.0, -2.0, 0.5), Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0)).unwrap(), r).unwrap()
}

fn ellipse(a: f64, b: f64) -> Ellipse {
    Ellipse::new(Frame::from_zx(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 0.0, 0.0)).unwrap(), a, b).unwrap()
}

/// Rational quadratic quarter circle of radius 1: parameter is not arc length.
fn nurbs_quarter() -> NurbsCurve {
    let w = core::f64::consts::FRAC_1_SQRT_2;
    NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0), Point3::new(0.0, 1.0, 0.0)], Some(vec![1.0, w, 1.0])).unwrap()
}

/// A cubic with strongly varying speed.
fn nurbs_wiggly() -> NurbsCurve {
    let pts = [(0.0, 0.0), (0.2, 1.5), (0.5, -1.0), (3.0, 1.2), (3.2, 0.1), (4.0, 0.5)];
    let control = pts.iter().map(|&(x, y)| Point3::new(x, y, 0.3 * x)).collect();
    NurbsCurve::new(3, vec![0.0, 0.0, 0.0, 0.0, 0.3, 0.8, 1.0, 1.0, 1.0, 1.0], control, None).unwrap()
}

fn rel_spread(v: &[f64]) -> f64 {
    let mean = v.iter().sum::<f64>() / v.len() as f64;
    v.iter().map(|x| (x - mean).abs() / mean).fold(0.0, f64::max)
}

#[test]
fn equal_on_a_line_is_i_l_over_n_to_an_ulp() {
    for sense in [Sense::Forward, Sense::Reversed] {
        let l = 7.3;
        let line = Line::new(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)).unwrap();
        let cc = carrier(1.0, 1.0 + l, sense);
        for n in [2u32, 3, 7, 10, 1000] {
            let fr = node_params(&line, cc, n, &Spec::Equal).unwrap();
            assert_eq!(fr.len(), n as usize - 1);
            let start = if sense == Sense::Forward { 1.0 } else { 1.0 + l };
            let dir = if sense == Sense::Forward { 1.0 } else { -1.0 };
            for (i, f) in fr.iter().enumerate() {
                // What the crate controls: uniform curves give the exact fractions (i+1)/n.
                assert_eq!(*f, (i as f64 + 1.0) / n as f64, "n={n} i={i}");
                // Positions also carry the rounding of edge_param/Line::eval (outside this crate).
                let x = line.eval(param(&cc, *f)).x;
                let want = start + dir * (i as f64 + 1.0) * l / n as f64;
                assert!((x - want).abs() <= 16.0 * f64::EPSILON * l, "n={n} i={i}: {x} vs {want}");
            }
        }
    }
}

#[test]
fn equal_on_a_circular_arc_has_equal_arc_lengths() {
    let c = circle(2.5);
    for (lo, hi, sense) in [(0.3, 2.9, Sense::Forward), (0.3, 2.9, Sense::Reversed), (-1.0, 4.5, Sense::Forward)] {
        let cc = carrier(lo, hi, sense);
        let total = 2.5 * (hi - lo);
        for n in [2u32, 5, 17, 64] {
            let fr = node_params(&c, cc, n, &Spec::Equal).unwrap();
            let seg = segments(&c, &cc, &fr);
            for s in &seg {
                assert!((s - total / n as f64).abs() <= 1e-12 * total, "n={n}: {s}");
            }
        }
    }
}

#[test]
fn equal_on_an_ellipse_has_equal_arc_lengths() {
    let e = ellipse(3.0, 1.0);
    for sense in [Sense::Forward, Sense::Reversed] {
        let cc = carrier(0.0, 2.2, sense);
        let total = fine_length(&e, cc.range.lo, cc.range.hi);
        for n in [3u32, 11, 40] {
            let fr = node_params(&e, cc, n, &Spec::Equal).unwrap();
            let seg = segments(&e, &cc, &fr);
            for s in &seg {
                assert!((s - total / n as f64).abs() <= 1e-12 * total, "n={n}: {s} vs {}", total / n as f64);
            }
        }
    }
}

#[test]
fn equal_on_nurbs_has_equal_arc_lengths() {
    let w = nurbs_wiggly();
    let cc = carrier(0.0, 1.0, Sense::Forward);
    let total = fine_length(&w, cc.range.lo, cc.range.hi);
    for n in [4u32, 13, 50] {
        let fr = node_params(&w, cc, n, &Spec::Equal).unwrap();
        let seg = segments(&w, &cc, &fr);
        for s in &seg {
            assert!((s - total / n as f64).abs() <= 1e-9 * total, "n={n}: {s}");
        }
    }
}

#[test]
fn equal_on_a_rational_quarter_circle_hits_the_analytic_angles() {
    let q = nurbs_quarter();
    for sense in [Sense::Forward, Sense::Reversed] {
        let cc = carrier(0.0, 1.0, sense);
        let n = 9u32;
        let fr = node_params(&q, cc, n, &Spec::Equal).unwrap();
        for (k, f) in fr.iter().enumerate() {
            let p = q.eval(param(&cc, *f));
            let ang = p.y.atan2(p.x);
            let want = match sense {
                Sense::Forward => FRAC_PI_2 * (k as f64 + 1.0) / n as f64,
                Sense::Reversed => FRAC_PI_2 * (1.0 - (k as f64 + 1.0) / n as f64),
            };
            assert!((ang - want).abs() <= 1e-10, "k={k}: {ang} vs {want}");
        }
    }
}

#[test]
fn bias_on_an_arc_has_edge_ratio_r_and_reverse_flips_it() {
    let c = circle(1.7);
    let cc = carrier(0.0, 3.0, Sense::Forward);
    let (n, r) = (12u32, 1.18);
    let mode = BiasMode::Ratio(r);
    let fwd = node_params(&c, cc, n, &Spec::Bias { mode, reverse: false }).unwrap();
    let seg = segments(&c, &cc, &fwd);
    for w in seg.windows(2) {
        assert!((w[1] / w[0] - r).abs() <= 1e-12 * r, "{}", w[1] / w[0]);
    }
    let total: f64 = seg.iter().sum();
    assert!((total - 1.7 * 3.0).abs() <= 1e-12 * total, "total length exact");
    let rev = node_params(&c, cc, n, &Spec::Bias { mode, reverse: true }).unwrap();
    let seg_r = segments(&c, &cc, &rev);
    for (a, b) in seg.iter().zip(seg_r.iter().rev()) {
        assert!((a - b).abs() <= 1e-12 * total);
    }
}

#[test]
fn bias_first_size_on_a_nurbs_curve_closes() {
    let w = nurbs_wiggly();
    let cc = carrier(0.0, 1.0, Sense::Forward);
    let total = fine_length(&w, cc.range.lo, cc.range.hi);
    let first = 0.05 * total;
    let n = 15u32;
    let fr = node_params(&w, cc, n, &Spec::Bias { mode: BiasMode::First(first), reverse: false }).unwrap();
    let seg = segments(&w, &cc, &fr);
    assert!((seg[0] - first).abs() <= 1e-9 * total, "first {} vs {first}", seg[0]);
    assert!((seg.iter().sum::<f64>() - total).abs() <= 1e-12 * total);
    let r = seg[1] / seg[0];
    for p in seg.windows(2) {
        assert!((p[1] / p[0] - r).abs() <= 1e-8 * r);
    }
}

#[test]
fn dualbias_on_an_ellipse_is_mirror_symmetric() {
    let e = ellipse(2.0, 1.5);
    let cc = carrier(0.4, 2.6, Sense::Forward);
    for n in [8u32, 9] {
        let fr = node_params(&e, cc, n, &Spec::Dualbias(DualMode::Ratio(1.3))).unwrap();
        let seg = segments(&e, &cc, &fr);
        let total: f64 = seg.iter().sum();
        for k in 0..n as usize {
            assert!((seg[k] - seg[n as usize - 1 - k]).abs() <= 1e-12 * total, "n={n} k={k}");
        }
        let m = n as usize / 2;
        for k in 1..m {
            assert!((seg[k] / seg[k - 1] - 1.3).abs() <= 1e-10);
        }
    }
}

#[test]
fn stretch_first_segment_equals_first_size() {
    let c = circle(3.0);
    let cc = carrier(0.0, 2.0, Sense::Forward);
    let total = 6.0;
    let spec = Spec::Stretch { first: 0.25, last: Some(0.75), max_ratio: 2.0 };
    let n = 12u32;
    let fr = node_params(&c, cc, n, &spec).unwrap();
    let seg = segments(&c, &cc, &fr);
    assert!((seg[0] - 0.25).abs() <= 1e-12 * total, "{}", seg[0]);
    assert!((seg[n as usize - 1] - 0.75).abs() <= 1e-12 * total);
    let worst = seg.windows(2).map(|w| (w[0] / w[1]).max(w[1] / w[0])).fold(1.0, f64::max);
    assert!(worst <= 2.0, "{worst}");
    // Too tight a ratio is refused.
    let tight = Spec::Stretch { first: 0.25, last: Some(0.75), max_ratio: 1.0001 };
    assert!(matches!(node_params(&c, cc, n, &tight), Err(SpacingError::RatioLimit { .. })));
}

#[test]
fn curvature_reduces_to_equal_on_a_line_and_a_circle() {
    let line = Line::new(Point3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 2.0, 2.0)).unwrap();
    let ccl = carrier(0.0, 6.0, Sense::Forward);
    let circ = circle(2.0);
    let ccc = carrier(0.2, 3.9, Sense::Reversed);
    for (c, cc) in [(&line as &dyn ParametricCurve, ccl), (&circ as &dyn ParametricCurve, ccc)] {
        for factor in [3.0, -3.0, 50.0] {
            for n in [4u32, 9] {
                let a = node_params(c, cc, n, &Spec::Curvature { factor }).unwrap();
                let e = node_params(c, cc, n, &Spec::Equal).unwrap();
                for (x, y) in a.iter().zip(&e) {
                    assert!((x - y).abs() <= 1e-13, "factor={factor} n={n}: {x} vs {y}");
                }
            }
        }
    }
}

#[test]
fn curvature_on_an_ellipse_is_monotone_and_smaller_where_curvature_is_high() {
    // Quarter ellipse a = 2, b = 1 from (2, 0) (curvature a / b^2 = 2) to (0, 1) (curvature b / a^2 = 0.25).
    let e = ellipse(2.0, 1.0);
    let n = 12u32;
    let total = fine_length(&e, 0.0, FRAC_PI_2);
    for (sense, high_first) in [(Sense::Forward, true), (Sense::Reversed, false)] {
        let cc = carrier(0.0, FRAC_PI_2, sense);
        for factor in [1.0, 4.0] {
            let fr = node_params(&e, cc, n, &Spec::Curvature { factor }).unwrap();
            let seg = segments(&e, &cc, &fr);
            let (mut s, equal) = (seg.clone(), total / n as f64);
            if !high_first {
                s.reverse();
            }
            assert!(s.windows(2).all(|w| w[1] > w[0]), "factor={factor}: {seg:?}");
            assert!(s[0] < equal && s[n as usize - 1] > equal, "factor={factor}: {seg:?}");
            assert!((seg.iter().sum::<f64>() - total).abs() <= 1e-12 * total);
        }
        // Negative factor: the largest edges sit where curvature is largest.
        let fr = node_params(&e, cc, n, &Spec::Curvature { factor: -4.0 }).unwrap();
        let mut s = segments(&e, &cc, &fr);
        if !high_first {
            s.reverse();
        }
        assert!(s.windows(2).all(|w| w[1] < w[0]), "{s:?}");
    }
    // Stronger factor concentrates more.
    let cc = carrier(0.0, FRAC_PI_2, Sense::Forward);
    let first = |factor| segments(&e, &cc, &node_params(&e, cc, n, &Spec::Curvature { factor }).unwrap())[0];
    assert!(first(4.0) < first(1.0));
}

#[test]
fn curvature_density_matches_its_definition() {
    // The integral of rho = (1 + |c| L k) ds between consecutive nodes is equal (by the model's own
    // trapezoid-free check with a fine independent quadrature over the parameter).
    let e = ellipse(2.0, 1.0);
    let cc = carrier(0.0, FRAC_PI_2, Sense::Forward);
    let total = fine_length(&e, cc.range.lo, cc.range.hi);
    let (factor, n) = (2.0, 10u32);
    let fr = node_params(&e, cc, n, &Spec::Curvature { factor }).unwrap();
    let rho_ds = |t: f64| {
        let j = e.jet(t);
        let sp = j.d1.norm();
        (1.0 + factor * total * j.d1.cross(j.d2).norm() / (sp * sp * sp)) * sp
    };
    let integral = |a: f64, b: f64| {
        let m = 4000;
        let h = (b - a) / m as f64;
        (0..m).map(|i| {
            let (x0, x1) = (a + i as f64 * h, a + (i + 1) as f64 * h);
            // 2-point Gauss
            let g = 0.5 / 3f64.sqrt();
            0.5 * h * (rho_ds(0.5 * (x0 + x1) - h * g) + rho_ds(0.5 * (x0 + x1) + h * g))
        }).sum::<f64>()
    };
    let mut ts = vec![0.0];
    ts.extend(fr.iter().map(|f| f * FRAC_PI_2));
    ts.push(FRAC_PI_2);
    let w: Vec<f64> = ts.windows(2).map(|p| integral(p[0], p[1])).collect();
    // The model integrates with 64 n trapezoid panels: equal to ~1e-4 relative.
    assert!(rel_spread(&w) < 2e-4, "{w:?}");
}

#[test]
fn curvature_refuses_a_density_it_cannot_resolve() {
    let e = ellipse(2.0, 1.0);
    let cc = carrier(0.0, FRAC_PI_2, Sense::Forward);
    assert!(matches!(node_params(&e, cc, 10, &Spec::Curvature { factor: 1e9 }), Err(SpacingError::Unreachable(_))));
    assert!(matches!(node_params(&e, cc, 10, &Spec::Curvature { factor: f64::NAN }), Err(SpacingError::BadValue { .. })));
}

#[test]
fn degenerate_inputs_are_errors_not_panics() {
    let c = circle(1.0);
    assert!(matches!(node_params(&c, carrier(0.0, 1.0, Sense::Forward), 0, &Spec::Equal), Err(SpacingError::ZeroCount)));
    assert!(node_params(&c, carrier(0.0, 1.0, Sense::Forward), 1, &Spec::Equal).unwrap().is_empty());
    assert!(matches!(node_params(&c, carrier(1.0, 1.0, Sense::Forward), 4, &Spec::Equal), Err(SpacingError::Geometry(_))));
    assert!(matches!(node_params(&c, carrier(0.0, f64::INFINITY, Sense::Forward), 4, &Spec::Equal), Err(SpacingError::Geometry(_))));
    // A cusp-free but far-too-long first size.
    let bad = Spec::Bias { mode: BiasMode::First(100.0), reverse: false };
    assert!(matches!(node_params(&c, carrier(0.0, PI, Sense::Forward), 4, &bad), Err(SpacingError::Unreachable(_))));
}

#[test]
fn node_params_are_bit_identical_for_1_and_n_threads() {
    let w = nurbs_wiggly();
    let cc = carrier(0.0, 1.0, Sense::Forward);
    let e = ellipse(2.0, 1.0);
    let cce = carrier(0.0, FRAC_PI_2, Sense::Forward);
    // n above the parallel thresholds so the rayon paths run.
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| {
            let mut v = node_params(&w, cc, 5000, &Spec::Equal).unwrap();
            v.extend(node_params(&w, cc, 3000, &Spec::Bias { mode: BiasMode::Ratio(1.0007), reverse: true }).unwrap());
            v.extend(node_params(&e, cce, 2000, &Spec::Curvature { factor: 3.0 }).unwrap());
            v
        })
    };
    let threads = std::thread::available_parallelism().map_or(4, |n| n.get().max(2));
    sm_testkit::determinism::check_f64_bits(2, threads, run).unwrap();
}
