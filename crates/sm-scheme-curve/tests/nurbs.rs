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

// ---- very short knot spans (breakpoint-seeded arc map) ----

/// Clamped knot vector: `d + 1` end knots and each interior `(value, multiplicity)`.
fn clamped(d: usize, interior: &[(f64, usize)]) -> Vec<f64> {
    let mut k = vec![0.0; d + 1];
    for &(v, m) in interior {
        k.extend(std::iter::repeat_n(v, m));
    }
    k.extend(std::iter::repeat_n(1.0, d + 1));
    k
}

fn short_span_case(d: usize, interior: &[(f64, usize)], name: &str) {
    let k = clamped(d, interior);
    let c = spline(d, k.clone(), k.len() - d - 1);
    check_equal(&c, &k, name, &[5, 13], 1e-9);
}

#[test]
fn degree_one_short_span_mid_and_far_end() {
    let a = 20.0 / 64.0 + 3e-4;
    short_span_case(1, &[(a, 1), (a + 1e-6, 1)], "polyline short span near 20/64");
    let b = 1.0 - 7e-4;
    short_span_case(1, &[(b, 1), (b + 1e-6, 1)], "polyline short span near the far end");
}

#[test]
fn repeated_knots_at_a_short_span() {
    let a = 20.0 / 64.0 + 3e-4;
    short_span_case(2, &[(a, 2), (a + 1e-6, 2)], "quadratic double knots, short span");
    short_span_case(3, &[(a, 3), (a + 1e-6, 3)], "cubic triple knots, short span");
    let b = 1.0 - 7e-4;
    short_span_case(2, &[(b, 2), (b + 1e-6, 2)], "quadratic double knots, far end");
    short_span_case(3, &[(b, 3), (b + 1e-6, 3)], "cubic triple knots, far end");
}

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> f64 {
        // splitmix64
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Reference arc length of the parameter interval `[a, b]`, each knot span integrated on its
/// own with the panel count doubled until it settles.
fn ref_len(c: &dyn ParametricCurve, knots: &[f64], a: f64, b: f64) -> f64 {
    let mut cuts = vec![a];
    cuts.extend(knots.iter().copied().filter(|k| *k > a && *k < b));
    cuts.dedup();
    cuts.push(b);
    let estimate = |a: f64, b: f64, panels: usize| -> f64 {
        let h = (b - a) / panels as f64;
        let mut s = 0.0;
        for k in 0..panels {
            let mid = a + (k as f64 + 0.5) * h;
            for (x, wt) in GL_X.iter().zip(GL_W) {
                s += wt * c.d1(mid + 0.5 * h * x).norm();
            }
        }
        s * 0.5 * h
    };
    cuts.windows(2)
        .map(|w| {
            let (mut panels, mut prev) = (32, estimate(w[0], w[1], 32));
            loop {
                panels *= 2;
                let next = estimate(w[0], w[1], panels);
                if (next - prev).abs() <= 1e-11 * next.abs() || panels >= 1 << 14 {
                    return next;
                }
                prev = next;
            }
        })
        .sum()
}

fn carrier_r(sense: Sense, lo: f64, hi: f64) -> CurveCarrier {
    CurveCarrier { curve: sm_core::EntityId::new(sm_core::EntityKind::Curve, 0).unwrap(), range: Interval::new(lo, hi), sense }
}

/// Worst `|segment - want| / allowance` over the `n` equal-arc segments of the edge `[lo, hi]`.
/// The allowance is 1e-9 relative plus the double-precision limit of the parameter: a node
/// cannot be placed finer than one ulp of `t`, which is `max speed x eps x max(|lo|, |hi|)` of
/// arc length (relevant only where a short span carries large speed). A ratio at most 1 passes.
fn worst_ratio(c: &dyn ParametricCurve, knots: &[f64], lo: f64, hi: f64, sense: Sense, n: u32) -> f64 {
    let fr = node_params(c, carrier_r(sense, lo, hi), n, &Spec::Equal).unwrap();
    let fr: Vec<f64> = if sense == Sense::Reversed { fr.iter().rev().map(|f| 1.0 - f).collect() } else { fr };
    let mut ts = vec![lo];
    ts.extend(fr.iter().map(|f| lo + f * (hi - lo)));
    ts.push(hi);
    let want = ref_len(c, knots, lo, hi) / f64::from(n);
    // Largest speed over the edge, sampled inside every knot span.
    let mut edges = vec![lo];
    edges.extend(knots.iter().copied().filter(|k| *k > lo && *k < hi));
    edges.dedup();
    edges.push(hi);
    let vmax = edges.windows(2).flat_map(|w| (0..32).map(move |j| w[0] + (w[1] - w[0]) * (f64::from(j) + 0.5) / 32.0)).map(|t| c.d1(t).norm()).fold(0.0, f64::max);
    let tol = 1e-9 * want + f64::EPSILON * lo.abs().max(hi.abs()).max(1.0) * vmax;
    ts.windows(2).map(|w| (ref_len(c, knots, w[0], w[1]) - want).abs() / tol).fold(0.0, f64::max)
}

fn check_ratio(c: &dyn ParametricCurve, knots: &[f64], lo: f64, hi: f64, n: u32, name: &str) {
    for sense in [Sense::Forward, Sense::Reversed] {
        let r = worst_ratio(c, knots, lo, hi, sense, n);
        assert!(r <= 1.0, "{name} [{lo}, {hi}] {sense:?} n={n}: error is {r} x the allowance");
    }
}

/// Spline through wavy control points on `[lo, lo + w]`, knots given as fractions of `w`.
fn shifted(d: usize, unit_knots: &[f64], lo: f64, w: f64, weights: bool) -> (NurbsCurve, Vec<f64>) {
    let k: Vec<f64> = unit_knots.iter().map(|u| lo + w * u).collect();
    let np = k.len() - d - 1;
    let c = spline(d, k.clone(), np);
    let c = if weights {
        let wts = (0..np).map(|i| 0.6 + 0.8 * ((i * 7 % 5) as f64) / 4.0).collect();
        NurbsCurve::new(d, k.clone(), c.control, Some(wts)).unwrap()
    } else {
        c
    };
    (c, k)
}

/// Double `k` ulps above (`k > 0`) or below (`k < 0`) `x` (positive `x`).
fn ulps(x: f64, k: i32) -> f64 {
    f64::from_bits((x.to_bits() as i64 + i64::from(k)) as u64)
}

#[test]
fn edge_ends_within_ulps_of_a_knot_get_their_own_panel() {
    // Degree 1, a very short fast span [k1, k2]; edges end 0..8 ulps past and before a knot at
    // either end. The sliver beyond the knot must not be integrated at the fast piece's speed.
    let (k1, k2) = (0.500_108_071_832_742_9, 0.500_108_192_401_624_9);
    let knots = vec![0.0, 0.0, 0.25, 0.4, k1, k2, 0.7, 0.9, 1.0, 1.0];
    let c = spline(1, knots.clone(), knots.len() - 2);
    for k in -8..=8 {
        check_ratio(&c, &knots, 0.0, ulps(k2, k), 7, &format!("hi = k2 {k:+} ulps"));
        check_ratio(&c, &knots, 0.0, ulps(k1, k), 7, &format!("hi = k1 {k:+} ulps"));
        check_ratio(&c, &knots, ulps(k1, k), 1.0, 7, &format!("lo = k1 {k:+} ulps"));
        check_ratio(&c, &knots, ulps(k2, k), 1.0, 7, &format!("lo = k2 {k:+} ulps"));
    }
}

#[test]
fn quadratic_short_span_away_from_zero_is_not_a_geometry_error() {
    // Minimal repro: the speed inside the 1.63e-7 span carries parameter-scale noise at |t| ~ 100.
    let a = 0.431_845_142_746_103_9;
    for off in [0.0, 100.0, -7.5, 1000.0] {
        let unit = [0.0, 0.0, 0.0, a, a, a + 1.63e-7, 1.0, 1.0, 1.0];
        let (c, k) = shifted(2, &unit, off, 1.0, false);
        check_ratio(&c, &k, off, off + 1.0, 6, &format!("offset {off}"));
    }
}

#[test]
fn short_spans_with_shifted_parameter_ranges() {
    let a = 20.0 / 64.0 + 3e-4;
    let b = 1.0 - 7e-4;
    for off in [100.0, -7.5] {
        for (d, mult) in [(1usize, 1usize), (2, 2), (3, 3)] {
            for start in [a, b] {
                let mut unit = vec![0.0; d + 1];
                unit.extend(std::iter::repeat_n(start, mult));
                unit.extend(std::iter::repeat_n(start + 1e-6, mult));
                unit.extend(std::iter::repeat_n(1.0, d + 1));
                let (c, k) = shifted(d, &unit, off, 1.0, false);
                check_ratio(&c, &k, off, off + 1.0, 7, &format!("degree {d} offset {off} start {start}"));
            }
        }
    }
}

#[test]
fn sub_range_edge_on_a_shifted_curve() {
    let a = 0.4;
    let unit = [0.0, 0.0, 0.0, a, a, a + 2e-6, 0.9, 1.0, 1.0, 1.0];
    for off in [100.0, -7.5] {
        let (c, k) = shifted(2, &unit, off, 2.0, false);
        // Edge ranges containing the short span, ending at a knot, and cutting through it.
        for (x, y) in [(0.1, 0.95), (0.05, a + 1e-6), (a + 1e-6, 0.8), (a - 0.1, a + 2e-6 + 0.2)] {
            check_ratio(&c, &k, off + 2.0 * x, off + 2.0 * y, 5, &format!("sub-range offset {off}"));
        }
    }
}

#[test]
fn periodic_seam_next_to_a_short_span() {
    // Closed degree-1 and degree-2 splines with a 1e-6 span right before the seam.
    for d in [1usize, 2] {
        let (s0, s1, s2) = (0.3, 0.7 - 1e-6, 1e-6);
        let mut k = vec![-s2, 0.0, s0, s0 + s1, 1.0, 1.0 + s0];
        let mut pts = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0), Point3::new(2.5, -1.0, 0.5)];
        if d == 2 {
            // Three unique points need `unique + d` knots: extend the repeating spans.
            k = vec![-s1 - s2, -s2, 0.0, s0, s0 + s1, 1.0, 1.0 + s0, 1.0 + s0 + s1];
        }
        pts.extend(pts.clone().into_iter().take(d));
        let c = NurbsCurve::new_periodic(d, k.clone(), pts, None).unwrap();
        let mut knots = Vec::new();
        for shift in [-1.0, 0.0, 1.0, 2.0] {
            knots.extend(k.iter().map(|x| x + shift));
        }
        knots.sort_by(f64::total_cmp);
        for (lo, hi) in [(0.5, 1.5), (0.9, 1.3), (0.2, 1.2)] {
            check_ratio(&c, &knots, lo, hi, 6, &format!("periodic degree {d}"));
        }
    }
}

/// Random curves: degrees 1-3, spans down to 1e-7 (relative to the range), parameter offsets
/// 0/3/100/-7.5, scales 0.5-10, some rational, some sub-range edges, half reversed.
fn probe(seed: u64, cases: usize, offsets: &[f64]) {
    let mut rng = Rng(seed);
    for case in 0..cases {
        let d = 1 + (rng.next() * 3.0) as usize % 3;
        let width = 10f64.powf(-7.0 + 4.0 * rng.next());
        let start = if rng.next() < 0.3 { 1.0 - width - 10f64.powf(-4.0 + 2.0 * rng.next()) * 0.1 } else { 0.02 + 0.9 * rng.next() };
        let start = start.clamp(0.01, 1.0 - 2.0 * width - 1e-9);
        let mult = |rng: &mut Rng| 1 + (rng.next() * d as f64) as usize % d;
        let mut interior = vec![(start, mult(&mut rng)), (start + width, mult(&mut rng))];
        if rng.next() < 0.5 {
            // An extra knot elsewhere, kept clear of the short span.
            let extra = 0.01 + 0.98 * rng.next();
            if (extra - start).abs() > 1e-2 && (extra - start - width).abs() > 1e-2 {
                interior.push((extra, mult(&mut rng)));
                interior.sort_by(|a, b| a.0.total_cmp(&b.0));
            }
        }
        let unit = clamped(d, &interior);
        let np = unit.len() - d - 1;
        let off = offsets[(rng.next() * offsets.len() as f64) as usize % offsets.len()];
        let scale = 0.5 + 9.5 * rng.next();
        let k: Vec<f64> = unit.iter().map(|u| off + scale * u).collect();
        let control = (0..np).map(|_| Point3::new(2.0 * rng.next() - 1.0, 2.0 * rng.next() - 1.0, 2.0 * rng.next() - 1.0)).collect();
        let weights = (rng.next() < 0.3).then(|| (0..np).map(|_| 0.5 + 1.5 * rng.next()).collect());
        let c = NurbsCurve::new(d, k.clone(), control, weights).unwrap();
        let (mut lo, mut hi) = (off, off + scale);
        if rng.next() < 0.3 {
            lo += scale * 0.2 * rng.next();
            hi -= scale * 0.2 * rng.next();
        }
        let sense = if case % 2 == 0 { Sense::Forward } else { Sense::Reversed };
        let r = worst_ratio(&c, &k, lo, hi, sense, 6);
        assert!(r <= 1.0, "case {case} d={d} off={off} scale={scale} knots={interior:?} range=[{lo}, {hi}] {sense:?}: error is {r} x the allowance");
    }
}

#[test]
fn seeded_random_probe_with_short_spans() {
    probe(0x5eed_c0de, 200, &[0.0]);
}

#[test]
fn seeded_random_probe_with_shifted_ranges() {
    probe(0x0ff5_e7ed, 300, &[0.0, 3.0, 100.0, -7.5]);
}

#[test]
fn dense_knot_vectors_and_absurd_period_counts_are_typed() {
    // 70000 interior knots exceed the plain panel budget but are a valid curve.
    let n = 70_000usize;
    let mut k = vec![0.0, 0.0];
    k.extend((1..=n).map(|i| i as f64 / (n + 1) as f64));
    k.extend([1.0, 1.0]);
    let control = (0..n + 2).map(|i| Point3::new(i as f64, f64::from((i % 7) as u8), 0.0)).collect();
    let c = NurbsCurve::new(1, k, control, None).unwrap();
    let fr = node_params(&c, carrier(Sense::Forward), 4, &Spec::Equal).unwrap();
    assert_eq!(fr.len(), 3);
    // An edge spanning a huge number of periods is refused with a typed error, not a long loop.
    let (s0, s1, s2) = (0.3, 0.4, 0.3);
    let pts = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 2.0, 0.0), Point3::new(2.5, -1.0, 0.5), Point3::new(0.0, 0.0, 0.0)];
    let p = NurbsCurve::new_periodic(1, vec![-s2, 0.0, s0, s0 + s1, 1.0, 1.0 + s0], pts, None).unwrap();
    match node_params(&p, carrier_r(Sense::Forward, 0.0, 1e12), 4, &Spec::Equal) {
        Err(SpacingError::Geometry(_)) => {}
        other => panic!("expected a geometry error, got {other:?}"),
    }
}
