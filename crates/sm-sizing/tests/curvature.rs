//! Opt-in curvature, point and grade size fields against closed forms.
mod common;

use std::sync::atomic::{AtomicU64, Ordering};

use proptest::prelude::*;
use sm_core::Point3;
use sm_scheme_api::geom::{edge_point, edge_size_goal};
use sm_scheme_api::stubs::brick_setup;
use sm_scheme_api::{Params, SchemeError, SizeField, SizingSpec};
use sm_sizing::{Graded, size_field, size_field_geo};
use sm_topo_api::fixtures::{CylinderIds, SphereIds, TorusIds, cylinder, sphere, torus, with_op};
use sm_topo_api::{CellComplex, OpId};

fn func(name: &str, params: serde_json::Value) -> SizingSpec {
    SizingSpec::Function { name: name.into(), params: Params(params) }
}

fn geo(topo: &CellComplex, spec: &SizingSpec) -> Box<dyn SizeField> {
    size_field_geo(topo, &[], spec).expect("field").expect("some")
}

/// The chord-height rule: the longest chord of a circle of radius `r` with sagitta `t`.
fn rule(t: f64, r: f64) -> f64 {
    2.0 * (t * (2.0 * r - t)).sqrt()
}

fn cyl(r: f64, h: f64) -> (CellComplex, CylinderIds) {
    let mut topo = CellComplex::new();
    let ids = with_op(&mut topo, OpId(1), |t| cylinder(t, r, h)).expect("cylinder");
    (topo, ids)
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

#[test]
fn circle_edge_size_is_the_chord_height_rule_to_1e_12() {
    let (r, t) = (2.5, 0.01);
    let (topo, ids) = cyl(r, 4.0);
    let f = geo(&topo, &func("curvature", serde_json::json!({ "tol": t, "h_max": 100.0, "grade": "off" })));
    let want = rule(t, r);
    for i in 0..100 {
        let p = edge_point(&topo, ids.edges[0], f64::from(i) / 100.0).expect("point");
        assert!(rel(f.size(p), want) < 1e-12, "i={i}: {} vs {want}", f.size(p));
    }
    // The goal over the whole circle is 2 pi r / h exactly (constant field).
    let goal = edge_size_goal(&topo, ids.edges[0], &*f).expect("goal");
    assert!(rel(goal, 2.0 * std::f64::consts::PI * r / want) < 1e-9);
}

#[test]
fn meshed_circle_stays_within_the_tolerance_scaled_by_the_rounded_count() {
    let t = 0.01;
    for r in [0.5, 1.0, 2.5, 10.0] {
        let (topo, ids) = cyl(r, 1.0);
        let f = geo(&topo, &func("curvature", serde_json::json!({ "tol": t, "h_max": 1e6, "grade": "off" })));
        let n_goal = edge_size_goal(&topo, ids.edges[0], &*f).expect("goal");
        // The solver rounds the goal; both neighbours are checked.
        for n in [n_goal.floor(), n_goal.ceil()] {
            // Equal arc spacing: the sagitta of a chord spanning 2 pi / n on the exact circle.
            let sag = r * (1.0 - (std::f64::consts::PI / n).cos());
            assert!(sag <= t * (n_goal / n).powi(2) * (1.0 + 1e-12), "r={r} n={n} sag={sag}");
        }
    }
}

#[test]
fn cylinder_and_sphere_surfaces_use_the_smaller_principal_radius() {
    let (r, t) = (1.7, 0.02);
    let spec = func("curvature", serde_json::json!({ "tol": t, "h_max": 100.0, "grade": "off" }));
    let want = rule(t, r);
    let (topo, _) = cyl(r, 3.0);
    let f = geo(&topo, &spec);
    for i in 1..20 {
        for j in 1..6 {
            let a = 0.3 * f64::from(i);
            let p = Point3::new(r * a.cos(), r * a.sin(), 0.5 * f64::from(j));
            assert!(rel(f.size(p), want) < 1e-12, "{p:?}");
        }
    }
    // The sphere: radius r in both directions, away from the poles.
    let mut topo = CellComplex::new();
    let _s: SphereIds = with_op(&mut topo, OpId(1), |t| sphere(t, r)).expect("sphere");
    let f = geo(&topo, &spec);
    for i in 0..13 {
        for j in -4..=4 {
            let (a, b) = (0.5 * f64::from(i), 0.3 * f64::from(j));
            let p = Point3::new(r * b.cos() * a.cos(), r * b.cos() * a.sin(), r * b.sin());
            assert!(rel(f.size(p), want) < 1e-12, "{p:?}: {}", f.size(p));
        }
    }
    // The torus outer half: the tube radius is the smaller principal radius.
    let (big, small) = (3.0, 0.6);
    let mut topo = CellComplex::new();
    let _t: TorusIds = with_op(&mut topo, OpId(1), |t| torus(t, big, small)).expect("torus");
    let f = geo(&topo, &spec);
    let want = rule(t, small);
    for i in 0..9 {
        for j in -3..=3 {
            let (u, v) = (0.7 * f64::from(i), 0.3 * f64::from(j));
            let rho = big + small * v.cos();
            let p = Point3::new(rho * u.cos(), rho * u.sin(), small * v.sin());
            assert!(rel(f.size(p), want) < 1e-12, "{p:?}: {}", f.size(p));
        }
    }
}

#[test]
fn a_plane_and_far_points_give_h_max() {
    let (topo, _) = cyl(1.0, 2.0);
    let f = geo(&topo, &func("curvature", serde_json::json!({ "tol": 0.01, "h_max": 7.0, "grade": "off" })));
    // Interior of the bottom cap (a plane), a point inside the solid, and one far away.
    for p in [Point3::new(0.3, 0.2, 0.0), Point3::new(0.0, 0.0, 1.0), Point3::new(50.0, 0.0, 9.0)] {
        assert_eq!(f.size(p), 7.0);
    }
    // A brick has no curved feature at all.
    let (b, _, _) = brick_setup([1.0, 1.0, 1.0]);
    let f = geo(&b, &func("curvature", serde_json::json!({ "tol": 0.01, "h_max": 3.0, "grade": 0.5 })));
    assert_eq!(f.size(Point3::new(0.2, 0.3, 0.4)), 3.0);
}

#[test]
fn angle_cap_and_large_tolerance() {
    let r = 2.0;
    let (topo, ids) = cyl(r, 1.0);
    let p = edge_point(&topo, ids.edges[0], 0.25).expect("point");
    // Angle cap: h <= r * angle (tighter than the sagitta rule for a tight tolerance).
    let f = geo(&topo, &func("curvature", serde_json::json!({ "tol": 0.5, "h_max": 100.0, "angle": 0.1, "grade": "off" })));
    assert!(rel(f.size(p), r * 0.1) < 1e-12);
    // tol >= r: every chord is fine, the diameter 2 r is the largest.
    let f = geo(&topo, &func("curvature", serde_json::json!({ "tol": 2.0, "h_max": 100.0, "grade": "off" })));
    assert!(rel(f.size(p), 2.0 * r) < 1e-12);
}

#[test]
fn grade_is_exact_for_every_pair_of_samples() {
    let (r, t, g) = (1.0, 0.002, 0.4);
    let (topo, _) = cyl(r, 3.0);
    let f = geo(&topo, &func("curvature", serde_json::json!({ "tol": t, "h_max": 2.0, "grade": g, "samples": 6 })));
    // A coarse grid of points over and around the cylinder.
    let mut pts = Vec::new();
    for i in 0..8 {
        for j in 0..4 {
            let a = 0.8 * f64::from(i);
            pts.push(Point3::new(r * a.cos(), r * a.sin(), 1.0 * f64::from(j)));
            pts.push(Point3::new(0.4 * a.cos(), 0.4 * a.sin(), 1.0 * f64::from(j)));
        }
    }
    pts.push(Point3::new(5.0, 5.0, 5.0));
    for &p in &pts {
        for &q in &pts {
            let d = ((p.x - q.x).powi(2) + (p.y - q.y).powi(2) + (p.z - q.z).powi(2)).sqrt();
            assert!(f.size(p) <= f.size(q) + g * d + 1e-12, "{p:?} {q:?}");
        }
    }
    // On the wall the graded size equals the rule (the samples there are the minimum); far
    // away it rises by g per unit distance until h_max.
    let want = rule(t, r);
    assert!(rel(f.size(Point3::new(r, 0.0, 1.0)), want) < 1e-9);
    assert!(f.size(Point3::new(0.0, 0.0, 1.0)) > want);
    assert_eq!(f.size(Point3::new(500.0, 0.0, 0.0)), 2.0);
    // The same through the wrapper.
    let w = geo(
        &topo,
        &func(
            "grade",
            serde_json::json!({ "grade": g, "of": { "name": "curvature", "params": { "tol": t, "h_max": 2.0, "samples": 6 } } }),
        ),
    );
    for &p in &pts {
        assert_eq!(w.size(p).to_bits(), f.size(p).to_bits());
    }
}

#[test]
fn the_default_grade_is_one_half_and_the_wall_value_is_the_rule() {
    let (r, t) = (1.0, 0.002);
    let (topo, _) = cyl(r, 3.0);
    let d = geo(&topo, &func("curvature", serde_json::json!({ "tol": t, "h_max": 2.0 })));
    let e = geo(&topo, &func("curvature", serde_json::json!({ "tol": t, "h_max": 2.0, "grade": 0.5 })));
    for i in 0..50 {
        let p = Point3::new(0.02 * f64::from(i), 0.1, 0.05 * f64::from(i));
        assert_eq!(d.size(p).to_bits(), e.size(p).to_bits());
    }
    // A sample of the wall lies on a sample point of the circle edge.
    assert!(rel(d.size(Point3::new(r, 0.0, 0.0)), rule(t, r)) < 1e-12);
}

#[test]
fn point_source_grades_exactly_on_a_box() {
    let (topo, _, _) = brick_setup([1.0, 1.0, 1.0]);
    let (h0, g, hm) = (0.05f64, 0.3f64, 0.5f64);
    let x0 = Point3::new(0.2, 0.1, 0.0);
    let f = geo(&topo, &func("point", serde_json::json!({ "points": [[[0.2, 0.1, 0.0], h0]], "grade": g, "h_max": hm })));
    for i in 0..10 {
        for j in 0..10 {
            let p = Point3::new(0.1 * f64::from(i), 0.1 * f64::from(j), 0.7);
            let d = ((p.x - x0.x).powi(2) + (p.y - x0.y).powi(2) + (p.z - x0.z).powi(2)).sqrt();
            let want = hm.min(h0 + g * d);
            assert!(rel(f.size(p), want) < 1e-15, "{p:?}");
        }
    }
    // Two sources: the smaller wins at each point.
    let f2 = geo(&topo, &func("point", serde_json::json!({ "points": [[[0.0, 0.0, 0.0], 0.1], [[1.0, 0.0, 0.0], 0.02]], "grade": 0.2, "h_max": 1.0 })));
    let p = Point3::new(0.9, 0.0, 0.0);
    assert!(rel(f2.size(p), 0.02 + 0.2 * 0.1) < 1e-15);
    // The wrapper form.
    let w = geo(&topo, &func("grade", serde_json::json!({ "grade": g, "of": { "name": "point", "params": { "points": [[[0.2, 0.1, 0.0], h0]], "h_max": hm } } })));
    assert_eq!(w.size(Point3::new(0.5, 0.5, 0.5)).to_bits(), f.size(Point3::new(0.5, 0.5, 0.5)).to_bits());
}

#[test]
fn bad_parameters_name_the_key() {
    let (topo, _) = cyl(1.0, 1.0);
    let cases = [
        ("curvature", serde_json::json!({ "tol": 0.0, "h_max": 1.0 }), "tol"),
        ("curvature", serde_json::json!({ "tol": -1.0, "h_max": 1.0 }), "tol"),
        ("curvature", serde_json::json!({ "tol": 0.1 }), "h_max"),
        ("curvature", serde_json::json!({ "tol": 0.1, "h_max": 1.0, "grade": -0.1 }), "grade"),
        ("curvature", serde_json::json!({ "tol": 0.1, "h_max": 1.0, "grade": "fine" }), "grade"),
        ("curvature", serde_json::json!({ "tol": 0.1, "h_max": 1.0, "h_min": 2.0 }), "h_min"),
        ("curvature", serde_json::json!({ "tol": 0.1, "h_max": 1.0, "samples": 1 }), "samples"),
        ("point", serde_json::json!({ "points": [], "h_max": 1.0 }), "grade"),
        ("point", serde_json::json!({ "points": [[[0.0, 0.0], 1.0]], "h_max": 1.0, "grade": 0.1 }), "points[0]"),
        ("grade", serde_json::json!({ "grade": -1.0, "of": { "name": "point", "params": {} } }), "grade"),
        ("grade", serde_json::json!({ "grade": 1.0, "of": { "name": "linear", "params": {} } }), "of.name"),
    ];
    for (name, params, key) in cases {
        match size_field_geo(&topo, &[], &func(name, params.clone())) {
            Err(SchemeError::BadParams { scheme, reason }) => {
                assert!(scheme.0.contains(name), "{scheme:?}");
                assert!(reason.contains(key), "{name} {params}: {reason}");
            }
            other => panic!("{name} {params}: {:?}", other.map(|o| o.is_some())),
        }
    }
    // The plain builder keeps refusing the geometry functions (they need the view).
    assert!(size_field(&func("curvature", serde_json::json!({ "tol": 0.1, "h_max": 1.0 }))).is_err());
    // Other specs pass through unchanged.
    let f = size_field_geo(&topo, &[], &SizingSpec::Constant { size: 0.25 }).expect("ok").expect("some");
    assert_eq!(f.size(Point3::new(1.0, 2.0, 3.0)), 0.25);
}

/// Counts evaluations so the edge-goal cost is visible.
struct Counting<'a>(&'a dyn SizeField, AtomicU64);
impl SizeField for Counting<'_> {
    fn size(&self, p: Point3) -> f64 {
        self.1.fetch_add(1, Ordering::Relaxed);
        self.0.size(p)
    }
}

#[test]
fn edge_goal_cost_on_a_curved_model_stays_below_the_panel_cap() {
    // Panels = evaluations / 5 per estimate, doubling from 4; the cap is 2^14.
    let (topo, ids) = cyl(1.0, 3.0);
    for spec in [
        func("curvature", serde_json::json!({ "tol": 0.001, "h_max": 2.0, "grade": "off" })),
        func("curvature", serde_json::json!({ "tol": 0.001, "h_max": 2.0, "grade": 0.3 })),
    ] {
        let f = geo(&topo, &spec);
        for (k, &e) in ids.edges.iter().enumerate() {
            let c = Counting(&*f, AtomicU64::new(0));
            let goal = edge_size_goal(&topo, e, &c).expect("goal");
            let evals = c.1.load(Ordering::Relaxed);
            let last_panels = evals / 5 / 2; // the last two estimates use p and p/2 panels
            eprintln!("edge {k}: goal {goal:.3}, {evals} evaluations (~{last_panels} panels at the end)");
            assert!(evals < 5 * 2 * (1 << 14), "edge {k} hit the panel cap");
        }
    }
}

#[test]
fn bit_identical_across_runs_and_thread_counts() {
    let (topo, ids) = cyl(1.3, 2.0);
    let spec = func("curvature", serde_json::json!({ "tol": 0.003, "h_max": 1.0, "grade": 0.25, "angle": 0.5 }));
    let bits = |threads: usize| -> Vec<f64> {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool");
        pool.install(|| {
            let f = geo(&topo, &spec);
            let mut v: Vec<f64> = (0..200).map(|i| f.size(Point3::new(0.01 * f64::from(i), 0.5, 0.01 * f64::from(i * 7 % 200)))).collect();
            v.push(edge_size_goal(&topo, ids.edges[2], &*f).expect("goal"));
            v
        })
    };
    sm_testkit::determinism::check_f64_bits(2, 4, bits).expect("bit-identical");
}

proptest! {
    #![proptest_config(sm_testkit::strategies::config(64))]

    /// The kd-tree query equals the brute-force minimum and is `g`-Lipschitz.
    #[test]
    fn graded_matches_brute_force_and_is_lipschitz(
        pts in prop::collection::vec((-5.0f64..5.0, -5.0f64..5.0, -5.0f64..5.0, 0.01f64..2.0), 1..40),
        g in 0.0f64..2.0,
        q in (-6.0f64..6.0, -6.0f64..6.0, -6.0f64..6.0),
        r in (-6.0f64..6.0, -6.0f64..6.0, -6.0f64..6.0),
    ) {
        let samples: Vec<(Point3, f64)> = pts.iter().map(|&(x, y, z, h)| (Point3::new(x, y, z), h)).collect();
        let f = Graded::new(&samples, g, 3.0);
        let dist = |a: Point3, b: Point3| ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt();
        let brute = |p: Point3| samples.iter().fold(3.0f64, |m, &(s, h)| m.min(h + g * dist(p, s)));
        let (p, p2) = (Point3::new(q.0, q.1, q.2), Point3::new(r.0, r.1, r.2));
        prop_assert_eq!(f.size(p).to_bits(), brute(p).to_bits());
        prop_assert!(f.size(p) <= f.size(p2) + g * dist(p, p2) + 1e-12);
    }
}

/// With the default grade the field honours the chord-height rule on the carriers at dense
/// non-sample points, including when the tolerance is far below the sample spacing.
#[test]
fn default_grade_meets_the_rule_on_the_carriers_between_samples() {
    const DELTA: f64 = 0.05;
    for (r, t) in [(1.0, 0.01), (1.0, 0.001), (0.2, 0.001)] {
        let want = rule(t, r);
        let spec = func("curvature", serde_json::json!({ "tol": t, "h_max": 2.0 }));
        let (topo, ids) = cyl(r, 2.0);
        let f = geo(&topo, &spec);
        let mut worst: f64 = 0.0;
        for i in 0..400 {
            for j in 0..61 {
                let a = 0.0173 + 0.0157 * f64::from(i);
                let p = Point3::new(r * a.cos(), r * a.sin(), 2.0 * (0.0071 + f64::from(j) / 61.0).min(1.0));
                worst = worst.max(f.size(p) / want);
            }
        }
        assert!(worst <= 1.0 + DELTA, "cylinder r={r} t={t}: size/rule {worst}");
        for i in 0..500 {
            let p = edge_point(&topo, ids.edges[0], 0.00131 + f64::from(i) / 500.0).expect("point");
            assert!(f.size(p) <= want * (1.0 + DELTA), "edge r={r} t={t}");
        }
        let goal = edge_size_goal(&topo, ids.edges[0], &*f).expect("goal");
        let exact = 2.0 * std::f64::consts::PI * r / want;
        assert!(goal >= exact / (1.0 + DELTA) && goal <= exact * (1.0 + 1e-6), "r={r} t={t}: goal {goal} vs {exact}");
        // The sphere, away from the poles.
        let mut sph = CellComplex::new();
        let _s: SphereIds = with_op(&mut sph, OpId(1), |t| sphere(t, r)).expect("sphere");
        let f = geo(&sph, &spec);
        let mut worst: f64 = 0.0;
        for i in 0..200 {
            for j in -60..=60 {
                let (a, b) = (0.0191 + 0.0313 * f64::from(i), 0.0127 * f64::from(j) + 0.003);
                let p = Point3::new(r * b.cos() * a.cos(), r * b.cos() * a.sin(), r * b.sin());
                worst = worst.max(f.size(p) / want);
            }
        }
        assert!(worst <= 1.0 + DELTA, "sphere r={r} t={t}: size/rule {worst}");
    }
}
