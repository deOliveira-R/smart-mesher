//! Property tests on random NURBS: the prepared evaluator agrees with the reference, editing
//! keeps the shape, projection is globally optimal.

use proptest::prelude::*;
use sm_core::Point3;
use sm_geom::{PreparedCurve, PreparedSurface};
use sm_geom_api::{NurbsCurve, NurbsSurface, ParametricCurve, ParametricSurface, Uv};
use sm_testkit::strategies::config;

fn knots(p: usize, gaps: &[f64]) -> Vec<f64> {
    let total: f64 = gaps.iter().sum();
    let mut k = vec![0.0; p + 1];
    let mut acc = 0.0;
    for g in &gaps[..gaps.len() - 1] {
        acc += g / total;
        k.push(acc);
    }
    k.extend(vec![1.0; p + 1]);
    k
}

fn curve_strategy() -> impl Strategy<Value = NurbsCurve> {
    (1usize..=5).prop_flat_map(|p| {
        (Just(p), 1usize..=6).prop_flat_map(move |(p, spans)| {
            let n = p + spans;
            (
                Just(p),
                proptest::collection::vec(0.1f64..1.0, spans),
                proptest::collection::vec((-3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0), n),
                proptest::collection::vec(0.4f64..2.5, n),
                any::<bool>(),
            )
                .prop_map(|(p, gaps, pts, w, rational)| {
                    let control = pts.into_iter().map(|(x, y, z)| Point3::new(x, y, z)).collect();
                    NurbsCurve::new(p, knots(p, &gaps), control, rational.then_some(w)).unwrap()
                })
        })
    })
}

fn surface_strategy() -> impl Strategy<Value = NurbsSurface> {
    (1usize..=3, 1usize..=3, 1usize..=3, 1usize..=3).prop_flat_map(|(pu, pv, su, sv)| {
        let (nu, nv) = (pu + su, pv + sv);
        (
            Just((pu, pv, nu, nv)),
            proptest::collection::vec(0.1f64..1.0, su),
            proptest::collection::vec(0.1f64..1.0, sv),
            proptest::collection::vec((-2.0f64..2.0, -2.0f64..2.0, -1.0f64..1.0), nu * nv),
            proptest::collection::vec(0.5f64..2.0, nu * nv),
            any::<bool>(),
        )
            .prop_map(|((pu, pv, nu, nv), gu, gv, pts, w, rational)| {
                let control = pts.into_iter().map(|(x, y, z)| Point3::new(x, y, z)).collect();
                NurbsSurface::new(pu, pv, knots(pu, &gu), knots(pv, &gv), nu, nv, control, rational.then_some(w)).unwrap()
            })
    })
}

proptest! {
    #![proptest_config(config(64))]

    #[test]
    fn prepared_curve_equals_reference_and_edits_keep_shape(c in curve_strategy(), t_ins in 0.05f64..0.95) {
        let pc = PreparedCurve::new(&c).unwrap();
        let refined = pc.insert_knot(t_ins, 1).unwrap();
        let (left, right) = pc.split(t_ins).unwrap();
        for i in 0..=30 {
            let t = i as f64 / 30.0;
            let (a, b) = (c.jet(t), pc.jet(t));
            let scale = 1.0 + a.d2.norm();
            prop_assert!((a.p - b.p).norm() < 1e-12 && (a.d1 - b.d1).norm() < 1e-9 * scale && (a.d2 - b.d2).norm() < 1e-7 * scale);
            prop_assert!((refined.eval(t) - a.p).norm() < 1e-10, "insertion at {t}");
            if t <= t_ins { prop_assert!((left.eval(t) - a.p).norm() < 1e-10, "left at {t}"); }
            if t >= t_ins { prop_assert!((right.eval(t) - a.p).norm() < 1e-10, "right at {t}"); }
        }
        prop_assert!(refined.to_def().unwrap().validate().is_ok());
    }

    #[test]
    fn curve_projection_is_globally_optimal(c in curve_strategy(), qx in -4.0f64..4.0, qy in -4.0f64..4.0, qz in -4.0f64..4.0) {
        let pc = PreparedCurve::new(&c).unwrap();
        let q = Point3::new(qx, qy, qz);
        let pr = pc.project(q, None).unwrap();
        prop_assert!(((pc.eval(pr.t) - q).norm() - pr.distance).abs() < 1e-12);
        let mut best = f64::INFINITY;
        for i in 0..=4000 {
            best = best.min((pc.eval(i as f64 / 4000.0) - q).norm());
        }
        prop_assert!(pr.distance <= best + 1e-9, "projection {} vs dense scan {}", pr.distance, best);
        // Stationarity unless the minimum is at an end.
        let j = pc.jet(pr.t);
        // (A corner at a knot of a low-degree curve is a minimum without a vanishing derivative.)
        let interior = pr.t > 1e-9 && pr.t < 1.0 - 1e-9 && c.knots.iter().all(|k| (k - pr.t).abs() > 1e-7);
        if interior {
            prop_assert!((j.p - q).dot(j.d1).abs() <= 1e-6 * (1.0 + j.d1.norm() * pr.distance));
        }
    }

    #[test]
    fn prepared_surface_equals_reference_and_projection_is_optimal(s in surface_strategy(), t_ins in 0.1f64..0.9, q in (-3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0)) {
        let ps = PreparedSurface::new(&s).unwrap();
        let edited = ps.insert_knot_u(t_ins, 1).unwrap().insert_knot_v(1.0 - t_ins, 1).unwrap();
        for i in 0..=8 {
            for j in 0..=8 {
                let uv = Uv::new(i as f64 / 8.0, j as f64 / 8.0);
                let (a, b) = (s.jet(uv), ps.jet(uv));
                let scale = 1.0 + a.duu.norm() + a.dvv.norm();
                prop_assert!((a.p - b.p).norm() < 1e-12 && (a.du - b.du).norm() < 1e-9 * scale && (a.dv - b.dv).norm() < 1e-9 * scale);
                prop_assert!((a.duv - b.duv).norm() < 1e-7 * scale);
                prop_assert!((edited.eval(uv) - a.p).norm() < 1e-10);
            }
        }
        let q = Point3::new(q.0, q.1, q.2);
        let pr = ps.project(q, None).unwrap();
        let mut best = f64::INFINITY;
        for i in 0..=120 {
            for j in 0..=120 {
                best = best.min((ps.eval(Uv::new(i as f64 / 120.0, j as f64 / 120.0)) - q).norm());
            }
        }
        prop_assert!(pr.distance <= best + 1e-9, "projection {} vs scan {}", pr.distance, best);
    }
}
