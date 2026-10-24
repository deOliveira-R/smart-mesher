//! Property tests on the NURBS classes the basic suite does not generate: repeated interior
//! knots (multiplicity up to `degree + 1`, a C^-1 joint), unclamped non-periodic knots, high
//! degrees (8 to 15, the wide scratch paths) and periodic curves with non-uniform, repeated knots
//! whose period copies differ by a few ulps.

use proptest::prelude::*;
use sm_core::{Point3, Vector3};
use sm_geom::{CurveSurface, PreparedCurve, curve_plane};
use sm_geom_api::{CurveDef, Frame, ModelTolerance, NurbsCurve, ParametricCurve, Plane};
use sm_testkit::strategies::config;

fn mag(p: Point3) -> f64 {
    (p.x * p.x + p.y * p.y + p.z * p.z).sqrt()
}

fn points(n: usize) -> impl Strategy<Value = Vec<Point3>> {
    proptest::collection::vec((-3.0f64..3.0, -3.0f64..3.0, -3.0f64..3.0), n).prop_map(|v| v.into_iter().map(|(x, y, z)| Point3::new(x, y, z)).collect())
}

fn weights(n: usize, rational: bool) -> impl Strategy<Value = Option<Vec<f64>>> {
    proptest::collection::vec(0.4f64..2.5, n).prop_map(move |w| rational.then_some(w))
}

/// A clamped curve on [0, 1] with interior knots of the given multiplicities.
#[derive(Clone, Debug)]
struct Clamped {
    curve: NurbsCurve,
    /// All interior multiplicities are at most the degree (the curve is continuous).
    continuous: bool,
}

fn clamped_strategy() -> impl Strategy<Value = Clamped> {
    let degree = prop_oneof![3 => 1usize..=5, 1 => 8usize..=15];
    degree
        .prop_flat_map(|p| {
            let max_d = if p >= 8 { 2 } else { 3 };
            (0usize..=max_d).prop_flat_map(move |d| {
                let mult = prop_oneof![Just(1usize), Just(p), Just(p + 1), 1usize..=p + 1];
                (Just(p), proptest::collection::vec(mult, d), proptest::collection::vec(0.1f64..1.0, d + 1), any::<bool>())
            })
        })
        .prop_flat_map(|(p, mults, gaps, rational)| {
            let n = p + 1 + mults.iter().sum::<usize>();
            (Just((p, mults, gaps)), points(n), weights(n, rational))
        })
        .prop_map(|((p, mults, gaps), pts, w)| {
            let total: f64 = gaps.iter().sum();
            let mut k = vec![0.0; p + 1];
            let mut acc = 0.0;
            for (g, m) in gaps.iter().zip(&mults) {
                acc += g / total;
                k.extend(std::iter::repeat_n(acc, *m));
            }
            k.extend(vec![1.0; p + 1]);
            let continuous = mults.iter().all(|m| *m <= p);
            Clamped { curve: NurbsCurve::new(p, k, pts, w).unwrap(), continuous }
        })
}

/// An unclamped, non-periodic curve (strictly increasing knots; the domain is `[k_p, k_n]`).
fn unclamped_strategy() -> impl Strategy<Value = NurbsCurve> {
    (1usize..=5, 1usize..=5).prop_flat_map(|(p, spans)| {
        let n = p + spans;
        (Just(p), proptest::collection::vec(0.1f64..1.0, n + p), points(n), weights(n, true)).prop_map(|(p, gaps, pts, w)| {
            let mut k = vec![0.0];
            for g in &gaps {
                k.push(k.last().unwrap() + g);
            }
            NurbsCurve::new(p, k, pts, w).unwrap()
        })
    })
}

fn check_curve(c: &NurbsCurve, t_ins: f64, exact_samples: bool) -> Result<(), TestCaseError> {
    let pc = PreparedCurve::new(c).unwrap();
    let r = c.range();
    let at = |s: f64| r.lo + (r.hi - r.lo) * s;
    let refined = pc.insert_knot(at(t_ins), 1).unwrap();
    let (left, right) = pc.split(at(t_ins)).unwrap();
    for i in 0..=40 {
        let t = at((i as f64 + 0.37) / 41.0);
        let (a, b) = (c.jet(t), pc.jet(t));
        prop_assert!((a.p - b.p).norm() < 1e-10 * (1.0 + mag(a.p)), "position at {t}");
        prop_assert!((a.d1 - b.d1).norm() < 1e-7 * (1.0 + a.d1.norm()), "d1 at {t}: {:?} {:?}", a.d1, b.d1);
        prop_assert!((a.d2 - b.d2).norm() < 1e-5 * (1.0 + a.d2.norm()), "d2 at {t}");
        if exact_samples {
            prop_assert!((refined.eval(t) - a.p).norm() < 1e-8 * (1.0 + mag(a.p)), "insertion at {t}");
            if t <= at(t_ins) {
                prop_assert!((left.eval(t) - a.p).norm() < 1e-8 * (1.0 + mag(a.p)), "left at {t}");
            }
            if t >= at(t_ins) {
                prop_assert!((right.eval(t) - a.p).norm() < 1e-8 * (1.0 + mag(a.p)), "right at {t}");
            }
        }
    }
    prop_assert!(refined.to_def().unwrap().validate().is_ok());
    Ok(())
}

proptest! {
    #![proptest_config(config(96))]

    /// Repeated interior knots up to multiplicity `degree + 1`, degrees 1..5 and 8..15.
    #[test]
    fn repeated_knots_and_high_degrees_match_the_reference(c in clamped_strategy(), t_ins in 0.02f64..0.98) {
        check_curve(&c.curve, t_ins, true)?;
    }

    /// Plane hits of a continuous curve lie on the plane and are at least the observed sign
    /// changes (a false hit from a wrong Bezier piece fails the first check).
    #[test]
    fn plane_hits_of_repeated_knot_curves_are_on_the_plane(c in clamped_strategy(), nx in -1.0f64..1.0, ny in -1.0f64..1.0, nz in 0.2f64..1.0, off in -1.0f64..1.0) {
        prop_assume!(c.continuous);
        let n = Vector3::new(nx, ny, nz); let n = n * (1.0 / n.norm());
        let plane = Plane { frame: Frame::from_z(Point3::new(off * n.x, off * n.y, off * n.z), n).unwrap() };
        let CurveSurface::Points(hits) = curve_plane(&CurveDef::Nurbs(c.curve.clone()), None, &plane, &ModelTolerance::DEFAULT).unwrap() else { return Ok(()) };
        for h in &hits {
            prop_assert!((c.curve.eval(h.t) - h.point).norm() < 1e-9);
            prop_assert!(n.dot(h.point - plane.frame.origin).abs() < 1e-6, "hit {:?} off the plane", h);
        }
        let dist = |t: f64| n.dot(c.curve.eval(t) - plane.frame.origin);
        let mut changes = 0;
        let mut prev = dist(0.0);
        for i in 1..=3000 {
            let d = dist(i as f64 / 3000.0);
            if (d > 0.0) != (prev > 0.0) {
                changes += 1;
            }
            prev = d;
        }
        prop_assert!(hits.len() >= changes, "{} hits for {} sign changes", hits.len(), changes);
    }

    #[test]
    fn unclamped_nonperiodic_curves_match_the_reference(c in unclamped_strategy(), t_ins in 0.02f64..0.98) {
        check_curve(&c, t_ins, true)?;
    }
}

/// A periodic curve with non-uniform knots, some repeated, whose copies in the extension regions
/// differ from the shifted window by a few ulps (still inside the contract's 1e-12 tolerance).
fn periodic_strategy() -> impl Strategy<Value = (NurbsCurve, Vec<f64>)> {
    (1usize..=4, 0usize..=3).prop_flat_map(|(p, extra)| {
        let m = p + 1 + extra;
        (
            Just((p, m)),
            proptest::collection::vec(prop_oneof![1 => Just(0.0), 3 => 0.1f64..1.0], m),
            0.5f64..4.0,
            -2.0f64..2.0,
            proptest::collection::vec(0u32..4, 2 * p + 1),
            points(m),
            proptest::bool::ANY.prop_flat_map(move |r| weights(m, r)),
        )
    })
    .prop_filter_map("valid periodic knots", |((p, m), mut gaps, period, b0, ulps, pts, w)| {
        // The closing gap must be positive.
        gaps[m - 1] = gaps[m - 1].max(0.1);
        let total: f64 = gaps.iter().sum();
        let mut b = vec![b0];
        for g in &gaps[..m - 1] {
            b.push(b.last().unwrap() + g / total * period);
        }
        let len = m + 2 * p + 1;
        let mut k: Vec<f64> = (0..len).map(|i| b[i % m] + (i / m) as f64 * period).collect();
        // Runs of equal knots must stay within the degree (cyclically, in the extended vector).
        let mut run = 1;
        for w in k.windows(2) {
            run = if w[0] == w[1] { run + 1 } else { 1 };
            if run > p {
                return None;
            }
        }
        // Period copies outside the window [p, p + m) off by a few ulps, keeping order.
        for i in 0..p {
            k[i] = k[i] - ulps[i] as f64 * k[i].abs() * f64::EPSILON;
        }
        for i in (0..p).rev() {
            k[i] = k[i].min(k[i + 1]);
        }
        for i in 0..p {
            let j = p + m + 1 + i;
            k[j] += ulps[p + i] as f64 * k[j].abs() * f64::EPSILON;
        }
        for j in p + m + 1..len {
            k[j] = k[j].max(k[j - 1]);
        }
        let mut control: Vec<Point3> = pts.clone();
        control.extend(pts[..p].iter().copied());
        let weights = w.map(|mut v| {
            let head: Vec<f64> = v[..p].to_vec();
            v.extend(head);
            v
        });
        let c = NurbsCurve::new_periodic(p, k, control, weights).ok()?;
        Some((c, b))
    })
}

proptest! {
    #![proptest_config(config(256))]

    /// Periodic knot insertion keeps the knot vector sorted and the shape, also when inserting
    /// at an existing (repeated) knot.
    #[test]
    fn periodic_insertion_with_nonuniform_repeated_knots((c, window) in periodic_strategy(), s1 in 0.02f64..0.98, s2 in 0.02f64..0.98, pick in 0usize..8) {
        let pc = PreparedCurve::new(&c).unwrap();
        let r = c.range();
        let at = |s: f64| r.lo + (r.hi - r.lo) * s;
        let mut cur = pc.insert_knot(at(s1), 1).unwrap();
        prop_assert!(cur.to_def().unwrap().validate().is_ok(), "knots after the first insertion");
        cur = cur.insert_knot(at(s2), 1).unwrap();
        // An existing knot of the original window (not the seam), when the multiplicity allows.
        let t_old = window[1 + pick % (window.len() - 1)];
        if t_old > r.lo
            && t_old < r.hi
            && let Ok(next) = cur.insert_knot(t_old, 1)
        {
            cur = next;
        }
        prop_assert!(cur.to_def().unwrap().validate().is_ok(), "knots after the insertions");
        for i in 0..=60 {
            let t = r.lo - 0.3 * (r.hi - r.lo) + 1.7 * (r.hi - r.lo) * (i as f64 + 0.31) / 61.0;
            let (a, b) = (c.eval(t), cur.eval(t));
            prop_assert!((a - b).norm() < 1e-9 * (1.0 + mag(a)), "t={t}: {a:?} vs {b:?}");
        }
        // Bezier pieces never have zero length: the curve is hit by a plane at most a few times.
        let mid = c.eval(at(0.5));
        let plane = Plane { frame: Frame::at(mid) };
        prop_assert!(curve_plane(&CurveDef::Nurbs(cur.to_def().unwrap()), None, &plane, &ModelTolerance::DEFAULT).is_ok());
    }
}
