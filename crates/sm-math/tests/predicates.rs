//! Predicates against an exact `i128` oracle: degenerate grids, nudged degenerate inputs,
//! Shewchuk's near-degenerate figure, and full-mantissa orient2d.

mod common;
use common::*;
use sm_math::{Sign, incircle, insphere, orient2d, orient3d};

#[test]
fn orient2d_all_grid_triples() {
    let pts: Vec<[i64; 2]> = (0..6).flat_map(|x| (0..6).map(move |y| [x, y])).collect();
    let mut zeros = 0;
    for (n, &a) in pts.iter().enumerate() {
        for &b in &pts {
            for &c in &pts {
                let k = (n % 7) as i32 - 3;
                let want = orient2d_i(a, b, c);
                assert_eq!(
                    orient2d(f2(a, k), f2(b, k), f2(c, k)),
                    want,
                    "{a:?} {b:?} {c:?}"
                );
                zeros += usize::from(want == Sign::Zero);
            }
        }
    }
    assert!(zeros > 1000, "grid must contain many exact degeneracies");
}

#[test]
fn orient3d_all_grid_quadruples() {
    let pts: Vec<[i64; 3]> = (0..3)
        .flat_map(|x| (0..3).flat_map(move |y| (0..3).map(move |z| [x, y, z])))
        .collect();
    let mut zeros = 0;
    for (n, &a) in pts.iter().enumerate() {
        for &b in &pts {
            for &c in &pts {
                for &d in &pts {
                    let k = (n % 5) as i32 - 2;
                    let want = orient3d_i(a, b, c, d);
                    assert_eq!(orient3d(f3(a, k), f3(b, k), f3(c, k), f3(d, k)), want);
                    zeros += usize::from(want == Sign::Zero);
                }
            }
        }
    }
    assert!(zeros > 10_000);
}

#[test]
fn incircle_all_grid_quadruples() {
    let pts: Vec<[i64; 2]> = (0..5).flat_map(|x| (0..5).map(move |y| [x, y])).collect();
    let mut zeros = 0;
    for (n, &a) in pts.iter().enumerate() {
        for &b in &pts {
            for &c in &pts {
                for &d in &pts {
                    let k = (n % 5) as i32 - 2;
                    let want = incircle_i(a, b, c, d);
                    assert_eq!(
                        incircle(f2(a, k), f2(b, k), f2(c, k), f2(d, k)),
                        want,
                        "{a:?} {b:?} {c:?} {d:?}"
                    );
                    zeros += usize::from(want == Sign::Zero);
                }
            }
        }
    }
    assert!(zeros > 10_000);
}

#[test]
fn incircle_cocircular_lattice_points() {
    // All 12 lattice points on x^2 + y^2 = 25 are cocircular: every quadruple is exactly zero.
    let circle: Vec<[i64; 2]> = [
        (3, 4),
        (4, 3),
        (5, 0),
        (4, -3),
        (3, -4),
        (0, -5),
        (-3, -4),
        (-4, -3),
        (-5, 0),
        (-4, 3),
        (-3, 4),
        (0, 5),
    ]
    .iter()
    .map(|&(x, y)| [x, y])
    .collect();
    let mut seen_zero = 0;
    for &a in &circle {
        for &b in &circle {
            for &c in &circle {
                for &d in &circle {
                    let got = incircle(f2(a, -4), f2(b, -4), f2(c, -4), f2(d, -4));
                    assert_eq!(got, incircle_i(a, b, c, d));
                    if a != b && b != c && a != c && orient2d_i(a, b, c) != Sign::Zero {
                        assert_eq!(got, Sign::Zero, "cocircular points must be exactly zero");
                        seen_zero += 1;
                    }
                }
            }
        }
    }
    assert!(seen_zero > 10_000);
}

#[test]
fn insphere_cospherical_and_grid_samples() {
    // Points of the 3x3x3 grid, random 5-tuples: lots of exactly cospherical ones.
    let pts: Vec<[i64; 3]> = (0..3)
        .flat_map(|x| (0..3).flat_map(move |y| (0..3).map(move |z| [x, y, z])))
        .collect();
    let mut rng = Rng(7);
    let mut zeros = 0;
    for n in 0..200_000 {
        let p: Vec<[i64; 3]> = (0..5).map(|_| pts[rng.below(pts.len())]).collect();
        let k = n % 7 - 3;
        let want = insphere_i(p[0], p[1], p[2], p[3], p[4]);
        let got = insphere(
            f3(p[0], k),
            f3(p[1], k),
            f3(p[2], k),
            f3(p[3], k),
            f3(p[4], k),
        );
        assert_eq!(got, want, "{p:?}");
        zeros += usize::from(want == Sign::Zero);
    }
    assert!(zeros > 20_000);
    // Lattice points on x^2+y^2+z^2 = 9 (all 30 of them) are cospherical.
    let mut sph = vec![];
    for x in -3i64..=3 {
        for y in -3i64..=3 {
            for z in -3i64..=3 {
                if x * x + y * y + z * z == 9 {
                    sph.push([x, y, z]);
                }
            }
        }
    }
    assert_eq!(sph.len(), 30);
    for _ in 0..20_000 {
        let p: Vec<[i64; 3]> = (0..5).map(|_| sph[rng.below(sph.len())]).collect();
        let got = insphere(
            f3(p[0], 0),
            f3(p[1], 0),
            f3(p[2], 0),
            f3(p[3], 0),
            f3(p[4], 0),
        );
        assert_eq!(got, Sign::Zero);
    }
}

/// Degenerate lattice configurations nudged by one unit in one coordinate.
#[test]
fn nudged_degenerate_inputs() {
    let mut rng = Rng(11);
    let big = 1 << 18;
    for n in 0..60_000 {
        let k = (rng.int(40)) as i32 - 20;
        // orient2d: c on the line ab (c = a + t (b - a), t integer), then nudge.
        let a = [rng.int(big), rng.int(big)];
        let dlt = [rng.int(1 << 8), rng.int(1 << 8)];
        let t = rng.int(100);
        let b = [a[0] + dlt[0], a[1] + dlt[1]];
        let mut c = [a[0] + t * dlt[0], a[1] + t * dlt[1]];
        if n % 2 == 1 {
            c[(n / 2) % 2] += if n % 4 == 1 { 1 } else { -1 };
        }
        assert_eq!(orient2d(f2(a, k), f2(b, k), f2(c, k)), orient2d_i(a, b, c));

        // orient3d: d in the plane abc, then nudge.
        let a3 = [rng.int(big), rng.int(big), rng.int(big)];
        let u = [rng.int(1 << 7), rng.int(1 << 7), rng.int(1 << 7)];
        let v = [rng.int(1 << 7), rng.int(1 << 7), rng.int(1 << 7)];
        let (s, t) = (rng.int(50), rng.int(50));
        let add = |p: [i64; 3], q: [i64; 3]| [p[0] + q[0], p[1] + q[1], p[2] + q[2]];
        let b3 = add(a3, u);
        let c3 = add(a3, v);
        let mut d3 = [
            a3[0] + s * u[0] + t * v[0],
            a3[1] + s * u[1] + t * v[1],
            a3[2] + s * u[2] + t * v[2],
        ];
        if n % 2 == 1 {
            d3[(n / 2) % 3] += if n % 4 == 1 { 1 } else { -1 };
        }
        assert_eq!(
            orient3d(f3(a3, k), f3(b3, k), f3(c3, k), f3(d3, k)),
            orient3d_i(a3, b3, c3, d3)
        );

        // incircle: four lattice points on a circle (Pythagorean quadruples), nudged.
        let r = [
            (3, 4),
            (4, 3),
            (5, 0),
            (0, 5),
            (-3, 4),
            (-4, -3),
            (3, -4),
            (0, -5),
        ];
        let sc = rng.int(2000) + 2001;
        let ctr = [rng.int(big), rng.int(big)];
        let p: Vec<[i64; 2]> = (0..4)
            .map(|_| {
                let (x, y) = r[rng.below(r.len())];
                [ctr[0] + sc * x, ctr[1] + sc * y]
            })
            .collect();
        let mut q = p.clone();
        if n % 3 != 0 {
            q[3][n % 2] += if n % 5 < 3 { 1 } else { -1 };
        }
        assert_eq!(
            incircle(f2(q[0], k), f2(q[1], k), f2(q[2], k), f2(q[3], k)),
            incircle_i(q[0], q[1], q[2], q[3])
        );

        // insphere: points on a sphere of radius 3 about a center, nudged.
        let on_sphere: Vec<[i64; 3]> = [
            (3, 0, 0),
            (0, 3, 0),
            (0, 0, 3),
            (-3, 0, 0),
            (0, -3, 0),
            (0, 0, -3),
            (1, 2, 2),
            (2, 1, -2),
            (-2, 2, 1),
            (2, -2, -1),
        ]
        .iter()
        .map(|&(x, y, z)| [x, y, z])
        .collect();
        let c0 = [rng.int(big), rng.int(big), rng.int(big)];
        let scale = rng.int(300) + 301;
        let mut sp: Vec<[i64; 3]> = (0..5)
            .map(|_| {
                let o = on_sphere[rng.below(on_sphere.len())];
                [
                    c0[0] + scale * o[0],
                    c0[1] + scale * o[1],
                    c0[2] + scale * o[2],
                ]
            })
            .collect();
        if n % 3 != 0 {
            sp[4][n % 3] += if n % 5 < 3 { 1 } else { -1 };
        }
        assert_eq!(
            insphere(
                f3(sp[0], k),
                f3(sp[1], k),
                f3(sp[2], k),
                f3(sp[3], k),
                f3(sp[4], k)
            ),
            insphere_i(sp[0], sp[1], sp[2], sp[3], sp[4])
        );
    }
}

/// Shewchuk's Figure: orient2d of `(0.5 + i u, 0.5 + j u)` against the line `(12,12)-(24,24)`,
/// `u = 2^-53`. Plain floating point gets many cells wrong; every cell must match the oracle.
#[test]
fn shewchuk_near_degenerate_grid() {
    let u = 2f64.powi(-53);
    let n = 128;
    let (b, c) = ([12.0, 12.0], [24.0, 24.0]);
    // Exact oracle: the point is (0.5 + i u, 0.5 + j u); with 2^53 scaling all quantities are
    // integers: p = 2^53*0.5 + i, line direction (12, 12) -> sign of (j - i) up to the line's
    // own offset, so the exact sign is sign((c-b) x (p-b)) = 12 (py - px) = 12 (j - i) u.
    let mut float_wrong = 0;
    for i in 0..n {
        for j in 0..n {
            let p = [0.5 + f64::from(i) * u, 0.5 + f64::from(j) * u];
            let want = Sign::of(f64::from(i32::from(j > i) - i32::from(i > j)));
            // orient2d(b, c, p) > 0 iff p is left of b->c, i.e. p.y > p.x.
            assert_eq!(orient2d(b, c, p), want, "i={i} j={j}");
            let naive = (b[0] - p[0]) * (c[1] - p[1]) - (b[1] - p[1]) * (c[0] - p[0]);
            if Sign::of(naive) != want {
                float_wrong += 1;
            }
        }
    }
    assert!(
        float_wrong > 0,
        "the grid is supposed to defeat naive evaluation"
    );
}

/// Full-mantissa orient2d: same binade, so everything is an exact integer in i128.
#[test]
fn orient2d_full_mantissa_near_collinear() {
    let mut rng = Rng(99);
    for n in 0..100_000 {
        let e = rng.int(60) as i32;
        let s = 2f64.powi(e - 52);
        let m = |rng: &mut Rng| (1i64 << 52) + (rng.next() % (1u64 << 52)) as i64;
        let a = [m(&mut rng), m(&mut rng)];
        let b = [m(&mut rng), m(&mut rng)];
        // c near the segment ab: midpoint-ish with a tiny perturbation in the last bits.
        let t = rng.int(3);
        let mut c = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
        if t != 0 && n % 2 == 0 {
            c[0] += t;
        }
        let want = {
            let (ax, ay, bx, by, cx, cy) = (
                a[0] as i128,
                a[1] as i128,
                b[0] as i128,
                b[1] as i128,
                c[0] as i128,
                c[1] as i128,
            );
            sgn((ax - cx) * (by - cy) - (ay - cy) * (bx - cx))
        };
        let f = |p: [i64; 2]| [p[0] as f64 * s, p[1] as f64 * s];
        assert_eq!(orient2d(f(a), f(b), f(c)), want);
    }
}

#[test]
fn conventions_match_shewchuk() {
    use sm_math::Point3;
    // ccw triangle -> positive.
    assert_eq!(orient2d([0.0, 0.0], [1.0, 0.0], [0.0, 1.0]), Sign::Positive);
    // d below the plane of a ccw (seen from above) triangle -> positive.
    let (a, b, c) = (
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    assert_eq!(
        orient3d(a, b, c, Point3::new(0.0, 0.0, -1.0)),
        Sign::Positive
    );
    assert_eq!(
        orient3d(a, b, c, Point3::new(0.0, 0.0, 1.0)),
        Sign::Negative
    );
    // d inside circle of ccw abc -> positive.
    assert_eq!(
        incircle([0.0, 0.0], [2.0, 0.0], [0.0, 2.0], [0.5, 0.5]),
        Sign::Positive
    );
    assert_eq!(
        incircle([0.0, 0.0], [2.0, 0.0], [0.0, 2.0], [3.0, 3.0]),
        Sign::Negative
    );
    // positive tetra, e at centroid -> positive.
    let d = Point3::new(0.0, 0.0, -1.0);
    assert_eq!(orient3d(a, b, c, d), Sign::Positive);
    assert_eq!(
        insphere(a, b, c, d, Point3::new(0.25, 0.25, -0.25)),
        Sign::Positive
    );
    assert_eq!(
        insphere(a, b, c, d, Point3::new(5.0, 5.0, 5.0)),
        Sign::Negative
    );
}

#[test]
fn try_variants_reject_out_of_range() {
    use sm_math::{PredicateError, try_orient2d};
    assert_eq!(
        try_orient2d([f64::NAN, 0.0], [1.0, 0.0], [0.0, 1.0]),
        Err(PredicateError::OutOfRange)
    );
    assert_eq!(
        try_orient2d([1e-200, 0.0], [1.0, 0.0], [0.0, 1.0]),
        Err(PredicateError::OutOfRange)
    );
    assert_eq!(
        try_orient2d([1e40, 0.0], [1.0, 0.0], [0.0, 1.0]),
        Err(PredicateError::OutOfRange)
    );
    assert_eq!(
        try_orient2d([0.0, 0.0], [1.0, 0.0], [0.0, 1.0]),
        Ok(Sign::Positive)
    );
}

/// Near-degenerate random inputs (stage-B territory) against expansion-arithmetic oracles built
/// from `det_exact`, independent of the predicate code paths.
#[test]
fn near_degenerate_matches_expansion_oracle() {
    use sm_core::Point3;
    use sm_math::exact::{Expansion, det_exact};
    let mut rng = Rng(0xB0B);
    let u = |rng: &mut Rng| 1.0 + (rng.next() >> 11) as f64 / (1u64 << 53) as f64;
    let tiny = |rng: &mut Rng| rng.int(3) as f64 * f64::EPSILON;
    let diff = |a: f64, b: f64| Expansion::from_diff(a, b);
    let mut isp_mul = None;
    for _ in 0..20000 {
        // orient3d: near-coplanar.
        let mk = |rng: &mut Rng| {
            let (x, y) = (u(rng), u(rng));
            Point3::new(x, y, 0.3 * x + 0.7 * y + tiny(rng))
        };
        let p: Vec<Point3> = (0..5).map(|_| mk(&mut rng)).collect();
        let m: Vec<Vec<Expansion>> = (0..3)
            .map(|i| {
                vec![
                    diff(p[i].x, p[3].x),
                    diff(p[i].y, p[3].y),
                    diff(p[i].z, p[3].z),
                ]
            })
            .collect();
        assert_eq!(orient3d(p[0], p[1], p[2], p[3]), det_exact(&m).sign());
        // insphere: the global sign convention is fixed but unknown here, so require that the
        // relation between predicate and oracle is the same for every nonzero case.
        let lift = |q: Point3| {
            let (x, y, z) = (diff(q.x, p[4].x), diff(q.y, p[4].y), diff(q.z, p[4].z));
            let w = x.square().add(&y.square()).add(&z.square());
            vec![x, y, z, w]
        };
        let m4: Vec<Vec<Expansion>> = (0..4).map(|i| lift(p[i])).collect();
        let want = det_exact(&m4).sign();
        let got = insphere(p[0], p[1], p[2], p[3], p[4]);
        if want.is_zero() || got.is_zero() {
            assert_eq!(want, got);
        } else {
            let same = got == want;
            assert_eq!(*isp_mul.get_or_insert(same), same);
        }
        // orient2d / incircle: near-collinear points.
        let q: Vec<[f64; 2]> = (0..4)
            .map(|_| {
                let t = u(&mut rng);
                [t, 0.25 * t + 0.5 + tiny(&mut rng)]
            })
            .collect();
        let m2: Vec<Vec<Expansion>> = (0..2)
            .map(|i| vec![diff(q[i][0], q[2][0]), diff(q[i][1], q[2][1])])
            .collect();
        assert_eq!(orient2d(q[0], q[1], q[2]), det_exact(&m2).sign());
        let ml: Vec<Vec<Expansion>> = (0..3)
            .map(|i| {
                let (x, y) = (diff(q[i][0], q[3][0]), diff(q[i][1], q[3][1]));
                let w = x.square().add(&y.square());
                vec![x, y, w]
            })
            .collect();
        assert_eq!(incircle(q[0], q[1], q[2], q[3]), det_exact(&ml).sign());
    }
}

/// Range edge: lattice data scaled to the extremes of the documented `[2^-100, 2^100]`
/// exactness range, through the checked `try_*` entry points, with one-unit nudges.
#[test]
fn range_edge_scaled_lattices() {
    use sm_math::{try_incircle, try_insphere, try_orient2d, try_orient3d};
    let ks: Vec<i32> = (-100..=-96).chain(92..=96).collect();
    let mut rng = Rng(0xED6E);
    let circ = [
        (3, 4),
        (4, 3),
        (5, 0),
        (0, 5),
        (-3, 4),
        (-4, -3),
        (3, -4),
        (0, -5),
    ];
    let sph = [
        (3, 0, 0),
        (0, 3, 0),
        (0, 0, 3),
        (-3, 0, 0),
        (0, -3, 0),
        (0, 0, -3),
        (1, 2, 2),
        (2, 1, -2),
        (-2, 2, 1),
        (2, -2, -1),
    ];
    let mut zeros = [0usize; 4];
    for n in 0..4000 {
        let k = ks[n % ks.len()];
        let nudge = if n % 2 == 1 {
            if n % 4 == 1 { 1 } else { -1 }
        } else {
            0
        };
        // Coordinates stay within +-15 so that 15 * 2^96 < 2^100 and a nudge to 16 = 2^4 is
        // still in range; at k = -100 every nonzero coordinate is >= 2^-100.

        // orient2d: collinear c = a + t d.
        let a = [rng.int(4), rng.int(4)];
        let d = [rng.int(2), rng.int(2)];
        let t = rng.int(3);
        let b = [a[0] + d[0], a[1] + d[1]];
        let mut c = [a[0] + t * d[0], a[1] + t * d[1]];
        c[(n / 2) % 2] += nudge;
        let want = orient2d_i(a, b, c);
        zeros[0] += usize::from(want == Sign::Zero);
        assert_eq!(
            try_orient2d(f2(a, k), f2(b, k), f2(c, k)),
            Ok(want),
            "k={k}"
        );

        // orient3d: coplanar d = a + s u + t v.
        let a3 = [rng.int(3), rng.int(3), rng.int(3)];
        let u = [rng.int(1), rng.int(1), rng.int(1)];
        let v = [rng.int(1), rng.int(1), rng.int(1)];
        let (s, t) = (rng.int(2), rng.int(2));
        let add = |p: [i64; 3], q: [i64; 3]| [p[0] + q[0], p[1] + q[1], p[2] + q[2]];
        let (b3, c3) = (add(a3, u), add(a3, v));
        let mut d3 = [
            a3[0] + s * u[0] + t * v[0],
            a3[1] + s * u[1] + t * v[1],
            a3[2] + s * u[2] + t * v[2],
        ];
        d3[(n / 2) % 3] += nudge;
        let want = orient3d_i(a3, b3, c3, d3);
        zeros[1] += usize::from(want == Sign::Zero);
        assert_eq!(
            try_orient3d(f3(a3, k), f3(b3, k), f3(c3, k), f3(d3, k)),
            Ok(want),
            "k={k}"
        );

        // incircle: four lattice points on a radius-5 circle.
        let ctr = [rng.int(4), rng.int(4)];
        let mut q: Vec<[i64; 2]> = (0..4)
            .map(|_| {
                let (x, y) = circ[rng.below(circ.len())];
                [ctr[0] + x, ctr[1] + y]
            })
            .collect();
        q[3][n % 2] += nudge;
        let want = incircle_i(q[0], q[1], q[2], q[3]);
        zeros[2] += usize::from(want == Sign::Zero);
        assert_eq!(
            try_incircle(f2(q[0], k), f2(q[1], k), f2(q[2], k), f2(q[3], k)),
            Ok(want),
            "k={k}"
        );

        // insphere: five lattice points on a radius-3 sphere.
        let c0 = [rng.int(4), rng.int(4), rng.int(4)];
        let mut sp: Vec<[i64; 3]> = (0..5)
            .map(|_| {
                let o = sph[rng.below(sph.len())];
                [c0[0] + o.0, c0[1] + o.1, c0[2] + o.2]
            })
            .collect();
        sp[4][n % 3] += nudge;
        let want = insphere_i(sp[0], sp[1], sp[2], sp[3], sp[4]);
        zeros[3] += usize::from(want == Sign::Zero);
        assert_eq!(
            try_insphere(
                f3(sp[0], k),
                f3(sp[1], k),
                f3(sp[2], k),
                f3(sp[3], k),
                f3(sp[4], k)
            ),
            Ok(want),
            "k={k}"
        );
    }
    assert!(
        zeros.iter().all(|&z| z > 100),
        "need exact degeneracies: {zeros:?}"
    );
}

/// Range edge with ulp nudges: degenerate lattice data, one coordinate moved by +-1 or +-2 ulp,
/// scaled by `2^k`. Scaling by a power of two is exact, so the oracle is the same nudged data
/// through `try_*` at `k = 0` (the i128 oracle cannot hold degree-5 terms at ulp resolution).
/// Cases with a scaled coordinate outside `[2^-100, 2^100]` are skipped (rejected by `try_*`).
#[test]
fn range_edge_ulp_nudged_lattices() {
    use sm_math::{try_incircle, try_insphere, try_orient2d, try_orient3d};
    let ks: Vec<i32> = (-100..=-96).chain(92..=96).collect();
    let deltas: [i64; 4] = [-2, -1, 1, 2];
    let circ = [
        (3, 4),
        (4, 3),
        (5, 0),
        (0, 5),
        (-3, 4),
        (-4, -3),
        (3, -4),
        (0, -5),
    ];
    let sph = [
        (3, 0, 0),
        (0, 3, 0),
        (0, 0, 3),
        (-3, 0, 0),
        (0, -3, 0),
        (0, 0, -3),
        (1, 2, 2),
        (2, 1, -2),
        (-2, 2, 1),
        (2, -2, -1),
    ];
    let mut rng = Rng(0xA1B2);
    let (mut tested, mut skipped, mut flips) = (0usize, 0usize, 0usize);
    for n in 0..6000 {
        let k = ks[n % ks.len()];
        let delta = deltas[(n / ks.len()) % 4];
        let dim3 = n % 2 == 1;
        let kind = (n / 2) % 2; // 0: orient, 1: incircle/insphere
        let npts = match (dim3, kind) {
            (false, 0) => 3,
            (false, _) => 4,
            (true, 0) => 4,
            (true, _) => 5,
        };
        // Degenerate lattice points as flat coordinate rows (dimension 2 or 3).
        let d = if dim3 { 3 } else { 2 };
        let mut pts: Vec<Vec<f64>> = Vec::new();
        if kind == 0 {
            let a: Vec<i64> = (0..d).map(|_| rng.int(3)).collect();
            let us: Vec<Vec<i64>> = (0..d - 1)
                .map(|_| (0..d).map(|_| rng.int(1)).collect())
                .collect();
            pts.push(a.iter().map(|&v| v as f64).collect());
            for u in &us {
                pts.push((0..d).map(|i| (a[i] + u[i]) as f64).collect());
            }
            let co: Vec<i64> = (0..d - 1).map(|_| rng.int(2)).collect();
            pts.push(
                (0..d)
                    .map(|i| (a[i] + (0..d - 1).map(|j| co[j] * us[j][i]).sum::<i64>()) as f64)
                    .collect(),
            );
        } else {
            let c: Vec<i64> = (0..d).map(|_| rng.int(4)).collect();
            for _ in 0..npts {
                if dim3 {
                    let o = sph[rng.below(sph.len())];
                    pts.push(vec![
                        (c[0] + o.0) as f64,
                        (c[1] + o.1) as f64,
                        (c[2] + o.2) as f64,
                    ]);
                } else {
                    let o = circ[rng.below(circ.len())];
                    pts.push(vec![(c[0] + o.0) as f64, (c[1] + o.1) as f64]);
                }
            }
        }
        // Nudge a nonzero coordinate of the last point by `delta` ulps.
        let last = npts - 1;
        let Some(ci) = (0..d).map(|i| (i + n) % d).find(|&i| pts[last][i] != 0.0) else {
            skipped += 1;
            continue;
        };
        let bits =
            pts[last][ci].to_bits() as i64 + if pts[last][ci] > 0.0 { delta } else { -delta };
        pts[last][ci] = f64::from_bits(bits as u64);

        let s = 2f64.powi(k);
        let in_range = pts.iter().flatten().all(|&v| {
            v == 0.0 || ((v * s).abs() >= 2f64.powi(-100) && (v * s).abs() <= 2f64.powi(100))
        });
        if !in_range {
            skipped += 1;
            continue;
        }
        let sc = |p: &Vec<f64>, s: f64| -> (Vec<f64>, sm_math::Point3) {
            let v: Vec<f64> = p.iter().map(|x| x * s).collect();
            let p3 = sm_math::Point3::new(v[0], v[1], *v.get(2).unwrap_or(&0.0));
            (v, p3)
        };
        let run = |s: f64| {
            let q: Vec<_> = pts.iter().map(|p| sc(p, s)).collect();
            let a2 = |i: usize| [q[i].0[0], q[i].0[1]];
            let a3 = |i: usize| q[i].1;
            match (dim3, kind) {
                (false, 0) => try_orient2d(a2(0), a2(1), a2(2)),
                (false, _) => try_incircle(a2(0), a2(1), a2(2), a2(3)),
                (true, 0) => try_orient3d(a3(0), a3(1), a3(2), a3(3)),
                (true, _) => try_insphere(a3(0), a3(1), a3(2), a3(3), a3(4)),
            }
        };
        let want = run(1.0);
        assert!(want.is_ok(), "unscaled reference must be in range");
        assert_eq!(run(s), want, "k={k} delta={delta} n={n}");
        tested += 1;
        flips += usize::from(want != Ok(Sign::Zero));
    }
    assert!(
        tested > 2000 && flips > 500,
        "coverage: tested={tested} skipped={skipped} nonzero={flips}"
    );
}

#[test]
fn range_edge_just_outside_is_rejected() {
    use sm_math::{PredicateError, try_orient2d};
    for x in [2f64.powi(101), 2f64.powi(-101)] {
        assert_eq!(
            try_orient2d([0.0, 0.0], [x, 0.0], [0.0, 1.0]),
            Err(PredicateError::OutOfRange)
        );
    }
}
