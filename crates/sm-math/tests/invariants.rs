//! Predicate invariants: permutation parity, exact transform behaviour, scale invariance.

use proptest::prelude::*;
use sm_core::{Point3, Transform, Vector3};
use sm_math::{Sign, incircle, insphere, orient2d, orient3d};
use sm_testkit::strategies::config;

/// Lattice coordinates (small range so degeneracies are common) times a power of two.
fn lattice() -> impl Strategy<Value = f64> {
    (-4i32..=4, -3i32..=3).prop_map(|(v, k)| f64::from(v) * 2f64.powi(k))
}

/// Mix of lattice values and arbitrary doubles.
fn coord() -> impl Strategy<Value = f64> {
    prop_oneof![3 => lattice(), 1 => -100.0f64..100.0]
}

fn p3() -> impl Strategy<Value = Point3> {
    // All points share one scale so coplanar/cospherical lattice configurations occur.
    (coord(), coord(), coord()).prop_map(|(x, y, z)| Point3::new(x, y, z))
}

fn p2() -> impl Strategy<Value = [f64; 2]> {
    (coord(), coord()).prop_map(|(x, y)| [x, y])
}

/// All permutations of `0..n` with their parity (+1 even, -1 odd).
fn perms(n: usize) -> Vec<(Vec<usize>, i32)> {
    fn rec(cur: &mut Vec<usize>, used: &mut Vec<bool>, n: usize, out: &mut Vec<Vec<usize>>) {
        if cur.len() == n {
            out.push(cur.clone());
            return;
        }
        for i in 0..n {
            if !used[i] {
                used[i] = true;
                cur.push(i);
                rec(cur, used, n, out);
                cur.pop();
                used[i] = false;
            }
        }
    }
    let mut all = vec![];
    rec(&mut vec![], &mut vec![false; n], n, &mut all);
    all.into_iter()
        .map(|p| {
            let inv = (0..n)
                .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
                .filter(|&(i, j)| p[i] > p[j])
                .count();
            (p, if inv % 2 == 0 { 1 } else { -1 })
        })
        .collect()
}

fn signed(s: Sign, parity: i32) -> Sign {
    if parity < 0 { -s } else { s }
}

proptest! {
    #![proptest_config(config(400))]

    #[test]
    fn orient2d_permutation_parity(p in prop::array::uniform3(p2())) {
        let base = orient2d(p[0], p[1], p[2]);
        for (q, par) in perms(3) {
            prop_assert_eq!(orient2d(p[q[0]], p[q[1]], p[q[2]]), signed(base, par));
        }
    }

    #[test]
    fn orient3d_permutation_parity(p in prop::array::uniform4(p3())) {
        let base = orient3d(p[0], p[1], p[2], p[3]);
        for (q, par) in perms(4) {
            prop_assert_eq!(orient3d(p[q[0]], p[q[1]], p[q[2]], p[q[3]]), signed(base, par));
        }
    }

    #[test]
    fn incircle_permutation_parity(p in prop::array::uniform4(p2())) {
        let base = incircle(p[0], p[1], p[2], p[3]);
        for (q, par) in perms(4) {
            prop_assert_eq!(incircle(p[q[0]], p[q[1]], p[q[2]], p[q[3]]), signed(base, par));
        }
    }

    #[test]
    fn insphere_permutation_parity(p in prop::array::uniform5(p3())) {
        let base = insphere(p[0], p[1], p[2], p[3], p[4]);
        for (q, par) in perms(5) {
            prop_assert_eq!(insphere(p[q[0]], p[q[1]], p[q[2]], p[q[3]], p[q[4]]), signed(base, par));
        }
    }

    /// Duplicated arguments give exactly zero.
    #[test]
    fn repeated_points_are_degenerate(a in p3(), b in p3(), c in p3()) {
        prop_assert_eq!(orient3d(a, a, b, c), Sign::Zero);
        prop_assert_eq!(orient3d(a, b, c, c), Sign::Zero);
        prop_assert_eq!(insphere(a, b, c, a, b), Sign::Zero);
        prop_assert_eq!(incircle([a.x, a.y], [b.x, b.y], [c.x, c.y], [b.x, b.y]), Sign::Zero);
        prop_assert_eq!(orient2d([a.x, a.y], [b.x, b.y], [a.x, a.y]), Sign::Zero);
    }

    /// Power-of-two scaling and integer translation of lattice data are exact, so the signs
    /// must not change; a mirror (improper transform) flips orientation-sensitive signs.
    #[test]
    fn exact_transforms(p in prop::array::uniform5(p3()), k in -20i32..20) {
        let s = 2f64.powi(k);
        let scale = Transform { m: [[s, 0.0, 0.0], [0.0, s, 0.0], [0.0, 0.0, s]], t: Vector3::new(0.0, 0.0, 0.0) };
        let mirror = Transform { m: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) };
        // 90 degree rotation about z: entries 0/+-1, exact, proper.
        let rot = Transform { m: [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) };
        prop_assert!(mirror.is_improper() && !rot.is_improper() && !scale.is_improper());
        let o = orient3d(p[0], p[1], p[2], p[3]);
        let i = insphere(p[0], p[1], p[2], p[3], p[4]);
        let map = |t: &Transform| -> Vec<Point3> { p.iter().map(|&q| t.apply_point(q)).collect() };
        for t in [&scale, &rot] {
            let q = map(t);
            prop_assert_eq!(orient3d(q[0], q[1], q[2], q[3]), o);
            prop_assert_eq!(insphere(q[0], q[1], q[2], q[3], q[4]), i);
        }
        let q = map(&mirror);
        prop_assert_eq!(orient3d(q[0], q[1], q[2], q[3]), -o);
        prop_assert_eq!(insphere(q[0], q[1], q[2], q[3], q[4]), -i);
        // Integer translation of lattice points (all values are multiples of 1/8 and < 2^10).
        let shift = Transform::translation(Vector3::new(64.0, -32.0, 16.0));
        if p.iter().all(|q| [q.x, q.y, q.z].iter().all(|v| (v * 8.0).fract() == 0.0)) {
            let q = map(&shift);
            prop_assert_eq!(orient3d(q[0], q[1], q[2], q[3]), o);
            prop_assert_eq!(insphere(q[0], q[1], q[2], q[3], q[4]), i);
        }
    }

    /// Cocircular points built from Pythagorean triples are exactly on the circle.
    #[test]
    fn pythagorean_cocircular(cx in -50i32..50, cy in -50i32..50, idx in prop::array::uniform4(0usize..12), k in 1i32..8) {
        let r = [(3, 4), (4, 3), (5, 0), (4, -3), (3, -4), (0, -5), (-3, -4), (-4, -3), (-5, 0), (-4, 3), (-3, 4), (0, 5)];
        let pt = |i: usize| [f64::from(cx + k * r[i].0), f64::from(cy + k * r[i].1)];
        let (a, b, c, d) = (pt(idx[0]), pt(idx[1]), pt(idx[2]), pt(idx[3]));
        prop_assert_eq!(incircle(a, b, c, d), Sign::Zero);
    }
}
