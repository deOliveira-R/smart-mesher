//! Simulation of simplicity: agreement with an independent symbolic oracle, invariants, and
//! global consistency (SoS-Delaunay triangulations tile their hull).

mod common;
use common::*;
use sm_math::sos::{Tagged, incircle_sos, insphere_sos, orient2d_sos, orient3d_sos};
use sm_math::{Point3, Sign};
use std::collections::{BTreeMap, BTreeSet};

type Poly = BTreeMap<u64, i128>;

fn poly_mul(a: &Poly, b: &Poly) -> Poly {
    let mut out = Poly::new();
    for (&ma, &ca) in a {
        for (&mb, &cb) in b {
            assert_eq!(ma & mb, 0, "monomials must use distinct variables");
            *out.entry(ma | mb).or_insert(0) += ca * cb;
        }
    }
    out
}

fn poly_add(a: &mut Poly, b: &Poly, neg: bool) {
    for (&m, &c) in b {
        *a.entry(m).or_insert(0) += if neg { -c } else { c };
    }
}

/// Oracle: expand `det [base + eps | 1]` with variable `(id, col)` of weight `2^(id*cols+col)`,
/// then take the sign of the lowest-order nonzero coefficient. Uses global ids directly.
fn oracle(rows: &[(Vec<i128>, u64)]) -> Sign {
    let n = rows.len();
    let cols = n - 1;
    let entry = |r: usize, c: usize| -> Poly {
        let mut p = Poly::new();
        if c == cols {
            p.insert(0, 1);
        } else {
            let (ref base, id) = rows[r];
            if base[c] != 0 {
                p.insert(0, base[c]);
            }
            p.insert(1u64 << (id as usize * cols + c), 1);
        }
        p
    };
    let mut total = Poly::new();
    let mut perm: Vec<usize> = (0..n).collect();
    permute(&mut perm, 0, &mut |p| {
        let inv = (0..n)
            .flat_map(|i| (i + 1..n).map(move |j| (i, j)))
            .filter(|&(i, j)| p[i] > p[j])
            .count();
        let mut prod: Poly = [(0u64, 1i128)].into_iter().collect();
        for (r, &c) in p.iter().enumerate() {
            prod = poly_mul(&prod, &entry(r, c));
        }
        poly_add(&mut total, &prod, inv % 2 == 1);
    });
    for (_, c) in total {
        if c != 0 {
            return sgn(c);
        }
    }
    Sign::Zero
}

fn permute(v: &mut Vec<usize>, k: usize, f: &mut impl FnMut(&[usize])) {
    if k == v.len() {
        f(v);
        return;
    }
    for i in k..v.len() {
        v.swap(k, i);
        permute(v, k + 1, f);
        v.swap(k, i);
    }
}

fn t2(p: [i64; 2], id: u64) -> Tagged<[f64; 2]> {
    Tagged::new([p[0] as f64, p[1] as f64], id)
}
fn t3(p: [i64; 3], id: u64) -> Tagged<Point3> {
    Tagged::new(Point3::new(p[0] as f64, p[1] as f64, p[2] as f64), id)
}

fn shuffled_ids(rng: &mut Rng, n: usize) -> Vec<u64> {
    let mut ids: Vec<u64> = (0..n as u64).collect();
    for i in (1..n).rev() {
        ids.swap(i, rng.below(i + 1));
    }
    ids
}

fn row2(p: [i64; 2], id: u64) -> (Vec<i128>, u64) {
    (vec![i128::from(p[0]), i128::from(p[1])], id)
}
fn row2l(p: [i64; 2], id: u64) -> (Vec<i128>, u64) {
    (
        vec![
            i128::from(p[0]),
            i128::from(p[1]),
            i128::from(p[0] * p[0] + p[1] * p[1]),
        ],
        id,
    )
}
fn row3(p: [i64; 3], id: u64) -> (Vec<i128>, u64) {
    (
        vec![i128::from(p[0]), i128::from(p[1]), i128::from(p[2])],
        id,
    )
}
fn row3l(p: [i64; 3], id: u64) -> (Vec<i128>, u64) {
    (
        vec![
            i128::from(p[0]),
            i128::from(p[1]),
            i128::from(p[2]),
            i128::from(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]),
        ],
        id,
    )
}

#[test]
fn orient2d_sos_matches_oracle() {
    let pts: Vec<[i64; 2]> = (0..4).flat_map(|x| (0..4).map(move |y| [x, y])).collect();
    let mut rng = Rng(1);
    for _ in 0..4000 {
        let ids = shuffled_ids(&mut rng, 8);
        let p: Vec<[i64; 2]> = (0..3).map(|_| pts[rng.below(pts.len())]).collect();
        let got = orient2d_sos(t2(p[0], ids[0]), t2(p[1], ids[1]), t2(p[2], ids[2])).unwrap();
        let want = oracle(&[row2(p[0], ids[0]), row2(p[1], ids[1]), row2(p[2], ids[2])]);
        assert_eq!(got, want, "{p:?} {ids:?}");
        assert_ne!(got, Sign::Zero);
        let exact = orient2d_i(p[0], p[1], p[2]);
        if exact != Sign::Zero {
            assert_eq!(got, exact);
        }
    }
}

#[test]
fn orient3d_sos_matches_oracle() {
    let pts: Vec<[i64; 3]> = (0..3)
        .flat_map(|x| (0..3).flat_map(move |y| (0..3).map(move |z| [x, y, z])))
        .collect();
    let mut rng = Rng(2);
    for _ in 0..3000 {
        let ids = shuffled_ids(&mut rng, 8);
        let p: Vec<[i64; 3]> = (0..4).map(|_| pts[rng.below(pts.len())]).collect();
        let got = orient3d_sos(
            t3(p[0], ids[0]),
            t3(p[1], ids[1]),
            t3(p[2], ids[2]),
            t3(p[3], ids[3]),
        )
        .unwrap();
        let want = oracle(&(0..4).map(|i| row3(p[i], ids[i])).collect::<Vec<_>>());
        assert_eq!(got, want, "{p:?} {ids:?}");
        let exact = orient3d_i(p[0], p[1], p[2], p[3]);
        if exact != Sign::Zero {
            assert_eq!(got, exact);
        }
    }
}

#[test]
fn incircle_sos_matches_oracle() {
    let pts: Vec<[i64; 2]> = (0..4).flat_map(|x| (0..4).map(move |y| [x, y])).collect();
    let mut rng = Rng(3);
    let mut degenerate = 0;
    for _ in 0..3000 {
        let ids = shuffled_ids(&mut rng, 8);
        let p: Vec<[i64; 2]> = (0..4).map(|_| pts[rng.below(pts.len())]).collect();
        let got = incircle_sos(
            t2(p[0], ids[0]),
            t2(p[1], ids[1]),
            t2(p[2], ids[2]),
            t2(p[3], ids[3]),
        )
        .unwrap();
        let want = oracle(&(0..4).map(|i| row2l(p[i], ids[i])).collect::<Vec<_>>());
        assert_eq!(got, want, "{p:?} {ids:?}");
        let exact = incircle_i(p[0], p[1], p[2], p[3]);
        if exact != Sign::Zero {
            assert_eq!(got, exact);
        } else {
            degenerate += 1;
        }
    }
    assert!(degenerate > 500);
}

#[test]
fn insphere_sos_matches_oracle() {
    let pts: Vec<[i64; 3]> = (0..3)
        .flat_map(|x| (0..3).flat_map(move |y| (0..2).map(move |z| [x, y, z])))
        .collect();
    let mut rng = Rng(4);
    let mut degenerate = 0;
    for _ in 0..1500 {
        let ids = shuffled_ids(&mut rng, 6);
        let p: Vec<[i64; 3]> = (0..5).map(|_| pts[rng.below(pts.len())]).collect();
        let got = insphere_sos(
            t3(p[0], ids[0]),
            t3(p[1], ids[1]),
            t3(p[2], ids[2]),
            t3(p[3], ids[3]),
            t3(p[4], ids[4]),
        )
        .unwrap();
        let want = oracle(&(0..5).map(|i| row3l(p[i], ids[i])).collect::<Vec<_>>());
        assert_eq!(got, want, "{p:?} {ids:?}");
        let exact = insphere_i(p[0], p[1], p[2], p[3], p[4]);
        if exact != Sign::Zero {
            assert_eq!(got, exact);
        } else {
            degenerate += 1;
        }
    }
    assert!(degenerate > 300);
}

#[test]
fn duplicate_ids_are_an_error_only_when_degenerate() {
    use sm_math::sos::SosError;
    let a = t2([0, 0], 5);
    let b = t2([1, 1], 5);
    let c = t2([2, 2], 6);
    assert_eq!(orient2d_sos(a, b, c), Err(SosError::DuplicateId(5)));
    // Nondegenerate input: the exact answer needs no perturbation.
    assert_eq!(
        orient2d_sos(t2([0, 0], 5), t2([1, 0], 5), t2([0, 1], 5)),
        Ok(Sign::Positive)
    );
}

#[test]
fn swap_antisymmetry_and_cyclic_invariance() {
    let pts: Vec<[i64; 3]> = (0..3)
        .flat_map(|x| (0..3).flat_map(move |y| (0..3).map(move |z| [x, y, z])))
        .collect();
    let mut rng = Rng(5);
    for _ in 0..2000 {
        let ids = shuffled_ids(&mut rng, 10);
        let p: Vec<[i64; 3]> = (0..5).map(|_| pts[rng.below(pts.len())]).collect();
        let t = |i: usize| t3(p[i], ids[i]);
        let base = insphere_sos(t(0), t(1), t(2), t(3), t(4)).unwrap();
        assert_eq!(insphere_sos(t(1), t(0), t(2), t(3), t(4)).unwrap(), -base);
        assert_eq!(insphere_sos(t(0), t(1), t(3), t(2), t(4)).unwrap(), -base);
        assert_eq!(insphere_sos(t(0), t(1), t(2), t(4), t(3)).unwrap(), -base);
        // Even permutation (3-cycle of the first three arguments).
        assert_eq!(insphere_sos(t(1), t(2), t(0), t(3), t(4)).unwrap(), base);
        let o = orient3d_sos(t(0), t(1), t(2), t(3)).unwrap();
        assert_eq!(orient3d_sos(t(1), t(0), t(2), t(3)).unwrap(), -o);
        assert_eq!(orient3d_sos(t(3), t(0), t(1), t(2)).unwrap(), -o); // 4-cycle is odd
    }
}

/// Twice the area of the triangle (exact integer).
fn area2(a: [i64; 2], b: [i64; 2], c: [i64; 2]) -> i128 {
    (i128::from(a[0] - c[0])) * i128::from(b[1] - c[1])
        - i128::from(a[1] - c[1]) * i128::from(b[0] - c[0])
}

/// SoS-Delaunay of a grid, brute force: the triangles must tile the grid's hull.
#[test]
fn sos_delaunay_2d_tiles_the_hull() {
    for (w, h, seed) in [
        (4i64, 4i64, 1u64),
        (5, 3, 2),
        (3, 5, 3),
        (4, 4, 4),
        (6, 2, 5),
    ] {
        let pts: Vec<[i64; 2]> = (0..w).flat_map(|x| (0..h).map(move |y| [x, y])).collect();
        let mut rng = Rng(seed);
        let ids = shuffled_ids(&mut rng, pts.len());
        let t = |i: usize| t2(pts[i], ids[i]);
        let n = pts.len();
        let mut directed: BTreeSet<(usize, usize)> = BTreeSet::new();
        let mut total: i128 = 0;
        for i in 0..n {
            for j in i + 1..n {
                for k in j + 1..n {
                    let (mut a, mut b, c) = (i, j, k);
                    if orient2d_sos(t(a), t(b), t(c)).unwrap() == Sign::Negative {
                        std::mem::swap(&mut a, &mut b);
                    }
                    let empty = (0..n)
                        .filter(|&m| m != i && m != j && m != k)
                        .all(|m| incircle_sos(t(a), t(b), t(c), t(m)).unwrap() == Sign::Negative);
                    if empty {
                        for (u, v) in [(a, b), (b, c), (c, a)] {
                            assert!(directed.insert((u, v)), "directed edge {u}->{v} used twice");
                        }
                        total += area2(pts[a], pts[b], pts[c]).abs();
                    }
                }
            }
        }
        assert_eq!(
            total,
            2 * i128::from((w - 1) * (h - 1)),
            "areas must sum to the hull area"
        );
        for &(u, v) in &directed {
            if !directed.contains(&(v, u)) {
                let (p, q) = (pts[u], pts[v]);
                let on_boundary = (p[0] == q[0] && (p[0] == 0 || p[0] == w - 1))
                    || (p[1] == q[1] && (p[1] == 0 || p[1] == h - 1));
                assert!(on_boundary, "unmatched interior edge {p:?}->{q:?}");
            }
        }
    }
}

#[test]
fn sos_delaunay_3d_tiles_the_hull() {
    for (dims, seed) in [([3i64, 3, 2], 1u64), ([2, 2, 3], 2), ([3, 2, 2], 3)] {
        let pts: Vec<[i64; 3]> = (0..dims[0])
            .flat_map(|x| (0..dims[1]).flat_map(move |y| (0..dims[2]).map(move |z| [x, y, z])))
            .collect();
        let mut rng = Rng(seed);
        let ids = shuffled_ids(&mut rng, pts.len());
        let t = |i: usize| t3(pts[i], ids[i]);
        let n = pts.len();
        let mut faces: BTreeSet<[usize; 3]> = BTreeSet::new();
        let mut volume6: i128 = 0;
        for i in 0..n {
            for j in i + 1..n {
                for k in j + 1..n {
                    for l in k + 1..n {
                        let (a, mut b, mut c, d) = (i, j, k, l);
                        if orient3d_sos(t(a), t(b), t(c), t(d)).unwrap() == Sign::Negative {
                            std::mem::swap(&mut b, &mut c);
                        }
                        let empty = (0..n)
                            .filter(|&m| m != i && m != j && m != k && m != l)
                            .all(|m| {
                                insphere_sos(t(a), t(b), t(c), t(d), t(m)).unwrap()
                                    == Sign::Negative
                            });
                        if !empty {
                            continue;
                        }
                        let rows: Vec<Vec<i128>> = [pts[a], pts[b], pts[c]]
                            .iter()
                            .map(|p| (0..3).map(|q| i128::from(p[q] - pts[d][q])).collect())
                            .collect();
                        volume6 += det(&rows).abs();
                        // Faces of a positively oriented (a,b,c,d), oriented consistently.
                        for f in [[a, b, c], [a, d, b], [b, d, c], [c, d, a]] {
                            // Canonical rotation so cyclic duplicates compare equal.
                            let m = (0..3).min_by_key(|&s| f[s]).unwrap();
                            let g = [f[m], f[(m + 1) % 3], f[(m + 2) % 3]];
                            assert!(
                                faces.insert(g),
                                "face {g:?} used twice in the same direction"
                            );
                        }
                    }
                }
            }
        }
        assert_eq!(
            volume6,
            6 * i128::from((dims[0] - 1) * (dims[1] - 1) * (dims[2] - 1))
        );
        for g in &faces {
            let rev = [g[0], g[2], g[1]];
            let m = (0..3).min_by_key(|&s| rev[s]).unwrap();
            let rev = [rev[m], rev[(m + 1) % 3], rev[(m + 2) % 3]];
            if !faces.contains(&rev) {
                let p: Vec<[i64; 3]> = g.iter().map(|&v| pts[v]).collect();
                let on_boundary = (0..3).any(|axis| {
                    (p[0][axis] == p[1][axis] && p[1][axis] == p[2][axis])
                        && (p[0][axis] == 0 || p[0][axis] == dims[axis] - 1)
                });
                assert!(on_boundary, "unmatched interior face {p:?}");
            }
        }
    }
}
