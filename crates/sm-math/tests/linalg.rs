//! Linear algebra against analytic results.

mod common;
use common::Rng;
use proptest::prelude::*;
use sm_core::{Point3, Transform, Vector3};
use sm_math::linalg::*;
use sm_testkit::strategies::{config, rotation};

fn maxdiff(a: &Mat3, b: &Mat3) -> f64 {
    (0..3)
        .flat_map(|i| (0..3).map(move |j| (i, j)))
        .map(|(i, j)| (a[i][j] - b[i][j]).abs())
        .fold(0.0, f64::max)
}

fn diag(d: [f64; 3]) -> Mat3 {
    [[d[0], 0.0, 0.0], [0.0, d[1], 0.0], [0.0, 0.0, d[2]]]
}

fn q_rot(axis: Vector3, angle: f64) -> Mat3 {
    Transform::rotation(Point3::new(0.0, 0.0, 0.0), axis, angle)
        .unwrap()
        .m
}

fn conj(q: &Mat3, d: &Mat3) -> Mat3 {
    mat3_mul(&mat3_mul(q, d), &mat3_transpose(q))
}

#[test]
fn determinants_match_cofactor_values() {
    assert_eq!(
        det3(&[[2.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 4.0]]),
        24.0
    );
    assert_eq!(
        det3(&[[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 10.0]]),
        -3.0
    );
    let m4 = [
        [1.0, 2.0, 3.0, 4.0],
        [5.0, 6.0, 7.0, 8.0],
        [2.0, 6.0, 4.0, 8.0],
        [3.0, 1.0, 1.0, 2.0],
    ];
    assert_eq!(det4(&m4), 72.0);
    let id4 = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ];
    assert_eq!(det4(&id4), 1.0);
}

#[test]
fn solve_recovers_integer_solutions() {
    let mut rng = Rng(3);
    for _ in 0..500 {
        // Unimodular A = L U with unit diagonals => well conditioned integer matrix.
        let l = [
            [1.0, 0.0, 0.0],
            [rng.int(3) as f64, 1.0, 0.0],
            [rng.int(3) as f64, rng.int(3) as f64, 1.0],
        ];
        let u = [
            [1.0, rng.int(3) as f64, rng.int(3) as f64],
            [0.0, 1.0, rng.int(3) as f64],
            [0.0, 0.0, 1.0],
        ];
        let a = mat3_mul(&l, &u);
        let x = Vector3::new(rng.int(50) as f64, rng.int(50) as f64, rng.int(50) as f64);
        let b = mat3_mul_vec(&a, x);
        let got = solve3(&a, b).unwrap();
        assert!((got - x).norm() < 1e-11, "{got:?} vs {x:?}");
        let inv = inverse3(&a).unwrap();
        assert!(maxdiff(&mat3_mul(&a, &inv), &IDENTITY3) < 1e-11);
        assert!((det3(&a) - 1.0).abs() < 1e-12);
    }
    let a4 = [
        [2.0, 1.0, 0.0, 0.0],
        [1.0, 3.0, 1.0, 0.0],
        [0.0, 1.0, 4.0, 1.0],
        [0.0, 0.0, 1.0, 5.0],
    ];
    let x4 = [1.0, -2.0, 3.0, -4.0];
    let b4: Vec<f64> = (0..4)
        .map(|i| (0..4).map(|j| a4[i][j] * x4[j]).sum())
        .collect();
    let got = solve4(&a4, [b4[0], b4[1], b4[2], b4[3]]).unwrap();
    for i in 0..4 {
        assert!((got[i] - x4[i]).abs() < 1e-14);
    }
}

#[test]
fn pivoting_and_scale_invariance() {
    // Zero leading entry forces a row swap.
    let a = [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]];
    assert_eq!(
        solve3(&a, Vector3::new(2.0, 3.0, 4.0)).unwrap(),
        Vector3::new(3.0, 2.0, 4.0)
    );
    // The singularity test is relative: a uniformly tiny/huge well-conditioned matrix solves.
    for s in [1e-200, 1e-8, 1e8, 1e200] {
        let b = [[2.0 * s, s, 0.0], [s, 3.0 * s, s], [0.0, s, 4.0 * s]];
        let x = solve3(&b, Vector3::new(s, 2.0 * s, 3.0 * s)).unwrap();
        let r = mat3_mul_vec(&b, x) - Vector3::new(s, 2.0 * s, 3.0 * s);
        // (norm() itself would overflow at 1e200, so compare components.)
        assert!(
            r.x.abs().max(r.y.abs()).max(r.z.abs()) <= 1e-14 * s * 6.0,
            "s={s} r={r:?} x={x:?}"
        );
    }
}

#[test]
fn singular_and_nonfinite_are_errors() {
    let sing = [[1.0, 2.0, 3.0], [2.0, 4.0, 6.0], [1.0, 0.0, 1.0]];
    assert!(matches!(
        solve3(&sing, Vector3::new(1.0, 2.0, 3.0)),
        Err(LinAlgError::Singular { .. })
    ));
    assert!(matches!(inverse3(&sing), Err(LinAlgError::Singular { .. })));
    assert!(matches!(
        solve3(&[[0.0; 3]; 3], Vector3::default()),
        Err(LinAlgError::Singular { .. })
    ));
    let nan = [[f64::NAN, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    assert_eq!(
        solve3(&nan, Vector3::default()),
        Err(LinAlgError::NonFinite)
    );
    assert_eq!(sym3_eigen(&nan), Err(LinAlgError::NonFinite));
    assert_eq!(svd3(&nan), Err(LinAlgError::NonFinite));
}

fn check_eigen(a: &Mat3, e: &SymEigen3, tol: f64) {
    let scale = mat3_frobenius(a).max(f64::MIN_POSITIVE);
    for k in 0..3 {
        let v = e.vector(k);
        assert!((v.norm() - 1.0).abs() < 1e-14, "unit");
        let r = mat3_mul_vec(a, v) - v * e.values[k];
        assert!(
            r.norm() <= tol * scale,
            "residual {} for lambda {}",
            r.norm(),
            e.values[k]
        );
        for j in 0..k {
            assert!(v.dot(e.vector(j)).abs() < 1e-14, "orthogonal");
        }
    }
    assert!(e.values[0] <= e.values[1] && e.values[1] <= e.values[2]);
}

#[test]
fn eigen_of_rotated_diagonals() {
    let q = q_rot(Vector3::new(1.0, 2.0, -0.5), 0.7);
    for d in [
        [1.0, 2.0, 3.0],
        [-4.0, 0.5, 9.0],
        [1.0, 1.0, 5.0],
        [2.0, 2.0, 2.0],
        [0.0, 0.0, 0.0],
        [1e-6, 1.0, 1e6],
        [1e150, 2e150, 3e150],
    ] {
        let a = conj(&q, &diag(d));
        let e = sym3_eigen(&a).unwrap();
        let mut want = d;
        want.sort_by(f64::total_cmp);
        let sc = want.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        for (got, w) in e.values.iter().zip(want) {
            assert!((got - w).abs() <= 1e-14 * sc, "{d:?}: {:?}", e.values);
        }
        check_eigen(&a, &e, 1e-14);
    }
}

#[test]
fn eigen_of_cuboid_inertia_tensor() {
    // Uniform box mass m, edges (a, b, c): principal moments m/12 (b^2+c^2, a^2+c^2, a^2+b^2).
    let (m, a, b, c) = (3.0, 2.0, 0.5, 1.5);
    let principal = [
        m / 12.0 * (b * b + c * c),
        m / 12.0 * (a * a + c * c),
        m / 12.0 * (a * a + b * b),
    ];
    let q = q_rot(Vector3::new(0.3, -1.0, 0.8), 1.1);
    let tensor = conj(&q, &diag(principal));
    let e = sym3_eigen(&tensor).unwrap();
    let mut want = principal;
    want.sort_by(f64::total_cmp);
    for (got, w) in e.values.iter().zip(want) {
        assert!((got - w).abs() < 1e-14);
    }
    // The eigenvector of the largest moment is the rotated x-axis of the box (sign-canonical).
    let axis = [mat3_column(&q, 0), mat3_column(&q, 1), mat3_column(&q, 2)];
    let big = principal
        .iter()
        .enumerate()
        .max_by(|x, y| x.1.total_cmp(y.1))
        .unwrap()
        .0;
    assert!(e.vector(2).dot(axis[big]).abs() > 1.0 - 1e-13);
}

#[test]
fn eigen_trivial_cases_and_determinism() {
    let e = sym3_eigen(&IDENTITY3).unwrap();
    assert_eq!(e.values, [1.0, 1.0, 1.0]);
    assert_eq!(e.vectors, IDENTITY3);
    let a = [[2.0, 1.0, 0.0], [1.0, 2.0, 0.0], [0.0, 0.0, 5.0]];
    let e = sym3_eigen(&a).unwrap();
    assert!(
        (e.values[0] - 1.0).abs() < 1e-15
            && (e.values[1] - 3.0).abs() < 1e-15
            && (e.values[2] - 5.0).abs() < 1e-15
    );
    assert_eq!(sym3_eigen(&a).unwrap(), e);
    // Asymmetric input is symmetrised.
    let skew = [[1.0, 2.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let e = sym3_eigen(&skew).unwrap();
    assert!((e.values[2] - 2.0).abs() < 1e-14 && (e.values[0] - 0.0).abs() < 1e-14);
}

fn check_svd(a: &Mat3, s: &Svd3, tol: f64) {
    let scale = mat3_frobenius(a).max(f64::MIN_POSITIVE);
    assert!(
        maxdiff(&s.reconstruct(), a) <= tol * scale,
        "reconstruction {:?}",
        s
    );
    for m in [&s.u, &s.v] {
        let g = mat3_mul(&mat3_transpose(m), m);
        assert!(maxdiff(&g, &IDENTITY3) < 1e-13, "orthogonality {g:?}");
    }
    assert!(s.s[0] >= s.s[1] && s.s[1] >= s.s[2] && s.s[2] >= 0.0);
}

#[test]
fn svd_known_matrices() {
    // Scaling by a diagonal: singular values are |d| sorted.
    let a = diag([3.0, -5.0, 1.0]);
    let s = svd3(&a).unwrap();
    assert_eq!(s.s, [5.0, 3.0, 1.0]);
    check_svd(&a, &s, 1e-15);
    // Rotation and reflection: all singular values 1, condition 1.
    let r = q_rot(Vector3::new(1.0, 1.0, 1.0), 2.0);
    let s = svd3(&r).unwrap();
    check_svd(&r, &s, 1e-14);
    for v in s.s {
        assert!((v - 1.0).abs() < 1e-14);
    }
    assert!((s.condition() - 1.0).abs() < 1e-13);
    let mirror = Transform::reflection(Vector3::new(1.0, 2.0, 2.0), 0.0)
        .unwrap()
        .m;
    let s = svd3(&mirror).unwrap();
    check_svd(&mirror, &s, 1e-14);
    assert!(s.s.iter().all(|v| (v - 1.0).abs() < 1e-14));
    // Rank 1: u v^T has singular values (|u||v|, 0, 0).
    let (u, v) = ([1.0, 2.0, 2.0], [3.0, 0.0, 4.0]);
    let mut o = [[0.0; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            o[i][j] = u[i] * v[j];
        }
    }
    let s = svd3(&o).unwrap();
    check_svd(&o, &s, 1e-14);
    assert!((s.s[0] - 15.0).abs() < 1e-13 && s.s[1] < 1e-13 && s.s[2] < 1e-13);
    assert!(s.condition() > 1e12);
    // Rank 2 and zero.
    let r2 = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
    let s = svd3(&r2).unwrap();
    check_svd(&r2, &s, 1e-14);
    assert!(s.s[2] < 1e-14 * s.s[0]);
    let s = svd3(&[[0.0; 3]; 3]).unwrap();
    assert_eq!(s.s, [0.0; 3]);
    check_svd(&[[0.0; 3]; 3], &s, 0.0);
}

#[test]
fn svd_agrees_with_eigen_of_gram_matrix() {
    let mut rng = Rng(8);
    for _ in 0..300 {
        let a: Mat3 =
            core::array::from_fn(|_| core::array::from_fn(|_| rng.int(1000) as f64 / 100.0));
        let s = svd3(&a).unwrap();
        check_svd(&a, &s, 1e-14);
        let g = mat3_mul(&mat3_transpose(&a), &a);
        let e = sym3_eigen(&g).unwrap();
        for k in 0..3 {
            let sig2 = s.s[2 - k] * s.s[2 - k];
            assert!(
                (sig2 - e.values[k]).abs() <= 1e-12 * e.values[2].max(1e-300),
                "{sig2} vs {}",
                e.values[k]
            );
        }
    }
}

proptest! {
    #![proptest_config(config(300))]

    #[test]
    fn eigen_and_svd_of_random_transforms(r1 in rotation(), r2 in rotation(), d in prop::array::uniform3(-10.0f64..10.0)) {
        let a = mat3_mul(&mat3_mul(&r1.m, &diag(d)), &r2.m);
        let s = svd3(&a).unwrap();
        check_svd(&a, &s, 1e-14);
        let mut want = [d[0].abs(), d[1].abs(), d[2].abs()];
        want.sort_by(|x, y| y.total_cmp(x));
        for (got, w) in s.s.iter().zip(want) { prop_assert!((got - w).abs() < 1e-13); }
        let sym = conj(&r1.m, &diag(d));
        let e = sym3_eigen(&sym).unwrap();
        check_eigen(&sym, &e, 1e-14);
    }
}
