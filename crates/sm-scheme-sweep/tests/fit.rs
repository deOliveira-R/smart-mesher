//! Acceptance 9: `fit_affine` and `fit_target_transform` against known maps.

mod common;

use common::rng::Rng;
use sm_core::{Point3, Transform, Vector3};
use sm_scheme_sweep::{SweepError, fit_affine, fit_target_transform};

fn rz(phi: f64) -> Transform {
    Transform::rotation(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), phi).expect("rz")
}

fn random_points(rng: &mut Rng, n: usize, planar: bool) -> Vec<Point3> {
    (0..n)
        .map(|_| {
            Point3::new(
                rng.range(-2.0, 2.0),
                rng.range(-2.0, 2.0),
                if planar { 0.0 } else { rng.range(-2.0, 2.0) },
            )
        })
        .collect()
}

fn random_affine(rng: &mut Rng) -> Transform {
    loop {
        let mut m = [[0.0; 3]; 3];
        for row in &mut m {
            for e in row.iter_mut() {
                *e = rng.range(-1.5, 1.5);
            }
        }
        let t = Transform {
            m,
            t: Vector3::new(
                rng.range(-3.0, 3.0),
                rng.range(-3.0, 3.0),
                rng.range(-3.0, 3.0),
            ),
        };
        if t.det() > 0.2 {
            return t;
        }
    }
}

fn close(a: Point3, b: Point3, tol: f64) -> bool {
    (a - b).norm() <= tol
}

#[test]
fn a_pure_translation_is_returned_bit_exactly() {
    // A step that is not representable: the mean of equal differences could round differently.
    let src = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let dst: Vec<Point3> = src
        .iter()
        .map(|p| *p + Vector3::new(0.0, 0.0, 0.1))
        .collect();
    let t = fit_target_transform(
        &src,
        &dst,
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(0.0, 0.0, 1.0),
        1e-9,
    )
    .expect("translation");
    assert_eq!(t.t, dst[0] - src[0]);
    for (i, row) in t.m.iter().enumerate() {
        for (j, e) in row.iter().enumerate() {
            assert_eq!(*e, if i == j { 1.0 } else { 0.0 });
        }
    }
}

#[test]
fn fit_affine_recovers_a_random_affine_map() {
    let mut rng = Rng(7);
    for n in [6usize, 9, 40] {
        for _ in 0..20 {
            let a = random_affine(&mut rng);
            let src = random_points(&mut rng, n, false);
            let dst: Vec<Point3> = src.iter().map(|p| a.apply_point(*p)).collect();
            let t = fit_affine(&src, &dst).expect("fit");
            for i in 0..3 {
                for j in 0..3 {
                    assert!((t.m[i][j] - a.m[i][j]).abs() < 1e-12, "m[{i}][{j}]");
                }
            }
            assert!((t.t - a.t).norm() < 1e-12);
            assert!(t.det() > 0.0);
        }
    }
}

#[test]
fn fit_affine_of_a_planar_set_is_the_in_plane_map() {
    let mut rng = Rng(11);
    for _ in 0..20 {
        let mut a = random_affine(&mut rng);
        // The in-plane block must be orientation preserving for det > 0 of the completed map.
        while a.m[0][0] * a.m[1][1] - a.m[0][1] * a.m[1][0] < 0.2 {
            a = random_affine(&mut rng);
        }
        let src = random_points(&mut rng, 8, true);
        let dst: Vec<Point3> = src.iter().map(|p| a.apply_point(*p)).collect();
        let t = fit_affine(&src, &dst).expect("planar fit");
        for (s, d) in src.iter().zip(&dst) {
            assert!(close(t.apply_point(*s), *d, 1e-12));
        }
        // The null (normal) direction is completed with the identity.
        let n = t.apply_vector(Vector3::new(0.0, 0.0, 1.0));
        assert!((n - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12, "{n:?}");
        // In-plane vectors map like the true map's in-plane part.
        let e = Vector3::new(0.3, -0.7, 0.0);
        assert!((t.apply_vector(e) - a.apply_vector(e)).norm() < 1e-12);
        assert!(t.det() > 0.0);
    }
}

#[test]
fn fit_affine_rejects_too_few_or_collinear_points() {
    let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
    let three = [p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0)];
    assert!(matches!(
        fit_affine(&three, &three),
        Err(SweepError::TooFewPoints { have: 3, need: 4 })
    ));
    let line: Vec<Point3> = (0..6)
        .map(|i| p(f64::from(i), 2.0 * f64::from(i), 0.5))
        .collect();
    assert!(matches!(
        fit_affine(&line, &line),
        Err(SweepError::Degenerate { .. })
    ));
    let same = vec![p(1.0, 1.0, 1.0); 5];
    assert!(matches!(
        fit_affine(&same, &same),
        Err(SweepError::Degenerate { .. })
    ));
    assert!(matches!(
        fit_affine(&line, &line[..5]),
        Err(SweepError::LengthMismatch { .. })
    ));
    let mut bad = line.clone();
    bad[2].x = f64::NAN;
    assert!(matches!(
        fit_affine(&bad, &line),
        Err(SweepError::NonFinite)
    ));
}

fn square(z: f64) -> Vec<Point3> {
    vec![
        Point3::new(0.0, 0.0, z),
        Point3::new(2.0, 0.0, z),
        Point3::new(2.0, 1.0, z),
        Point3::new(0.5, 1.5, z),
        Point3::new(-0.3, 0.8, z),
    ]
}

#[test]
fn a_pure_translation_is_a_bit_exact_translation() {
    let src = square(0.0);
    let d = Vector3::new(0.25, -1.5, 3.0);
    let dst: Vec<Point3> = src.iter().map(|p| *p + d).collect();
    let t = fit_target_transform(
        &src,
        &dst,
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(0.0, 0.0, 1.0),
        1e-9,
    )
    .expect("translation");
    assert_eq!(t, Transform::translation(d));
}

#[test]
fn a_rotation_by_phi_is_rz_phi() {
    let src = square(0.0);
    for phi in [0.3, 1.0, std::f64::consts::FRAC_PI_2, 2.5, -0.9] {
        let g = rz(phi);
        let dst: Vec<Point3> = src.iter().map(|p| g.apply_point(*p)).collect();
        let t = fit_target_transform(
            &src,
            &dst,
            Vector3::new(0.0, 0.0, -1.0),
            Vector3::new(0.0, 0.0, 1.0),
            1e-9,
        )
        .expect("rotation");
        for i in 0..3 {
            for j in 0..3 {
                assert!(
                    (t.m[i][j] - g.m[i][j]).abs() < 1e-12,
                    "phi {phi} m[{i}][{j}]"
                );
            }
        }
        assert!(t.t.norm() < 1e-12);
    }
}

#[test]
fn a_rotation_of_a_tilted_face_about_an_axis_is_recovered() {
    // A face in the plane y = 0 swept about z: target normal is the rotated, negated normal.
    let src: Vec<Point3> = vec![
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
    ];
    let g = rz(std::f64::consts::FRAC_PI_2);
    let dst: Vec<Point3> = src.iter().map(|p| g.apply_point(*p)).collect();
    let n_s = Vector3::new(0.0, -1.0, 0.0);
    let n_t = -g.apply_vector(n_s);
    let t = fit_target_transform(&src, &dst, n_s, n_t, 1e-9).expect("rotation");
    for (s, d) in src.iter().zip(&dst) {
        assert!(close(t.apply_point(*s), *d, 1e-12));
    }
    assert!((t.det() - 1.0).abs() < 1e-12);
}

#[test]
fn a_non_affine_correspondence_gives_none() {
    let src = square(0.0);
    let n_s = Vector3::new(0.0, 0.0, -1.0);
    let n_t = Vector3::new(0.0, 0.0, 1.0);
    // Swap two vertices of a rotated copy.
    let g = rz(0.6);
    let mut dst: Vec<Point3> = src
        .iter()
        .map(|p| g.apply_point(*p) + Vector3::new(0.0, 0.0, 2.0))
        .collect();
    dst.swap(1, 3);
    assert!(fit_target_transform(&src, &dst, n_s, n_t, 1e-9).is_none());
    // One vertex moved off the affine image.
    let mut dst: Vec<Point3> = src
        .iter()
        .map(|p| *p + Vector3::new(0.0, 0.0, 2.0))
        .collect();
    dst[2].x += 1e-3;
    assert!(fit_target_transform(&src, &dst, n_s, n_t, 1e-9).is_none());
    // A mirror image is not an orientation-preserving fit.
    let dst: Vec<Point3> = src.iter().map(|p| Point3::new(-p.x, p.y, 2.0)).collect();
    assert!(fit_target_transform(&src, &dst, n_s, n_t, 1e-9).is_none());
    // Wrong lengths.
    assert!(fit_target_transform(&src, &dst[..3], n_s, n_t, 1e-9).is_none());
}

#[test]
fn a_scaled_target_is_affine() {
    let src = square(0.0);
    let dst: Vec<Point3> = src
        .iter()
        .map(|p| Point3::new(1.5 * p.x, 0.5 * p.y, 4.0))
        .collect();
    let t = fit_target_transform(
        &src,
        &dst,
        Vector3::new(0.0, 0.0, -1.0),
        Vector3::new(0.0, 0.0, 1.0),
        1e-9,
    )
    .expect("tapered sweep");
    for (s, d) in src.iter().zip(&dst) {
        assert!(close(t.apply_point(*s), *d, 1e-12));
    }
}
