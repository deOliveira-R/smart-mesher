//! Finite-volume metrics against analytic values (OpenFOAM checkMesh / Fluent conventions).
#![allow(missing_docs)]

mod common;
use common::*;
use sm_mesh_api::{CellIx, CellShape, FaceIx, MeshRead, SerialAlloc, cell_volume};
use sm_quality::*;
use sm_testkit::broken::Tampered;
use sm_testkit::determinism::{check, hash_f64_bits};
use sm_testkit::meshes::{CellSpec, all_mesh_fixtures, build_mesh, hexagonal_prism, single_hex};

const TOL: f64 = 1e-12;
const FV_ALL: &[FvMetric] = FvMetric::ALL;

fn with_threads<R: Send>(t: usize, f: impl FnOnce() -> R + Send) -> R {
    rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap().install(f)
}

fn col(cols: &[FvColumn], m: FvMetric) -> &[f64] {
    &cols.iter().find(|c| c.metric == m).unwrap().values
}

const CUBE: [[f64; 3]; 8] = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];

/// Hex A = [0,1]^3 and hex B on [1,2] in x whose x=2 layer is shifted by 2 in y.
fn sheared_pair() -> sm_mesh_api::reference::ReferenceMesh {
    let mut p = CUBE.to_vec();
    p.extend([[2., 2., 0.], [2., 3., 0.], [2., 2., 1.], [2., 3., 1.]]); // 8..11
    let cells = [
        CellSpec::Template(CellShape::Hex, (0..8).collect()),
        CellSpec::Template(CellShape::Hex, vec![1, 8, 9, 2, 5, 10, 11, 6]),
    ];
    build_mesh(&p, &cells, &mut SerialAlloc::new()).unwrap()
}

/// Two bricks, widths 1 and 2 along x.
fn stretched_pair() -> sm_mesh_api::reference::ReferenceMesh {
    let mut p = CUBE.to_vec();
    p.extend([[3., 0., 0.], [3., 1., 0.], [3., 0., 1.], [3., 1., 1.]]);
    let cells = [
        CellSpec::Template(CellShape::Hex, (0..8).collect()),
        CellSpec::Template(CellShape::Hex, vec![1, 8, 9, 2, 5, 10, 11, 6]),
    ];
    build_mesh(&p, &cells, &mut SerialAlloc::new()).unwrap()
}

fn interior_face(m: &dyn MeshRead) -> usize {
    (0..m.face_count()).find(|&f| m.face_right(FaceIx::new(f)).is_some()).unwrap()
}

fn face_at(g: &FvGeometry, c: [f64; 3]) -> usize {
    g.face_centre.iter().position(|p| (0..3).all(|i| (p[i] - c[i]).abs() < 1e-12)).unwrap_or_else(|| panic!("no face at {c:?}"))
}

type P = [f64; 3];
fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn len(a: P) -> f64 {
    dot(a, a).sqrt()
}

#[test]
fn sheared_pair_matches_analytic_values() {
    let m = sheared_pair();
    let g = fv_geometry(&m).unwrap();
    assert!((g.cell_volume[0] - 1.0).abs() < TOL && (g.cell_volume[1] - 1.0).abs() < TOL);
    let cols = fv_compute(&m, &g, FV_ALL).unwrap();
    let shared = interior_face(&m);
    let c = g.face_centre[shared];
    assert!((0..3).all(|i| (c[i] - [1.0, 0.5, 0.5][i]).abs() < TOL), "shared face centre {c:?}");
    assert!((col(&cols, FvMetric::NonOrthogonality)[shared] - 45.0).abs() < TOL);
    assert!((col(&cols, FvMetric::Skewness)[shared] - 1.0).abs() < TOL);
    assert!((col(&cols, FvMetric::InterpolationWeight)[shared] - 0.5).abs() < TOL);
    assert!((col(&cols, FvMetric::VolumeRatio)[shared] - 1.0).abs() < TOL);
    let oq = col(&cols, FvMetric::OrthogonalQuality);
    assert!((oq[0] - 0.5f64.sqrt()).abs() < TOL, "A {}", oq[0]);
    assert!((oq[1] - 0.2f64.sqrt()).abs() < TOL, "B {}", oq[1]);
    let b = face_at(&g, [2.0, 2.5, 0.5]);
    assert!((col(&cols, FvMetric::Skewness)[b] - 2.0).abs() < TOL);
    // The other boundary faces of A are orthogonal.
    let a_bottom = face_at(&g, [0.5, 0.5, 0.0]);
    assert!(col(&cols, FvMetric::Skewness)[a_bottom].abs() < TOL);
}

/// Direct, mesh-free re-derivation of the internal-face metrics from the eight corner points.
#[test]
fn sheared_pair_matches_an_independent_implementation() {
    // Cell centres by hand: the pyramid rule gives the vertex mean for these parallelepipeds.
    let ca = [0.5, 0.5, 0.5];
    let cb = [1.5, 1.5, 0.5];
    let cf = [1.0, 0.5, 0.5];
    let sf = [1.0, 0.0, 0.0];
    let d = sub(cb, ca);
    let non_orth = (dot(d, sf) / (len(d) * len(sf))).acos().to_degrees();
    let cpf = sub(cf, ca);
    let k = dot(sf, cpf) / dot(sf, d);
    let sv = [cpf[0] - k * d[0], cpf[1] - k * d[1], cpf[2] - k * d[2]];
    // Face corners: (1,0,0) (1,1,0) (1,1,1) (1,0,1): extent along sv_hat = y is 0.5.
    let fd = (0.2f64 * len(d)).max(0.5);
    let skew = len(sv) / fd;
    let w_own = dot(sf, sub(cf, ca)).abs();
    let w_nei = dot(sf, sub(cb, cf)).abs();
    let w = w_nei / (w_own + w_nei);
    let m = sheared_pair();
    let g = fv_geometry(&m).unwrap();
    let cols = fv_compute(&m, &g, FV_ALL).unwrap();
    let s = interior_face(&m);
    for (cell, want) in [(0, ca), (1, cb)] {
        assert!((0..3).all(|i| (g.cell_centre[cell][i] - want[i]).abs() < TOL), "centre {cell}");
    }
    assert!((col(&cols, FvMetric::NonOrthogonality)[s] - non_orth).abs() < TOL);
    assert!((col(&cols, FvMetric::Skewness)[s] - skew).abs() < TOL);
    assert!((col(&cols, FvMetric::InterpolationWeight)[s] - w.min(1.0 - w)).abs() < TOL);
}

#[test]
fn stretched_pair_matches_analytic_values() {
    let m = stretched_pair();
    let g = fv_geometry(&m).unwrap();
    let cols = fv_compute(&m, &g, FV_ALL).unwrap();
    let s = interior_face(&m);
    assert!((col(&cols, FvMetric::VolumeRatio)[s] - 2.0).abs() < TOL);
    assert!((col(&cols, FvMetric::InterpolationWeight)[s] - 1.0 / 3.0).abs() < TOL);
    assert!(col(&cols, FvMetric::NonOrthogonality)[s].abs() < TOL);
    assert!(col(&cols, FvMetric::Skewness)[s].abs() < TOL);
}

#[test]
fn cell_determinant_matches_analytic_values() {
    let cube = single_hex().unwrap();
    let g = fv_geometry(&cube.mesh).unwrap();
    let det = |m: &dyn MeshRead, g: &FvGeometry| fv_compute(m, g, &[FvMetric::CellDeterminant]).unwrap()[0].values[0];
    assert!((det(&cube.mesh, &g) - 1.0).abs() < TOL);

    let mut p = CUBE.to_vec();
    for q in &mut p {
        q[2] *= 2.0;
    }
    let brick = build_mesh(&p, &[CellSpec::Template(CellShape::Hex, (0..8).collect())], &mut SerialAlloc::new()).unwrap();
    let g = fv_geometry(&brick).unwrap();
    assert!((det(&brick, &g) - 32.0 * 27.0 / 1000.0).abs() < TOL);

    let hp = hexagonal_prism(1.0, 2.0).unwrap();
    let g = fv_geometry(&hp.mesh).unwrap();
    let s3 = 3.0f64.sqrt();
    let want = 108.0 * s3 / ((12.0 + 3.0 * s3) / 3.0).powi(3);
    assert!((want - 0.9932).abs() < 1e-4);
    assert!((det(&hp.mesh, &g) - want).abs() < TOL);
}

/// A prism over `base` (CCW, in the xy-plane, with z offsets `zb`) of height 1, as one polyhedron.
/// Returns the mesh and the index of its bottom face.
fn prism_over(base: &[[f64; 3]]) -> (sm_mesh_api::reference::ReferenceMesh, usize) {
    let n = base.len();
    let mut pts: Vec<[f64; 3]> = base.to_vec();
    pts.extend(base.iter().map(|p| [p[0], p[1], p[2] + 1.0]));
    let mut loops: Vec<Vec<usize>> = vec![(0..n).rev().collect(), (n..2 * n).collect()];
    for i in 0..n {
        let j = (i + 1) % n;
        loops.push(vec![i, j, n + j, n + i]);
    }
    let m = build_mesh(&pts, &[CellSpec::Poly(loops)], &mut SerialAlloc::new()).unwrap();
    let g = fv_geometry(&m).unwrap();
    let bottom = (0..m.face_count()).min_by(|&a, &b| g.face_centre[a][2].total_cmp(&g.face_centre[b][2])).unwrap();
    (m, bottom)
}

#[test]
fn face_shape_metrics_match_analytic_values() {
    let h = 0.25;
    let (m, f) = prism_over(&[[0., 0., h], [1., 0., -h], [1., 1., h], [0., 1., -h]]);
    let g = fv_geometry(&m).unwrap();
    let cols = fv_compute(&m, &g, FV_ALL).unwrap();
    let want_flat = 1.0 / (1.0 + 4.0 * h * h).sqrt();
    assert!((want_flat - 0.894427191).abs() < 1e-9);
    assert!((col(&cols, FvMetric::FaceFlatness)[f] - want_flat).abs() < TOL);
    let want_warp = (2.0 * h).atan().to_degrees();
    assert!((want_warp - 26.5650511771).abs() < 1e-9);
    assert!((col(&cols, FvMetric::FaceWarp)[f] - want_warp).abs() < TOL);
    assert!(col(&cols, FvMetric::FaceConcavity)[f].abs() < TOL, "saddle is convex in projection");

    let (m, f) = prism_over(&[[0., 0., 0.], [2., 0., 0.], [0.5, 0.5, 0.], [0., 2., 0.]]);
    let g = fv_geometry(&m).unwrap();
    let cols = fv_compute(&m, &g, FV_ALL).unwrap();
    let want = 0.8f64.asin().to_degrees();
    assert!((want - 53.1301023542).abs() < 1e-9);
    assert!((col(&cols, FvMetric::FaceConcavity)[f] - want).abs() < TOL);
    assert!((col(&cols, FvMetric::FaceFlatness)[f] - 1.0).abs() < TOL);
    assert!(col(&cols, FvMetric::FaceWarp)[f].abs() < 1e-9);

    let (m, f) = prism_over(&[[0., 0., 0.], [2., 0., 0.], [2., 1., 0.], [1., 1., 0.], [1., 2., 0.], [0., 2., 0.]]);
    let g = fv_geometry(&m).unwrap();
    let cols = fv_compute(&m, &g, FV_ALL).unwrap();
    assert!((col(&cols, FvMetric::FaceConcavity)[f] - 90.0).abs() < TOL);
    // The convex top and side faces report 0.
    let c = col(&cols, FvMetric::FaceConcavity);
    assert_eq!(c.iter().filter(|x| **x > 1e-9).count(), 2, "only the two L-shaped faces: {c:?}");
}

#[test]
fn fv_volumes_equal_cell_volume_on_planar_fixtures() {
    for fx in all_mesh_fixtures().unwrap() {
        let g = fv_geometry(&fx.mesh).unwrap();
        for c in 0..fx.mesh.cell_count() {
            let v = cell_volume(&fx.mesh, CellIx::new(c));
            assert!((g.cell_volume[c] - v).abs() < TOL * v.max(1.0), "{} cell {c}: {} vs {v}", fx.name, g.cell_volume[c]);
        }
    }
}

#[test]
fn names_round_trip() {
    for &m in Metric::ALL {
        assert_eq!(Metric::from_name(m.name()), Some(m));
    }
    for &m in FvMetric::ALL {
        assert_eq!(FvMetric::from_name(m.name()), Some(m));
        assert_eq!(Measure::from_name(m.name()), Some(Measure::Fv(m)));
    }
    for &m in Metric::ALL {
        assert_eq!(Measure::from_name(m.name()), Some(Measure::Verdict(m)));
    }
    for &p in Profile::ALL {
        assert_eq!(Profile::from_name(p.name()), Some(p));
        assert!(!rules(p).is_empty());
    }
    assert_eq!(Metric::from_name("nope"), None);
    assert_eq!(Metric::MaxAspectFrobenius.name(), "max_aspect_frobenius");
}

#[test]
fn boundary_faces_are_nan_for_internal_only_metrics() {
    let m = sheared_pair();
    let g = fv_geometry(&m).unwrap();
    let cols = fv_compute(&m, &g, FV_ALL).unwrap();
    for metric in [FvMetric::NonOrthogonality, FvMetric::VolumeRatio, FvMetric::InterpolationWeight] {
        let v = col(&cols, metric);
        for (f, x) in v.iter().enumerate() {
            assert_eq!(x.is_nan(), m.face_right(FaceIx::new(f)).is_none(), "{metric:?} face {f}");
        }
    }
    for metric in [FvMetric::Skewness, FvMetric::FaceFlatness, FvMetric::FaceWarp, FvMetric::FaceConcavity] {
        assert!(col(&cols, metric).iter().all(|x| x.is_finite()), "{metric:?}");
    }
}

#[test]
fn two_dimensional_mesh_is_a_bad_argument() {
    let m = surface_mesh(&[[0., 0.], [1., 0.], [1., 1.], [0., 1.]], &[vec![0, 1, 2, 3]]);
    assert!(matches!(fv_geometry(&m), Err(QualityError::BadArgument(_))));
    let g = fv_geometry(&single_hex().unwrap().mesh).unwrap();
    assert!(matches!(fv_compute(&m, &g, FV_ALL), Err(QualityError::BadArgument(_))));
}

#[test]
fn geometry_of_the_wrong_mesh_is_rejected() {
    let a = single_hex().unwrap();
    let b = sheared_pair();
    let g = fv_geometry(&a.mesh).unwrap();
    assert!(matches!(fv_compute(&b, &g, FV_ALL), Err(QualityError::BadArgument(_))));
}

#[test]
fn face_to_cell_reduces_over_faces_and_skips_nan() {
    let m = sheared_pair();
    let g = fv_geometry(&m).unwrap();
    let cols = fv_compute(&m, &g, &[FvMetric::NonOrthogonality, FvMetric::Skewness]).unwrap();
    let no = col(&cols, FvMetric::NonOrthogonality);
    let mx = face_to_cell(&m, no, Reduce::Max);
    assert!((mx[0] - 45.0).abs() < TOL && (mx[1] - 45.0).abs() < TOL);
    let sk = col(&cols, FvMetric::Skewness);
    let min = face_to_cell(&m, sk, Reduce::Min);
    assert!(min[0].abs() < TOL, "an orthogonal boundary face gives skewness 0");
    // All-NaN faces give NaN.
    assert!(face_to_cell(&m, &vec![f64::NAN; m.face_count()], Reduce::Max).iter().all(|x| x.is_nan()));
}

#[test]
fn rules_grade_with_equality_passing() {
    let r = rules(Profile::Generic).iter().find(|r| r.measure == Measure::Fv(FvMetric::Skewness)).unwrap();
    assert_eq!(r.grade(2.0, false), Grade::Ok);
    assert_eq!(r.grade(2.5, false), Grade::Warn);
    assert_eq!(r.grade(4.0, false), Grade::Warn);
    assert_eq!(r.grade(4.5, false), Grade::Fail);
    assert_eq!(r.grade(4.5, true), Grade::Warn);
    assert_eq!(r.grade(20.5, true), Grade::Fail);
    assert_eq!(r.grade(f64::NAN, false), Grade::Undefined);
    let r = rules(Profile::Generic).iter().find(|r| r.measure == Measure::Fv(FvMetric::InterpolationWeight)).unwrap();
    assert_eq!(r.grade(0.05, false), Grade::Warn);
    assert_eq!(r.grade(0.0499, false), Grade::Fail);
    assert_eq!(r.grade(0.2, false), Grade::Ok);
    let r = rules(Profile::OpenFoam).iter().find(|r| r.measure == Measure::Fv(FvMetric::FaceFlatness)).unwrap();
    assert_eq!(r.grade(0.5, false), Grade::Warn, "flatness only warns in checkMesh");
    assert!(Grade::Fail.reaches(Grade::Warn) && !Grade::Warn.reaches(Grade::Fail) && !Grade::Ok.reaches(Grade::Warn));
}

fn fingerprint(m: &dyn MeshRead) -> Vec<u64> {
    let g = fv_geometry(m).unwrap();
    let mut out = vec![hash_f64_bits(&g.cell_volume), hash_f64_bits(&g.cell_centre.concat()), hash_f64_bits(&g.face_area.concat())];
    for c in fv_compute(m, &g, FV_ALL).unwrap() {
        out.push(hash_f64_bits(&c.values));
        if c.entity == FvEntity::Face {
            out.push(hash_f64_bits(&face_to_cell(m, &c.values, Reduce::Max)));
        }
    }
    out
}

#[test]
fn every_fv_column_is_bit_identical_for_1_to_4_threads_on_all_fixtures() {
    for fx in all_mesh_fixtures().unwrap() {
        let base = with_threads(1, || fingerprint(&fx.mesh));
        for t in 2..=4 {
            assert_eq!(base, with_threads(t, || fingerprint(&fx.mesh)), "{} with {t} threads", fx.name);
        }
    }
    let m = sheared_pair();
    check(2, 4, |t| with_threads(t, || fingerprint(&m))).unwrap();
}

#[test]
fn perturbed_block_is_bit_identical_across_runs_and_thread_counts() {
    let m = perturbed_block(20, 0.3); // 8000 cells, several reduction chunks
    check(2, 4, |t| with_threads(t, || fingerprint(&m))).unwrap();
}

fn close(a: f64, b: f64) -> bool {
    (a.is_nan() && b.is_nan()) || (a - b).abs() <= 1e-9 * a.abs().max(b.abs()).max(1.0)
}

fn transformed(m: &sm_mesh_api::reference::ReferenceMesh, scale: f64, rotate: bool) -> Tampered<'_> {
    let mut t = Tampered::of(m);
    // Proper rotation (Rodrigues) about the axis (1, 2, 3) plus a translation.
    let (ax, ang) = ([1.0 / 14f64.sqrt(), 2.0 / 14f64.sqrt(), 3.0 / 14f64.sqrt()], 0.7f64);
    let (s, c) = ang.sin_cos();
    for i in 0..t.x.len() {
        let p = [t.x[i], t.y[i], t.z[i]];
        let q = if rotate {
            let cr = [ax[1] * p[2] - ax[2] * p[1], ax[2] * p[0] - ax[0] * p[2], ax[0] * p[1] - ax[1] * p[0]];
            let d = dot(ax, p);
            [0, 1, 2].map(|k| p[k] * c + cr[k] * s + ax[k] * d * (1.0 - c) + [10.0, -3.0, 7.0][k])
        } else {
            p
        };
        (t.x[i], t.y[i], t.z[i]) = (q[0] * scale, q[1] * scale, q[2] * scale);
    }
    t
}

#[test]
fn rigid_motion_and_uniform_scale_leave_every_fv_metric_unchanged() {
    let mut meshes: Vec<(String, sm_mesh_api::reference::ReferenceMesh)> =
        all_mesh_fixtures().unwrap().into_iter().map(|f| (f.name.to_string(), f.mesh)).collect();
    meshes.push(("perturbed".into(), perturbed_block(4, 0.3)));
    meshes.push(("sheared".into(), sheared_pair()));
    for (name, m) in &meshes {
        let g0 = fv_geometry(m).unwrap();
        let c0 = fv_compute(m, &g0, FV_ALL).unwrap();
        for (scale, rotate) in [(1.0, true), (3.7, false), (0.25, true)] {
            let t = transformed(m, scale, rotate);
            let g1 = fv_geometry(&t).unwrap();
            let c1 = fv_compute(&t, &g1, FV_ALL).unwrap();
            for (a, b) in c0.iter().zip(&c1) {
                for (i, (x, y)) in a.values.iter().zip(&b.values).enumerate() {
                    assert!(close(*x, *y), "{name} {:?} #{i} scale {scale}: {x} vs {y}", a.metric);
                }
            }
        }
    }
}
