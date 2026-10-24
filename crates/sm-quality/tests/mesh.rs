//! Mesh-level drivers against sm-testkit fixtures.
#![allow(missing_docs)]

mod common;
use common::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellIx, CellShape, MeshRead, SerialAlloc, cell_volume};
use sm_quality::*;
use sm_testkit::check;
use sm_testkit::meshes::*;

fn opts() -> Options<'static> {
    Options::default()
}

#[test]
fn volumes_match_fixture_exact_values_and_mesh_geometry_rule() {
    for fx in all_mesh_fixtures().unwrap() {
        let v = compute(&fx.mesh, Metric::Volume, &opts()).unwrap();
        assert_eq!(v.len(), fx.mesh.cell_count());
        for (c, (&got, &want)) in v.iter().zip(&fx.cell_volumes).enumerate() {
            assert!((got - want).abs() <= 1e-12 * want.max(1.0), "{} cell {c}: {got} vs {want}", fx.name);
            let g = cell_volume(&fx.mesh, CellIx::new(c));
            assert!((got - g).abs() <= 1e-12 * g.max(1.0), "{} cell {c}: {got} vs geometry rule {g}", fx.name);
        }
        let val = validity(&fx.mesh, &opts()).unwrap();
        assert!(val.all_valid(), "{}: {:?}", fx.name, val.status);
        assert_eq!(val.n_valid, fx.mesh.cell_count());
        check::mesh_valid(&fx.mesh).unwrap();
    }
}

#[test]
fn polyhedral_metrics_on_fixtures() {
    // Regular-ish hexagonal prism: 8 faces, dihedrals 90 (top/bottom) and 120 (sides).
    let fx = hexagonal_prism(1.0, 1.0).unwrap();
    assert_eq!(fx.mesh.cell_shape(CellIx::new(0)), CellShape::Polyhedron);
    let t = compute_many(&fx.mesh, &[Metric::MinAngle, Metric::MaxAngle, Metric::EdgeRatio, Metric::Sphericity, Metric::Starness], &opts()).unwrap();
    let col = |m| t.column(m).unwrap()[0];
    assert!((col(Metric::MinAngle) - 90.0).abs() < 1e-9);
    assert!((col(Metric::MaxAngle) - 120.0).abs() < 1e-9);
    assert!((col(Metric::EdgeRatio) - 1.0).abs() < 1e-12);
    // Wadell sphericity: pi^(1/3) (6V)^(2/3) / S, V = 3 sqrt3/2, S = 2 * 3sqrt3/2 + 6.
    let (v, s) = (fx.volume, fx.area);
    let want = std::f64::consts::PI.cbrt() * (6.0 * v).powf(2.0 / 3.0) / s;
    assert!((col(Metric::Sphericity) - want).abs() < 1e-12);
    assert!(col(Metric::Starness) > 0.5);
    // Metrics undefined for polyhedra are an error, not NaN.
    assert!(matches!(compute(&fx.mesh, Metric::Skew, &opts()), Err(QualityError::Unsupported { .. })));
}

#[test]
fn single_element_meshes_match_element_kernels() {
    let tet = single_tet().unwrap();
    let sj = compute(&tet.mesh, Metric::ScaledJacobian, &opts()).unwrap();
    assert!((sj[0] - 0.5f64.sqrt()).abs() < 1e-14);
    let hex = single_hex().unwrap();
    let r = report(&hex.mesh, &[Metric::ScaledJacobian, Metric::Shape, Metric::Oddy], &opts()).unwrap();
    assert!((r[0].1.mean - 1.0).abs() < 1e-14 && (r[1].1.mean - 1.0).abs() < 1e-14 && r[2].1.mean.abs() < 1e-14);
    assert_eq!(r[0].1.count, 1);
}

#[test]
fn mixed_mesh_marks_inapplicable_cells_nan() {
    let fx = mixed_transition().unwrap();
    let m = &fx.mesh;
    let skew = compute(m, Metric::Skew, &opts()).unwrap();
    for (c, x) in skew.iter().enumerate() {
        let hex = m.cell_shape(CellIx::new(c)) == CellShape::Hex;
        assert_eq!(x.is_nan(), !hex, "cell {c}");
    }
    let s = summarize(&skew);
    assert_eq!(s.count + s.undefined, m.cell_count());
    // Only tets have a collapse ratio.
    assert!(compute(m, Metric::CollapseRatio, &opts()).is_ok());
    assert!(matches!(compute(&single_hex().unwrap().mesh, Metric::CollapseRatio, &opts()), Err(QualityError::Unsupported { metric: Metric::CollapseRatio })));
}

#[test]
fn mirrored_cells_are_inverted() {
    let pts: Vec<[f64; 3]> = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]]
        .iter()
        .map(|p| [-p[0], p[1], p[2]])
        .collect();
    let m = build_mesh(&pts, &[CellSpec::Template(CellShape::Hex, (0..8).collect())], &mut SerialAlloc::new()).unwrap();
    let v = validity(&m, &opts()).unwrap();
    assert_eq!(v.n_inverted, 1);
    assert_eq!(v.inverted(), vec![CellIx::new(0)]);
    assert!(!v.all_valid());
    let sj = compute(&m, Metric::ScaledJacobian, &opts()).unwrap();
    assert!((sj[0] + 1.0).abs() < 1e-14);
}

#[test]
fn collapsed_cells_are_degenerate() {
    let pts = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0.25, 0.25, 0.0]];
    let m = build_mesh(&pts, &[CellSpec::Template(CellShape::Tet, vec![0, 1, 2, 3])], &mut SerialAlloc::new()).unwrap();
    let v = validity(&m, &opts()).unwrap();
    assert_eq!(v.status[0], CellStatus::Degenerate);
}

#[test]
fn surface_meshes_use_reference_normals() {
    let pts = [[0., 0.], [2., 0.], [2., 1.], [0., 1.], [3., 0.5]];
    let m = surface_mesh(&pts, &[vec![0, 1, 2, 3], vec![1, 4, 2]]);
    let a = compute(&m, Metric::Volume, &opts()).unwrap();
    assert!((a[0] - 2.0).abs() < 1e-14 && (a[1] - 0.5).abs() < 1e-14);
    // Valid against +z and against its own normal; fully inverted against -z.
    let up = Options::default().with_normals(SurfaceNormals::Uniform([0., 0., 1.]));
    let down = Options::default().with_normals(SurfaceNormals::Uniform([0., 0., -1.]));
    assert!(validity(&m, &opts()).unwrap().all_valid());
    assert!(validity(&m, &up).unwrap().all_valid());
    assert_eq!(validity(&m, &down).unwrap().n_inverted, 2);
    let per = [[0., 0., 1.], [0., 0., -1.]];
    let v = validity(&m, &Options::default().with_normals(SurfaceNormals::PerCell(&per))).unwrap();
    assert_eq!((v.n_valid, v.n_inverted), (1, 1));
    assert!(matches!(
        compute(&m, Metric::Volume, &Options::default().with_normals(SurfaceNormals::PerCell(&per[..1]))),
        Err(QualityError::BadArgument(_))
    ));
    let ang = compute(&m, Metric::MaxAngle, &opts()).unwrap();
    assert!((ang[0] - 90.0).abs() < 1e-12);
}

#[test]
fn polygon_cells_in_a_surface_mesh() {
    // Regular hexagon (circumradius 1) next to a triangle sharing an edge.
    let h = 3f64.sqrt() / 2.0;
    let pts = [[1., 0.], [0.5, h], [-0.5, h], [-1., 0.], [-0.5, -h], [0.5, -h], [2., 0.]];
    let m = surface_mesh(&pts, &[vec![0, 1, 2, 3, 4, 5], vec![0, 6, 1]]);
    assert_eq!(m.cell_shape(CellIx::new(0)), CellShape::Polygon);
    let t = compute_many(&m, &[Metric::Volume, Metric::AspectRatio, Metric::MinAngle, Metric::EdgeRatio], &opts()).unwrap();
    assert!((t.column(Metric::Volume).unwrap()[0] - 3.0 * 3f64.sqrt() / 2.0).abs() < 1e-14);
    assert!((t.column(Metric::AspectRatio).unwrap()[0] - 1.0).abs() < 1e-14);
    assert!((t.column(Metric::MinAngle).unwrap()[0] - 120.0).abs() < 1e-12);
    // Skew is undefined for both cells of this mesh.
    assert!(matches!(compute(&m, Metric::Skew, &opts()), Err(QualityError::Unsupported { .. })));
}

#[test]
fn inactive_cells_are_skipped_unless_requested() {
    let m = hex_block([2, 1, 1], [1.0, 1.0, 1.0], &mut SerialAlloc::new()).unwrap();
    let w = WithInactive(&m, vec![0]);
    let v = compute(&w, Metric::Volume, &opts()).unwrap();
    assert!(v[0].is_nan() && (v[1] - 1.0).abs() < 1e-14);
    let s = summarize(&v);
    assert_eq!((s.count, s.undefined), (1, 1));
    let all = compute(&w, Metric::Volume, &Options::default().with_inactive(true)).unwrap();
    assert!(all.iter().all(|x| (x - 1.0).abs() < 1e-14));
    let val = validity(&w, &opts()).unwrap();
    assert_eq!((val.n_skipped, val.n_valid), (1, 1));
}

#[test]
fn relative_size_uses_the_mesh_mean() {
    // Two cells of volume 1 and 3: mean 2 -> tau = 1/2 and 3/2 -> min(tau, 1/tau)^2 = 1/4, 4/9.
    let p2: Vec<[f64; 3]> = vec![
        [0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.],
        [4., 0., 0.], [4., 1., 0.], [4., 0., 1.], [4., 1., 1.],
    ];
    let cells = [
        CellSpec::Template(CellShape::Hex, (0..8).collect()),
        CellSpec::Template(CellShape::Hex, vec![1, 8, 9, 2, 5, 10, 11, 6]),
    ];
    let m = build_mesh(&p2, &cells, &mut SerialAlloc::new()).unwrap();
    let rs = compute(&m, Metric::RelativeSizeSquared, &opts()).unwrap();
    assert!((rs[0] - 0.25).abs() < 1e-14 && (rs[1] - 4.0 / 9.0).abs() < 1e-14, "{rs:?}");
    let fixed = compute(&m, Metric::RelativeSizeSquared, &Options::default().with_reference_size(ReferenceSize::Fixed(1.0))).unwrap();
    assert!((fixed[0] - 1.0).abs() < 1e-14 && (fixed[1] - 1.0 / 9.0).abs() < 1e-14);
    let ss = compute(&m, Metric::ShapeAndSize, &opts()).unwrap();
    assert!((ss[0] - 0.25).abs() < 1e-14);
}

struct Down;
impl sm_mesh_api::FaceNormals for Down {
    fn normal(&self, _: sm_core::EntityId, _: sm_core::Point3) -> Option<sm_core::Vector3> {
        Some(sm_core::Vector3::new(0.0, 0.0, -1.0))
    }
}
struct NoCarrier;
impl sm_mesh_api::FaceNormals for NoCarrier {
    fn normal(&self, _: sm_core::EntityId, _: sm_core::Point3) -> Option<sm_core::Vector3> {
        None
    }
}

#[test]
fn face_normals_field_is_the_shell_orientation_rule() {
    let m = surface_mesh(&[[0., 0.], [1., 0.], [1., 1.], [0., 1.]], &[vec![0, 1, 2, 3]]);
    let down = Options::default().with_normals(SurfaceNormals::Field(&Down));
    assert_eq!(validity(&m, &down).unwrap().n_inverted, 1);
    let none = Options::default().with_normals(SurfaceNormals::Field(&NoCarrier));
    assert!(validity(&m, &none).unwrap().all_valid());
}

#[test]
fn zero_or_non_finite_reference_normals_are_rejected() {
    let m = surface_mesh(&[[0., 0.], [2., 0.], [2., 1.], [0., 1.]], &[vec![0, 1, 2, 3]]);
    let bad_uniform = [[0., 0., 0.], [f64::NAN, 0., 1.], [0., f64::INFINITY, 1.]];
    for n in bad_uniform {
        let o = Options::default().with_normals(SurfaceNormals::Uniform(n));
        assert!(matches!(compute(&m, Metric::Volume, &o), Err(QualityError::BadArgument(_))), "{n:?}");
    }
    let per = [[0., 0., 0.]];
    let o = Options::default().with_normals(SurfaceNormals::PerCell(&per));
    assert!(matches!(compute(&m, Metric::Volume, &o), Err(QualityError::BadArgument(_))));
    assert!(matches!(validity(&m, &o), Err(QualityError::BadArgument(_))));
}
