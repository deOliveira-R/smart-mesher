//! W3b metrics (equiangle skew, mean ratio, inradius, squish, quad radius ratio, nodal Jacobian
//! ratio, wedge max stretch, FV equiangle skewness, cell aspect ratio, boundary-face flag)
//! against closed-form values.
#![allow(missing_docs)]

mod common;
use common::*;
use sm_mesh_api::reference::hex_block;
use sm_mesh_api::{CellShape, MeshRead, SerialAlloc};
use sm_quality::*;

const TOL: f64 = 1e-12;

fn ev(m: Metric, s: CellShape, p: &[V]) -> f64 {
    eval(m, s, p, &Context::default()).unwrap()
}
fn close(got: f64, want: f64) {
    assert!((got - want).abs() < TOL, "got {got}, want {want}");
}
fn corner_tet() -> Vec<V> {
    vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
}
fn sheared(p: &[V], s: f64) -> Vec<V> {
    p.iter().map(|q| [q[0] + s * q[1], q[1], q[2]]).collect()
}

#[test]
fn equiangle_skew_closed_forms() {
    for (shape, p) in [(CellShape::Tri, tri()), (CellShape::Quad, quad()), (CellShape::Tet, tet()), (CellShape::Hex, hex())] {
        close(ev(Metric::EquiangleSkew, shape, &p), 0.0);
    }
    // Right isosceles triangle: angles 45, 45, 90.
    close(ev(Metric::EquiangleSkew, CellShape::Tri, &[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]), 0.25);
    // Sheared square / cube: angles atan2(1, s) and its supplement.
    for s in [1.0, 0.5, 2.0] {
        let want = (90.0 - 1.0f64.atan2(s).to_degrees()) / 90.0;
        close(ev(Metric::EquiangleSkew, CellShape::Quad, &sheared(&quad(), s)), want);
        close(ev(Metric::EquiangleSkew, CellShape::Hex, &sheared(&hex(), s)), want);
    }
    // Corner tetrahedron: dihedrals 90 (x3) and acos(1/sqrt3) (x3).
    let ae = (1.0f64 / 3.0).acos().to_degrees();
    let lo = (1.0f64 / 3.0f64.sqrt()).acos().to_degrees();
    let want = ((90.0 - ae) / (180.0 - ae)).max((ae - lo) / ae);
    close(ev(Metric::EquiangleSkew, CellShape::Tet, &corner_tet()), want);
}

#[test]
fn tet_mean_ratio_inradius_squish() {
    close(ev(Metric::MeanRatio, CellShape::Tet, &tet()), 1.0);
    close(ev(Metric::MeanRatio, CellShape::Tet, &corner_tet()), 12.0 * 0.5f64.powf(2.0 / 3.0) / 9.0);
    close(ev(Metric::Inradius, CellShape::Tet, &tet()), 6.0f64.sqrt() / 12.0);
    close(ev(Metric::Inradius, CellShape::Tet, &corner_tet()), 0.5 / (1.5 + 3.0f64.sqrt() / 2.0));
    close(ev(Metric::SquishIndex, CellShape::Tet, &tet()), 0.0);
    close(ev(Metric::SquishIndex, CellShape::Tet, &corner_tet()), 1.0 - 3.0 / 11.0f64.sqrt());
    // Inverted tet: mean ratio 0, inradius negative.
    let mut inv = tet();
    inv.swap(1, 2);
    close(ev(Metric::MeanRatio, CellShape::Tet, &inv), 0.0);
    assert!(ev(Metric::Inradius, CellShape::Tet, &inv) < 0.0);
}

#[test]
fn inradius_scales_with_length() {
    let big: Vec<V> = tet().iter().map(|q| [q[0] * 1e6, q[1] * 1e6, q[2] * 1e6]).collect();
    let r = ev(Metric::Inradius, CellShape::Tet, &big);
    assert!((r / 1e6 - 6.0f64.sqrt() / 12.0).abs() < 1e-12, "{r}");
}

#[test]
fn quad_radius_ratio_closed_forms() {
    close(ev(Metric::RadiusRatio, CellShape::Quad, &quad()), 1.0);
    let rect: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [1., 2., 0.], [0., 2., 0.]];
    close(ev(Metric::RadiusRatio, CellShape::Quad, &rect), 3.0 * 5.0f64.sqrt() / (2.0 * 2.0f64.sqrt() * 2.0));
    // Degenerate (zero area) gives the sentinel.
    let flat: Vec<V> = vec![[0., 0., 0.], [1., 0., 0.], [2., 0., 0.], [3., 0., 0.]];
    assert_eq!(ev(Metric::RadiusRatio, CellShape::Quad, &flat), DBL_MAX);
}

#[test]
fn hex_nodal_jacobian_ratio_closed_forms() {
    close(ev(Metric::NodalJacobianRatio, CellShape::Hex, &hex()), 1.0);
    // Shear keeps every corner determinant equal to 1.
    close(ev(Metric::NodalJacobianRatio, CellShape::Hex, &sheared(&hex(), 0.7)), 1.0);
    // Frustum, top square of side k about the centre: top corner determinants are k^2.
    for k in [0.5, 0.25] {
        let o = 0.5 * (1.0 - k);
        let mut p = hex();
        for (i, q) in p.iter_mut().enumerate() {
            if i >= 4 {
                *q = [o + k * q[0], o + k * q[1], q[2]];
            }
        }
        close(ev(Metric::NodalJacobianRatio, CellShape::Hex, &p), k * k);
    }
}

#[test]
fn wedge_max_stretch_closed_forms() {
    close(ev(Metric::MaxStretch, CellShape::Wedge, &wedge()), 1.0);
    for h in [2.0, 0.5] {
        let p: Vec<V> = wedge().iter().map(|q| [q[0], q[1], q[2] * h]).collect();
        let (l, d) = (1.0f64.min(h), (1.0 + h * h).sqrt());
        close(ev(Metric::MaxStretch, CellShape::Wedge, &p), d / (2.0f64.sqrt() * l));
    }
}

#[test]
fn applicability_matrix() {
    use CellShape as S;
    let skew = [S::Tri, S::Quad, S::Tet, S::Hex];
    for s in [S::Tri, S::Quad, S::Tet, S::Hex, S::Wedge, S::Pyramid, S::Polygon, S::Polyhedron] {
        assert_eq!(Metric::EquiangleSkew.applies(s), skew.contains(&s), "{s:?}");
        assert_eq!(Metric::MeanRatio.applies(s), s == S::Tet);
        assert_eq!(Metric::Inradius.applies(s), s == S::Tet);
        assert_eq!(Metric::SquishIndex.applies(s), s == S::Tet);
        assert_eq!(Metric::NodalJacobianRatio.applies(s), s == S::Hex);
        assert_eq!(Metric::MaxStretch.applies(s), s == S::Wedge);
    }
    assert!(Metric::RadiusRatio.applies(S::Quad));
}

#[test]
fn fv_cell_aspect_ratio_and_boundary_count() {
    let m = hex_block([1, 1, 1], [1.0, 1.0, 1.0], &mut SerialAlloc::new()).unwrap();
    let g = fv_geometry(&m).unwrap();
    let cols = fv_compute(&m, &g, &[FvMetric::CellAspectRatio, FvMetric::EquiangleSkewness, FvMetric::BoundaryFaceCount]).unwrap();
    close(cols[0].values[0], 1.0);
    assert!(cols[1].values.iter().all(|v| v.abs() < TOL));
    assert_eq!(cols[2].values[0], 6.0);
    // 2 x 1 x 1 brick: sums of |Sf| are (2, 4, 4).
    let b = hex_block([1, 1, 1], [2.0, 1.0, 1.0], &mut SerialAlloc::new()).unwrap();
    let g = fv_geometry(&b).unwrap();
    let c = fv_compute(&b, &g, &[FvMetric::CellAspectRatio]).unwrap();
    close(c[0].values[0], 2.0);
}

#[test]
fn fv_equiangle_skewness_parallelogram_face() {
    // Sheared cube: the z faces are parallelograms with edges (1,0,0) and (s,1,0); the other
    // faces are rectangles.
    let s = 2.0;
    let mut m = hex_block([1, 1, 1], [1.0, 1.0, 1.0], &mut SerialAlloc::new()).unwrap();
    use sm_core::Point3;
    use sm_mesh_api::{MeshEdit, PointIx};
    for i in 0..m.vertex_count() {
        let p = m.point(i);
        m.set_point(PointIx::new(i), Point3::new(p.x + s * p.y, p.y, p.z)).unwrap();
    }
    let g = fv_geometry(&m).unwrap();
    let c = fv_compute(&m, &g, &[FvMetric::EquiangleSkewness]).unwrap();
    let want = (90.0 - 1.0f64.atan2(s).to_degrees()) / 90.0;
    let max = c[0].values.iter().cloned().fold(0.0, f64::max);
    close(max, want);
    let min = c[0].values.iter().cloned().fold(f64::INFINITY, f64::min);
    close(min, 0.0);
}

fn count_ge2(m: &dyn MeshRead) -> (usize, usize) {
    let g = fv_geometry(m).unwrap();
    let c = fv_compute(m, &g, &[FvMetric::BoundaryFaceCount]).unwrap();
    let rule = transport_extra_rules().iter().find(|r| r.measure == Measure::Fv(FvMetric::BoundaryFaceCount)).unwrap();
    let by_value = c[0].values.iter().filter(|&&v| v >= 2.0).count();
    let by_rule = c[0].values.iter().filter(|&&v| rule.grade(v, false).reaches(Grade::Warn)).count();
    (by_value, by_rule)
}

#[test]
fn transport_flags_cells_with_two_boundary_faces() {
    let slab = hex_block([3, 3, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    // Every slab cell has a top and a bottom boundary face.
    assert_eq!(count_ge2(&slab), (9, 9));
    let cube = hex_block([3, 3, 3], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    // 8 corners + 12 edges; the 6 face cells and the centre have fewer than two.
    assert_eq!(count_ge2(&cube), (20, 20));
}

#[test]
fn cell_aspect_ratio_is_axis_dependent_and_scale_free() {
    use sm_core::Point3;
    use sm_mesh_api::{MeshEdit, PointIx};
    let mut m = hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    for i in 0..m.vertex_count() {
        let p = m.point(i);
        // Rotate 45 degrees about z, scale by 3.
        let c = std::f64::consts::FRAC_1_SQRT_2;
        m.set_point(PointIx::new(i), Point3::new(3.0 * c * (p.x - p.y), 3.0 * c * (p.x + p.y), 3.0 * p.z)).unwrap();
    }
    let g = fv_geometry(&m).unwrap();
    let c = fv_compute(&m, &g, &[FvMetric::CellAspectRatio]).unwrap();
    // Lateral faces contribute 4 * cos45 * A to x and y, the caps 2 A to z.
    close(c[0].values[0], 2.0 * std::f64::consts::SQRT_2 / 2.0);
}

#[test]
fn transport_rules_table_is_unchanged() {
    let names: Vec<&str> = rules(Profile::Transport).iter().map(|r| r.measure.name()).collect();
    assert_eq!(names, ["face_flatness", "face_concavity", "starness", "edge_ratio", "volume_ratio"]);
}
