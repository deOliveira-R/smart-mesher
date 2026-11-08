//! Acceptance 10: a lone closed solid with curved faces (cylinder, sphere) is tet-meshed through
//! the driver with the real curve and triangle schemes. The mesh is valid, and the total tet
//! volume equals the volume enclosed by its boundary triangulation (divergence theorem).

use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh};
use sm_mesh_api::{CellIx, CellShape, FaceIx, MeshEntityKind, MeshRead, SerialAlloc, cell_volume, validate};
use sm_scheme_api::{
    Driver, MeshInput, MeshIntent, ReferenceCornerPicker, ReferenceIntervalSolver, SchemeRegistry, Sequential,
    SizingSpec, WholeFaces,
};
use sm_topo_api::fixtures::{cylinder, sphere, with_op};
use sm_topo_api::{CellComplex, OpId};

fn registry() -> SchemeRegistry {
    let mut r = SchemeRegistry::new();
    sm_scheme_curve::register(&mut r).unwrap();
    sm_scheme_tri::register(&mut r).unwrap();
    sm_scheme_tet::register(&mut r).unwrap();
    r
}

/// Returns `(n_tets, sum of tet volumes, volume enclosed by the boundary triangles)`.
fn mesh_volumes(topo: &CellComplex, vol: sm_core::EntityId, h: f64) -> (usize, f64, f64) {
    let model = sm_model_api::Model::new();
    let mut intent = MeshIntent::default();
    intent.defaults.sizing = Some(SizingSpec::Constant { size: h });
    let reg = registry();
    let (solver, picker) = (ReferenceIntervalSolver::default(), ReferenceCornerPicker);
    let driver = Driver::new(&reg, &solver, &picker);
    let sel = [vol];
    let input = MeshInput::new(topo, &model, &WholeFaces, &sel);
    let out = driver
        .mesh(&input, &intent, &Sequential, ReferenceBuilder::new(), &mut SerialAlloc::new())
        .expect("lone closed solid meshes");
    let m: &ReferenceMesh = &out.finished.mesh;
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    sm_testkit::check::mesh_valid(m).unwrap();

    let n = m.count(MeshEntityKind::Cell);
    let mut sum = 0.0;
    for c in 0..n {
        let c = CellIx::new(c);
        assert_eq!(m.cell_shape(c), CellShape::Tet);
        let v = cell_volume(m, c);
        assert!(v > 0.0, "inverted or flat tet {c:?}");
        sum += v;
    }
    // Boundary facets carry their loop with the outward (left-cell) normal.
    let p = |v: sm_mesh_api::VertexIx| m.point(v.get());
    let mut enclosed = 0.0;
    for f in 0..m.count(MeshEntityKind::Face) {
        let f = FaceIx::new(f);
        if m.face_right(f).is_none() {
            let l = m.face_vertices(f);
            assert_eq!(l.len(), 3);
            let (a, b, c) = (p(l[0]), p(l[1]), p(l[2]));
            let (a, b, c) = ([a.x, a.y, a.z], [b.x, b.y, b.z], [c.x, c.y, c.z]);
            let cx = [b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]];
            enclosed += (a[0] * cx[0] + a[1] * cx[1] + a[2] * cx[2]) / 6.0;
        }
    }
    (n, sum, enclosed)
}

#[test]
fn lone_cylinder_volume_matches_its_boundary_triangulation() {
    let (r, h) = (0.4, 1.0);
    let mut topo = CellComplex::new();
    let c = with_op(&mut topo, OpId(1), |t| cylinder(t, r, h)).unwrap();
    let (n, sum, enclosed) = mesh_volumes(&topo, c.volume, 0.2);
    assert!(n > 100, "{n} tets");
    assert!((sum - enclosed).abs() <= 1e-12 * enclosed.abs(), "sum {sum} vs boundary {enclosed}");
    // A polygonal prism inscribed in the circle lies below the analytic volume.
    let exact = std::f64::consts::PI * r * r * h;
    assert!(sum < exact && sum > 0.9 * exact, "volume {sum} vs pi r^2 h = {exact}");
}

#[test]
fn lone_sphere_volume_matches_its_boundary_triangulation() {
    let mut topo = CellComplex::new();
    let s = with_op(&mut topo, OpId(1), |t| sphere(t, 1.0)).unwrap();
    let (n, sum, enclosed) = mesh_volumes(&topo, s.volume, 0.3);
    assert!(n > 100, "{n} tets");
    assert!((sum - enclosed).abs() <= 1e-12 * enclosed.abs(), "sum {sum} vs boundary {enclosed}");
    let exact = 4.0 / 3.0 * std::f64::consts::PI;
    assert!(sum < exact && sum > 0.9 * exact, "volume {sum} vs 4 pi/3 = {exact}");
}
