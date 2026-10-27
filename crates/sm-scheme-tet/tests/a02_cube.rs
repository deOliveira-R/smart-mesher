//! Acceptance 2: `tri_cube(4)`.

mod common;
use common::*;
use sm_mesh_api::MeshRead;
use sm_scheme_tet::{TetOptions, tetrahedralize};
use sm_testkit::meshes::{CellSpec, build_mesh};
use sm_testkit::surf3;

#[test]
fn cube4_exact_boundary_volume_and_valid_mesh() {
    let f = surf3::tri_cube(4);
    assert_eq!((f.points.len(), f.tris.len()), (98, 192));
    let inp = input_of(&f, 0);
    let m = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    assert_eq!(m.n_input, 98);
    // First 98 points bit-identical.
    for (a, b) in m.points[..98].iter().zip(&f.points) {
        assert_eq!(a.map(f64::to_bits), b.map(f64::to_bits));
    }
    assert!(all_exact_positive(&m), "every tet strictly positive under exact arithmetic");
    let v = total_volume(&m);
    assert!((v - 1.0).abs() < 1e-12, "volume {v}");
    check_boundary(&m, &inp.tris).unwrap();
    // Build the mesh and run the structural validator.
    let cells: Vec<CellSpec> = m.tets.iter().map(|t| CellSpec::Template(sm_mesh_api::CellShape::Tet, t.iter().map(|&i| i as usize).collect())).collect();
    let mesh = build_mesh(&m.points, &cells, &mut sm_mesh_api::SerialAlloc::new()).unwrap();
    let violations = sm_mesh_api::validate(&mesh);
    assert!(violations.is_empty(), "violations: {violations:?}");
    assert_eq!(mesh.count(sm_mesh_api::MeshEntityKind::Cell), m.tets.len());
    sm_testkit::check::mesh_valid(&mesh).unwrap();
    sm_testkit::check::positive_volumes(&mesh).unwrap();
}

#[test]
fn cube4_without_optimization_is_also_exact() {
    let f = surf3::tri_cube(4);
    let inp = input_of(&f, 0);
    let m = tetrahedralize(&inp, &TetOptions { optimize: 0, ..TetOptions::default() }).unwrap();
    assert!(all_exact_positive(&m));
    assert!((total_volume(&m) - 1.0).abs() < 1e-12);
    check_boundary(&m, &inp.tris).unwrap();
}
