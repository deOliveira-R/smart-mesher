//! Cross-checks against sm-quality (L2) on meshes this crate produced.

mod common;

use common::*;
use sm_mesh_api::MeshRead;
use sm_quality::{Metric, Options, SurfaceNormals, compute, validity};
use sm_scheme_api::Sequential;
use sm_testkit::{loops2d, topo::planar_face};

#[test]
fn sm_quality_agrees_on_angles_and_finds_no_inverted_cell() {
    let fx = planar_face(&loops2d::l_shape(), 0.0).unwrap();
    let counts = counts_for(&fx, 0.1);
    let out = mesh_faces(&fx.complex, &[fx.face], &counts, "trimesh", &Sequential).unwrap();
    let m = &out.finished.mesh;
    let min_here = min_angle_deg(m);
    let col = compute(m, Metric::MinAngle, &Options::default()).unwrap();
    let min_there = col.iter().cloned().fold(f64::INFINITY, f64::min);
    assert!((min_here - min_there).abs() < 1e-9, "{min_here} vs {min_there}");
    let up = [0.0, 0.0, 1.0];
    let opts = Options::default().with_normals(SurfaceNormals::Uniform(up));
    let v = validity(m, &opts).unwrap();
    assert!(v.all_valid(), "inverted cells: {:?}", v.inverted());
    assert_eq!(col.len(), m.cell_count());
}
