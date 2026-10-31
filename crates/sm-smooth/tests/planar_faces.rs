//! Regression (W3-INT row 6): smoothing must not make planar quad faces non-planar.
//! Interior column nodes of an extruded mesh move differently from their top/bottom face
//! nodes under a Laplacian, which tilts the columns and warps the side quads.

mod common;

use common::*;
use sm_mesh_api::{GeomTol, MeshRead, ViolationCode, validate_geometry};
use sm_smooth::{Method, Schedule};
use sm_testkit::lattice;

/// A hex lattice whose interior columns are shifted coherently in x/y: every face stays planar.
fn sheared_columns() -> lattice::LatticeFixture {
    let mut f = lattice::hex([4, 4, 4], [4.0, 4.0, 4.0]).expect("hex");
    let mut pts = pts_of(&f.mesh);
    for p in pts.iter_mut() {
        let (i, j) = (p[0].round() as i64, p[1].round() as i64);
        if (1..=3).contains(&i) && (1..=3).contains(&j) {
            p[0] += 0.12 * ((i * j) % 3) as f64;
            p[1] += 0.09 * ((i + j) % 2) as f64;
        }
    }
    set_points(&mut f.mesh, &pts);
    f
}

fn planar_violations(m: &dyn MeshRead) -> usize {
    validate_geometry(m, &GeomTol::default()).iter().filter(|v| v.code == ViolationCode::GeomPlanar).count()
}

#[test]
fn smoothing_keeps_planar_faces_planar() {
    let f = sheared_columns();
    assert_eq!(planar_violations(&f.mesh), 0, "fixture starts planar");
    for fix_boundary in [false, true] {
        for method in [Method::Laplacian { relax: 1.0 }, Method::CentroidArea { relax: 1.0 }] {
            for schedule in [Schedule::Colored, Schedule::Jacobi] {
                let o = run(&f, &req(method, schedule, 5, fix_boundary));
                let m = applied(&f, &o);
                assert_eq!(planar_violations(&m), 0, "{method:?} {schedule:?} fix_boundary={fix_boundary}");
                assert!(validate_geometry(&m, &GeomTol::default()).is_empty());
            }
        }
    }
}
