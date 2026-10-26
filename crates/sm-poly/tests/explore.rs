//! scratch
mod common;
use common::*;
use sm_mesh_api::{GeomTol, MeshEntityKind, MeshRead, validate, validate_geometry};
use sm_poly::{FaceMode, PolyOptions, VertexClassResolver};
use sm_testkit::meshes;

fn summary(name: &str, src: &dyn MeshRead, mode: FaceMode, tol: GeomTol) {
    let r = dual_ref(src, &PolyOptions::new().with_faces(mode), &VertexClassResolver);
    match r {
        Err(e) => println!("{name} {mode:?}: ERR {e}"),
        Ok((m, rep)) => {
            let v = validate(&m);
            let g = validate_geometry(&m, &tol);
            println!("{name} {mode:?}: {rep:?}\n  validate {} geom {} {:?}", v.len(), g.len(), g.iter().take(3).collect::<Vec<_>>());
            println!("  {:?}", v.iter().take(3).collect::<Vec<_>>());
            println!("  faces {} cells {}", m.count(MeshEntityKind::Face), m.count(MeshEntityKind::Cell));
        }
    }
}

#[test]
fn explore() {
    let f = meshes::tet_block([3, 3, 3], [1.0; 3]).unwrap();
    summary("block333", &f.mesh, FaceMode::Keep, GeomTol::new().with_planarity(f64::INFINITY));
    summary("block333", &f.mesh, FaceMode::Planar, GeomTol::new());
    let f = meshes::tet_block_two_volumes([4, 2, 2], [1.0; 3], 2).unwrap();
    summary("two", &f.mesh, FaceMode::Planar, GeomTol::new());
    let f = meshes::tet_block_warped([3, 3, 3], [1.0; 3], 0.1).unwrap();
    summary("warp", &f.mesh, FaceMode::Planar, GeomTol::new());
    summary("warp", &f.mesh, FaceMode::Keep, GeomTol::new().with_planarity(f64::INFINITY));
    let f = meshes::tet_lshape(2, 1.0).unwrap();
    summary("lshape", &f.mesh, FaceMode::Planar, GeomTol::new());
    summary("lshape", &f.mesh, FaceMode::Keep, GeomTol::new().with_planarity(f64::INFINITY));
}
