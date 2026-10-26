mod common;
use common::*;
use sm_scheme_api::{Sequential, Target};
use sm_testkit::{loops2d, topo::planar_face};
use sm_mesh_api::{MeshRead, validate};

#[test]
fn square_through_the_driver() {
    let fx = planar_face(&loops2d::square(), 0.0).unwrap();
    let counts: Vec<_> = fx.edges[0].iter().map(|&e| (e, 10u32)).collect();
    let out = mesh_faces(&fx.complex, &[fx.face], &counts, "trimesh", &Sequential).expect("mesh");
    let m = &out.finished.mesh;
    println!("cells {} verts {}", m.cell_count(), m.vertex_count());
    assert!(validate(m).is_empty(), "{:?}", validate(m));
    assert!((mesh_area(m) - 1.0).abs() < 1e-12);
    let _ = Target::Surface(fx.face);
}
