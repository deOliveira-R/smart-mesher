mod common;
use common::*;
use sm_scheme_api::Sequential;
use sm_mesh_api::{MeshRead, validate, CellIx};
use sm_topo_api::fixtures::{cylinder, sphere, with_op};
use sm_topo_api::{CellComplex, OpId};

#[test]
fn cylinder_lateral_dbg() {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| cylinder(t, 1.0, 2.0)).unwrap();
    println!("{ids:?}");
    let counts = vec![(ids.edges[0], 12u32), (ids.edges[1], 12), (ids.edges[2], 6)];
    let out = mesh_faces(&c, &[ids.faces[2]], &counts, "trimesh", &Sequential);
    match out {
        Err(e) => println!("ERR {e}"),
        Ok(out) => {
            let m = &out.finished.mesh;
            println!("cells {} verts {} valid {:?} area {} want {}", m.cell_count(), m.vertex_count(), validate(m), mesh_area(m), 2.0*std::f64::consts::PI*2.0);
        }
    }
}

#[test]
fn sphere_dbg() {
    let mut c = CellComplex::new();
    let ids = with_op(&mut c, OpId(1), |t| sphere(t, 1.0)).unwrap();
    println!("{ids:?}");
    let counts = vec![(ids.edges[1], 16u32)];
    let out = mesh_faces(&c, &[ids.face], &counts, "trimesh", &Sequential);
    match out {
        Err(e) => println!("ERR {e}"),
        Ok(out) => {
            let m = &out.finished.mesh;
            println!("cells {} verts {} valid {:?} area {} want {}", m.cell_count(), m.vertex_count(), validate(m), mesh_area(m), 4.0*std::f64::consts::PI);
            let _ = CellIx::new(0);
        }
    }
}
