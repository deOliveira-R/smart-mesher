use super::*;
use sm_testkit::meshes::{hex_block_two_volumes, mixed_transition};

/// With `SM_MSH_OUT=<dir>` the samples are written there for external readers (gmsh, meshio).
#[test]
fn smoke_and_dump() {
    let f = hex_block_two_volumes([2, 2, 2], [1.0; 3], 1).unwrap();
    let m = mixed_transition().unwrap();
    for (name, mesh) in [("hex2", &f.mesh), ("mixed", &m.mesh)] {
        for binary in [false, true] {
            let mut o = GmshOptions::default();
            o.binary = binary;
            let mut out = Vec::new();
            let r = write_msh4(mesh, &o, &mut out).unwrap();
            let p = parse_msh4(&out).unwrap();
            assert_eq!(p.node_count() as u64, r.nodes);
            assert_eq!(p.elements_by_type(), r.elements_by_type);
            if let Ok(dir) = std::env::var("SM_MSH_OUT") {
                std::fs::write(format!("{dir}/{name}{}.msh", if binary { "_bin" } else { "" }), &out).unwrap();
            }
        }
    }
}
