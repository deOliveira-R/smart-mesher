use super::*;
use sm_testkit::meshes::{hex_block_two_volumes, tet_block_two_volumes};

#[test]
fn smoke_two_volume_h5m_is_written() {
    let f = tet_block_two_volumes([4, 4, 4], [0.25; 3], 2).unwrap();
    let mut out = Vec::new();
    let r = export_h5m(&f.mesh, &DagmcOptions::default(), &mut out).unwrap();
    assert_eq!(r.volumes, 2);
    assert_eq!(r.surfaces, 11);
    if let Ok(p) = std::env::var("SM_H5M_OUT") {
        std::fs::write(p, &out).unwrap();
    }
    let h = hex_block_two_volumes([2, 2, 2], [1.0; 3], 1).unwrap();
    export_h5m(&h.mesh, &DagmcOptions::default(), &mut Vec::new()).unwrap();
}
