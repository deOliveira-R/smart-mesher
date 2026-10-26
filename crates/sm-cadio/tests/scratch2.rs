use sm_cadio::*;
use sm_topo_api::{CellComplex, Dim, OpId, TopologyEdit, TopologyView, TopologyViewExt};

#[test]
fn dump() {
    let b = std::fs::read(format!("{}/tests/data/sphere_seam_r3.stp", env!("CARGO_MANIFEST_DIR"))).unwrap();
    let m = read_step(&b, &ReadOptions::default()).unwrap();
    println!("{:#?}", m.solids[0].edges);
    println!("{:#?}", m.solids[0].faces);
    let mut t = CellComplex::new();
    t.begin_op(OpId(1)).unwrap();
    let r = realize(&m, &mut t, &RealizeOptions::default()).unwrap();
    t.end_op().unwrap();
    for f in t.cells(Dim::Face) {
        for lp in t.loops(*f) {
            for k in lp.keys() {
                println!("{:?} flags {:?} pc {:?}", k, t.flags(k.edge), t.pcurve(k));
            }
        }
        println!("carrier {:?}", t.carrier(*f));
    }
    println!("{:?}", r);
}
