use sm_cadio::*;
use sm_topo_api::{CellComplex, Dim, OpId, TopologyEdit, TopologyView, validate, validate_geometry};

fn run(name: &str, bytes: &[u8]) {
    let m = match read_step(bytes, &ReadOptions::default()) {
        Ok(m) => m,
        Err(e) => {
            println!("{name}: READ ERR {e}");
            return;
        }
    };
    let mut t = CellComplex::new();
    t.begin_op(OpId(1)).unwrap();
    let r = realize(&m, &mut t, &RealizeOptions::default());
    match r {
        Ok(r) => {
            if let Err(e) = t.end_op() {
                println!("{name}: END_OP ERR {e}");
                return;
            }
            let vols: Vec<f64> = r.volumes.iter().map(|v| t.volume_moments(*v).map(|m| m.volume).unwrap_or(f64::NAN)).collect();
            let area: f64 = t.cells(Dim::Face).iter().map(|f| t.face_moments(*f).map(|m| m.area).unwrap_or(f64::NAN)).sum();
            println!("{name}: V{} E{} F{} vol {:?} area {} inverted {:?} poles {}", r.vertices.len(), r.edges.len(), r.faces.len(), vols, area, r.inverted, r.poles);
            let iss: Vec<_> = validate(&t).into_iter().chain(validate_geometry(&t)).collect();
            if !iss.is_empty() { println!("  issues: {iss:?}"); }
        }
        Err(e) => println!("{name}: REALIZE ERR {e}"),
    }
}

#[test]
fn dump() {
    for f in sm_testkit::step::ALL {
        run(f.name, f.bytes);
    }
    for f in ["brick_2x3x4", "brick_ap203", "brick_inch", "two_bricks", "brick_with_void", "brick_with_void_inward", "frustum_r2_r1_h3", "sphere_seam_r3", "block_fillet_nurbs"] {
        let b = std::fs::read(format!("{}/tests/data/{f}.stp", env!("CARGO_MANIFEST_DIR"))).unwrap();
        run(f, &b);
    }
}
