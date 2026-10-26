//! S1: informational timing of a large dualization (run with `--release -- --ignored --nocapture`).
#![allow(missing_docs)]

mod common;

use common::*;
use sm_mesh_api::MeshRead;
use sm_poly::{PolyOptions, VertexClassResolver};
use sm_testkit::lattice;
use std::time::Instant;

fn peak_rss_mb() -> f64 {
    std::fs::read_to_string("/proc/self/status")
        .ok()
        .and_then(|s| s.lines().find(|l| l.starts_with("VmHWM")).and_then(|l| l.split_whitespace().nth(1).and_then(|v| v.parse::<f64>().ok())))
        .map_or(f64::NAN, |kb| kb / 1024.0)
}

#[test]
#[ignore = "informational timing, run with --release -- --ignored --nocapture"]
fn s1_dualize_384k_tets() {
    let t = Instant::now();
    let lf = lattice::tet_kuhn([40, 40, 40], [40.0; 3]).unwrap();
    println!("fixture: {:.2?}  cells {}  rss {:.0} MB", t.elapsed(), lf.mesh.cell_count(), peak_rss_mb());
    for mode in [sm_poly::FaceMode::Planar, sm_poly::FaceMode::Keep] {
        // reset the high-water mark so the figure is the dualization's own peak (source included)
        let _ = std::fs::write("/proc/self/clear_refs", "5");
        let base = peak_rss_mb();
        let t = Instant::now();
        let (m, rep) = dual_prod(&lf.mesh, &PolyOptions::new().with_faces(mode), &VertexClassResolver).unwrap();
        println!(
            "{mode:?}: {:.2?}  cells {}  faces {}  points {}  vol {:.6}  peak rss {:.0} MB (before {base:.0} MB)",
            t.elapsed(),
            m.cell_count(),
            m.face_count(),
            rep.points,
            rep.total_volume,
            peak_rss_mb()
        );
    }
}
