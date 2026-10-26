//! Acceptance 10: performance of a planar face of about 10^5 triangles. Record-only
//! (`cargo test -p sm-scheme-tri --release --test perf -- --ignored --nocapture`).

mod common;

use common::*;
use sm_mesh_api::MeshRead;
use sm_scheme_api::Sequential;
use sm_scheme_tri::cdt::{Cdt, OptimizeOptions, Pslg, RefineOptions, Segment, SegmentKind};
use sm_testkit::{loops2d, topo::planar_face};
use std::time::Instant;

fn square_pslg(n: usize) -> Pslg {
    let mut p = Pslg::default();
    let mut ring = Vec::new();
    for k in 0..n {
        ring.push([k as f64 / n as f64, 0.0]);
    }
    for k in 0..n {
        ring.push([1.0, k as f64 / n as f64]);
    }
    for k in 0..n {
        ring.push([1.0 - k as f64 / n as f64, 1.0]);
    }
    for k in 0..n {
        ring.push([0.0, 1.0 - k as f64 / n as f64]);
    }
    let m = ring.len() as u32;
    p.points = ring;
    for i in 0..m {
        p.segments.push(Segment { a: i, b: (i + 1) % m, kind: SegmentKind::Boundary });
    }
    p
}

#[test]
#[ignore = "performance record, run in release"]
fn acc10_kernel_one_hundred_thousand_triangles() {
    let n = 200;
    let p = square_pslg(n);
    let g = Flat(1.0 / n as f64);
    let t0 = Instant::now();
    let mut c = Cdt::new(&p).unwrap();
    let t1 = Instant::now();
    let rep = c.refine(&g, &RefineOptions::default());
    let t2 = Instant::now();
    c.optimize(&g, &OptimizeOptions::default());
    let t3 = Instant::now();
    let tris = c.triangles().len();
    println!(
        "kernel: {} triangles, new {:.3}s refine {:.3}s ({} inserted, {} unfixable) optimize {:.3}s, total {:.3}s",
        tris,
        (t1 - t0).as_secs_f64(),
        (t2 - t1).as_secs_f64(),
        rep.inserted,
        rep.unfixable.len(),
        (t3 - t2).as_secs_f64(),
        (t3 - t0).as_secs_f64()
    );
    assert!(tris > 80_000);
}

#[test]
#[ignore = "performance record, run in release"]
fn acc10_driver_one_hundred_thousand_triangles() {
    let fx = planar_face(&loops2d::square(), 0.0).unwrap();
    let counts: Vec<_> = fx.edges[0].iter().map(|&e| (e, 200u32)).collect();
    let t0 = Instant::now();
    let out = mesh_faces(&fx.complex, &[fx.face], &counts, "trimesh", &Sequential).unwrap();
    let dt = t0.elapsed().as_secs_f64();
    println!("driver: {} triangles in {dt:.3}s (plan + curves + trimesh + commit)", out.finished.mesh.cell_count());
}
