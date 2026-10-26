//! Acceptance 1 and 2: Laplacian / centroid-area recovery of perturbed lattices.

mod common;

use common::*;
use sm_mesh_api::{MeshEntityKind, MeshRead};
use sm_smooth::{Method, Schedule};
use sm_testkit::lattice;

fn assert_vertices_bit_identical(f: &lattice::LatticeFixture, pts: &[[f64; 3]], start: &[[f64; 3]]) {
    for i in 0..f.mesh.vertex_count() {
        let c = f.mesh.classification(MeshEntityKind::Vertex, i as u32);
        if c.entity.kind() == sm_core::EntityKind::TopoVertex {
            for k in 0..3 {
                assert_eq!(pts[i][k].to_bits(), f.exact[i][k].to_bits(), "vertex {i}");
                assert_eq!(start[i][k].to_bits(), f.exact[i][k].to_bits());
            }
        }
    }
}

#[test]
fn a1_laplacian_restores_the_perturbed_hex_lattice() {
    let f = lattice::hex([4, 4, 4], [1.0; 3]).expect("lattice");
    let p = f.perturbed(7, 0.25).expect("perturb");
    let start = pts_of(&p.mesh);
    assert!(max_dev(&start, &p.exact) > 0.05 * p.h, "the jitter moved nodes");
    for sched in [Schedule::Colored, Schedule::Jacobi] {
        let o = run(&p, &req(Method::Laplacian { relax: 1.0 }, sched, 150, false));
        let dev = max_dev(&o.points, &p.exact);
        assert!(dev <= 1e-9 * p.h, "{sched:?}: deviation {dev:e} (h = {})", p.h);
        assert_vertices_bit_identical(&p, &o.points, &start);
        eprintln!("a1 {sched:?}: iterations {} dev {dev:e} rej_guard {} rej_geom {}", o.report.iterations_run, o.report.rejected_guard, o.report.rejected_geometry);
    }
}
