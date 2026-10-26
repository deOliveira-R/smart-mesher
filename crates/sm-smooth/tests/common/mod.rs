//! Shared helpers for the acceptance tests.
#![allow(dead_code)]

use sm_mesh_api::{MeshEdit, MeshRead, PointIx};
use sm_core::Point3;
use sm_smooth::{Method, NodeSelect, Schedule, SmoothInput, SmoothOutcome, SmoothRequest, apply_outcome, smooth};
use sm_testkit::lattice::LatticeFixture;

pub fn input(f: &LatticeFixture) -> SmoothInput<'_> {
    SmoothInput { mesh: &f.mesh, topo: &f.topo, model: &f.model }
}

pub fn req(method: Method, schedule: Schedule, iterations: u32, fix_boundary: bool) -> SmoothRequest {
    SmoothRequest { nodes: NodeSelect::All, method, iterations, tolerance: 1e-12, schedule, fix_boundary, ..SmoothRequest::default() }
}

pub fn run(f: &LatticeFixture, r: &SmoothRequest) -> SmoothOutcome {
    smooth(&input(f), r).expect("smooth")
}

/// Largest distance between `pts` and `exact`.
pub fn max_dev(pts: &[[f64; 3]], exact: &[[f64; 3]]) -> f64 {
    pts.iter()
        .zip(exact)
        .map(|(p, e)| ((p[0] - e[0]).powi(2) + (p[1] - e[1]).powi(2) + (p[2] - e[2]).powi(2)).sqrt())
        .fold(0.0, f64::max)
}

/// A copy of the fixture mesh with the outcome applied.
pub fn applied(f: &LatticeFixture, o: &SmoothOutcome) -> sm_mesh_api::reference::ReferenceMesh {
    let mut m = f.mesh.clone();
    apply_outcome(&mut m, o).expect("apply");
    m
}

pub fn set_points(m: &mut dyn MeshEdit, pts: &[[f64; 3]]) {
    for (i, p) in pts.iter().enumerate() {
        m.set_point(PointIx::new(i), Point3::new(p[0], p[1], p[2])).expect("set_point");
    }
}

pub fn pts_of(m: &dyn MeshRead) -> Vec<[f64; 3]> {
    (0..m.vertex_count()).map(|i| {
        let p = m.point(i);
        [p.x, p.y, p.z]
    }).collect()
}
