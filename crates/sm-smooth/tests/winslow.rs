//! Stretch 2: Winslow smoothing of the structured interior of a quad surface mesh.

#![allow(clippy::needless_range_loop)]

mod common;

use common::*;
use sm_core::Point3;
use sm_mesh_api::{MeshEdit, MeshEntityKind, MeshRead, PointIx};
use sm_smooth::{Method, Schedule};
use sm_testkit::lattice::{self, LatticeFixture};

/// Interior (face-classified) nodes jittered, everything else exactly on the lattice.
fn interior_jitter(f: &LatticeFixture, seed: u64, amp: f64) -> LatticeFixture {
    let mut p = f.perturbed(seed, amp).expect("perturb");
    for i in 0..p.mesh.vertex_count() {
        if p.mesh.classification(MeshEntityKind::Vertex, i as u32).entity.kind() != sm_core::EntityKind::TopoFace {
            let e = f.exact[i];
            p.mesh.set_point(PointIx::new(i), Point3::new(e[0], e[1], e[2])).expect("set");
        }
    }
    p
}

fn shear(a: [f64; 3]) -> [f64; 3] {
    [a[0] + 0.15 * a[1], a[1], a[2]]
}

#[test]
fn winslow_restores_the_lattice_and_is_affine_invariant() {
    let f = lattice::quad([6, 6], [6.0; 2]).expect("quad");
    let p = interior_jitter(&f, 17, 0.25);
    assert!(max_dev(&pts_of(&p.mesh), &p.exact) > 0.05);
    for sched in [Schedule::Colored, Schedule::Jacobi] {
        let o = run(&p, &req(Method::Winslow { relax: 1.0 }, sched, 600, false));
        let dev = max_dev(&o.points, &p.exact);
        eprintln!("winslow {sched:?}: {} sweeps, dev/h {:e}, pinned {:?}", o.report.iterations_run, dev / p.h, o.report.pinned);
        assert!(dev <= 1e-9 * p.h, "{sched:?}: {dev:e}");
        assert!(o.report.pinned["winslow_unstructured"] > 0);

        // The image under a fixed shear.
        let mut q = p.clone();
        let start: Vec<[f64; 3]> = pts_of(&p.mesh).into_iter().map(shear).collect();
        set_points(&mut q.mesh, &start);
        let exact: Vec<[f64; 3]> = p.exact.iter().copied().map(shear).collect();
        let o2 = run(&q, &req(Method::Winslow { relax: 1.0 }, sched, 600, false));
        let dev2 = max_dev(&o2.points, &exact);
        eprintln!("winslow sheared {sched:?}: {} sweeps, dev/h {:e}", o2.report.iterations_run, dev2 / p.h);
        assert!(dev2 <= 1e-9 * p.h, "{sched:?} sheared: {dev2:e}");
    }
}

#[test]
fn winslow_is_two_dimensional_only() {
    let f = lattice::hex([3, 3, 3], [1.0; 3]).expect("hex");
    let r = req(Method::Winslow { relax: 1.0 }, Schedule::Colored, 5, false);
    assert!(matches!(sm_smooth::smooth(&input(&f), &r), Err(sm_smooth::SmoothError::Unsupported(_))));
}
