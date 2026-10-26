//! Acceptance 8: bit-identical results for any thread count, replay included.

#![allow(clippy::needless_range_loop)]

mod common;

use common::*;
use serde_json::json;
use sm_layer_api::MeshLayer;
use sm_mesh_api::geometry_hash;
use sm_smooth::{Method, Objective, Schedule};
use sm_testkit::determinism::check_f64_bits;
use sm_testkit::lattice;

fn pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool").install(f)
}

fn methods() -> Vec<(&'static str, Method)> {
    vec![
        ("laplacian", Method::Laplacian { relax: 1.3 }),
        ("smart", Method::SmartLaplacian { relax: 1.0 }),
        ("centroid", Method::CentroidArea { relax: 1.0 }),
        ("optimize", Method::Optimize { objective: Objective::MeanRatio, p: 4.0 }),
        ("condition", Method::Optimize { objective: Objective::Condition, p: 4.0 }),
        ("winslow", Method::Winslow { relax: 1.0 }),
    ]
}

fn flat(pts: &[[f64; 3]]) -> Vec<f64> {
    pts.iter().flatten().copied().collect()
}

#[test]
fn a8_points_are_bit_identical_for_every_method_and_schedule() {
    let fixtures = [
        ("hex", lattice::hex([5, 5, 5], [1.0; 3]).expect("hex").perturbed(2, 0.2).expect("perturb")),
        ("tet", lattice::tet_kuhn([3, 3, 3], [1.0; 3]).expect("tet").perturbed(2, 0.2).expect("perturb")),
        ("quad", lattice::quad([8, 8], [1.0; 2]).expect("quad").perturbed(2, 0.3).expect("perturb")),
        ("tube", lattice::tube_quad(12, 6, 1.0, 2.0).expect("tube").perturbed(2, 0.2).expect("perturb")),
    ];
    for (fname, f) in &fixtures {
        for (mname, m) in methods() {
            for sched in [Schedule::Colored, Schedule::Jacobi] {
                if matches!(m, Method::Winslow { .. }) && !matches!(*fname, "quad" | "tube") {
                    continue; // 2-D only
                }
                let mut r = req(m, sched, 12, false);
                r.tolerance = 0.0;
                let base = check_f64_bits(3, 4, |t| pool(t, || flat(&run(f, &r).points))).unwrap_or_else(|e| panic!("{fname} {mname} {sched:?}: {e}"));
                let default = flat(&run(f, &r).points);
                assert_eq!(base.iter().map(|x| x.to_bits()).collect::<Vec<_>>(), default.iter().map(|x| x.to_bits()).collect::<Vec<_>>(), "{fname} {mname} {sched:?}: default pool");
            }
        }
    }
}

#[test]
fn a8_geometry_hash_and_state_hash_do_not_depend_on_threads() {
    let go = |threads: usize| {
        pool(threads, || {
            let (mut s, mesh) = meshed_brick([6.0; 3], 1.0);
            apply(&mut s, &JitterMesh { mesh, seed: 4, amp: 0.15, volume_only: false }).expect("jitter");
            run_json(
                &mut s,
                "mesh.smooth",
                json!({"mesh": mesh.to_string(), "method": {"kind": "optimize", "p": 4.0}, "iterations": 6, "tolerance": 0.0, "fix_boundary": false}),
            )
            .expect("smooth");
            let layer = s.state().layer::<MeshLayer>().expect("layer");
            (geometry_hash(layer.get(mesh).expect("mesh").mesh()), s.state().hash())
        })
    };
    let one = go(1);
    assert_eq!(go(4), one);
    assert_eq!(go(2), one);
    assert_eq!(go(1), one, "a rerun agrees");
}
