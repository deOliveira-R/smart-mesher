//! The dual passes sm-quality's starness criterion (star-shaped about the vertex mean), not only
//! the centroid rule of `validate_geometry`.

mod common;
use common::*;
use sm_poly::{FaceMode, PolyOptions, TopologyResolver, VertexClassResolver};
use sm_quality::{Metric, Options, compute, validity};
use sm_testkit::{lattice, meshes};

fn min_starness(m: &dyn sm_mesh_api::MeshRead) -> (f64, usize) {
    let st = compute(m, Metric::Starness, &Options::default()).unwrap();
    let bad = st.iter().filter(|s| s.is_nan() || **s <= 1e-9).count();
    (st.iter().copied().fold(f64::INFINITY, f64::min), bad)
}

fn check(name: &str, m: &sm_mesh::Mesh) {
    let v = validity(m, &Options::default()).unwrap();
    let (lo, bad) = min_starness(m);
    assert!(v.n_inverted == 0 && v.n_degenerate == 0, "{name}: inverted {} degenerate {}", v.n_inverted, v.n_degenerate);
    assert!(bad == 0, "{name}: {bad} cells with starness <= 1e-9 (min {lo})");
}

#[test]
fn duals_are_star_shaped_about_the_vertex_mean() {
    for mode in [FaceMode::Planar, FaceMode::Keep] {
        let opts = PolyOptions::new().with_faces(mode);
        let b = meshes::tet_block([4, 4, 4], [1.0; 3]).unwrap();
        let (m, _) = dual_prod(&b.mesh, &opts, &VertexClassResolver).unwrap();
        check(&format!("tet_block {mode:?}"), &m);
        let w = meshes::tet_block_warped([5, 5, 5], [1.0; 3], 0.35).unwrap();
        let (m, _) = dual_prod(&w.mesh, &opts, &VertexClassResolver).unwrap();
        check(&format!("warped {mode:?}"), &m);
        let l = meshes::tet_lshape(3, 1.0).unwrap();
        let (m, _) = dual_prod(&l.mesh, &opts, &VertexClassResolver).unwrap();
        check(&format!("lshape {mode:?}"), &m);
        for (n, h) in [([10, 10, 6], [1.0; 3]), ([5, 5, 3], [2.0, 2.0, 2.0]), ([3, 7, 5], [1.0, 0.3, 2.0]), ([6, 6, 6], [1.0, 1.0, 0.2])] {
            let b = meshes::tet_block(n, h).unwrap();
            let (m, _) = dual_prod(&b.mesh, &opts, &VertexClassResolver).unwrap();
            check(&format!("tet_block {n:?} {h:?} {mode:?}"), &m);
        }
        for seed in 0..8u64 {
            let lf = lattice::tet_kuhn([4, 4, 4], [4.0; 3]).unwrap().perturbed(seed, 0.3 + 0.01 * seed as f64).unwrap();
            let (m, _) = dual_prod(&lf.mesh, &opts, &TopologyResolver(&lf.topo)).unwrap();
            check(&format!("kuhn seed {seed} {mode:?}"), &m);
        }
    }
}
