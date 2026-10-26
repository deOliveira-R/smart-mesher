//! Acceptance 1: Delaunay of random points plus the cube corners.

use sm_scheme_tet::{delaunay, in_sphere, ori};
use sm_testkit::surf3::SplitMix64;

fn cloud(n: usize, seed: u64) -> Vec<[f64; 3]> {
    let mut r = SplitMix64(seed);
    let mut p: Vec<[f64; 3]> = (0..n).map(|_| [0.5 * (r.unit() + 1.0), 0.5 * (r.unit() + 1.0), 0.5 * (r.unit() + 1.0)]).collect();
    for i in 0..8 {
        p.push([(i & 1) as f64, ((i >> 1) & 1) as f64, ((i >> 2) & 1) as f64]);
    }
    p
}

#[test]
fn sign_conventions_pinned() {
    let f = sm_testkit::meshes::single_tet().unwrap();
    assert!(sm_mesh_api::cell_volume(&f.mesh, sm_mesh_api::CellIx::new(0)) > 0.0);
    // The fixture's template tet: (0,0,0) (1,0,0) (0,1,0) (0,0,1).
    let p = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    assert_eq!(ori(p[0], p[1], p[2], p[3]), 1);
    assert_eq!(ori(p[0], p[2], p[1], p[3]), -1);
    let q = |a: [f64; 3]| sm_core::Point3::new(a[0], a[1], a[2]);
    // Template convention: orient3d < 0 for the positive tet.
    assert_eq!(sm_math::orient3d(q(p[0]), q(p[1]), q(p[2]), q(p[3])).as_i32(), -1);
    let cen = [0.25 * (p[0][0] + p[1][0] + p[2][0] + p[3][0]), 0.25 * (p[0][1] + p[1][1] + p[2][1] + p[3][1]), 0.25 * (p[0][2] + p[1][2] + p[2][2] + p[3][2])];
    assert!(in_sphere(p[0], p[1], p[2], p[3], cen));
    assert!(!in_sphere(p[0], p[1], p[2], p[3], [50.0, 50.0, 50.0]));
}

#[test]
fn delaunay_2000_empty_spheres_volume() {
    let pts = cloud(2000, 7);
    let m = delaunay(&pts).unwrap();
    let mut vol = 0.0;
    for t in &m.tets {
        let q = t.map(|i| pts[i as usize]);
        assert_eq!(ori(q[0], q[1], q[2], q[3]), 1);
        let d = |a: [f64; 3], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
        let (u, v, w) = (d(q[1], q[0]), d(q[2], q[0]), d(q[3], q[0]));
        vol += (u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) + u[2] * (v[0] * w[1] - v[1] * w[0])) / 6.0;
    }
    assert!((vol - 1.0).abs() < 1e-12, "vol {vol}");
    use rayon::prelude::*;
    let bad = m.tets.par_iter().filter(|t| {
        let q = t.map(|i| pts[i as usize]);
        (0..pts.len() as u32).any(|j| !t.contains(&j) && in_sphere(q[0], q[1], q[2], q[3], pts[j as usize]))
    }).count();
    assert_eq!(bad, 0);
}

#[test]
fn delaunay_permutation_invariant() {
    let pts = cloud(500, 3);
    let a = delaunay(&pts).unwrap();
    // Shuffle the input order; carry ids by coordinates.
    let mut idx: Vec<usize> = (0..pts.len()).collect();
    let mut r = SplitMix64(99);
    for i in (1..idx.len()).rev() {
        let j = (r.next_u64() % (i as u64 + 1)) as usize;
        idx.swap(i, j);
    }
    let shuf: Vec<[f64; 3]> = idx.iter().map(|&i| pts[i]).collect();
    let b = delaunay(&shuf).unwrap();
    let set = |m: &sm_scheme_tet::TetMesh, pts: &[[f64; 3]]| {
        let mut s: Vec<Vec<[u64; 3]>> = m.tets.iter().map(|t| {
            // Sort by coordinates to make it order independent but keep orientation parity out of it.
            let mut v: Vec<[u64; 3]> = t.iter().map(|&i| pts[i as usize].map(f64::to_bits)).collect();
            v.sort();
            v
        }).collect();
        s.sort();
        s
    };
    assert_eq!(set(&a, &pts), set(&b, &shuf));
}
