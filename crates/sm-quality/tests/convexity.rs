//! W4-POLY-MEASURE (a): `cell_convexity` and `fast_path_eligible` on analytic cells.
#![allow(missing_docs)]

use proptest::prelude::*;
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{CellIx, CellShape, MeshRead, SerialAlloc};
use sm_quality::{CONVEX_CHECK_MAX, RT_PLANARITY_TOL, cell_convexity, convexity_column, convexity_report, fast_path_eligible};
use sm_testkit::determinism::{check, hash_f64_bits};
use sm_testkit::meshes::{self, CellSpec};
use sm_testkit::strategies::config;

const TOL: f64 = RT_PLANARITY_TOL;

fn poly(points: &[[f64; 3]], loops: Vec<Vec<usize>>) -> ReferenceMesh {
    meshes::build_mesh(points, &[CellSpec::Poly(loops)], &mut SerialAlloc::new()).unwrap()
}

const CUBE: [[f64; 3]; 8] =
    [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];

/// Outward loops of the unit cube without its top face.
fn cube_sides() -> Vec<Vec<usize>> {
    vec![vec![0, 3, 2, 1], vec![0, 1, 5, 4], vec![1, 2, 6, 5], vec![2, 3, 7, 6], vec![3, 0, 4, 7]]
}

/// Outward loops of the prism over a counter-clockwise polygon of `n` points at indices `0..n`
/// (z = 0) and `n..2n` (z = 1).
fn prism(n: usize) -> Vec<Vec<usize>> {
    let mut l = vec![(0..n).rev().collect::<Vec<_>>(), (n..2 * n).collect()];
    for i in 0..n {
        let j = (i + 1) % n;
        l.push(vec![i, j, n + j, n + i]);
    }
    l
}

fn extrude(ring: &[[f64; 2]]) -> Vec<[f64; 3]> {
    [0.0, 1.0].iter().flat_map(|&z| ring.iter().map(move |p| [p[0], p[1], z])).collect()
}

#[test]
fn unit_cube_is_convex_and_eligible() {
    let m = meshes::single_hex().unwrap().mesh;
    let x = cell_convexity(&m, CellIx::new(0), TOL);
    assert!(x.convex && x.eligible && x.planar);
    assert_eq!(x.faces, 6);
    assert!(x.convexity.abs() < 1e-14, "{}", x.convexity);
    assert!(fast_path_eligible(&m, CellIx::new(0), TOL));
}

#[test]
fn l_shaped_prism_is_not_convex_excess_is_the_hand_value() {
    // L = [0,2]x[0,1] u [0,1]x[0,2], height 1: volume 3. The walls y = 1 and x = 1 each have a
    // vertex 1 outside ((0,2) and (2,0) respectively).
    let m = poly(&extrude(&[[0., 0.], [2., 0.], [2., 1.], [1., 1.], [1., 2.], [0., 2.]]), prism(6));
    let x = cell_convexity(&m, CellIx::new(0), TOL);
    assert!(!x.convex && !x.eligible && x.planar);
    assert!((x.excess - 1.0).abs() < 1e-12, "{}", x.excess);
    assert!((x.convexity - 3f64.powf(-1.0 / 3.0)).abs() < 1e-12, "{}", x.convexity);
}

#[test]
fn cube_with_a_face_split_into_two_coplanar_triangles_is_convex() {
    let mut loops = cube_sides();
    loops.push(vec![4, 5, 6]);
    loops.push(vec![4, 6, 7]);
    let m = poly(&CUBE, loops);
    let x = cell_convexity(&m, CellIx::new(0), TOL);
    assert_eq!(x.faces, 7);
    assert!(x.convex && x.eligible, "{x:?}");
    assert!(x.convexity.abs() < 1e-12);
}

/// The dent: the top face is four triangles to the apex `(1/2, 1/2, 1 - d)`. Each triangle plane
/// is `z = 1 - 2 d y` (or a mirror), with unit normal `(0, 2d, 1) / sqrt(1 + 4 d^2)`; the far
/// top corner is `2 d / sqrt(1 + 4 d^2)` outside it. The volume is `1 - d/3`.
#[test]
fn inward_pyramid_dent_has_the_hand_convexity() {
    let d = 0.02;
    let mut pts = CUBE.to_vec();
    pts.push([0.5, 0.5, 1.0 - d]);
    let mut loops = cube_sides();
    loops.extend([vec![4, 5, 8], vec![5, 6, 8], vec![6, 7, 8], vec![7, 4, 8]]);
    let m = poly(&pts, loops);
    let x = cell_convexity(&m, CellIx::new(0), TOL);
    let v = 1.0 - d / 3.0;
    let hand = 2.0 * d / (1.0 + 4.0 * d * d).sqrt() / v.cbrt();
    assert!(!x.convex && !x.eligible);
    assert!((x.convexity - hand).abs() < 1e-12, "{} vs {hand}", x.convexity);
}

#[test]
fn convex_prism_over_a_63_gon_is_convex_but_over_the_cap() {
    let n = 63;
    let ring: Vec<[f64; 2]> = (0..n)
        .map(|i| {
            let a = std::f64::consts::TAU * i as f64 / n as f64;
            [a.cos(), a.sin()]
        })
        .collect();
    let m = poly(&extrude(&ring), prism(n));
    let x = cell_convexity(&m, CellIx::new(0), TOL);
    assert_eq!(x.faces, 65);
    assert!(x.faces > CONVEX_CHECK_MAX);
    assert!(x.convex && !x.eligible, "{x:?}");
    let r = convexity_report(&m, TOL);
    assert_eq!((r.cells, r.convex, r.eligible, r.over_cap, r.convex_over_cap), (1, 1, 0, 1, 1));
}

#[test]
fn nonplanar_face_makes_a_cell_ineligible() {
    // A hexahedron with one warped top corner: the top quad is not planar.
    let mut pts = CUBE.to_vec();
    pts[6][2] = 1.2;
    let cells = [CellSpec::Template(CellShape::Hex, (0..8).collect())];
    let m = meshes::build_mesh(&pts, &cells, &mut SerialAlloc::new()).unwrap();
    let x = cell_convexity(&m, CellIx::new(0), TOL);
    assert!(!x.planar && !x.convex && !x.eligible);
}

fn fingerprint(m: &dyn MeshRead) -> (u64, [usize; 4]) {
    let col = convexity_column(m, TOL);
    let r = convexity_report(m, TOL);
    (hash_f64_bits(&col), [r.cells, r.convex, r.eligible, r.over_cap])
}

#[test]
fn column_hash_is_independent_of_the_thread_count() {
    let fx = meshes::tet_block_warped([8, 8, 8], [1.0; 3], 0.2).unwrap();
    assert!(fx.mesh.cell_count() > 2048, "more than one chunk");
    check(2, 4, |t| rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap().install(|| fingerprint(&fx.mesh))).unwrap();
    // The ambient pool (RAYON_NUM_THREADS) equals an explicit single thread.
    let one = rayon::ThreadPoolBuilder::new().num_threads(1).build().unwrap().install(|| fingerprint(&fx.mesh));
    assert_eq!(fingerprint(&fx.mesh), one);
}

proptest! {
    #![proptest_config(config(16))]
    // Every tetrahedron is convex with four planar faces, whatever the block shape.
    #[test]
    fn tets_are_always_eligible(n in 1u32..4, h in 0.1f64..5.0, w in 0.1f64..5.0) {
        let fx = meshes::tet_block([n, 2, 2], [h, w, 1.0]).unwrap();
        let r = convexity_report(&fx.mesh, TOL);
        prop_assert_eq!(r.eligible, r.cells);
        prop_assert!(r.max_convexity < 1e-12);
    }
}
