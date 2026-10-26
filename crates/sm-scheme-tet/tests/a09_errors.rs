//! Acceptance 9: typed errors, never a panic and never an invalid mesh.

mod common;
use common::*;
use proptest::prelude::*;
use sm_scheme_tet::{SurfaceInput, TetError, TetOptions, tetrahedralize};
use sm_testkit::surf3;

fn opts() -> TetOptions<'static> {
    TetOptions::default()
}

#[test]
fn missing_triangle_is_not_closed_and_names_the_edge() {
    let full = surf3::tri_cube(3);
    let open = surf3::open_cube_missing_tri(3);
    let missing: Vec<[u32; 3]> = full.tris.iter().copied().filter(|t| !open.tris.contains(t)).collect();
    assert_eq!(missing.len(), 1);
    let m = missing[0];
    let edges: Vec<[u32; 2]> = (0..3).map(|k| [m[k].min(m[(k + 1) % 3]), m[k].max(m[(k + 1) % 3])]).collect();
    let inp = SurfaceInput { points: open.points.clone(), tris: open.tris.clone() };
    match tetrahedralize(&inp, &opts()) {
        Err(TetError::NotClosed { edge }) => assert!(edges.contains(&edge), "{edge:?} not an edge of {m:?}"),
        other => panic!("expected NotClosed, got {other:?}"),
    }
}

#[test]
fn duplicated_point_is_reported() {
    let f = surf3::tri_cube(2);
    let mut inp = input_of(&f, 0);
    let dup = inp.points[5];
    inp.points.push(dup);
    let last = (inp.points.len() - 1) as u32;
    assert_eq!(tetrahedralize(&inp, &opts()).unwrap_err(), TetError::DuplicatePoint { a: 5, b: last });
}

#[test]
fn all_triangles_flipped_is_an_inverted_shell() {
    let f = surf3::tri_cube(2);
    let mut inp = input_of(&f, 0);
    for t in &mut inp.tris {
        t.swap(1, 2);
    }
    assert!(matches!(tetrahedralize(&inp, &opts()), Err(TetError::InvertedShell { .. })));
}

#[test]
fn edge_shared_by_four_triangles_is_non_manifold() {
    // Two tetrahedra glued along the edge (0,1).
    let points = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]];
    let t1 = [[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]];
    let t2 = [[0, 4, 1], [0, 1, 5], [0, 5, 4], [1, 4, 5]];
    let mut tris: Vec<[u32; 3]> = t1.to_vec();
    tris.extend_from_slice(&t2);
    let inp = SurfaceInput { points, tris };
    assert_eq!(tetrahedralize(&inp, &opts()).unwrap_err(), TetError::NonManifold { edge: [0, 1] });
}

#[test]
fn bad_options_are_typed() {
    let inp = input_of(&surf3::tri_cube(2), 0);
    for o in [
        TetOptions { optimize: 3, ..opts() },
        TetOptions { radius_edge: 0.5, ..opts() },
        TetOptions { gradation: 0.9, ..opts() },
        TetOptions { gradation: f64::NAN, ..opts() },
        TetOptions { size: sm_scheme_tet::SizeSpec::Constant(-1.0), ..opts() },
    ] {
        assert!(matches!(tetrahedralize(&inp, &o), Err(TetError::BadOption(_))));
    }
}

#[test]
fn degenerate_triangles_and_bad_indices() {
    let f = surf3::tri_cube(2);
    let mut inp = input_of(&f, 0);
    inp.tris[3] = [inp.tris[3][0], inp.tris[3][0], inp.tris[3][1]];
    assert_eq!(tetrahedralize(&inp, &opts()).unwrap_err(), TetError::DegenerateTri { tri: 3 });
    let mut inp = input_of(&f, 0);
    inp.tris[4][2] = 10_000;
    assert_eq!(tetrahedralize(&inp, &opts()).unwrap_err(), TetError::DegenerateTri { tri: 4 });
}

#[test]
fn out_of_range_coordinates() {
    for bad in [1e40, f64::NAN, f64::INFINITY, 1e-40] {
        let mut inp = input_of(&surf3::tri_cube(2), 0);
        inp.points[0][0] = bad;
        assert!(matches!(tetrahedralize(&inp, &opts()), Err(TetError::OutOfRange(_))), "{bad}");
    }
    // The pure Delaunay entry point reports the same.
    let pts = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1e40]];
    assert!(matches!(sm_scheme_tet::delaunay(&pts), Err(TetError::OutOfRange(_))));
}

fn cube_soup(offset: [f64; 3]) -> (Vec<[f64; 3]>, Vec<[u32; 3]>) {
    let f = surf3::tri_cube(2);
    let pts: Vec<[f64; 3]> = f.points.iter().map(|p| [p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]]).collect();
    (pts, f.oriented_tris(0))
}

#[test]
fn interpenetrating_cubes_fail_with_a_typed_error() {
    let (p1, t1) = cube_soup([0.0; 3]);
    let (p2, t2) = cube_soup([0.37, 0.41, 0.29]);
    let n = p1.len() as u32;
    let mut points = p1;
    points.extend(p2);
    let mut tris = t1;
    tris.extend(t2.iter().map(|t| t.map(|i| i + n)));
    let r = tetrahedralize(&SurfaceInput { points, tris }, &opts());
    assert!(matches!(r, Err(TetError::RecoveryFailed { .. }) | Err(TetError::SelfIntersection { .. })), "{r:?}");
}

fn perturbed_icosphere(seed: u64, amp: f64) -> SurfaceInput {
    let f = surf3::icosphere(1.0, 2);
    let mut r = surf3::SplitMix64(seed);
    let points = f.points.iter().map(|p| {
        let s = 1.0 + amp * r.unit();
        [p[0] * s, p[1] * s, p[2] * s]
    });
    SurfaceInput { points: points.collect(), tris: f.oriented_tris(0) }
}

proptest! {
    #![proptest_config(ProptestConfig { cases: 24, ..ProptestConfig::default() })]
    #[test]
    fn perturbed_icosphere_is_ok_and_valid_or_a_typed_error(seed in 0u64..10_000, amp in 0.05f64..0.30) {
        let inp = perturbed_icosphere(seed, amp);
        match tetrahedralize(&inp, &opts()) {
            Ok(m) => {
                prop_assert!(all_exact_positive(&m));
                prop_assert!(check_boundary(&m, &inp.tris).is_ok());
                let want = surf3::polyhedron_volume(&inp.points, &inp.tris);
                let v = total_volume(&m);
                prop_assert!(((v - want) / want).abs() < 1e-9, "{} vs {}", v, want);
            }
            Err(e) => {
                prop_assert!(matches!(
                    e,
                    TetError::RecoveryFailed { .. } | TetError::SelfIntersection { .. } | TetError::InvertedShell { .. } | TetError::SteinerBudget { .. }
                ), "unexpected error {:?}", e);
            }
        }
    }
}
