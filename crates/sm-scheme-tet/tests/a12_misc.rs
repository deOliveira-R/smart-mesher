//! Extra robustness: scaling, disjoint solids, budgets and the smoke performance bound.

mod common;
use common::*;
use sm_scheme_tet::{SurfaceInput, TetError, TetOptions, tetrahedralize};
use sm_testkit::surf3;

fn scaled(f: &surf3::SurfaceFixture, s: f64, offset: [f64; 3]) -> SurfaceInput {
    let mut inp = input_of(f, 0);
    for p in &mut inp.points {
        for k in 0..3 {
            p[k] = p[k] * s + offset[k];
        }
    }
    inp
}

#[test]
fn scale_and_translation_do_not_matter() {
    let f = surf3::tri_cube(4);
    for (s, off) in [(1e-9, [0.0; 3]), (1e9, [0.0; 3]), (1.0, [1e3, -2e3, 5e2]), (3e-4, [7.0, 7.0, 7.0])] {
        let inp = scaled(&f, s, off);
        let m = tetrahedralize(&inp, &TetOptions::default()).unwrap_or_else(|e| panic!("scale {s}: {e}"));
        assert!(all_exact_positive(&m), "scale {s}");
        check_boundary(&m, &inp.tris).unwrap();
        let want = surf3::polyhedron_volume(&inp.points, &inp.tris);
        let v = total_volume(&m);
        assert!(((v - want) / want).abs() < 1e-9, "scale {s}: {v} vs {want}");
    }
}

#[test]
fn disjoint_solids_in_one_surface() {
    let f = surf3::tri_cube(3);
    let a = input_of(&f, 0);
    let n = a.points.len() as u32;
    let mut points = a.points.clone();
    points.extend(a.points.iter().map(|p| [p[0] + 3.0, p[1] + 0.2, p[2] - 0.7]));
    let mut tris = a.tris.clone();
    tris.extend(a.tris.iter().map(|t| t.map(|i| i + n)));
    let inp = SurfaceInput { points, tris };
    let m = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    assert!(all_exact_positive(&m));
    check_boundary(&m, &inp.tris).unwrap();
    assert!((total_volume(&m) - 2.0).abs() < 1e-12);
}

#[test]
fn steiner_budget_is_enforced() {
    let inp = input_of(&surf3::tri_cube(6), 0);
    let r = tetrahedralize(&inp, &TetOptions { max_steiner: 3, ..TetOptions::default() });
    assert!(matches!(r, Err(TetError::SteinerBudget { limit: 3 })), "{r:?}");
    // The default budget scales with the input.
    assert!(tetrahedralize(&inp, &TetOptions::default()).is_ok());
}

/// S3: a non-ignored smoke bound, generous enough for debug builds and slow machines.
#[test]
fn smoke_performance_bound() {
    let inp = input_of(&surf3::tri_cube(8), 0);
    let t = std::time::Instant::now();
    let m = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    let s = t.elapsed().as_secs_f64();
    eprintln!("tri_cube(8): {} tets in {s:.3} s", m.tets.len());
    assert!(s < 60.0, "took {s} s");
}
