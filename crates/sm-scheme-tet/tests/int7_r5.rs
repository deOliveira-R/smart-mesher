//! Round 5 (review of ruling INT-7): the spacing pass may not lower the mesh minima, all quality
//! violations are located, the deferred-removal count is of distinct points. Counters and
//! quality numbers only; timings are in `#[ignore]` records.

mod common;
use common::*;
use sm_scheme_tet::{SizeSpec, SurfaceInput, TetOptions, tetrahedralize, tetrahedralize_without_respace};
use sm_testkit::surf3::{self, SplitMix64};

fn relief_cube(n: u32, seed: u64, depth: f64) -> SurfaceInput {
    let f = surf3::tri_cube(n);
    let mut inp = input_of(&f, 0);
    let mut rng = SplitMix64(seed);
    for p in &mut inp.points {
        if p[2] == 1.0 && p[0] > 0.0 && p[0] < 1.0 && p[1] > 0.0 && p[1] < 1.0 {
            p[2] = 1.0 - depth * rng.unit().abs();
        }
    }
    inp
}

fn nan_field(p: [f64; 3]) -> f64 {
    if p[2] > 0.5 { f64::NAN } else { 0.08 }
}

/// The good meshes of the review plus a relief case: (label, input, options).
fn respace_cases() -> Vec<(&'static str, SurfaceInput, TetOptions<'static>)> {
    let d = TetOptions::default;
    vec![
        ("ball icosphere(1,4) h0.1", input_of(&surf3::icosphere(1.0, 4), 0), TetOptions { size: SizeSpec::Constant(0.1), ..d() }),
        ("cube(4) NaN/0.08 field", input_of(&surf3::tri_cube(4), 0), TetOptions { size: SizeSpec::Field(&nan_field), ..d() }),
        ("cube_with_cavity(4)", input_of(&surf3::cube_with_cavity(4), 0), d()),
        ("ball icosphere(1,3) gradation 1", input_of(&surf3::icosphere(1.0, 3), 0), TetOptions { gradation: 1.0, ..d() }),
        ("relief n6 seed2 d0.9", relief_cube(6, 2, 0.9), d()),
    ]
}

/// Ruling INT-7, review round 4: the spacing pass may not lower the mesh minimum of the
/// dihedral angle or of the mean ratio. Both are at least what the same build without the pass
/// gives; the spacing deviation does not get worse. The gain kept is printed.
#[test]
fn respace_never_lowers_the_mesh_minima() {
    for (label, inp, opts) in respace_cases() {
        let with = tetrahedralize(&inp, &opts).unwrap_or_else(|e| panic!("{label}: {e}"));
        let without = tetrahedralize_without_respace(&inp, &opts).unwrap_or_else(|e| panic!("{label}: {e}"));
        check_boundary(&with, &inp.tris).unwrap_or_else(|e| panic!("{label}: {e}"));
        let (a, b) = (&with.stats, &without.stats);
        eprintln!(
            "{label}: min dihedral {:.3} -> {:.3}, min mean ratio {:.4} -> {:.4}, mean mean ratio {:.4} -> {:.4}, rms_dev {:.4} -> {:.4}",
            b.min_dihedral_deg, a.min_dihedral_deg, b.min_mean_ratio, a.min_mean_ratio, b.mean_mean_ratio, a.mean_mean_ratio, b.spacing.rms_dev, a.spacing.rms_dev
        );
        assert!(a.min_dihedral_deg >= b.min_dihedral_deg - 1e-9, "{label}: min dihedral {} < {}", a.min_dihedral_deg, b.min_dihedral_deg);
        assert!(a.min_mean_ratio >= b.min_mean_ratio - 1e-12, "{label}: min mean ratio {} < {}", a.min_mean_ratio, b.min_mean_ratio);
        assert!(a.spacing.rms_dev <= b.spacing.rms_dev + 1e-9, "{label}: spacing got worse");
    }
}

/// Every violation of the quality floor is located by default (thin slab, many violations);
/// `max_diagnostics` caps the list and keeps the same worst-first prefix.
#[test]
fn diagnostics_are_uncapped_by_default() {
    let mut f = surf3::tri_cube(6);
    for p in &mut f.points {
        p[2] *= 0.02;
    }
    let inp = input_of(&f, 0);
    let m = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    let st = &m.stats;
    eprintln!("slab: {} violations, {} located, {} tets", st.n_quality_violations, st.quality_diagnostics.len(), m.tets.len());
    assert!(st.n_quality_violations > 0, "the slab must violate the floor for this test to mean anything");
    assert_eq!(st.quality_diagnostics.len(), st.n_quality_violations);
    let capped = tetrahedralize(&inp, &TetOptions { max_diagnostics: 5, ..TetOptions::default() }).unwrap();
    assert_eq!(capped.stats.n_quality_violations, st.n_quality_violations);
    assert_eq!(capped.stats.quality_diagnostics.len(), 5.min(st.n_quality_violations));
    for (x, y) in capped.stats.quality_diagnostics.iter().zip(&st.quality_diagnostics) {
        assert_eq!(x.value.to_bits(), y.value.to_bits());
        assert_eq!(x.position.map(f64::to_bits), y.position.map(f64::to_bits));
    }
}
