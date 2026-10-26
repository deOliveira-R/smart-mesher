//! Acceptance 11: bit-identical output over repeated runs and thread counts.

mod common;
use common::*;
use core::hash::Hasher;
use sm_scheme_tet::{TetMesh, TetOptions, delaunay, tetrahedralize};
use sm_testkit::determinism::{self, Fnv};
use sm_testkit::surf3;

/// Hash of `(points bits, tets, n_steiner, flips)`; `stage_ms` is excluded.
pub fn mesh_hash(m: &TetMesh) -> u64 {
    let mut h = Fnv::default();
    for p in &m.points {
        for c in p {
            h.write(&c.to_bits().to_le_bytes());
        }
    }
    for t in &m.tets {
        for i in t {
            h.write(&i.to_le_bytes());
        }
    }
    h.write(&(m.stats.n_steiner as u64).to_le_bytes());
    h.write(&(m.stats.flips as u64).to_le_bytes());
    h.write(&(m.stats.smooth_passes as u64).to_le_bytes());
    h.write(&(m.stats.unrefinable as u64).to_le_bytes());
    for x in [m.stats.min_dihedral_deg, m.stats.max_dihedral_deg, m.stats.max_radius_edge, m.stats.min_mean_ratio, m.stats.mean_mean_ratio] {
        h.write(&x.to_bits().to_le_bytes());
    }
    h.finish()
}

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn tetrahedralize_is_bit_identical_across_runs_and_threads() {
    let cases = [
        ("cube4", surf3::tri_cube(4)),
        ("icosphere3", surf3::icosphere(1.0, 3)),
        ("torus", surf3::tri_torus(2.0, 0.7, 24, 12)),
        ("cavity", surf3::cube_with_cavity(3)),
        ("schonhardt", surf3::schonhardt()),
        ("noisy", surf3::noisy_sphere(2)),
    ];
    for (name, f) in cases {
        let inp = input_of(&f, 0);
        for optimize in [0u8, 2] {
            let opts = TetOptions { optimize, ..TetOptions::default() };
            let h = determinism::check(3, 4, |threads| in_pool(threads, || mesh_hash(&tetrahedralize(&inp, &opts).unwrap())))
                .unwrap_or_else(|e| panic!("{name} optimize={optimize}: {e}"));
            assert_ne!(h, 0);
        }
    }
}

#[test]
fn delaunay_is_bit_identical() {
    let mut r = surf3::SplitMix64(5);
    let pts: Vec<[f64; 3]> = (0..3000).map(|_| [r.unit(), r.unit(), r.unit()]).collect();
    determinism::check(2, 4, |threads| in_pool(threads, || mesh_hash(&delaunay(&pts).unwrap()))).unwrap();
}

#[test]
fn stage_ms_is_excluded_from_equality() {
    let inp = input_of(&surf3::tri_cube(3), 0);
    let a = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    let mut b = tetrahedralize(&inp, &TetOptions::default()).unwrap();
    b.stats.stage_ms = [1.0, 2.0, 3.0, 4.0, 5.0];
    assert_eq!(a, b);
}
