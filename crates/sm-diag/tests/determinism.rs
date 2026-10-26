//! Bit-identical output for 1 thread and N, repeated runs and concurrent callers.
#![allow(missing_docs)]

mod common;
use common::*;
use serde_json::json;
use sm_diag::{MeshQuality, QualityOptions, ValidateOptions, flagged_cells, quality_report, validate_mesh};
use sm_mesh_api::reference::ReferenceMesh;
use sm_mesh_api::{MeshEdit, MeshRead, PointIx, SerialAlloc};
use sm_quality::Profile;
use sm_testkit::determinism::{Fnv, check};
use std::hash::Hasher;

fn with_threads<R: Send>(t: usize, f: impl FnOnce() -> R + Send) -> R {
    rayon::ThreadPoolBuilder::new().num_threads(t).build().unwrap().install(f)
}

/// A 20^3 block (8000 cells) with every interior vertex moved by an LCG seeded from its index.
fn perturbed(n: u32, amp: f64) -> ReferenceMesh {
    let mut m = block([n, n, n], [1.0; 3], &[], &mut SerialAlloc::new());
    for v in 0..m.vertex_count() {
        let p = m.point(v);
        if [p.x, p.y, p.z].iter().all(|c| *c > 1e-9 && (*c - n as f64).abs() > 1e-9) {
            let mut s = (v as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0xD1B5_4A32_D192_ED03;
            let mut next = || {
                s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                ((s >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 2.0 * amp
            };
            let (dx, dy, dz) = (next(), next(), next());
            m.set_point(PointIx::new(v), sm_core::Point3::new(p.x + dx, p.y + dy, p.z + dz)).unwrap();
        }
    }
    m
}

fn digest(bytes: &[u8]) -> u64 {
    let mut h = Fnv::default();
    h.write(bytes);
    h.finish()
}

#[test]
fn quality_and_validate_json_are_bit_identical_across_threads() {
    let m = perturbed(20, 0.3);
    assert_eq!(m.cell_count(), 8000);
    for profile in [Profile::Generic, Profile::Transport, Profile::OpenFoam] {
        let o = QualityOptions { profile, ..Default::default() };
        let (bytes, h) = check(2, 4, |t| {
            with_threads(t, || {
                let r = quality_report(&m, &o, &|op| op == "mesh.smooth").unwrap();
                let b = serde_json::to_vec(&r).unwrap();
                let v = serde_json::to_vec(&validate_mesh(&m, &ValidateOptions::default(), None)).unwrap();
                (digest(&b), digest(&v))
            })
        })
        .map(|x| (x, 0))
        .unwrap();
        let _ = (bytes, h);
    }
    // The report on this mesh is not trivially empty.
    let r = quality_report(&m, &QualityOptions::default(), &|_| true).unwrap();
    assert!(r.metrics.iter().any(|x| x.n_warn + x.n_fail > 0), "the perturbation should grade somewhere");
    assert!(r.metrics.iter().all(|x| !x.worst.is_empty()));
}

#[test]
fn flagged_cells_are_identical_across_threads() {
    let m = perturbed(12, 0.35);
    let cells = check(2, 4, |t| with_threads(t, || flagged_cells(&m, Profile::Generic, None, sm_diag::Level::Warn, true).unwrap().iter().map(|c| c.0).collect::<Vec<u32>>())).unwrap();
    assert!(!cells.is_empty());
}

#[test]
fn query_bytes_through_the_engine_are_identical_for_concurrent_callers() {
    let (rig, _) = Rig::with_block([6, 6, 6], &[(100, [3.3, 3.3, 3.3])]);
    let run = |t: usize| {
        with_threads(t, || {
            let a = rig.query("mesh.quality", json!({})).unwrap();
            let b = rig.query("mesh.validate", json!({})).unwrap();
            digest(&serde_json::to_vec(&[a, b]).unwrap())
        })
    };
    check(3, 4, run).unwrap();
    let _ = MeshQuality { mesh: None, profile: Default::default(), metrics: None, thresholds: Default::default(), worst_n: 10, bins: 20 };
}
