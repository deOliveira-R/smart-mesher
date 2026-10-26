//! Determinism (CLAUDE.md, binding): the kernel and the scheme give bit-identical output on
//! repeated runs, from concurrent callers and under both executors; no hash collections in
//! the crate. CI also runs this binary with `RAYON_NUM_THREADS=1`.

mod common;

use common::*;
use sm_scheme_api::stubs::brick_setup;
use sm_scheme_api::{ScopedThreads, Sequential};
use sm_scheme_pave::{PaveOptions, pave_region};
use sm_testkit::determinism::{check, check_f64_bits};
use sm_testkit::loops2d;

fn resampled(l: &[(f64, f64)], h: f64) -> Vec<[f64; 2]> {
    let mut out = Vec::new();
    for i in 0..l.len() {
        let (a, b) = (l[i], l[(i + 1) % l.len()]);
        let k = (((b.0 - a.0).hypot(b.1 - a.1) / h).ceil() as usize).max(1);
        for s in 0..k {
            let t = s as f64 / k as f64;
            out.push([a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1)]);
        }
    }
    out
}

fn kernel_bits(loops: &[Vec<[f64; 2]>], metric: &(dyn Fn([f64; 2]) -> [f64; 3] + Sync)) -> Vec<f64> {
    let m = pave_region(loops, metric, None, &PaveOptions::default()).expect("pave");
    let mut v: Vec<f64> = m.points.iter().flatten().copied().collect();
    v.extend(m.quads.iter().flatten().map(|q| f64::from(*q)));
    v.push(f64::from(m.rounds));
    v
}

#[test]
fn the_kernel_is_bit_identical_across_runs_and_concurrent_callers() {
    let circle = vec![resampled(&loops2d::circle(1.5, 32).outer, 0.125)];
    let id = |_: [f64; 2]| [1.0, 0.0, 1.0];
    check_f64_bits(3, 4, |_| kernel_bits(&circle, &id)).expect("circle");
    // A non-trivial constant metric and a region with holes.
    let aniso = |_: [f64; 2]| [4.0, 0.5, 1.0];
    check_f64_bits(2, 4, |_| kernel_bits(&circle, &aniso)).expect("anisotropic");
    let r = loops2d::rectangle_with_holes();
    let holes: Vec<Vec<[f64; 2]>> = r.loops().map(|l| resampled(l, 0.5)).collect();
    check_f64_bits(2, 4, |_| kernel_bits(&holes, &id)).expect("holes");
}

#[test]
fn the_scheme_is_bit_identical_under_both_executors_and_concurrent_callers() {
    let (topo, ids, model) = brick_setup([4.0, 3.0, 1.0]);
    let face = ids.faces[5];
    let pairs: Vec<_> = face_edges(&topo, face).into_iter().map(|e| (e, if (edge_len(&topo, e) - 4.0).abs() < 1e-9 { 8 } else { 6 })).collect();
    let planar = hard(&pairs);
    let (ctopo, cids, cmodel) = cylinder(3.0, 1.0);
    let lateral = hard(&[(cids.edges[0], 72), (cids.edges[1], 72), (cids.edges[2], 4)]);
    let cap = hard(&[(cids.edges[0], 32)]);
    let run = |threads: usize| {
        let hash = |t: &sm_topo_api::CellComplex, m: &sm_model_api::Model, f, i: &sm_scheme_api::MeshIntent| {
            let out = if threads <= 1 { mesh_faces(t, m, &[f], i, &Sequential) } else { mesh_faces(t, m, &[f], i, &ScopedThreads(threads)) }.expect("mesh");
            (sm_mesh_api::topology_hash(&out.finished.mesh), sm_mesh_api::geometry_hash(&out.finished.mesh))
        };
        [hash(&topo, &model, face, &planar), hash(&ctopo, &cmodel, cids.faces[2], &lateral), hash(&ctopo, &cmodel, cids.faces[0], &cap)]
    };
    check(2, 4, run).expect("planar, cylinder and cap meshes are bit-identical");
}

/// The library never iterates a hash collection, reads the environment, the clock or files,
/// or prints (the in-crate `tests.rs` surveys are excluded: they only run under `--ignored`).
#[test]
fn the_library_has_no_hash_collections_env_clock_or_io() {
    fn visit(dir: &std::path::Path, hits: &mut Vec<String>) {
        for e in std::fs::read_dir(dir).expect("read_dir") {
            let p = e.expect("entry").path();
            if p.is_dir() {
                visit(&p, hits);
            } else if p.extension().is_some_and(|x| x == "rs") {
                let text = std::fs::read_to_string(&p).expect("read");
                // The needle is split so this file does not match itself.
                for needle in [concat!("Hash", "Map"), concat!("Hash", "Set"), "RandomState", "thread_rng", "SystemTime", "Instant::now", "std::env", "eprintln!", "std::fs", "println!"] {
                    if text.contains(needle) && !p.ends_with("tests.rs") && !p.ends_with("determinism.rs") {
                        hits.push(format!("{}: {needle}", p.display()));
                    }
                }
            }
        }
    }
    let mut hits = Vec::new();
    visit(&std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src"), &mut hits);
    assert!(hits.is_empty(), "{hits:?}");
}
