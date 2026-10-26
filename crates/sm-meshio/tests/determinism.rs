//! Bit-identical output across runs and thread counts: OpenFOAM files and tar, `.smsh` 1.1 bytes
//! with element types and stats (`RAYON_NUM_THREADS=1` vs default is the same experiment as the
//! explicit pools below).

mod common;

use common::foam::sets_block;
use common::wrap::Wrap;
use sm_meshio::foam::*;
use sm_meshio::SmshWriter;
use sm_mesh_api::reference::{ReferenceMesh, hex_block};
use sm_mesh_api::*;
use sm_testkit::determinism;
use sm_testkit::meshes::{all_mesh_fixtures, dual_patch, nonconvex_poly};

fn fnv(h: &mut u64, bytes: &[u8]) {
    for x in bytes {
        *h = (*h ^ u64::from(*x)).wrapping_mul(0x0100_0000_01b3);
    }
}

fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

/// Hash of every foam file, the tar stream, the index maps and the report of one build.
fn foam_hash(m: &dyn MeshRead, opts: &FoamOptions, threads: usize) -> u64 {
    in_pool(threads, || {
        let b = build_polymesh(m, opts).unwrap();
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        for f in b.mesh.files().unwrap() {
            fnv(&mut h, f.name.as_bytes());
            fnv(&mut h, &f.bytes);
        }
        let mut tar = Vec::new();
        b.mesh.write_tar(&mut tar, "constant/polyMesh/").unwrap();
        fnv(&mut h, &tar);
        for v in b.face_of_foam.iter().chain(&b.point_of_foam) {
            fnv(&mut h, &v.to_le_bytes());
        }
        fnv(&mut h, serde_json::to_string(&b.report).unwrap().as_bytes());
        h
    })
}

#[test]
fn foam_output_is_bit_identical_across_runs_and_thread_counts() {
    for f in all_mesh_fixtures().unwrap().into_iter().chain([nonconvex_poly().unwrap()]) {
        determinism::check(2, 4, |t| foam_hash(&f.mesh, &FoamOptions::default(), t)).unwrap_or_else(|e| panic!("{}: {e}", f.name));
    }
    let sets = sets_block();
    determinism::check(2, 4, |t| foam_hash(&sets, &FoamOptions::default(), t)).unwrap();
    // The sort path: a reversed face order.
    let m = hex_block([6, 5, 4], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let w = Wrap::new(&m).permute((0..m.face_count()).rev().collect());
    determinism::check(2, 4, |t| foam_hash(&w, &FoamOptions::default(), t)).unwrap();
    // A block larger than every chunk: 64 000 cells, 68 921 points (chunks are 4096 lines).
    let big = hex_block([40, 40, 40], [0.025; 3], &mut SerialAlloc::new()).unwrap();
    determinism::check(1, 3, |t| foam_hash(&big, &FoamOptions::default(), t)).unwrap();
    let b = build_polymesh(&big, &FoamOptions::default()).unwrap();
    assert_eq!((b.mesh.n_cells, b.mesh.points.len(), b.mesh.n_internal()), (64_000, 68_921, 3 * 40 * 40 * 39));
    assert!(check_polymesh(&b.mesh, 1e-9).is_empty());
}

fn typed(mut m: ReferenceMesh, name: &str, n: usize) -> ReferenceMesh {
    MeshEdit::add_set(&mut m, name, SetMembers::Cells((0..n).map(CellIx::new).collect()), &mut SerialAlloc::new()).unwrap();
    m
}

fn smsh_hash(m: &dyn MeshRead, set: &str, ty: &str, threads: usize) -> u64 {
    in_pool(threads, || {
        let b = SmshWriter::new(m).element_type(set, ty).provenance("det").to_bytes().unwrap();
        let mut h = 0xcbf2_9ce4_8422_2325u64;
        fnv(&mut h, &b);
        h
    })
}

#[test]
fn smsh_with_element_types_is_bit_identical() {
    let hex = typed(hex_block([12, 11, 10], [1.0; 3], &mut SerialAlloc::new()).unwrap(), "block_1", 12 * 11 * 10);
    determinism::check(2, 4, |t| smsh_hash(&hex, "block_1", "HEX8", t)).unwrap();
    let poly = typed(dual_patch().unwrap().mesh, "cells", 7);
    determinism::check(2, 4, |t| smsh_hash(&poly, "cells", "POLYHEDRON", t)).unwrap();
    let big = typed(hex_block([40, 40, 40], [0.025; 3], &mut SerialAlloc::new()).unwrap(), "block_1", 64_000);
    determinism::check(1, 3, |t| smsh_hash(&big, "block_1", "HEX8", t)).unwrap();
}
