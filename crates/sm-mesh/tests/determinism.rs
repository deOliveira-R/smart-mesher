//! Determinism: identical bits for 1 thread and N threads and across repeated runs, for the
//! meshes, the face remap, the edge table, the adjacency tables, and the reported errors.

mod common;
use common::replay::{Mode, replay};
use sm_mesh::generate::{hex_block, hex_block_bulk};
use sm_mesh::{Mesh, MeshBuilder};
use sm_mesh_api::*;
use sm_testkit::check::mesh_hash;
use sm_testkit::determinism::{Fnv, check};
use std::hash::Hasher;
use sm_testkit::meshes::all_mesh_fixtures;

/// Run `f` in a rayon pool of `threads` workers.
fn pooled<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("pool").install(f)
}

fn fingerprint(m: &Mesh, remap: &[FaceIx]) -> u64 {
    let mut h = Fnv::default();
    h.write_u64(mesh_hash(m));
    for f in remap {
        h.write_u64(u64::from(f.0));
    }
    for r in [m.vertex_cells(), m.vertex_faces(), m.cell_neighbors()] {
        for o in r.offsets() {
            h.write_u64(*o);
        }
        for i in r.indices() {
            h.write_u64(u64::from(*i));
        }
    }
    if let Some(e) = m.edge_faces() {
        for i in e.indices() {
            h.write_u64(u64::from(*i));
        }
    }
    h.finish()
}

#[test]
fn bulk_block_is_bit_identical_across_threads_and_runs() {
    let got = check(3, 4, |t| {
        pooled(t, || {
            let f = hex_block_bulk([13, 11, 9], [1.0, 0.5, 0.25], &mut SerialAlloc::new()).unwrap();
            fingerprint(&f.mesh, &f.face_remap)
        })
    })
    .unwrap();
    assert_ne!(got, 0);
}

#[test]
fn incremental_block_with_edges_is_bit_identical_across_threads_and_runs() {
    check(2, 4, |t| {
        pooled(t, || {
            let f = hex_block(MeshBuilder::new(), [9, 8, 7], [1.0; 3], true, &mut SerialAlloc::new()).unwrap();
            fingerprint(&f.mesh, &f.face_remap)
        })
    })
    .unwrap();
}

#[test]
fn bulk_and_incremental_agree_at_every_thread_count() {
    let reference = {
        let f = hex_block(MeshBuilder::new(), [6, 5, 4], [1.0; 3], false, &mut SerialAlloc::new()).unwrap();
        fingerprint(&f.mesh, &f.face_remap)
    };
    for t in [1, 2, 3, 8] {
        let got = pooled(t, || {
            let f = hex_block_bulk([6, 5, 4], [1.0; 3], &mut SerialAlloc::new()).unwrap();
            fingerprint(&f.mesh, &f.face_remap)
        });
        assert_eq!(got, reference, "{t} threads");
    }
}

#[test]
fn fixtures_are_bit_identical_across_threads() {
    for fx in all_mesh_fixtures().unwrap() {
        for mode in [Mode::Explicit, Mode::Mesher] {
            check(2, 4, |t| {
                pooled(t, || {
                    let (f, prov) = replay(&fx.mesh, MeshBuilder::new(), mode, &mut SerialAlloc::new()).unwrap();
                    let mut h = Fnv::default();
                    h.write_u64(fingerprint(&f.mesh, &f.face_remap));
                    for p in prov {
                        h.write_u64(u64::from(p.0));
                    }
                    h.finish()
                })
            })
            .unwrap_or_else(|e| panic!("{} {mode:?}: {e:?}", fx.name));
        }
    }
}

#[test]
fn errors_do_not_depend_on_the_thread_count() {
    // Several independent defects: the lowest one must always be the one reported.
    let build = || {
        let mut b = MeshBuilder::new();
        let n = [20usize, 20, 20];
        let m = [n[0] + 1, n[1] + 1, n[2] + 1];
        for k in 0..m[2] {
            for j in 0..m[1] {
                for i in 0..m[0] {
                    let v = b.add_vertex(sm_core::Point3::new(i as f64, j as f64, k as f64)).unwrap();
                    // Unclassified vertices at several places: the first is vertex 5.
                    if v.get() % 977 != 5 {
                        b.set_class(MeshEntityRef::Vertex(v), Classification::on(
                            sm_core::EntityId::new(sm_core::EntityKind::TopoVolume, 1).unwrap(),
                        ))
                        .unwrap();
                    }
                }
            }
        }
        b.finish(&mut SerialAlloc::new()).err()
    };
    let want = check(2, 4, |t| pooled(t, build)).unwrap();
    assert_eq!(want, Some(MeshError::Unclassified(MeshEntity::Vertex(VertexIx(5)))));
}

#[test]
fn many_concurrent_builds_share_no_state() {
    // `check` runs `threads` concurrent `f(1)` calls on std threads: any hidden global state
    // (a shared pool, a static) would show up as a mismatch.
    check(1, 6, |_| {
        let f = hex_block_bulk([5, 5, 5], [1.0; 3], &mut SerialAlloc::new()).unwrap();
        fingerprint(&f.mesh, &f.face_remap)
    })
    .unwrap();
}
