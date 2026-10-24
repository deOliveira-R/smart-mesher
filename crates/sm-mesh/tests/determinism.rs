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

// ------------------------------------------------------------- multi-chunk parallel paths
//
// `finish` chunks entities by 4096 and the bulk run resolution by 32768 face records, so these
// meshes (> 4096 cells, > 32768 records) run the per-chunk merges, the chunk bounds moved to run
// boundaries and the cross-chunk error minimum on more than one chunk.

#[test]
fn large_bulk_block_spans_several_chunks_and_is_bit_identical() {
    // 24 x 20 x 18 = 8640 cells, 51840 face records.
    check(3, 3, |t| {
        pooled(t, || {
            let f = hex_block_bulk([24, 20, 18], [1.0, 0.5, 0.25], &mut SerialAlloc::new()).unwrap();
            assert_eq!(f.mesh.cell_count(), 8640);
            fingerprint(&f.mesh, &f.face_remap)
        })
    })
    .unwrap();
}

#[test]
fn large_incremental_block_with_edges_spans_several_chunks_and_is_bit_identical() {
    // 20 x 18 x 16 = 5760 cells.
    check(2, 3, |t| {
        pooled(t, || {
            let f = hex_block(MeshBuilder::new(), [20, 18, 16], [1.0; 3], true, &mut SerialAlloc::new()).unwrap();
            fingerprint(&f.mesh, &f.face_remap)
        })
    })
    .unwrap();
}

/// Deterministic Fisher-Yates shuffle (splitmix-style generator seeded from `seed`).
fn shuffled(n: usize, seed: u64) -> Vec<usize> {
    let mut p: Vec<usize> = (0..n).collect();
    let mut s = seed;
    for i in (1..n).rev() {
        s = s.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = s;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^= z >> 31;
        p.swap(i, (z % (i as u64 + 1)) as usize);
    }
    p
}

/// A hex lattice with randomly numbered vertices and cells in random order, built either in bulk
/// or cell by cell; every face is classified (boundary faces on one model face).
fn shuffled_block(n: [usize; 3], bulk: bool) -> Finished<Mesh> {
    use sm_core::{EntityId, EntityKind, Point3};
    let m = [n[0] + 1, n[1] + 1, n[2] + 1];
    let nv = m[0] * m[1] * m[2];
    let vperm = shuffled(nv, 11);
    let ix = |i: usize, j: usize, k: usize| VertexIx::new(vperm[(k * m[1] + j) * m[0] + i]);
    let mut cells: Vec<[VertexIx; 8]> = Vec::new();
    for k in 0..n[2] {
        for j in 0..n[1] {
            for i in 0..n[0] {
                cells.push([
                    ix(i, j, k),
                    ix(i + 1, j, k),
                    ix(i + 1, j + 1, k),
                    ix(i, j + 1, k),
                    ix(i, j, k + 1),
                    ix(i + 1, j, k + 1),
                    ix(i + 1, j + 1, k + 1),
                    ix(i, j + 1, k + 1),
                ]);
            }
        }
    }
    let order = shuffled(cells.len(), 29);
    let cells: Vec<[VertexIx; 8]> = order.iter().map(|&c| cells[c]).collect();
    let on = |k, s| Classification::on(EntityId::new(k, s).unwrap());
    let mut b = MeshBuilder::new();
    let mut pts = vec![Point3::new(0.0, 0.0, 0.0); nv];
    for k in 0..m[2] {
        for j in 0..m[1] {
            for i in 0..m[0] {
                pts[vperm[(k * m[1] + j) * m[0] + i]] = Point3::new(i as f64, j as f64, k as f64);
            }
        }
    }
    for p in &pts {
        let v = b.add_vertex(*p).unwrap();
        b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1)).unwrap();
    }
    if bulk {
        let flat: Vec<VertexIx> = cells.iter().flatten().copied().collect();
        b.add_uniform_cells(CellShape::Hex, &flat).unwrap();
    } else {
        for c in &cells {
            b.add_cell_from_vertices(CellShape::Hex, c).unwrap();
        }
    }
    for c in 0..b.cell_count() {
        b.set_class(MeshEntityRef::Cell(CellIx::new(c)), on(EntityKind::TopoVolume, 1)).unwrap();
    }
    for f in 0..b.face_count() {
        if b.face_degree(FaceIx::new(f)) == 1 {
            b.set_class(MeshEntityRef::Face(FaceIx::new(f)), on(EntityKind::TopoFace, 1)).unwrap();
        }
    }
    b.finish(&mut SerialAlloc::new()).unwrap()
}

#[test]
fn shuffled_numbering_bulk_and_incremental_agree_and_are_bit_identical_across_threads() {
    // 22 x 20 x 18 = 7920 cells, 47520 face records.
    let n = [22, 20, 18];
    let want = pooled(1, || {
        let f = shuffled_block(n, true);
        fingerprint(&f.mesh, &f.face_remap)
    });
    for t in [1, 3, 10] {
        for bulk in [true, false] {
            let got = pooled(t, || {
                let f = shuffled_block(n, bulk);
                fingerprint(&f.mesh, &f.face_remap)
            });
            assert_eq!(got, want, "{t} threads, bulk={bulk}");
        }
    }
}

#[test]
fn large_bulk_error_is_the_lowest_one_at_every_thread_count() {
    // Cells 3000 and 7000 reference an unknown vertex; 5000 duplicates one. Lowest wins.
    let n = 8640usize;
    let mut verts: Vec<VertexIx> = (0..n * 8).map(VertexIx::new).collect();
    verts[7000 * 8 + 2] = VertexIx(u32::MAX - 5);
    verts[3000 * 8 + 5] = VertexIx(u32::MAX - 6);
    verts[5000 * 8 + 1] = verts[5000 * 8];
    let run = || {
        let mut b = MeshBuilder::new();
        b.add_vertices_with(n * 8, |i| sm_core::Point3::new(i as f64, 0.0, 0.0)).unwrap();
        b.add_uniform_cells(CellShape::Hex, &verts).err()
    };
    for t in [1, 3, 8] {
        let e = pooled(t, run);
        assert_eq!(e, Some(MeshError::UnknownVertex(VertexIx(u32::MAX - 6))), "{t} threads");
    }
}
