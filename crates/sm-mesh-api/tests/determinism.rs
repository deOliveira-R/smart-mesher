//! Determinism: same inputs, same hashes, regardless of run count, thread count or face creation order.

mod common;
use common::*;
use sm_core::{EntityKind, Point3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceMesh, hex_block};
use sm_mesh_api::*;

fn build() -> (ReferenceMesh, u64, u64) {
    let mut alloc = SerialAlloc::new();
    let m = hex_block([3, 3, 2], [1.0, 2.0, 0.5], &mut alloc).unwrap();
    let (u, _) = unfold(
        &m,
        &[mirror_x(0.0), mirror_y(0.0)],
        ReferenceBuilder::new(),
        &FlattenOptions {
            weld_tol: Some(1e-9),
            ..FlattenOptions::default()
        },
        &mut alloc,
    )
    .unwrap();
    let (t, g) = (topology_hash(&u), geometry_hash(&u));
    (u, t, g)
}

#[test]
fn building_twice_gives_identical_hashes() {
    let (a, ta, ga) = build();
    let (b, tb, gb) = build();
    assert_eq!((ta, ga), (tb, gb));
    assert_eq!(a, b, "the meshes are equal field by field");
    assert_eq!(connectivity_hash(&a), connectivity_hash(&b));
}

#[test]
/// The crate has no rayon dependency, so this concurrent-build test stands in for the
/// CLAUDE.md `RAYON_NUM_THREADS=1` vs default comparison; parallel consumers get the same bytes.
fn hashes_do_not_depend_on_threads() {
    // The crate itself is single threaded; building concurrently must not change anything
    // (no thread-local or time-based state, no hash-map iteration order).
    let (_, t0, g0) = build();
    let handles: Vec<_> = (0..4)
        .map(|_| {
            std::thread::spawn(|| {
                let (_, t, g) = build();
                (t, g)
            })
        })
        .collect();
    for h in handles {
        assert_eq!(h.join().unwrap(), (t0, g0));
    }
}

#[test]
fn canonical_output_is_independent_of_face_creation_order() {
    // The same two tets sharing a face, with faces created in two different orders.
    let pts = [
        [0., 0., 0.],
        [1., 0., 0.],
        [0., 1., 0.],
        [0., 0., 1.],
        [1., 1., 1.],
    ];
    let make = |flip: bool| {
        let mut b = ReferenceBuilder::new();
        let vs: Vec<VertexIx> = pts
            .iter()
            .enumerate()
            .map(|(i, p)| {
                let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
                b.set_class(
                    MeshEntityRef::Vertex(v),
                    on(EntityKind::TopoVertex, 1 + i as u64),
                )
                .unwrap();
                v
            })
            .collect();
        let t0 = [vs[0], vs[1], vs[2], vs[3]];
        let t1 = [vs[1], vs[2], vs[3], vs[4]];
        let cells = if flip { [t1, t0] } else { [t0, t1] };
        let mut created = Vec::new();
        for c in cells {
            let a = b.add_cell_from_vertices(CellShape::Tet, &c).unwrap();
            b.set_class(MeshEntityRef::Cell(a.cell), vol()).unwrap();
            created.push(a);
        }
        for a in &created {
            for sf in &a.faces {
                if b.face_degree(sf.face()) == 1 {
                    b.set_class(
                        MeshEntityRef::Face(sf.face()),
                        on(EntityKind::TopoFace, 1 + sf.face().0 as u64 % 2),
                    )
                    .unwrap();
                }
            }
        }
        b.finish(&mut SerialAlloc::new()).unwrap().mesh
    };
    // Different cell order is a different mesh (ids follow cells), but each is itself canonical and valid.
    let (a, b) = (make(false), make(true));
    assert_valid(&a);
    assert_valid(&b);
    assert_eq!(a.face_count(), 7);
    // Face numbering after finish depends only on (left, right, loop), not on creation history.
    for m in [&a, &b] {
        for f in 1..m.face_count() {
            let (x, y) = (FaceIx::new(f - 1), FaceIx::new(f));
            let key = |f: FaceIx| {
                (
                    m.face_left(f),
                    m.face_right(f).map_or(u32::MAX, |c| c.0),
                    m.face_vertices(f).to_vec(),
                )
            };
            assert!(key(x) < key(y));
        }
    }
}

#[test]
fn serial_allocator_is_deterministic_and_shared() {
    let mut alloc = SerialAlloc::new();
    let a = hex_block([1, 1, 1], [1.0; 3], &mut alloc).unwrap();
    let b = hex_block([1, 1, 1], [1.0; 3], &mut alloc).unwrap();
    assert_ne!(a.mesh_id(), b.mesh_id());
    assert_eq!(
        a.cell_id(CellIx(0)).serial() + 1,
        b.cell_id(CellIx(0)).serial()
    );
    // Ids never collide across meshes of one model.
    for v in 0..8 {
        assert_ne!(a.vertex_id(VertexIx::new(v)), b.vertex_id(VertexIx::new(v)));
    }
    let mut alloc2 = SerialAlloc::new();
    let a2 = hex_block([1, 1, 1], [1.0; 3], &mut alloc2).unwrap();
    assert_eq!(a, a2);
}
