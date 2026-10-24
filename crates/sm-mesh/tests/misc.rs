//! Zero-copy views, ids, bulk edge cases, thread-safety markers.

mod common;
use common::replay::assert_same;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh::generate::{hex_block, hex_block_bulk};
use sm_mesh::{Catalog, Mesh, MeshBuilder, Validation, as_bytes};
use sm_mesh_api::*;

fn assert_send_sync<T: Send + Sync>() {}

#[test]
fn stores_are_thread_safe() {
    assert_send_sync::<Mesh>();
    assert_send_sync::<MeshBuilder>();
    assert_send_sync::<Catalog>();
    assert_send_sync::<sm_mesh::Csr>();
}

#[test]
fn raw_columns_are_zero_copy_views_of_the_store() {
    let m = hex_block_bulk([4, 3, 2], [1.0, 2.0, 3.0], &mut SerialAlloc::new()).unwrap().mesh;
    let raw = m.raw_columns();
    let cols = m.columns();
    // Same memory, not a copy.
    assert_eq!(raw.face_vertex_offsets.as_ptr(), cols.face_vertex_offsets.as_ptr());
    assert_eq!(raw.cell_face_offsets.as_ptr(), cols.cell_face_offsets.as_ptr());
    assert_eq!(raw.face_right.as_ptr(), cols.face_right.as_ptr());
    assert_eq!(raw.x.as_ptr(), m.coords().x.as_ptr());
    // Byte views have the on-disk widths and round-trip.
    assert_eq!(as_bytes(raw.x).len(), 8 * m.coords().len());
    assert_eq!(as_bytes(raw.face_vertex_offsets).len(), 8 * (m.face_count() + 1));
    assert_eq!(as_bytes(raw.face_right).len(), 4 * m.face_count());
    assert_eq!(as_bytes(raw.x).as_ptr() as usize, raw.x.as_ptr() as usize);
    let back: &[f64] = bytemuck::cast_slice(as_bytes(raw.y));
    assert_eq!(back, raw.y);
    // CSR consistency of the offsets.
    assert_eq!(*raw.face_vertex_offsets.last().unwrap() as usize, cols.face_vertices.len());
    assert_eq!(*raw.cell_face_offsets.last().unwrap() as usize, cols.cell_faces.len());
    assert_eq!(raw.corner_offsets.len(), m.cell_count() + 1);
    assert!(raw.cell_parent.iter().all(|p| *p == NO_CELL));
    // Boundary faces of a 4x3x2 block: 2 (4*3 + 4*2 + 3*2).
    assert_eq!(raw.face_right.iter().filter(|r| **r == NO_CELL).count(), 2 * (4 * 3 + 4 * 2 + 3 * 2));
}

#[test]
fn memory_is_compact_and_grows_linearly() {
    let small = hex_block_bulk([8; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap().mesh;
    let big = hex_block_bulk([16; 3], [1.0; 3], &mut SerialAlloc::new()).unwrap().mesh;
    let per = |m: &Mesh| m.memory_bytes() as f64 / m.cell_count() as f64;
    assert!(per(&big) < per(&small), "per-cell cost falls as the surface/volume ratio does");
    assert!(per(&big) < 330.0, "{} bytes per cell", per(&big));
}

#[test]
fn ids_are_implicit_consecutive_and_disjoint_across_meshes() {
    let mut alloc = SerialAlloc::new();
    let a = hex_block(MeshBuilder::new(), [2, 2, 1], [1.0; 3], false, &mut alloc).unwrap().mesh;
    let b = hex_block(MeshBuilder::new(), [1, 1, 1], [1.0; 3], false, &mut alloc).unwrap().mesh;
    assert_ne!(a.mesh_id(), b.mesh_id());
    for k in [MeshEntityKind::Vertex, MeshEntityKind::Face, MeshEntityKind::Cell] {
        for i in 0..a.count(k) {
            let id = a.id(k, i as u32);
            assert_eq!(a.index_of(id), Some(i as u32));
            assert_eq!(b.index_of(id), None, "foreign id");
        }
        assert!(a.id(k, a.count(k) as u32 - 1) < b.id(k, 0), "ranges are disjoint and ordered");
    }
    // Wrong kinds and out-of-range serials are not found.
    assert_eq!(a.index_of(EntityId::new(EntityKind::TopoFace, 1).unwrap()), None);
    assert_eq!(a.index_of(EntityId::new(EntityKind::MeshCell, 999).unwrap()), None);
    assert_eq!(a.index_of(EntityId::new(EntityKind::MeshEdge, 1).unwrap()), None);
}

#[test]
fn fast_and_full_validation_build_the_same_mesh() {
    let build = |v| {
        let mut b = MeshBuilder::new().with_validation(v);
        // A 3x3x3 block through the generic generator.
        let m = [4usize, 4, 4];
        let vol = Classification::on(EntityId::new(EntityKind::TopoVolume, 1).unwrap());
        for k in 0..m[2] {
            for j in 0..m[1] {
                for i in 0..m[0] {
                    let v = b.add_vertex(Point3::new(i as f64, j as f64, k as f64)).unwrap();
                    b.set_class(MeshEntityRef::Vertex(v), vol).unwrap();
                }
            }
        }
        for k in 0..3usize {
            for j in 0..3usize {
                for i in 0..3usize {
                    let ix = |i: usize, j: usize, k: usize| VertexIx::new((k * 4 + j) * 4 + i);
                    let vs = [ix(i, j, k), ix(i + 1, j, k), ix(i + 1, j + 1, k), ix(i, j + 1, k), ix(i, j, k + 1), ix(i + 1, j, k + 1), ix(i + 1, j + 1, k + 1), ix(i, j + 1, k + 1)];
                    let c = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
                    b.set_class(MeshEntityRef::Cell(c.cell), vol).unwrap();
                }
            }
        }
        for f in 0..b.face_count() {
            if b.face_degree(FaceIx::new(f)) == 1 {
                b.set_class(MeshEntityRef::Face(FaceIx::new(f)), Classification::on(EntityId::new(EntityKind::TopoFace, 1).unwrap())).unwrap();
            }
        }
        b.finish(&mut SerialAlloc::new())
    };
    // Every vertex on the volume while boundary faces lie on a TopoFace: dimension order is
    // violated, and both modes must say so with the same report.
    let (fast, full) = (build(Validation::Fast), build(Validation::Full));
    match (fast, full) {
        (Err(a), Err(b)) => {
            assert_eq!(a, b);
            assert!(matches!(a, MeshError::Invalid(v) if v.iter().any(|x| x.code == ViolationCode::ClassDimOrder)));
        }
        _ => panic!("expected the dimension-order violation"),
    }
    // A valid block: identical meshes.
    let f = hex_block(MeshBuilder::new().with_validation(Validation::Fast), [3, 3, 3], [1.0; 3], false, &mut SerialAlloc::new()).unwrap();
    let u = hex_block(MeshBuilder::new().with_validation(Validation::Full), [3, 3, 3], [1.0; 3], false, &mut SerialAlloc::new()).unwrap();
    assert_same(&f.mesh, &u.mesh, "fast vs full");
}

#[test]
fn bulk_edge_cases() {
    // Zero cells is fine and leaves the builder usable.
    let mut b = MeshBuilder::new();
    for p in [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]] {
        b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
    }
    assert_eq!(b.add_uniform_cells(CellShape::Hex, &[]).unwrap(), 0..0);
    let vs: Vec<VertexIx> = (0..8).map(VertexIx).collect();
    assert_eq!(b.add_uniform_cells(CellShape::Hex, &vs).unwrap(), 0..1);
    // A second bulk call is refused (the builder holds cells now) and changes nothing.
    assert!(matches!(b.add_uniform_cells(CellShape::Hex, &vs), Err(MeshError::State(_))));
    assert_eq!((b.cell_count(), b.face_count()), (1, 6));
    // Offsets that are not a CSR array over the vertices.
    let mut c = MeshBuilder::new();
    for p in 0..8 {
        c.add_vertex(Point3::new(p as f64, 0.0, 0.0)).unwrap();
    }
    assert!(c.add_cells_bulk(&[CellShape::Hex], &[0, 9], &vs).is_err());
    assert!(c.add_cells_bulk(&[CellShape::Hex], &[1, 8], &vs).is_err());
    assert!(c.add_cells_bulk(&[CellShape::Hex, CellShape::Hex], &[0, 8], &vs).is_err());
    // classify_*_with: bad classes are reported at the lowest index and change nothing.
    let bad = Classification::on(EntityId::new(EntityKind::MeshCell, 1).unwrap());
    let good = Classification::on(EntityId::new(EntityKind::TopoVertex, 1).unwrap());
    let r = c.classify_vertices_with(|i| if i >= 3 { Some(bad) } else { Some(good) });
    assert!(matches!(r, Err(MeshError::BadClassification(_))));
    assert!(c.vertex_class(VertexIx(0)).is_none(), "nothing applied on error");
    c.classify_vertices_with(|i| (i % 2 == 0).then_some(good)).unwrap();
    assert!(c.vertex_class(VertexIx(0)).is_some() && c.vertex_class(VertexIx(1)).is_none());
    // Cells must lie on volumes in a dimension-3 builder.
    c.add_uniform_cells(CellShape::Hex, &vs).unwrap();
    assert!(c.classify_cells_with(|_| Some(good)).is_err());
}
