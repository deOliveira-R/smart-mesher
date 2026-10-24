//! Default `MeshBuild::add_template_cells` and `reserve`.

use sm_core::Point3;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{CellShape, MeshBuild, MeshError, VertexIx};

fn builder() -> ReferenceBuilder {
    let mut b = ReferenceBuilder::new();
    for k in 0..4 {
        for (x, y) in [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)] {
            b.add_vertex(Point3::new(x, y, k as f64)).unwrap();
        }
    }
    b
}

fn hex(k: u32) -> Vec<VertexIx> {
    [0, 1, 3, 2].into_iter().map(|o| 4 * k + o).chain([0, 1, 3, 2].into_iter().map(|o| 4 * (k + 1) + o)).map(VertexIx).collect()
}

#[test]
fn default_equals_the_loop_and_returns_the_new_range() {
    let verts: Vec<VertexIx> = (0..3).flat_map(hex).collect();
    let mut a = builder();
    assert_eq!(a.add_template_cells(&[CellShape::Hex; 3], &[0, 8, 16, 24], &verts).unwrap(), 0..3);
    let mut b = builder();
    for k in 0..3 {
        b.add_cell_from_vertices(CellShape::Hex, &hex(k)).unwrap();
    }
    assert_eq!((a.cell_count(), a.face_count()), (b.cell_count(), b.face_count()));
    for c in 0..3 {
        let c = sm_mesh_api::CellIx::new(c);
        assert_eq!(a.cell_faces(c), b.cell_faces(c));
    }
    // Appending continues the numbering.
    assert_eq!(a.add_template_cells(&[], &[0], &[]).unwrap(), 3..3);
}

#[test]
fn bad_offsets_and_bad_cells_error_like_the_loop() {
    let verts: Vec<VertexIx> = (0..3).flat_map(hex).collect();
    let mut b = builder();
    for bad in [&[0u64, 8, 16][..], &[1, 8, 16, 24], &[0, 8, 24, 16], &[0, 8, 16, 25]] {
        assert!(matches!(b.add_template_cells(&[CellShape::Hex; 3], bad, &verts), Err(MeshError::State(_))));
    }
    assert_eq!(b.cell_count(), 0, "a malformed offsets array changes nothing");
    // Second cell is a tet given eight vertices: the first stays, the error is the loop's.
    let shapes = [CellShape::Hex, CellShape::Tet, CellShape::Hex];
    let e = b.add_template_cells(&shapes, &[0, 8, 16, 24], &verts).unwrap_err();
    assert!(matches!(e, MeshError::State(_)));
    assert_eq!(b.cell_count(), 1);
}

#[test]
fn reserve_defaults_to_a_no_op() {
    let mut b = builder();
    let before = (b.vertex_count(), b.cell_count());
    b.reserve(1 << 20, 1 << 20);
    assert_eq!((b.vertex_count(), b.cell_count()), before);
}
