//! `MeshBuild::add_template_cells` / `reserve` on `MeshBuilder`: the bulk override gives the same
//! result as the default loop, including errors and the cells kept before them.

use sm_mesh::MeshBuilder;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::{CellShape, MeshBuild, VertexIx};
use sm_testkit::check::template_cells_match_default;

/// A 2x2 column of `n` hexes: points on a (2, 2, n + 1) grid.
fn column(n: usize) -> (Vec<[f64; 3]>, Vec<CellShape>, Vec<u64>, Vec<VertexIx>) {
    let mut pts = Vec::new();
    for k in 0..=n {
        for (x, y) in [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)] {
            pts.push([x, y, k as f64]);
        }
    }
    let mut verts = Vec::new();
    for k in 0..n as u32 {
        let (b, t) = (4 * k, 4 * (k + 1));
        for o in [0, 1, 3, 2] {
            verts.push(VertexIx(b + o));
        }
        for o in [0, 1, 3, 2] {
            verts.push(VertexIx(t + o));
        }
    }
    let offsets = (0..=n as u64).map(|i| i * 8).collect();
    (pts, vec![CellShape::Hex; n], offsets, verts)
}

#[test]
fn bulk_override_matches_the_default_loop() {
    let (pts, shapes, offsets, verts) = column(5);
    template_cells_match_default(MeshBuilder::new, &pts, &shapes, &offsets, &verts).unwrap();
    // The reference builder uses the default itself.
    template_cells_match_default(ReferenceBuilder::new, &pts, &shapes, &offsets, &verts).unwrap();
    // Mixed shapes.
    let mut shapes = shapes;
    let mut offsets = offsets;
    let mut verts = verts;
    shapes.push(CellShape::Tet);
    offsets.push(offsets[offsets.len() - 1] + 4);
    verts.extend([VertexIx(20), VertexIx(21), VertexIx(23), VertexIx(0)]);
    template_cells_match_default(MeshBuilder::new, &pts, &shapes, &offsets, &verts).unwrap();
}

#[test]
fn errors_and_partial_state_match_the_default() {
    let (pts, shapes, offsets, mut verts) = column(4);
    // Cell 2 repeats a vertex: cells 0 and 1 stay in both paths.
    verts[2 * 8 + 1] = verts[2 * 8];
    template_cells_match_default(MeshBuilder::new, &pts, &shapes, &offsets, &verts).unwrap();
    let mut b = MeshBuilder::new();
    for p in &pts {
        b.add_vertex(sm_core::Point3::new(p[0], p[1], p[2])).unwrap();
    }
    assert!(b.add_template_cells(&shapes, &offsets, &verts).is_err());
    assert_eq!(b.cell_count(), 2, "cells before the first bad one are kept");
    // A vertex that does not exist, bad offsets, wrong vertex count for a shape.
    let (pts, shapes, offsets, mut verts) = column(3);
    verts[9] = VertexIx(99);
    template_cells_match_default(MeshBuilder::new, &pts, &shapes, &offsets, &verts).unwrap();
    let (pts, shapes, mut offsets, verts) = column(3);
    offsets[1] = 7;
    template_cells_match_default(MeshBuilder::new, &pts, &shapes, &offsets, &verts).unwrap();
    let (pts, shapes, offsets, verts) = column(3);
    template_cells_match_default(MeshBuilder::new, &pts, &shapes, &offsets[..3], &verts).unwrap();
    let shapes = vec![CellShape::Tet; 3];
    template_cells_match_default(MeshBuilder::new, &pts, &shapes, &offsets, &verts).unwrap();
}

#[test]
fn a_non_empty_builder_falls_back_to_the_loop() {
    let (pts, shapes, offsets, verts) = column(4);
    let mut b = MeshBuilder::new();
    b.reserve(pts.len(), 4);
    for p in &pts {
        b.add_vertex(sm_core::Point3::new(p[0], p[1], p[2])).unwrap();
    }
    assert_eq!(b.add_template_cells(&shapes[..1], &offsets[..2], &verts[..8]).unwrap(), 0..1);
    // The builder holds a cell now: the inherent bulk path refuses, the trait method appends.
    assert_eq!(b.add_template_cells(&shapes[1..], &[0, 8, 16, 24], &verts[8..]).unwrap(), 1..4);
    assert_eq!((b.cell_count(), b.face_count()), (4, 21));
}

#[test]
fn reserve_changes_no_result() {
    let (pts, shapes, offsets, verts) = column(3);
    let run = |reserve: bool| {
        let mut b = MeshBuilder::new();
        if reserve {
            b.reserve(1000, 1000);
        }
        for p in &pts {
            b.add_vertex(sm_core::Point3::new(p[0], p[1], p[2])).unwrap();
        }
        b.add_template_cells(&shapes, &offsets, &verts).unwrap();
        (b.vertex_count(), b.face_count(), b.cell_count())
    };
    assert_eq!(run(true), run(false));
}
