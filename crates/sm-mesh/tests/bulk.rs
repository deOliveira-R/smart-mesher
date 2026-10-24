//! The bulk (parallel face table) path is indistinguishable from adding cells one by one.

mod common;
use common::replay::assert_same;
use sm_core::{EntityKind, Point3};
use sm_mesh::generate::{hex_block, hex_block_bulk};
use sm_mesh::{Mesh, MeshBuilder};
use sm_mesh_api::reference::{hex_block as ref_hex_block, ReferenceBuilder};
use sm_mesh_api::*;
use sm_testkit::meshes::all_mesh_fixtures;

#[test]
fn bulk_hex_block_equals_reference_and_incremental() {
    for n in [[1, 1, 1], [2, 3, 4], [5, 1, 7], [3, 3, 3]] {
        let h = [1.0, 0.5, 2.0];
        let want = ref_hex_block(n, h, &mut SerialAlloc::new()).unwrap();
        let mut a1 = SerialAlloc::new();
        let inc = hex_block(MeshBuilder::new(), n, h, false, &mut a1).unwrap();
        let mut a2 = SerialAlloc::new();
        let bulk = hex_block_bulk(n, h, &mut a2).unwrap();
        assert_same(&want, &inc.mesh, &format!("{n:?} incremental"));
        assert_same(&want, &bulk.mesh, &format!("{n:?} bulk"));
        assert_eq!(inc.face_remap, bulk.face_remap, "{n:?}: provisional numbering");
        assert_eq!(a1.next_serial(EntityKind::MeshFace), a2.next_serial(EntityKind::MeshFace));
        // Same through the reference builder (the generic generator).
        let r = hex_block(ReferenceBuilder::new(), n, h, false, &mut SerialAlloc::new()).unwrap();
        assert_eq!(r.face_remap, bulk.face_remap);
    }
}

#[test]
fn generic_generator_with_edges_equals_reference() {
    use sm_mesh_api::reference::hex_block_with_edges;
    for n in [[1, 1, 1], [2, 2, 2], [3, 1, 2]] {
        let want = hex_block_with_edges(n, [1.0; 3], &mut SerialAlloc::new()).unwrap();
        let got = hex_block(MeshBuilder::new(), n, [1.0; 3], true, &mut SerialAlloc::new()).unwrap();
        assert_same(&want, &got.mesh, &format!("{n:?}"));
    }
}

/// Build `src`'s template cells either one by one or in bulk, classified like `src`.
fn rebuild(src: &dyn MeshRead, bulk: bool) -> Result<Finished<Mesh>> {
    let mut b = MeshBuilder::new();
    let co = src.coords();
    for i in 0..src.vertex_count() {
        let v = b.add_vertex(co.get(i))?;
        b.set_class(MeshEntityRef::Vertex(v), src.vertex_class(v))?;
    }
    let cells: Vec<(CellShape, Vec<VertexIx>)> = (0..src.cell_count())
        .map(|c| {
            let c = CellIx::new(c);
            (src.cell_shape(c), src.cell_corners(c).unwrap().to_vec())
        })
        .collect();
    if bulk {
        let shapes: Vec<CellShape> = cells.iter().map(|c| c.0).collect();
        let mut offsets = vec![0u64];
        let mut verts = Vec::new();
        for (_, vs) in &cells {
            verts.extend_from_slice(vs);
            offsets.push(verts.len() as u64);
        }
        b.add_cells_bulk(&shapes, &offsets, &verts)?;
    } else {
        for (s, vs) in &cells {
            b.add_cell_from_vertices(*s, vs)?;
        }
    }
    for c in 0..src.cell_count() {
        b.set_class(MeshEntityRef::Cell(CellIx::new(c)), src.cell_class(CellIx::new(c)))?;
    }
    for f in 0..src.face_count() {
        let fi = FaceIx::new(f);
        let (p, _) = b.find_face(src.face_vertices(fi)).unwrap();
        b.set_class(MeshEntityRef::Face(p), src.face_class(fi))?;
    }
    b.finish(&mut SerialAlloc::new())
}

#[test]
fn bulk_equals_incremental_on_every_template_fixture() {
    for fx in all_mesh_fixtures().unwrap() {
        if (0..fx.mesh.cell_count()).any(|c| fx.mesh.cell_corners(CellIx::new(c)).is_none()) {
            continue; // polyhedra have no bulk form
        }
        let inc = rebuild(&fx.mesh, false).unwrap();
        let blk = rebuild(&fx.mesh, true).unwrap();
        assert_same(&inc.mesh, &blk.mesh, fx.name);
        assert_eq!(inc.face_remap, blk.face_remap, "{}", fx.name);
        assert_same(&fx.mesh, &blk.mesh, fx.name);
    }
}

fn unit(b: &mut MeshBuilder, nx: usize) -> Vec<VertexIx> {
    let mut vs = Vec::new();
    for i in 0..=nx {
        for (y, z) in [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)] {
            vs.push(b.add_vertex(Point3::new(i as f64, y, z)).unwrap());
        }
    }
    vs
}

/// A hex from slab `i` to `i + 1` of a 4-points-per-slab lattice.
fn slab(vs: &[VertexIx], i: usize, flip: bool) -> Vec<VertexIx> {
    let s = |k: usize, q: usize| vs[4 * k + q];
    let mut h = vec![s(i, 0), s(i + 1, 0), s(i + 1, 1), s(i, 1), s(i, 2), s(i + 1, 2), s(i + 1, 3), s(i, 3)];
    if flip {
        h.swap(1, 3);
        h.swap(5, 7);
    }
    h
}

#[test]
fn bulk_errors_match_incremental_and_leave_the_builder_unchanged() {
    // Inconsistent orientation: cell 1 is the mirror image of a neighbour-compatible hex.
    for case in 0..2 {
        let mut a = MeshBuilder::new();
        let vs = unit(&mut a, 3);
        let cells = [slab(&vs, 0, false), slab(&vs, 1, case == 1), slab(&vs, 2, false)];
        let inc_err = {
            let mut r = None;
            for c in &cells {
                if let Err(e) = a.add_cell_from_vertices(CellShape::Hex, c) {
                    r = Some(e);
                    break;
                }
            }
            r
        };
        let mut b = MeshBuilder::new();
        let vs2 = unit(&mut b, 3);
        assert_eq!(vs, vs2);
        let flat: Vec<VertexIx> = cells.iter().flatten().copied().collect();
        let res = b.add_uniform_cells(CellShape::Hex, &flat);
        assert_eq!(res.clone().err(), inc_err, "case {case}");
        assert_eq!(inc_err.is_some(), case == 1, "case {case} must fail iff flipped");
        if res.is_err() {
            assert_eq!((b.cell_count(), b.face_count()), (0, 0), "builder changed on error");
            // ... and is still usable.
            b.add_uniform_cells(CellShape::Hex, &slab(&vs, 0, false)).unwrap();
            assert_eq!((b.cell_count(), b.face_count()), (1, 6));
        }
    }
    // A face shared by three cells.
    let mut a = MeshBuilder::new();
    let vs = unit(&mut a, 1);
    let top = |z: f64| -> Vec<VertexIx> {
        let _ = z;
        slab(&vs, 0, false)
    };
    let mut flat = top(0.0);
    flat.extend(top(0.0));
    flat.extend(top(0.0));
    let mut inc = MeshBuilder::new();
    unit(&mut inc, 1);
    let mut err = None;
    for k in 0..3 {
        if let Err(e) = inc.add_cell_from_vertices(CellShape::Hex, &flat[k * 8..k * 8 + 8]) {
            err = Some(e);
            break;
        }
    }
    let res = a.add_uniform_cells(CellShape::Hex, &flat);
    assert!(matches!(err, Some(MeshError::InconsistentOrientation(_) | MeshError::NonManifoldFace(_))));
    assert_eq!(res.err(), err);
    // Bulk input checks.
    let mut b = MeshBuilder::new();
    unit(&mut b, 1);
    let bad = [VertexIx(0), VertexIx(1), VertexIx(2), VertexIx(3), VertexIx(4), VertexIx(5), VertexIx(6), VertexIx(99)];
    assert!(matches!(b.add_uniform_cells(CellShape::Hex, &bad), Err(MeshError::UnknownVertex(_))));
    let dup = [VertexIx(0), VertexIx(0), VertexIx(2), VertexIx(3), VertexIx(4), VertexIx(5), VertexIx(6), VertexIx(7)];
    assert!(b.add_uniform_cells(CellShape::Hex, &dup).is_err());
    assert!(b.add_uniform_cells(CellShape::Polyhedron, &dup).is_err());
    assert!(b.add_cells_bulk(&[CellShape::Hex], &[0, 3], &dup[..3]).is_err());
    // Not on a non-empty builder.
    b.add_uniform_cells(CellShape::Hex, &slab(&unit_vs(), 0, false)).ok();
}

fn unit_vs() -> Vec<VertexIx> {
    (0..8).map(VertexIx).collect()
}

#[test]
fn bulk_then_incremental_calls_work_together() {
    // find_face must see the faces created by the bulk path.
    let mut b = MeshBuilder::new();
    let vs = unit(&mut b, 2);
    b.add_uniform_cells(CellShape::Hex, &slab(&vs, 0, false)).unwrap();
    let shared: Vec<VertexIx> = vec![vs[4], vs[6], vs[7], vs[5]];
    let (_, rel) = b.find_face(&shared).expect("bulk faces are findable");
    assert!(matches!(rel, LoopRelation::Same | LoopRelation::Reversed));
    let added = b.add_cell_from_vertices(CellShape::Hex, &slab(&vs, 1, false)).unwrap();
    assert_eq!(added.reused.iter().filter(|r| **r).count(), 1);
    assert!(b.add_uniform_cells(CellShape::Hex, &slab(&vs, 1, false)).is_err());
}
