//! W0f gaps4: extrusion maps (base mesh x layers) and the unfold deck check.

mod common;
use common::*;
use sm_core::{EntityKind, Point3, Transform, Vector3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceCatalog, ReferenceMesh, hex_block};
use sm_mesh_api::*;

/// Two unit quads `[0,2] x [0,1]` in the plane `z = 0`, classified like a CAD sheet.
fn base_strip(alloc: &mut SerialAlloc) -> ReferenceMesh {
    let mut b = ReferenceBuilder::surface();
    let pts = [[0., 0.], [1., 0.], [2., 0.], [0., 1.], [1., 1.], [2., 1.]];
    let vs: Vec<VertexIx> = pts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], 0.0)).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, 1 + i as u64)).unwrap();
            v
        })
        .collect();
    for q in [[0, 1, 4, 3], [1, 2, 5, 4]] {
        let c = b.add_polygon(&q.map(|i| vs[i])).unwrap();
        b.set_class(MeshEntityRef::Cell(c.cell), on(EntityKind::TopoFace, 1)).unwrap();
    }
    for f in 0..b.face_count() {
        let fi = FaceIx::new(f);
        let cls = if b.face_degree(fi) == 1 { on(EntityKind::TopoEdge, 1 + f as u64) } else { on(EntityKind::TopoFace, 1) };
        b.set_class(MeshEntityRef::Face(fi), cls).unwrap();
    }
    b.finish(alloc).unwrap().mesh
}

/// `(base cell, layer)` of every cell of a `2 x 1 x 3` unit hex block over [`base_strip`].
fn cell_of(ext: &ReferenceMesh) -> Vec<(CellIx, u32)> {
    (0..ext.cell_count())
        .map(|c| {
            let p = centre(ext, CellIx::new(c));
            (CellIx::new(p.x.floor() as usize), p.z.floor() as u32)
        })
        .collect()
}

#[test]
fn extrusion_map_links_base_cells_and_layers() {
    let mut alloc = SerialAlloc::new();
    let base = base_strip(&mut alloc);
    let ext = hex_block([2, 1, 3], [1.0; 3], &mut alloc).unwrap();
    let z = vec![0.0, 1.0, 2.0, 3.0];
    let map = ExtrusionMap::new(&base, &ext, z.clone(), cell_of(&ext)).unwrap();
    assert_eq!(map.n_layers(), 3);
    assert_eq!((map.base_id(), map.extruded_id()), (base.mesh_id(), ext.mesh_id()));
    for c in 0..ext.cell_count() {
        let (b, k) = map.cell_of(CellIx::new(c));
        assert_eq!(map.cell_at(b, k), CellIx::new(c));
    }
    map.check_meshes(&base, &ext).unwrap();
    assert!(map.check_meshes(&ext, &base).is_err());
    // Face roles, derived on demand: 2 axial faces per level, the rest lateral.
    let mut axial = [0usize; 4];
    let mut lateral = [0usize; 3];
    for f in 0..ext.face_count() {
        match map.face_role(&ext, FaceIx::new(f)).unwrap() {
            ExtrusionFaceRole::Axial { level } => axial[level as usize] += 1,
            ExtrusionFaceRole::Lateral { layer } => lateral[layer as usize] += 1,
        }
    }
    assert_eq!(axial, [2; 4]);
    // Per layer: 2 cells x 4 sides, minus the shared one counted once = 7.
    assert_eq!(lateral, [7; 3]);
    assert!(map.face_role(&base, FaceIx::new(0)).is_err());

    // Validation.
    let mut dup = cell_of(&ext);
    dup[1] = dup[0];
    assert!(ExtrusionMap::new(&base, &ext, z.clone(), dup).is_err(), "a (base, layer) twice");
    assert!(ExtrusionMap::new(&base, &ext, vec![0.0, 1.0, 2.0, 4.0], cell_of(&ext)).is_err(), "vertices off the levels");
    assert!(ExtrusionMap::new(&base, &ext, vec![0.0, 2.0, 1.0, 3.0], cell_of(&ext)).is_err(), "not monotone");
    assert!(ExtrusionMap::new(&base, &ext, vec![0.0, 1.0, 2.0], cell_of(&ext)).is_err(), "wrong layer count");
    assert!(ExtrusionMap::new(&ext, &base, z.clone(), cell_of(&ext)).is_err(), "dimensions swapped");
    let mut wrong = cell_of(&ext);
    for e in &mut wrong {
        e.0 = CellIx::new(1 - e.0.get());
    }
    assert!(ExtrusionMap::new(&base, &ext, z.clone(), wrong).is_err(), "cells over the wrong base cell");

    // Catalog.
    let mut cat = ReferenceCatalog::new();
    let (bid, eid) = (base.mesh_id(), ext.mesh_id());
    assert!(cat.link_extrusion(map.clone()).is_err(), "unknown meshes");
    cat.insert_mesh(Box::new(base)).unwrap();
    cat.insert_mesh(Box::new(ext)).unwrap();
    assert!(cat.extrusions_of(bid).is_empty());
    cat.link_extrusion(map.clone()).unwrap();
    assert_eq!(cat.extrusions_of(bid).len(), 1);
    assert_eq!(cat.extrusions_of(eid), vec![&map]);
    assert!(cat.link_extrusion(map).is_err(), "second map for the pair");
}

fn rot_z90() -> Transform {
    Transform { m: [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) }
}

#[test]
fn unfold_rejects_decks_that_are_not_commuting_involutions() {
    let mut alloc = SerialAlloc::new();
    let m = hex_block([1, 1, 1], [1.0; 3], &mut alloc).unwrap();
    let o = FlattenOptions { weld_tol: Some(1e-9), ..FlattenOptions::default() };
    let before = alloc.clone();
    // A quarter turn is not an involution.
    let r = unfold(&m, &[rot_z90()], ReferenceBuilder::new(), &o, &mut alloc);
    assert!(matches!(r, Err(MeshError::Unsupported(_))));
    // A mirror and a diagonal mirror (a dihedral pair) do not commute.
    let diag = mirror_transform(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 0.0));
    let r = unfold(&m, &[mirror_x(0.0), diag], ReferenceBuilder::new(), &o, &mut alloc);
    assert!(matches!(r, Err(MeshError::Unsupported(_))));
    assert_eq!(format!("{alloc:?}"), format!("{before:?}"), "alloc unchanged on Err");
    // Orthogonal mirrors work as before: 4 images, volume 4.
    let (u, map) = unfold(&m, &[mirror_x(0.0), mirror_y(0.0)], ReferenceBuilder::new(), &o, &mut alloc).unwrap();
    assert_eq!(u.cell_count(), 4);
    assert!((total_volume(&u) - 4.0).abs() < 1e-12);
    assert_eq!(map.cells.len(), 4);
    // Image k = product of the deck elements with bit i set: image 3 is in x < 0, y < 0.
    // Instance ids ascend with k.
    let mut insts: Vec<_> = (0..4).filter_map(|c| map.locate(CellIx::new(c)).map(|(i, _)| i)).collect();
    insts.sort();
    insts.dedup();
    assert_eq!(insts.len(), 4);
    let c3 = (0..4).map(CellIx::new).find(|c| map.locate(*c).is_some_and(|(i, _)| i == insts[3])).unwrap();
    let p = centre(&u, c3);
    assert!(p.x < 0.0 && p.y < 0.0, "{p:?}");
    // A half turn about z commutes with both mirrors and squares to the identity.
    let half = compose(&rot_z90(), &rot_z90());
    assert!(unfold(&m, &[mirror_x(0.0), half], ReferenceBuilder::new(), &o, &mut alloc).is_ok());
}
