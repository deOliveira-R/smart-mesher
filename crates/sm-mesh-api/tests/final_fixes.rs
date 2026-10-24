//! Regression tests for the review fixes: allocator atomicity, edit rules, edge derivation,
//! seam kinds, and serde round trips.

mod common;
use common::*;
use sm_core::{EntityKind, Point3};
use sm_mesh_api::reference::{ReferenceBuilder, hex_block, hex_block_with_edges};
use sm_mesh_api::*;

fn state(a: &SerialAlloc) -> Vec<(EntityKind, u64)> {
    a.state().collect()
}

/// A classified one-hex builder (not finished).
fn hex_builder() -> ReferenceBuilder {
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = template_points(CellShape::Hex)
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
    let added = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
    b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
    for (i, sf) in added.faces.iter().enumerate() {
        b.set_class(
            MeshEntityRef::Face(sf.face()),
            on(EntityKind::TopoFace, 1 + i as u64),
        )
        .unwrap();
    }
    b
}

#[test]
fn failed_finish_draws_no_ids() {
    // A set member out of range fails late in `finish`.
    let mut b = hex_builder();
    b.add_set("bad", SetMembers::Cells(vec![CellIx(7)]))
        .unwrap();
    let mut alloc = SerialAlloc::new();
    assert!(b.finish(&mut alloc).is_err());
    assert!(
        state(&alloc).iter().all(|(_, n)| *n == 1),
        "{:?}",
        state(&alloc)
    );
    // A tag of the wrong length likewise.
    let mut b = hex_builder();
    b.add_tag("t", MeshEntityKind::Cell, TagValues::I64(vec![1, 2, 3]))
        .unwrap();
    assert!(b.finish(&mut alloc).is_err());
    assert_eq!(alloc.next_serial(EntityKind::Mesh), 1);
    // The next success sees the ids a fresh allocator would give (serials start at 1).
    let m = hex_builder().finish(&mut alloc).unwrap().mesh;
    assert_eq!(m.mesh_id(), id(EntityKind::Mesh, 1));
    assert_eq!(
        m.id(MeshEntityKind::Vertex, 0),
        id(EntityKind::MeshVertex, 1)
    );
}

#[test]
fn failed_edge_derivation_draws_no_ids() {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([1, 1, 1], [1.0; 3], &mut alloc).unwrap();
    let before = state(&alloc);
    let e = MeshEdit::ensure_edges(&mut m, &mut alloc, &[]).unwrap_err();
    // The error names a vertex pair, not an index into a table that was never made.
    assert!(
        matches!(e, MeshError::UnclassifiedEdge(a, b) if a < b),
        "{e:?}"
    );
    assert_eq!(state(&alloc), before);
    assert!(m.edges().is_none());
    // add_set also leaves the allocator alone on error.
    assert!(
        MeshEdit::add_set(&mut m, "s", SetMembers::Cells(vec![CellIx(9)]), &mut alloc).is_err()
    );
    assert_eq!(state(&alloc), before);
}

#[test]
fn failed_unfold_and_flatten_draw_no_ids() {
    let mut alloc = SerialAlloc::new();
    let m = hex_block([1, 1, 1], [1.0; 3], &mut alloc).unwrap();
    let before = state(&alloc);
    // The builder already holds a bad set, so `finish` fails after the instances were created.
    let mut b = ReferenceBuilder::new();
    b.add_set("bad", SetMembers::Cells(vec![CellIx(500)]))
        .unwrap();
    let o = FlattenOptions::default();
    let r = unfold(&m, &[mirror_x(5.0)], b, &o, &mut alloc);
    assert!(r.is_err());
    assert_eq!(state(&alloc), before);
    // Too long a deck fails before anything is drawn.
    let deck = vec![mirror_x(0.0); 17];
    assert!(unfold(&m, &deck, ReferenceBuilder::new(), &o, &mut alloc).is_err());
    assert_eq!(state(&alloc), before);
}

#[test]
fn edge_class_comes_from_incident_faces() {
    // One model face carries the whole boundary: every edge is derivable.
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = template_points(CellShape::Hex)
        .iter()
        .map(|p| {
            let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoFace, 1))
                .unwrap();
            v
        })
        .collect();
    let added = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
    b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
    for sf in &added.faces {
        b.set_class(MeshEntityRef::Face(sf.face()), on(EntityKind::TopoFace, 1))
            .unwrap();
    }
    let mut alloc = SerialAlloc::new();
    let mut m = b.finish(&mut alloc).unwrap().mesh;
    MeshEdit::ensure_edges(&mut m, &mut alloc, &[]).unwrap();
    let t = m.edges().unwrap();
    assert_eq!(t.len(), 12);
    assert!(
        t.class
            .iter()
            .all(|c| c.entity == id(EntityKind::TopoFace, 1))
    );
    assert_valid(&m);
}

#[test]
fn edits_keep_the_per_entity_rules() {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block([2, 1, 1], [1.0; 3], &mut alloc).unwrap();
    let bf = (0..m.face_count())
        .map(FaceIx::new)
        .find(|&f| m.face_kind(f) == FaceKind::Boundary)
        .unwrap();
    // A Boundary face cannot move onto a volume, mirroring `set_face_kind`.
    assert!(MeshEdit::set_class(&mut m, MeshEntity::Face(bf), vol()).is_err());
    assert!(validate(&m).is_empty());
    // An edge tag needs an edge table.
    let e = MeshEdit::set_tag(&mut m, "w", MeshEntityKind::Edge, TagValues::I64(vec![]));
    assert!(e.is_err());
    assert!(validate(&m).is_empty());
    // Batch: all or nothing, and one index rebuild.
    let other = on(EntityKind::TopoFace, 77);
    let ok = [(MeshEntity::Face(bf), other)];
    MeshEdit::set_classes(&mut m, &ok).unwrap();
    assert_eq!(m.face_class(bf), other);
    assert_eq!(m.classified_on(other.entity).unwrap().faces, vec![bf]);
    let before = m.clone();
    let mixed = [
        (MeshEntity::Face(bf), on(EntityKind::TopoFace, 5)),
        (MeshEntity::Cell(CellIx(0)), on(EntityKind::TopoFace, 5)),
    ];
    assert!(MeshEdit::set_classes(&mut m, &mixed).is_err());
    assert_eq!(m, before);
    assert!(validate(&m).is_empty());
}

#[test]
fn welding_a_non_conformal_seam_clears_the_kind() {
    let mut alloc = SerialAlloc::new();
    let mut m = single_cell(CellShape::Hex, &mut alloc);
    let seam = (0..m.face_count())
        .map(FaceIx::new)
        .find(|&f| face_centroid(&m, f).x.abs() < 1e-12)
        .unwrap();
    MeshEdit::set_face_kind(&mut m, seam, FaceKind::NonConformal).unwrap();
    let o = FlattenOptions::new().with_weld_tol(1e-9).with_on_merge(&|c: &MergeCtx| Ok(Classification::on(c.cells[0].entity)));
    let (u, map) = unfold(
        &m,
        &[mirror_x(0.0)],
        ReferenceBuilder::new(),
        &o,
        &mut alloc,
    )
    .unwrap();
    assert_valid(&u);
    assert_eq!(map.merged_faces.len(), 1);
    assert_eq!(u.face_kind(map.merged_faces[0]), FaceKind::Interior);
}

#[test]
fn builder_set_face_kind_boundary_clears_a_mark() {
    let mut b = hex_builder();
    let f = FaceIx(0);
    b.set_face_kind(f, FaceKind::PartitionCut).unwrap();
    b.set_face_kind(f, FaceKind::Boundary).unwrap();
    assert!(b.set_face_kind(f, FaceKind::Interior).is_err());
    let m = b.finish(&mut SerialAlloc::new()).unwrap().mesh;
    assert!(m.is_boundary_face(FaceIx(0)));
}

#[test]
fn serial_alloc_state_round_trips() {
    let mut a = SerialAlloc::new();
    a.take(EntityKind::MeshCell, 5).unwrap();
    a.one(EntityKind::Mesh).unwrap();
    let b = SerialAlloc::from_state(a.state()).unwrap();
    assert_eq!(b.next_serial(EntityKind::MeshCell), 6);
    assert_eq!(b.next_serial(EntityKind::MeshSet), 1);
    assert!(SerialAlloc::from_state([(EntityKind::Mesh, 0)]).is_err(), "serials start at 1");
    assert!(
        SerialAlloc::from_state([(EntityKind::Mesh, 1), (EntityKind::Mesh, 2)]).is_err(),
        "a repeated kind is rejected"
    );
    assert!(SerialAlloc::from_state([(EntityKind::Mesh, u64::MAX)]).is_err());
}

#[cfg(feature = "serde")]
mod serde_round_trips {
    use super::*;

    #[test]
    fn serial_alloc_rejects_bad_snapshots_at_load_time() {
        let mut a = SerialAlloc::new();
        a.take(EntityKind::MeshCell, 1).unwrap();
        let good = serde_json::to_string(&a).unwrap();
        assert!(serde_json::from_str::<SerialAlloc>(&good).is_ok());
        let bad = good.replace(":2", ":18446744073709551615");
        assert_ne!(good, bad);
        assert!(serde_json::from_str::<SerialAlloc>(&bad).is_err());
    }

    fn rt<T: serde::Serialize + serde::de::DeserializeOwned + PartialEq + std::fmt::Debug>(
        v: &T,
    ) -> String {
        let s = serde_json::to_string(v).unwrap();
        let back: T = serde_json::from_str(&s).unwrap();
        assert_eq!(&back, v, "{s}");
        s
    }

    #[test]
    fn classification_uses_kind_hash_serial() {
        let c = Classification {
            entity: id(EntityKind::TopoFace, 3),
            param: Param::Surface([0.25, 0.5]),
        };
        let s = rt(&c);
        assert!(s.contains("\"TopoFace#3\""), "{s}");
        rt(&Classification::on(id(EntityKind::TopoVolume, 1)));
    }

    #[test]
    fn mesh_tables_round_trip() {
        let mut alloc = SerialAlloc::new();
        let mut m = hex_block([2, 1, 1], [1.0; 3], &mut alloc).unwrap();
        MeshEdit::add_set(&mut m, "s", SetMembers::Cells(vec![CellIx(1)]), &mut alloc).unwrap();
        MeshEdit::set_tag(
            &mut m,
            "t",
            MeshEntityKind::Cell,
            TagValues::F64(vec![1.5, 2.5]),
        )
        .unwrap();
        for sf in m.cell_faces(CellIx(0)) {
            rt(sf);
        }
        rt(&m.sets()[0]);
        rt(&m.tags()[0]);
        let cache = compute_geometry_cache(&m);
        rt(&cache);
        let e = hex_block_edges(&mut alloc);
        rt(e.edges().unwrap());
        rt(&HighOrderCell::new(2, Basis::LagrangeEquispaced, vec![PointIx(0), PointIx(3)]));
        let v = validate(&m);
        assert!(v.is_empty());
        let viol = Violation {
            code: ViolationCode::FaceOrder,
            entity: Some(MeshEntity::Face(FaceIx(2))),
            detail: "x".into(),
        };
        rt(&viol);
        rt(&LossNote::new(id(EntityKind::MeshInstance, 4), LossKind::Tag(MeshEntityKind::Cell, "t".into())));
        let inst = Instance::new(id(EntityKind::MeshInstance, 1), id(EntityKind::Mesh, 1), mirror_x(1.0));
        rt(&InstancedView::new(id(EntityKind::MeshInstance, 2), vec![inst]));
    }

    #[test]
    fn serial_alloc_serializes_its_state() {
        let mut a = SerialAlloc::new();
        a.take(EntityKind::MeshFace, 9).unwrap();
        let s = serde_json::to_string(&a).unwrap();
        let b: SerialAlloc = serde_json::from_str(&s).unwrap();
        assert_eq!(state(&a), state(&b));
    }
}

#[cfg(feature = "serde")]
fn hex_block_edges(alloc: &mut SerialAlloc) -> reference::ReferenceMesh {
    reference::hex_block_with_edges([1, 1, 1], [1.0; 3], alloc).unwrap()
}

#[test]
fn serial_overflow_leaves_no_trace() {
    let mut a = SerialAlloc::new();
    let before = state(&a);
    assert!(a.take(EntityKind::MeshSet, usize::MAX).is_err());
    assert_eq!(
        state(&a),
        before,
        "failed take on a fresh kind adds no entry"
    );
}

#[test]
fn edge_derivation_uses_lowest_dimension_entity() {
    let mut alloc = SerialAlloc::new();
    let full = hex_block_with_edges([2, 2, 2], [1.0; 3], &mut alloc).unwrap();
    let t = full.edges().unwrap();
    // Only the model-edge classes; surface and interior edges must be derived.
    let explicit: Vec<_> = t
        .vertices
        .iter()
        .zip(&t.class)
        .filter(|(_, c)| c.entity.kind() == EntityKind::TopoEdge)
        .map(|(v, c)| (v[0], v[1], *c))
        .collect();
    assert!(!explicit.is_empty());
    let mut m = hex_block([2, 2, 2], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    MeshEdit::ensure_edges(&mut m, &mut SerialAlloc::new(), &explicit).unwrap();
    assert_eq!(m.edges().unwrap().class, t.class);
    assert_valid(&m);
}

#[test]
fn set_face_kind_drops_the_geometry_cache() {
    let mut m = hex_block([1, 1, 1], [1.0; 3], &mut SerialAlloc::new()).unwrap();
    let c = compute_geometry_cache(&m);
    MeshEdit::set_geometry_cache(&mut m, Some(c)).unwrap();
    MeshEdit::set_face_kind(&mut m, FaceIx(0), FaceKind::NonConformal).unwrap();
    assert!(m.geometry_cache().is_none());
    assert!(validate(&m).is_empty());
}

#[test]
fn corners_that_do_not_realise_the_template_fail_at_finish() {
    let mut b = ReferenceBuilder::new();
    let vs: Vec<VertexIx> = template_points(CellShape::Hex)
        .iter()
        .map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap())
        .collect();
    let mut swapped = vs.clone();
    swapped.swap(0, 6);
    let faces: Vec<Vec<VertexIx>> = CellShape::Hex
        .local_faces()
        .iter()
        .map(|lf| lf.iter().map(|&i| vs[i as usize]).collect())
        .collect();
    let refs: Vec<&[VertexIx]> = faces.iter().map(Vec::as_slice).collect();
    let added = b
        .add_polyhedron_with_corners(&refs, Some(&swapped))
        .unwrap();
    for (i, &v) in vs.iter().enumerate() {
        let c = on(EntityKind::TopoVertex, 1 + i as u64);
        b.set_class(MeshEntityRef::Vertex(v), c).unwrap();
    }
    b.set_class(MeshEntityRef::Cell(added.cell), vol()).unwrap();
    for (i, sf) in added.faces.iter().enumerate() {
        let c = on(EntityKind::TopoFace, 1 + i as u64);
        b.set_class(MeshEntityRef::Face(sf.face()), c).unwrap();
    }
    let r = b.finish(&mut SerialAlloc::new());
    assert!(
        matches!(r, Err(MeshError::ShapeMismatch(..))),
        "{:?}",
        r.err()
    );
}
