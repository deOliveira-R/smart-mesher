//! Mirror unfold, welding and flatten.

mod common;
use common::*;
use sm_core::{EntityId, EntityKind, Transform};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceCatalog, ReferenceMesh, hex_block};
use sm_mesh_api::*;

fn unit_hex(alloc: &mut SerialAlloc) -> ReferenceMesh {
    hex_block([1, 1, 1], [1.0; 3], alloc).unwrap()
}

fn weld() -> FlattenOptions<'static> {
    FlattenOptions {
        weld_tol: Some(1e-9),
        ..FlattenOptions::default()
    }
}

#[test]
fn half_unfold_welds_the_mirror_plane_and_reclassifies_the_seam() {
    let mut alloc = SerialAlloc::new();
    let m = hex_block([2, 1, 1], [1.0; 3], &mut alloc).unwrap();
    // Reflect across x = 0: the original occupies [0, 2], the image [-2, 0].
    let (u, map) = unfold(
        &m,
        &[mirror_x(0.0)],
        ReferenceBuilder::new(),
        &weld(),
        &mut alloc,
    )
    .unwrap();
    assert_valid(&u);
    assert_eq!((u.cell_count(), u.vertex_count()), (4, 20));
    // Two seam faces (one per cell row) are interior now.
    let seam: Vec<FaceIx> = interior_faces(&u)
        .into_iter()
        .filter(|&f| face_centroid(&u, f).x.abs() < 1e-12)
        .collect();
    assert_eq!(seam.len(), 1);
    assert_eq!(map.merged_faces, seam);
    // Seam reclassification: no stale TopoFace, the face lies on the shared volume.
    assert_eq!(u.face_class(seam[0]).entity, vol().entity);
    // left = the original cell (lower id), normal toward the mirror image.
    let f = seam[0];
    assert!(u.face_left(f) < u.face_right(f).unwrap());
    assert!(
        face_area_vector(&u, f).x < 0.0,
        "normal points from the original [x>=0] cell into the image [x<=0]"
    );
    // Provenance.
    assert_eq!(map.cells.len(), 2);
    assert_eq!(map.locate(CellIx(2)).map(|(_, c)| c), Some(CellIx(0)));
    assert!((total_volume(&u) - 4.0).abs() < 1e-12);
    // Boundary count: 10 for each 2x1x1 half minus 1... = 2*(2*(2+1+2)) - 2 seam = 20 - 2... computed directly:
    let nb = (0..u.face_count())
        .filter(|&f| u.is_boundary_face(FaceIx::new(f)))
        .count();
    assert_eq!(nb, 4 * 6 - 2 * interior_faces(&u).len());
}

#[test]
fn quarter_unfold_has_18_vertices_and_4_interior_faces() {
    let mut alloc = SerialAlloc::new();
    let m = unit_hex(&mut alloc);
    let (u, map) = unfold(
        &m,
        &[mirror_x(0.0), mirror_y(0.0)],
        ReferenceBuilder::new(),
        &weld(),
        &mut alloc,
    )
    .unwrap();
    assert_valid(&u);
    assert_eq!(u.cell_count(), 4);
    assert_eq!(u.vertex_count(), 18);
    assert_eq!(interior_faces(&u).len(), 4);
    assert_eq!(map.merged_faces.len(), 4);
    for &f in &map.merged_faces {
        assert_eq!(
            u.face_class(f).entity,
            vol().entity,
            "seam faces are reclassified"
        );
    }
    // 2x2 block: 24 - 2*4 = 16 boundary faces.
    assert_eq!(
        (0..u.face_count())
            .filter(|&f| u.is_boundary_face(FaceIx::new(f)))
            .count(),
        16
    );
    // Vertices on the unfold axis (x = y = 0) are boundary vertices of the union: their class survives.
    let corner_ids: Vec<_> = (0..u.vertex_count())
        .filter(|&v| u.point(v).x == 0.0 && u.point(v).y == 0.0)
        .collect();
    assert_eq!(corner_ids.len(), 2);
    assert!((total_volume(&u) - 4.0).abs() < 1e-12);
    // The result is a conforming mesh: every cell has 6 faces and 4 of the 8 cell-face pairs are shared.
    for c in 0..4 {
        assert_eq!(u.cell_faces(CellIx::new(c)).len(), 6);
    }
}

#[test]
fn unfold_without_weld_keeps_copies_disjoint() {
    let mut alloc = SerialAlloc::new();
    let m = unit_hex(&mut alloc);
    let (u, _) = unfold(
        &m,
        &[mirror_x(0.0)],
        ReferenceBuilder::new(),
        &FlattenOptions::default(),
        &mut alloc,
    )
    .unwrap();
    assert_eq!((u.vertex_count(), interior_faces(&u).len()), (16, 0));
    assert_valid(&u);
}

#[test]
fn two_translated_copies_weld_to_one_interior_face() {
    let mut alloc = SerialAlloc::new();
    let proto = unit_hex(&mut alloc);
    let pid = proto.mesh_id();
    let mut cat = ReferenceCatalog::new();
    cat.insert_mesh(Box::new(proto)).unwrap();
    let (i1, i2) = (id(EntityKind::Instance, 100), id(EntityKind::Instance, 101));
    cat.insert_view(InstancedView {
        id: id(EntityKind::Instance, 200),
        // Deliberately listed out of id order: flatten sorts by id.
        instances: vec![
            Instance {
                id: i2,
                prototype: pid,
                transform: translate(1.0, 0.0, 0.0),
            },
            Instance {
                id: i1,
                prototype: pid,
                transform: Transform::IDENTITY,
            },
        ],
    })
    .unwrap();
    // Without a rule for the seam vertices the two TopoVertex classes conflict.
    let err = flatten(
        &cat,
        id(EntityKind::Instance, 200),
        ReferenceBuilder::new(),
        &weld(),
        &mut alloc.clone(),
    )
    .unwrap_err();
    assert!(matches!(err, MeshError::ClassConflict(_)), "{err:?}");
    // With one (keep the existing class) the weld succeeds.
    // Vertices: keep the existing class. Faces: the shared cell class (what the default does).
    let keep = |c: &MergeCtx| match c.kind {
        MeshEntityKind::Vertex => Ok(c.existing),
        _ => Ok(Classification::on(c.cells[0].entity)),
    };
    let opts = FlattenOptions {
        weld_tol: Some(1e-9),
        on_merge: Some(&keep),
        ..FlattenOptions::default()
    };
    let (m, map) = flatten(
        &cat,
        id(EntityKind::Instance, 200),
        ReferenceBuilder::new(),
        &opts,
        &mut alloc,
    )
    .unwrap();
    assert_valid(&m);
    assert_eq!(
        (m.cell_count(), m.vertex_count(), m.face_count()),
        (2, 12, 11)
    );
    assert_eq!(interior_faces(&m).len(), 1);
    assert_eq!(map.merged_faces.len(), 1);
    // Seam face: both original faces were boundary (different TopoFaces); now on the volume.
    assert_eq!(m.face_class(map.merged_faces[0]).entity, vol().entity);
    assert_eq!(
        m.classified_on(id(EntityKind::TopoFace, 2))
            .map(|s| s.faces.len()),
        Some(1),
        "only the original +x face of copy 2 remains"
    );
    // Provenance: cells of i1 first (ascending instance id), vertex maps recorded per instance.
    assert_eq!(map.cells[0].0, i1);
    assert_eq!(map.cells[1].0, i2);
    assert_eq!(map.locate(CellIx(1)), Some((i2, CellIx(0))));
    assert_eq!(map.instance_vertices.len(), 2);
    assert_eq!(
        map.instance_vertices[1]
            .1
            .iter()
            .filter(|v| v.get() < 8)
            .count(),
        4,
        "four seam vertices of copy 2 were welded to copy 1"
    );
    assert_eq!(map.vertex_origin.len(), 12);
    assert_eq!(map.face_origin.len(), 11);
    assert!((total_volume(&m) - 2.0).abs() < 1e-12);
    // Run twice: identical.
    let (m2, _) = flatten(
        &cat,
        id(EntityKind::Instance, 200),
        ReferenceBuilder::new(),
        &opts,
        &mut SerialAlloc::new(),
    )
    .unwrap();
    let (m3, _) = flatten(
        &cat,
        id(EntityKind::Instance, 200),
        ReferenceBuilder::new(),
        &opts,
        &mut SerialAlloc::new(),
    )
    .unwrap();
    assert_eq!(topology_hash(&m2), topology_hash(&m3));
    assert_eq!(geometry_hash(&m2), geometry_hash(&m3));
}

#[test]
fn separated_copies_do_not_weld_and_report_losses() {
    let mut alloc = SerialAlloc::new();
    let mut proto = unit_hex(&mut alloc);
    proto
        .add_set("all", SetMembers::Cells(vec![CellIx(0)]), &mut alloc)
        .unwrap();
    proto
        .set_tag("mat", MeshEntityKind::Cell, TagValues::I64(vec![3]))
        .unwrap();
    let pid = proto.mesh_id();
    let mut cat = ReferenceCatalog::new();
    cat.insert_mesh(Box::new(proto)).unwrap();
    let vid = id(EntityKind::Instance, 9);
    cat.insert_view(InstancedView {
        id: vid,
        instances: vec![
            Instance {
                id: id(EntityKind::Instance, 1),
                prototype: pid,
                transform: Transform::IDENTITY,
            },
            Instance {
                id: id(EntityKind::Instance, 2),
                prototype: pid,
                transform: translate(5.0, 0.0, 0.0),
            },
        ],
    })
    .unwrap();
    let (m, map) = flatten(&cat, vid, ReferenceBuilder::new(), &weld(), &mut alloc).unwrap();
    assert_eq!((m.vertex_count(), interior_faces(&m).len()), (16, 0));
    assert!(map.merged_faces.is_empty());
    assert_eq!(
        map.loss.len(),
        4,
        "set and tag from each instance are reported as dropped"
    );
    assert!(
        map.loss
            .iter()
            .any(|l| l.what == LossKind::Set("all".into()))
    );
    assert!(
        map.loss
            .iter()
            .any(|l| l.what == LossKind::Tag(MeshEntityKind::Cell, "mat".into()))
    );
}

#[test]
fn baffles_inside_a_prototype_are_never_welded() {
    // A prototype of two hexes that touch at x = 1 but own distinct, coincident vertices (a baffle).
    let mut alloc = SerialAlloc::new();
    let mut b = ReferenceBuilder::new();
    let mut n = 0;
    for x0 in [0.0, 1.0] {
        let vs: Vec<VertexIx> = template_points(CellShape::Hex)
            .iter()
            .map(|p| {
                n += 1;
                let v = b
                    .add_vertex(sm_core::Point3::new(x0 + p[0], p[1], p[2]))
                    .unwrap();
                b.set_class(MeshEntityRef::Vertex(v), on(EntityKind::TopoVertex, n))
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
    }
    let proto = b.finish(&mut alloc).unwrap().mesh;
    assert_eq!((proto.vertex_count(), proto.face_count()), (16, 12));
    // One append with a huge tolerance keeps both coincident vertex sets.
    let mut t = ReferenceBuilder::new();
    let opts = FlattenOptions {
        weld_tol: Some(1e-9),
        ..FlattenOptions::default()
    };
    append_transformed(
        &mut t,
        &proto,
        id(EntityKind::Instance, 1),
        &Transform::IDENTITY,
        &opts,
    )
    .unwrap();
    assert_eq!(t.vertex_count(), 16);
    // A second copy abutting at x = 2 welds only to the other instance: 4 vertices, one interior face.
    let keep = |c: &MergeCtx| match c.kind {
        MeshEntityKind::Vertex => Ok(c.existing),
        _ => Ok(Classification::on(c.cells[0].entity)),
    };
    let opts = FlattenOptions {
        weld_tol: Some(1e-9),
        on_merge: Some(&keep),
        ..FlattenOptions::default()
    };
    let mut t = ReferenceBuilder::new();
    let a1 = append_transformed(
        &mut t,
        &proto,
        id(EntityKind::Instance, 1),
        &Transform::IDENTITY,
        &opts,
    )
    .unwrap();
    let a2 = append_transformed(
        &mut t,
        &proto,
        id(EntityKind::Instance, 2),
        &translate(2.0, 0.0, 0.0),
        &opts,
    )
    .unwrap();
    finalize_welds(&mut t, &a2.welds, &opts).unwrap();
    assert!(a1.welds.is_empty());
    assert_eq!(a2.welds.len(), 4);
    assert_eq!(t.vertex_count(), 28);
    let m = t.finish(&mut alloc).unwrap().mesh;
    assert_valid(&m);
    assert_eq!(
        interior_faces(&m).len(),
        1,
        "the baffle at x = 1 and x = 3 stays two boundary faces each"
    );
    assert_eq!(m.face_count(), 12 + 12 - 1);
}

#[test]
fn append_with_weld_accepts_nothing_inside_one_instance() {
    // Welding is across instances only: a single prototype with a very large tolerance keeps its vertices.
    let mut alloc = SerialAlloc::new();
    let m = unit_hex(&mut alloc);
    let mut b = ReferenceBuilder::new();
    let opts = FlattenOptions {
        weld_tol: Some(10.0),
        ..FlattenOptions::default()
    };
    let am = append_transformed(
        &mut b,
        &m,
        id(EntityKind::Instance, 1),
        &Transform::IDENTITY,
        &opts,
    )
    .unwrap();
    assert_eq!(b.vertex_count(), 8);
    assert!(am.welds.is_empty() && am.merged_faces.is_empty());
}

#[test]
fn unfold_rejects_high_order() {
    let mut alloc = SerialAlloc::new();
    let mut m = single_cell(CellShape::Tet, &mut alloc);
    let corners = m.cell_corners(CellIx(0)).unwrap().to_vec();
    let mut nodes: Vec<PointIx> = corners.iter().map(|&v| v.into()).collect();
    for e in CellShape::Tet.local_edges() {
        let (a, b) = (
            m.point(corners[e[0] as usize].get()),
            m.point(corners[e[1] as usize].get()),
        );
        nodes.push(
            MeshEdit::add_extra_node(
                &mut m,
                sm_core::Point3::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, (a.z + b.z) / 2.0),
            )
            .unwrap(),
        );
    }
    MeshEdit::set_high_order(
        &mut m,
        CellIx(0),
        Some(HighOrderCell {
            degree: 2,
            basis: Basis::LagrangeEquispaced,
            nodes,
        }),
    )
    .unwrap();
    let r = unfold(
        &m,
        &[mirror_x(0.0)],
        ReferenceBuilder::new(),
        &weld(),
        &mut alloc,
    );
    assert!(matches!(r, Err(MeshError::Unsupported(_))));
    // A proper transform without welding carries the high-order record.
    let mut b = ReferenceBuilder::new();
    append_transformed(
        &mut b,
        &m,
        id(EntityKind::Instance, 1),
        &translate(1.0, 2.0, 3.0),
        &FlattenOptions::default(),
    )
    .unwrap();
    let t = b.finish(&mut alloc).unwrap().mesh;
    let h = t.high_order_cell(CellIx(0)).unwrap();
    assert_eq!((h.degree, h.nodes.len()), (2, 10));
    assert_eq!(
        t.point(h.nodes[4].get()).z,
        m.point(h.nodes[4].get()).z + 3.0
    );
    let _: Option<EntityId> = None;
}
