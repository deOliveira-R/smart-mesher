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
    // One seam face (a single cell row) is interior now.
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
    // Each cell has 6 faces; every interior face is shared by two cells, so
    // boundary faces = 4 cells * 6 - 2 * interior faces.
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
    // Documented stale state (`FlattenOptions::on_merge` is the caller's job): the axis vertices
    // keep the prototype's TopoVertex even though that model vertex no longer bounds the union.
    for &v in &corner_ids {
        let z = u.point(v).z;
        let sv = (0..m.vertex_count())
            .find(|&i| m.point(i).x == 0.0 && m.point(i).y == 0.0 && m.point(i).z == z)
            .unwrap();
        let want = m.vertex_class(VertexIx::new(sv));
        assert_eq!(want.entity.kind(), EntityKind::TopoVertex);
        assert_eq!(u.vertex_class(VertexIx::new(v)), want);
    }
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
    let (i1, i2) = (id(EntityKind::MeshInstance, 100), id(EntityKind::MeshInstance, 101));
    cat.insert_view(InstancedView {
        id: id(EntityKind::MeshInstance, 200),
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
        id(EntityKind::MeshInstance, 200),
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
        id(EntityKind::MeshInstance, 200),
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
        id(EntityKind::MeshInstance, 200),
        ReferenceBuilder::new(),
        &opts,
        &mut SerialAlloc::new(),
    )
    .unwrap();
    let (m3, _) = flatten(
        &cat,
        id(EntityKind::MeshInstance, 200),
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
    let vid = id(EntityKind::MeshInstance, 9);
    cat.insert_view(InstancedView {
        id: vid,
        instances: vec![
            Instance {
                id: id(EntityKind::MeshInstance, 1),
                prototype: pid,
                transform: Transform::IDENTITY,
            },
            Instance {
                id: id(EntityKind::MeshInstance, 2),
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
        id(EntityKind::MeshInstance, 1),
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
        id(EntityKind::MeshInstance, 1),
        &Transform::IDENTITY,
        &opts,
    )
    .unwrap();
    let a2 = append_transformed(
        &mut t,
        &proto,
        id(EntityKind::MeshInstance, 2),
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

/// Two hexes on a 3x2x2 vertex lattice sharing their x = 1 vertices, but built with one
/// `add_face` per template face: the coincident x = 1 faces stay two one-cell faces (a baffle).
fn lattice_baffle(nc_kind: Option<FaceKind>, alloc: &mut SerialAlloc) -> ReferenceMesh {
    let mut b = ReferenceBuilder::new();
    let mut lat = Vec::new();
    for k in 0..2 {
        for j in 0..2 {
            for i in 0..3 {
                let v = b
                    .add_vertex(sm_core::Point3::new(i as f64, j as f64, k as f64))
                    .unwrap();
                b.set_class(
                    MeshEntityRef::Vertex(v),
                    on(EntityKind::TopoVertex, lat.len() as u64),
                )
                .unwrap();
                lat.push(v);
            }
        }
    }
    for x0 in [0usize, 1] {
        let vs: Vec<VertexIx> = template_points(CellShape::Hex)
            .iter()
            .map(|p| lat[(x0 + p[0] as usize) + 3 * (p[1] as usize + 2 * p[2] as usize)])
            .collect();
        let mut faces = Vec::new();
        for lf in CellShape::Hex.local_faces() {
            let l: Vec<VertexIx> = lf.iter().map(|&i| vs[i as usize]).collect();
            let f = b.add_face(&l).unwrap();
            let on_baffle = l.iter().all(|v| b.vertex_point(*v).x == 1.0);
            let cls = if on_baffle && nc_kind.is_some() {
                vol()
            } else {
                on(EntityKind::TopoFace, 1 + faces.len() as u64)
            };
            b.set_class(MeshEntityRef::Face(f), cls).unwrap();
            if on_baffle && let Some(k) = nc_kind {
                b.set_face_kind(f, k).unwrap();
            }
            faces.push(SignedFace::new(f, Side::Left));
        }
        let c = b.add_cell(&faces, Some(&vs)).unwrap();
        b.set_class(MeshEntityRef::Cell(c), vol()).unwrap();
    }
    b.finish(alloc).unwrap().mesh
}

#[test]
fn identity_append_keeps_topological_baffles() {
    for kind in [None, Some(FaceKind::NonConformal)] {
        let mut alloc = SerialAlloc::new();
        let proto = lattice_baffle(kind, &mut alloc);
        assert_valid(&proto);
        assert_eq!((proto.face_count(), interior_faces(&proto).len()), (12, 0));
        let mut t = ReferenceBuilder::new();
        let am = append_transformed(
            &mut t,
            &proto,
            id(EntityKind::MeshInstance, 1),
            &Transform::IDENTITY,
            &FlattenOptions::default(),
        )
        .unwrap();
        assert!(am.merged_faces.is_empty());
        let m = t.finish(&mut alloc).unwrap().mesh;
        assert_valid(&m);
        assert_eq!(
            (m.face_count(), interior_faces(&m).len()),
            (12, 0),
            "{kind:?}"
        );
        let nc = (0..m.face_count())
            .filter(|&f| m.face_kind(FaceIx::new(f)) == FaceKind::NonConformal)
            .count();
        assert_eq!(nc, if kind.is_some() { 2 } else { 0 });
    }
}

#[test]
fn mirrored_append_keeps_topological_baffles() {
    let mut alloc = SerialAlloc::new();
    let proto = lattice_baffle(None, &mut alloc);
    let mut t = ReferenceBuilder::new();
    append_transformed(
        &mut t,
        &proto,
        id(EntityKind::MeshInstance, 1),
        &mirror_x(-1.0),
        &FlattenOptions::default(),
    )
    .unwrap();
    let m = t.finish(&mut alloc).unwrap().mesh;
    assert_valid(&m);
    assert_eq!((m.face_count(), interior_faces(&m).len()), (12, 0));
    assert!((total_volume(&m) - 2.0).abs() < 1e-12);
}

#[test]
fn welding_to_interior_vertices_of_an_earlier_instance_is_refused() {
    // The cube [2,3]^3 only touches the 2x2x2 block [0,2]^3 at its corner (2,2,2), but the
    // generous tolerance puts the block's interior vertex (1,1,1) (lowest index) within reach
    // of that corner. Interior vertices are never weld targets.
    let mut alloc = SerialAlloc::new();
    let big = hex_block([2, 2, 2], [1.0; 3], &mut alloc).unwrap();
    let small = unit_hex(&mut alloc);
    let mut t = ReferenceBuilder::new();
    let opts = FlattenOptions {
        weld_tol: Some(1.0),
        ..FlattenOptions::default()
    };
    append_transformed(
        &mut t,
        &big,
        id(EntityKind::MeshInstance, 1),
        &Transform::IDENTITY,
        &opts,
    )
    .unwrap();
    let a = append_transformed(
        &mut t,
        &small,
        id(EntityKind::MeshInstance, 2),
        &translate(2.0, 2.0, 2.0),
        &opts,
    )
    .unwrap();
    for w in &a.welds {
        let p = t.vertex_point(w.vertex);
        assert!(
            (p.x, p.y, p.z) != (1.0, 1.0, 1.0),
            "welded to an interior vertex"
        );
    }
}

#[test]
fn shares_the_free_face_of_a_coincident_pair() {
    // The baffle lattice holds two coincident faces at x=1 (one per cell). A cube appended at
    // [0,1]^3 has a +x face matching both; the first is taken, so it must use the second.
    let mut alloc = SerialAlloc::new();
    let proto = lattice_baffle(None, &mut alloc);
    let small = unit_hex(&mut alloc);
    let mut t = ReferenceBuilder::new();
    append_transformed(
        &mut t,
        &proto,
        id(EntityKind::MeshInstance, 1),
        &Transform::IDENTITY,
        &FlattenOptions::default(),
    )
    .unwrap();
    let before = t.face_count();
    append_transformed(
        &mut t,
        &small,
        id(EntityKind::MeshInstance, 2),
        &Transform::IDENTITY,
        &weld(),
    )
    .unwrap();
    assert_eq!(t.face_count(), before + 5, "the +x face is shared");
    assert!(
        (0..before).any(|f| t.face_degree(FaceIx::new(f)) == 2),
        "a baffle face got its second cell"
    );
}

#[test]
fn finalize_welds_rejects_unknown_and_cellless_vertices() {
    let mut alloc = SerialAlloc::new();
    let m = unit_hex(&mut alloc);
    let mut t = ReferenceBuilder::new();
    append_transformed(
        &mut t,
        &m,
        id(EntityKind::MeshInstance, 1),
        &Transform::IDENTITY,
        &FlattenOptions::default(),
    )
    .unwrap();
    let lone = t.add_vertex(sm_core::Point3::new(5.0, 5.0, 5.0)).unwrap();
    t.set_class(MeshEntityRef::Vertex(lone), on(EntityKind::TopoVertex, 99))
        .unwrap();
    let w = |vertex| Weld {
        vertex,
        incoming: on(EntityKind::TopoVertex, 1),
        incoming_cell: vol(),
    };
    let o = FlattenOptions::default();
    assert!(finalize_welds(&mut t, &[w(VertexIx::new(500))], &o).is_err());
    assert!(finalize_welds(&mut t, &[w(lone)], &o).is_err());
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
        id(EntityKind::MeshInstance, 1),
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
        id(EntityKind::MeshInstance, 1),
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
