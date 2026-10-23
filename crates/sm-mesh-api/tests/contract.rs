//! Contract tests against the reference implementation.

use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_mesh_api::reference::{Mesh, MeshBuilder, hex_block, hex_block_builder};
use sm_mesh_api::*;
use smallvec::SmallVec;
use std::sync::Arc;

fn id(k: EntityKind, n: u64) -> EntityId {
    EntityId::new(k, n).unwrap()
}

fn unit_block(n: [u32; 3], mesh: u64, base: u64) -> Mesh {
    hex_block(id(EntityKind::Mesh, mesh), base, n, [0.0; 3], [1.0; 3]).unwrap()
}

fn dot(a: Vector3, b: Vector3) -> f64 {
    a.dot(b)
}

#[test]
fn hex_block_2x2x2_orientation_rule() {
    let m = unit_block([2, 2, 2], 1, 0);
    assert_eq!(
        (m.count(Stratum::Vertex), m.count(Stratum::Edge), m.count(Stratum::Face), m.count(Stratum::Cell)),
        (27, 54, 36, 8)
    );
    assert_eq!(validate(&m), vec![]);
    let boundary = m.boundary_faces();
    assert_eq!(boundary.len(), 24);
    assert_eq!(m.count(Stratum::Face) as usize - boundary.len(), 12);

    let mut boundary_area = 0.0;
    for f in m.points(Stratum::Face) {
        let s = m.face_sides(f);
        let a = m.face_area_vector(f);
        assert!((a.norm() - 0.25).abs() < 1e-14);
        let cl = m.cell_centroid(s.left);
        match s.right {
            Some(r) => {
                assert!(m.entity_id(s.left) < m.entity_id(r), "left must be the lower cell id");
                // Normal points from left to right.
                assert!(dot(m.cell_centroid(r) - cl, a) > 0.0);
                assert!((dot(m.cell_centroid(r) - cl, a) / (a.norm() * 0.5) - 1.0).abs() < 1e-12);
                assert_eq!(m.across(s.left, f), Some(r));
                assert_eq!(m.across(r, f), Some(s.left));
                // Support carries the same signs as the cell cones.
                let sup = m.support(f);
                assert_eq!((sup.len(), sup.orient(0), sup.orient(1)), (2, Orient::Fwd, Orient::Rev));
            }
            None => {
                // Boundary: normal points outward.
                assert!(dot(m.face_centroid(f) - cl, a) > 0.0);
                assert_eq!(m.support(f).len(), 1);
                boundary_area += a.norm();
            }
        }
    }
    assert!((boundary_area - 6.0).abs() < 1e-12);

    for c in m.points(Stratum::Cell) {
        assert!((m.cell_volume(c) - 0.125).abs() < 1e-14);
        let cone = m.cone(c);
        assert_eq!(cone.len(), 6);
        let mut closure = Vector3::new(0.0, 0.0, 0.0);
        for (f, o) in cone.iter() {
            let s = m.face_sides(f);
            // Sign is + (outward) exactly when the cell is `left`.
            assert_eq!(o == Orient::Fwd, s.left == c);
            assert_eq!(s.right == Some(c), o == Orient::Rev);
            closure = closure + m.face_area_vector(f) * o.sign();
        }
        assert!(closure.norm() < 1e-14, "signed area vectors of a closed cell sum to zero");
    }
    // Hasse relations: every face loop's edges exist and the central vertex touches 12 faces.
    for f in m.points(Stratum::Face) {
        assert_eq!(m.face_edges(f).len(), m.cone(f).len());
    }
    assert_eq!(m.support(Point::vertex(13)).len(), 12);
    // id <-> point round trip.
    for p in m.points(Stratum::Cell) {
        assert_eq!(m.point_of(m.entity_id(p)), Some(p));
    }
}

fn tet_faces(v: [u32; 4]) -> SmallVec<[FaceLoop; 6]> {
    let l = |a: usize, b: usize, c: usize| -> FaceLoop { [v[a], v[b], v[c]].iter().map(|&i| Point::vertex(i)).collect() };
    [l(0, 2, 1), l(0, 1, 3), l(1, 2, 3), l(0, 3, 2)].into_iter().collect()
}

#[test]
fn single_polyhedron_and_shape_tags() {
    // Regular pentagonal prism: not one of the five fast-path shapes.
    let mut b = MeshBuilder::new(id(EntityKind::Mesh, 1), 0);
    let n = 5;
    let pt = |i: usize, z: f64| {
        let a = std::f64::consts::TAU * i as f64 / n as f64;
        Point3::new(a.cos(), a.sin(), z)
    };
    let bot: Vec<Point> = (0..n).map(|i| b.add_vertex(pt(i, 0.0), None)).collect();
    let top: Vec<Point> = (0..n).map(|i| b.add_vertex(pt(i, 1.0), None)).collect();
    let mut faces: SmallVec<[FaceLoop; 6]> = SmallVec::new();
    faces.push(bot.iter().rev().copied().collect());
    faces.push(top.iter().copied().collect());
    for i in 0..n {
        let j = (i + 1) % n;
        faces.push([bot[i], bot[j], top[j], top[i]].into_iter().collect());
    }
    let spec = CellSpec { shape: CellShape::Polyhedron, faces: faces.clone(), class: None };
    // A wrong tag is rejected, and the builder is left untouched.
    let bad = CellSpec { shape: CellShape::Hex, ..spec.clone() };
    assert!(matches!(b.add_cell(&bad), Err(MeshError::ShapeMismatch { .. })));
    let added = b.add_cell(&spec).unwrap();
    assert_eq!(added.cell, Point::cell(0));
    let m = b.finish().unwrap();
    assert_eq!(validate(&m), vec![]);
    assert_eq!(m.count(Stratum::Face), 7);
    assert_eq!(m.cell_shape(added.cell), CellShape::Polyhedron);
    assert_eq!(m.boundary_faces().len(), 7);
    let area = 2.5 * (std::f64::consts::TAU / 5.0).sin();
    assert!((m.cell_volume(added.cell) - area).abs() < 1e-12);
    let c = m.cell_centroid(added.cell);
    assert!(c.x.abs() < 1e-12 && c.y.abs() < 1e-12 && (c.z - 0.5).abs() < 1e-12);
    assert_eq!(FaceShape::from_len(m.cone(Point::face(0)).len()), FaceShape::Polygon);
    // Euler characteristic of a closed polyhedron.
    let chi = m.count(Stratum::Vertex) as i64 - m.count(Stratum::Edge) as i64 + m.count(Stratum::Face) as i64;
    assert_eq!(chi, 2);
}

#[test]
fn tets_share_a_face_and_reject_bad_winding() {
    let mut b = MeshBuilder::new(id(EntityKind::Mesh, 1), 0);
    let p = [(0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.), (1., 1., 1.)];
    for (x, y, z) in p {
        b.add_vertex(Point3::new(x, y, z), None);
    }
    let t0 = CellSpec { shape: CellShape::Tet, faces: tet_faces([0, 1, 2, 3]), class: None };
    b.add_cell(&t0).unwrap();
    // Same winding on the shared face (1,2,3): the cell would sit on the same side.
    let l = |v: [u32; 3]| -> FaceLoop { v.iter().map(|&i| Point::vertex(i)).collect() };
    let wrong: SmallVec<[FaceLoop; 6]> = [l([1, 2, 3]), l([2, 1, 4]), l([3, 2, 4]), l([1, 3, 4])].into_iter().collect();
    assert_eq!(
        b.add_cell(&CellSpec { shape: CellShape::Tet, faces: wrong, class: None }).unwrap_err(),
        MeshError::InconsistentOrientation
    );
    let right: SmallVec<[FaceLoop; 6]> = [l([1, 3, 2]), l([3, 1, 4]), l([2, 3, 4]), l([1, 2, 4])].into_iter().collect();
    let added = b.add_cell(&CellSpec { shape: CellShape::Tet, faces: right, class: None }).unwrap();
    assert_eq!(added.cell, Point::cell(1));
    let m = b.finish().unwrap();
    assert_eq!(validate(&m), vec![]);
    assert_eq!(m.count(Stratum::Face), 7);
    let shared = added.faces[0];
    assert_eq!(m.face_sides(shared), FaceSides { left: Point::cell(0), right: Some(Point::cell(1)) });
    assert!((m.cell_volume(Point::cell(0)) - 1.0 / 6.0).abs() < 1e-14);
    // The face loop is wound as in the lower cell: 1 -> 2 -> 3.
    assert_eq!(m.cone(shared).indices, &[1, 2, 3]);
}

#[test]
fn high_order_tet_nodes() {
    let mut b = MeshBuilder::new(id(EntityKind::Mesh, 1), 0);
    for (x, y, z) in [(0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.)] {
        b.add_vertex(Point3::new(x, y, z), None);
    }
    let c = b.add_cell(&CellSpec { shape: CellShape::Tet, faces: tet_faces([0, 1, 2, 3]), class: None }).unwrap().cell;
    let mid = b.add_node(Point3::new(0.5, 0.0, 0.0));
    let mut nodes: Vec<NodeRef> = (0..4).map(NodeRef::vertex).collect();
    nodes.push(mid);
    b.set_cell_order(c, 2, Basis::LagrangeGll, &nodes).unwrap();
    let m = b.finish().unwrap();
    assert_eq!(m.cell_geometry(c), CellGeometry { degree: 2, basis: Basis::LagrangeGll });
    assert_eq!(m.cell_nodes(c).len(), 5);
    assert_eq!(m.node_coord(m.cell_nodes(c)[4]), Point3::new(0.5, 0.0, 0.0));
}

#[test]
fn classification_queries() {
    let make = || hex_block_builder(id(EntityKind::Mesh, 1), 0, [2, 2, 2], [0.0; 3], [1.0; 3]).unwrap();
    let probe = make().finish().unwrap();
    let xmin: Vec<Point> = probe.boundary_faces().into_iter().filter(|&f| probe.face_centroid(f).x.abs() < 1e-12).collect();
    assert_eq!(xmin.len(), 4);

    let (tv, te, tf, tvol) = (
        id(EntityKind::TopoVertex, 1),
        id(EntityKind::TopoEdge, 1),
        id(EntityKind::TopoFace, 1),
        id(EntityKind::TopoVolume, 1),
    );
    let mut b = make();
    b.classify(Point::vertex(0), Classification::on(tv)).unwrap();
    b.classify(Point::vertex(1), Classification { entity: te, param: Param::Curve(0.5) }).unwrap();
    b.classify(Point::vertex(12), Classification { entity: tf, param: Param::Surface([0.5, 0.5]) }).unwrap();
    b.classify_edge(Point::vertex(1), Point::vertex(0), Classification::on(te)).unwrap();
    for &f in &xmin {
        b.classify(f, Classification::on(tf)).unwrap();
    }
    for c in 0..8 {
        b.classify(Point::cell(c), Classification::on(tvol)).unwrap();
    }
    // Not a model entity.
    let not_model = id(EntityKind::MeshCell, 3);
    assert_eq!(b.classify(Point::vertex(2), Classification::on(not_model)), Err(MeshError::NotAModelEntity(not_model)));
    // Violating the dimension order: vertex 12 on a volume while a face it bounds is on a face.
    let mut bad = make();
    for &f in &xmin {
        bad.classify(f, Classification::on(tf)).unwrap();
    }
    bad.classify(Point::vertex(12), Classification::on(tvol)).unwrap();
    let bad = bad.finish().unwrap();
    assert!(validate(&bad).iter().any(|v| matches!(v, Violation::ClassificationOrder { lower, .. } if *lower == Point::vertex(12))));
    // Unknown edge.
    let mut e = make();
    e.classify_edge(Point::vertex(0), Point::vertex(26), Classification::on(te)).unwrap();
    assert_eq!(e.finish().unwrap_err(), MeshError::NoSuchEdge(0, 26));

    let m = b.finish().unwrap();
    assert_eq!(validate(&m), vec![]);
    assert_eq!(m.classified_entities(), vec![tv, te, tf, tvol]);
    assert_eq!(m.points_on(tvol), m.points(Stratum::Cell));
    let on_face = m.points_on(tf);
    assert_eq!(on_face.len(), 5);
    assert_eq!(on_face[0], Point::vertex(12));
    assert_eq!(m.classification(Point::vertex(12)).unwrap().param, Param::Surface([0.5, 0.5]));
    assert_eq!(m.classification(Point::vertex(1)).unwrap().param, Param::Curve(0.5));
    assert_eq!(m.points_on(te).len(), 2);
    assert_eq!(m.points_on(te)[1].stratum, Stratum::Edge);
    assert!(m.points_on(id(EntityKind::TopoVolume, 99)).is_empty());
    assert_eq!(model_dim(tf), Some(2));
}

#[test]
fn labels_are_secondary_indexes() {
    let mut b = hex_block_builder(id(EntityKind::Mesh, 1), 0, [2, 1, 1], [0.0; 3], [1.0; 3]).unwrap();
    b.set_label("material", Point::cell(0), 1).unwrap();
    b.set_label("material", Point::cell(1), 2).unwrap();
    b.set_label("bc", Point::face(0), 7).unwrap();
    assert!(b.set_label("bc", Point::face(999), 1).is_err());
    let m = b.finish().unwrap();
    assert_eq!(m.label_names(), vec!["bc".to_string(), "material".to_string()]);
    assert_eq!(m.label_points("material", 2), vec![Point::cell(1)]);
    assert_eq!(m.label_value("bc", Point::face(0)), Some(7));
    assert_eq!(m.label_value("bc", Point::face(1)), None);
}

#[test]
fn two_level_hierarchy_map() {
    let fine = Arc::new(unit_block([2, 2, 2], 1, 0));
    let coarse = Arc::new(unit_block([2, 1, 1], 2, 1000));
    // Fine cell (i,j,k) -> coarse cell i.
    let cell_map: Vec<u32> = (0..8).map(|c| c % 2).collect();
    let cm = |c: Point| cell_map[c.index as usize];
    let face_map: Vec<u32> = fine
        .points(Stratum::Face)
        .into_iter()
        .map(|f| {
            let s = fine.face_sides(f);
            let a = cm(s.left);
            if s.right.is_some_and(|r| cm(r) == a) {
                return NONE;
            }
            let (af, cf) = (fine.face_area_vector(f), fine.face_centroid(f));
            coarse
                .points(Stratum::Face)
                .into_iter()
                .find(|&g| {
                    let cs = coarse.face_sides(g);
                    let ag = coarse.face_area_vector(g);
                    let incl = cs.left.index == a || cs.right.is_some_and(|r| r.index == a);
                    incl && dot(af, ag) / (af.norm() * ag.norm()) > 0.99 && dot(cf - coarse.face_centroid(g), ag).abs() < 1e-12
                })
                .map(|g| g.index)
                .expect("every non-interior fine face lies on a coarse face")
        })
        .collect();
    let map = FineToCoarse { fine: fine.mesh_id(), coarse: coarse.mesh_id(), cell_map: cell_map.clone(), face_map: Some(face_map.clone()) };
    let mut col = MeshCollection::new(Some(id(EntityKind::TopoBody, 1)));
    col.add_mesh(fine.clone()).unwrap();
    col.add_mesh(coarse.clone()).unwrap();
    assert!(col.add_mesh(fine.clone()).is_err());
    col.add_map(map.clone()).unwrap();
    assert_eq!(col.levels_from(fine.mesh_id()), vec![fine.mesh_id(), coarse.mesh_id()]);
    let m = col.coarser_of(fine.mesh_id()).unwrap();
    assert_eq!(m.children(Point::cell(0)).len(), 4);
    assert_eq!(m.coarse_cell(Point::cell(5)), Point::cell(1));
    // Total mapped boundary-face area is conserved across levels (24 fine quarter-faces -> 10 coarse faces).
    let interior = face_map.iter().filter(|&&g| g == NONE).count();
    assert_eq!(interior, 36 - 4 - 24); // 36 faces: 24 boundary, 4 on the coarse interface, 8 interior to coarse cells
    // Corrupt maps are rejected.
    let mut short = map.clone();
    short.cell_map.pop();
    assert!(short.validate(fine.as_ref(), coarse.as_ref()).is_err());
    let mut oob = map.clone();
    oob.cell_map[0] = 5;
    assert!(oob.validate(fine.as_ref(), coarse.as_ref()).is_err());
    let mut swapped = map.clone();
    let fm = swapped.face_map.as_mut().unwrap();
    let i = fm.iter().position(|&g| g != NONE).unwrap();
    fm[i] = (fm[i] + 1) % coarse.count(Stratum::Face);
    assert!(swapped.validate(fine.as_ref(), coarse.as_ref()).is_err());
}

#[test]
fn mirror_unfold_flips_loops_and_reapplies_rule() {
    // Fundamental domain x in [0,1]; mirror plane x = 0 (improper transform).
    let proto = Arc::new(hex_block(id(EntityKind::Mesh, 1), 0, [2, 1, 1], [0.0; 3], [1.0; 3]).unwrap());
    let mirror = Transform { m: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(0.0, 0.0, 0.0) };
    assert!(mirror.is_improper());
    let run = || {
        let mut v = InstancedView::new();
        v.add_prototype(proto.clone());
        for (n, t) in [(1, Transform::IDENTITY), (2, mirror)] {
            v.add_instance(Instance {
                id: id(EntityKind::Instance, n),
                prototype: proto.mesh_id(),
                transform: t,
                weld_tol: Some(1e-12),
            })
            .unwrap();
        }
        v.flatten(MeshBuilder::new(id(EntityKind::Mesh, 9), 5000)).unwrap()
    };
    let (m, fmap) = run();
    assert_eq!(validate(&m), vec![], "all cells closed with positive volume under the outward rule");
    assert_eq!(m.count(Stratum::Cell), 4);
    assert_eq!(m.count(Stratum::Vertex), 12 + 8, "the 4 vertices on the mirror plane are shared");
    assert_eq!(m.count(Stratum::Face), 21);
    assert_eq!(m.boundary_faces().len(), 18);
    for c in m.points(Stratum::Cell) {
        assert!((m.cell_volume(c) - 0.5).abs() < 1e-14);
    }
    // The stitched face lies in x = 0, is interior, left = cell 0 (x>0 copy), right = its mirror.
    let seam: Vec<Point> = m.points(Stratum::Face).into_iter().filter(|&f| m.face_centroid(f).x.abs() < 1e-12).collect();
    assert_eq!(seam.len(), 1);
    let s = m.face_sides(seam[0]);
    assert_eq!((s.left, s.right), (Point::cell(0), Some(Point::cell(2))));
    assert!(m.face_area_vector(seam[0]).x < 0.0, "normal points from left (x>0) to right (x<0)");
    // Instance-path map.
    assert_eq!(fmap.instance_cells[1], (id(EntityKind::Instance, 2), 2..4));
    assert_eq!(fmap.cell_origin[2], CellOrigin { instance: id(EntityKind::Instance, 2), proto_cell: Point::cell(0) });
    // Mirrored cell centroids are mirror images.
    let (c0, c2) = (m.cell_centroid(Point::cell(0)), m.cell_centroid(Point::cell(2)));
    assert!((c0.x + c2.x).abs() < 1e-14 && (c0.y - c2.y).abs() < 1e-14);
    // Without welding, copies stay disjoint (no seam, 40 boundary faces... 2 blocks of 11 faces).
    let mut v = InstancedView::new();
    v.add_prototype(proto.clone());
    for (n, t) in [(1, Transform::IDENTITY), (2, mirror)] {
        v.add_instance(Instance { id: id(EntityKind::Instance, n), prototype: proto.mesh_id(), transform: t, weld_tol: None }).unwrap();
    }
    let (d, _) = v.flatten(MeshBuilder::new(id(EntityKind::Mesh, 9), 5000)).unwrap();
    assert_eq!(d.count(Stratum::Face), 22);
    assert_eq!(validate(&d), vec![]);
    // Deterministic: same input, same hash.
    assert_eq!(content_hash(&m), content_hash(&run().0));
}

#[test]
fn flatten_carries_classification_and_labels() {
    let tvol = id(EntityKind::TopoVolume, 1);
    let mut b = hex_block_builder(id(EntityKind::Mesh, 1), 0, [1, 1, 1], [0.0; 3], [1.0; 3]).unwrap();
    b.classify(Point::cell(0), Classification::on(tvol)).unwrap();
    b.set_label("material", Point::cell(0), 4).unwrap();
    let proto = Arc::new(b.finish().unwrap());
    let mut v = InstancedView::new();
    v.add_prototype(proto.clone());
    let shift = Transform { t: Vector3::new(2.0, 0.0, 0.0), ..Transform::IDENTITY };
    for (n, t) in [(1, Transform::IDENTITY), (2, shift)] {
        v.add_instance(Instance { id: id(EntityKind::Instance, n), prototype: proto.mesh_id(), transform: t, weld_tol: None }).unwrap();
    }
    let (m, _) = v.flatten(MeshBuilder::new(id(EntityKind::Mesh, 3), 100)).unwrap();
    assert_eq!(m.points_on(tvol).len(), 2);
    assert_eq!(m.label_points("material", 4).len(), 2);
    assert_eq!(validate(&m), vec![]);
}

#[test]
fn determinism_same_input_same_hash() {
    let a = unit_block([3, 2, 2], 1, 0);
    let b = unit_block([3, 2, 2], 1, 0);
    assert_eq!(content_hash(&a), content_hash(&b));
    let c = hex_block(id(EntityKind::Mesh, 1), 0, [3, 2, 2], [0.0; 3], [1.0, 1.0, 1.5]).unwrap();
    assert_ne!(content_hash(&a), content_hash(&c));
    // Canonical loops start at the smallest vertex index.
    for f in a.points(Stratum::Face) {
        let l = a.cone(f).indices;
        assert_eq!(l.iter().min(), l.first());
    }
}
