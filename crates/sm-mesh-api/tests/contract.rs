//! Contract tests against the reference implementation.

use sm_core::{EntityId, EntityKind, Point3, Transform, Vector3};
use sm_mesh_api::reference::{ReferenceBuilder, ReferenceCatalog, ReferenceMesh};
use sm_mesh_api::*;

fn id(kind: EntityKind, n: u64) -> EntityId {
    EntityId::new(kind, n).unwrap()
}

/// Build an `n x n x n` hex block on `[0,n*h]^3` shifted by `x0`, classified like a CAD box:
/// 8 corner vertices, 12 edges, 6 faces, 1 volume (all `TopoVolume#1` etc.).
fn hex_block(mesh_serial: u64, n: usize, h: f64, x0: f64, alloc: &mut SerialAlloc) -> ReferenceMesh {
    let mut b = ReferenceBuilder::new(id(EntityKind::Mesh, mesh_serial)).unwrap();
    let m = n + 1;
    let ix = |i: usize, j: usize, k: usize| VertexIx::new((k * m + j) * m + i);
    for k in 0..m {
        for j in 0..m {
            for i in 0..m {
                let v = b.add_vertex(Point3::new(x0 + i as f64 * h, j as f64 * h, k as f64 * h)).unwrap();
                let at = |q: usize| u8::from(q == 0) + u8::from(q == n);
                let nb = at(i) + at(j) + at(k);
                let (kind, serial) = match nb {
                    3 => (EntityKind::TopoVertex, 1 + (i / n + 2 * (j / n) + 4 * (k / n)) as u64),
                    2 => (EntityKind::TopoEdge, 1 + (i + j + k) as u64),
                    1 => {
                        let (axis, side) = if at(i) == 1 { (0, i / n) } else if at(j) == 1 { (1, j / n) } else { (2, k / n) };
                        (EntityKind::TopoFace, 1 + (axis * 2 + side) as u64)
                    }
                    _ => (EntityKind::TopoVolume, 1),
                };
                b.set_class(MeshEntity::Vertex(v), Classification::on(id(kind, serial))).unwrap();
            }
        }
    }
    for k in 0..n {
        for j in 0..n {
            for i in 0..n {
                let vs = [
                    ix(i, j, k),
                    ix(i + 1, j, k),
                    ix(i + 1, j + 1, k),
                    ix(i, j + 1, k),
                    ix(i, j, k + 1),
                    ix(i + 1, j, k + 1),
                    ix(i + 1, j + 1, k + 1),
                    ix(i, j + 1, k + 1),
                ];
                let c = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
                b.set_class(MeshEntity::Cell(c), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
            }
        }
    }
    classify_faces(&mut b, n, h, x0);
    b.finish(alloc).unwrap()
}

fn classify_faces(b: &mut ReferenceBuilder, n: usize, h: f64, x0: f64) {
    let ext = n as f64 * h;
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        let class = if b.face_degree(f) == 1 {
            let pts: Vec<Point3> = b.face_loop(f).iter().map(|&v| b.vertex_point(v)).collect();
            let same = |g: &dyn Fn(&Point3) -> f64, t: f64| pts.iter().all(|p| (g(p) - t).abs() < 1e-12);
            let serial = if same(&|p| p.x, x0) {
                1
            } else if same(&|p| p.x, x0 + ext) {
                2
            } else if same(&|p| p.y, 0.0) {
                3
            } else if same(&|p| p.y, ext) {
                4
            } else if same(&|p| p.z, 0.0) {
                5
            } else {
                6
            };
            Classification::on(id(EntityKind::TopoFace, serial))
        } else {
            Classification::on(id(EntityKind::TopoVolume, 1))
        };
        b.set_class(MeshEntity::Face(f), class).unwrap();
    }
}

#[test]
fn hex_block_2x2x2_orientation_and_boundary() {
    let mut alloc = SerialAlloc::new();
    let m = hex_block(1, 2, 0.5, 0.0, &mut alloc);
    assert_eq!((m.vertex_count(), m.face_count(), m.cell_count()), (27, 36, 8));
    assert!(validate(&m).is_ok(), "{:?}", validate(&m));
    assert!(validate_geometry(&m).is_ok(), "{:?}", validate_geometry(&m));
    let interior: Vec<FaceIx> = (0..36).map(FaceIx::new).filter(|&f| !m.is_boundary_face(f)).collect();
    assert_eq!(interior.len(), 12);
    assert_eq!(m.boundary_faces().len(), 24);
    for f in interior {
        let (l, r) = m.face_cells(f);
        let r = r.unwrap();
        assert!(l < r, "left must be the lower cell");
        assert!(m.cell_id(l) < m.cell_id(r));
        // Normal points left -> right: axis-aligned here, so compare with centre offsets.
        let n = face_area_dir(&m, f);
        let d = cell_centre(&m, r) - cell_centre(&m, l);
        assert!(n.dot(d) > 0.0);
        // Sides recorded in cell_faces.
        assert!(m.cell_faces(l).contains(&CellFace { face: f, side: Side::Left }));
        assert!(m.cell_faces(r).contains(&CellFace { face: f, side: Side::Right }));
    }
    // Boundary normals point outward; each cell is a unit-ish hex with volume 0.125.
    for f in m.boundary_faces() {
        let c = m.face_left(f);
        let n = face_area_dir(&m, f);
        let fc = sm_mesh_api::geometry::face_geometry(&m, f).centroid;
        assert!(n.dot(fc - cell_centre(&m, c)) > 0.0);
        assert!(m.face_cells(f).1.is_none());
    }
    for c in 0..8 {
        assert!((geometry::cell_volume(&m, CellIx::new(c)) - 0.125).abs() < 1e-14);
        assert_eq!(m.cell_shape(CellIx::new(c)), CellShape::Hex);
        assert_eq!(m.cell_ordered_vertices(CellIx::new(c)).unwrap().len(), 8);
        assert_eq!(m.cell_faces(CellIx::new(c)).len(), 6);
    }
    // Mesh walking: from cell 0 the neighbours are 1 (x), 2 (y), 4 (z).
    let mut nb: Vec<usize> = m.cell_neighbors(CellIx(0)).iter().map(|c| c.get()).collect();
    nb.sort_unstable();
    assert_eq!(nb, vec![1, 2, 4]);
    // Id lookups round trip.
    for c in 0..8 {
        assert_eq!(m.cell_index(m.cell_id(CellIx::new(c))), Some(CellIx::new(c)));
    }
}

fn face_area_dir(m: &ReferenceMesh, f: FaceIx) -> Vector3 {
    geometry::face_area_vector(m, f)
}
fn cell_centre(m: &ReferenceMesh, c: CellIx) -> Point3 {
    geometry::cell_volume_centroid(m, c).1
}

#[test]
fn single_pentagonal_prism_polyhedron() {
    let mut b = ReferenceBuilder::new(id(EntityKind::Mesh, 1)).unwrap();
    let (r, hgt) = (1.0_f64, 2.0_f64);
    let ring = |z: f64, b: &mut ReferenceBuilder| -> Vec<VertexIx> {
        (0..5)
            .map(|i| {
                let a = 2.0 * std::f64::consts::PI * f64::from(i) / 5.0;
                let v = b.add_vertex(Point3::new(r * a.cos(), r * a.sin(), z)).unwrap();
                b.set_class(MeshEntity::Vertex(v), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
                v
            })
            .collect()
    };
    let bot = ring(0.0, &mut b);
    let top = ring(hgt, &mut b);
    let mut loops: Vec<Vec<VertexIx>> = vec![bot.iter().rev().copied().collect(), top.clone()];
    for i in 0..5 {
        let j = (i + 1) % 5;
        loops.push(vec![bot[i], bot[j], top[j], top[i]]);
    }
    let c = b.add_polyhedron(&loops).unwrap();
    b.set_class(MeshEntity::Cell(c), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
    for f in 0..b.face_count() {
        b.set_class(MeshEntity::Face(FaceIx::new(f)), Classification::on(id(EntityKind::TopoFace, 1 + f as u64))).unwrap();
    }
    let m = b.finish(&mut SerialAlloc::new()).unwrap();
    assert_eq!(m.cell_shape(CellIx(0)), CellShape::Polyhedron);
    assert!(m.cell_ordered_vertices(CellIx(0)).is_none());
    assert_eq!(m.face_count(), 7);
    assert_eq!(m.boundary_faces().len(), 7);
    assert!(validate(&m).is_ok() && validate_geometry(&m).is_ok());
    let area = 2.5 * r * r * (2.0 * std::f64::consts::PI / 5.0).sin();
    let (vol, cen) = geometry::cell_volume_centroid(&m, CellIx(0));
    assert!((vol - area * hgt).abs() < 1e-12, "{vol}");
    assert!((cen.z - hgt / 2.0).abs() < 1e-12 && cen.x.abs() < 1e-12);
    let shapes: Vec<FaceShape> = (0..7).map(|f| m.face_shape(FaceIx::new(f))).collect();
    assert_eq!(shapes.iter().filter(|&&s| s == FaceShape::Polygon).count(), 2);
    assert_eq!(shapes.iter().filter(|&&s| s == FaceShape::Quad).count(), 5);
}

#[test]
fn fast_path_shapes_are_right_handed() {
    // One of each fast-path shape on its reference vertices; geometry validation checks volume > 0.
    let cases: [(CellShape, Vec<[f64; 3]>); 4] = [
        (CellShape::Tet, vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]),
        (CellShape::Wedge, vec![[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [0., 1., 1.]]),
        (CellShape::Pyramid, vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0.5, 0.5, 1.]]),
        (CellShape::Hex, vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]]),
    ];
    let expect = [1.0 / 6.0, 0.5, 1.0 / 3.0, 1.0];
    for ((shape, pts), vol) in cases.iter().zip(expect) {
        let mut b = ReferenceBuilder::new(id(EntityKind::Mesh, 1)).unwrap();
        let vs: Vec<VertexIx> = pts
            .iter()
            .map(|p| {
                let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
                b.set_class(MeshEntity::Vertex(v), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
                v
            })
            .collect();
        let c = b.add_cell_from_vertices(*shape, &vs).unwrap();
        b.set_class(MeshEntity::Cell(c), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
        for f in 0..b.face_count() {
            b.set_class(MeshEntity::Face(FaceIx::new(f)), Classification::on(id(EntityKind::TopoFace, 1))).unwrap();
        }
        let m = b.finish(&mut SerialAlloc::new()).unwrap();
        assert_eq!(m.cell_shape(CellIx(0)), *shape);
        assert!(validate(&m).is_ok() && validate_geometry(&m).is_ok(), "{shape:?}");
        assert!((geometry::cell_volume(&m, CellIx(0)) - vol).abs() < 1e-12, "{shape:?}");
    }
}

#[test]
fn classification_queries() {
    let mut alloc = SerialAlloc::new();
    let m = hex_block(1, 2, 0.5, 0.0, &mut alloc);
    let vol = id(EntityKind::TopoVolume, 1);
    let set = m.classified_on(vol).unwrap();
    assert_eq!((set.cells.len(), set.faces.len(), set.vertices.len()), (8, 12, 1));
    // Face x=0 (TopoFace#1): 4 boundary faces, and one interior vertex (the others lie on edges/corners).
    let f1 = m.classified_on(id(EntityKind::TopoFace, 1)).unwrap();
    assert_eq!((f1.faces.len(), f1.vertices.len(), f1.cells.len()), (4, 1, 0));
    for &f in &f1.faces {
        assert!(m.is_boundary_face(f));
        assert_eq!(m.face_class(f).entity, id(EntityKind::TopoFace, 1));
    }
    let corners = (1..=8).filter(|&i| m.classified_on(id(EntityKind::TopoVertex, i)).is_some_and(|s| s.vertices.len() == 1)).count();
    assert_eq!(corners, 8);
    // Every boundary face is classified on a TopoFace; reverse index is ascending.
    assert!(m.boundary_faces().iter().all(|&f| m.face_class(f).entity.kind() == EntityKind::TopoFace));
    assert!(set.cells.windows(2).all(|w| w[0] < w[1]));
    assert_eq!(m.classified_entities().first().copied(), Some(id(EntityKind::TopoVertex, 1)));
    // Parametric coordinates round trip through an edit.
    let mut m = m;
    let v = VertexIx(13); // interior face-centre vertex of the x=0 plane? use any vertex
    let c = Classification { entity: id(EntityKind::TopoFace, 9), param: Param::Surface([0.25, 0.75]) };
    m.set_class(MeshEntity::Vertex(v), c).unwrap();
    assert_eq!(m.vertex_class(v), c);
    assert!(m.classified_on(id(EntityKind::TopoFace, 9)).unwrap().vertices.contains(&v));
    assert!(m.classified_on(vol).is_none() || !m.classified_on(vol).unwrap().vertices.contains(&v));
    assert!(m.set_class(MeshEntity::Vertex(v), Classification::on(id(EntityKind::MeshCell, 0))).is_err());
}

#[test]
fn unclassified_entities_are_rejected() {
    let mut b = ReferenceBuilder::new(id(EntityKind::Mesh, 1)).unwrap();
    b.add_vertex(Point3::new(0.0, 0.0, 0.0)).unwrap();
    assert!(matches!(b.finish(&mut SerialAlloc::new()), Err(MeshError::Unclassified(_))));
}

#[test]
fn two_level_hierarchy_map() {
    let mut alloc = SerialAlloc::new();
    let fine = hex_block(1, 2, 0.5, 0.0, &mut alloc);
    // Coarse: 2 x 1 x 1 hexes over the same box (independent mesh, same model volume).
    let mut b = ReferenceBuilder::new(id(EntityKind::Mesh, 2)).unwrap();
    for k in 0..2 {
        for j in 0..2 {
            for i in 0..3 {
                let v = b.add_vertex(Point3::new(i as f64 * 0.5, j as f64, k as f64)).unwrap();
                b.set_class(MeshEntity::Vertex(v), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
            }
        }
    }
    let ix = |i: usize, j: usize, k: usize| VertexIx::new((k * 2 + j) * 3 + i);
    for i in 0..2 {
        let vs = [ix(i, 0, 0), ix(i + 1, 0, 0), ix(i + 1, 1, 0), ix(i, 1, 0), ix(i, 0, 1), ix(i + 1, 0, 1), ix(i + 1, 1, 1), ix(i, 1, 1)];
        let c = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
        b.set_class(MeshEntity::Cell(c), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
    }
    for f in 0..b.face_count() {
        let f = FaceIx::new(f);
        let class = if b.face_degree(f) == 1 { id(EntityKind::TopoFace, 1) } else { id(EntityKind::TopoVolume, 1) };
        b.set_class(MeshEntity::Face(f), Classification::on(class)).unwrap();
    }
    let coarse = b.finish(&mut alloc).unwrap();
    assert!(validate(&coarse).is_ok() && validate_geometry(&coarse).is_ok());

    // Fine cell (i,j,k) at index i + 2j + 4k lies in coarse cell i.
    let coarse_of: Vec<CellIx> = (0..8).map(|c| CellIx::new(c % 2)).collect();
    let map = CellMap::new(2, coarse_of).unwrap();
    assert_eq!(map.fines_of(CellIx(0)), &[CellIx(0), CellIx(2), CellIx(4), CellIx(6)]);
    assert_eq!(map.coarse_of(CellIx(7)), CellIx(1));
    // Volumes aggregate exactly.
    for c in 0..2 {
        let cv = geometry::cell_volume(&coarse, CellIx::new(c));
        let sum: f64 = map.fines_of(CellIx::new(c)).iter().map(|&f| geometry::cell_volume(&fine, f)).sum();
        assert!((cv - sum).abs() < 1e-14);
    }
    assert!(CellMap::new(3, (0..8).map(|c| CellIx::new(c % 2)).collect()).is_err(), "a coarse cell without children");

    let (fid, cid) = (fine.mesh_id(), coarse.mesh_id());
    let mut cat = ReferenceCatalog::new();
    cat.insert_mesh(Box::new(fine), 0).unwrap();
    cat.insert_mesh(Box::new(coarse), 1).unwrap();
    cat.link_levels(fid, cid, map).unwrap();
    assert_eq!(cat.mesh_ids(), vec![fid, cid]);
    assert_eq!(cat.level(cid), Some(1));
    let (up, m) = cat.coarse_of(fid).unwrap();
    assert_eq!((up, m.fines_of(CellIx(1)).len()), (cid, 4));
    assert!(cat.coarse_of(cid).is_none());
    // Wrong sizes are rejected.
    let bad = CellMap::new(1, vec![CellIx(0); 8]).unwrap();
    assert!(cat.link_levels(fid, cid, bad).is_err());
}

fn mirror_x() -> Transform {
    Transform { m: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::default() }
}

#[test]
fn mirror_unfold_flips_loops_and_reapplies_rule() {
    let mut alloc = SerialAlloc::new();
    let half = hex_block(1, 2, 0.5, 0.0, &mut alloc);
    let t = mirror_x();
    assert!(t.is_improper());
    // Mirror plane x = 0 is TopoFace#1 of the reduced model; in the unfolded model those faces
    // become interior, so remap the mirror image classification of other entities by identity.
    let b = ReferenceBuilder::new(id(EntityKind::Mesh, 2)).unwrap();
    let full = unfold(&half, &t, 1e-12, None, b, &mut alloc).unwrap();
    assert_eq!(full.cell_count(), 16);
    assert_eq!(full.vertex_count(), 27 + 27 - 9);
    assert_eq!(full.face_count(), 36 + 36 - 4);
    assert!(validate(&full).is_ok(), "{:?}", validate(&full));
    assert!(validate_geometry(&full).is_ok(), "{:?}", validate_geometry(&full));
    assert_eq!((0..full.face_count()).filter(|&f| full.is_boundary_face(FaceIx::new(f))).count(), 40);
    // Shared plane faces: left is an original cell (<8), right a mirrored cell (>=8), normal is -x.
    let mut shared = 0;
    for f in 0..full.face_count() {
        let f = FaceIx::new(f);
        if let (l, Some(r)) = full.face_cells(f)
            && l.get() < 8 && r.get() >= 8 {
                shared += 1;
                let n = geometry::face_area_vector(&full, f);
                assert!(n.x < 0.0 && n.y.abs() < 1e-14 && n.z.abs() < 1e-14, "{n:?}");
            }
    }
    assert_eq!(shared, 4);
    // Mirrored fast-path cells keep a valid right-handed template (validate checks the template
    // faces against the outward loops) and positive volume.
    for c in 8..16 {
        let c = CellIx::new(c);
        assert_eq!(full.cell_shape(c), CellShape::Hex);
        assert!(full.cell_ordered_vertices(c).is_some());
        assert!((geometry::cell_volume(&full, c) - 0.125).abs() < 1e-14);
    }
    // Total volume of the unfolded block.
    let vol: f64 = (0..16).map(|c| geometry::cell_volume(&full, CellIx::new(c))).sum();
    assert!((vol - 2.0).abs() < 1e-13);
}

#[test]
fn naive_mirror_without_loop_flip_would_fail_validation() {
    // Guard for the test above: place mirrored vertices but reuse un-reversed template order.
    let mut b = ReferenceBuilder::new(id(EntityKind::Mesh, 1)).unwrap();
    let pts = [[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.], [0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
    let vs: Vec<VertexIx> = pts
        .iter()
        .map(|p| {
            let v = b.add_vertex(Point3::new(-p[0], p[1], p[2])).unwrap();
            b.set_class(MeshEntity::Vertex(v), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
            v
        })
        .collect();
    let c = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
    b.set_class(MeshEntity::Cell(c), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
    for f in 0..6 {
        b.set_class(MeshEntity::Face(FaceIx::new(f)), Classification::on(id(EntityKind::TopoFace, 1))).unwrap();
    }
    let m = b.finish(&mut SerialAlloc::new()).unwrap();
    assert!(!validate_geometry(&m).is_ok(), "an inside-out hex must be caught by the geometric check");
}

#[test]
fn flatten_instances_is_deterministic_and_mapped() {
    let mut alloc = SerialAlloc::new();
    let proto = hex_block(1, 1, 1.0, 0.0, &mut alloc);
    let pid = proto.mesh_id();
    let mut cat = ReferenceCatalog::new();
    cat.insert_mesh(Box::new(proto), 0).unwrap();
    let shift = |x: f64| Transform { t: Vector3::new(x, 0.0, 0.0), ..Transform::IDENTITY };
    let mk = |order: [u64; 3]| InstancedView {
        id: id(EntityKind::Instance, 100),
        instances: order
            .iter()
            .map(|&s| Instance {
                id: id(EntityKind::Instance, s),
                prototype: pid,
                transform: if s == 2 { mirror_about(5.0) } else { shift(s as f64 * 2.0) },
            })
            .collect(),
    };
    let run = |view: &InstancedView| {
        let mut a = SerialAlloc::new();
        let b = ReferenceBuilder::new(id(EntityKind::Mesh, 9)).unwrap();
        flatten(view, &cat, b, None, &mut a).unwrap()
    };
    let (m1, map1) = run(&mk([1, 2, 3]));
    let (m2, _) = run(&mk([3, 1, 2]));
    assert_eq!(content_hash(&m1), content_hash(&m2), "instance listing order must not matter");
    assert_eq!(m1, m2);
    assert_eq!(m1.cell_count(), 3);
    assert!(validate(&m1).is_ok() && validate_geometry(&m1).is_ok());
    assert_eq!(map1.locate(CellIx(1)), Some((id(EntityKind::Instance, 2), CellIx(0))));
    assert_eq!(map1.locate(CellIx(5)), None);
}

fn mirror_about(x: f64) -> Transform {
    Transform { m: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t: Vector3::new(2.0 * x, 0.0, 0.0) }
}

#[test]
fn canonical_output_is_independent_of_face_creation_order() {
    // Same two tets sharing a face, faces created in different orders.
    let build = |reverse: bool| {
        let mut b = ReferenceBuilder::new(id(EntityKind::Mesh, 1)).unwrap();
        let p = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1., 1., 1.]];
        let v: Vec<VertexIx> = p
            .iter()
            .map(|p| {
                let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
                b.set_class(MeshEntity::Vertex(v), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
                v
            })
            .collect();
        // Tet A: 0,1,2,3 ; Tet B shares face (1,2,3), apex 4. Outward loops given explicitly.
        let a = vec![vec![v[0], v[2], v[1]], vec![v[0], v[1], v[3]], vec![v[1], v[2], v[3]], vec![v[2], v[0], v[3]]];
        let bl = vec![vec![v[1], v[3], v[2]], vec![v[1], v[2], v[4]], vec![v[2], v[3], v[4]], vec![v[3], v[1], v[4]]];
        let mut pre: Vec<&Vec<VertexIx>> = a.iter().chain(bl.iter()).collect();
        if reverse {
            pre.reverse();
        }
        for l in pre {
            if b.find_face(l).is_none() {
                b.add_face(l).unwrap();
            }
        }
        for loops in [&a, &bl] {
            let c = b.add_polyhedron(loops).unwrap();
            b.set_class(MeshEntity::Cell(c), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
        }
        for f in 0..b.face_count() {
            let f = FaceIx::new(f);
            let k = if b.face_degree(f) == 1 { EntityKind::TopoFace } else { EntityKind::TopoVolume };
            b.set_class(MeshEntity::Face(f), Classification::on(id(k, 1))).unwrap();
        }
        let faces: Vec<FaceIx> = (0..b.face_count()).map(FaceIx::new).collect();
        b.add_set("all", SetMembers::Faces(faces)).unwrap();
        b.finish(&mut SerialAlloc::new()).unwrap()
    };
    let (m1, m2) = (build(false), build(true));
    assert_eq!(m1, m2);
    assert_eq!(content_hash(&m1), content_hash(&m2));
    assert_eq!(m1.face_count(), 7);
    // Cell shapes are derived: both are tets built without ordered vertices.
    assert_eq!(m1.cell_shape(CellIx(0)), CellShape::Tet);
    assert!(m1.cell_ordered_vertices(CellIx(0)).is_none());
    assert!(validate(&m1).is_ok() && validate_geometry(&m1).is_ok());
    assert_eq!(m1.set_by_name("all").unwrap().members.len(), 7);
}

#[test]
fn inconsistent_orientation_is_an_error() {
    // Two tets claiming the same face in the same direction (second one inverted).
    let mut b = ReferenceBuilder::new(id(EntityKind::Mesh, 1)).unwrap();
    let v: Vec<VertexIx> = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [0., 0., 1.], [1., 1., 1.]]
        .iter()
        .map(|p| b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap())
        .collect();
    let a = vec![vec![v[0], v[2], v[1]], vec![v[0], v[1], v[3]], vec![v[1], v[2], v[3]], vec![v[2], v[0], v[3]]];
    // Inverted second tet: shared face given in the same direction as tet A's.
    let bl = vec![vec![v[1], v[2], v[3]], vec![v[1], v[4], v[2]], vec![v[2], v[4], v[3]], vec![v[3], v[4], v[1]]];
    b.add_polyhedron(&a).unwrap();
    b.add_polyhedron(&bl).unwrap();
    for &vi in &v {
        b.set_class(MeshEntity::Vertex(vi),Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
    }
    for c in 0..2 {
        b.set_class(MeshEntity::Cell(CellIx(c)), Classification::on(id(EntityKind::TopoVolume, 1))).unwrap();
    }
    for f in 0..b.face_count() {
        b.set_class(MeshEntity::Face(FaceIx::new(f)), Classification::on(id(EntityKind::TopoFace, 1))).unwrap();
    }
    assert!(matches!(b.finish(&mut SerialAlloc::new()), Err(MeshError::InconsistentOrientation(_))));
}

#[test]
fn high_order_sets_tags_and_edges() {
    let mut alloc = SerialAlloc::new();
    let mut m = hex_block(1, 1, 1.0, 0.0, &mut alloc);
    // Linear by default.
    let g = m.cell_geometry(CellIx(0));
    assert_eq!((g.shape, g.degree), (CellShape::Hex, 1));
    // Edit: promote to degree 2 with one extra node reference to an existing vertex list.
    let nodes: Vec<u32> = (0..8).collect();
    m.set_high_order(CellIx(0), HighOrderCell { degree: 2, basis: Basis::LagrangeGll, nodes }).unwrap();
    assert_eq!(m.cell_geometry(CellIx(0)).degree, 2);
    assert!(m.set_high_order(CellIx(0), HighOrderCell { degree: 2, basis: Basis::Bernstein, nodes: vec![99] }).is_err());
    m.clear_high_order(CellIx(0)).unwrap();
    assert!(m.high_order_cell(CellIx(0)).is_none());
    // Sets and tags.
    let bf = m.boundary_faces();
    let sid = m.add_set("walls", SetMembers::Faces(bf.clone()), &mut alloc).unwrap();
    assert_eq!(m.set_by_name("walls").unwrap().id, sid);
    assert!(m.add_set("walls", SetMembers::Faces(bf), &mut alloc).is_err());
    m.set_tag("mat", MeshEntityKind::Cell, vec![7]).unwrap();
    assert_eq!(m.tag_by_name("mat").unwrap().values, vec![7]);
    assert!(m.set_tag("bad", MeshEntityKind::Cell, vec![1, 2]).is_err());
    // Edges: a cube has 12.
    m.ensure_edges(&mut alloc).unwrap();
    let e = m.edges().unwrap();
    assert_eq!(e.len(), 12);
    assert_eq!(e.of_face(FaceIx(0)).len(), 4);
    m.set_class(MeshEntity::Edge(EdgeIx(0)), Classification::on(id(EntityKind::TopoEdge, 1))).unwrap();
    assert!(m.edges().unwrap().class[0].is_some());
    // Moving a point keeps topology valid; geometry changes.
    m.set_point(0, Point3::new(-0.1, 0.0, 0.0)).unwrap();
    assert!(validate(&m).is_ok());
    assert!(geometry::cell_volume(&m, CellIx(0)) > 1.0);
}

#[test]
fn determinism_same_inputs_same_hash() {
    let h = || {
        let mut a = SerialAlloc::new();
        content_hash(&hex_block(1, 3, 0.25, 0.0, &mut a))
    };
    assert_eq!(h(), h());
    let mut a = SerialAlloc::new();
    let m = hex_block(1, 2, 1.0, 0.0, &mut a);
    let mut m2 = m.clone();
    m2.set_point(0, Point3::new(0.0, 0.0, 1e-9)).unwrap();
    assert_ne!(content_hash(&m), content_hash(&m2));
}
