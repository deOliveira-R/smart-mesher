//! Orientation rule, shapes, polyhedra, high order, errors and determinism.
#![allow(missing_docs)]

mod common;

use common::*;
use sm_core::Point3;
use sm_mesh_api::reference::Mesh;
use sm_mesh_api::validate::{ValidateOptions, canonical_hash, validate};
use sm_mesh_api::{
    Basis, CellIx, CellShape, CellSpec, Dim, FaceIx, FaceShape, MeshBuilder, MeshEdit, MeshError, MeshGeometry,
    MeshTopology, Sign, VertexIx,
};

fn assert_valid(m: &Mesh) {
    let v = validate(m, &ValidateOptions::default());
    assert!(v.is_empty(), "violations: {v:?}");
}

#[test]
fn hex_block_2x2x2_orientation_rule() {
    let m = block(1, [2, 2, 2], [1.0, 1.0, 1.0]);
    assert_eq!(m.len(Dim::Vertex), 27);
    assert_eq!(m.len(Dim::Cell), 8);
    assert_eq!(m.len(Dim::Face), 36);
    assert_eq!(m.boundary_faces().len(), 24);
    assert_eq!(m.len(Dim::Edge), 54);
    assert_valid(&m);

    let mut interior = 0;
    for f in 0..m.len(Dim::Face) as u32 {
        let f = FaceIx(f);
        let left = m.face_left(f);
        let a = m.face_area_vector(f);
        assert!((a.norm() - 1.0).abs() < 1e-12);
        match m.face_right(f) {
            Some(right) => {
                interior += 1;
                assert!(left < right, "left must be the lower cell id");
                // normal points left -> right
                let d = m.cell_volume_centroid(right).1 - m.cell_volume_centroid(left).1;
                assert!(a.dot(d) > 0.0);
                // signs: + for left, - for right (CGNS NFACE)
                let sl = m.cell_faces(left).iter().find(|s| s.face() == f).map(|s| s.sign());
                let sr = m.cell_faces(right).iter().find(|s| s.face() == f).map(|s| s.sign());
                assert_eq!((sl, sr), (Some(Sign::Plus), Some(Sign::Minus)));
            }
            None => {
                // boundary: normal outward from left
                let d = m.face_centroid(f) - m.cell_volume_centroid(left).1;
                assert!(a.dot(d) > 0.0);
            }
        }
    }
    assert_eq!(interior, 12);
    for c in 0..8 {
        let c = CellIx(c);
        assert!((m.cell_volume(c) - 1.0).abs() < 1e-12);
        assert_eq!(m.cell_corners(c).map(<[_]>::len), Some(8));
        assert_eq!(m.cell_shape(c), CellShape::Hex);
        // outward area vectors of a closed cell sum to zero
        let s = m.cell_faces(c).iter().fold(sm_core::Vector3::default(), |acc, sf| acc + m.outward_area_vector(*sf));
        assert!(s.norm() < 1e-12);
    }
    // mesh walking: the corner cell has 3 neighbours; leaving by local face 0 (bottom) of cell 0 hits the boundary.
    assert_eq!(m.cell_neighbors(CellIx(0)).len(), 3);
    assert_eq!(m.next_cell(CellIx(0), 0), None);
    assert_eq!(m.next_cell(CellIx(0), 1), Some(CellIx(4))); // top -> cell above
    assert_eq!(m.next_cell(CellIx(0), 3), Some(CellIx(1))); // +x
    // id <-> index
    let id = m.id_of(Dim::Cell, 5).expect("id");
    assert_eq!(m.index_of(id), Some(5));
    assert_eq!(m.face_shape(FaceIx(0)), FaceShape::Quad);
}

#[test]
fn single_non_hex_polyhedron() {
    // Regular pentagonal prism, circumradius 1, height 2: 7 faces, no template.
    let mut b = Mesh::builder(mesh_id(1)).expect("id");
    let ring: Vec<[f64; 2]> = (0..5)
        .map(|i| {
            let a = std::f64::consts::TAU * f64::from(i) / 5.0;
            [a.cos(), a.sin()]
        })
        .collect();
    let bot: Vec<VertexIx> = ring.iter().map(|p| b.add_vertex(Point3::new(p[0], p[1], 0.0))).collect();
    let top: Vec<VertexIx> = ring.iter().map(|p| b.add_vertex(Point3::new(p[0], p[1], 2.0))).collect();
    let mut faces: Vec<Vec<VertexIx>> = vec![bot.iter().rev().copied().collect(), top.clone()];
    for i in 0..5 {
        let j = (i + 1) % 5;
        faces.push(vec![bot[i], bot[j], top[j], top[i]]);
    }
    let refs: Vec<&[VertexIx]> = faces.iter().map(Vec::as_slice).collect();
    let c = b.add_polyhedron(&refs).expect("polyhedron");
    let m = b.finish().expect("finish");
    assert_valid(&m);
    assert_eq!(m.cell_shape(c), CellShape::Polyhedron);
    assert!(m.cell_corners(c).is_none());
    assert_eq!(m.len(Dim::Face), 7);
    assert_eq!(m.boundary_faces().len(), 7);
    assert_eq!((0..7).filter(|&f| m.face_shape(FaceIx(f)) == FaceShape::Polygon(5)).count(), 2);
    let area = 2.5 * (std::f64::consts::TAU / 5.0).sin();
    assert!((m.cell_volume(c) - 2.0 * area).abs() < 1e-12);
    assert_eq!(m.cell_vertices(c).len(), 10);
}

#[test]
fn template_shapes_have_correct_volumes_and_survive_mirroring() {
    let p = |x, y, z| Point3::new(x, y, z);
    let cases: Vec<(CellShape, Vec<Point3>, f64)> = vec![
        (CellShape::Tet, vec![p(0., 0., 0.), p(1., 0., 0.), p(0., 1., 0.), p(0., 0., 1.)], 1.0 / 6.0),
        (
            CellShape::Hex,
            vec![
                p(0., 0., 0.),
                p(1., 0., 0.),
                p(1., 1., 0.),
                p(0., 1., 0.),
                p(0., 0., 1.),
                p(1., 0., 1.),
                p(1., 1., 1.),
                p(0., 1., 1.),
            ],
            1.0,
        ),
        (CellShape::Wedge, vec![p(0., 0., 0.), p(1., 0., 0.), p(0., 1., 0.), p(0., 0., 1.), p(1., 0., 1.), p(0., 1., 1.)], 0.5),
        (CellShape::Pyramid, vec![p(0., 0., 0.), p(1., 0., 0.), p(1., 1., 0.), p(0., 1., 0.), p(0.5, 0.5, 1.)], 1.0 / 3.0),
    ];
    for (shape, pts, vol) in cases {
        let mut b = Mesh::builder(mesh_id(1)).expect("id");
        let vs: Vec<VertexIx> = pts.iter().map(|&q| b.add_vertex(q)).collect();
        let c = b.add_cell(shape, &vs).expect("cell");
        let mut m = b.finish().expect("finish");
        assert_valid(&m);
        assert!((m.cell_volume(c) - vol).abs() < 1e-12, "{shape:?}");
        assert_eq!(m.len(Dim::Face), shape.local_faces().len());
        let before: Vec<Vec<u32>> =
            m.cell_faces(c).iter().map(|sf| m.face_vertices(sf.face()).iter().map(|v| v.0).collect()).collect();

        // improper transform: loops flip, template corners permute, rule still holds
        m.apply_transform(&mirror_x()).expect("mirror");
        assert_valid(&m);
        assert!((m.cell_volume(c) - vol).abs() < 1e-12);
        for (k, lf) in shape.local_faces().iter().enumerate() {
            let corners = m.cell_corners(c).expect("corners");
            let mut want: Vec<u32> = lf.iter().map(|&i| corners[i as usize].0).collect();
            let mut got: Vec<u32> = m.face_vertices(m.cell_faces(c)[k].face()).iter().map(|v| v.0).collect();
            want.sort_unstable();
            got.sort_unstable();
            assert_eq!(want, got, "local face {k} of {shape:?} follows the template after mirroring");
        }
        let after: Vec<Vec<u32>> =
            m.cell_faces(c).iter().map(|sf| m.face_vertices(sf.face()).iter().map(|v| v.0).collect()).collect();
        assert_ne!(before, after);
    }
}

#[test]
fn high_order_cell_and_curved_face() {
    let mut b = Mesh::builder(mesh_id(1)).expect("id");
    let mut nodes = Vec::new();
    // 3x3x3 lattice; corners must come first in the node list.
    let corner = |i: usize, j: usize, k: usize| i.is_multiple_of(2) && j.is_multiple_of(2) && k.is_multiple_of(2);
    let mut lattice = Vec::new();
    for k in 0..3 {
        for j in 0..3 {
            for i in 0..3 {
                lattice.push((i, j, k));
            }
        }
    }
    let order = [(0, 0, 0), (2, 0, 0), (2, 2, 0), (0, 2, 0), (0, 0, 2), (2, 0, 2), (2, 2, 2), (0, 2, 2)];
    for &(i, j, k) in &order {
        nodes.push(b.add_vertex(Point3::new(i as f64 / 2.0, j as f64 / 2.0, k as f64 / 2.0)));
    }
    for &(i, j, k) in lattice.iter().filter(|&&(i, j, k)| !corner(i, j, k)) {
        nodes.push(b.add_vertex(Point3::new(i as f64 / 2.0, j as f64 / 2.0, k as f64 / 2.0)));
    }
    let spec = CellSpec { shape: CellShape::Hex, degree: 2, basis: Basis::LagrangeGll };
    assert_eq!(CellShape::Hex.num_nodes(2), Some(27));
    assert!(matches!(b.add_high_order_cell(spec, &nodes[..26]), Err(MeshError::BadArity { .. })));
    let c = b.add_high_order_cell(spec, &nodes).expect("ho cell");
    let mut m = b.finish().expect("finish");
    assert_valid(&m);
    assert_eq!(m.cell_spec(c), spec);
    assert_eq!(m.cell_nodes(c).len(), 27);
    assert_eq!(m.cell_corners(c).map(<[_]>::len), Some(8));
    m.set_face_curve(FaceIx(0), 2, Basis::Bernstein, vec![VertexIx(0), VertexIx(1)]).expect("curve");
    assert_eq!(m.face_curve(FaceIx(0)).map(|f| f.degree), Some(2));
    assert!(m.face_curve(FaceIx(1)).is_none());
    // improper transforms of high-order cells are refused, not silently wrong
    assert!(matches!(m.apply_transform(&mirror_x()), Err(MeshError::Unsupported(_))));
}

#[test]
fn builder_rejects_bad_input() {
    let mut b = Mesh::builder(mesh_id(1)).expect("id");
    let v: Vec<VertexIx> = (0..5).map(|i| b.add_vertex(Point3::new(f64::from(i), 0.0, 0.0))).collect();
    assert!(matches!(b.add_cell(CellShape::Tet, &v[..3]), Err(MeshError::BadArity { .. })));
    assert!(matches!(b.add_cell(CellShape::Tet, &[v[0], v[1], v[2], VertexIx(99)]), Err(MeshError::OutOfRange { .. })));
    assert!(Mesh::builder(id(sm_core::EntityKind::MeshCell, 1)).is_err());

    // Two tets glued on a face but traversing it the same way: inconsistent orientation.
    let mut b = Mesh::builder(mesh_id(2)).expect("id");
    let p: Vec<VertexIx> = [(0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.), (1., 1., 1.)]
        .iter()
        .map(|&(x, y, z)| b.add_vertex(Point3::new(x, y, z)))
        .collect();
    b.add_cell(CellShape::Tet, &[p[0], p[1], p[2], p[3]]).expect("t0");
    // faces of t1 listed with the shared face (0,2,1) in the same direction as t0 uses it
    let t1: &[&[VertexIx]] = &[&[p[0], p[2], p[1]], &[p[0], p[1], p[4]], &[p[1], p[2], p[4]], &[p[2], p[0], p[4]]];
    b.add_polyhedron(t1).expect("t1");
    assert!(matches!(b.finish(), Err(MeshError::InconsistentOrientation(0, 1))));
}

#[test]
fn determinism_same_hash_across_runs_and_threads() {
    let h0 = canonical_hash(&block(1, [2, 2, 2], [1.0, 1.0, 1.0]));
    let h1 = canonical_hash(&block(1, [2, 2, 2], [1.0, 1.0, 1.0]));
    let h2 = std::thread::spawn(|| canonical_hash(&block(1, [2, 2, 2], [1.0, 1.0, 1.0]))).join().expect("thread");
    assert_eq!(h0, h1);
    assert_eq!(h0, h2);
    assert_ne!(h0, canonical_hash(&block(1, [2, 2, 2], [1.0, 1.0, 2.0])));
}
