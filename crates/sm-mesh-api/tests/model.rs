//! Classification, tags/sets/selectors, hierarchy and instancing/unfold.
#![allow(missing_docs)]

mod common;

use common::*;
use sm_core::{EntityKind, Point3, Transform, Vector3};
use sm_mesh_api::hierarchy::{FlattenOptions, Hierarchy, Instance, InstancedMesh, LevelMap};
use sm_mesh_api::orient::reversed_loop;
use sm_mesh_api::reference::Mesh;
use sm_mesh_api::schema::{TagDType, TagDef};
use sm_mesh_api::tag::TagRow;
use sm_mesh_api::validate::{ValidateOptions, canonical_hash, validate};
use sm_mesh_api::{
    CellIx, Classification, Dim, EdgeIx, FaceIx, MeshBuilder, MeshClassification, MeshEdit, MeshError, MeshGeometry,
    MeshTags, MeshTopology, Param, Selector, VertexIx,
};

const N: usize = 2;

/// Boundary constraints `(axis, side)` of a lattice vertex of the 2x2x2 block.
fn constraints(v: VertexIx) -> Vec<(usize, usize)> {
    let i = v.index();
    let c = [i % 3, (i / 3) % 3, i / 9];
    (0..3).filter(|&a| c[a] != 1).map(|a| (a, c[a] / 2)).collect()
}

fn model_face(a: usize, s: usize) -> sm_core::EntityId {
    id(EntityKind::TopoFace, (a * 2 + s + 1) as u64)
}

fn model_for(cons: &[(usize, usize)]) -> sm_core::EntityId {
    match cons.len() {
        0 => id(EntityKind::TopoVolume, 1),
        1 => model_face(cons[0].0, cons[0].1),
        2 => id(EntityKind::TopoEdge, 100 + (cons[0].0 * 10 + cons[1].0) as u64 * 4 + (cons[0].1 * 2 + cons[1].1) as u64),
        _ => id(EntityKind::TopoVertex, 200 + (cons[0].1 * 4 + cons[1].1 * 2 + cons[2].1) as u64),
    }
}

fn classified_block() -> Mesh {
    let mut b = block_builder(1, [N, N, N], [1.0; 3]);
    let nv = (N + 1).pow(3);
    for v in (0..nv as u32).map(VertexIx) {
        let cons = constraints(v);
        let p = v.index();
        let c = [(p % 3) as f64 / 2.0, ((p / 3) % 3) as f64 / 2.0, (p / 9) as f64 / 2.0];
        let param = match cons.as_slice() {
            [(a, _)] => {
                let rest: Vec<f64> = (0..3).filter(|x| x != a).map(|x| c[x]).collect();
                Param::Surface([rest[0], rest[1]])
            }
            [(a, _), (bb, _)] => Param::Curve(c[3 - a - bb]),
            _ => Param::None,
        };
        b.classify_vertex(v, Classification { entity: model_for(&cons), param }).expect("classify vertex");
    }
    for c in 0..8 {
        b.classify_cell(CellIx(c), id(EntityKind::TopoVolume, 1)).expect("classify cell");
    }
    let mut m = b.finish().expect("finish");
    // Faces and edges are derived at finish, so classify them from their vertices afterwards.
    let common = |vs: &[VertexIx]| -> Vec<(usize, usize)> {
        let mut acc = constraints(vs[0]);
        for v in &vs[1..] {
            let cv = constraints(*v);
            acc.retain(|x| cv.contains(x));
        }
        acc
    };
    for f in 0..m.len(Dim::Face) as u32 {
        let cons = common(m.face_vertices(FaceIx(f)));
        m.set_classification(Dim::Face, f, Some(Classification::on(model_for(&cons)))).expect("face class");
    }
    for e in 0..m.len(Dim::Edge) as u32 {
        let cons = common(&m.edge_vertices(EdgeIx(e)));
        m.set_classification(Dim::Edge, e, Some(Classification::on(model_for(&cons)))).expect("edge class");
    }
    m
}

#[test]
fn classification_queries() {
    let m = classified_block();
    let opts = ValidateOptions { require_classification: true, ..Default::default() };
    assert_eq!(validate(&m, &opts), vec![]);

    // 4 faces per boundary side, 24 boundary faces on 6 model faces, interior faces on the volume
    let x0 = model_face(0, 0);
    assert_eq!(m.entities_on(Dim::Face, x0).len(), 4);
    let on_faces: usize = (0..3).flat_map(|a| (0..2).map(move |s| (a, s))).map(|(a, s)| m.entities_on(Dim::Face, model_face(a, s)).len()).sum();
    assert_eq!(on_faces, 24);
    assert_eq!(m.entities_on(Dim::Face, id(EntityKind::TopoVolume, 1)).len(), 12);
    // 8 corner vertices on model vertices, centre on the volume
    assert_eq!(m.entities_on(Dim::Vertex, id(EntityKind::TopoVolume, 1)).len(), 1);
    let on_model_vertices: usize =
        m.classified_models(Dim::Vertex).iter().filter(|e| e.kind() == EntityKind::TopoVertex).map(|e| m.entities_on(Dim::Vertex, *e).len()).sum();
    assert_eq!(on_model_vertices, 8);
    assert_eq!(m.entities_on(Dim::Cell, id(EntityKind::TopoVolume, 1)).len(), 8);
    // parametric coordinates: vertex 1 is the midpoint of the bottom front edge -> Curve(u)
    let c = m.classification(Dim::Vertex, 1).expect("classified");
    assert_eq!(c.param, Param::Curve(0.5));
    // face-centred vertex on x=0: index 3 + 9 -> (0, 1, 1)? i=0,j=1,k=1 -> 9 + 3 = 12
    let c = m.classification(Dim::Vertex, 12).expect("classified");
    assert_eq!(c.entity, x0);
    assert_eq!(c.param, Param::Surface([0.5, 0.5]));
    // reverse index stays in sync on reclassification
    let mut m = m;
    m.set_classification(Dim::Cell, 0, None).expect("clear");
    assert_eq!(m.entities_on(Dim::Cell, id(EntityKind::TopoVolume, 1)).len(), 7);
    assert!(m.classification(Dim::Cell, 0).is_none());
    // dimension rule: a cell cannot sit on a model face
    assert!(matches!(
        m.set_classification(Dim::Cell, 0, Some(Classification::on(x0))),
        Err(MeshError::BadClassification { .. })
    ));
}

#[test]
fn tags_sets_and_selectors() {
    let mut m = classified_block();
    m.define_tag(TagDef { name: "material".into(), dim: Dim::Cell, dtype: TagDType::I64, width: 1 }).expect("tag");
    for c in 0..8u32 {
        m.set_tag(Dim::Cell, "material", c, TagRow::I64(&[i64::from(c / 4) + 1])).expect("set tag");
    }
    assert!(matches!(
        m.set_tag(Dim::Cell, "material", 0, TagRow::F64(&[1.0])),
        Err(MeshError::TagMismatch(_))
    ));
    m.define_set("odd", Dim::Cell, vec![7, 1, 3, 5, 3]).expect("set");
    assert_eq!(m.set("odd").expect("set").members(), &[1, 3, 5, 7]);
    assert_eq!(validate(&m, &ValidateOptions::default()), vec![]);

    let mat2 = Selector::TagEq("material".into(), 2);
    assert_eq!(mat2.eval(&m, Dim::Cell).expect("eval"), vec![4, 5, 6, 7]);
    let odd_mat2 = Selector::Intersect(vec![mat2.clone(), Selector::Set("odd".into())]);
    assert_eq!(odd_mat2.eval(&m, Dim::Cell).expect("eval"), vec![5, 7]);
    let not_odd = Selector::Complement(Box::new(Selector::Set("odd".into())));
    assert_eq!(not_odd.eval(&m, Dim::Cell).expect("eval"), vec![0, 2, 4, 6]);
    let diff = Selector::Difference(Box::new(mat2), Box::new(Selector::Set("odd".into())));
    assert_eq!(diff.eval(&m, Dim::Cell).expect("eval"), vec![4, 6]);

    // boundary faces on x=0 or z=2 (union), minus those on x=0
    let x0 = Selector::ClassifiedOn(model_face(0, 0));
    let z2 = Selector::ClassifiedOn(model_face(2, 1));
    let u = Selector::Union(vec![x0.clone(), z2.clone()]);
    assert_eq!(u.eval(&m, Dim::Face).expect("eval").len(), 8);
    let b = Selector::Intersect(vec![Selector::Boundary, Selector::Complement(Box::new(x0))]);
    assert_eq!(b.eval(&m, Dim::Face).expect("eval").len(), 20);
    assert_eq!(Selector::Interior.eval(&m, Dim::Face).expect("eval").len(), 12);
    assert_eq!(Selector::Shape(sm_mesh_api::CellShape::Hex).eval(&m, Dim::Cell).expect("eval").len(), 8);
    assert!(Selector::Set("nope".into()).eval(&m, Dim::Cell).is_err());
    assert!(Selector::Shape(sm_mesh_api::CellShape::Hex).eval(&m, Dim::Face).is_err());
}

#[test]
fn two_level_hierarchy() {
    let fine = block(10, [2, 2, 2], [1.0; 3]);
    let coarse = block(11, [1, 1, 2], [2.0, 2.0, 1.0]);
    let (fid, cid) = (fine.mesh_id(), coarse.mesh_id());
    // coarse cell = k layer of the fine cell (fine index i + 2j + 4k)
    let f2c: Vec<CellIx> = (0..8u32).map(|c| CellIx(c / 4)).collect();
    let map = LevelMap::new(fid, cid, f2c, 2).expect("map");
    assert_eq!(map.fines_of(CellIx(1)), &[CellIx(4), CellIx(5), CellIx(6), CellIx(7)]);
    assert_eq!(map.coarse_of(CellIx(6)), CellIx(1));

    // volumes restrict exactly
    let vols: Vec<f64> = (0..8).map(|c| fine.cell_volume(CellIx(c))).collect();
    let rest = map.restrict_sum(&vols).expect("restrict");
    assert!((rest[0] - coarse.cell_volume(CellIx(0))).abs() < 1e-12);
    assert!((rest[1] - coarse.cell_volume(CellIx(1))).abs() < 1e-12);
    assert_eq!(map.prolong_copy(&[1.0, 2.0]).expect("prolong"), vec![1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0]);

    // a third, coarsest level and composition
    let top = block(12, [1, 1, 1], [2.0, 2.0, 2.0]);
    let tid = top.mesh_id();
    let up = LevelMap::new(cid, tid, vec![CellIx(0), CellIx(0)], 1).expect("map2");
    let all = map.compose(&up).expect("compose");
    assert_eq!(all.fines_of(CellIx(0)).len(), 8);

    let mut h = Hierarchy::new();
    h.add_level(fine).expect("fine");
    h.add_level(coarse).expect("coarse");
    h.add_level(top).expect("top");
    h.add_map(map).expect("add map");
    h.add_map(up).expect("add map");
    assert_eq!(h.level_ids(), vec![fid, cid, tid]);
    assert!(h.map(fid, cid).is_some());
    assert!(h.map(fid, tid).is_none());
    // wrong sizes and empty coarse cells are rejected
    assert!(h.add_map(LevelMap::new(fid, tid, vec![CellIx(0); 7], 1).expect("m")).is_err());
    assert!(LevelMap::new(fid, cid, vec![CellIx(0); 8], 2).is_err());
}

fn inst(serial: u64, proto: sm_core::EntityId, t: Transform) -> Instance {
    Instance { id: id(EntityKind::Instance, serial), prototype: proto, transform: t }
}

#[test]
fn mirror_unfold_flips_loops_and_keeps_the_rule() {
    let proto = block(1, [2, 1, 1], [1.0; 3]); // x in [0,2]; the mirror plane x=0 is its left side
    let pid = proto.mesh_id();
    let mut view = InstancedMesh::new();
    view.add_prototype(proto.clone()).expect("proto");
    // inserted out of order on purpose: flatten is by ascending instance id
    view.add_instance(inst(2, pid, mirror_x())).expect("i2");
    view.add_instance(inst(1, pid, Transform::IDENTITY)).expect("i1");

    let flat_id = mesh_id(2);
    let run = || {
        view.flatten_into(Mesh::builder(flat_id).expect("b"), FlattenOptions { weld_exact: true }).expect("flatten")
    };
    let (flat, map) = run();
    assert_eq!(validate(&flat, &ValidateOptions::default()), vec![]);
    assert_eq!(flat.len(Dim::Cell), 4);
    assert_eq!(flat.len(Dim::Vertex), 20); // 4 vertices welded on the plane
    assert_eq!(flat.len(Dim::Face), 21);
    assert_eq!(flat.boundary_faces().len(), 18);
    assert_eq!(map.ranges[0].cells, 0..2);
    assert_eq!(map.ranges[1].cells, 2..4);
    assert_eq!(map.ranges[0].instance, id(EntityKind::Instance, 1));

    // the welded face on x=0 is interior, left = lower id (instance 1), normal points -x into the mirror image
    let plane: Vec<FaceIx> = (0..21)
        .map(FaceIx)
        .filter(|&f| flat.face_vertices(f).iter().all(|&v| flat.point(v).x == 0.0))
        .collect();
    assert_eq!(plane.len(), 1);
    let f = plane[0];
    assert_eq!((flat.face_left(f), flat.face_right(f)), (CellIx(0), Some(CellIx(2))));
    assert!((flat.face_area_vector(f).x + 1.0).abs() < 1e-12);

    // flatten is deterministic
    assert_eq!(canonical_hash(&flat), canonical_hash(&run().0));

    // In-place mirror of the prototype: every loop is exactly the reversal of the original loop.
    let mut mirrored = proto.clone();
    mirrored.apply_transform(&mirror_x()).expect("mirror");
    for f in 0..proto.len(Dim::Face) as u32 {
        let f = FaceIx(f);
        assert_eq!(mirrored.face_vertices(f), reversed_loop(proto.face_vertices(f)).as_slice());
        assert_eq!((mirrored.face_left(f), mirrored.face_right(f)), (proto.face_left(f), proto.face_right(f)));
    }
    assert_eq!(validate(&mirrored, &ValidateOptions::default()), vec![]);
    // geometry is the mirror image
    assert_eq!(mirrored.point(VertexIx(1)), Point3::new(-1.0, 0.0, 0.0));
}

#[test]
fn unfold_keeps_polyhedra_valid_and_reports_losses() {
    // a wedge as a general polyhedron plus tags/sets that flatten cannot carry
    let mut b = Mesh::builder(mesh_id(1)).expect("b");
    let v: Vec<VertexIx> = [(1., 0., 0.), (2., 0., 0.), (1., 1., 0.), (1., 0., 1.), (2., 0., 1.), (1., 1., 1.)]
        .iter()
        .map(|&(x, y, z)| b.add_vertex(Point3::new(x, y, z)))
        .collect();
    let faces: Vec<Vec<VertexIx>> = vec![
        vec![v[0], v[2], v[1]],
        vec![v[3], v[4], v[5]],
        vec![v[0], v[1], v[4], v[3]],
        vec![v[1], v[2], v[5], v[4]],
        vec![v[2], v[0], v[3], v[5]],
    ];
    let refs: Vec<&[VertexIx]> = faces.iter().map(Vec::as_slice).collect();
    b.add_polyhedron(&refs).expect("poly");
    b.define_tag(TagDef { name: "material".into(), dim: Dim::Cell, dtype: TagDType::I64, width: 1 }).expect("tag");
    b.set_tag(Dim::Cell, "material", 0, TagRow::I64(&[7])).expect("set");
    let mut proto = b.finish().expect("finish");
    proto.define_set("all", Dim::Cell, vec![0]).expect("set");
    assert_eq!(validate(&proto, &ValidateOptions::default()), vec![]);

    let pid = proto.mesh_id();
    let mut view = InstancedMesh::new();
    view.add_prototype(proto).expect("proto");
    let shift = Transform { m: Transform::IDENTITY.m, t: Vector3::new(5.0, 0.0, 0.0) };
    view.add_instance(inst(1, pid, shift)).expect("i1");
    view.add_instance(inst(2, pid, mirror_x())).expect("i2");
    let (flat, map) = view.flatten_into(Mesh::builder(mesh_id(9)).expect("b"), FlattenOptions::default()).expect("flatten");
    assert_eq!(validate(&flat, &ValidateOptions::default()), vec![]);
    assert_eq!(flat.len(Dim::Cell), 2);
    assert_eq!(flat.tag_row(Dim::Cell, "material", 1), Some(TagRow::I64(&[7])));
    assert_eq!(map.loss, vec!["sets dropped".to_string()]);
    // wrong-kind instance ids and unknown prototypes are rejected
    assert!(view.add_instance(Instance { id: pid, prototype: pid, transform: Transform::IDENTITY }).is_err());
    assert!(view.add_instance(inst(3, mesh_id(99), Transform::IDENTITY)).is_err());
}
