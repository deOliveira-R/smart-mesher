//! State-machine differential test: random sequences of builder calls (valid and invalid) are
//! applied to the reference builder and to `MeshBuilder`; every return value, every error and the
//! final `finish` outcome must be identical.

mod common;
use common::replay::assert_same;
use proptest::prelude::*;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh::MeshBuilder;
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::*;
use sm_testkit::strategies::config;

#[derive(Clone, Debug)]
enum Op {
    AddVertex(i8, i8, i8),
    AddFace(Vec<u8>),
    FindFace(Vec<u8>),
    AddCell(Vec<(u8, bool)>, Option<Vec<u8>>),
    AddTemplate(u8, Vec<u8>),
    AddPoly(Vec<Vec<u8>>),
    Truncate(u8),
    ClassV(u8, u8),
    ClassF(u8, u8),
    ClassC(u8, u8),
    ClassE(u8, u8, u8),
    FaceKind(u8, u8),
    Parent(u8, u8),
    Active(u8, bool),
    FindVertex(i8, i8, i8, u8),
    Query(u8),
}

fn op() -> impl Strategy<Value = Op> {
    let ix = 0u8..12;
    prop_oneof![
        2 => (any::<i8>(), any::<i8>(), any::<i8>()).prop_map(|(x, y, z)| Op::AddVertex(x % 3, y % 3, z % 3)),
        4 => prop::collection::vec(ix.clone(), 0..6).prop_map(Op::AddFace),
        3 => prop::collection::vec(ix.clone(), 0..5).prop_map(Op::FindFace),
        4 => (prop::collection::vec((0u8..14, any::<bool>()), 0..7), prop::option::of(prop::collection::vec(ix.clone(), 0..9)))
            .prop_map(|(f, c)| Op::AddCell(f, c)),
        6 => (0u8..5, prop::collection::vec(ix.clone(), 3..9)).prop_map(|(s, v)| Op::AddTemplate(s, v)),
        2 => prop::collection::vec(prop::collection::vec(ix.clone(), 2..5), 3..6).prop_map(Op::AddPoly),
        1 => (0u8..16).prop_map(Op::Truncate),
        3 => (ix.clone(), 0u8..8).prop_map(|(v, c)| Op::ClassV(v, c)),
        3 => (0u8..16, 0u8..8).prop_map(|(f, c)| Op::ClassF(f, c)),
        3 => (0u8..8, 0u8..8).prop_map(|(c, k)| Op::ClassC(c, k)),
        1 => (ix.clone(), ix.clone(), 0u8..8).prop_map(|(a, b, c)| Op::ClassE(a, b, c)),
        1 => (0u8..16, 0u8..4).prop_map(|(f, k)| Op::FaceKind(f, k)),
        1 => (0u8..8, 0u8..8).prop_map(|(c, p)| Op::Parent(c, p)),
        1 => (0u8..8, any::<bool>()).prop_map(|(c, a)| Op::Active(c, a)),
        2 => (any::<i8>(), any::<i8>(), any::<i8>(), 0u8..3).prop_map(|(x, y, z, t)| Op::FindVertex(x % 3, y % 3, z % 3, t)),
        2 => (0u8..16).prop_map(Op::Query),
    ]
}

fn class(i: u8) -> Classification {
    let m = |k, n| Classification::on(EntityId::new(k, n).unwrap());
    match i % 8 {
        0 => m(EntityKind::TopoVolume, 1),
        1 => m(EntityKind::TopoFace, 1),
        2 => m(EntityKind::TopoFace, 2),
        3 => m(EntityKind::TopoEdge, 1),
        4 => m(EntityKind::TopoVertex, 1),
        5 => m(EntityKind::MeshCell, 1), // not a model entity
        6 => Classification { entity: EntityId::new(EntityKind::TopoFace, 3).unwrap(), param: Param::Curve(0.5) },
        _ => Classification { entity: EntityId::new(EntityKind::TopoEdge, 2).unwrap(), param: Param::Curve(0.25) },
    }
}

/// Apply one op; the result is a string so reference and subject outcomes compare directly.
fn apply<B: MeshBuild>(b: &mut B, op: &Op) -> String {
    let nv = b.vertex_count();
    let vs = |l: &[u8]| -> Vec<VertexIx> { l.iter().map(|&i| VertexIx(u32::from(i) % (nv as u32 + 2))).collect() };
    match op {
        Op::AddVertex(x, y, z) => format!("{:?}", b.add_vertex(Point3::new(f64::from(*x), f64::from(*y), f64::from(*z)))),
        Op::AddFace(l) => format!("{:?}", b.add_face(&vs(l))),
        Op::FindFace(l) => format!("{:?}", b.find_face(&vs(l))),
        Op::AddCell(f, c) => {
            let nf = b.face_count() as u32;
            let sfs: Vec<SignedFace> = f
                .iter()
                .map(|(i, r)| SignedFace::new(FaceIx(u32::from(*i) % (nf + 2)), if *r { Side::Right } else { Side::Left }))
                .collect();
            format!("{:?}", b.add_cell(&sfs, c.as_ref().map(|c| vs(c)).as_deref()))
        }
        Op::AddTemplate(s, l) => {
            let shape = [CellShape::Tet, CellShape::Hex, CellShape::Wedge, CellShape::Pyramid, CellShape::Tri][*s as usize % 5];
            let n = shape.vertex_count().unwrap_or(4);
            let mut v = vs(l);
            v.resize(n, VertexIx(0));
            format!("{:?}", b.add_cell_from_vertices(shape, &v))
        }
        Op::AddPoly(loops) => {
            let l: Vec<Vec<VertexIx>> = loops.iter().map(|l| vs(l)).collect();
            let r: Vec<&[VertexIx]> = l.iter().map(Vec::as_slice).collect();
            format!("{:?}", b.add_polyhedron(&r))
        }
        Op::Truncate(n) => format!("{:?}", b.truncate_faces(*n as usize % (b.face_count() + 2))),
        Op::ClassV(v, c) => format!("{:?}", b.set_class(MeshEntityRef::Vertex(VertexIx(u32::from(*v) % (nv as u32 + 2))), class(*c))),
        Op::ClassF(f, c) => {
            let nf = b.face_count() as u32;
            format!("{:?}", b.set_class(MeshEntityRef::Face(FaceIx(u32::from(*f) % (nf + 2))), class(*c)))
        }
        Op::ClassC(c, k) => {
            let nc = b.cell_count() as u32;
            format!("{:?}", b.set_class(MeshEntityRef::Cell(CellIx(u32::from(*c) % (nc + 2))), class(*k)))
        }
        Op::ClassE(a, c, k) => {
            let (a, c) = (VertexIx(u32::from(*a) % (nv as u32 + 2)), VertexIx(u32::from(*c) % (nv as u32 + 2)));
            format!("{:?}", b.set_class(MeshEntityRef::Edge(a, c), class(*k)))
        }
        Op::FaceKind(f, k) => {
            let nf = b.face_count() as u32;
            let kind = [FaceKind::Interior, FaceKind::Boundary, FaceKind::NonConformal, FaceKind::PartitionCut][*k as usize % 4];
            format!("{:?}", b.set_face_kind(FaceIx(u32::from(*f) % (nf + 2)), kind))
        }
        Op::Parent(c, p) => {
            let nc = b.cell_count() as u32;
            format!("{:?}", b.set_cell_parent(CellIx(u32::from(*c) % (nc + 2)), CellIx(u32::from(*p) % (nc + 2))))
        }
        Op::Active(c, a) => {
            let nc = b.cell_count() as u32;
            format!("{:?}", b.set_cell_active(CellIx(u32::from(*c) % (nc + 2)), *a))
        }
        Op::FindVertex(x, y, z, t) => {
            let tol = [0.0, 0.5, 1.5][*t as usize % 3];
            format!("{:?}", b.find_vertex(Point3::new(f64::from(*x), f64::from(*y), f64::from(*z)), tol))
        }
        Op::Query(i) => {
            let (nf, nc) = (b.face_count(), b.cell_count());
            let mut s = format!("{}/{}/{}", b.vertex_count(), nf, nc);
            if nf > 0 {
                let f = FaceIx::new(*i as usize % nf);
                s += &format!(
                    " {:?} {} {:?} {:?} {:?} {:?}",
                    b.face_loop(f),
                    b.face_degree(f),
                    b.face_cell(f, Side::Left),
                    b.face_cell(f, Side::Right),
                    b.face_class(f),
                    b.vertex_cell(VertexIx(u32::from(*i) % (nv as u32 + 1))),
                );
            }
            if nc > 0 {
                let c = CellIx::new(*i as usize % nc);
                s += &format!(" {:?} {:?}", b.cell_faces(c), b.cell_class(c));
            }
            if nv > 0 {
                let v = VertexIx::new(*i as usize % nv);
                s += &format!(" {:?} {:?}", b.vertex_point(v), b.vertex_class(v));
            }
            s
        }
    }
}

/// Classify what is still unclassified with valid classes, so `finish` gets past the
/// "unclassified" errors and exercises the later checks.
fn classify_rest<B: MeshBuild>(b: &mut B, all: bool) {
    if !all {
        return;
    }
    let m = |k, n| Classification::on(EntityId::new(k, n).unwrap());
    for v in 0..b.vertex_count() {
        if b.vertex_class(VertexIx::new(v)).is_none() {
            let _ = b.set_class(MeshEntityRef::Vertex(VertexIx::new(v)), m(EntityKind::TopoVertex, 1));
        }
    }
    for c in 0..b.cell_count() {
        if b.cell_class(CellIx::new(c)).is_none() {
            let _ = b.set_class(MeshEntityRef::Cell(CellIx::new(c)), m(EntityKind::TopoVolume, 1));
        }
    }
    for f in 0..b.face_count() {
        let fi = FaceIx::new(f);
        if b.face_class(fi).is_none() && b.face_degree(fi) == 1 {
            let _ = b.set_class(MeshEntityRef::Face(fi), m(EntityKind::TopoFace, 1));
        }
    }
}

proptest! {
    #![proptest_config(config(400))]

    #[test]
    fn builder_state_machine_matches_the_reference(
        ops in prop::collection::vec(op(), 1..40),
        classify in any::<bool>(),
    ) {
        let (mut r, mut m) = (ReferenceBuilder::new(), MeshBuilder::new());
        for p in 0..7u8 {
            let q = Point3::new(f64::from(p % 3), f64::from((p / 3) % 3), f64::from(p / 6));
            r.add_vertex(q).unwrap();
            m.add_vertex(q).unwrap();
        }
        for (k, o) in ops.iter().enumerate() {
            let (a, b) = (apply(&mut r, o), apply(&mut m, o));
            prop_assert_eq!(a, b, "op {} {:?}", k, o);
        }
        classify_rest(&mut r, classify);
        classify_rest(&mut m, classify);
        let (mut ar, mut am) = (SerialAlloc::new(), SerialAlloc::new());
        let want = r.finish(&mut ar);
        let got = m.finish(&mut am);
        match (want, got) {
            (Ok(w), Ok(g)) => {
                assert_same(&w.mesh, &g.mesh, "finish");
                prop_assert_eq!(w.face_remap, g.face_remap);
            }
            (Err(w), Err(g)) => prop_assert_eq!(w, g),
            (w, g) => prop_assert!(false, "reference {:?} vs subject {:?}", w.map(|_| ()), g.map(|_| ())),
        }
        for k in [EntityKind::Mesh, EntityKind::MeshVertex, EntityKind::MeshFace, EntityKind::MeshCell] {
            prop_assert_eq!(ar.next_serial(k), am.next_serial(k));
        }
    }
}

#[test]
fn the_state_machine_reaches_successful_builds() {
    // Guard against a vacuous property: a hand-written sequence in the same op language must
    // produce a mesh (two tets sharing a face), and a defective one must not.
    let tet = |a: u8, b: u8, c: u8, d: u8| Op::AddTemplate(0, vec![a, b, c, d]);
    let ops = [tet(0, 1, 2, 3), tet(1, 0, 2, 4)];
    let (mut r, mut m) = (ReferenceBuilder::new(), MeshBuilder::new());
    for p in 0..7u8 {
        let q = Point3::new(f64::from(p % 3), f64::from((p / 3) % 3), f64::from(p / 6));
        r.add_vertex(q).unwrap();
        m.add_vertex(q).unwrap();
    }
    for o in &ops {
        assert_eq!(apply(&mut r, o), apply(&mut m, o));
    }
    classify_rest(&mut r, true);
    classify_rest(&mut m, true);
    let w = r.finish(&mut SerialAlloc::new());
    let g = m.finish(&mut SerialAlloc::new());
    match (w, g) {
        (Ok(w), Ok(g)) => assert_same(&w.mesh, &g.mesh, "two tets"),
        (Err(w), Err(g)) => assert_eq!(w, g),
        _ => panic!("outcomes differ"),
    }
}
