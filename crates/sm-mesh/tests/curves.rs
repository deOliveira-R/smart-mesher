//! Face curves and high-order cells: `finish` (with the default fast validation) must reject
//! exactly what the reference builder rejects, including nodes the builder cannot range-check.

mod common;
use common::replay::assert_same;
use proptest::prelude::*;
use sm_core::{EntityId, EntityKind, Point3};
use sm_mesh::{MeshBuilder, Validation};
use sm_mesh_api::reference::ReferenceBuilder;
use sm_mesh_api::*;
use sm_testkit::strategies::config;

fn cls(k: EntityKind, n: u64) -> Classification {
    Classification::on(EntityId::new(k, n).unwrap())
}

/// One fully classified hex (vertices, edges, faces, cell). Edges are classified so the edge
/// table exists, which curves need.
fn classified_hex<B: MeshBuild>(b: &mut B) -> Vec<VertexIx> {
    let pts = [
        [0., 0., 0.],
        [1., 0., 0.],
        [1., 1., 0.],
        [0., 1., 0.],
        [0., 0., 1.],
        [1., 0., 1.],
        [1., 1., 1.],
        [0., 1., 1.],
    ];
    let vs: Vec<VertexIx> = pts
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let v = b.add_vertex(Point3::new(p[0], p[1], p[2])).unwrap();
            b.set_class(MeshEntityRef::Vertex(v), cls(EntityKind::TopoVertex, 1 + i as u64)).unwrap();
            v
        })
        .collect();
    let added = b.add_cell_from_vertices(CellShape::Hex, &vs).unwrap();
    b.set_class(MeshEntityRef::Cell(added.cell), cls(EntityKind::TopoVolume, 1)).unwrap();
    for (i, sf) in added.faces.iter().enumerate() {
        b.set_class(MeshEntityRef::Face(sf.face()), cls(EntityKind::TopoFace, 1 + i as u64)).unwrap();
    }
    for (i, [a, c]) in CellShape::Hex.local_edges().iter().enumerate() {
        b.set_class(MeshEntityRef::Edge(vs[*a as usize], vs[*c as usize]), cls(EntityKind::TopoEdge, 1 + i as u64)).unwrap();
    }
    vs
}

fn face_curve_tail<B: MeshBuild>(b: &mut B, tail: &[NodeRef]) -> Result<()> {
    let mut nodes: Vec<NodeRef> = b.face_loop(FaceIx(0)).iter().map(|v| NodeRef::Vertex(*v)).collect();
    nodes.extend_from_slice(tail);
    b.set_face_curve(FaceIx(0), 2, Basis::LagrangeEquispaced, &nodes)
}

fn run(validation: Option<Validation>, tail: &[NodeRef], extras: usize) -> Result<Finished<Box<dyn MeshRead>>> {
    fn go<B: MeshBuild>(mut b: B, tail: &[NodeRef], extras: usize) -> Result<Finished<Box<dyn MeshRead>>>
    where
        B::Output: 'static,
    {
        classified_hex(&mut b);
        for i in 0..extras {
            b.add_extra_node(Point3::new(0.5, 0.5, -(i as f64)))?;
        }
        face_curve_tail(&mut b, tail)?;
        let f = b.finish(&mut SerialAlloc::new())?;
        Ok(Finished::new(Box::new(f.mesh) as Box<dyn MeshRead>, f.face_remap))
    }
    match validation {
        None => go(ReferenceBuilder::new(), tail, extras),
        Some(v) => go(MeshBuilder::new().with_validation(v), tail, extras),
    }
}

fn same_outcome(want: &Result<Finished<Box<dyn MeshRead>>>, got: &Result<Finished<Box<dyn MeshRead>>>) -> bool {
    match (want, got) {
        (Ok(w), Ok(g)) => {
            assert_same(w.mesh.as_ref(), g.mesh.as_ref(), "curve");
            w.face_remap == g.face_remap
        }
        (Err(w), Err(g)) => w == g,
        _ => false,
    }
}

#[test]
fn face_curve_with_a_vertex_node_out_of_range_is_rejected_like_the_reference() {
    let bad = vec![NodeRef::Vertex(VertexIx(999)); 5];
    let want = run(None, &bad, 0);
    assert!(matches!(&want, Err(MeshError::Invalid(v)) if v.iter().any(|x| x.code == ViolationCode::FaceCurve)));
    for v in [Validation::Fast, Validation::Full] {
        let got = run(Some(v), &bad, 0);
        assert!(same_outcome(&want, &got), "{v:?}: {:?} vs {:?}", want.as_ref().map(|_| ()), got.as_ref().map(|_| ()));
    }
}

#[test]
fn valid_face_curve_builds_like_the_reference() {
    let tail: Vec<NodeRef> = (0..5).map(NodeRef::Extra).collect();
    let want = run(None, &tail, 5);
    for v in [Validation::Fast, Validation::Full] {
        let got = run(Some(v), &tail, 5);
        assert!(same_outcome(&want, &got), "{v:?}");
    }
}

proptest! {
    #![proptest_config(config(200))]

    /// Random curve tails (vertex and extra nodes, in and out of range) on a classified hex.
    #[test]
    fn random_face_curve_tails_match_the_reference(
        tail in prop::collection::vec((any::<bool>(), 0u32..14), 5..=5),
        extras in 0usize..6,
    ) {
        let tail: Vec<NodeRef> = tail
            .into_iter()
            .map(|(e, x)| if e { NodeRef::Extra(x % 7) } else { NodeRef::Vertex(VertexIx(x)) })
            .collect();
        let want = run(None, &tail, extras);
        for v in [Validation::Fast, Validation::Full] {
            let got = run(Some(v), &tail, extras);
            prop_assert!(same_outcome(&want, &got), "{:?}: {:?} vs {:?}", v, want.as_ref().map(|_| ()), got.as_ref().map(|_| ()));
        }
    }
}
