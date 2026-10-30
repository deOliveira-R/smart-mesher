//! A face shared with a volume that is not meshed in this entry keeps its nodes (conformity).

mod common;

use common::*;
use sm_core::{EntityKind, Point3};
use sm_mesh_api::{MeshEdit, MeshRead, PointIx};
use sm_model_api::Model;
use sm_smooth::{Method, NodeSelect, SmoothError, SmoothInput, SmoothRequest, smooth};
use sm_testkit::lattice;
use sm_testkit::topo::finish_boundaries;
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::{CellComplex, OpId, TopologyView};

#[test]
fn shared_face_with_an_unmeshed_volume_does_not_move() {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).expect("brick");
    let w = with_op(&mut t, OpId(2), |t| fixtures::webcut_brick(t, &b, 0.5)).expect("webcut");
    let mut model = Model::new();
    finish_boundaries(&mut model, &t, 7).expect("boundaries");
    let lower = w.volumes[0];
    let mut mesh = stub_mesh(&t, &model, lower, 0.25);
    jitter(&mut mesh, &t, 5, 0.06, false);
    // The cap (z = 1/2) is shared with the unmeshed upper volume.
    let face_at = |z: f64| {
        mesh.classified_entities()
            .iter()
            .copied()
            .find(|e| e.kind() == EntityKind::TopoFace && (centroid_of(&mesh, *e)[2] - z).abs() < 1e-9)
            .expect("face")
    };
    let (cap, bottom) = (face_at(0.5), face_at(0.0));
    assert_eq!(t.coboundary(cap).len(), 2, "the cap has two volumes above");
    for nodes in [NodeSelect::All, NodeSelect::Entities(vec![lower])] {
        let input = SmoothInput { mesh: &mesh, topo: &t, model: &model };
        let r = SmoothRequest { nodes, method: Method::Laplacian { relax: 1.0 }, iterations: 10, fix_boundary: false, ..SmoothRequest::default() };
        let o = smooth(&input, &r).expect("smooth");
        let moved = |e| {
            mesh.classified_on(e).expect("nodes").vertices.iter().any(|v| {
                let p = mesh.point(v.get());
                [p.x, p.y, p.z].iter().zip(&o.points[v.get()]).any(|(a, b)| a.to_bits() != b.to_bits())
            })
        };
        assert!(!moved(cap), "cap nodes must stay (conformity with the unmeshed volume)");
        assert!(moved(bottom), "boundary nodes of the meshed volume alone still move");
    }
}

#[test]
fn non_finite_coordinates_are_rejected() {
    let f = lattice::quad([3, 3], [1.0; 2]).expect("quad");
    for bad in [f64::NAN, f64::INFINITY] {
        let mut mesh = f.mesh.clone();
        mesh.set_point(PointIx::new(5), Point3::new(bad, 0.5, 0.0)).expect("set");
        let input = SmoothInput { mesh: &mesh, topo: &f.topo, model: &f.model };
        assert_eq!(smooth(&input, &SmoothRequest::default()).unwrap_err(), SmoothError::NonFiniteInput(5));
    }
}
