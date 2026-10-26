//! The edge resolvers on a brick topology.
#![allow(missing_docs)]

mod common;

use common::id;
use sm_core::{EntityKind, Point3};
use sm_mesh_api::Classification;
use sm_poly::{EdgeResolver, TopologyResolver, VertexClassResolver};
use sm_topo_api::fixtures::{brick, with_op};
use sm_topo_api::{CellComplex, OpId};

#[test]
fn topology_resolver_finds_the_shared_curve() {
    let mut topo = CellComplex::new();
    let b = with_op(&mut topo, OpId(1), |t| brick(t, Point3::new(0.0, 0.0, 0.0), [1.0, 2.0, 3.0])).unwrap();
    let r = TopologyResolver(&topo);
    let on = Classification::on;
    // faces -x (0) and -y (2) share the z-edge at the origin: edges[8], corners 0 and 4
    let z0 = b.edges[8];
    assert_eq!(r.resolve(b.faces[0], b.faces[2], [on(z0), on(z0)]), Some(z0));
    assert_eq!(r.resolve(b.faces[2], b.faces[0], [on(z0), on(b.vertices[0])]), Some(z0));
    // single interval: both ends are the model vertices of that edge
    assert_eq!(r.resolve(b.faces[0], b.faces[2], [on(b.vertices[0]), on(b.vertices[4])]), Some(z0));
    // an end that is not on the curve rules it out
    assert_eq!(r.resolve(b.faces[0], b.faces[2], [on(b.vertices[0]), on(b.vertices[7])]), None);
    assert_eq!(r.resolve(b.faces[0], b.faces[2], [on(b.edges[9]), on(z0)]), None);
    assert_eq!(r.resolve(b.faces[0], b.faces[2], [on(b.faces[0]), on(z0)]), None);
    // opposite faces share no curve
    assert_eq!(r.resolve(b.faces[0], b.faces[1], [on(z0), on(z0)]), None);
}

#[test]
fn vertex_class_resolver_needs_a_curve_end() {
    let on = Classification::on;
    let (e1, e2) = (id(EntityKind::TopoEdge, 1), id(EntityKind::TopoEdge, 2));
    let (v1, v2) = (id(EntityKind::TopoVertex, 1), id(EntityKind::TopoVertex, 2));
    let (f1, f2) = (id(EntityKind::TopoFace, 1), id(EntityKind::TopoFace, 2));
    let r = VertexClassResolver;
    assert_eq!(r.resolve(f1, f2, [on(e1), on(v1)]), Some(e1));
    assert_eq!(r.resolve(f1, f2, [on(v1), on(e2)]), Some(e2));
    assert_eq!(r.resolve(f1, f2, [on(e1), on(e1)]), Some(e1));
    assert_eq!(r.resolve(f1, f2, [on(e1), on(e2)]), None, "two different curves");
    assert_eq!(r.resolve(f1, f2, [on(v1), on(v2)]), None, "single-interval curve");
}
