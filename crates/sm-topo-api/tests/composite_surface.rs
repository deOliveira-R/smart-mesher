//! ACR-V2a: `TopologyView::composite_surface` on hosts and overlay composites.

use sm_core::{EntityId, Point3, Vector3};
use sm_geom_api::{CurveDef, Interval, Line, SurfaceDef};
use sm_topo_api::fixtures::{self, with_op};
use sm_topo_api::*;
use std::sync::Arc;

fn diagonal() -> CurveDef {
    CurveDef::Line(Line::new(Point3::new(0.0, 0.0, 1.0), Vector3::new(1.0, 1.0, 0.0)).unwrap())
}

/// A unit brick whose top face is split by a chord into two coplanar halves.
fn split_brick() -> (CellComplex, fixtures::BrickIds, [EntityId; 2]) {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let halves = with_op(&mut t, OpId(2), |t| {
        let curve = t.shapes_mut().unwrap().add_curve(diagonal()).unwrap();
        let chord = t.make_edge(
            b.vertices[4],
            b.vertices[7],
            Some(CurveCarrier { curve, range: Interval::new(0.0, 2f64.sqrt()), sense: Sense::Forward }),
        )?;
        Ok(t.split_face(b.faces[5], FaceCut::Chord { edge: chord }, InnerLoopAssign::None)?)
    })
    .unwrap();
    (t, b, [halves[0], halves[1]])
}

#[test]
fn host_default_is_the_faces_own_surface() {
    let (t, b, _) = split_brick();
    let Carrier::Surface(sc) = *t.carrier(b.faces[0]) else { panic!("brick face has a surface") };
    assert_eq!(t.composite_surface(b.faces[0]), Some(sc));
    assert_eq!(t.composite_surface(b.vertices[0]), None);
    assert_eq!(t.composite_surface(b.edges[0]), None);
}

#[test]
fn planar_composite_returns_the_members_surface() {
    let (t, _, halves) = split_brick();
    let Carrier::Surface(sc) = *t.carrier(halves[0]) else { panic!() };
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let mut ov = Overlay::new(host);
    let vf = ov.composite_faces(&halves).unwrap();
    assert!(matches!(ov.carrier(vf), Carrier::Composite(_)));
    assert_eq!(ov.composite_surface(vf), Some(sc));
}

#[test]
fn composite_of_faces_on_different_surfaces_is_none() {
    let (t, b, _) = split_brick();
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let mut ov = Overlay::new(host);
    // Two adjacent side faces: different planes.
    let vf = ov.composite_faces(&[b.faces[0], b.faces[2]]).unwrap();
    assert_eq!(ov.composite_surface(vf), None);
}

#[test]
fn partition_halves_and_their_composite_keep_the_surface() {
    let mut t = CellComplex::new();
    let b = with_op(&mut t, OpId(1), |t| fixtures::brick(t, Point3::new(0.0, 0.0, 0.0), [1.0; 3])).unwrap();
    let Carrier::Surface(sc) = *t.carrier(b.faces[5]) else { panic!() };
    let host: Arc<dyn TopologyView> = Arc::new(t);
    let mut ov = Overlay::new(host);
    let cut = VirtualCut { from: b.vertices[4], to: b.vertices[7], curve: diagonal(), range: Interval::new(0.0, 2f64.sqrt()) };
    let h = ov.partition_face(b.faces[5], cut).unwrap();
    assert_eq!(ov.composite_surface(h[0]), Some(sc));
    assert_eq!(ov.composite_surface(h[1]), Some(sc));
}

/// Two quads sharing an edge, both carried by `surface` (topology only: the composite does
/// not look at the geometry, only at the carrier handles).
fn two_quads_on(def: SurfaceDef) -> (Arc<dyn TopologyView>, [EntityId; 2], SurfaceCarrier) {
    let mut t = CellComplex::new();
    let (faces, sc) = with_op(&mut t, OpId(1), |t| {
        let p = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0), (0.0, 1.0)];
        let v: Vec<EntityId> = p.iter().map(|q| t.make_vertex(Point3::new(q.0, q.1, 0.0)).unwrap()).collect();
        let edge = |t: &mut CellComplex, a: usize, b: usize| {
            let (pa, pb) = (Point3::new(p[a].0, p[a].1, 0.0), Point3::new(p[b].0, p[b].1, 0.0));
            let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(pa, pb - pa).unwrap())).unwrap();
            t.make_edge(v[a], v[b], Some(CurveCarrier { curve, range: Interval::new(0.0, 1.0), sense: Sense::Forward }))
        };
        let (e01, e12, e23, e34, e45, e50, e14) = (edge(t, 0, 1)?, edge(t, 1, 2)?, edge(t, 2, 3)?, edge(t, 3, 4)?, edge(t, 4, 5)?, edge(t, 5, 0)?, edge(t, 1, 4)?);
        let surface = t.shapes_mut().unwrap().add_surface(def).unwrap();
        let sc = SurfaceCarrier { surface, sense: Sense::Forward };
        let f = |e: EntityId, fwd: bool| CoedgeSpec { edge: e, sense: if fwd { Sense::Forward } else { Sense::Reversed } };
        let a = t.make_face(&[&[f(e01, true), f(e14, true), f(e45, true), f(e50, true)]], Some(sc))?;
        let b = t.make_face(&[&[f(e12, true), f(e23, true), f(e34, true), f(e14, false)]], Some(sc))?;
        Ok(([a, b], sc))
    })
    .unwrap();
    (Arc::new(t), faces, sc)
}

#[test]
fn periodic_composite_is_none_but_the_host_face_keeps_its_surface() {
    use sm_geom_api::{Cylinder, Frame, Plane};
    let (host, faces, sc) = two_quads_on(SurfaceDef::Cylinder(Cylinder::new(Frame::at(Point3::new(0.0, 0.0, 0.0)), 1.0).unwrap()));
    // The host face answers with its own surface (the pcurve path of a periodic face).
    assert_eq!(host.composite_surface(faces[0]), Some(sc));
    let mut ov = Overlay::new(host);
    let vf = ov.composite_faces(&faces).unwrap();
    assert_eq!(ov.composite_surface(vf), None, "cylinder side is periodic in u");
    // Control: the same topology on a plane resolves.
    let (host, faces, sc) = two_quads_on(SurfaceDef::Plane(Plane { frame: Frame::at(Point3::new(0.0, 0.0, 0.0)) }));
    let mut ov = Overlay::new(host);
    let vf = ov.composite_faces(&faces).unwrap();
    assert_eq!(ov.composite_surface(vf), Some(sc));
}
