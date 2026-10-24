//! `bridge_loops` and `FaceCut::Chain` (W0f): a plane webcut of a holed plate by two sequential
//! chords, and a polyline imprint, checked against exact areas.

// Scenario copied from sm-topo-api/tests/bridge_chain.rs and run against the production store (the
// reference's name is rebound so the assertions are untouched).
use sm_topo::TopoStore as CellComplex;
use sm_core::{EntityId, EntityKind, Point3, Vector3};
use sm_geom_api::{CurveDef, Frame, Interval, Line, Plane, SurfaceDef};
use sm_topo_api::*;

fn v(t: &mut CellComplex, x: f64, y: f64) -> (EntityId, Point3) {
    let p = Point3::new(x, y, 0.0);
    (t.make_vertex(p).unwrap(), p)
}

fn line(t: &mut CellComplex, a: (EntityId, Point3), b: (EntityId, Point3)) -> EntityId {
    let d = b.1 - a.1;
    let curve = t.shapes_mut().unwrap().add_curve(CurveDef::Line(Line::new(a.1, d).unwrap())).unwrap();
    t.make_edge(a.0, b.0, Some(CurveCarrier { curve, range: Interval::new(0.0, d.norm()), sense: Sense::Forward })).unwrap()
}

fn fwd(edge: EntityId) -> CoedgeSpec {
    CoedgeSpec { edge, sense: Sense::Forward }
}

fn plane(t: &mut CellComplex) -> SurfaceCarrier {
    let frame = Frame::from_z(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)).unwrap();
    let surface = t.shapes_mut().unwrap().add_surface(SurfaceDef::Plane(Plane { frame })).unwrap();
    SurfaceCarrier { surface, sense: Sense::Forward }
}

/// Closed polygon loop through `pts` (counter-clockwise seen from +z for an outer loop).
fn polygon(t: &mut CellComplex, pts: &[(EntityId, Point3)]) -> Vec<CoedgeSpec> {
    (0..pts.len()).map(|i| fwd(line(t, pts[i], pts[(i + 1) % pts.len()]))).collect()
}

fn area(t: &CellComplex, f: EntityId) -> f64 {
    t.face_moments(f).unwrap().area
}

fn assert_valid(t: &CellComplex) {
    let bad = validate(t);
    assert!(bad.is_empty(), "violations: {bad:#?}");
}

struct Plate {
    face: EntityId,
    outer: Vec<(EntityId, Point3)>,
    hole: Vec<(EntityId, Point3)>,
}

/// A 4 x 4 plate in z = 0 with a 2 x 2 square hole in the middle.
fn holed_plate(t: &mut CellComplex) -> Plate {
    t.begin_op(OpId(1)).unwrap();
    let outer = vec![v(t, 0.0, 0.0), v(t, 4.0, 0.0), v(t, 4.0, 4.0), v(t, 0.0, 4.0)];
    // The hole runs clockwise seen from +z.
    let hole = vec![v(t, 1.0, 1.0), v(t, 1.0, 3.0), v(t, 3.0, 3.0), v(t, 3.0, 1.0)];
    let lo = polygon(t, &outer);
    let li = polygon(t, &hole);
    let s = plane(t);
    let face = t.make_face(&[&lo, &li], Some(s)).unwrap();
    t.end_op().unwrap();
    Plate { face, outer, hole }
}

#[test]
fn ids_start_at_serial_one() {
    let mut t = CellComplex::new();
    let p = holed_plate(&mut t);
    assert_eq!(p.outer[0].0, EntityId::new(EntityKind::TopoVertex, 1).unwrap());
    assert_eq!(p.face, EntityId::new(EntityKind::TopoFace, 1).unwrap());
    let zero = EntityId::new(EntityKind::TopoVertex, 0).unwrap();
    assert_eq!(t.dim_of(zero), None);
    assert_eq!(t.state(zero), None);
}

#[test]
fn holed_plate_webcut_by_two_sequential_chords() {
    let mut t = CellComplex::new();
    let p = holed_plate(&mut t);
    assert!((area(&t, p.face) - 12.0).abs() < 1e-12);
    assert_eq!(t.loops(p.face).count(), 2);
    // A diagonal plane x = y crosses the plate at (0,0)-(1,1) and (3,3)-(4,4).
    t.begin_op(OpId(2)).unwrap();
    let e1 = line(&mut t, p.outer[0], p.hole[0]);
    let e2 = line(&mut t, p.hole[2], p.outer[2]);
    // A one-chord split of a holed face is still impossible: the first chord ends on the hole.
    assert!(t.split_face(p.face, FaceCut::Chord { edge: e1 }, InnerLoopAssign::None).is_err());
    let before = t.canonical_hash();
    // Same loop on both ends, or an edge not touching the face: rejected, nothing changes.
    let stray = line(&mut t, p.outer[0], p.outer[2]);
    assert!(t.bridge_loops(p.face, stray).is_err());
    assert_ne!(t.canonical_hash(), before, "stray edge was created");
    let before = t.canonical_hash();
    assert!(t.bridge_loops(p.face, stray).is_err());
    assert_eq!(t.canonical_hash(), before);
    t.kill(stray).unwrap();
    // First chord: bridge the outer loop and the hole. The face is not split.
    t.bridge_loops(p.face, e1).unwrap();
    t.end_op().unwrap();
    assert_valid(&t);
    assert!(t.is_alive(p.face));
    assert_eq!(t.loops(p.face).count(), 1);
    assert!((area(&t, p.face) - 12.0).abs() < 1e-12, "the bridge adds no area");
    let keys: Vec<_> = t.coedges(p.face).filter(|c| c.key.edge == e1).map(|c| c.key.side).collect();
    assert_eq!(keys.len(), 2, "the bridge is used once from each side");
    assert!(keys.contains(&Side::Left) && keys.contains(&Side::Right));
    assert!(
        t.lineage().for_op(OpId(2)).iter().any(|r| r.entity == p.face && r.kind == LineageKind::Modified(ModKind::InnerLoop)),
        "lineage records Modified on the face"
    );
    // Second chord: both ends now on the single loop, so it splits the face.
    t.begin_op(OpId(3)).unwrap();
    let [a, b] = t.split_face(p.face, FaceCut::Chord { edge: e2 }, InnerLoopAssign::None).unwrap();
    t.end_op().unwrap();
    assert_valid(&t);
    assert!(!t.is_alive(p.face));
    assert!((area(&t, a) - 6.0).abs() < 1e-12, "{}", area(&t, a));
    assert!((area(&t, b) - 6.0).abs() < 1e-12, "{}", area(&t, b));
    assert_eq!(t.loops(a).count(), 1);
    assert_eq!(t.loops(b).count(), 1);
    assert_eq!(t.lineage().children(p.face).as_slice(), &[a, b]);
}

#[test]
fn polyline_imprint_with_a_chain_cut() {
    let mut t = CellComplex::new();
    t.begin_op(OpId(1)).unwrap();
    let c = [v(&mut t, 0.0, 0.0), v(&mut t, 4.0, 0.0), v(&mut t, 4.0, 4.0), v(&mut t, 0.0, 4.0)];
    let l = polygon(&mut t, &c);
    let s = plane(&mut t);
    let face = t.make_face(&[&l], Some(s)).unwrap();
    t.end_op().unwrap();
    t.begin_op(OpId(2)).unwrap();
    let mid = v(&mut t, 2.0, 1.0);
    let e1 = line(&mut t, c[0], mid);
    let e2 = line(&mut t, mid, c[2]);
    // Bad chains are rejected atomically: a gap, a chain closing on itself, an interior vertex
    // on the loop.
    let h = t.canonical_hash();
    assert!(t.split_face(face, FaceCut::Chain { edges: &[fwd(e2), fwd(e1)] }, InnerLoopAssign::None).is_err());
    assert!(t.split_face(face, FaceCut::Chain { edges: &[fwd(e1), CoedgeSpec { edge: e1, sense: Sense::Reversed }] }, InnerLoopAssign::None).is_err());
    assert!(t.split_face(face, FaceCut::Chain { edges: &[CoedgeSpec { edge: l[0].edge, sense: Sense::Forward }] }, InnerLoopAssign::None).is_err());
    assert_eq!(t.canonical_hash(), h);
    let [a, b] = t.split_face(face, FaceCut::Chain { edges: &[fwd(e1), fwd(e2)] }, InnerLoopAssign::None).unwrap();
    t.end_op().unwrap();
    assert_valid(&t);
    // (0,0) (4,0) (4,4) (2,1): area 6; the rest: 10.
    let (aa, ab) = (area(&t, a), area(&t, b));
    assert!((aa - 6.0).abs() < 1e-12 && (ab - 10.0).abs() < 1e-12, "{aa} {ab}");
    // Both children use the chain, once from each side.
    for f in [a, b] {
        assert!(t.face_edges(f).contains(&e1) && t.face_edges(f).contains(&e2));
    }
    // The reversed chain gives the same split.
    let mut t2 = CellComplex::new();
    t2.begin_op(OpId(1)).unwrap();
    let c2 = [v(&mut t2, 0.0, 0.0), v(&mut t2, 4.0, 0.0), v(&mut t2, 4.0, 4.0), v(&mut t2, 0.0, 4.0)];
    let l2 = polygon(&mut t2, &c2);
    let s2 = plane(&mut t2);
    let face2 = t2.make_face(&[&l2], Some(s2)).unwrap();
    t2.end_op().unwrap();
    t2.begin_op(OpId(2)).unwrap();
    let mid2 = v(&mut t2, 2.0, 1.0);
    let f1 = line(&mut t2, c2[0], mid2);
    let f2 = line(&mut t2, mid2, c2[2]);
    let rev = |e| CoedgeSpec { edge: e, sense: Sense::Reversed };
    let [a2, b2] = t2.split_face(face2, FaceCut::Chain { edges: &[rev(f2), rev(f1)] }, InnerLoopAssign::None).unwrap();
    t2.end_op().unwrap();
    assert_valid(&t2);
    assert!((area(&t2, a2) - 6.0).abs() < 1e-12 && (area(&t2, b2) - 10.0).abs() < 1e-12);
}

#[test]
fn chain_ending_on_a_hole_needs_a_bridge_first() {
    let mut t = CellComplex::new();
    let p = holed_plate(&mut t);
    t.begin_op(OpId(2)).unwrap();
    let mid = v(&mut t, 0.5, 0.6);
    let e1 = line(&mut t, p.outer[0], mid);
    let e2 = line(&mut t, mid, p.hole[0]);
    assert!(matches!(
        t.split_face(p.face, FaceCut::Chain { edges: &[fwd(e1), fwd(e2)] }, InnerLoopAssign::None),
        Err(TopoError::Unsupported(_))
    ));
    t.abort_op();
}
