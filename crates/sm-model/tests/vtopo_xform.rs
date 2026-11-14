//! W4-VTOPO-FIX2: a transform of a body carries the overlay's stored virtual geometry (chords,
//! cut-face planes, edge-partition parameters) with it. Analytic positions, `validate_geometry`
//! clean, replay / undo / redo hash equality.

mod common;

use std::sync::Arc;

use common::{Factory, cell_complex, id, meta, topo_store};
use serde_json::json;
use sm_boolean::BooleanPlugin;
use sm_core::{EntityId, EntityKind, Point3};
use sm_geom_api::SurfaceDef;
use sm_layer_api::{CoreLayers, ModelLayer};
use sm_model::{CreateBrick, CreateCylinder, ModelPlugin, Move, MoveTo, MoveTarget, PartitionEdge, PartitionFace, Reflect, Rotate, Scale, SplitRealized, UnsplitRealized};
use sm_op_api::{DerivedCache, Engine, Op, QuietControl, Registry, Services, Session};
use sm_topo_api::{Carrier, Dim, TopologyView};

fn session_for(factory: Factory) -> Session {
    let mut reg = Registry::new();
    reg.install(&CoreLayers::new(factory)).unwrap();
    reg.install(&ModelPlugin).unwrap();
    reg.install(&BooleanPlugin).unwrap();
    Session::new(Engine::new(Arc::new(reg), Services::new(), Arc::new(DerivedCache::default())).unwrap())
}

fn go(s: &mut Session, op: &dyn Op) -> sm_op_api::Delta {
    let q = QuietControl::new();
    s.apply(op, meta(), &q.control()).map(|(_, d)| d).unwrap_or_else(|e| panic!("{e}"))
}

fn layer(s: &Session) -> &ModelLayer {
    s.state().layer::<ModelLayer>().unwrap()
}

fn pt(t: &dyn TopologyView, v: EntityId) -> Point3 {
    match t.carrier(v) {
        Carrier::Point(p) => *p,
        c => panic!("vertex carrier {c:?}"),
    }
}

fn dist(a: Point3, b: Point3) -> f64 {
    (a - b).norm()
}

fn vertex_at(s: &Session, p: [f64; 3]) -> EntityId {
    let t = layer(s).topo();
    *t.cells(Dim::Vertex).iter().find(|&&v| dist(pt(t, v), Point3::new(p[0], p[1], p[2])) < 1e-9).expect("vertex")
}

/// Every virtual vertex point and five samples along every virtual edge, in id order.
fn virtual_points(s: &Session) -> Vec<Point3> {
    let t = layer(s).topo();
    let mut out = Vec::new();
    for &v in t.cells(Dim::Vertex) {
        if v.kind() == EntityKind::PartitionVertex {
            out.push(pt(t, v));
        }
    }
    for &e in t.cells(Dim::Edge) {
        if e.kind() == EntityKind::PartitionEdge {
            let sm = t.edge_sampler(e).expect("sampler");
            out.extend((0..=4).map(|k| sm.eval(k as f64 / 4.0)));
        }
    }
    out
}

/// The plane of every virtual face's carrier surface (a cut face), as `(origin, normal)`.
fn virtual_planes(s: &Session) -> Vec<(Point3, sm_core::Vector3)> {
    let t = layer(s).topo();
    let mut out = Vec::new();
    for &f in t.cells(Dim::Face) {
        if f.kind() != EntityKind::PartitionFace {
            continue;
        }
        if let Carrier::Surface(sc) = t.carrier(f)
            && let Some(SurfaceDef::Plane(p)) = t.shapes().surface_def(sc.surface)
        {
            out.push((p.frame.origin, p.frame.z));
        }
    }
    out
}

fn assert_clean(s: &Session, tag: &str) {
    let t = layer(s).topo();
    assert!(sm_topo_api::validate(t).is_empty(), "{tag}: validate {:?}", sm_topo_api::validate(t));
    assert!(sm_topo_api::validate_geometry(t).is_empty(), "{tag}: validate_geometry {:?}", sm_topo_api::validate_geometry(t));
}

/// Apply `op` and check that every virtual point is the image of its old position under `f`,
/// every virtual plane contains the image of its old origin, the view is geometrically valid,
/// and replay / undo / redo agree on the hash.
fn follows(mut s: Session, op: &dyn Op, f: &dyn Fn(Point3) -> Point3, tag: &str) -> Session {
    assert_clean(&s, tag);
    let before = virtual_points(&s);
    let planes = virtual_planes(&s);
    assert!(!before.is_empty(), "{tag}: no virtual geometry to follow");
    let base = s.state().hash();
    go(&mut s, op);
    assert_clean(&s, tag);
    let after = virtual_points(&s);
    assert_eq!(before.len(), after.len(), "{tag}");
    for (b, a) in before.iter().zip(&after) {
        let want = f(*b);
        assert!(dist(want, *a) < 1e-9, "{tag}: {b:?} should be at {want:?}, is at {a:?}");
    }
    let new_planes = virtual_planes(&s);
    assert_eq!(planes.len(), new_planes.len(), "{tag}");
    for ((o, _), (o2, n2)) in planes.iter().zip(&new_planes) {
        // The image of a point of the old plane lies on the new plane.
        let d = (f(*o) - *o2).dot(*n2).abs();
        assert!(d < 1e-9, "{tag}: plane misses the image of its old point by {d:e}");
    }
    let full = s.state().hash();
    let q = QuietControl::new();
    let head = s.position().unwrap();
    let cold = s.state_at_from_root(head, &q.control()).expect("cold replay");
    assert_eq!(cold.hash(), full, "{tag}: cold replay");
    s.undo().unwrap();
    assert_eq!(s.state().hash(), base, "{tag}: undo");
    s.redo().unwrap();
    assert_eq!(s.state().hash(), full, "{tag}: redo");
    s
}

fn body() -> EntityId {
    id(EntityKind::TopoBody, 1)
}

fn split_brick(factory: Factory) -> Session {
    let mut s = session_for(factory);
    go(&mut s, &CreateBrick { x: 4.0, y: Some(2.0), z: Some(2.0), center: Some([2.0, 1.0, 1.0]) });
    go(&mut s, &SplitRealized { piece: id(EntityKind::TopoVolume, 1), normal: [1.0, 0.0, 0.0], offset: 1.0 });
    s
}

fn partitioned_top(factory: Factory) -> Session {
    let mut s = session_for(factory);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(2.0), z: Some(2.0), center: Some([1.0, 1.0, 1.0]) });
    let (a, b) = (vertex_at(&s, [0., 0., 2.]), vertex_at(&s, [2., 2., 2.]));
    let top = {
        let t = layer(&s).topo();
        let ws = [a, b, vertex_at(&s, [2., 0., 2.]), vertex_at(&s, [0., 2., 2.])];
        *t.cells(Dim::Face).iter().find(|&&f| ws.iter().all(|w| t.boundary(f).iter().any(|c| t.boundary(c.cell).iter().any(|e| e.cell == *w)))).expect("top face")
    };
    go(&mut s, &PartitionFace { face: top, start: a, end: b });
    s
}

fn rot_z90(p: Point3) -> Point3 {
    Point3::new(-p.y, p.x, p.z)
}

#[test]
fn move_of_a_realized_split_moves_the_cut_chords_and_plane_repro_a() {
    for factory in [cell_complex as Factory, topo_store] {
        let s = split_brick(factory);
        let s = follows(s, &Move { entities: vec![body()], delta: [10.0, 0.0, 0.0] }, &|p| Point3::new(p.x + 10.0, p.y, p.z), "move");
        // Analytic: every cut vertex and chord sample is on x = 11, the cut plane is x = 11.
        assert!(virtual_points(&s).iter().all(|p| p.x > 10.0 - 1e-9), "{:?}", virtual_points(&s));
        let cut_pts: Vec<Point3> = virtual_points(&s).into_iter().filter(|p| (p.x - 11.0).abs() < 1e-9).collect();
        assert!(cut_pts.len() >= 8, "{}", cut_pts.len());
        let cut_planes: Vec<_> = virtual_planes(&s).into_iter().filter(|(o, n)| n.x.abs() > 0.5 && (o.x - 11.0).abs() < 1e-9).collect();
        assert_eq!(cut_planes.len(), 1, "{:?}", virtual_planes(&s));
        // The cut still unsplits cleanly on the moved model.
        let pieces = layer(&s).overlay().unwrap().ops().iter().find_map(|o| if let sm_topo_api::VirtualOp::PartitionVolume { children, .. } = o { Some(children[0]) } else { None }).unwrap();
        let mut s = s;
        go(&mut s, &UnsplitRealized { piece: pieces });
        assert_clean(&s, "unsplit");
    }
}

#[test]
fn rotate_of_a_realized_split_rotates_the_cut_repro_b() {
    let s = split_brick(cell_complex);
    let s = follows(s, &Rotate { entities: vec![body()], axis: [0.0, 0.0, 1.0], angle_deg: 90.0, origin: None }, &rot_z90, "rotate");
    // The cut x = 1 becomes y = 1.
    let cut_pts: Vec<Point3> = virtual_points(&s).into_iter().filter(|p| (p.y - 1.0).abs() < 1e-9).collect();
    assert!(cut_pts.len() >= 8, "{}", cut_pts.len());
    // Exactly one virtual plane is the cut y = 1 (the others are halves of host faces).
    let cut_planes: Vec<_> = virtual_planes(&s).into_iter().filter(|(o, n)| n.y.abs() > 0.5 && (o.y - 1.0).abs() < 1e-9 && n.x.abs() < 1e-9 && n.z.abs() < 1e-9).collect();
    assert_eq!(cut_planes.len(), 1, "{:?}", virtual_planes(&s));
}

#[test]
fn scale_and_reflect_of_a_realized_split_follow() {
    let s = split_brick(cell_complex);
    follows(s, &Scale { entities: vec![body()], factor: 2.0, origin: None }, &|p| Point3::new(2.0 * p.x, 2.0 * p.y, 2.0 * p.z), "scale");
    let s = split_brick(topo_store);
    follows(s, &Scale { entities: vec![body()], factor: 0.5, origin: Some([1.0, 1.0, 1.0]) }, &|p| Point3::new(1.0 + 0.5 * (p.x - 1.0), 1.0 + 0.5 * (p.y - 1.0), 1.0 + 0.5 * (p.z - 1.0)), "scale about a point");
    for factory in [cell_complex as Factory, topo_store] {
        let s = split_brick(factory);
        follows(s, &Reflect { entities: vec![body()], normal: [1.0, 0.0, 0.0], origin: Some([0.0, 0.0, 0.0]) }, &|p| Point3::new(-p.x, p.y, p.z), "reflect x");
        let s = split_brick(factory);
        follows(s, &Reflect { entities: vec![body()], normal: [0.0, 1.0, 1.0], origin: Some([0.0, 0.0, 0.0]) }, &|p| Point3::new(p.x, -p.z, -p.y), "reflect y+z");
    }
}

#[test]
fn move_to_and_rotate_about_follow_too() {
    let s = split_brick(cell_complex);
    let c = Point3::new(2.0, 1.0, 1.0);
    follows(s, &MoveTo { entities: vec![body()], to: MoveTarget::Coords([0.0, 0.0, 0.0]), anchor: None }, &|p| Point3::new(p.x - c.x, p.y - c.y, p.z - c.z), "move_to");
}

#[test]
fn a_plain_face_partition_follows_a_move_a_rotation_a_scale_and_a_reflection_repro_c() {
    let s = partitioned_top(cell_complex);
    // The chord runs (0,0,2) -> (2,2,2).
    let s = follows(s, &Move { entities: vec![body()], delta: [10.0, 0.0, 0.0] }, &|p| Point3::new(p.x + 10.0, p.y, p.z), "move");
    let t = layer(&s).topo();
    let chord = t.cells(Dim::Edge).iter().copied().find(|e| e.kind() == EntityKind::PartitionEdge).unwrap();
    let sm = t.edge_sampler(chord).unwrap();
    assert!(dist(sm.eval(0.0), Point3::new(10., 0., 2.)) < 1e-9 && dist(sm.eval(1.0), Point3::new(12., 2., 2.)) < 1e-9);
    assert!((sm.length() - 8f64.sqrt()).abs() < 1e-12);
    follows(partitioned_top(topo_store), &Rotate { entities: vec![body()], axis: [1.0, 0.0, 0.0], angle_deg: 90.0, origin: Some([0.0, 1.0, 1.0]) }, &|p| Point3::new(p.x, 1.0 - (p.z - 1.0), 1.0 + (p.y - 1.0)), "rotate x");
    let s = follows(partitioned_top(cell_complex), &Scale { entities: vec![body()], factor: 3.0, origin: None }, &|p| Point3::new(3.0 * p.x, 3.0 * p.y, 3.0 * p.z), "scale");
    let t = layer(&s).topo();
    let chord = t.cells(Dim::Edge).iter().copied().find(|e| e.kind() == EntityKind::PartitionEdge).unwrap();
    assert!((t.edge_sampler(chord).unwrap().length() - 3.0 * 8f64.sqrt()).abs() < 1e-9);
    follows(partitioned_top(topo_store), &Reflect { entities: vec![body()], normal: [0.0, 0.0, 1.0], origin: Some([0.0, 0.0, 0.0]) }, &|p| Point3::new(p.x, p.y, -p.z), "reflect z");
}

#[test]
fn an_edge_partition_follows_scale_on_a_line_and_rotation_on_a_circle() {
    // Line edge: the parameter is arc length, so it scales.
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(3.0), z: Some(4.0), center: None });
    let t = layer(&s).topo();
    let edge = *t.cells(Dim::Edge).iter().find(|&&e| e.kind() == EntityKind::TopoEdge && t.edge_sampler(e).is_some_and(|x| x.length() > 3.5)).unwrap();
    let Carrier::Curve(cc) = t.carrier(edge).clone() else { panic!() };
    go(&mut s, &PartitionEdge { edge, param: 0.5 * (cc.range.lo + cc.range.hi) });
    follows(s, &Scale { entities: vec![body()], factor: 2.5, origin: Some([1.0, 0.0, 0.0]) }, &|p| Point3::new(1.0 + 2.5 * (p.x - 1.0), 2.5 * p.y, 2.5 * p.z), "line scale");

    // Circle edge: the parameter is an angle, it stays.
    let mut s = session_for(topo_store);
    go(&mut s, &CreateCylinder { height: 4.0, radius: 1.5, center: None });
    let t = layer(&s).topo();
    let circle = *t.cells(Dim::Edge).iter().find(|&&e| e.kind() == EntityKind::TopoEdge && t.edge_sampler(e).is_some_and(|x| x.length() > 4.0)).unwrap();
    let Carrier::Curve(cc) = t.carrier(circle).clone() else { panic!() };
    go(&mut s, &PartitionEdge { edge: circle, param: 0.25 * (cc.range.lo + cc.range.hi) });
    let s = follows(s, &Rotate { entities: vec![body()], axis: [0.0, 1.0, 0.0], angle_deg: 90.0, origin: None }, &|p| Point3::new(p.z, p.y, -p.x), "circle rotate");
    follows(s, &Scale { entities: vec![body()], factor: 2.0, origin: None }, &|p| Point3::new(2.0 * p.x, 2.0 * p.y, 2.0 * p.z), "circle scale");
}

#[test]
fn a_transform_of_another_body_leaves_the_overlay_alone() {
    let mut s = session_for(cell_complex);
    go(&mut s, &CreateBrick { x: 2.0, y: Some(2.0), z: Some(2.0), center: Some([1.0, 1.0, 1.0]) });
    go(&mut s, &CreateBrick { x: 2.0, y: Some(2.0), z: Some(2.0), center: Some([11.0, 1.0, 1.0]) });
    let edge = layer(&s).topo().cells(Dim::Edge).iter().copied().find(|&e| e.kind() == EntityKind::TopoEdge && layer(&s).topo().boundary(e).iter().all(|i| pt(layer(&s).topo(), i.cell).x > 5.0)).unwrap();
    go(&mut s, &PartitionEdge { edge, param: 0.5 });
    let before = layer(&s).overlay().unwrap().data();
    let pts = virtual_points(&s);
    go(&mut s, &Move { entities: vec![body()], delta: [0.0, 5.0, 0.0] });
    assert_eq!(layer(&s).overlay().unwrap().data().ops, before.ops);
    assert_eq!(virtual_points(&s), pts);
    assert_clean(&s, "other body");
    // Moving the partitioned one carries it.
    go(&mut s, &Move { entities: vec![id(EntityKind::TopoBody, 2)], delta: [0.0, 5.0, 0.0] });
    let moved = virtual_points(&s);
    assert!(dist(moved[0], Point3::new(pts[0].x, pts[0].y + 5.0, pts[0].z)) < 1e-9);
    assert_clean(&s, "partitioned body");
    let _ = json!(null);
}
